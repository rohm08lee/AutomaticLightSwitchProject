/*
 * Adaptive Light Switch
 * ----------------------
 * Automatically actuates an existing exterior light switch based on ambient
 * light level, without any rewiring. An ESP32-C3 reads lux from a BH1750
 * sensor, decides on/off state using a dual-exponential-moving-average
 * filter with adaptive thresholds, and physically flips the switch with two
 * SG-90 servos mounted on the switch plate.
 *
 * A small web server on the ESP32 exposes a live dashboard, a JSON status
 * endpoint, and a CSV log of every reading and switch event, all served
 * from the device's own flash storage (SPIFFS) — no external database.
 *
 * Setup:
 *   1. Copy secrets.h.example to secrets.h and fill in your Wi-Fi
 *      credentials. secrets.h is gitignored and will not be committed.
 *   2. Flash to an ESP32-C3 with the BH1750 wired to the I2C pins defined
 *      below and two SG-90 servos on SERVO_TOP_PIN / SERVO_BOTTOM_PIN.
 */

#include <Wire.h>
#include <BH1750.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <WebServer.h>
#include <SPIFFS.h>
#include <time.h>

#include "secrets.h"  // defines WIFI_SSID / WIFI_PASSWORD, see secrets.h.example

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

// NTP time sync, used only to timestamp log entries.
static const char* NTP_SERVER = "pool.ntp.org";
static const char* TIME_ZONE  = "CET-1CEST,M3.5.0,M10.5.0/3";  // CET/CEST

// Absolute lux floors. Adaptive thresholds (see loop()) are clamped to these
// so the switch never becomes hypersensitive on an unusually dark or bright
// day.
static const float MIN_ON_LUX  = 2.0;   // never trigger ON above this
static const float MIN_OFF_LUX = 10.0;  // never trigger OFF below this

// Adaptive thresholds are a percentage of the slow-tracked ambient baseline
// (see the dual-EMA filter in loop()): the light turns on once smoothed lux
// drops below DROP_PERCENTAGE of the baseline, and off once it rises above
// RISE_PERCENTAGE of the baseline. Asymmetric on purpose, so the switch
// doesn't chatter near a single fixed point at dusk/dawn.
static const float DROP_PERCENTAGE = 0.10;
static const float RISE_PERCENTAGE = 0.25;

// Dual exponential moving average: ALPHA_FAST smooths short-term sensor
// noise; ALPHA_SLOW tracks the true ambient baseline as daylight drifts
// over hours. A much smaller ALPHA_SLOW means the baseline barely reacts
// to a single noisy reading.
static const float ALPHA_FAST = 0.15;
static const float ALPHA_SLOW = 0.001;

// Placeholder for calibrating out light bleed from the fixture itself
// hitting the sensor when the light is on. Currently uncalibrated (0.0);
// raise this if the sensor reads artificially bright while the light is on.
static const float INTERNAL_BLEED_LUX = 0.0;

static const int I2C_SDA_PIN = 8;
static const int I2C_SCL_PIN = 9;

static const int SERVO_TOP_PIN    = 5;
static const int SERVO_BOTTOM_PIN = 6;

// Top servo presses and returns to a resting angle (like a finger tapping
// the switch). Bottom servo instead holds a fixed position for on vs off.
static const int TOP_SERVO_ON_ANGLE   = 154;
static const int TOP_SERVO_OFF_ANGLE  = 180;
static const int TOP_SERVO_REST_ANGLE = 167;

static const float BOTTOM_SERVO_ON_ANGLE  = 0.0;
static const float BOTTOM_SERVO_OFF_ANGLE = 15.0;

static const unsigned long SERVO_PRESS_MS   = 350;
static const unsigned long POLL_INTERVAL_MS = 120000;  // one reading every 2 minutes

static const size_t LOG_ROTATION_BYTES = 1000000;  // rotate logs.csv past ~1 MB
static const char*  LOG_HEADER =
    "Timestamp,Raw_Lux,Clean_Lux,Smoothed_Lux,Baseline_Lux,Target_ON,Target_OFF,Top_Light,Bottom_Light";

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------

BH1750 lightMeter;
Servo  servoTop;
Servo  servoBottom;
WebServer server(80);

bool  topLightOn         = false;
bool  bottomLightOn      = false;
float cleanOutsideLux    = 0.0;
float smoothedLux        = -1.0;  // -1 = not yet initialized
float ambientBaselineLux = -1.0;
bool  sensorOk           = false;

unsigned long lastPollMs = 0;

// ---------------------------------------------------------------------------
// Time / formatting helpers
// ---------------------------------------------------------------------------

String getFormattedTime() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    return "N/A_Time";
  }
  char buffer[30];
  strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &timeinfo);
  return String(buffer);
}

// ---------------------------------------------------------------------------
// Servo actuation
// ---------------------------------------------------------------------------

// Servo::write() only supports integer degrees; this helper writes a raw
// microsecond pulse width so the bottom servo can hold sub-degree angles.
void writeServoAngle(Servo& sv, float angleDeg) {
  int pulseUs = (int)(544 + (angleDeg / 180.0f) * (2400 - 544));
  sv.writeMicroseconds(pulseUs);
}

// Presses the top servo to targetAngle and returns it to restAngle, mimicking
// a finger tapping the physical switch.
void pressSwitch(Servo& sv, int targetAngle, int restAngle, const char* label) {
  sv.write(targetAngle);
  delay(SERVO_PRESS_MS);
  sv.write(restAngle);
  delay(200);
  Serial.printf("[ACTUATOR] %s pressed to %d deg, returned to rest (%d deg)\n",
                label, targetAngle, restAngle);
}

// Moves the bottom servo directly to its on/off angle (no press-and-return).
void setBottomServoState(bool turnOn) {
  float angle = turnOn ? BOTTOM_SERVO_ON_ANGLE : BOTTOM_SERVO_OFF_ANGLE;
  writeServoAngle(servoBottom, angle);
  Serial.printf("[ACTUATOR] Bottom servo moved to %s position (%.1f deg)\n",
                turnOn ? "ON" : "OFF", angle);
}

// ---------------------------------------------------------------------------
// Logging (SPIFFS)
// ---------------------------------------------------------------------------

void appendLogToFile(const String& logLine) {
  if (SPIFFS.exists("/logs.csv")) {
    File checkFile = SPIFFS.open("/logs.csv", FILE_READ);
    if (checkFile && checkFile.size() > LOG_ROTATION_BYTES) {
      checkFile.close();
      File newFile = SPIFFS.open("/logs.csv", FILE_WRITE);
      if (newFile) {
        newFile.println(LOG_HEADER);
        newFile.close();
      }
    } else if (checkFile) {
      checkFile.close();
    }
  }

  File file = SPIFFS.open("/logs.csv", FILE_APPEND);
  if (!file) {
    Serial.println("[SPIFFS ERROR] Failed to open /logs.csv for appending");
    return;
  }
  file.println(logLine);
  file.close();
}

// ---------------------------------------------------------------------------
// Web dashboard
// ---------------------------------------------------------------------------

void handleRoot() {
  Serial.printf("[HTTP] GET / from %s\n", server.client().remoteIP().toString().c_str());
  String html = "<html><head><meta name='viewport' content='width=device-width, initial-scale=1.0'>";
  html += "<style>body{font-family:Arial,sans-serif;margin:30px;} a{display:inline-block;margin:8px 0;padding:10px 16px;background:#007bff;color:white;text-decoration:none;border-radius:4px;} a:hover{background:#0056b3;}</style></head><body>";
  html += "<h2>Adaptive Light Switch Monitor</h2>";
  html += "<p><a href='/logs'>View log spreadsheet</a></p>";
  html += "<p><a href='/download.csv' style='background:#28a745;'>Download raw CSV</a></p>";
  html += "<p><a href='/data' style='background:#6c757d;'>Live JSON status (/data)</a></p>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

// Recomputes the same adaptive thresholds used in loop(), for display only.
void computeDynamicThresholds(float& outOn, float& outOff) {
  outOn  = (ambientBaselineLux > 0) ? (ambientBaselineLux * DROP_PERCENTAGE) : MIN_ON_LUX;
  outOff = (ambientBaselineLux > 0) ? (ambientBaselineLux * RISE_PERCENTAGE) : MIN_OFF_LUX;
  if (outOn  < MIN_ON_LUX)  outOn  = MIN_ON_LUX;
  if (outOff < MIN_OFF_LUX) outOff = MIN_OFF_LUX;
}

void handleGetData() {
  Serial.printf("[HTTP] GET /data from %s\n", server.client().remoteIP().toString().c_str());

  float dynamicOn, dynamicOff;
  computeDynamicThresholds(dynamicOn, dynamicOff);

  String json = "{";
  json += "\"timestamp\":\"" + getFormattedTime() + "\",";
  json += "\"cleanOutsideLux\":" + String(cleanOutsideLux, 1) + ",";
  json += "\"smoothedLux\":" + String(smoothedLux, 1) + ",";
  json += "\"ambientBaselineLux\":" + String(ambientBaselineLux, 1) + ",";
  json += "\"targetOn\":" + String(dynamicOn, 1) + ",";
  json += "\"targetOff\":" + String(dynamicOff, 1) + ",";
  json += "\"topLightOn\":" + String(topLightOn ? "true" : "false") + ",";
  json += "\"bottomLightOn\":" + String(bottomLightOn ? "true" : "false") + ",";
  json += "\"sensorOk\":" + String(sensorOk ? "true" : "false");
  json += "}";

  server.send(200, "application/json", json);
}

void handleGetLogsTable() {
  Serial.printf("[HTTP] GET /logs from %s\n", server.client().remoteIP().toString().c_str());
  if (!SPIFFS.exists("/logs.csv")) {
    server.send(404, "text/plain", "Log file does not exist yet.");
    return;
  }

  File file = SPIFFS.open("/logs.csv", FILE_READ);
  if (!file) {
    server.send(500, "text/plain", "Failed to open log file.");
    return;
  }

  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html", "");

  static const char PAGE_HEADER[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Adaptive Light Switch — Log Spreadsheet</title>
  <style>
    body { font-family: Segoe UI, Tahoma, Geneva, Verdana, sans-serif; margin: 20px; background-color: #f4f6f9; color: #333; }
    h2 { color: #1f2d3d; }
    .btn { display: inline-block; padding: 8px 16px; margin-right: 8px; margin-bottom: 15px; background-color: #007bff; color: white; text-decoration: none; border-radius: 4px; font-weight: bold; font-size: 14px; border: none; cursor: pointer; }
    .btn:hover { background-color: #0056b3; }
    .btn-green { background-color: #28a745; }
    .btn-green:hover { background-color: #218838; }
    .btn-red { background-color: #dc3545; }
    .btn-red:hover { background-color: #c82333; }
    .table-container { overflow-x: auto; background: white; border-radius: 8px; box-shadow: 0 2px 8px rgba(0,0,0,0.1); max-height: 80vh; overflow-y: auto; }
    table { width: 100%; border-collapse: collapse; min-width: 900px; font-size: 14px; }
    th, td { padding: 10px 14px; text-align: left; border-bottom: 1px solid #e9ecef; }
    th { background-color: #343a40; color: white; position: sticky; top: 0; z-index: 10; }
    tr:nth-child(even) { background-color: #f8f9fa; }
    tr:hover { background-color: #e2e6ea; }
    .status-on { color: #28a745; font-weight: bold; }
    .status-off { color: #dc3545; font-weight: bold; }
  </style>
</head>
<body>
  <h2>Adaptive Light Switch — Log Spreadsheet</h2>
  <a class="btn btn-green" href="/download.csv" download>Download raw CSV</a>
  <a class="btn" href="/" style="background-color: #6c757d;">Home</a>
  <form action="/clear-logs" method="POST" style="display:inline;" onsubmit="return confirm('Permanently clear all log history?');">
    <button type="submit" class="btn btn-red">Clear history</button>
  </form>
  <div class="table-container">
    <table>
)rawliteral";

  server.sendContent(FPSTR(PAGE_HEADER));

  bool isHeaderRow = true;
  while (file.available()) {
    String line = file.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;

    String rowHtml = "<tr>";
    int startIndex = 0;
    int commaIndex = line.indexOf(',');

    auto appendCell = [&](const String& raw) {
      String value = raw;
      value.trim();
      if (isHeaderRow) {
        rowHtml += "<th>" + value + "</th>";
      } else if (value == "ON") {
        rowHtml += "<td class='status-on'>ON</td>";
      } else if (value == "OFF") {
        rowHtml += "<td class='status-off'>OFF</td>";
      } else {
        rowHtml += "<td>" + value + "</td>";
      }
    };

    while (commaIndex != -1) {
      appendCell(line.substring(startIndex, commaIndex));
      startIndex = commaIndex + 1;
      commaIndex = line.indexOf(',', startIndex);
    }
    appendCell(line.substring(startIndex));

    rowHtml += "</tr>\n";

    if (isHeaderRow) {
      rowHtml = "<thead>" + rowHtml + "</thead><tbody>";
      isHeaderRow = false;
    }

    server.sendContent(rowHtml);
  }

  file.close();
  server.sendContent("</tbody></table></div></body></html>");
  server.sendContent("");
}

void handleClearLogs() {
  Serial.printf("[HTTP] POST /clear-logs from %s\n", server.client().remoteIP().toString().c_str());

  File file = SPIFFS.open("/logs.csv", FILE_WRITE);
  if (file) {
    file.println(LOG_HEADER);
    file.close();
    Serial.println("[SPIFFS] Log history cleared.");
  } else {
    Serial.println("[SPIFFS ERROR] Failed to clear log file.");
  }

  server.sendHeader("Location", "/logs");
  server.send(303);
}

void handleDownloadCsv() {
  Serial.printf("[HTTP] GET /download.csv from %s\n", server.client().remoteIP().toString().c_str());
  if (!SPIFFS.exists("/logs.csv")) {
    server.send(404, "text/plain", "Log file does not exist.");
    return;
  }
  File file = SPIFFS.open("/logs.csv", FILE_READ);
  server.sendHeader("Content-Disposition", "attachment; filename=light_switch_logs.csv");
  server.streamFile(file, "text/csv");
  file.close();
}

void handlePostData() {
  Serial.printf("[HTTP] POST /data from %s\n", server.client().remoteIP().toString().c_str());
  if (!server.hasArg("plain")) {
    server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Empty body\"}");
    return;
  }
  String body = server.arg("plain");
  appendLogToFile("[POST_RECEIVED] " + body);
  server.send(200, "application/json", "{\"status\":\"success\",\"message\":\"Data logged\"}");
}

void handleNotFound() {
  Serial.printf("[HTTP] Unknown route %s from %s\n",
                server.uri().c_str(), server.client().remoteIP().toString().c_str());
  server.send(404, "text/plain", "404: Route not found");
}

// ---------------------------------------------------------------------------
// Setup / loop
// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n[INIT] Adaptive Light Switch booting");

  if (!SPIFFS.begin(true)) {
    Serial.println("[SPIFFS] Mount failed");
  } else {
    Serial.println("[SPIFFS] Mounted");
    if (!SPIFFS.exists("/logs.csv")) {
      File file = SPIFFS.open("/logs.csv", FILE_WRITE);
      if (file) {
        file.println(LOG_HEADER);
        file.close();
      }
    }
  }

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  sensorOk = lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE);
  Serial.println(sensorOk ? "[INIT] BH1750 OK" : "[INIT] BH1750 FAILED — check wiring");

  servoTop.attach(SERVO_TOP_PIN);
  servoBottom.attach(SERVO_BOTTOM_PIN);
  servoTop.write(TOP_SERVO_REST_ANGLE);
  writeServoAngle(servoBottom, BOTTOM_SERVO_OFF_ANGLE);
  delay(500);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("[WIFI] Connecting to ");
  Serial.print(WIFI_SSID);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("[WIFI] Connected, IP: ");
  Serial.println(WiFi.localIP());

  configTzTime(TIME_ZONE, NTP_SERVER);
  Serial.println("[NTP] Time sync requested");

  server.on("/", HTTP_GET, handleRoot);
  server.on("/logs", HTTP_GET, handleGetLogsTable);
  server.on("/clear-logs", HTTP_POST, handleClearLogs);
  server.on("/download.csv", HTTP_GET, handleDownloadCsv);
  server.on("/data", HTTP_GET, handleGetData);
  server.on("/data", HTTP_POST, handlePostData);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("[HTTP] Server listening on port 80");
}

void loop() {
  server.handleClient();

  if (millis() - lastPollMs < POLL_INTERVAL_MS) {
    return;
  }
  lastPollMs = millis();

  if (!sensorOk) {
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    sensorOk = lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE);
    if (!sensorOk) return;
  }

  float rawLux = lightMeter.readLightLevel();
  if (rawLux < 0) {
    sensorOk = false;
    return;
  }

  // Subtract the fixture's own light bleed when a light is on, so the
  // reading reflects actual ambient conditions rather than our own output.
  cleanOutsideLux = rawLux;
  if (topLightOn || bottomLightOn) {
    cleanOutsideLux = max(0.0f, cleanOutsideLux - INTERNAL_BLEED_LUX);
  }

  // Dual-EMA filter: fast tracks the current reading (smooths noise), slow
  // tracks the long-run ambient baseline the thresholds adapt against.
  if (smoothedLux < 0) {
    smoothedLux = cleanOutsideLux;
    ambientBaselineLux = cleanOutsideLux;
  } else {
    smoothedLux        = ALPHA_FAST * cleanOutsideLux + (1.0f - ALPHA_FAST) * smoothedLux;
    ambientBaselineLux = ALPHA_SLOW * cleanOutsideLux + (1.0f - ALPHA_SLOW) * ambientBaselineLux;
  }

  float dynamicOn, dynamicOff;
  computeDynamicThresholds(dynamicOn, dynamicOff);

  char logBuffer[160];
  snprintf(logBuffer, sizeof(logBuffer), "%s,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%s,%s",
           getFormattedTime().c_str(), rawLux, cleanOutsideLux, smoothedLux, ambientBaselineLux,
           dynamicOn, dynamicOff,
           topLightOn ? "ON" : "OFF",
           bottomLightOn ? "ON" : "OFF");

  Serial.println(logBuffer);
  appendLogToFile(String(logBuffer));

  if (smoothedLux < dynamicOn) {
    if (!topLightOn) {
      topLightOn = true;
      pressSwitch(servoTop, TOP_SERVO_ON_ANGLE, TOP_SERVO_REST_ANGLE, "Top servo");
    }
    if (!bottomLightOn) {
      bottomLightOn = true;
      setBottomServoState(true);
    }
  }

  if (smoothedLux > dynamicOff) {
    if (topLightOn) {
      topLightOn = false;
      pressSwitch(servoTop, TOP_SERVO_OFF_ANGLE, TOP_SERVO_REST_ANGLE, "Top servo");
    }
    if (bottomLightOn) {
      bottomLightOn = false;
      setBottomServoState(false);
    }
  }
}
