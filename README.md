# Adaptive Light Switch

An ESP32-C3 project that automatically flips an existing exterior light switch based on ambient light, with no rewiring required.

## How it works

- A **BH1750** light sensor reports ambient lux to the ESP32-C3 every 2 minutes.
- Two **SG-90 servos**, mounted directly on the existing switch plate, physically press the switch — one presses-and-returns like a finger tap, the other holds a fixed on/off position.
- Instead of a single fixed lux threshold, the firmware runs a **dual exponential moving average**: a fast filter smooths short-term sensor noise, and a much slower filter tracks the true ambient baseline as daylight drifts over hours. On/off thresholds are computed as a percentage of that slow-tracked baseline (a smaller drop to turn on, a larger rise to turn off), so the system adapts to how bright a given day actually is instead of using one hardcoded value for every condition.
- A small **web server** on the ESP32 exposes a live dashboard, a JSON status endpoint (`/data`), and a browsable/downloadable CSV log of every reading and switch event, backed by the device's own flash storage (SPIFFS) — no external server or database.

## Hardware

- ESP32-C3 DevKit
- BH1750 ambient light sensor (I2C)
- 2x SG-90 servo motors

## Setup

1. Copy `secrets.h.example` to `secrets.h` and fill in your Wi-Fi credentials. `secrets.h` is gitignored and will never be committed.
2. Wire the BH1750 to the I2C pins defined in `AdaptiveLightSwitch.ino` (`I2C_SDA_PIN` / `I2C_SCL_PIN`), and the two servos to `SERVO_TOP_PIN` / `SERVO_BOTTOM_PIN`.
3. Open `AdaptiveLightSwitch.ino` in the Arduino IDE and flash it to the board.
4. On boot, the device connects to Wi-Fi and prints its IP address over serial — visit that IP in a browser for the dashboard.

## Endpoints

| Route | Method | Description |
|---|---|---|
| `/` | GET | Dashboard home |
| `/data` | GET | Live JSON status (lux readings, thresholds, switch state) |
| `/logs` | GET | Browsable log spreadsheet |
| `/download.csv` | GET | Download the raw CSV log |
| `/clear-logs` | POST | Wipe log history |
