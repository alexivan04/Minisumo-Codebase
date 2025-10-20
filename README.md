# Minisumo ESP32 (ESP32-S3) — Competition Robot Firmware

This repository contains firmware for a minisumo competition robot running on an ESP32-S3 (tested). The code implements a small strategy framework with multiple selectable behaviors (modes), sensor reading, line detection (dohyo), and motor control via MCPWM-driven H-bridge DC motor drivers.

The firmware is written for ESP-IDF (CMake build). main features include:

- Multi-mode strategies (Bulldozer, Hunter, Matador, Calibration)
- Function-pointer based strategy selection for compact, extensible logic
- Sensor-based conditional retreat to avoid leaving the dohyo
- Proportional control based calibration/aiming mode
- Motor control using a bdc_motor component with MCPWM
- Start/stop module and mode selection via button

## Project status

- Last tested target: ESP32-S3
- Build system: ESP-IDF (CMake / idf.py)
- This repo is intended as firmware for a physical minisumo robot. It assumes a simple digital seat-of-the-pants sensor layout (digital distance/IR sensors and two digital line sensors).

## Hardware / Wiring / Pinout

Below are the GPIO assignments used by the firmware (see `main/main.c`):

- Onboard/status LED: ONBOARD_LED = GPIO 38
- Mode select button: MODE_BUTTON = GPIO 2
- Start/stop module (optional): START_STOP_MODULE = GPIO 1

Digital distance sensors (binary digital outputs):
- DIST_CENTER = GPIO 16
- DIST_R90    = GPIO 5
- DIST_R      = GPIO 15
- DIST_L90    = GPIO 6
- DIST_L      = GPIO 4

Line sensors (used to detect dohyo edge — logical polarity can be inverted by build flag):
- LINE_RIGHT = GPIO 35
- LINE_LEFT  = GPIO 13

Motor PWM / H-bridge pins (bdc motor component using MCPWM):
- Left motor (motor1): PWMA = GPIO 10, PWMB = GPIO 9
- Right motor (motor2): PWMA = GPIO 11, PWMB = GPIO 12

Notes:
- The code expects digital sensors (HIGH/LOW) rather than analog values. If you use analog distance sensors (e.g., analog IR), you'll need to adapt the code to read ADC values and convert them to thresholds.
- The `BLACK_FIELD` compile-time option inverts the line sensor logic; set it to the proper value for your dohyo/line sensor wiring (see "Configuration" below).

## Configuration (compile-time flags & runtime options)

Most useful options are in `main/main.c` near the top. Important flags:

- USE_START_STOP_MODULE — if defined, firmware waits for the `START_STOP_MODULE` pin to go HIGH before running. Comment out to start immediately.
- ACTIVE_DEBUG — enables more verbose logging. Use during development but be careful of log spam.
- BLACK_FIELD — set to `1` if the dohyo is black (line sensors read inverted); set to `0` for a white dohyo. This toggles inversion for `LINE_LEFT`/`LINE_RIGHT` reads.

Timing & behavior tuning constants (also in `main/main.c`):
- MOTOR_CONTROL_TIMER_PERIOD (ms) — low-latency motor control timer period (default 15 ms)
- SENSOR_READING_DELAY (ms) — delay inside strategy loop between iterations
- Kp, MIN_TURN_SPEED, MAX_TURN_SPEED — tuning for the calibration (proportional) mode

Mode selection:
- A button (MODE_BUTTON) cycles between modes 1..4. A short press increments the mode; a double-press confirms the selection. See `mode_select()` in `main.c`.

## Modes / Strategies

- Mode 1 — Bulldozer: aggressive, initial blitz forward then full-power attack when a target is seen.
- Mode 2 — Hunter: scans and attempts to center the target before attacking.
- Mode 3 — Matador: evasive opening move then attacks/counters (currently falls back to Hunter logic after opening maneuver).
- Mode 4 — Calibration Aiming: rotates in place using proportional control until the center sensor reports the target is centered.

The mode is selected at startup (via button). If `USE_START_STOP_MODULE` is defined, the robot will wait for the start signal pin to go high before executing the selected strategy.

## Build & flash (ESP-IDF)

Prerequisites:
- Install ESP-IDF (see https://docs.espressif.com/projects/esp-idf). This project uses the ESP-IDF build system (CMake + idf.py).

Recommended quick start:

1. Open a terminal and source ESP-IDF environment (example for zsh):

```bash
# from your ESP-IDF installation
. $HOME/esp/esp-idf/export.sh
```

2. Configure (optional):

```bash
idf.py set-target esp32s3  # if your IDF version supports multiple chips
idf.py menuconfig         # adjust sdkconfig if needed
```

3. Build:

```bash
idf.py build
```

4. Flash and monitor (replace /dev/ttyUSB0 with your serial port):

```bash
idf.py -p /dev/ttyUSB0 flash monitor
```

Tip: If you don't have a start/stop module wired and prefer to start immediately, comment out `#define USE_START_STOP_MODULE` in `main/main.c` before building.

## Runtime behaviour & quick checks

- On power-on the firmware runs `mode_select()` which uses the `MODE_BUTTON` (GPIO 2). Short press cycles mode; double-press confirms. The onboard LED blinks the selected mode count.
- When `START_STOP_MODULE` is enabled, the firmware waits for that pin to go HIGH to begin running. Otherwise it starts immediately.
- During operation, the motor controller is driven from a periodic esp_timer callback while a FreeRTOS task runs the chosen strategy.

## Debugging & tuning

- Enable `ACTIVE_DEBUG` to get more logs. Use `idf.py -p PORT monitor` to view logs.
- Adjust Kp / MIN_TURN_SPEED / MAX_TURN_SPEED for the calibration mode if turns are too slow or overshoot.
- If your line sensors behave inverted, toggle `BLACK_FIELD` in `main.c`.


## Files of interest

- `main/main.c` — main application, strategies, pin definitions and tunables
- `components/bdc_motor` — motor driver wrapper (MCPWM abstraction)
- `CMakeLists.txt`, `sdkconfig` — build configuration
