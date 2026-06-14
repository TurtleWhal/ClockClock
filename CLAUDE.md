# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A 24-clock "ClockClock"-style kinetic display: an 8×3 grid of analog clock faces whose hands are positioned by stepper motors to draw digits, text, and animations. The hardware is ESP32-S3 based. This repo holds the firmware (`Code/`), the KiCad schematic (`Schematic/`), and the CAD (`CAD/`).

There are **two firmware programs** plus a shared library, all PlatformIO projects under `Code/`:

- **`Code/Master`** — one ESP32-S3. Connects to WiFi, serves a web UI, keeps NTP time, decides what to display, and streams motor commands out over UART to the module chain. Holds the font, animations, and calibration logic.
- **`Code/Module`** — identical firmware flashed onto every module board. Each module drives **4 clocks = 8 stepper motors**. Modules are wired in a **daisy chain**; this firmware handles receiving its own packet, forwarding the rest downstream, and the actual microstepping motor control.
- **`Code/ClockSerial`** — a small shared library declared as a `lib_deps` of both. (Note: the live comms path uses the `SerialTransfer` library, not `ClockSerial` — don't assume `ClockSerial` is on the hot path.)

Open `Code/all.code-workspace` to get both Master and Module as workspace folders at once.

## Build / upload / monitor

All commands use the PlatformIO CLI (`pio`) and must run from the relevant project directory (`Code/Master` or `Code/Module`).

```bash
# Master — build + flash over USB
cd Code/Master && pio run -e Master -t upload

# Master — flash over the air (device must be on WiFi as clockclock.local)
cd Code/Master && pio run -e MasterOTA -t upload

# Master — upload the web UI assets (data/ -> SPIFFS). Required after editing
# index.html / style.css / clockclock-cv.js, separate from the firmware flash.
cd Code/Master && pio run -e Master -t uploadfs

# Module — build + flash over USB (uses the custom board in Code/Module/boards/)
cd Code/Module && pio run -e Module -t upload

# Serial monitor (115200 for Master; Module logs at 1000000)
pio device monitor
```

There is no test suite — this is embedded firmware verified on hardware.

### Versioning
Every build runs `autoincrement.py` (a `post:` extra_script) which bumps `VERSION_BUILD` and stamps the date/time in `src/version.h`. **`version.h` changes on every build** — expect it dirty in git and don't hand-edit it.

### Module firmware updates (the interesting path)
Modules are **not** normally flashed individually after deployment. Instead the Master receives a `firmware.bin` over its web UI (stored in PSRAM), then cascades it down the UART chain. When a module gets the "start firmware" header (address 201) it reconfigures its UART via `gpio_matrix_in`/`gpio_matrix_out` to **mirror RX straight to TX**, so the entire chain updates simultaneously, then applies it with the `Update` library and reboots. See `sendFile()` in Master `main.cpp` and the `firmwareUpdate` handling in Module `main.cpp`.

## Architecture

### Display buffer and the Master→Module pipeline
The Master holds the display as `buffer[WIDTH=8][HEIGHT=3][2]` of `MotorControl_t` — one entry per clock per hand (index `[..][..][0]`=hour hand, `[1]`=minute hand). Drawing (digits via `drawChar`/`Font.h`, time, animations like `wave`/`radialwave`) just mutates this buffer, then `writeBuffer()` ships it.

`writeBuffer()` does two non-obvious things:
1. **Remaps the 8×3 X-Y grid into the physical daisy-chain order**, which is a serpentine/Z layout (every other row's columns are reversed). The 24 clocks are grouped into `MODULES = 6` modules of 4 clocks each.
2. Sends one `SerialTransfer` packet per module, **highest address first**, each tagged with the module's address.

### Daisy-chain addressing
Every module runs the same code and has no fixed ID. The protocol is **relative**: a packet carries an `address` byte. If `address > 0`, the module decrements it and forwards the packet downstream; if `address == 0`, the packet is for *this* module and it applies the 4-clock × 2-hand `MotorControl_t[4][2]` to its motors. Addresses `>= 200` are control opcodes (200 = chain init/pin discovery, 201 = firmware start, 202 = firmware end, 220 = calibration zero).

At boot a module doesn't know which of its two UART pins (`UART_A`/`UART_B`) faces upstream. It listens on both and the first one to receive data becomes input; the other becomes the forwarding output (`Code/Module/src/main.cpp` `setup()`).

### `MotorControl_t` — the universal command unit
Defined in `Code/Master/src/motorcontrol.h` and `#include`d by the Module via a **relative path** (`../../Master/src/motorcontrol.h`) — this header is the shared contract between the two projects; changing it affects both. Fields: `position` (degrees), `speed`, `acceleration`, `direction` (CW/CCW/SHORTEST), `time` (ms to reach target — drives a trapezoidal/triangular velocity profile), `keepRunning` (continuous spin mode), and `optimize` (let the module swap which physical hand goes to which target to minimize travel — **must be true on both hands** of a clock to take effect).

### Motor control (Module)
`StepperMotor` (`Code/Module/src/StepperMotor.h`) does sine-microstepping at `MICROSTEPS=32`, driven off an `esp_timer` that re-arms itself every ~100µs. It implements both position mode (with accel/decel profile, time-based or distance-based speed scaling) and continuous mode. PWM is **not** the LEDC peripheral — it's a hand-rolled software PWM in `pwm.cpp` running a busy-loop `PWMTask` pinned to core 1, toggling GPIO via direct `REG_WRITE` to the `GPIO_OUT`/`GPIO_OUT1` registers (note the >GPIO32 pins use the `_OUT1_` register and a `-32` bit offset). `pins.h` maps each of the 4 motors' 8 coil pins.

### Web UI + calibration
`Code/Master/data/` is served from SPIFFS: `index.html` (control panel — mode selection, custom text, firmware upload, reboot), `clockclock-cv.js` (in-browser **computer vision** that reads hand angles from a camera image), and `style.css`. The Master pushes live state to the page over a WebSocket at `/ws` (`sendStatus()` emits the current mode and the full per-hand motor state so the page can emulate the real motion).

**Calibration** is a 3-step camera-assisted routine (WebSocket `type:"calibrate"`, `step` 0/1/2): step 0 parks all hands, step 1 nudges one hand +180° and the page reports detected angles, step 2 reports angles again and the Master figures out per-hand offsets (handling that the CV may reorder the two hands between captures). The result zeros each motor; opcode 220 tells modules to treat straight-down as 90°.

### Display modes
The Master's `loop()` switches on `mode` (`MODE_CYCLE`, `MODE_TIME`, `MODE_CUSTOM`, `MODE_CLEAR`, `MODE_DIAGONAL`, and animation modes). `MODE_CYCLE` is the default: shows the time, refreshes on minute change, and at second==20 (between 6am–10pm) plays a random animation. Mode names map to these enums in both `handleWebSocketMessage()` (incoming) and `sendStatus()` (outgoing) — keep both in sync when adding a mode.

## Hardware notes
- Both Master and Module target **ESP32-S3 with PSRAM** (QIO). Master uses `esp32-s3-devkitc-1`; Module uses a custom board def in `Code/Module/boards/clockclock_module.json`.
- USB CDC on boot is enabled (`ARDUINO_USB_MODE=1`, `ARDUINO_USB_CDC_ON_BOOT=1`) — serial output goes over native USB.
- Master partition layout (`partitions.csv`) reserves dual OTA app slots + SPIFFS + coredump; keep it in sync if firmware size grows.
- `ESP32-S3FH4R2.pdf` in the repo root is the MCU datasheet (the pin comments in `pins.h` reference its tables).
