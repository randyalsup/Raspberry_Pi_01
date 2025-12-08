# ESP32 firmware (PlatformIO)

This folder contains PlatformIO projects for the ESP32 devices used by the project.

Layout
- `platformio.ini` — two environments: `esp32_remote` and `esp32_robot`.
- `boards/remote/` — PlatformIO project for the handheld remote (example sketch in `src/`).
- `boards/robot/` — PlatformIO project for the robot-side ESP32 (example sketch in `src/`).
- `scripts/` — helper scripts for flashing multiple attached devices.

Quick start (Windows PowerShell)

1. Install PlatformIO (via VSCode extension or `pip install platformio`).
2. Build an environment:

```powershell
cd firmware\esp32
platformio run -e esp32_remote
```

3. Upload to a specific COM port (recommended when multiple devices are attached):

```powershell
platformio run -e esp32_remote -t upload --upload-port COM3
platformio run -e esp32_robot -t upload --upload-port COM4
```

See `scripts/flash-multiple.ps1` for a helper that prompts for ports.
