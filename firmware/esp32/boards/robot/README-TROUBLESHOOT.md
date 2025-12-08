ESP32 Robot flashing troubleshooting
===================================

If you see the error "Wrong boot mode detected (0x13)! The chip needs to be in download mode" when uploading, follow these steps.

1) Basic checks
- Use a good USB data cable (some cables are power-only). Try another cable/port.
- Close any serial monitors (PlatformIO device monitor, Arduino IDE, etc.) that may hold the port open.
- Confirm the correct COM port (COM3) and that only one program is connected to it.

2) Manual boot into download (flash) mode
 - Many ESP32 dev-boards have two buttons: "BOOT" (GPIO0) and "EN" or "RST" (reset).
 - To enter download mode manually:
   a) Press and hold the BOOT button (keep it pressed).
   b) While holding BOOT, press and release the EN (reset) button.
   c) **IMPORTANT: Keep holding BOOT through the entire connect and download cycle** - do NOT release it until upload completes.
 - Start the upload command while holding BOOT, and release only after you see "Leaving..." or "Hard resetting via RTS pin..." in the output.

3) Auto-reset circuit issues
- If your board lacks proper DTR/RTS wiring (auto-reset/auto-boot), PlatformIO's automatic reset may fail. Use the manual BOOT+EN sequence above while starting the upload.

4) Slow the upload speed (we set `upload_speed = 115200` in the env)
- Some CH340/FTDI/CP210x adapters are flaky at high baud. Using 115200 reduces errors.

5) If it still fails
- Reboot the PC to clear any drivers/locks.
- Try another USB port or a powered USB hub.
- Try `& $pioPath run -v` for verbose output and paste the logs here so we can diagnose.
