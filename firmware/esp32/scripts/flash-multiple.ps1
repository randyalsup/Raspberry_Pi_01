<#
Simple helper to upload PlatformIO environments to multiple attached ESP32s.

Behavior:
- Shows `platformio device list` output to help you see COM ports.
- Prompts for the COM port for the remote and robot (press Enter to skip/auto-detect).
- Runs platformio upload per environment using the provided ports.

Usage:
  powershell -ExecutionPolicy Bypass -File .\scripts\flash-multiple.ps1
#>

param()

function Show-Header($t) { Write-Host "`n=== $t ===`n" -ForegroundColor Cyan }

Push-Location (Split-Path -Path $MyInvocation.MyCommand.Path -Parent)
Pop-Location

Show-Header "PlatformIO device list"
platformio device list

Write-Host "`nEnter COM port for remote (e.g. COM3). Press Enter to let PlatformIO auto-detect:" -NoNewline
$remotePort = Read-Host

Write-Host "Enter COM port for robot (e.g. COM4). Press Enter to let PlatformIO auto-detect:" -NoNewline
$robotPort = Read-Host

if ($remotePort -ne "") {
  Write-Host "Uploading remote to $remotePort..." -ForegroundColor Green
  platformio run -e esp32_remote -t upload --upload-port $remotePort
} else {
  Write-Host "Uploading remote (auto-detect)..." -ForegroundColor Yellow
  platformio run -e esp32_remote -t upload
}

if ($robotPort -ne "") {
  Write-Host "Uploading robot to $robotPort..." -ForegroundColor Green
  platformio run -e esp32_robot -t upload --upload-port $robotPort
} else {
  Write-Host "Uploading robot (auto-detect)..." -ForegroundColor Yellow
  platformio run -e esp32_robot -t upload
}

Write-Host "`nDone." -ForegroundColor Cyan
