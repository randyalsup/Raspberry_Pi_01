<#
PowerShell helper to deploy Raspberry_Pi_01 to your Pi.
Usage (PowerShell):
    ./Raspberry_Pi_01/deploy.ps1

This script sets PI_USER/PI_HOST/PI_PATH and attempts to invoke the existing deploy.sh.
It will try to run the bash script using the `bash` command (Git Bash or WSL). If bash
is not available it will print the exact environment variables and command you can run
from a bash-capable prompt.
#>

param(
    [string]$PiUser = 'pi',
    [string]$PiHost = 'raspi01',    # hostname you provided
    [string]$PiHostIP = '192.168.1.106',
    [string]$PiPath = "/home/pi/Raspberry_Pi_01",
    [switch]$UseIP,
    [switch]$UIOnly
)

if ($UseIP) { $PiHostToUse = $PiHostIP } else { $PiHostToUse = $PiHost }

Write-Host ("Deploying to {0}@{1}:{2}" -f $PiUser, $PiHostToUse, $PiPath)

# Export environment variables for bash script
$env:PI_USER = $PiUser
$env:PI_HOST = $PiHostToUse
$env:PI_PATH = $PiPath

# Try to run the bash deploy script using bash (WSL or Git Bash)
if (Get-Command bash -ErrorAction SilentlyContinue) {
    if ($UIOnly) {
        Write-Host "Running tools/deploy_ui_only.sh via bash..."
        bash -lc "export PI_USER=$PiUser; export PI_HOST=$PiHostToUse; export PI_PATH=$PiPath; ./Raspberry_Pi_01/tools/deploy_ui_only.sh"
    } else {
        Write-Host "Running deploy.sh via bash..."
        # Use --login -i for Git Bash compatibility if needed. We'll call the script relative to repo root.
        bash -lc "export PI_USER=$PiUser; export PI_HOST=$PiHostToUse; export PI_PATH=$PiPath; ./Raspberry_Pi_01/deploy.sh"
    }
} else {
    Write-Host "bash not found in PATH."
    Write-Host "You can run the deploy script from Git Bash or WSL with these commands:"
    Write-Host ""
    Write-Host "export PI_USER=$PiUser"
    Write-Host "export PI_HOST=$PiHostToUse"
    Write-Host "export PI_PATH=$PiPath"
    if ($UIOnly) {
        Write-Host "./Raspberry_Pi_01/tools/deploy_ui_only.sh"
    } else {
        Write-Host "./Raspberry_Pi_01/deploy.sh"
    }
}

Write-Host "If the script completes, it will have uploaded files and installed requirements on the Pi."