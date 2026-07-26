@echo off
setlocal
cd /d "%~dp0"
if not exist "%~dp0.flash_gui" mkdir "%~dp0.flash_gui"
start "ESP32-S3 Car Flasher" powershell.exe -NoProfile -WindowStyle Hidden -ExecutionPolicy Bypass -STA -File "%~dp0flash_gui.ps1" 1>>"%~dp0.flash_gui\launcher.log" 2>&1
