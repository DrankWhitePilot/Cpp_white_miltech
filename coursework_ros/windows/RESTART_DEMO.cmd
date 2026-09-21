@echo off
title Coursework AirSim - direct restart
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0RESTART_DEMO.ps1"
if errorlevel 1 (
  echo.
  pause
  exit /b 1
)
