@echo off
title Coursework AirSim
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0START_COURSEWORK.ps1"
if errorlevel 1 (
  echo.
  echo Startup failed. Read the error above.
  pause
  exit /b 1
)
