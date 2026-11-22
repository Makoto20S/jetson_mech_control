@echo off
setlocal
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0tools\bench_panel\Open-Bench.ps1" -Nx
if errorlevel 1 (
  pause
  exit /b 1
)
