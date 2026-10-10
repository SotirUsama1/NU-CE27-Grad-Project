@echo off
rem Double-click this file to start the dashboard. Close this window to stop it.
title Graduation Project Dashboard
cd /d "%~dp0"

where node >nul 2>nul
if errorlevel 1 (
  echo Node.js is not installed. Install it from https://nodejs.org and try again.
  pause
  exit /b 1
)

if not exist node_modules (
  echo Installing packages...
  call npm install --no-audit --no-fund
)

rem Open the browser a few seconds after the server starts
start "" /b cmd /c "timeout /t 3 /nobreak >nul & start http://localhost:3000"

node server.js
pause
