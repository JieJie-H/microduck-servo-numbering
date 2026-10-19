@echo off
cd /d "%~dp0"
echo MicroDuck servo numbering guide
echo Opening http://127.0.0.1:8090/
echo Close FD / servo-web first: only one program can use the COM port.
python --version >nul 2>nul
if errorlevel 1 (
  py -3 --version >nul 2>nul
  if errorlevel 1 (
    echo Python not found. Install Python 3.11+ and enable "Add Python to PATH".
    pause
    exit /b 1
  )
  py -3 server.py --open
) else (
  python server.py --open
)
pause
