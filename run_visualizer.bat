@echo off
REM STM32 obstacle monitor. Resolve paths relative to this launcher.
cd /d "%~dp0"

echo ========================================
echo  STM32 Obstacle Monitor - STL-06NBJ
echo ========================================
echo.

REM Check if Python is installed
where python >nul 2>nul
if %ERRORLEVEL% NEQ 0 (
    echo [ERROR] Python not found! Please install Python 3.8 or later.
    echo Download from: https://www.python.org/downloads/
    pause
    exit /b 1
)

REM Check if pyserial is installed
python -c "import serial" >nul 2>nul
if %ERRORLEVEL% NEQ 0 (
    echo [INFO] Installing required package: pyserial
    python -m pip install pyserial
    echo.
)

echo [INFO] Starting radar visualizer...
echo.
echo Hardware connection:
echo   3.3V TTL USB-UART RX ^<-- PA2 (USART2_TX)
echo   USB-UART GND -- GND; adapter TX is unused
echo.
echo Instructions:
echo   1. Select COM port from dropdown
echo   2. Keep baud rate at 230400
echo   3. Click the connect button
echo.

python tools\radar_visualizer.py %*

if %ERRORLEVEL% NEQ 0 (
    echo.
    echo [ERROR] Visualizer exited with error code: %ERRORLEVEL%
    pause
)
