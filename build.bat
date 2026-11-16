@echo off
chcp 65001 >nul
title CAN Flash Tool - Build

echo ============================================
echo   CAN Flash Tool - Build EXE
echo ============================================
echo.

:: Проверка установки PyInstaller
python3 -m pip show pyinstaller >nul 2>&1
if %errorlevel% neq 0 (
    echo [INFO] PyInstaller not found. Installing...
    python3 -m pip install pyinstaller
    if %errorlevel% neq 0 (
        echo [ERROR] Failed to install PyInstaller!
        pause
        exit /b 1
    )
)

:: Проверка зависимостей
python3 -m pip show python-can >nul 2>&1
if %errorlevel% neq 0 (
    echo [INFO] python-can not found. Installing...
    python3 -m pip install python-can
    if %errorlevel% neq 0 (
        echo [ERROR] Failed to install python-can!
        pause
        exit /b 1
    )
)

python3 -m pip show python-can-candle >nul 2>&1
if %errorlevel% neq 0 (
    echo [INFO] python-can-candle not found. Installing...
    python3 -m pip install python-can-candle
    if %errorlevel% neq 0 (
        echo [ERROR] Failed to install python-can-candle!
        pause
        exit /b 1
    )
)

python3 -m pip show pyserial >nul 2>&1
if %errorlevel% neq 0 (
    echo [INFO] pyserial not found. Installing...
    python3 -m pip install pyserial
    if %errorlevel% neq 0 (
        echo [ERROR] Failed to install pyserial!
        pause
        exit /b 1
    )
)

:: Закрыть запущенные экземпляры приложения, чтобы освободить заблокированный exe
tasklist /FI "IMAGENAME eq CAN_Flash_Tool.exe" 2>nul | find /I "CAN_Flash_Tool.exe" >nul
if %errorlevel% equ 0 (
    echo [WARN] CAN_Flash_Tool.exe is running. Closing it...
    taskkill /F /IM CAN_Flash_Tool.exe >nul 2>&1
    timeout /t 1 /nobreak >nul
)

echo [INFO] Cleaning previous build...
if exist dist rmdir /s /q dist 2>nul
if exist build rmdir /s /q build 2>nul
if exist "CAN_Flash_Tool.spec" del /f /q "CAN_Flash_Tool.spec" 2>nul
echo.

echo [INFO] Starting build...
echo.

:: Сборка единого exe файла
python3 -m PyInstaller --onefile --noconsole ^
    --name "CAN_Flash_Tool" ^
    --icon can_flash_icon.ico ^
    --add-data "azure.tcl;." ^
    --add-data "theme;theme" ^
    --hidden-import can ^
    --hidden-import candle ^
    --hidden-import candle.candle_bus ^
    --hidden-import candle_api ^
    --collect-all candle ^
    --collect-all candle_api ^
    --hidden-import can.interfaces.slcan ^
    --hidden-import can.interfaces.virtual ^
    --hidden-import can.interfaces.serial ^
    --hidden-import serial ^
    --hidden-import serial.tools.list_ports ^
    --hidden-import intelhex ^
    --hidden-import can_bootloader_sim ^
    can_flash_gui.py

if %errorlevel% neq 0 (
    echo.
    echo [ERROR] Build failed!
    pause
    exit /b 1
)

echo.
echo ============================================
echo   Build complete!
echo   Output: dist\CAN_Flash_Tool.exe
echo   Size:
echo ============================================
for %%I in (dist\CAN_Flash_Tool.exe) do echo   %%~zI bytes

:: Очистка временных файлов PyInstaller
echo.
echo [INFO] Cleaning temporary build files...
rmdir /s /q build 2>nul
del /f /q "CAN_Flash_Tool.spec" 2>nul

echo.
