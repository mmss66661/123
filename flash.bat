@echo off
setlocal

REM ============================================================
REM  STM32F407 Flash Script (Hero-Gimbal)
REM  Double-click to flash build\Debug\Hero-Gimbal.elf
REM ============================================================

REM ---- Config ----
set "CLT_ROOT=E:\STM32CubeCLT_1.18.0"
set "PROG=%CLT_ROOT%\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"
set "ELF=E:\stm32_project\arm\123\build\Debug\Hero-Gimbal.elf"
set "CONNECT_MODE=UR"
set "PORT=SWD"

REM ---- Check tools ----
if not exist "%PROG%" (
    echo [ERROR] STM32_Programmer_CLI.exe not found.
    echo         Check CLT_ROOT: %CLT_ROOT%
    pause
    exit /b 1
)

if not exist "%ELF%" (
    echo [ERROR] Firmware not found:
    echo        %ELF%
    echo         Build Debug version in CLion first.
    pause
    exit /b 1
)

echo ============================================================
echo  STM32F407 Flash Script
echo  Firmware: %ELF%
echo ============================================================
echo.

REM ---- 1. Connect ----
echo [1/3] Connecting to target...
"%PROG%" -c port=%PORT% mode=%CONNECT_MODE% >nul 2>&1
if errorlevel 1 (
    echo [ERROR] Cannot connect to target. Check:
    echo        - ST-Link connected
    echo        - Board powered
    pause
    exit /b 1
)
echo       Connected.

REM ---- 2. Flash + verify ----
echo.
echo [2/3] Flashing and verifying...
"%PROG%" -c port=%PORT% mode=%CONNECT_MODE% -w "%ELF%" -v
if errorlevel 1 (
    echo [ERROR] Flash failed.
    pause
    exit /b 1
)

REM ---- 3. Reset ----
echo.
echo [3/3] Resetting MCU...
"%PROG%" -c port=%PORT% mode=%CONNECT_MODE% -rst >nul 2>&1

echo.
echo ============================================================
echo  Flash complete. Firmware is running.
echo ============================================================
pause
