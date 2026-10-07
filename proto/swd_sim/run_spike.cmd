@echo off
rem ============================================================================
rem  SWD programming engine spike - build and run (host, pure simulation)
rem
rem  注意：SWD 引擎已经搬进固件源码树了（U1/Core/Src/swd_*.c），
rem        这个 spike 直接编译**固件真源码**（-DSWD_SIM_BUILD 只切换最底层
rem        5 个 GPIO 原语），线级目标模型在 test/swdsim。
rem  ASCII-only on purpose: cmd.exe parses .cmd in the OEM codepage.
rem ============================================================================
setlocal

set "ROOT=%~dp0"
set "REPO=%~dp0..\.."
set "GCC=C:\Program Files\mingw64\bin\gcc.exe"
set "OUT=%ROOT%build"
set "INC=-I%REPO%\test\swdsim -I%REPO%\U1\Core\Inc"

if exist "%GCC%" goto :have_gcc
where gcc >nul 2>nul
if errorlevel 1 goto :no_gcc
set "GCC=gcc"
:have_gcc

if not exist "%OUT%" mkdir "%OUT%"

echo [build] compiling...
"%GCC%" -std=c99 -Wall -Wextra -O1 -DSWD_SIM_BUILD %INC% ^
  -o "%OUT%\spike.exe" ^
  "%ROOT%tests\spike_main.c" ^
  "%REPO%\U1\Core\Src\swd_phy.c" ^
  "%REPO%\U1\Core\Src\swd_dp.c" ^
  "%REPO%\U1\Core\Src\swd_memap.c" ^
  "%REPO%\U1\Core\Src\cortex_m.c" ^
  "%REPO%\U1\Core\Src\stm32f4_flash.c" ^
  "%REPO%\test\swdsim\sim_hal.c" ^
  "%REPO%\test\swdsim\sim_target.c" > "%OUT%\build.log" 2>&1
if errorlevel 1 goto :build_fail
type "%OUT%\build.log"
echo [build] OK
echo.
"%OUT%\spike.exe" > "%OUT%\run.log" 2>&1
set "RC=%ERRORLEVEL%"
type "%OUT%\run.log"
echo.
if not "%RC%"=="0" goto :run_fail
echo [spike] ALL CHECKS PASSED
endlocal & exit /b 0

:build_fail
echo [build] FAILED
type "%OUT%\build.log"
endlocal & exit /b 2

:run_fail
echo [spike] FAILURES PRESENT (see above)
endlocal & exit /b 1

:no_gcc
echo [build] gcc not found
endlocal & exit /b 3
