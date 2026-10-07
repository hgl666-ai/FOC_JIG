@echo off
rem ============================================================================
rem  FOC JIG U1 - full regression test runner
rem  Usage: double-click, or run  test\run_all_tests.cmd
rem  NOTE: ASCII-only on purpose. cmd.exe parses .cmd in the OEM codepage,
rem        so non-ASCII text corrupts line continuations and blocks.
rem ============================================================================
setlocal

set "ROOT=%~dp0.."
set "GCC=C:\Program Files\mingw64\bin\gcc.exe"
set "PY=D:\Develop\python\python.exe"
set "UV4=D:\Develop\Keil_MDK\UV4\UV4.exe"
set "INC=-I%ROOT%\test\hostshim -I%ROOT%\U1\Core\Inc"
set "SRC=%ROOT%\U1\Core\Src"
set FAILED=0

if exist "%GCC%" goto :have_gcc
where gcc >nul 2>nul
if errorlevel 1 set "GCC="
if not defined GCC goto :no_gcc
set "GCC=gcc"
:have_gcc

echo ============================================================
echo  1/8  Protocol layer host unit test
echo ============================================================
"%GCC%" -std=c99 -Wall -Wextra -O1 -I"%ROOT%\U1\Core\Inc" -o "%ROOT%\test\protocol_test.exe" "%ROOT%\test\protocol_test.c" "%ROOT%\U1\Core\Src\protocol_frame.c" "%SRC%\app_frame.c"
if errorlevel 1 goto :proto_build_fail
"%ROOT%\test\protocol_test.exe"
if errorlevel 1 goto :proto_fail
echo [PASS] protocol_test
goto :isp
:proto_build_fail
echo [FAIL] protocol_test compile error
set FAILED=1
goto :isp
:proto_fail
echo [FAIL] protocol_test
set FAILED=1

:isp
echo.
echo ============================================================
echo  2/8  ISP flow byte-level test (simulated STM32 bootloader)
echo       2a: verify off   2b: verify on   2c: pins confirmed
echo ============================================================
"%GCC%" -std=c99 -Wall -Wextra -O1 "-DISP_VERIFY_ENABLED=0" %INC% -o "%ROOT%\test\isp_test_v0.exe" "%ROOT%\test\isp_test.c" "%SRC%\isp_programmer.c"
if errorlevel 1 goto :isp_build_fail
"%ROOT%\test\isp_test_v0.exe"
if errorlevel 1 goto :isp_fail
echo [PASS] isp_test (verify off)

"%GCC%" -std=c99 -Wall -Wextra -O1 "-DISP_VERIFY_ENABLED=1" %INC% -o "%ROOT%\test\isp_test_v1.exe" "%ROOT%\test\isp_test.c" "%SRC%\isp_programmer.c"
if errorlevel 1 goto :isp_build_fail
"%ROOT%\test\isp_test_v1.exe"
if errorlevel 1 goto :isp_fail
echo [PASS] isp_test (verify on)

"%GCC%" -std=c99 -Wall -Wextra -O1 "-DISP_VERIFY_ENABLED=1" "-DISP_CTRL_PINS_CONFIRMED=1" %INC% -o "%ROOT%\test\isp_test_pins_v1.exe" "%ROOT%\test\isp_test.c" "%SRC%\isp_programmer.c"
if errorlevel 1 goto :isp_build_fail
"%ROOT%\test\isp_test_pins_v1.exe"
if errorlevel 1 goto :isp_fail
echo [PASS] isp_test (pins confirmed + verify on)
goto :app
:isp_build_fail
echo [FAIL] isp_test compile error
set FAILED=1
goto :app
:isp_fail
echo [FAIL] isp_test
set FAILED=1

:app
echo.
echo ============================================================
echo  3/8  Full-chain test (PC frame to target flash, all simulated)
echo ============================================================
"%GCC%" -std=c99 -Wall -Wextra -O1 "-DSWD_SIM_BUILD" %INC% -I"%ROOT%\test\swdsim" -o "%ROOT%\test\app_test.exe" "%ROOT%\test\app_test.c" "%SRC%\app.c" "%SRC%\app_frame.c" "%SRC%\uart_cfg.c" "%ROOT%\U1\Core\Src\protocol_frame.c" "%ROOT%\U1\Core\Src\uart_port.c" "%ROOT%\U1\Core\Src\uart_link.c" "%SRC%\fw_upload.c" "%SRC%\gd25q128.c" "%SRC%\swd_programmer.c" "%SRC%\swd_phy.c" "%SRC%\swd_dp.c" "%SRC%\swd_memap.c" "%SRC%\cortex_m.c" "%SRC%\stm32f4_flash.c" "%ROOT%\test\swdsim\sim_target.c"
if errorlevel 1 goto :app_build_fail
"%ROOT%\test\app_test.exe"
if errorlevel 1 goto :app_fail
echo [PASS] app_test
goto :meas
:app_build_fail
echo [FAIL] app_test compile error
set FAILED=1
goto :meas
:app_fail
echo [FAIL] app_test
set FAILED=1

:meas
echo.
echo ============================================================
echo  4/8  Measurement module test (divider to voltage, thresholds)
echo ============================================================
"%GCC%" -std=c99 -Wall -Wextra -O1 "-DMEAS_HOST_TEST" %INC% -o "%ROOT%\test\measure_test.exe" "%ROOT%\test\measure_test.c" "%SRC%\measure.c"
if errorlevel 1 goto :meas_build_fail
"%ROOT%\test\measure_test.exe"
if errorlevel 1 goto :meas_fail
echo [PASS] measure_test
goto :uart
:meas_build_fail
echo [FAIL] measure_test compile error
set FAILED=1
goto :uart
:meas_fail
echo [FAIL] measure_test
set FAILED=1

:uart
echo.
echo ============================================================
echo  5/8  UART port driver + U1 to U2 internal link
echo ============================================================
"%GCC%" -std=c99 -Wall -Wextra -O1 %INC% -o "%ROOT%\test\uart_test.exe" "%ROOT%\test\uart_test.c" "%ROOT%\U1\Core\Src\uart_cfg.c" "%ROOT%\U1\Core\Src\uart_port.c" "%ROOT%\U1\Core\Src\uart_link.c" "%ROOT%\U1\Core\Src\protocol_frame.c"
if errorlevel 1 goto :uart_build_fail
"%ROOT%\test\uart_test.exe"
if errorlevel 1 goto :uart_fail
echo [PASS] uart_test
goto :py
:uart_build_fail
echo [FAIL] uart_test compile error
set FAILED=1
goto :py
:uart_fail
echo [FAIL] uart_test
set FAILED=1

:py
echo.
echo ============================================================
echo  6/8  Cross-language byte check (Python vs C firmware)
echo ============================================================
if not exist "%PY%" goto :no_py
"%PY%" "%ROOT%\tools\foc_jig_tool.py" selftest
if errorlevel 1 goto :py_fail
echo [PASS] foc_jig_tool selftest
goto :dup
:py_fail
echo [FAIL] foc_jig_tool selftest
set FAILED=1
goto :dup
:no_py
echo [SKIP] python not found at %PY%

:dup
echo.
echo ============================================================
echo  7/8  Duplicated copies in sync (U1 vs U2)
echo ============================================================
rem  uart_port / uart_link / protocol_frame exist once per board (in each project folder);
rem  the two copies must stay byte-identical, or the link behaviour drifts silently.
rem  If the copies ever must differ on purpose, delete this stage (or make it a WARN).
set DUPFAIL=0
call :dupcmp uart_port.c
call :dupcmp uart_port.h
call :dupcmp uart_link.c
call :dupcmp uart_link.h
call :dupcmp protocol_frame.c
call :dupcmp protocol_frame.h
if "%DUPFAIL%"=="1" goto :dup_fail
echo [PASS] 6 duplicated files are identical
goto :build
:dup_fail
echo [FAIL] U1 and U2 copies differ -- sync them before flashing
set FAILED=1
goto :build

rem  ---- compare one duplicated file pair (arg: file name) ----
:dupcmp
if not exist "%ROOT%\U1\Core\Src\%1" goto :dupcmp_inc
fc /b "%ROOT%\U1\Core\Src\%1" "%ROOT%\U2\Core\Src\%1" >nul
if errorlevel 1 goto :dupcmp_bad
echo     %1 : ok
exit /b 0
:dupcmp_inc
fc /b "%ROOT%\U1\Core\Inc\%1" "%ROOT%\U2\Core\Inc\%1" >nul
if errorlevel 1 goto :dupcmp_bad
echo     %1 : ok
exit /b 0
:dupcmp_bad
echo     %1 : DIFFERENT
set DUPFAIL=1
exit /b 0

:build
echo.
echo ============================================================
echo  8/8  Keil MDK build (U1 + U2)
echo ============================================================
rem  Set SKIP_KEIL=1 to run host tests only (seconds, no UV4 needed)
if /i "%SKIP_KEIL%"=="1" goto :skip_build
if not exist "%UV4%" goto :no_uv4
call :keil U1 %ROOT%\U1\MDK-ARM\U1.uvprojx %ROOT%\_render\build_regress.log
call :keil U2 %ROOT%\U2\MDK-ARM\U2.uvprojx %ROOT%\_render\build_regress_u2.log
goto :end

rem  ---- build one project + judge the result (args: target name, project, log) ----
:keil
echo.
echo     --- Keil build %1 ---
rem  delete the old log first: its presence then means "this build finished"
if exist "%3" del "%3"
"%UV4%" -b "%2" -j0 -o "%3"
echo     (UV4 is a GUI app; waiting for the build to finish...)
set KEIL_WAIT=0
:keil_wait
set /a KEIL_WAIT=KEIL_WAIT+1
if %KEIL_WAIT% GTR 150 goto :keil_timeout
rem  the "Error(s)" summary line appears only when UV4 finished writing the log
if not exist "%3" goto :keil_sleep
findstr /C:"Error(s)" "%3" >nul
if not errorlevel 1 goto :keil_ready
:keil_sleep
ping -n 3 127.0.0.1 >nul
goto :keil_wait
:keil_timeout
echo [WARN] %1: no build summary after 5 min -- UV4 may still be running
:keil_ready
findstr /C:"Program Size" "%3"
findstr /C:"Error(s)" "%3"
rem  errors take precedence: a build with errors AND warnings must still FAIL
findstr /C:"0 Error(s)" "%3" >nul
if errorlevel 1 goto :keil_fail
findstr /C:"0 Warning(s)" "%3" >nul
if errorlevel 1 goto :keil_warn
echo [PASS] Keil build %1: 0 Error, 0 Warning
exit /b 0
:keil_warn
echo [WARN] Keil build %1 has WARNINGs -- see the log
exit /b 0
:keil_fail
echo [FAIL] Keil build %1 has ERRORs -- see the log
set FAILED=1
exit /b 0
:no_uv4
echo [SKIP] UV4.exe not found
goto :end
:skip_build
echo [SKIP] SKIP_KEIL=1

:end
echo.
if "%FAILED%"=="1" goto :bad
echo ===== ALL PASS (host tests; Keil builds unless SKIP_KEIL=1) =====
goto :done
:bad
echo ===== FAILURES PRESENT =====
endlocal & exit /b 1
:done
endlocal
