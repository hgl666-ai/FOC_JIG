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
echo  1/5  Protocol layer host unit test
echo ============================================================
"%GCC%" -std=c99 -Wall -Wextra -O1 -I"%ROOT%\U1\Core\Inc" -o "%ROOT%\test\protocol_test.exe" "%ROOT%\test\protocol_test.c" "%SRC%\protocol_frame.c" "%SRC%\app_frame.c"
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
echo  2/5  ISP flow byte-level test (simulated STM32 bootloader)
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
echo  3/5  Full-chain test (PC frame to target flash, all simulated)
echo ============================================================
"%GCC%" -std=c99 -Wall -Wextra -O1 %INC% -o "%ROOT%\test\app_test.exe" "%ROOT%\test\app_test.c" "%SRC%\app.c" "%SRC%\app_frame.c" "%SRC%\protocol_frame.c" "%SRC%\fw_upload.c" "%SRC%\gd25q128.c" "%SRC%\isp_programmer.c"
if errorlevel 1 goto :app_build_fail
"%ROOT%\test\app_test.exe"
if errorlevel 1 goto :app_fail
echo [PASS] app_test
goto :py
:app_build_fail
echo [FAIL] app_test compile error
set FAILED=1
goto :py
:app_fail
echo [FAIL] app_test
set FAILED=1

:py
echo.
echo ============================================================
echo  4/5  Cross-language byte check (Python vs C firmware)
echo ============================================================
if not exist "%PY%" goto :no_py
"%PY%" "%ROOT%\tools\foc_jig_tool.py" selftest
if errorlevel 1 goto :py_fail
echo [PASS] foc_jig_tool selftest
goto :build
:py_fail
echo [FAIL] foc_jig_tool selftest
set FAILED=1
goto :build
:no_py
echo [SKIP] python not found at %PY%

:build
echo.
echo ============================================================
echo  5/5  Keil MDK build
echo ============================================================
if not exist "%UV4%" goto :no_uv4
"%UV4%" -b "%ROOT%\U1\MDK-ARM\U1.uvprojx" -j0 -o "%ROOT%\_render\build_regress.log"
echo     UV4 is a GUI-subsystem app; waiting for it to finish...
:waitloop
ping -n 3 127.0.0.1 >nul
tasklist /fi "imagename eq UV4.exe" 2>nul | find /i "UV4.exe" >nul
if not errorlevel 1 goto :waitloop
echo     --- Keil build summary ---
findstr /C:"Program Size" "%ROOT%\_render\build_regress.log"
findstr /C:"Error(s)" "%ROOT%\_render\build_regress.log"
findstr /C:"0 Warning(s)" "%ROOT%\_render\build_regress.log" >nul
if errorlevel 1 goto :build_warn
findstr /C:"0 Error(s)" "%ROOT%\_render\build_regress.log" >nul
if errorlevel 1 goto :build_fail
echo [PASS] Keil build: 0 Error, 0 Warning
goto :end
:build_fail
echo [FAIL] Keil build has ERRORs -- see _render\build_regress.log
set FAILED=1
goto :end
:build_warn
echo [WARN] Keil build has WARNINGs -- see _render\build_regress.log
goto :end
:no_uv4
echo [SKIP] UV4.exe not found

:end
echo.
if "%FAILED%"=="1" goto :bad
echo ===== host tests: ALL PASS =====
goto :done
:bad
echo ===== host tests: FAILURES PRESENT =====
endlocal & exit /b 1
:done
endlocal
