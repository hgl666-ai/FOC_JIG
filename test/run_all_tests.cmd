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
echo  1/12  Protocol layer host unit test
echo ============================================================
"%GCC%" -std=c99 -Wall -Wextra -O1 -I"%ROOT%\U1\Core\Inc" -o "%ROOT%\test\protocol_test.exe" "%ROOT%\test\protocol_test.c" "%ROOT%\U1\Core\Src\protocol_frame.c" "%SRC%\app_frame.c"
if errorlevel 1 goto :proto_build_fail
"%ROOT%\test\protocol_test.exe"
if errorlevel 1 goto :proto_fail
echo [PASS] protocol_test
goto :app

:app
echo.
echo ============================================================
echo  2/12  Full-chain test (PC frame to target flash, all simulated)
echo ============================================================
"%GCC%" -std=c99 -Wall -Wextra -O1 "-DSWD_SIM_BUILD" "-DMEAS_HOST_TEST" %INC% -I"%ROOT%\test\swdsim" -o "%ROOT%\test\app_test.exe" "%ROOT%\test\app_test.c" "%SRC%\app.c" "%SRC%\app_frame.c" "%SRC%\uart_cfg.c" "%SRC%\measure.c" "%SRC%\meas_pack.c" "%SRC%\pulse_cap.c" "%SRC%\pulse_pack.c" "%SRC%\enc_plan.c" "%SRC%\phase_chk.c" "%SRC%\enc_chk.c" "%ROOT%\U1\Core\Src\protocol_frame.c" "%ROOT%\U1\Core\Src\uart_port.c" "%ROOT%\U1\Core\Src\uart_link.c" "%SRC%\fw_upload.c" "%SRC%\gd25q128.c" "%SRC%\swd_programmer.c" "%SRC%\swd_phy.c" "%SRC%\swd_dp.c" "%SRC%\swd_memap.c" "%SRC%\cortex_m.c" "%SRC%\stm32f4_flash.c" "%ROOT%\test\swdsim\sim_target.c"
if errorlevel 1 goto :app_build_fail
"%ROOT%\test\app_test.exe"
if errorlevel 1 goto :app_fail
echo [PASS] app_test
goto :appdut

rem  ---- 2b) the SAME test rebuilt with the DUT link enabled (docs/15) ----
rem  In that build port 0 belongs to the DUT link, so the fake-U2 cases (T16)
rem  are compiled out and the DUT passthrough cases (T17) run instead.
:appdut
echo.
echo     --- app_test in DUT-link mode ---
"%GCC%" -std=c99 -Wall -Wextra -O1 "-DSWD_SIM_BUILD" "-DMEAS_HOST_TEST" "-DJIG_DUT_LINK=1" %INC% -I"%ROOT%\test\swdsim" -o "%ROOT%\test\app_dut_test.exe" "%ROOT%\test\app_test.c" "%SRC%\app.c" "%SRC%\app_frame.c" "%SRC%\uart_cfg.c" "%SRC%\measure.c" "%SRC%\meas_pack.c" "%SRC%\pulse_cap.c" "%SRC%\pulse_pack.c" "%SRC%\enc_plan.c" "%SRC%\phase_chk.c" "%SRC%\enc_chk.c" "%SRC%\cur_chk.c" "%SRC%\vol_chk.c" "%ROOT%\U1\Core\Src\protocol_frame.c" "%ROOT%\U1\Core\Src\uart_port.c" "%ROOT%\U1\Core\Src\uart_link.c" "%SRC%\fw_upload.c" "%SRC%\gd25q128.c" "%SRC%\swd_programmer.c" "%SRC%\swd_phy.c" "%SRC%\swd_dp.c" "%SRC%\swd_memap.c" "%SRC%\cortex_m.c" "%SRC%\stm32f4_flash.c" "%SRC%\dut_proto.c" "%ROOT%\test\swdsim\sim_target.c"
if errorlevel 1 goto :appdut_build_fail
"%ROOT%\test\app_dut_test.exe"
if errorlevel 1 goto :appdut_fail
echo [PASS] app_test (DUT link mode)
goto :meas
:appdut_build_fail
echo [FAIL] app_test (DUT link mode) compile error
set FAILED=1
goto :meas
:appdut_fail
echo [FAIL] app_test (DUT link mode)
set FAILED=1
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
echo  3/12  Measurement module test (divider to voltage, thresholds)
echo ============================================================
"%GCC%" -std=c99 -Wall -Wextra -O1 "-DMEAS_HOST_TEST" "-DMEAS_PRINT_ENABLE=1" %INC% -o "%ROOT%\test\measure_test.exe" "%ROOT%\test\measure_test.c" "%SRC%\measure.c" "%SRC%\meas_pack.c" "%SRC%\pulse_cap.c" "%SRC%\pulse_pack.c" "%ROOT%\U1\Core\Src\protocol_frame.c"
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
echo  4/12  UART port driver + U1 to U2 internal link
echo ============================================================
"%GCC%" -std=c99 -Wall -Wextra -O1 %INC% -o "%ROOT%\test\uart_test.exe" "%ROOT%\test\uart_test.c" "%ROOT%\U1\Core\Src\uart_cfg.c" "%ROOT%\U1\Core\Src\uart_port.c" "%ROOT%\U1\Core\Src\uart_link.c" "%ROOT%\U1\Core\Src\protocol_frame.c" "%ROOT%\U1\Core\Src\meas_pack.c" "%ROOT%\U1\Core\Src\pulse_cap.c" "%ROOT%\U1\Core\Src\pulse_pack.c"
if errorlevel 1 goto :uart_build_fail
"%ROOT%\test\uart_test.exe"
if errorlevel 1 goto :uart_fail
echo [PASS] uart_test
goto :rep
:uart_build_fail
echo [FAIL] uart_test compile error
set FAILED=1
goto :rep
:uart_fail
echo [FAIL] uart_test
set FAILED=1

:rep
echo.
echo ============================================================
echo  5/12  TIM pulse capture (edge timestamps to period/duty)
echo ============================================================
"%GCC%" -std=c99 -Wall -Wextra -O1 %INC% -o "%ROOT%\test\pulse_test.exe" "%ROOT%\test\pulse_test.c" "%SRC%\pulse_cap.c" "%SRC%\pulse_pack.c"
if errorlevel 1 goto :rep_build_fail
"%ROOT%\test\pulse_test.exe"
if errorlevel 1 goto :rep_fail
echo [PASS] pulse_test
goto :pulsehw

rem  ---- 5b) pulse_cfg.c ???????? ???????? (docs/03 D4/D5) ----
rem  ??????????????test/timsim (TIM ?????+ NVIC ?????+ PRIMASK),
rem  ????-I ?????timsim ???, ??? hostshim??
:pulsehw
echo.
echo     --- pulse_cfg (register-level TIM/NVIC shim) ---
"%GCC%" -std=c99 -Wall -Wextra -O1 -I"%ROOT%\test\timsim" -I"%ROOT%\U1\Core\Inc" -o "%ROOT%\test\pulse_cfg_test.exe" "%ROOT%\test\pulse_cfg_test.c" "%SRC%\pulse_cfg.c" "%SRC%\pulse_cap.c"
if errorlevel 1 goto :pulsehw_build_fail
"%ROOT%\test\pulse_cfg_test.exe"
if errorlevel 1 goto :pulsehw_fail
echo [PASS] pulse_cfg_test
goto :dutproto
:pulsehw_build_fail
echo [FAIL] pulse_cfg_test compile error
set FAILED=1
goto :dutproto
:pulsehw_fail
echo [FAIL] pulse_cfg_test
set FAILED=1
goto :dutproto
:rep_build_fail
echo [FAIL] pulse_test compile error
set FAILED=1
goto :py
:rep_fail
echo [FAIL] pulse_test
set FAILED=1

:dutproto
echo.
echo ============================================================
echo  6/12  DUT protocol pack/unpack (docs/15 layout)
echo ============================================================
"%GCC%" -std=c99 -Wall -Wextra -O1 -I"%ROOT%\U1\Core\Inc" -o "%ROOT%\test\dut_proto_test.exe" "%ROOT%\test\dut_proto_test.c" "%SRC%\dut_proto.c"
if errorlevel 1 goto :dutproto_build_fail
"%ROOT%\test\dut_proto_test.exe"
if errorlevel 1 goto :dutproto_fail
echo [PASS] dut_proto_test
goto :phasechk

rem  ---- 6b) three-phase output judgement (ADC based, docs/12 4) ----
:phasechk
echo.
echo ============================================================
echo  7/12 Three-phase output check (phase voltage vs duty x VBUS)
echo ============================================================
"%GCC%" -std=c99 -Wall -Wextra -O1 -I"%ROOT%\U1\Core\Inc" -o "%ROOT%\test\phase_chk_test.exe" "%ROOT%\test\phase_chk_test.c" "%SRC%\phase_chk.c"
if errorlevel 1 goto :phasechk_build_fail
"%ROOT%\test\phase_chk_test.exe"
if errorlevel 1 goto :phasechk_fail
echo [PASS] phase_chk_test
goto :encplan

rem  ---- 7b) encoder pulse planner (docs/12 5) ----
:encplan
echo.
echo ============================================================
echo  8/12 Encoder pulse planner (counts, PSC/CCR, duration)
echo ============================================================
"%GCC%" -std=c99 -Wall -Wextra -O1 -I"%ROOT%\U1\Core\Inc" -o "%ROOT%\test\enc_plan_test.exe" "%ROOT%\test\enc_plan_test.c" "%SRC%\enc_plan.c"
if errorlevel 1 goto :encplan_build_fail
"%ROOT%\test\enc_plan_test.exe"
if errorlevel 1 goto :encplan_fail
echo [PASS] enc_plan_test
goto :curchk

rem  ---- 8b) phase-current judgement (docs/12 6) ----
:curchk
echo.
echo ============================================================
echo  9/12 Judgement: current + voltage + encoder three-way (jig vs U2 vs DUT)
echo ============================================================
"%GCC%" -std=c99 -Wall -Wextra -O1 -I"%ROOT%\U1\Core\Inc" -o "%ROOT%\test\cur_chk_test.exe" "%ROOT%\test\cur_chk_test.c" "%SRC%\cur_chk.c"
if errorlevel 1 goto :curchk_build_fail
"%ROOT%\test\cur_chk_test.exe"
if errorlevel 1 goto :curchk_fail
"%GCC%" -std=c99 -Wall -Wextra -O1 -I"%ROOT%\U1\Core\Inc" -o "%ROOT%\test\vol_chk_test.exe" "%ROOT%\test\vol_chk_test.c" "%SRC%\vol_chk.c"
if errorlevel 1 goto :curchk_build_fail
"%ROOT%\test\vol_chk_test.exe"
if errorlevel 1 goto :curchk_fail
rem  enc_chk: encoder three-way judgement (jig intent vs U2 vs DUT)
"%GCC%" -std=c99 -Wall -Wextra -O1 -I"%ROOT%\U1\Core\Inc" -o "%ROOT%\test\enc_chk_test.exe" "%ROOT%\test\enc_chk_test.c" "%SRC%\enc_chk.c"
if errorlevel 1 goto :curchk_build_fail
"%ROOT%\test\enc_chk_test.exe"
if errorlevel 1 goto :curchk_fail
echo [PASS] judgement tests (cur_chk + vol_chk + enc_chk)
goto :py
:curchk_build_fail
echo [FAIL] cur_chk_test compile error
set FAILED=1
goto :py
:curchk_fail
echo [FAIL] cur_chk_test
set FAILED=1
goto :py
:encplan_build_fail
echo [FAIL] enc_plan_test compile error
set FAILED=1
goto :py
:encplan_fail
echo [FAIL] enc_plan_test
set FAILED=1
goto :py
:phasechk_build_fail
echo [FAIL] phase_chk_test compile error
set FAILED=1
goto :py
:phasechk_fail
echo [FAIL] phase_chk_test
set FAILED=1
goto :py
:dutproto_build_fail
echo [FAIL] dut_proto_test compile error
set FAILED=1
goto :py
:dutproto_fail
echo [FAIL] dut_proto_test
set FAILED=1

:py
echo.
echo ============================================================
echo  10/12  Cross-language byte check (Python vs C firmware)
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
echo  11/12  Duplicated copies in sync (U1 vs U2)
echo ============================================================
rem  the shared modules exist once per board (uart_port/uart_link/measure/meas_pack/pulse_*) (in each project folder);
rem  the two copies must stay byte-identical, or the link behaviour drifts silently.
rem  If the copies ever must differ on purpose, delete this stage (or make it a WARN).
set DUPFAIL=0
call :dupcmp uart_port.c
call :dupcmp uart_port.h
call :dupcmp uart_link.c
call :dupcmp uart_link.h
call :dupcmp protocol_frame.c
call :dupcmp protocol_frame.h
call :dupcmp measure.c
call :dupcmp measure.h
call :dupcmp meas_pack.c
call :dupcmp meas_pack.h
call :dupcmp pulse_cap.c
call :dupcmp pulse_cap.h
call :dupcmp pulse_pack.c
call :dupcmp pulse_pack.h
call :dupcmp pulse_cfg.c
call :dupcmp pulse_cfg.h
call :dupcmp jig_mode.h
if "%DUPFAIL%"=="1" goto :dup_fail
echo [PASS] 17 duplicated files are identical
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
echo  12/12  Keil MDK build (U1 + U2)
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
rem  Delete the target image too. Once bitten: under a too-narrow sandbox UV4 reports
rem  "0 Error(s), 0 Warning(s)" while producing NOTHING (.o/.axf/.map all stale, no
rem  "Program Size" line at all) -- that fake success used to turn the regression green.
rem  Removing the .axf first makes "it exists afterwards" real evidence that THIS run
rem  actually linked, so the check below cannot be fooled by a stale artifact.
set "AXF=%ROOT%\%1\MDK-ARM\%1\%1.axf"
if exist "%AXF%" del "%AXF%"
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
rem  no .axf back = the toolchain never really ran (see the note above the build)
if not exist "%AXF%" goto :keil_noout
findstr /C:"0 Warning(s)" "%3" >nul
if errorlevel 1 goto :keil_warn
echo [PASS] Keil build %1: 0 Error, 0 Warning
exit /b 0
:keil_noout
echo [FAIL] %1: UV4 reported 0 Error but no .axf came back (armlink never really ran)
set FAILED=1
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
