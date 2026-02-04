@echo off
REM ============================================================================
REM Run Feature Tests for Robot Controller
REM ============================================================================

setlocal enabledelayedexpansion

set PROJECT_ROOT=%~dp0..
set BUILD_DIR=%PROJECT_ROOT%\src\core\build
set SCRIPTS_DIR=%PROJECT_ROOT%\scripts

echo ============================================================
echo    Robot Controller - Feature Test Runner
echo ============================================================
echo.

REM Check for arguments
set BUILD_FIRST=0
set FILTER=
set VERBOSE=0

:parse_args
if "%1"=="" goto :done_args
if /i "%1"=="--build" set BUILD_FIRST=1
if /i "%1"=="-b" set BUILD_FIRST=1
if /i "%1"=="--filter" (
    set FILTER=%2
    shift
)
if /i "%1"=="-f" (
    set FILTER=%2
    shift
)
if /i "%1"=="--verbose" set VERBOSE=1
if /i "%1"=="-v" set VERBOSE=1
shift
goto :parse_args
:done_args

REM Build if requested
if %BUILD_FIRST%==1 (
    echo Building tests...
    echo.

    if not exist "%BUILD_DIR%" mkdir "%BUILD_DIR%"

    pushd "%BUILD_DIR%"
    cmake .. -DCMAKE_BUILD_TYPE=Release
    if errorlevel 1 (
        echo CMake configuration failed!
        popd
        exit /b 1
    )

    cmake --build . --config Release
    if errorlevel 1 (
        echo Build failed!
        popd
        exit /b 1
    )
    popd

    echo.
    echo Build successful!
    echo.
)

REM Run Python script
echo Running feature tests...
echo.

python "%SCRIPTS_DIR%\run_feature_tests.py" --output FEATURE_TEST_REPORT.md

if errorlevel 1 (
    echo.
    echo Some tests failed. Check docs\FEATURE_TEST_REPORT.md for details.
    exit /b 1
) else (
    echo.
    echo All tests passed! Report saved to docs\FEATURE_TEST_REPORT.md
    exit /b 0
)
