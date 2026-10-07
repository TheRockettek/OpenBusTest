@echo off
setlocal

set "AUDIT_ROOT=%~1"
if not defined AUDIT_ROOT set "AUDIT_ROOT=%OMSI_ROOT%"
if not defined AUDIT_ROOT set /p "AUDIT_ROOT=Enter the OMSI root or directory to scan: "
if not defined AUDIT_ROOT (
    echo No directory was provided.
    exit /b 2
)

set "BUILD_DIR=%OPENBUS_BUILD_DIR%"
if not defined BUILD_DIR set "BUILD_DIR=build-ode"

pushd "%~dp0" || exit /b 1
if not exist "%BUILD_DIR%\CMakeCache.txt" (
    echo CMake build directory not found: "%BUILD_DIR%"
    echo Configure the project first, or set OPENBUS_BUILD_DIR to an existing build directory.
    popd
    exit /b 2
)

call "%~dp0windows_clang_env.bat"
if errorlevel 1 goto :failed

cmake --build "%BUILD_DIR%" --target OpenBusConfigAudit
if errorlevel 1 goto :failed

"%BUILD_DIR%\OpenBusConfigAudit.exe" "%AUDIT_ROOT%"
set "EXIT_CODE=%ERRORLEVEL%"
popd
exit /b %EXIT_CODE%

:failed
set "EXIT_CODE=%ERRORLEVEL%"
popd
exit /b %EXIT_CODE%
