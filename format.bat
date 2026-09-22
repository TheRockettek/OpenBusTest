@echo off
setlocal

set "CLANG_FORMAT="
for /f "delims=" %%P in ('where clang-format 2^>nul') do if not defined CLANG_FORMAT set "CLANG_FORMAT=%%P"
if not defined CLANG_FORMAT if exist "C:\Program Files\LLVM\bin\clang-format.exe" set "CLANG_FORMAT=C:\Program Files\LLVM\bin\clang-format.exe"

if not defined CLANG_FORMAT (
    echo clang-format was not found on PATH.
    echo Install LLVM or add its bin directory to PATH.
    exit /b 1
)

set "ROOT=%~dp0"
set "FAILED=0"

for /r "%ROOT%src" %%F in (*.cpp *.h) do (
    echo Formatting %%~fF
    "%CLANG_FORMAT%" -i "%%~fF"
    if errorlevel 1 set "FAILED=1"
)

for /r "%ROOT%uno3d" %%F in (*.cpp *.h) do (
    echo Formatting %%~fF
    "%CLANG_FORMAT%" -i "%%~fF"
    if errorlevel 1 set "FAILED=1"
)

if "%FAILED%"=="1" (
    echo clang-format failed for one or more files.
    exit /b 1
)

echo Formatting completed successfully.
exit /b 0
