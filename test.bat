@echo off
setlocal
pushd "%~dp0"
if errorlevel 1 exit /b 1

call "%~dp0windows_clang_env.bat"
if errorlevel 1 goto :failed

cmake --preset windows-clang-ninja
if errorlevel 1 goto :failed
cmake --build --preset windows-clang-ninja --target check-probes
set "RESULT=%ERRORLEVEL%"
goto :cleanup

:failed
set "RESULT=%ERRORLEVEL%"

:cleanup
popd
exit /b %RESULT%
