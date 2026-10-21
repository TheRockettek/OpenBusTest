@echo off
setlocal
pushd "%~dp0"
if errorlevel 1 exit /b 1

tasklist /FI "IMAGENAME eq OpenBus.exe" /NH | findstr /I /C:"OpenBus.exe" >nul
if not errorlevel 1 goto :already_running

set "OPENBUS_VSYNC=off"
set "OPENBUS_CAPTURE_VIEWS=1"
call "%~dp0windows_clang_env.bat"
if errorlevel 1 goto :failed

cmake -S . -B build-ode -G "NMake Makefiles" -DCMAKE_CXX_COMPILER="%OPENBUS_CLANG_COMPILER%" -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=C:/vcpkg/scripts/buildsystems/vcpkg.cmake -DOPENBUS_ENABLE_PERF_TRACE=OFF
if errorlevel 1 goto :failed
cmake --build build-ode --target OpenBus
if errorlevel 1 goto :failed
.\build-ode\OpenBus.exe
set "RESULT=%ERRORLEVEL%"
goto :cleanup

:already_running
echo OpenBus.exe is already running. Close it before using this helper.
set "RESULT=1"
goto :cleanup

:failed
set "RESULT=%ERRORLEVEL%"

:cleanup
popd
exit /b %RESULT%
