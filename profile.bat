@echo off
setlocal
pushd "%~dp0"
if errorlevel 1 exit /b 1

tasklist /FI "IMAGENAME eq OpenBus.exe" /NH | findstr /I /C:"OpenBus.exe" >nul
if not errorlevel 1 goto :already_running

call "%~dp0windows_clang_env.bat"
if errorlevel 1 goto :failed

set "OPENBUS_BUS_CONFIG=Vehicles/MAN_DL05/MAN_DL05.bus"
set "OPENBUS_MODEL_CONFIG=Vehicles/MAN_DL05/Model/DL05.cfg"
set "OPENBUS_MAP_PATH=DBC_Map"
set "OPENBUS_MAP_SPAWN=1"
set "OPENBUS_VSYNC=off"
set "OPENBUS_MAP_AI_COUNT=1000"
set "OPENBUS_MAP_BUS_COUNT=20"
set "OPENBUS_MAP_HUMAN_COUNT=100"
set "OPENBUS_MAP_TRAFFIC_DENSITY_MULTIPLIER=100"
set "OPENBUS_MATERIAL_BATCHING=1"
set "OPENBUS_REFLECTION_SIZE=1024"
set "OPENBUS_REFLECTION_INTERVAL=4"
set "OPENBUS_REFLECTION_TRANSPARENT=1"
set "OPENBUS_FRUSTUM_CULLING=1"
set "OPENBUS_WHEELS_FROM_ODE=0"
set "OPENBUS_SCRIPT_BACKEND=native"
set "OPENBUS_SCRIPT_HZ=60"
set "OPENBUS_ASSET_WORKERS=4"
set "OPENBUS_DOPPLER=1"
set "OPENBUS_TRACE=0"
set "OPENBUS_TRACE_FILE=openbus_trace.json"
set "OPENBUS_TRACE_MAX_EVENTS=10000000"
set "OPENBUS_TRACE_MIN_US=1"
set "OPENBUS_CULLING_BACKEND=cpu"

cmake -S "%~dp0" -B "%~dp0build-ode" -G "NMake Makefiles" -DCMAKE_CXX_COMPILER="%OPENBUS_CLANG_COMPILER%" -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE="%OPENBUS_VCPKG_ROOT%/scripts/buildsystems/vcpkg.cmake" -DOPENBUS_ENABLE_PERF_TRACE=ON
if errorlevel 1 goto :failed
cmake --build "%~dp0build-ode" --target OpenBus
if errorlevel 1 goto :failed
"%~dp0build-ode\OpenBus.exe"
set "RESULT=%ERRORLEVEL%"
goto :cleanup

:already_running
echo OpenBus.exe is already running. Close it before using this helper.
set "RESULT=1"
goto :cleanup

:failed
set "RESULT=%ERRORLEVEL%"
if "%RESULT%"=="0" set "RESULT=1"

:cleanup
popd
exit /b %RESULT%
