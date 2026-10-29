@echo off
setlocal
pushd "%~dp0"
if errorlevel 1 exit /b 1

tasklist /FI "IMAGENAME eq OpenBus.exe" /NH | findstr /I /C:"OpenBus.exe" >nul
if not errorlevel 1 goto :already_running

call "%~dp0windows_clang_env.bat"
if errorlevel 1 goto :failed

if /I "%OPENBUS_BENCHMARK%"=="1" (
    if not defined OPENBUS_BUS_CONFIG set "OPENBUS_BUS_CONFIG=Vehicles/MAN_DL05/MAN_DL05.bus"
    if not defined OPENBUS_MODEL_CONFIG set "OPENBUS_MODEL_CONFIG=Vehicles/MAN_DL05/Model/DL05.cfg"
)
set "OPENBUS_MAP_PATH=Cotterell"
set "OPENBUS_VSYNC=off"
set "OPENBUS_MAP_AI_COUNT=500"
set "OPENBUS_MAP_BUS_COUNT=20"
set "OPENBUS_MAP_HUMAN_COUNT=1"
set "OPENBUS_MAP_TRAFFIC_DENSITY_MULTIPLIER=1"
set "OPENBUS_REFLECTION_TRANSPARENT=1"
set "OPENBUS_MATERIAL_BATCHING=1"
set "OPENBUS_REFLECTION_MAX_FPS=60"
set "OPENBUS_FRUSTUM_CULLING=1"
set "OPENBUS_WHEELS_FROM_ODE=0"
set "OPENBUS_SCRIPT_BACKEND=native"
set "OPENBUS_SCRIPT_HZ=60"
set "OPENBUS_ASSET_WORKERS=4"
set "OPENBUS_DOPPLER=1"
set "OPENBUS_CAPTURE_VIEWS="
set "OPENBUS_CULLING_BACKEND=compute-indirect"
set "OPENBUS_STEERING_SMOOTHING=1"

cmake --preset windows-clang-ninja
if errorlevel 1 goto :failed
cmake --build --preset windows-clang-ninja --target OpenBus
if errorlevel 1 goto :failed
.\build-ninja\OpenBus.exe
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
