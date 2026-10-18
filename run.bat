taskkill.exe /F /IM OpenBus.exe

call "%~dp0windows_clang_env.bat"
if errorlevel 1 exit /b %errorlevel%

set "OPENBUS_BUS_CONFIG=Vehicles/MAN_DL05/MAN_DL05.bus"
set "OPENBUS_MODEL_CONFIG=Vehicles/MAN_DL05/Model/DL05.cfg"

set "OPENBUS_MAP_PATH=Cotterell"
set "OPENBUS_MAP_SPAWN=0"

set "OPENBUS_VSYNC=off"

set "OPENBUS_REFLECTION_TRANSPARENT=0"
set "OPENBUS_MATERIAL_BATCHING=1"

set "OPENBUS_REFLECTION_MAX_FPS=60"

set "OPENBUS_REFLECTION_TRANSPARENT=0"
set "OPENBUS_MATERIAL_BATCHING=1"
set "OPENBUS_FRUSTUM_CULLING=1"
set "OPENBUS_WHEELS_FROM_ODE=0"

set "OPENBUS_SCRIPT_BACKEND=native"
set "OPENBUS_SCRIPT_HZ=60"

set "OPENBUS_ASSET_WORKERS=4"
set "OPENBUS_DOPPLER=1"

set "OPENBUS_CAPTURE_VIEWS="
set "OPENBUS_CULLING_BACKEND=cpu"
set "OPENBUS_STEERING_SMOOTHING=1"

cmake --preset windows-clang-ninja && cmake --build --preset windows-clang-ninja --target OpenBus && .\build-ninja\OpenBus.exe