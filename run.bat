taskkill.exe /F /IM OpenBus.exe

set "OPENBUS_BUS_CONFIG=Vehicles/MAN_DL05/MAN_DL05.bus"
set "OPENBUS_MODEL_CONFIG=Vehicles/MAN_DL05/Model/DL05.cfg"

set "OPENBUS_MAP_PATH=Grundorf"

@REM set "OPENBUS_BUS_CONFIG=Vehicles/Caetano Levante/Caetano.bus"
@REM set "OPENBUS_MODEL_CONFIG=Vehicles/Caetano Levante/Model/model_caetano.cfg"

call "%~dp0windows_clang_env.bat"
if errorlevel 1 exit /b %errorlevel%

@REM set "OPENBUS_BUS_CONFIG=Vehicles\[SP] Studio Polygon 400MMC\E400MMC_ADL_10.9m_Voith_LowHeight.bus"
@REM set "OPENBUS_MODEL_CONFIG=Vehicles\[SP] Studio Polygon 400MMC\Model\Configuration Files\E400MMC_ADL_10.9m_Voith_LowHeight.cfg"

@REM set "OPENBUS_BUS_CONFIG=Vehicles\V3D - Enviro 200 MMC\e200mmc_115.bus"
@REM set "OPENBUS_MODEL_CONFIG=Vehicles\V3D - Enviro 200 MMC\Model\e200mmc_115.cfg"


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

cmake -S . -B build-ode -G "NMake Makefiles" -DCMAKE_CXX_COMPILER="%OPENBUS_CLANG_COMPILER%" -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=C:/vcpkg/scripts/buildsystems/vcpkg.cmake -DOPENBUS_ENABLE_PERF_TRACE=OFF && cmake --build build-ode --target OpenBus && .\build-ode\OpenBus.exe