@echo off
setlocal

taskkill.exe /F /IM OpenBus.exe

@REM set "OPENBUS_BUS_CONFIG=Vehicles\[SP] Studio Polygon 400MMC\E400MMC_ADL_10.9m_Voith_LowHeight.bus"
@REM set "OPENBUS_MODEL_CONFIG=Vehicles\[SP] Studio Polygon 400MMC\Model\Configuration Files\E400MMC_ADL_10.9m_Voith_LowHeight.cfg"

@REM set "OPENBUS_AI_BUS_CONFIG=Vehicles/VW_Golf_2/ai_vw_golf_2.bus"
@REM set "OPENBUS_AI_MODEL_CONFIG=Vehicles/VW_Golf_2/model/model.cfg"
@REM set "OPENBUS_AI_COUNT=200"

set "OPENBUS_BUS_CONFIG=Vehicles/Caetano Levante/Caetano.bus"
set "OPENBUS_MODEL_CONFIG=Vehicles/Caetano Levante/Model/model_caetano.cfg"

set "OPENBUS_MAP_PATH=Cotterell"
@REM set "OPENBUS_MAP_PATH=Grundorf"
@REM set "OPENBUS_MAP_PATH=Grande Porto 2022"

set "OPENBUS_VSYNC=off"

set "OPENBUS_REFLECTION_TRANSPARENT=0"
set "OPENBUS_MATERIAL_BATCHING=1"
@REM set "OPENBUS_REFLECTION_MAX_FPS=60"

set "OPENBUS_REFLECTION_TRANSPARENT=0"
set "OPENBUS_MATERIAL_BATCHING=1"
set "OPENBUS_FRUSTUM_CULLING=1"
set "OPENBUS_WHEELS_FROM_ODE=0"

set "OPENBUS_SCRIPT_BACKEND=native"
set "OPENBUS_SCRIPT_HZ=60"

set "OPENBUS_ASSET_WORKERS=4"
set "OPENBUS_DOPPLER=1"

set "OPENBUS_TRACE=1"
set "OPENBUS_TRACE_FILE=openbus_trace.json"
set "OPENBUS_TRACE_MAX_EVENTS=10000000"
set "OPENBUS_TRACE_MIN_US=1"

call "%~dp0windows_clang_env.bat"
if errorlevel 1 exit /b %errorlevel%

cmake -S . -B build-ode -G "NMake Makefiles" -DCMAKE_CXX_COMPILER="%OPENBUS_CLANG_COMPILER%" -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=C:/vcpkg/scripts/buildsystems/vcpkg.cmake -DOPENBUS_ENABLE_PERF_TRACE=ON && cmake --build build-ode --target OpenBus && .\build-ode\OpenBus.exe
