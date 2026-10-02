taskkill.exe /F /IM OpenBus.exe

@REM set "OPENBUS_BUS_CONFIG=Vehicles/MAN_DL05/MAN_DL05.bus"
@REM set "OPENBUS_MODEL_CONFIG=Vehicles/MAN_DL05/Model/DL05.cfg"

@REM set "OPENBUS_BUS_CONFIG=Vehicles/Caetano Levante/Caetano.bus"
@REM set "OPENBUS_MODEL_CONFIG=Vehicles/Caetano Levante/Model/model_caetano.cfg"

call "%~dp0windows_clang_env.bat"
if errorlevel 1 exit /b %errorlevel%

set "OPENBUS_BUS_CONFIG=Vehicles\[SP] Studio Polygon 400MMC\E400MMC_ADL_10.9m_Voith_LowHeight.bus"
set "OPENBUS_MODEL_CONFIG=Vehicles\[SP] Studio Polygon 400MMC\Model\Configuration Files\E400MMC_ADL_10.9m_Voith_LowHeight.cfg"

set "OPENBUS_AI_BUS_CONFIG=Vehicles/VW_Golf_2/ai_vw_golf_2.bus"
set "OPENBUS_AI_MODEL_CONFIG=Vehicles/VW_Golf_2/model/model.cfg"

set "OPENBUS_SCRIPT_HZ=0"
set "OPENBUS_VSYNC=off"

set "OPENBUS_REFLECTION_TRANSPARENT=0"
set "OPENBUS_MATERIAL_BATCHING=1"
set "OPENBUS_FRUSTUM_CULLING=1"
set "OPENBUS_WHEELS_FROM_ODE=0"

set "OPENBUS_SCRIPT_BACKEND=native"
set "OPENBUS_SCRIPT_HZ=0"

cmake -S . -B build-ode -G "NMake Makefiles" -DCMAKE_CXX_COMPILER="%OPENBUS_CLANG_COMPILER%" -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=C:/vcpkg/scripts/buildsystems/vcpkg.cmake -DOPENBUS_ENABLE_PERF_TRACE=OFF && cmake --build build-ode --target OpenBus && .\build-ode\OpenBus.exe