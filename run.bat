taskkill.exe /F /IM OpenBus.exe

del .\SP_E400MMC\Script\*.lua
del .\SP_E400MMC\Script\E400MMC\ADL\*.lua
del .\SP_E400MMC\Script\E400MMC\Scania\*.lua

del .\MAN_DL05\Script\*.lua

set "OPENBUS_VEHICLE=DL05"
@REM set "OPENBUS_VEHICLE=e400"
set "OPENBUS_BUS_CONFIG="

set "OPENBUS_SCRIPT_HZ=0"
set "OPENBUS_VSYNC=off"

set "OPENBUS_REFLECTION_TRANSPARENT=0"
set "OPENBUS_MATERIAL_BATCHING=1"
set "OPENBUS_FRUSTUM_CULLING=1"

set "OPENBUS_SCRIPT_BACKEND=native"
set "OPENBUS_SCRIPT_HZ=0"

cmake -S . -B build-ode -DCMAKE_TOOLCHAIN_FILE=C:/vcpkg/scripts/buildsystems/vcpkg.cmake && cmake --build build-ode --config Release && .\build-ode\Release\OpenBus.exe