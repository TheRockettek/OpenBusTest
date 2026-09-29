taskkill.exe /F /IM OpenBus.exe

del .\SP_E400MMC\Script\*.lua
del .\MAN_DL05\Script\*.lua

set "OPENBUS_VEHICLE=e400"
set "OPENBUS_BUS_CONFIG="
set "OPENBUS_VSYNC=off"

cmake -S . -B build-ode -DCMAKE_TOOLCHAIN_FILE=C:/vcpkg/scripts/buildsystems/vcpkg.cmake && cmake --build build-ode --config Release && .\build-ode\Release\OpenBus.exe