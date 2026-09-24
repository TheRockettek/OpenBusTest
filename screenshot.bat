taskkill.exe /F /IM OpenBus.exe

set "OPENBUS_VEHICLE=e400"
set "OPENBUS_VSYNC=off"
set "OPENBUS_CAPTURE_VIEWS=1"
cmake -S . -B build-ode -DCMAKE_TOOLCHAIN_FILE=C:/vcpkg/scripts/buildsystems/vcpkg.cmake && cmake --build build-ode --config Release && .\build-ode\Release\OpenBus.exe