@echo off
setlocal

taskkill.exe /F /IM OpenBus.exe

@REM set "OPENBUS_VEHICLE=e400"
set "OPENBUS_VSYNC=off"
set "OPENBUS_TRACE=1"
set "OPENBUS_TRACE_COLLAPSED=1"
set "OPENBUS_TRACE_FILE=openbus_trace.json"
set "OPENBUS_TRACE_COLLAPSED_FILE=openbus_trace.collapsed"

cmake -S . -B build-ode -DCMAKE_TOOLCHAIN_FILE=C:/vcpkg/scripts/buildsystems/vcpkg.cmake && cmake --build build-ode --config Release && .\build-ode\Release\OpenBus.exe
