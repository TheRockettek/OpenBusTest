@echo off
setlocal

taskkill.exe /F /IM OpenBus.exe

set "OPENBUS_VEHICLE=DL05"

set "OPENBUS_VSYNC=off"

set "OPENBUS_REFLECTION_TRANSPARENT=0"
set "OPENBUS_MATERIAL_BATCHING=1"

set "OPENBUS_SCRIPT_BACKEND=native"
set "OPENBUS_SCRIPT_HZ=0"

set "OPENBUS_TRACE=1"
set "OPENBUS_TRACE_FILE=openbus_trace.json"
set "OPENBUS_TRACE_MAX_EVENTS=500000"
set "OPENBUS_TRACE_MIN_US=1"


cmake -S . -B build-ode -DCMAKE_TOOLCHAIN_FILE=C:/vcpkg/scripts/buildsystems/vcpkg.cmake && cmake --build build-ode --config Release && .\build-ode\Release\OpenBus.exe
