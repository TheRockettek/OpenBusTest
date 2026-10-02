taskkill.exe /F /IM OpenBus.exe

@REM set "OPENBUS_VEHICLE=e400"
set "OPENBUS_VSYNC=off"
set "OPENBUS_CAPTURE_VIEWS=1"
call "%~dp0windows_clang_env.bat"
if errorlevel 1 exit /b %errorlevel%
cmake -S . -B build-ode -G "NMake Makefiles" -DCMAKE_CXX_COMPILER="%OPENBUS_CLANG_COMPILER%" -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=C:/vcpkg/scripts/buildsystems/vcpkg.cmake -DOPENBUS_ENABLE_PERF_TRACE=OFF && cmake --build build-ode --target OpenBus && .\build-ode\OpenBus.exe