@echo off
setlocal

where clang-tidy >nul 2>nul
if errorlevel 1 (
    echo clang-tidy was not found on PATH.
    echo Install LLVM from https://releases.llvm.org/ or add its bin directory to PATH.
    exit /b 1
)

if not exist C:\vcpkg\scripts\buildsystems\vcpkg.cmake (
    echo vcpkg was not found at C:\vcpkg.
    echo Install vcpkg or pass a valid CMAKE_TOOLCHAIN_FILE argument.
    exit /b 1
)

cmake -S . -B build-lint -DCMAKE_TOOLCHAIN_FILE=C:/vcpkg/scripts/buildsystems/vcpkg.cmake -DODE_DIR=C:/vcpkg/installed/x64-windows/share/ode -DOPENBUS_ENABLE_CLANG_TIDY=ON -DOPENBUS_ENABLE_PERF_TRACE=OFF %*
if errorlevel 1 exit /b %errorlevel%

cmake --build build-lint --config Release
exit /b %errorlevel%