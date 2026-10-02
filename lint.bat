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

call "%~dp0windows_clang_env.bat"
if errorlevel 1 exit /b %errorlevel%

cmake -S . -B build-lint -G "NMake Makefiles" -DCMAKE_CXX_COMPILER="%OPENBUS_CLANG_COMPILER%" -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=C:/vcpkg/scripts/buildsystems/vcpkg.cmake -DODE_DIR=C:/vcpkg/installed/x64-windows/share/ode -DOPENBUS_ENABLE_CLANG_TIDY=ON -DOPENBUS_ENABLE_PERF_TRACE=OFF %*
if errorlevel 1 exit /b %errorlevel%

cmake --build build-lint
exit /b %errorlevel%