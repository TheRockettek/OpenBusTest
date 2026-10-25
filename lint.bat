@echo off
setlocal
pushd "%~dp0"
if errorlevel 1 exit /b 1

call "%~dp0windows_clang_env.bat"
if errorlevel 1 goto :failed

where clang-tidy >nul 2>nul
if errorlevel 1 (
    echo clang-tidy was not found on PATH.
    echo Install LLVM or set OPENBUS_CLANG_BIN to its bin directory.
    goto :failed
)

cmake -S "%~dp0" -B "%~dp0build-lint" -G "NMake Makefiles" -DCMAKE_CXX_COMPILER="%OPENBUS_CLANG_COMPILER%" -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE="%OPENBUS_VCPKG_ROOT%/scripts/buildsystems/vcpkg.cmake" -DOPENBUS_ENABLE_CLANG_TIDY=ON -DOPENBUS_ENABLE_PERF_TRACE=OFF %*
if errorlevel 1 goto :failed

cmake --build "%~dp0build-lint"
set "RESULT=%ERRORLEVEL%"
goto :cleanup

:failed
set "RESULT=%ERRORLEVEL%"
if "%RESULT%"=="0" set "RESULT=1"

:cleanup
popd
exit /b %RESULT%
