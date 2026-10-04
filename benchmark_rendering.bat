@echo off
setlocal

call "%~dp0windows_clang_env.bat"
if errorlevel 1 exit /b %errorlevel%

pushd "%~dp0" || exit /b 1
cmake --preset windows-clang-benchmark
if errorlevel 1 goto :failed

cmake --build --preset windows-clang-benchmark --target OpenBus
if errorlevel 1 goto :failed

if defined OPENBUS_PYTHON (
	"%OPENBUS_PYTHON%" "benchmark_rendering.py" %*
) else (
	python "benchmark_rendering.py" %*
)
set "EXIT_CODE=%ERRORLEVEL%"
popd
exit /b %EXIT_CODE%

:failed
set "EXIT_CODE=%ERRORLEVEL%"
popd
exit /b %EXIT_CODE%
