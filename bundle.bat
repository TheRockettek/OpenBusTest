@echo off
setlocal
pushd "%~dp0"
if errorlevel 1 exit /b 1

if "%~1"=="" (
    echo Usage: bundle.bat ^<model-root^> [output.obx]
    set "RESULT=2"
    goto :cleanup
)

call "%~dp0windows_clang_env.bat"
if errorlevel 1 goto :failed
cmake --preset windows-clang-ninja
if errorlevel 1 goto :failed
cmake --build --preset windows-clang-ninja --target obj_bundle
if errorlevel 1 goto :failed
"%~dp0build-ninja\obj_bundle.exe" %*
set "RESULT=%ERRORLEVEL%"
goto :cleanup

:failed
set "RESULT=%ERRORLEVEL%"
if "%RESULT%"=="0" set "RESULT=1"

:cleanup
popd
exit /b %RESULT%