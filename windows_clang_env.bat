@echo off

if not defined OPENBUS_CLANG_BIN if defined OPENBUS_CLANG_COMPILER for %%I in ("%OPENBUS_CLANG_COMPILER%") do set "OPENBUS_CLANG_BIN=%%~dpI"
if not defined OPENBUS_CLANG_BIN (
    for /f "delims=" %%I in ('where clang-cl.exe 2^>nul') do if not defined OPENBUS_CLANG_BIN set "OPENBUS_CLANG_BIN=%%~dpI"
)
if not defined OPENBUS_CLANG_BIN set "OPENBUS_CLANG_BIN=C:\Program Files\LLVM\bin"
if "%OPENBUS_CLANG_BIN:~-1%"=="\" set "OPENBUS_CLANG_BIN=%OPENBUS_CLANG_BIN:~0,-1%"
if not defined OPENBUS_CLANG_COMPILER set "OPENBUS_CLANG_COMPILER=%OPENBUS_CLANG_BIN%\clang-cl.exe"
if not exist "%OPENBUS_CLANG_COMPILER%" (
    echo clang-cl was not found at "%OPENBUS_CLANG_COMPILER%".
    echo Set OPENBUS_CLANG_BIN or OPENBUS_CLANG_COMPILER to the installed LLVM path.
    exit /b 1
)

if not defined OPENBUS_VS_ROOT (
    if exist "%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe" (
        for /f "usebackq tokens=*" %%I in (`"%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "OPENBUS_VS_ROOT=%%I"
    )
)
if not defined OPENBUS_MSVC_ROOT (
    for /f "delims=" %%I in ('dir /b /ad /o-n "%OPENBUS_VS_ROOT%\VC\Tools\MSVC" 2^>nul') do if not defined OPENBUS_MSVC_ROOT set "OPENBUS_MSVC_ROOT=%OPENBUS_VS_ROOT%\VC\Tools\MSVC\%%I"
)
if not defined OPENBUS_MSVC_ROOT (
    echo Visual Studio C++ build tools were not found.
    echo Set OPENBUS_MSVC_ROOT to the installed VC\Tools\MSVC\<version> directory.
    exit /b 1
)
if not exist "%OPENBUS_MSVC_ROOT%\include\vcruntime.h" (
    echo Visual Studio MSVC headers were not found at "%OPENBUS_MSVC_ROOT%".
    echo Set OPENBUS_MSVC_ROOT to the installed VC\Tools\MSVC\<version> directory.
    exit /b 1
)

if not defined OPENBUS_WINDOWS_KIT set "OPENBUS_WINDOWS_KIT=%ProgramFiles(x86)%\Windows Kits\10"
if not defined OPENBUS_WINDOWS_KIT_VERSION (
    for /f "delims=" %%I in ('dir /b /ad /o-n "%OPENBUS_WINDOWS_KIT%\Include" 2^>nul') do if not defined OPENBUS_WINDOWS_KIT_VERSION set "OPENBUS_WINDOWS_KIT_VERSION=%%I"
)
if not defined OPENBUS_WINDOWS_KIT_VERSION (
    echo Windows 10/11 SDK was not found under "%OPENBUS_WINDOWS_KIT%\Include".
    echo Set OPENBUS_WINDOWS_KIT and OPENBUS_WINDOWS_KIT_VERSION to the installed SDK.
    exit /b 1
)
if not exist "%OPENBUS_WINDOWS_KIT%\Include\%OPENBUS_WINDOWS_KIT_VERSION%\ucrt\stdlib.h" (
    echo Windows SDK %OPENBUS_WINDOWS_KIT_VERSION% was not found.
    echo Set OPENBUS_WINDOWS_KIT and OPENBUS_WINDOWS_KIT_VERSION to the installed SDK.
    exit /b 1
)

if not defined OPENBUS_NINJA if defined OPENBUS_VS_ROOT set "OPENBUS_NINJA=%OPENBUS_VS_ROOT%\Common7\IDE\CommonExtensions\Microsoft\CMake\Ninja\ninja.exe"
if defined OPENBUS_NINJA if not exist "%OPENBUS_NINJA%" set "OPENBUS_NINJA="
if not defined OPENBUS_NINJA (
    for /f "delims=" %%I in ('where ninja.exe 2^>nul') do if not defined OPENBUS_NINJA set "OPENBUS_NINJA=%%I"
)
if not defined OPENBUS_NINJA (
    echo Ninja was not found.
    echo Set OPENBUS_NINJA to ninja.exe or install Ninja and add it to PATH.
    exit /b 1
)
if not defined OPENBUS_NINJA_BIN for %%I in ("%OPENBUS_NINJA%") do set "OPENBUS_NINJA_BIN=%%~dpI"

if not defined OPENBUS_VCPKG_ROOT set "OPENBUS_VCPKG_ROOT=C:\vcpkg"

set "PATH=%OPENBUS_CLANG_BIN%;%OPENBUS_NINJA_BIN%;%OPENBUS_MSVC_ROOT%\bin\Hostx64\x64;%PATH%"
set "INCLUDE=%OPENBUS_MSVC_ROOT%\include;%OPENBUS_WINDOWS_KIT%\Include\%OPENBUS_WINDOWS_KIT_VERSION%\ucrt;%OPENBUS_WINDOWS_KIT%\Include\%OPENBUS_WINDOWS_KIT_VERSION%\shared;%OPENBUS_WINDOWS_KIT%\Include\%OPENBUS_WINDOWS_KIT_VERSION%\um;%INCLUDE%"
set "LIB=%OPENBUS_MSVC_ROOT%\lib\x64;%OPENBUS_WINDOWS_KIT%\Lib\%OPENBUS_WINDOWS_KIT_VERSION%\ucrt\x64;%OPENBUS_WINDOWS_KIT%\Lib\%OPENBUS_WINDOWS_KIT_VERSION%\um\x64;%LIB%"
exit /b 0
