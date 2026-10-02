@echo off

set "OPENBUS_CLANG_BIN=C:\Program Files\LLVM\bin"
set "OPENBUS_CLANG_COMPILER=%OPENBUS_CLANG_BIN%\clang-cl.exe"
if not exist "%OPENBUS_CLANG_COMPILER%" (
    echo clang-cl was not found at "%OPENBUS_CLANG_COMPILER%".
    echo Install LLVM or set OPENBUS_CLANG_BIN before running this script.
    exit /b 1
)

set "OPENBUS_MSVC_ROOT=C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Tools\MSVC\14.44.35207"
if not exist "%OPENBUS_MSVC_ROOT%\include\vcruntime.h" (
    echo Visual Studio MSVC headers were not found at "%OPENBUS_MSVC_ROOT%".
    echo Update OPENBUS_MSVC_ROOT in this script for the installed MSVC version.
    exit /b 1
)

set "OPENBUS_WINDOWS_KIT=C:\Program Files (x86)\Windows Kits\10"
set "OPENBUS_WINDOWS_KIT_VERSION=10.0.26100.0"
if not exist "%OPENBUS_WINDOWS_KIT%\Include\%OPENBUS_WINDOWS_KIT_VERSION%\ucrt\stdlib.h" (
    echo Windows SDK %OPENBUS_WINDOWS_KIT_VERSION% was not found.
    echo Update OPENBUS_WINDOWS_KIT_VERSION in this script for the installed SDK.
    exit /b 1
)

set "PATH=%OPENBUS_CLANG_BIN%;%OPENBUS_MSVC_ROOT%\bin\Hostx64\x64;%PATH%"
set "INCLUDE=%OPENBUS_MSVC_ROOT%\include;%OPENBUS_WINDOWS_KIT%\Include\%OPENBUS_WINDOWS_KIT_VERSION%\ucrt;%OPENBUS_WINDOWS_KIT%\Include\%OPENBUS_WINDOWS_KIT_VERSION%\shared;%OPENBUS_WINDOWS_KIT%\Include\%OPENBUS_WINDOWS_KIT_VERSION%\um;%INCLUDE%"
set "LIB=%OPENBUS_MSVC_ROOT%\lib\x64;%OPENBUS_WINDOWS_KIT%\Lib\%OPENBUS_WINDOWS_KIT_VERSION%\ucrt\x64;%OPENBUS_WINDOWS_KIT%\Lib\%OPENBUS_WINDOWS_KIT_VERSION%\um\x64;%LIB%"
exit /b 0
