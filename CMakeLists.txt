cmake_minimum_required(VERSION 3.20)

project(OpenBus VERSION 0.1.0 LANGUAGES CXX)

enable_testing()

set(OPENBUS_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD ${OPENBUS_CXX_STANDARD})
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POLICY_VERSION_MINIMUM 3.5)

if(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    message(FATAL_ERROR
        "OpenBus requires an exact C++23 compiler mode. Native MSVC has no "
        "/std:c++23 switch and CMake maps C++23 to /std:c++latest. Configure "
        "with Clang-cl 17+ or another compiler that exposes an exact C++23 mode.")
endif()

if(NOT DEFINED CMAKE_CXX23_STANDARD_COMPILE_OPTION OR
   CMAKE_CXX23_STANDARD_COMPILE_OPTION STREQUAL "" OR
   CMAKE_CXX23_STANDARD_COMPILE_OPTION MATCHES "latest|2b")
    message(FATAL_ERROR
        "OpenBus requires CMake to provide an exact C++23 compiler option; "
        "the selected compiler does not meet that requirement.")
endif()

option(OPENBUS_ENABLE_CLANG_TIDY
       "Run clang-tidy during compilation"
       OFF)

option(OPENBUS_ENABLE_IPO
    "Enable interprocedural/link-time optimization for Release builds"
    ON)

option(OPENBUS_ENABLE_PERF_TRACE
       "Compile performance tracing instrumentation"
       OFF)

option(OPENBUS_BUILD_BGFX_LIGHTING_PROBE
    "Build the opt-in bgfx OpenGL/GLSL lighting preview"
    OFF)

if(OPENBUS_ENABLE_PERF_TRACE)
    set(OPENBUS_PERF_TRACE_DEFINE 1)
else()
    set(OPENBUS_PERF_TRACE_DEFINE 0)
endif()

if(OPENBUS_ENABLE_CLANG_TIDY)
    find_program(OPENBUS_CLANG_TIDY_EXECUTABLE NAMES clang-tidy REQUIRED)
    set(CMAKE_CXX_CLANG_TIDY "${OPENBUS_CLANG_TIDY_EXECUTABLE}")
endif()

include(FetchContent)

find_package(ODE CONFIG QUIET)
find_package(glfw3 CONFIG QUIET)
find_package(OpenAL CONFIG REQUIRED)

if(OPENBUS_BUILD_BGFX_LIGHTING_PROBE)
    if(NOT WIN32)
        message(FATAL_ERROR "The bgfx lighting probe currently uses GLFW's Win32 native handle.")
    endif()
    find_package(bgfx CONFIG REQUIRED)
    if(NOT TARGET bgfx::shaderc)
        message(FATAL_ERROR "bgfx was found without its tools feature; bgfx::shaderc is required.")
    endif()
endif()

if(TARGET glfw3::glfw AND NOT TARGET glfw)
    add_library(OpenBus::GLFW ALIAS glfw3::glfw)
elseif(TARGET glfw AND NOT TARGET OpenBus::GLFW)
    add_library(OpenBus::GLFW ALIAS glfw)
endif()

if(NOT TARGET OpenBus::GLFW)
    message(STATUS
        "glfw3 was not found via CMake package config; fetching GLFW from GitHub.")
    FetchContent_Declare(glfw
        GIT_REPOSITORY https://github.com/glfw/glfw.git
        GIT_TAG 3.4
    )
    set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(glfw)
    add_library(OpenBus::GLFW ALIAS glfw)
endif()

FetchContent_Declare(gli
    GIT_REPOSITORY https://github.com/g-truc/gli.git
    GIT_TAG 0.8.2.0
)
FetchContent_GetProperties(gli)
if(NOT gli_POPULATED)
    cmake_policy(PUSH)
    if(POLICY CMP0169)
        cmake_policy(SET CMP0169 OLD)
    endif()
    FetchContent_Populate(gli)
    cmake_policy(POP)
endif()

FetchContent_Declare(stb
    GIT_REPOSITORY https://github.com/nothings/stb.git
    GIT_TAG master
)
FetchContent_GetProperties(stb)
if(NOT stb_POPULATED)
    cmake_policy(PUSH)
    if(POLICY CMP0169)
        cmake_policy(SET CMP0169 OLD)
    endif()
    FetchContent_Populate(stb)
    cmake_policy(POP)
endif()

if(NOT TARGET glfw)
    message(FATAL_ERROR
        "GLFW was not found and could not be fetched automatically. "
        "Install glfw3 with a package manager such as vcpkg, or set glfw3_DIR "
        "or CMAKE_PREFIX_PATH to a GLFW installation.")
endif()

find_package(OpenGL REQUIRED)
find_package(ZLIB REQUIRED)

if(TARGET ODE::ODE)
    set(OPENBUS_ODE_TARGET ODE::ODE)
else()
    find_path(ODE_INCLUDE_DIR NAMES ode/ode.h)
    find_library(ODE_LIBRARY NAMES ode ode_double ode_single)

    if(NOT ODE_INCLUDE_DIR OR NOT ODE_LIBRARY)
        message(FATAL_ERROR
            "ODE was not found. Install the ODE development package or set "
            "CMAKE_PREFIX_PATH to an ODE installation.")
    endif()

    add_library(OpenBus::ODE UNKNOWN IMPORTED)
    set_target_properties(OpenBus::ODE PROPERTIES
        IMPORTED_LOCATION "${ODE_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${ODE_INCLUDE_DIR}")
    set(OPENBUS_ODE_TARGET OpenBus::ODE)
endif()

find_package(Threads REQUIRED)

# Keep every executable beside the copied runtime DLLs so the game, probes and
# helper scripts can all be run straight from the build directory.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

# Module includes are rooted here, e.g. #include "core/Logger.h".
set(OPENBUS_SOURCE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/src")

# Header-only third-party code used by the asset and render modules.
add_library(openbus_third_party_headers INTERFACE)
target_include_directories(openbus_third_party_headers SYSTEM INTERFACE
    ${gli_SOURCE_DIR}
    ${gli_SOURCE_DIR}/external/glm
    ${stb_SOURCE_DIR})

# Vendored Lua 5.1 (LuaJIT-compatible) headers and Windows import library.
set(OPENBUS_LUA_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/third_party/lua51")
if(WIN32)
    add_library(OpenBusLua SHARED IMPORTED)
    set_target_properties(OpenBusLua PROPERTIES
        IMPORTED_IMPLIB "${OPENBUS_LUA_ROOT}/lib/lua51.lib"
        IMPORTED_LOCATION "${OPENBUS_LUA_ROOT}/lib/lua51.dll"
        INTERFACE_INCLUDE_DIRECTORIES "${OPENBUS_LUA_ROOT}/include")
else()
    find_library(OPENBUS_LUA_LIBRARY NAMES lua5.1 lua51 REQUIRED)
    add_library(OpenBusLua INTERFACE)
    target_link_libraries(OpenBusLua INTERFACE ${OPENBUS_LUA_LIBRARY})
    target_include_directories(OpenBusLua SYSTEM INTERFACE "${OPENBUS_LUA_ROOT}/include")
endif()

function(openbus_copy_lua_runtime target)
    if(WIN32)
        add_custom_command(TARGET ${target} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${OPENBUS_LUA_ROOT}/lib/lua51.dll"
                "$<TARGET_FILE_DIR:${target}>/lua51.dll")
    endif()
endfunction()

if(OPENBUS_ENABLE_IPO)
    include(CheckIPOSupported)
    check_ipo_supported(RESULT OPENBUS_IPO_SUPPORTED OUTPUT OPENBUS_IPO_ERROR LANGUAGES CXX)
    if(NOT OPENBUS_IPO_SUPPORTED)
        message(WARNING "IPO/LTO is unavailable; continuing without it: ${OPENBUS_IPO_ERROR}")
    endif()
endif()

# Warning level, platform defines and optimisation settings shared by all
# first-party targets (module libraries and the game executable).
function(openbus_apply_project_settings target)
    if(MSVC)
        target_compile_options(${target} PRIVATE /W4 /permissive-
            $<$<CONFIG:Release>:/Gy>
            $<$<CONFIG:Release>:/Gw>
            $<$<CONFIG:Release>:/Zi>
            $<$<CONFIG:RelWithDebInfo>:/Gy>
            $<$<CONFIG:RelWithDebInfo>:/Gw>)
        target_compile_definitions(${target} PRIVATE _CRT_SECURE_NO_WARNINGS GLFW_INCLUDE_NONE)
    else()
        target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
    endif()
    if(OPENBUS_IPO_SUPPORTED)
        set_property(TARGET ${target} PROPERTY INTERPROCEDURAL_OPTIMIZATION_RELEASE TRUE)
        set_property(TARGET ${target} PROPERTY INTERPROCEDURAL_OPTIMIZATION_RELWITHDEBINFO TRUE)
    endif()
endfunction()

add_subdirectory(src)
add_subdirectory(tests)
add_subdirectory(tools)