cmake_minimum_required(VERSION 3.20)

project(OpenBus VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POLICY_VERSION_MINIMUM 3.5)

option(OPENBUS_ENABLE_CLANG_TIDY
       "Run clang-tidy during compilation"
       OFF)

if(OPENBUS_ENABLE_CLANG_TIDY)
    find_program(OPENBUS_CLANG_TIDY_EXECUTABLE NAMES clang-tidy REQUIRED)
    set(CMAKE_CXX_CLANG_TIDY "${OPENBUS_CLANG_TIDY_EXECUTABLE}")
endif()

include(FetchContent)

find_package(ODE CONFIG QUIET)
find_package(glfw3 CONFIG QUIET)

if(NOT TARGET glfw)
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

if(TARGET ODE::ODE)
    set(OPENBUS_ODE_TARGET ODE::ODE)
else()
    find_path(ODE_INCLUDE_DIR NAMES ode/ode.h)
    find_library(ODE_LIBRARY NAMES ode ode_double ode_single)

    if(NOT ODE_INCLUDE_DIR OR NOT ODE_LIBRARY)
        message(FATAL_ERROR
            "ODE was not found. Install it with vcpkg (vcpkg install ode:x64-windows) "
            "or set CMAKE_PREFIX_PATH to an ODE installation.")
    endif()

    add_library(OpenBus::ODE UNKNOWN IMPORTED)
    set_target_properties(OpenBus::ODE PROPERTIES
        IMPORTED_LOCATION "${ODE_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${ODE_INCLUDE_DIR}")
    set(OPENBUS_ODE_TARGET OpenBus::ODE)
endif()

add_executable(OpenBus
    src/main.cpp
    src/BusConfiguration.cpp
    src/CrashHandler.cpp
    src/RoadFeatures.cpp
    src/BusSimulation.cpp
    src/Logger.cpp
    src/Variables.cpp
    src/ConfigurationParser.cpp
    src/VehicleConfigLoader.cpp
    src/ModelConfigLoader.cpp
    src/ModelConfigJson.cpp
    src/BusConfigLoader.cpp
    src/BusModelLoader.cpp
    src/CameraMath.cpp
    src/RenderPrimitives.cpp
    src/SceneryObjectConfigLoader.cpp
    src/PerfTrace.cpp
    src/Renderer.cpp
    src/ScreenshotWriter.cpp
    src/TextureLoader.cpp
    src/ObjLoader.cpp
)

target_include_directories(OpenBus PRIVATE src)
target_include_directories(OpenBus SYSTEM PRIVATE
    ${gli_SOURCE_DIR}
    ${gli_SOURCE_DIR}/external/glm
    ${stb_SOURCE_DIR})
target_link_libraries(OpenBus PRIVATE ${OPENBUS_ODE_TARGET} glfw OpenGL::GL)

add_executable(uno3d_converter
    uno3d/o3d_to_obj.cpp
)

target_include_directories(uno3d_converter SYSTEM PRIVATE ${stb_SOURCE_DIR})

if(MSVC)
    target_compile_options(OpenBus PRIVATE /W4 /permissive-)
    target_compile_definitions(OpenBus PRIVATE _CRT_SECURE_NO_WARNINGS GLFW_INCLUDE_NONE)
    target_link_libraries(OpenBus PRIVATE windowscodecs dbghelp)
else()
    target_compile_options(OpenBus PRIVATE -Wall -Wextra -Wpedantic)
endif()
