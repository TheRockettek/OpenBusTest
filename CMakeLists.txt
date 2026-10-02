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

if(OPENBUS_ENABLE_PERF_TRACE)
    set(OPENBUS_PERF_TRACE_DEFINE 1)
else()
    set(OPENBUS_PERF_TRACE_DEFINE 0)
endif()

function(openbus_configure_perf_trace target)
    target_compile_definitions(${target} PRIVATE
        OPENBUS_ENABLE_PERF_TRACE=${OPENBUS_PERF_TRACE_DEFINE})
endfunction()

if(OPENBUS_ENABLE_CLANG_TIDY)
    find_program(OPENBUS_CLANG_TIDY_EXECUTABLE NAMES clang-tidy REQUIRED)
    set(CMAKE_CXX_CLANG_TIDY "${OPENBUS_CLANG_TIDY_EXECUTABLE}")
endif()

include(FetchContent)

find_package(ODE CONFIG QUIET)
find_package(glfw3 CONFIG QUIET)

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

if(UNIX AND NOT APPLE)
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
        pkg_check_modules(PIPEWIRE QUIET IMPORTED_TARGET libpipewire-0.3)
    endif()
endif()

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
    src/CoreRenderer.cpp
    src/RenderPrimitives.cpp
    src/RendererReflection.cpp
    src/SceneryObjectConfigLoader.cpp
    src/PerfTrace.cpp
    src/RenderLoop.cpp
    src/AssetRequestManager.cpp
    src/ScreenshotWriter.cpp
    src/TextureAssetLoader.cpp
    src/TextureLoader.cpp
    src/ObjLoader.cpp
    src/O3DLoader.cpp
    src/OpenGLFunctions.cpp
    src/ScriptRuntime.cpp
    src/SoundEngine.cpp
    src/osc/osc.cpp
)

openbus_configure_perf_trace(OpenBus)
target_include_directories(OpenBus PRIVATE src)
target_include_directories(OpenBus PRIVATE src/osc)
add_library(OpenBusLua SHARED IMPORTED)
set_target_properties(OpenBusLua PROPERTIES
    IMPORTED_IMPLIB "${CMAKE_CURRENT_SOURCE_DIR}/src/osc/lua51.lib"
    IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/src/osc/lua51.dll")
add_custom_command(TARGET OpenBus POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_SOURCE_DIR}/src/osc/lua51.dll"
        "$<TARGET_FILE_DIR:OpenBus>/lua51.dll")
target_include_directories(OpenBus SYSTEM PRIVATE
    ${gli_SOURCE_DIR}
    ${gli_SOURCE_DIR}/external/glm
    ${stb_SOURCE_DIR})
target_link_libraries(OpenBus PRIVATE ${OPENBUS_ODE_TARGET} OpenBus::GLFW OpenGL::GL OpenBusLua)

if(TARGET PkgConfig::PIPEWIRE)
    target_compile_definitions(OpenBus PRIVATE OPENBUS_HAS_PIPEWIRE=1)
    target_link_libraries(OpenBus PRIVATE PkgConfig::PIPEWIRE)
    message(STATUS "PipeWire audio backend enabled")
else()
    message(STATUS "PipeWire audio backend disabled (libpipewire-0.3 not found)")
endif()

if(OPENBUS_ENABLE_IPO)
    include(CheckIPOSupported)
    check_ipo_supported(RESULT OPENBUS_IPO_SUPPORTED OUTPUT OPENBUS_IPO_ERROR LANGUAGES CXX)
    if(OPENBUS_IPO_SUPPORTED)
        set_property(TARGET OpenBus PROPERTY INTERPROCEDURAL_OPTIMIZATION_RELEASE TRUE)
        set_property(TARGET OpenBus PROPERTY INTERPROCEDURAL_OPTIMIZATION_RELWITHDEBINFO TRUE)
    else()
        message(WARNING "IPO/LTO is unavailable; continuing without it: ${OPENBUS_IPO_ERROR}")
    endif()
endif()

add_executable(OpenBusPhysicsProbe
    tests/BusSimulationSteeringProbe.cpp
    src/BusSimulation.cpp
    src/Logger.cpp
    src/PerfTrace.cpp
    src/RoadFeatures.cpp
    src/Variables.cpp
)

openbus_configure_perf_trace(OpenBusPhysicsProbe)
target_include_directories(OpenBusPhysicsProbe PRIVATE src)
target_link_libraries(OpenBusPhysicsProbe PRIVATE ${OPENBUS_ODE_TARGET})

add_executable(OpenBusStabilityProbe
    tests/BusSimulationStabilityProbe.cpp
    src/BusSimulation.cpp
    src/Logger.cpp
    src/PerfTrace.cpp
    src/RoadFeatures.cpp
    src/Variables.cpp
)

openbus_configure_perf_trace(OpenBusStabilityProbe)
target_include_directories(OpenBusStabilityProbe PRIVATE src)
target_link_libraries(OpenBusStabilityProbe PRIVATE ${OPENBUS_ODE_TARGET})

add_executable(OpenBusVariablesProbe
    tests/VariablesProbe.cpp
    src/Variables.cpp
)
target_include_directories(OpenBusVariablesProbe PRIVATE src)
add_test(NAME OpenBusVariablesProbe COMMAND OpenBusVariablesProbe)

add_executable(OpenBusConfigurationParserProbe
    tests/ConfigurationParserProbe.cpp
    src/ConfigurationParser.cpp
)
target_include_directories(OpenBusConfigurationParserProbe PRIVATE src)
add_test(NAME OpenBusConfigurationParserProbe COMMAND OpenBusConfigurationParserProbe)

add_executable(OpenBusTextureLoaderProbe
    tests/TextureLoaderProbe.cpp
    src/Logger.cpp
    src/PerfTrace.cpp
    src/TextureLoader.cpp
)
target_include_directories(OpenBusTextureLoaderProbe PRIVATE src)
target_include_directories(OpenBusTextureLoaderProbe SYSTEM PRIVATE ${stb_SOURCE_DIR})
add_test(NAME OpenBusTextureLoaderProbe COMMAND OpenBusTextureLoaderProbe)

add_test(NAME OpenBusPhysicsProbe COMMAND OpenBusPhysicsProbe)
add_test(NAME OpenBusStabilityProbe COMMAND OpenBusStabilityProbe)

add_executable(OpenBusCameraMathProbe
    tests/CameraMathProbe.cpp
    src/CameraMath.cpp
)
target_include_directories(OpenBusCameraMathProbe PRIVATE src)
add_test(NAME OpenBusCameraMathProbe COMMAND OpenBusCameraMathProbe)

add_executable(OpenBusRoadFeaturesProbe
    tests/RoadFeaturesProbe.cpp
    src/RoadFeatures.cpp
)
target_include_directories(OpenBusRoadFeaturesProbe PRIVATE src)
add_test(NAME OpenBusRoadFeaturesProbe COMMAND OpenBusRoadFeaturesProbe)

add_executable(OpenBusO3DLoaderProbe
    tests/O3DLoaderProbe.cpp
    src/O3DLoader.cpp
    src/PerfTrace.cpp
)
openbus_configure_perf_trace(OpenBusO3DLoaderProbe)
target_include_directories(OpenBusO3DLoaderProbe PRIVATE src)
add_test(NAME OpenBusO3DLoaderProbe
         COMMAND OpenBusO3DLoaderProbe ${CMAKE_CURRENT_SOURCE_DIR}/uno3d/SitzeU.o3d)

add_executable(OpenBusModelConfigProbe
    tests/ModelConfigProbe.cpp
    src/ConfigurationParser.cpp
    src/ModelConfigLoader.cpp
    src/PerfTrace.cpp
    src/Variables.cpp
)
target_include_directories(OpenBusModelConfigProbe PRIVATE src)
add_test(NAME OpenBusModelConfigProbe COMMAND OpenBusModelConfigProbe)

add_executable(OpenBusVehicleConfigProbe
    tests/VehicleConfigProbe.cpp
    src/ConfigurationParser.cpp
    src/ModelConfigLoader.cpp
    src/PerfTrace.cpp
    src/Variables.cpp
    src/VehicleConfigLoader.cpp
    src/osc/osc.cpp
)
target_include_directories(OpenBusVehicleConfigProbe PRIVATE src src/osc)
add_test(NAME OpenBusVehicleConfigProbe COMMAND OpenBusVehicleConfigProbe)

add_executable(OpenBusScriptTextureRuntimeProbe
    tests/ScriptTextureRuntimeProbe.cpp
    src/Logger.cpp
    src/PerfTrace.cpp
    src/ScriptRuntime.cpp
    src/TextureLoader.cpp
    src/Variables.cpp
    src/osc/osc.cpp
)
target_include_directories(OpenBusScriptTextureRuntimeProbe PRIVATE src src/osc)
target_include_directories(OpenBusScriptTextureRuntimeProbe SYSTEM PRIVATE
    ${gli_SOURCE_DIR}
    ${gli_SOURCE_DIR}/external/glm
    ${stb_SOURCE_DIR})
target_link_libraries(OpenBusScriptTextureRuntimeProbe PRIVATE OpenBusLua)
add_test(NAME OpenBusScriptTextureRuntimeProbe COMMAND OpenBusScriptTextureRuntimeProbe)

add_executable(uno3d_converter
    uno3d/o3d_to_obj.cpp
    src/PerfTrace.cpp
)
openbus_configure_perf_trace(uno3d_converter)

target_include_directories(uno3d_converter SYSTEM PRIVATE ${stb_SOURCE_DIR})
target_include_directories(uno3d_converter PRIVATE src)

add_executable(obj_bundle
    tools/obj_bundle.cpp
    src/ObjLoader.cpp
    src/PerfTrace.cpp
)
openbus_configure_perf_trace(obj_bundle)
target_include_directories(obj_bundle PRIVATE src)

if(MSVC)
    target_compile_options(OpenBus PRIVATE /W4 /permissive-)
    target_compile_options(OpenBus PRIVATE
        $<$<CONFIG:Release>:/Gy>
        $<$<CONFIG:Release>:/Gw>
        $<$<CONFIG:RelWithDebInfo>:/Gy>
        $<$<CONFIG:RelWithDebInfo>:/Gw>)
    target_compile_options(OpenBus PRIVATE $<$<CONFIG:Release>:/Zi>)
    target_link_options(OpenBus PRIVATE
        $<$<CONFIG:Release>:/DEBUG>
        $<$<CONFIG:Release>:/OPT:REF>
        $<$<CONFIG:Release>:/OPT:ICF>
        $<$<CONFIG:RelWithDebInfo>:/OPT:REF>
        $<$<CONFIG:RelWithDebInfo>:/OPT:ICF>)
    target_compile_definitions(OpenBus PRIVATE _CRT_SECURE_NO_WARNINGS GLFW_INCLUDE_NONE)
    target_link_libraries(OpenBus PRIVATE windowscodecs dbghelp)
    target_link_libraries(OpenBus PRIVATE winmm)
else()
    target_compile_options(OpenBus PRIVATE -Wall -Wextra -Wpedantic)
endif()
