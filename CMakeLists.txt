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
    src/MapConfigLoader.cpp
    src/MapSceneryPlacement.cpp
    src/MapCollisionBuilder.cpp
    src/MapCollisionStreamer.cpp
    src/MapRoadGeometry.cpp
    src/MapSplineGeometry.cpp
    src/MapSplineProfile.cpp
    src/VehicleConfigLoader.cpp
    src/ModelConfigLoader.cpp
    src/ModelConfigJson.cpp
    src/BusConfigLoader.cpp
    src/BusModelLoader.cpp
    src/CameraMath.cpp
    src/MapRenderer.cpp
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
    src/XLoader.cpp
    src/OpenGLFunctions.cpp
    src/ScriptRuntime.cpp
    src/SoundEngine.cpp
    src/VehicleSoundBank.cpp
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
target_link_libraries(OpenBus PRIVATE ${OPENBUS_ODE_TARGET} OpenBus::GLFW OpenGL::GL OpenBusLua
                                      OpenAL::OpenAL)

if(OPENBUS_BUILD_BGFX_LIGHTING_PROBE)
    set(OPENBUS_BGFX_SHADER_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/shaders/bgfx")
    set(OPENBUS_BGFX_SHADER_OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/bgfx-shaders")
    file(MAKE_DIRECTORY "${OPENBUS_BGFX_SHADER_OUTPUT_DIR}")

    foreach(shader IN ITEMS vs_lit_cube fs_lit_cube)
        if(shader STREQUAL "vs_lit_cube")
            set(shader_type vertex)
        else()
            set(shader_type fragment)
        endif()
        add_custom_command(
            OUTPUT "${OPENBUS_BGFX_SHADER_OUTPUT_DIR}/${shader}.bin"
            COMMAND $<TARGET_FILE:bgfx::shaderc>
                -f "${OPENBUS_BGFX_SHADER_SOURCE_DIR}/${shader}.sc"
                -o "${OPENBUS_BGFX_SHADER_OUTPUT_DIR}/${shader}.bin"
                --type ${shader_type}
                --platform windows
                --profile 150
                --varyingdef "${OPENBUS_BGFX_SHADER_SOURCE_DIR}/varying.def.sc"
                -i "${BGFX_SHADER_INCLUDE_PATH}"
            DEPENDS
                "${OPENBUS_BGFX_SHADER_SOURCE_DIR}/${shader}.sc"
                "${OPENBUS_BGFX_SHADER_SOURCE_DIR}/varying.def.sc"
                bgfx::shaderc
            COMMENT "Compiling bgfx GLSL 1.50 shader ${shader}"
            VERBATIM
        )
        list(APPEND OPENBUS_BGFX_SHADER_BINARIES
             "${OPENBUS_BGFX_SHADER_OUTPUT_DIR}/${shader}.bin")
    endforeach()

    add_custom_target(OpenBusBgfxShaders DEPENDS ${OPENBUS_BGFX_SHADER_BINARIES})
    add_executable(OpenBusBgfxLightingProbe src/BgfxLightingProbe.cpp)
    add_dependencies(OpenBusBgfxLightingProbe OpenBusBgfxShaders)
    target_link_libraries(OpenBusBgfxLightingProbe PRIVATE bgfx::bgfx OpenBus::GLFW)
    target_compile_features(OpenBusBgfxLightingProbe PRIVATE cxx_std_23)
    add_custom_command(TARGET OpenBusBgfxLightingProbe POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${OPENBUS_BGFX_SHADER_OUTPUT_DIR}"
            "$<TARGET_FILE_DIR:OpenBusBgfxLightingProbe>/bgfx-shaders"
        VERBATIM)
endif()

if(WIN32)
    add_custom_command(TARGET OpenBus POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy -t "$<TARGET_FILE_DIR:OpenBus>"
            "$<TARGET_FILE:OpenAL::OpenAL>"
        COMMAND_EXPAND_LISTS)
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

add_executable(OpenBusPhysicsStressProbe
    tests/BusSimulationStressProbe.cpp
    src/BusSimulation.cpp
    src/Logger.cpp
    src/PerfTrace.cpp
    src/RoadFeatures.cpp
    src/Variables.cpp
)

openbus_configure_perf_trace(OpenBusPhysicsStressProbe)
target_include_directories(OpenBusPhysicsStressProbe PRIVATE src)
target_link_libraries(OpenBusPhysicsStressProbe PRIVATE ${OPENBUS_ODE_TARGET})

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

add_executable(OpenBusMapConfigProbe
    tests/MapConfigLoaderProbe.cpp
    src/MapConfigLoader.cpp
)
target_include_directories(OpenBusMapConfigProbe PRIVATE src)
add_test(NAME OpenBusMapConfigProbe COMMAND OpenBusMapConfigProbe)

add_executable(OpenBusMapChronoTileProbe
    tests/MapChronoTileProbe.cpp
    src/MapConfigLoader.cpp
)
target_include_directories(OpenBusMapChronoTileProbe PRIVATE src)
add_test(NAME OpenBusMapChronoTileProbe COMMAND OpenBusMapChronoTileProbe)

add_executable(OpenBusMapSplineGeometryProbe
    tests/MapSplineGeometryProbe.cpp
    src/MapSplineGeometry.cpp
)
target_include_directories(OpenBusMapSplineGeometryProbe PRIVATE src)
add_test(NAME OpenBusMapSplineGeometryProbe COMMAND OpenBusMapSplineGeometryProbe)

add_executable(OpenBusMapRoadGeometryProbe
    tests/MapRoadGeometryProbe.cpp
    src/MapRoadGeometry.cpp
    src/MapSplineGeometry.cpp
)
target_include_directories(OpenBusMapRoadGeometryProbe PRIVATE src)
add_test(NAME OpenBusMapRoadGeometryProbe COMMAND OpenBusMapRoadGeometryProbe)

add_executable(OpenBusMapCollisionBuilderProbe
    tests/MapCollisionBuilderProbe.cpp
    src/ConfigurationParser.cpp
    src/MapCollisionBuilder.cpp
    src/MapConfigLoader.cpp
    src/MapSceneryPlacement.cpp
    src/MapRoadGeometry.cpp
    src/MapSplineGeometry.cpp
    src/MapSplineProfile.cpp
    src/ModelConfigLoader.cpp
    src/O3DLoader.cpp
    src/PerfTrace.cpp
    src/SceneryObjectConfigLoader.cpp
    src/Variables.cpp
    src/VehicleConfigLoader.cpp
    src/osc/osc.cpp
)
openbus_configure_perf_trace(OpenBusMapCollisionBuilderProbe)
target_include_directories(OpenBusMapCollisionBuilderProbe PRIVATE src)
target_link_libraries(OpenBusMapCollisionBuilderProbe PRIVATE ${OPENBUS_ODE_TARGET})
add_test(NAME OpenBusMapCollisionBuilderProbe COMMAND OpenBusMapCollisionBuilderProbe)

add_executable(OpenBusMapCollisionStreamerProbe
    tests/MapCollisionStreamerProbe.cpp
    src/BusSimulation.cpp
    src/ConfigurationParser.cpp
    src/Logger.cpp
    src/MapCollisionBuilder.cpp
    src/MapCollisionStreamer.cpp
    src/MapConfigLoader.cpp
    src/MapSceneryPlacement.cpp
    src/MapRoadGeometry.cpp
    src/MapSplineGeometry.cpp
    src/MapSplineProfile.cpp
    src/ModelConfigLoader.cpp
    src/O3DLoader.cpp
    src/PerfTrace.cpp
    src/RoadFeatures.cpp
    src/SceneryObjectConfigLoader.cpp
    src/Variables.cpp
    src/VehicleConfigLoader.cpp
    src/osc/osc.cpp
)
openbus_configure_perf_trace(OpenBusMapCollisionStreamerProbe)
target_include_directories(OpenBusMapCollisionStreamerProbe PRIVATE src src/osc)
target_link_libraries(OpenBusMapCollisionStreamerProbe PRIVATE ${OPENBUS_ODE_TARGET})
add_test(NAME OpenBusMapCollisionStreamerProbe COMMAND OpenBusMapCollisionStreamerProbe)

add_executable(OpenBusMapSceneryPlacementProbe
    tests/MapSceneryPlacementProbe.cpp
    src/MapSceneryPlacement.cpp
    src/MapSplineGeometry.cpp
)
target_include_directories(OpenBusMapSceneryPlacementProbe PRIVATE src)
add_test(NAME OpenBusMapSceneryPlacementProbe COMMAND OpenBusMapSceneryPlacementProbe)

add_executable(OpenBusMapSplineProfileProbe
    tests/MapSplineProfileProbe.cpp
    src/MapSplineProfile.cpp
)
target_include_directories(OpenBusMapSplineProfileProbe PRIVATE src)
add_test(NAME OpenBusMapSplineProfileProbe
         COMMAND OpenBusMapSplineProfileProbe
                 ${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/map_spline_profile.sli)

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
add_test(NAME OpenBusPhysicsStressProbe COMMAND OpenBusPhysicsStressProbe)

add_executable(OpenBusCameraMathProbe
    tests/CameraMathProbe.cpp
    src/CameraMath.cpp
)
target_include_directories(OpenBusCameraMathProbe PRIVATE src)
add_test(NAME OpenBusCameraMathProbe COMMAND OpenBusCameraMathProbe)

add_executable(OpenBusViewpointProbe tests/ViewpointProbe.cpp)
target_include_directories(OpenBusViewpointProbe PRIVATE src)
add_test(NAME OpenBusViewpointProbe COMMAND OpenBusViewpointProbe)

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

# Exercise text-X geometry parsing without requiring an OpenGL context.
add_executable(OpenBusXLoaderProbe
    tests/XLoaderProbe.cpp
    src/XLoader.cpp
)
openbus_configure_perf_trace(OpenBusXLoaderProbe)
target_include_directories(OpenBusXLoaderProbe PRIVATE src)
add_test(NAME OpenBusXLoaderProbe
         COMMAND OpenBusXLoaderProbe
                 ${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/scenery_text_mesh.x)

add_executable(OpenBusModelConfigProbe
    tests/ModelConfigProbe.cpp
    src/ConfigurationParser.cpp
    src/ModelConfigLoader.cpp
    src/SceneryObjectConfigLoader.cpp
    src/PerfTrace.cpp
    src/Variables.cpp
    src/VehicleConfigLoader.cpp
    src/osc/osc.cpp
)
target_include_directories(OpenBusModelConfigProbe PRIVATE src src/osc)
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

add_executable(OpenBusBusConfigurationCollisionProbe
    tests/BusConfigurationCollisionProbe.cpp
    src/BusConfiguration.cpp
    src/BusConfigLoader.cpp
    src/ConfigurationParser.cpp
    src/ModelConfigLoader.cpp
    src/O3DLoader.cpp
    src/ObjLoader.cpp
    src/PerfTrace.cpp
    src/Variables.cpp
    src/VehicleConfigLoader.cpp
    src/osc/osc.cpp
)
openbus_configure_perf_trace(OpenBusBusConfigurationCollisionProbe)
target_include_directories(OpenBusBusConfigurationCollisionProbe PRIVATE src src/osc)
add_test(NAME OpenBusBusConfigurationCollisionProbe
         COMMAND OpenBusBusConfigurationCollisionProbe)

add_executable(OpenBusSoundEngineProbe
    tests/SoundEngineProbe.cpp
    src/ConfigurationParser.cpp
    src/Logger.cpp
    src/PerfTrace.cpp
    src/VehicleSoundBank.cpp
    src/Variables.cpp
)
target_include_directories(OpenBusSoundEngineProbe PRIVATE src)
add_test(NAME OpenBusSoundEngineProbe COMMAND OpenBusSoundEngineProbe)

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
