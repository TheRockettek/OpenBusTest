# OpenBus module libraries.
#
# Each folder under src/ is a static library with an explicit dependency on the
# modules below it. Includes are rooted at src/ (for example
# "core/Logger.h"), so every module exposes src/ as its public include root.
#
#   core       logging, tracing, variables, shared value types
#   osc        OMSI script (.osc) to Lua converter
#   config     OMSI configuration parsers (.bus/.cfg/.sco/...)
#   assets     mesh/texture decoders and screenshot encoding
#   physics    ODE bus simulation and collision configuration
#   map        map configuration, geometry generation and collision streaming
#   scripting  vehicle/scenery script runtime (native OSC + Lua backends)
#   audio      OpenAL sound engine and vehicle sound banks
#   input      keyboard/mouse binding helpers
#   render     OpenGL renderer, map renderer and vehicle renderer
#   app        window/frame orchestration and the OpenBus executable

# Library defaults shared by every first-party module.
function(openbus_add_module name)
    cmake_parse_arguments(ARG "" "" "SOURCES;DEPENDS;PRIVATE_DEPENDS" ${ARGN})
    add_library(${name} STATIC ${ARG_SOURCES})
    add_library(OpenBus::${name} ALIAS ${name})
    target_include_directories(${name} PUBLIC "${OPENBUS_SOURCE_ROOT}")
    target_link_libraries(${name} PUBLIC ${ARG_DEPENDS} PRIVATE ${ARG_PRIVATE_DEPENDS})
    openbus_apply_project_settings(${name})
endfunction()

openbus_add_module(openbus_core
    SOURCES
        core/Logger.cpp
        core/OmsiRoot.cpp
        core/PerfTrace.cpp
        core/Variables.cpp
    DEPENDS
        Threads::Threads
)
target_compile_definitions(openbus_core PUBLIC
    OPENBUS_ENABLE_PERF_TRACE=${OPENBUS_PERF_TRACE_DEFINE})

openbus_add_module(openbus_osc
    SOURCES
        osc/osc.cpp
)

openbus_add_module(openbus_config
    SOURCES
        config/BusConfigLoader.cpp
        config/ConfigurationParser.cpp
        config/HumanConfigLoader.cpp
        config/ModelConfigJson.cpp
        config/ModelConfigLoader.cpp
        config/SceneryObjectConfigLoader.cpp
        config/VehicleConfigLoader.cpp
    DEPENDS
        openbus_core
    PRIVATE_DEPENDS
        openbus_osc
)

openbus_add_module(openbus_assets
    SOURCES
        assets/O3DLoader.cpp
        assets/ObjLoader.cpp
        assets/ScreenshotWriter.cpp
        assets/TextureAssetLoader.cpp
        assets/TextureLoader.cpp
        assets/XLoader.cpp
    DEPENDS
        openbus_core
        openbus_third_party_headers
    PRIVATE_DEPENDS
        ZLIB::ZLIB
)

openbus_add_module(openbus_physics
    SOURCES
        physics/BusConfiguration.cpp
        physics/BusSimulation.cpp
        physics/RoadFeatures.cpp
    DEPENDS
        openbus_config
        openbus_assets
        ${OPENBUS_ODE_TARGET}
)

openbus_add_module(openbus_map
    SOURCES
        map/MapCollisionBuilder.cpp
        map/MapCollisionStreamer.cpp
        map/MapConfigLoader.cpp
        map/MapRoadGeometry.cpp
        map/MapSceneryPlacement.cpp
        map/MapSplineGeometry.cpp
        map/MapSplineProfile.cpp
        map/MapTrafficRuntime.cpp
        map/MapWaterGeometry.cpp
    DEPENDS
        openbus_config
        openbus_assets
        openbus_physics
)

openbus_add_module(openbus_scripting
    SOURCES
        scripting/ScriptRuntime.cpp
        scripting/ScriptLuaBindings.cpp
        scripting/ScriptNativeBackend.cpp
        scripting/ScriptRuntimeImpl.cpp
        scripting/ScriptTextures.cpp
    DEPENDS
        openbus_config
        openbus_assets
    PRIVATE_DEPENDS
        openbus_osc
        OpenBusLua
)

openbus_add_module(openbus_audio
    SOURCES
        audio/SoundEngine.cpp
        audio/VehicleSoundBank.cpp
    DEPENDS
        openbus_config
    PRIVATE_DEPENDS
        OpenAL::OpenAL
)

openbus_add_module(openbus_input
    SOURCES
        input/VehicleInputBindings.cpp
        input/VehicleKeyBindings.cpp
    DEPENDS
        openbus_core
        OpenBus::GLFW
)

openbus_add_module(openbus_render
    SOURCES
        render/AssetRequestManager.cpp
        render/BusModelLoader.cpp
        render/CameraMath.cpp
        render/ComputeCulling.cpp
        render/CoreRenderer.cpp
        render/MatrixMath.cpp
        render/OpenGLFunctions.cpp
        render/RenderPrimitives.cpp
        render/RendererReflection.cpp
        render/WeatherEffects.cpp
        render/map/MapRenderer.cpp
        render/map/MapRendererDraw.cpp
        render/map/MapRendererHelpers.cpp
        render/map/MapSceneLoader.cpp
        render/map/MapSceneTiles.cpp
        render/map/MapSceneryAssets.cpp
    DEPENDS
        openbus_assets
        openbus_config
        openbus_map
        openbus_physics
        openbus_scripting
        OpenBus::GLFW
        OpenGL::GL
)

add_executable(OpenBus
    app/CrashHandler.cpp
    app/RenderLoop.cpp
    app/RenderLoopDraw.cpp
    app/RenderLoopInput.cpp
    app/RenderLoopSimulation.cpp
    app/StatusHud.cpp
    app/main.cpp
    app/vehicle/Vehicle.cpp
    app/vehicle/VehicleAnimation.cpp
    app/vehicle/VehicleDefinition.cpp
    app/vehicle/VehicleDraw.cpp
    app/vehicle/VehicleInteraction.cpp
    app/vehicle/VehicleMaterials.cpp
    app/vehicle/VehicleMeshLoading.cpp
    app/vehicle/VehicleTextures.cpp
    app/vehicle/VehicleVisibility.cpp
)
openbus_apply_project_settings(OpenBus)
target_link_libraries(OpenBus PRIVATE
    openbus_audio
    openbus_input
    openbus_map
    openbus_physics
    openbus_render
    openbus_scripting
)
openbus_copy_lua_runtime(OpenBus)

if(WIN32)
    target_link_libraries(OpenBus PRIVATE dbghelp windowscodecs winmm)
    add_custom_command(TARGET OpenBus POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy -t "$<TARGET_FILE_DIR:OpenBus>"
            "$<TARGET_FILE:OpenAL::OpenAL>"
        COMMAND_EXPAND_LISTS)
elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_link_options(OpenBus PRIVATE -rdynamic)
endif()

if(MSVC)
    target_link_options(OpenBus PRIVATE
        $<$<CONFIG:Release>:/DEBUG>
        $<$<CONFIG:Release>:/OPT:REF>
        $<$<CONFIG:Release>:/OPT:ICF>
        $<$<CONFIG:RelWithDebInfo>:/OPT:REF>
        $<$<CONFIG:RelWithDebInfo>:/OPT:ICF>)
endif()
