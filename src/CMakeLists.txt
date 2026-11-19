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
#   camera     backend-neutral camera matrices and pose helpers
#   motion_data backend-neutral camera/object frame history
#   human_geometry backend-neutral human mesh preparation, skinning, and matrix math
#   render     shared render services and legacy OpenGL renderer implementation
#   app        window/frame orchestration and the OpenBus executable

# Library defaults shared by every first-party module.
set(OPENBUS_METAR_STATION_CSV "${OPENBUS_SOURCE_ROOT}/metar/metar_station_details.csv")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${OPENBUS_METAR_STATION_CSV}")
file(STRINGS "${OPENBUS_METAR_STATION_CSV}" OPENBUS_METAR_STATION_ROWS ENCODING UTF-8)
if(NOT OPENBUS_METAR_STATION_ROWS)
    message(FATAL_ERROR "No METAR station data found in ${OPENBUS_METAR_STATION_CSV}")
endif()
set(OPENBUS_METAR_STATION_CHUNKS "")
set(OPENBUS_METAR_STATION_CHUNK "")
foreach(OPENBUS_METAR_STATION_ROW IN LISTS OPENBUS_METAR_STATION_ROWS)
    if(OPENBUS_METAR_STATION_ROW MATCHES "\\)OPENBUS_METAR\"")
        message(FATAL_ERROR "METAR station data contains the reserved resource delimiter")
    endif()
    string(LENGTH "${OPENBUS_METAR_STATION_CHUNK}" OPENBUS_METAR_CHUNK_LENGTH)
    if(OPENBUS_METAR_CHUNK_LENGTH GREATER 12000)
        string(APPEND OPENBUS_METAR_STATION_CHUNKS
               "R\"OPENBUS_METAR(${OPENBUS_METAR_STATION_CHUNK})OPENBUS_METAR\",\n")
        set(OPENBUS_METAR_STATION_CHUNK "")
    endif()
    string(APPEND OPENBUS_METAR_STATION_CHUNK "${OPENBUS_METAR_STATION_ROW}\n")
endforeach()
string(APPEND OPENBUS_METAR_STATION_CHUNKS
       "R\"OPENBUS_METAR(${OPENBUS_METAR_STATION_CHUNK})OPENBUS_METAR\",\n")
file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/generated")
file(GENERATE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/generated/MetarStations.h"
    CONTENT "#pragma once\ninline constexpr const char* openbusMetarStationChunks[] = {\n${OPENBUS_METAR_STATION_CHUNKS}};\n")

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
        core/ActiveMetarStations.cpp
        core/AtomicFile.cpp
        core/GraphicsPreferences.cpp
        core/Logger.cpp
        core/Metar.cpp
        core/MetarNoaa.cpp
        core/MetarNoaaHttp.cpp
        core/WeatherSelection.cpp
        core/OmsiRoot.cpp
        core/PerfTrace.cpp
        core/SolarLighting.cpp
        core/Variables.cpp
        core/WeatherEnvironment.cpp
        core/WeatherState.cpp
    DEPENDS
        Threads::Threads
)
target_include_directories(openbus_core PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/generated")
if(WIN32)
    if(MSVC)
        set_property(SOURCE core/WeatherSelection.cpp APPEND PROPERTY COMPILE_OPTIONS /utf-8)
    endif()
    target_link_libraries(openbus_core PRIVATE winhttp)
else()
    find_package(CURL REQUIRED)
    target_link_libraries(openbus_core PRIVATE CURL::libcurl)
endif()
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
        config/HofData.cpp
        config/HumanConfigLoader.cpp
        config/ModelConfigJson.cpp
        config/ModelConfigLoader.cpp
        config/SceneryObjectConfigLoader.cpp
        config/TextureSurfaceConfigLoader.cpp
        config/VehicleCatalog.cpp
        config/VehicleConfigLoader.cpp
        config/WeatherPresetLoader.cpp
    DEPENDS
        openbus_core
    PRIVATE_DEPENDS
        openbus_osc
)

openbus_add_module(openbus_assets
    SOURCES
        assets/O3DLoader.cpp
        assets/ObjLoader.cpp
        assets/PngWriter.cpp
        assets/ScreenshotWriter.cpp
        assets/TextureAssetLoader.cpp
        assets/TextureLoader.cpp
        assets/VehicleTextureResolver.cpp
        assets/VehicleMeshGeometry.cpp
        assets/WavLoader.cpp
        assets/XLoader.cpp
    DEPENDS
        openbus_core
        openbus_third_party_headers
    PRIVATE_DEPENDS
        ZLIB::ZLIB
        TIFF::TIFF
)

openbus_add_module(openbus_physics
    SOURCES
        physics/BusConfiguration.cpp
        physics/BusSimulation.cpp
        physics/RoadWeather.cpp
        physics/RoadFeatures.cpp
        physics/World.cpp
    DEPENDS
        openbus_core
        openbus_config
        openbus_assets
        ${OPENBUS_ODE_TARGET}
)

openbus_add_module(openbus_map
    SOURCES
        map/MapAssetValidation.cpp
        map/MapBakeArchive.cpp
        map/MapBakeMapIndex.cpp
        map/MapBakeManifest.cpp
        map/MapCollisionBuilder.cpp
        map/MapCollisionStreamer.cpp
        map/MapConfigLoader.cpp
        map/MapRoadGeometry.cpp
        map/MapRoadRenderGeometry.cpp
        map/MapSceneryGeometry.cpp
        map/MapSurfaceGeometry.cpp
        map/MapSceneryPlacement.cpp
        map/MapSplineGeometry.cpp
        map/MapSplineProfile.cpp
        map/MapTrafficRuntime.cpp
        map/MapWaterGeometry.cpp
        map/SceneryMaterialRuntime.cpp
    DEPENDS
        openbus_config
        openbus_assets
        openbus_physics
)

openbus_add_module(openbus_scenery_preparation
    SOURCES
        render/map/PreparedMapScenery.cpp
    DEPENDS
        openbus_assets
        openbus_config
        openbus_map
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
        openbus_assets
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

openbus_add_module(openbus_motion_data
    SOURCES
        render/MotionData.cpp
)

openbus_add_module(openbus_camera
    SOURCES
        render/CameraMath.cpp
    DEPENDS
        openbus_core
)

openbus_add_module(openbus_human_geometry
    SOURCES
        render/HumanMeshGeometry.cpp
        render/HumanSkinning.cpp
        render/MatrixMath.cpp
    DEPENDS
        openbus_assets
        openbus_config
)

openbus_add_module(openbus_render
    SOURCES
        render/AssetRequestManager.cpp
        render/BusModelLoader.cpp
        render/ComputeCulling.cpp
        render/CoreRenderer.cpp
        render/OpenGLFramePresenter.cpp
        render/OpenGLFunctions.cpp
        render/RenderPrimitives.cpp
        render/RendererReflection.cpp
        render/SpotlightShadowAtlas.cpp
        render/SpotlightShadowCamera.cpp
        render/map/MapRenderer.cpp
        render/map/MapRendererDraw.cpp
        render/map/MapRendererHelpers.cpp
        render/map/MapSceneLoader.cpp
        render/map/MapSceneTiles.cpp
        render/map/MapSceneryAssets.cpp
    DEPENDS
        openbus_assets
        openbus_camera
        openbus_config
        openbus_human_geometry
        openbus_map
        openbus_motion_data
        openbus_physics
        openbus_scenery_preparation
        openbus_scripting
        OpenBus::GLFW
)
add_library(openbus_render_legacy_gl INTERFACE)
target_link_libraries(openbus_render_legacy_gl INTERFACE openbus_render OpenGL::GL)
set(OPENBUS_SMAA_ROOT "${PROJECT_SOURCE_DIR}/third_party/smaa")
set(OPENBUS_SMAA_SHADER "${OPENBUS_SMAA_ROOT}/SMAA.hlsl")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${OPENBUS_SMAA_SHADER}")
file(READ "${OPENBUS_SMAA_SHADER}" OPENBUS_SMAA_SHADER_SOURCE)
string(ASCII 183 OPENBUS_SMAA_MIDDLE_DOT)
string(REPLACE "${OPENBUS_SMAA_MIDDLE_DOT}" "+" OPENBUS_SMAA_SHADER_SOURCE
       "${OPENBUS_SMAA_SHADER_SOURCE}")
string(FIND "${OPENBUS_SMAA_SHADER_SOURCE}" ")OPENBUS_SMAA\"" OPENBUS_SMAA_DELIMITER_POSITION)
if(NOT OPENBUS_SMAA_DELIMITER_POSITION EQUAL -1)
    message(FATAL_ERROR "The SMAA shader contains the reserved C++ resource delimiter")
endif()
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/generated/SmaaShader.h"
    "#pragma once\ninline constexpr char openbusSmaaShader[] = R\"OPENBUS_SMAA(\n${OPENBUS_SMAA_SHADER_SOURCE}\n)OPENBUS_SMAA\";\n")
target_include_directories(openbus_render
    PRIVATE
        "${CMAKE_CURRENT_BINARY_DIR}/generated"
        "${OPENBUS_SMAA_ROOT}/Textures"
)

if(OPENBUS_BUILD_VULKAN_PROBE)
    add_library(openbus_vulkan_backend STATIC
        render/VulkanFunctions.cpp
        render/VulkanRenderer.cpp
    )
    openbus_apply_project_settings(openbus_vulkan_backend)
    target_include_directories(openbus_vulkan_backend
        PUBLIC
            "${CMAKE_CURRENT_SOURCE_DIR}"
        PRIVATE
            "${openbus_vulkan_headers_SOURCE_DIR}/include"
            "${openbus_volk_SOURCE_DIR}"
    )
    target_compile_definitions(openbus_vulkan_backend PRIVATE GLFW_INCLUDE_NONE)
    target_link_libraries(openbus_vulkan_backend
        PUBLIC OpenBus::GLFW
        PRIVATE ${CMAKE_DL_LIBS}
    )
endif()

if(OPENBUS_BUILD_DILIGENT_VULKAN_PROBE OR OPENBUS_BUILD_DILIGENT_RENDERER)
    add_library(openbus_diligent_render STATIC
        render/diligent/DiligentColorPresent.cpp
        render/diligent/DiligentDynamicTexture2D.cpp
        render/diligent/DiligentHumanAsset.cpp
        render/diligent/DiligentMapLighting.cpp
        render/diligent/DiligentSceneLighting.cpp
        render/diligent/DiligentSpotlightShadowMaps.cpp
        render/diligent/DiligentMapTileAsset.cpp
        render/diligent/DiligentMapTileSet.cpp
        render/diligent/DiligentMotionRenderTargets.cpp
        render/diligent/DiligentMultisampling.cpp
        render/diligent/DiligentObjModel.cpp
        render/diligent/DiligentPrimitiveRenderer.cpp
        render/diligent/DiligentReflectionTarget.cpp
        render/diligent/DiligentRuntimeTexture2D.cpp
        render/diligent/DiligentSceneFrame.cpp
        render/diligent/DiligentSceneryAsset.cpp
        render/diligent/DiligentSceneryAssetCache.cpp
        render/diligent/DiligentSmaa.cpp
        render/diligent/DiligentTexture2D.cpp
        render/diligent/DiligentTexture2DArray.cpp
        render/diligent/DiligentTextureCompression.cpp
        render/diligent/DiligentTexturedMesh.cpp
        render/diligent/DiligentWeatherClouds.cpp
        render/diligent/DiligentWeatherPrecipitation.cpp
        render/diligent/DiligentVehicleAsset.cpp
    )
    target_include_directories(openbus_diligent_render PUBLIC
        "${OPENBUS_SOURCE_ROOT}"
        "${diligentcore_SOURCE_DIR}"
    )
    target_include_directories(openbus_diligent_render PRIVATE
        "${CMAKE_CURRENT_BINARY_DIR}/generated"
        "${OPENBUS_SMAA_ROOT}/Textures"
    )
    target_link_libraries(openbus_diligent_render PUBLIC
        Diligent-Common
        Diligent-GraphicsEngineInterface
        openbus_assets
        openbus_human_geometry
        openbus_map
        openbus_motion_data
        openbus_scenery_preparation
    )
    openbus_apply_project_settings(openbus_diligent_render)

    add_library(openbus_diligent_ui STATIC
        app/DiligentImGuiLayer.cpp
        render/diligent/DiligentImGuiRenderer.cpp
    )
    target_include_directories(openbus_diligent_ui PUBLIC "${OPENBUS_SOURCE_ROOT}")
    target_link_libraries(openbus_diligent_ui PUBLIC openbus_diligent_render openbus_imgui_core)
    target_compile_definitions(openbus_diligent_ui PRIVATE GLFW_INCLUDE_NONE)
    openbus_apply_project_settings(openbus_diligent_ui)

    add_library(openbus_diligent_vulkan STATIC
        render/diligent/DiligentVulkanFrame.cpp
    )
    target_include_directories(openbus_diligent_vulkan PUBLIC
        "${OPENBUS_SOURCE_ROOT}"
        "${diligentcore_SOURCE_DIR}"
    )
    target_link_libraries(openbus_diligent_vulkan PUBLIC
        Diligent-GraphicsEngineVk-static
    )
    target_compile_definitions(openbus_diligent_vulkan PUBLIC VK_USE_PLATFORM_WIN32_KHR=1)
    openbus_apply_project_settings(openbus_diligent_vulkan)

    add_library(openbus_diligent_app STATIC
        app/DiligentWindowBridge.cpp
        app/DiligentMapBridge.cpp
        app/DiligentHumanBridge.cpp
        app/DiligentHumanPreparationBridge.cpp
        app/DiligentFrameRenderer.cpp
        app/DiligentScriptTexture.cpp
        app/vehicle/DiligentVehicleSourceBridge.cpp
        app/vehicle/DiligentVehicleSceneBridge.cpp
        app/vehicle/DiligentVehicleRuntimeTextures.cpp
    )
    target_include_directories(openbus_diligent_app PUBLIC
        "${OPENBUS_SOURCE_ROOT}"
        "${diligentcore_SOURCE_DIR}"
    )
    target_link_libraries(openbus_diligent_app PUBLIC
        openbus_diligent_render
        openbus_diligent_ui
        openbus_diligent_vulkan
        openbus_camera
        openbus_render
        openbus_scripting
        OpenBus::GLFW
    )
    target_compile_definitions(openbus_diligent_app PRIVATE GLFW_INCLUDE_NONE)
    openbus_apply_project_settings(openbus_diligent_app)

endif()

add_executable(OpenBus
    app/CrashHandler.cpp
    app/CustomWeatherUi.cpp
    app/FrameTiming.cpp
    app/MetarStationUi.cpp
    app/MapTrafficWorker.cpp
    app/RenderLoop.cpp
    app/RenderLoopCamera.cpp
    app/RenderLoopDraw.cpp
    app/RenderLoopInput.cpp
    app/RenderLoopSimulation.cpp
    app/RenderLoopSetup.cpp
    app/SteamPresence.cpp
    app/traffic/PassengerNetwork.cpp
    app/traffic/PlayerTimetable.cpp
    app/traffic/PlayerTimetableController.cpp
    app/traffic/PlayerTimetableDiagnostics.cpp
    app/traffic/PlayerTripPolicy.cpp
    app/traffic/ScheduledBusTiming.cpp
    app/traffic/RenderLoopTraffic.cpp
    app/main.cpp
    app/vehicle/Vehicle.cpp
    app/vehicle/VehicleAnimation.cpp
    app/vehicle/VehicleDefinition.cpp
    app/vehicle/VehicleInteraction.cpp
    app/vehicle/VehicleMaterials.cpp
    app/vehicle/VehicleMeshLoading.cpp
    app/vehicle/VehiclePreferences.cpp
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
    ${CMAKE_DL_LIBS}
)
if(OPENBUS_BUILD_DILIGENT_RENDERER)
    target_link_libraries(OpenBus PRIVATE openbus_diligent_app)
    target_sources(OpenBus PRIVATE
        app/vehicle/DiligentVehicleBridge.cpp
        app/vehicle/DiligentVehicleScenePreparation.cpp
        app/vehicle/VehicleDiligentTextures.cpp
    )
    target_compile_definitions(OpenBus PRIVATE OPENBUS_DILIGENT_RENDERER_AVAILABLE=1)
endif()
openbus_copy_lua_runtime(OpenBus)

set(OPENBUS_STEAM_RUNTIME_ROOT "${PROJECT_SOURCE_DIR}/third_party/steamworks/redistributable_bin")
string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" OPENBUS_STEAM_PROCESSOR)
set(OPENBUS_STEAM_RUNTIME "")
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    if(WIN32 AND OPENBUS_STEAM_PROCESSOR MATCHES "^(x86_64|amd64|x64)$")
        set(OPENBUS_STEAM_RUNTIME "${OPENBUS_STEAM_RUNTIME_ROOT}/win64/steam_api64.dll")
    elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux")
        if(OPENBUS_STEAM_PROCESSOR MATCHES "^(x86_64|amd64|x64)$")
            set(OPENBUS_STEAM_RUNTIME "${OPENBUS_STEAM_RUNTIME_ROOT}/linux64/libsteam_api.so")
        elseif(OPENBUS_STEAM_PROCESSOR MATCHES "^(aarch64|arm64)$")
            set(OPENBUS_STEAM_RUNTIME "${OPENBUS_STEAM_RUNTIME_ROOT}/linuxarm64/libsteam_api.so")
        endif()
    endif()
endif()
if(OPENBUS_STEAM_RUNTIME)
    if(CMAKE_CONFIGURATION_TYPES)
        set(OPENBUS_STEAM_OUTPUT_DIRECTORY "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/$<CONFIG>")
    else()
        set(OPENBUS_STEAM_OUTPUT_DIRECTORY "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
    endif()
    add_custom_target(openbus_steam_runtime
        COMMAND ${CMAKE_COMMAND} -E make_directory "${OPENBUS_STEAM_OUTPUT_DIRECTORY}"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OPENBUS_STEAM_RUNTIME}"
            "${OPENBUS_STEAM_OUTPUT_DIRECTORY}"
        VERBATIM)
    add_dependencies(OpenBus openbus_steam_runtime)
elseif(WIN32 OR CMAKE_SYSTEM_NAME STREQUAL "Linux")
    message(WARNING "No Steamworks runtime is available for ${CMAKE_SYSTEM_PROCESSOR} "
        "with ${CMAKE_SIZEOF_VOID_P}-byte pointers; "
        "OpenBus will run without Steam presence")
endif()

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
