# Probe executables. Each probe links the module library it exercises instead
# of recompiling individual sources, so probes always test the same objects the
# game links.

set(OPENBUS_FIXTURES "${CMAKE_CURRENT_SOURCE_DIR}/fixtures")

# openbus_add_probe(<name> <source> LIBS <libs...> [ARGS <args...>] [NO_TEST])
function(openbus_add_probe name source)
    cmake_parse_arguments(ARG "NO_TEST" "" "LIBS;ARGS" ${ARGN})
    add_executable(${name} ${source})
    target_link_libraries(${name} PRIVATE ${ARG_LIBS})
    if(NOT ARG_NO_TEST)
        add_test(NAME ${name} COMMAND ${name} ${ARG_ARGS})
    endif()
endfunction()

# PerfTrace is compiled directly so both instrumentation modes can be tested
# regardless of the OPENBUS_ENABLE_PERF_TRACE setting used for the modules.
foreach(variant IN ITEMS Enabled Disabled)
    if(variant STREQUAL "Enabled")
        set(probe OpenBusPerfTraceProbe)
        set(enabled 1)
    else()
        set(probe OpenBusPerfTraceDisabledProbe)
        set(enabled 0)
    endif()
    add_executable(${probe} PerfTraceProbe.cpp "${OPENBUS_SOURCE_ROOT}/core/PerfTrace.cpp")
    target_include_directories(${probe} PRIVATE "${OPENBUS_SOURCE_ROOT}")
    target_compile_definitions(${probe} PRIVATE OPENBUS_ENABLE_PERF_TRACE=${enabled})
    target_link_libraries(${probe} PRIVATE Threads::Threads)
endforeach()
foreach(mode IN ITEMS toggle autostart cap failure)
    add_test(NAME OpenBusPerfTraceProbe.${mode} COMMAND OpenBusPerfTraceProbe ${mode})
endforeach()
add_test(NAME OpenBusPerfTraceDisabledProbe COMMAND OpenBusPerfTraceDisabledProbe)

# Core / configuration
openbus_add_probe(OpenBusLoggerProbe LoggerProbe.cpp LIBS openbus_core)
add_test(NAME OpenBusLoggerProbe.invalid-path COMMAND OpenBusLoggerProbe invalid-path)
add_test(NAME OpenBusLoggerProbe.write-failure COMMAND OpenBusLoggerProbe write-failure)
set_tests_properties(OpenBusLoggerProbe.write-failure PROPERTIES SKIP_RETURN_CODE 77)
openbus_add_probe(OpenBusVariablesProbe VariablesProbe.cpp LIBS openbus_core)
openbus_add_probe(OpenBusConfigurationParserProbe ConfigurationParserProbe.cpp
    LIBS openbus_config)
openbus_add_probe(OpenBusSceneryObjectConfigProbe SceneryObjectConfigProbe.cpp
    LIBS openbus_config openbus_map)
openbus_add_probe(OpenBusModelConfigProbe ModelConfigProbe.cpp LIBS openbus_config openbus_osc)
add_test(NAME OpenBusModelLogRegressionProbe COMMAND OpenBusModelConfigProbe --log-regression)
openbus_add_probe(OpenBusVehicleConfigProbe VehicleConfigProbe.cpp
    LIBS openbus_config openbus_osc openbus_physics)
openbus_add_probe(OpenBusVehicleCatalogProbe VehicleCatalogProbe.cpp LIBS openbus_config)
add_executable(OpenBusVehicleDefinitionProbe VehicleDefinitionProbe.cpp
    "${OPENBUS_SOURCE_ROOT}/app/vehicle/VehicleDefinition.cpp")
target_include_directories(OpenBusVehicleDefinitionProbe PRIVATE "${OPENBUS_SOURCE_ROOT}")
target_link_libraries(OpenBusVehicleDefinitionProbe PRIVATE openbus_config openbus_core
    openbus_render)
add_test(NAME OpenBusVehicleDefinitionProbe COMMAND OpenBusVehicleDefinitionProbe)
openbus_add_probe(OpenBusHumanConfigProbe HumanConfigLoaderProbe.cpp
    LIBS openbus_config
    ARGS "${OPENBUS_FIXTURES}/human_driver.hum")

# Input
openbus_add_probe(OpenBusVehicleInputBindingsProbe VehicleInputBindingsProbe.cpp
    LIBS openbus_input)

# Assets
openbus_add_probe(OpenBusTextureLoaderProbe TextureLoaderProbe.cpp LIBS openbus_assets)
openbus_add_probe(OpenBusO3DLoaderProbe O3DLoaderProbe.cpp LIBS openbus_assets
    ARGS "${PROJECT_SOURCE_DIR}/uno3d/SitzeU.o3d")
# Exercise text-X geometry parsing without requiring an OpenGL context.
openbus_add_probe(OpenBusXLoaderProbe XLoaderProbe.cpp LIBS openbus_assets ZLIB::ZLIB
    ARGS "${OPENBUS_FIXTURES}/scenery_text_mesh.x")

# Physics
openbus_add_probe(OpenBusOnFootMovementProbe OnFootMovementProbe.cpp LIBS openbus_physics)
openbus_add_probe(OpenBusPhysicsProbe BusSimulationSteeringProbe.cpp LIBS openbus_physics)
openbus_add_probe(OpenBusStabilityProbe BusSimulationStabilityProbe.cpp LIBS openbus_physics)
openbus_add_probe(OpenBusPhysicsStressProbe BusSimulationStressProbe.cpp LIBS openbus_physics)
openbus_add_probe(OpenBusPhysicsContactProbe BusSimulationContactProbe.cpp LIBS openbus_physics)
openbus_add_probe(OpenBusRoadFeaturesProbe RoadFeaturesProbe.cpp LIBS openbus_physics)
openbus_add_probe(OpenBusBusConfigurationCollisionProbe BusConfigurationCollisionProbe.cpp
    LIBS openbus_physics)

# Map
openbus_add_probe(OpenBusMapConfigProbe MapConfigLoaderProbe.cpp LIBS openbus_map)
openbus_add_probe(OpenBusMapChronoTileProbe MapChronoTileProbe.cpp LIBS openbus_map)
openbus_add_probe(OpenBusMapSplineGeometryProbe MapSplineGeometryProbe.cpp LIBS openbus_map)
openbus_add_probe(OpenBusMapRoadGeometryProbe MapRoadGeometryProbe.cpp LIBS openbus_map)
openbus_add_probe(OpenBusMapCoordinatesProbe MapCoordinatesProbe.cpp LIBS openbus_map)
openbus_add_probe(OpenBusMapWaterGeometryProbe MapWaterGeometryProbe.cpp LIBS openbus_map)
openbus_add_probe(OpenBusMapSceneryPlacementProbe MapSceneryPlacementProbe.cpp
    LIBS openbus_map)
openbus_add_probe(OpenBusMapCollisionBuilderProbe MapCollisionBuilderProbe.cpp LIBS openbus_map)
openbus_add_probe(OpenBusMapCollisionStreamerProbe MapCollisionStreamerProbe.cpp
    LIBS openbus_map)
openbus_add_probe(OpenBusMapSplineProfileProbe MapSplineProfileProbe.cpp LIBS openbus_map
    ARGS
        "${OPENBUS_FIXTURES}/map_spline_profile.sli"
        "${OPENBUS_FIXTURES}/map_spline_profile_explicit_cant_width.sli"
        "${OPENBUS_FIXTURES}/map_spline_profile_no_collision.sli")
openbus_add_probe(OpenBusMapTrafficRuntimeProbe MapTrafficRuntimeProbe.cpp LIBS openbus_map)
openbus_add_probe(OpenBusTrafficDebugDirectionProbe TrafficDebugDirectionProbe.cpp LIBS openbus_map)
add_executable(OpenBusMapTrafficWorkerProbe MapTrafficWorkerProbe.cpp
    "${OPENBUS_SOURCE_ROOT}/app/MapTrafficWorker.cpp")
target_include_directories(OpenBusMapTrafficWorkerProbe PRIVATE "${OPENBUS_SOURCE_ROOT}")
target_link_libraries(OpenBusMapTrafficWorkerProbe PRIVATE openbus_map)
add_test(NAME OpenBusMapTrafficWorkerProbe COMMAND OpenBusMapTrafficWorkerProbe)
add_test(NAME OpenBusMapTrafficIncrementalLaneNetworkProbe
    COMMAND OpenBusMapTrafficRuntimeProbe --incremental-lane-network)
add_test(NAME OpenBusMapTrafficTJunctionProbe
    COMMAND OpenBusMapTrafficRuntimeProbe --t-junction)
add_test(NAME OpenBusMapTrafficPriorityOrderProbe
    COMMAND OpenBusMapTrafficRuntimeProbe --authored-priority-order)
add_test(NAME OpenBusMapPassengerServiceProbe
    COMMAND OpenBusMapTrafficRuntimeProbe --passenger-service)

# Audio
openbus_add_probe(OpenBusSoundEngineProbe SoundEngineProbe.cpp LIBS openbus_audio)

# Scripting
openbus_add_probe(OpenBusScriptSchedulerProbe ScriptSchedulerProbe.cpp LIBS openbus_core)
foreach(probe IN ITEMS ScriptTextureRuntimeProbe MapPlacementLabelProbe ScriptInputRuntimeProbe)
    openbus_add_probe(OpenBus${probe} ${probe}.cpp LIBS openbus_scripting openbus_osc)
    openbus_copy_lua_runtime(OpenBus${probe})
endforeach()

# Render (header-only probes need only the include root)
openbus_add_probe(OpenBusImGuiLayerProbe ImGuiLayerProbe.cpp LIBS openbus_imgui)
target_sources(OpenBusImGuiLayerProbe PRIVATE "${OPENBUS_SOURCE_ROOT}/app/ImGuiLayer.cpp")
target_include_directories(OpenBusImGuiLayerProbe PRIVATE "${OPENBUS_SOURCE_ROOT}")
set_tests_properties(OpenBusImGuiLayerProbe PROPERTIES SKIP_RETURN_CODE 77)
openbus_add_probe(OpenBusCameraMathProbe CameraMathProbe.cpp LIBS openbus_render)
openbus_add_probe(OpenBusHumanGaitProbe HumanGaitProbe.cpp LIBS openbus_render)
openbus_add_probe(OpenBusAssetRequestManagerProbe AssetRequestManagerProbe.cpp LIBS openbus_render)
openbus_add_probe(OpenBusReflectionSettingsProbe ReflectionSettingsProbe.cpp
    LIBS openbus_render NO_TEST)
foreach(mode IN ITEMS default enabled disabled)
    add_test(NAME OpenBusReflectionSettingsProbe.${mode}
        COMMAND OpenBusReflectionSettingsProbe ${mode})
endforeach()
openbus_add_probe(OpenBusComputeCullingProbe ComputeCullingProbe.cpp LIBS openbus_render)
add_test(NAME OpenBusComputeCullingGpuProbe COMMAND OpenBusComputeCullingProbe --gpu)
set_tests_properties(OpenBusComputeCullingGpuProbe PROPERTIES SKIP_RETURN_CODE 77)
openbus_add_probe(OpenBusViewpointProbe ViewpointProbe.cpp LIBS openbus_core)
openbus_add_probe(OpenBusMaterialRenderingProbe MaterialRenderingProbe.cpp LIBS openbus_render)
openbus_add_probe(OpenBusMapOcclusionRenderingProbe MapOcclusionRenderingProbe.cpp
    LIBS openbus_render)
set_tests_properties(OpenBusMapOcclusionRenderingProbe PROPERTIES SKIP_RETURN_CODE 77)
openbus_add_probe(OpenBusMapTrafficLightRenderingProbe MapTrafficLightRenderingProbe.cpp
    LIBS openbus_render)
add_test(NAME OpenBusMapTrafficLightRenderingProbe.conditional
    COMMAND OpenBusMapTrafficLightRenderingProbe --conditional)
set_tests_properties(OpenBusMapTrafficLightRenderingProbe
    OpenBusMapTrafficLightRenderingProbe.conditional PROPERTIES
    SKIP_RETURN_CODE 77
    ENVIRONMENT "OPENBUS_SCRIPT_BACKEND=native;OPENBUS_CULLING_BACKEND=cpu;OPENBUS_MIN_OBJECT_SIZE_MAIN_VIEW_PERCENT=0;OPENBUS_MIN_OBJECT_SIZE_MIRRORS_PERCENT=0")
foreach(backend IN ITEMS cpu compute-indirect)
    foreach(mode IN ITEMS ordinary conditional)
        if(mode STREQUAL "conditional")
            add_test(NAME OpenBusMapTrafficLightRenderingProbe.${backend}.${mode}
                COMMAND OpenBusMapTrafficLightRenderingProbe --conditional)
        else()
            add_test(NAME OpenBusMapTrafficLightRenderingProbe.${backend}.${mode}
                COMMAND OpenBusMapTrafficLightRenderingProbe)
        endif()
        set_tests_properties(OpenBusMapTrafficLightRenderingProbe.${backend}.${mode}
            PROPERTIES SKIP_RETURN_CODE 77
            ENVIRONMENT "OPENBUS_SCRIPT_BACKEND=native;OPENBUS_CULLING_BACKEND=${backend};OPENBUS_OCCLUSION_CULLING=queries;OPENBUS_OCCLUSION_TRANSPARENT=1;OPENBUS_OCCLUSION_MAP_GEOMETRY=1;OPENBUS_MIN_OBJECT_SIZE_MAIN_VIEW_PERCENT=0;OPENBUS_MIN_OBJECT_SIZE_MIRRORS_PERCENT=0")
    endforeach()
endforeach()
set_tests_properties(OpenBusMaterialRenderingProbe PROPERTIES
    SKIP_RETURN_CODE 77
    ENVIRONMENT "OPENBUS_REFLECTION_SIZE=1024;OPENBUS_REFLECTION_INTERVAL=1;OPENBUS_REFLECTION_MAX_FPS=0")
openbus_add_probe(OpenBusVehicleDrawSortingProbe VehicleDrawSortingProbe.cpp
    LIBS openbus_render)
openbus_add_probe(OpenBusVehicleIndirectReuseProbe VehicleIndirectReuseProbe.cpp
    LIBS openbus_audio openbus_input openbus_map openbus_physics openbus_render openbus_scripting)
target_sources(OpenBusVehicleIndirectReuseProbe PRIVATE
    "${OPENBUS_SOURCE_ROOT}/app/vehicle/Vehicle.cpp"
    "${OPENBUS_SOURCE_ROOT}/app/vehicle/VehicleAnimation.cpp"
    "${OPENBUS_SOURCE_ROOT}/app/vehicle/VehicleDefinition.cpp"
    "${OPENBUS_SOURCE_ROOT}/app/vehicle/VehicleDraw.cpp"
    "${OPENBUS_SOURCE_ROOT}/app/vehicle/VehicleInteraction.cpp"
    "${OPENBUS_SOURCE_ROOT}/app/vehicle/VehicleMaterials.cpp"
    "${OPENBUS_SOURCE_ROOT}/app/vehicle/VehicleMeshLoading.cpp"
    "${OPENBUS_SOURCE_ROOT}/app/vehicle/VehicleTextures.cpp"
    "${OPENBUS_SOURCE_ROOT}/app/vehicle/VehicleVisibility.cpp")
openbus_copy_lua_runtime(OpenBusVehicleIndirectReuseProbe)
set_tests_properties(OpenBusVehicleIndirectReuseProbe PROPERTIES SKIP_RETURN_CODE 77)
add_test(NAME OpenBusVehicleTextureLookupProbe
    COMMAND OpenBusVehicleIndirectReuseProbe --texture-lookup)
openbus_add_probe(OpenBusVehiclePartLoadingProbe VehiclePartLoadingProbe.cpp
    LIBS openbus_render)
openbus_add_probe(OpenBusVehicleMaterialSelectionProbe VehicleMaterialSelectionProbe.cpp
    LIBS openbus_render)
openbus_add_probe(OpenBusWeatherEffectsProbe WeatherEffectsProbe.cpp LIBS openbus_render)
openbus_add_probe(OpenBusTextureAddressModeProbe TextureAddressModeProbe.cpp
    LIBS openbus_render)
openbus_add_probe(OpenBusStaticPrimitiveBufferProbe StaticPrimitiveBufferProbe.cpp)
target_include_directories(OpenBusStaticPrimitiveBufferProbe PRIVATE "${OPENBUS_SOURCE_ROOT}")

if(WIN32)
    find_program(OPENBUS_POWERSHELL_EXECUTABLE NAMES pwsh powershell)
    if(OPENBUS_POWERSHELL_EXECUTABLE)
        add_test(NAME OpenBusLoadingBenchmarkProbe
            COMMAND "${OPENBUS_POWERSHELL_EXECUTABLE}" -NoProfile -File
                "${CMAKE_CURRENT_SOURCE_DIR}/BenchmarkLoadingProbe.ps1")
    endif()
endif()

get_property(probeTargets DIRECTORY PROPERTY BUILDSYSTEM_TARGETS)
add_custom_target(check-probes
    COMMAND "${CMAKE_CTEST_COMMAND}" --test-dir "${PROJECT_BINARY_DIR}"
        -C "$<CONFIG>" --output-on-failure
    DEPENDS ${probeTargets}
    USES_TERMINAL
    VERBATIM
)
