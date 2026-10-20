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
openbus_add_probe(OpenBusVariablesProbe VariablesProbe.cpp LIBS openbus_core)
openbus_add_probe(OpenBusConfigurationParserProbe ConfigurationParserProbe.cpp
    LIBS openbus_config)
openbus_add_probe(OpenBusModelConfigProbe ModelConfigProbe.cpp LIBS openbus_config openbus_osc)
openbus_add_probe(OpenBusVehicleConfigProbe VehicleConfigProbe.cpp
    LIBS openbus_config openbus_osc)

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

# Audio
openbus_add_probe(OpenBusSoundEngineProbe SoundEngineProbe.cpp LIBS openbus_audio)

# Scripting
foreach(probe IN ITEMS ScriptTextureRuntimeProbe MapPlacementLabelProbe ScriptInputRuntimeProbe)
    openbus_add_probe(OpenBus${probe} ${probe}.cpp LIBS openbus_scripting openbus_osc)
    openbus_copy_lua_runtime(OpenBus${probe})
endforeach()

# Render (header-only probes need only the include root)
openbus_add_probe(OpenBusCameraMathProbe CameraMathProbe.cpp LIBS openbus_render)
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
set_tests_properties(OpenBusMaterialRenderingProbe PROPERTIES SKIP_RETURN_CODE 77)
openbus_add_probe(OpenBusStaticPrimitiveBufferProbe StaticPrimitiveBufferProbe.cpp)
target_include_directories(OpenBusStaticPrimitiveBufferProbe PRIVATE "${OPENBUS_SOURCE_ROOT}")
