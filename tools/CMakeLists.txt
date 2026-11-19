# Developer tools built alongside the game.

add_executable(OpenBusConfigAudit config_audit.cpp)
target_link_libraries(OpenBusConfigAudit PRIVATE openbus_config openbus_map openbus_osc)

add_executable(OpenBusMapAssetCheck map_asset_check.cpp)
target_link_libraries(OpenBusMapAssetCheck PRIVATE openbus_map)

add_executable(obj_bundle obj_bundle.cpp)
target_link_libraries(obj_bundle PRIVATE openbus_assets)

add_executable(uno3d_converter "${PROJECT_SOURCE_DIR}/uno3d/o3d_to_obj.cpp")
target_link_libraries(uno3d_converter PRIVATE openbus_core openbus_third_party_headers)

add_executable(OpenBusLightingBenchmark LightingBenchmark.cpp)
target_link_libraries(OpenBusLightingBenchmark PRIVATE openbus_render_legacy_gl OpenBus::GLFW)
add_executable(OpenBusSpotlightSelectionBenchmark SpotlightSelectionBenchmark.cpp)
target_include_directories(OpenBusSpotlightSelectionBenchmark PRIVATE "${OPENBUS_SOURCE_ROOT}")
add_test(NAME OpenBusSpotlightSelectionBenchmark.smoke
    COMMAND OpenBusSpotlightSelectionBenchmark 3000)
add_test(NAME OpenBusSpotlightSelectionBenchmark.invalid
    COMMAND OpenBusSpotlightSelectionBenchmark 0)
set_tests_properties(OpenBusSpotlightSelectionBenchmark.invalid PROPERTIES WILL_FAIL TRUE)
add_test(NAME OpenBusLightingBenchmark.smoke
    COMMAND OpenBusLightingBenchmark --lights 16 --width 32 --height 32 --warmup 1 --frames 2)
add_test(NAME OpenBusLightingBenchmark.large
    COMMAND OpenBusLightingBenchmark --lights 3000 --width 32 --height 32 --warmup 1 --frames 2)
add_test(NAME OpenBusLightingBenchmark.direct
    COMMAND OpenBusLightingBenchmark --lights 16 --storage direct --width 32 --height 32
        --warmup 1 --frames 2)
add_test(NAME OpenBusLightingBenchmark.mixed
    COMMAND OpenBusLightingBenchmark --lights 64 --vehicle-lights 16 --width 32 --height 32
        --warmup 1 --frames 2)
add_test(NAME OpenBusLightingBenchmark.stationary
    COMMAND OpenBusLightingBenchmark --lights 16 --vehicle-lights 8 --move-vehicles 0
        --width 32 --height 32 --warmup 1 --frames 2)
add_test(NAME OpenBusLightingBenchmark.sparse
    COMMAND OpenBusLightingBenchmark --lights 64 --vehicle-lights 16 --layout sparse
        --width 32 --height 32 --warmup 1 --frames 2)
add_test(NAME OpenBusLightingBenchmark.sparse-prefix
    COMMAND OpenBusLightingBenchmark --lights 500 --move-map-lights 1 --layout sparse-prefix
        --width 32 --height 32 --warmup 1 --frames 2)
add_test(NAME OpenBusLightingBenchmark.moving-map
    COMMAND OpenBusLightingBenchmark --lights 64 --vehicle-lights 16 --move-map-lights 1
        --layout sparse --width 32 --height 32 --warmup 1 --frames 2)
add_test(NAME OpenBusLightingBenchmark.moving-map-direct
    COMMAND OpenBusLightingBenchmark --lights 64 --move-map-lights 1 --storage direct
        --width 32 --height 32 --warmup 1 --frames 2)
set_tests_properties(OpenBusLightingBenchmark.smoke OpenBusLightingBenchmark.large
    OpenBusLightingBenchmark.direct OpenBusLightingBenchmark.mixed
    OpenBusLightingBenchmark.stationary OpenBusLightingBenchmark.sparse
    OpenBusLightingBenchmark.sparse-prefix
    OpenBusLightingBenchmark.moving-map OpenBusLightingBenchmark.moving-map-direct
    PROPERTIES SKIP_RETURN_CODE 77)
add_test(NAME OpenBusLightingBenchmark.invalid
    COMMAND OpenBusLightingBenchmark --lights -1)
set_tests_properties(OpenBusLightingBenchmark.invalid PROPERTIES WILL_FAIL TRUE)
add_test(NAME OpenBusLightingBenchmark.map-limit
    COMMAND OpenBusLightingBenchmark --lights 16 --width 32 --height 32 --warmup 1 --frames 2)
set_tests_properties(OpenBusLightingBenchmark.map-limit PROPERTIES
    ENVIRONMENT "OPENBUS_MAP_LIGHT_LIMIT=0" WILL_FAIL TRUE SKIP_RETURN_CODE 77)
add_test(NAME OpenBusLightingBenchmark.vehicle-limit
    COMMAND OpenBusLightingBenchmark --lights 16 --vehicle-lights 8 --width 32 --height 32
        --warmup 1 --frames 2)
set_tests_properties(OpenBusLightingBenchmark.vehicle-limit PROPERTIES
    ENVIRONMENT "OPENBUS_VEHICLE_SPOTLIGHT_LIMIT=0" WILL_FAIL TRUE SKIP_RETURN_CODE 77)

if(OPENBUS_BUILD_BGFX_LIGHTING_PROBE)
    if(NOT WIN32)
        message(FATAL_ERROR "The bgfx lighting probe currently uses GLFW's Win32 native handle.")
    endif()

    set(OPENBUS_BGFX_SHADER_SOURCE_DIR "${PROJECT_SOURCE_DIR}/shaders/bgfx")
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
    add_executable(OpenBusBgfxLightingProbe BgfxLightingProbe.cpp)
    add_dependencies(OpenBusBgfxLightingProbe OpenBusBgfxShaders)
    target_link_libraries(OpenBusBgfxLightingProbe PRIVATE bgfx::bgfx OpenBus::GLFW)
    target_compile_features(OpenBusBgfxLightingProbe PRIVATE cxx_std_23)
    add_custom_command(TARGET OpenBusBgfxLightingProbe POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${OPENBUS_BGFX_SHADER_OUTPUT_DIR}"
            "$<TARGET_FILE_DIR:OpenBusBgfxLightingProbe>/bgfx-shaders"
        VERBATIM)
endif()

if(OPENBUS_BUILD_DILIGENT_VULKAN_PROBE OR OPENBUS_BUILD_DILIGENT_RENDERER)
    add_executable(OpenBusDiligentFrameRendererProbe
        DiligentFrameRendererProbe.cpp
        DiligentVehicleSourceBridgeProbe.cpp
    )
    target_link_libraries(OpenBusDiligentFrameRendererProbe PRIVATE
        openbus_diligent_app
        OpenBus::GLFW
    )
    target_compile_features(OpenBusDiligentFrameRendererProbe PRIVATE cxx_std_23)
    add_test(NAME OpenBusDiligentFrameRendererProbe COMMAND OpenBusDiligentFrameRendererProbe)
    set_tests_properties(OpenBusDiligentFrameRendererProbe PROPERTIES SKIP_RETURN_CODE 77)
endif()

if(OPENBUS_BUILD_DILIGENT_VULKAN_PROBE)
    add_executable(OpenBusDiligentVulkanProbe DiligentVulkanProbe.cpp)
    target_link_libraries(OpenBusDiligentVulkanProbe PRIVATE
        Diligent-Common
        openbus_assets
        openbus_diligent_vulkan
        openbus_map
        openbus_scenery_preparation
        openbus_diligent_render
        openbus_diligent_ui
        openbus_diligent_app
        openbus_scripting
        OpenBus::GLFW
    )
    target_sources(OpenBusDiligentVulkanProbe PRIVATE
        "${PROJECT_SOURCE_DIR}/tests/DiligentColorPresent.cpp"
        "${PROJECT_SOURCE_DIR}/tests/DiligentImGuiRendering.cpp"
        "${PROJECT_SOURCE_DIR}/tests/DiligentPrimitiveRendering.cpp"
        "${PROJECT_SOURCE_DIR}/tests/DiligentTexturedMeshRendering.cpp"
        "${PROJECT_SOURCE_DIR}/tests/DiligentMotionDataRendering.cpp")
    target_include_directories(OpenBusDiligentVulkanProbe PRIVATE
        "${diligentcore_SOURCE_DIR}"
        "${diligentcore_SOURCE_DIR}/ThirdParty/Vulkan-Headers/include"
        "${PROJECT_SOURCE_DIR}/tests")
    target_compile_definitions(OpenBusDiligentVulkanProbe PRIVATE VK_USE_PLATFORM_WIN32_KHR=1)
    target_compile_features(OpenBusDiligentVulkanProbe PRIVATE cxx_std_23)
    add_test(NAME OpenBusDiligentVulkanProbe COMMAND OpenBusDiligentVulkanProbe
        "${PROJECT_SOURCE_DIR}/uno3d/SitzeU.o3d")
    set_tests_properties(OpenBusDiligentVulkanProbe PROPERTIES
        SKIP_RETURN_CODE 77
        ENVIRONMENT "OPENBUS_REFLECTION_AMBIENT_FLOOR=0.37"
    )
    add_test(NAME OpenBusDiligentVulkanProbe.invalid-arguments
        COMMAND OpenBusDiligentVulkanProbe ignored extra)
    set_tests_properties(OpenBusDiligentVulkanProbe.invalid-arguments PROPERTIES WILL_FAIL TRUE)
endif()
