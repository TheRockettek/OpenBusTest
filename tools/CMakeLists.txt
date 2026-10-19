# Developer tools built alongside the game.

add_executable(OpenBusConfigAudit config_audit.cpp)
target_link_libraries(OpenBusConfigAudit PRIVATE openbus_config openbus_osc)

add_executable(obj_bundle obj_bundle.cpp)
target_link_libraries(obj_bundle PRIVATE openbus_assets)

add_executable(uno3d_converter "${PROJECT_SOURCE_DIR}/uno3d/o3d_to_obj.cpp")
target_link_libraries(uno3d_converter PRIVATE openbus_core openbus_third_party_headers)

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
