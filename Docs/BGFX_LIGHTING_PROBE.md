# bgfx GLSL lighting probe

The production `OpenBus` executable uses Diligent Vulkan. This opt-in probe remains a separate, small bgfx-owned OpenGL rendering path that verifies the native GLFW window handoff, bgfx frame lifecycle, shader compilation, vertex buffers, transforms, and ambient-plus-directional GLSL lighting; it is not part of the production renderer.

## Build prerequisites

The configured Windows vcpkg triplet must contain bgfx and the shader tools feature. Install with:

`C:\vcpkg\vcpkg.exe install "bgfx[tools]:x64-windows"`

The `windows-clang-bgfx` CMake preset enables the probe in a separate `build-bgfx` directory.

## Build and run

Select the `windows-clang-bgfx` configure preset in CMake Tools and build `OpenBusBgfxLightingProbe`. The build compiles `shaders/bgfx/vs_lit_cube.sc` and `fs_lit_cube.sc` to Windows GLSL 1.50 shader binaries using bgfx `shaderc`, then copies them beside the executable. GLSL 1.50 is supported by the project's OpenGL 3.3 core fallback.

Run `build-bgfx/OpenBusBgfxLightingProbe.exe`. It opens a GLFW window with no client graphics API; bgfx owns the OpenGL context and presents frames. Close the window to exit.

## Scope

This probe deliberately renders a lit calibration cube, not bus geometry. It is an isolated legacy experiment and does not participate in production scene rendering. Do not mix direct OpenGL draws into the probe's bgfx-owned frame.
