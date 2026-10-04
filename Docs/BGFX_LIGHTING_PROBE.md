# bgfx GLSL lighting probe

The existing `OpenBus` executable remains on its OpenGL renderer. This opt-in probe is a separate, small bgfx-owned rendering path that verifies the native GLFW window handoff, bgfx frame lifecycle, shader compilation, vertex buffers, transforms, and ambient-plus-directional GLSL lighting before model batches are migrated.

## Build prerequisites

The configured Windows vcpkg triplet must contain bgfx and the shader tools feature. Install with:

`C:\vcpkg\vcpkg.exe install "bgfx[tools]:x64-windows"`

The `windows-clang-bgfx` CMake preset enables the probe in a separate `build-bgfx` directory, leaving the regular `build-ode` OpenGL build unchanged.

## Build and run

Select the `windows-clang-bgfx` configure preset in CMake Tools and build `OpenBusBgfxLightingProbe`. The build compiles `shaders/bgfx/vs_lit_cube.sc` and `fs_lit_cube.sc` to Windows GLSL 1.50 shader binaries using bgfx `shaderc`, then copies them beside the executable. GLSL 1.50 is supported by the project's OpenGL 3.3 baseline.

Run `build-bgfx/OpenBusBgfxLightingProbe.exe`. It opens a GLFW window with no client graphics API; bgfx owns the OpenGL context and presents frames. Close the window to exit.

## Scope

This probe deliberately renders a lit calibration cube, not bus geometry. The production OpenBus renderer, asset uploads, reflections, material variants, and capture still use OpenGL. The next migration slice is an opaque bus batch using its CPU-side position/UV/normal data, a base texture, and the same ambient/directional lighting. Do not mix direct OpenGL draws into the probe's bgfx-owned frame.
