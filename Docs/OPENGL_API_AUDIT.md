# OpenGL API/Profile Audit

Audit date: 2026-10-04

## Result

No active OpenGL compatibility-profile or fixed-function API calls were found in the source tree. In particular, the renderer does not call `glBegin`/`glEnd`, OpenGL matrix-stack or transform functions, immediate-mode vertex/color/normal/texture-coordinate functions, client-state arrays, fixed-function lighting/material functions, or texture-environment functions.

The application requests an OpenGL 3.3 **core** context in `src/RenderLoop.cpp` where it sets `GLFW_CONTEXT_VERSION_MAJOR`, `GLFW_CONTEXT_VERSION_MINOR`, and `GLFW_OPENGL_PROFILE` to `GLFW_OPENGL_CORE_PROFILE` before `glfwCreateWindow`. On Apple it also requests a forward-compatible context. No compatibility context is requested.

## Places that can look like compatibility API

- `src/CameraMath.cpp`: `pushMatrix`, `popMatrix`, `translate`, `rotate`, `scale`, and `multiplyMatrix` operate on the application's `std::vector<Matrix4>` and CPU-side matrix state. They do not invoke OpenGL's deprecated matrix stack or transform API. `src/CoreRenderer.cpp` uploads those matrices to shader uniforms.
- `src/CoreRenderer.cpp`: GLSL shaders use `#version 330 core`; vertex input is configured through VAOs/vertex attributes and submitted with `glDrawArrays`. The `pgl*` calls are loaded modern OpenGL entry points, not compatibility wrappers.
- `src/RenderLoop.cpp`: `glEnable`/`glDisable` calls for depth testing, blending, culling, and polygon offset are GL state controls supported by the requested core profile; they are not fixed-function lighting or texture processing.
- `src/RenderLoop.cpp` has compatibility-era enum fallback definitions near the top of the file (`GL_TEXTURE_ENV`, `GL_TEXTURE_ENV_MODE`, `GL_COMBINE`, and related tokens). They are constants, not function calls, and have no use sites in `src`; distinguish these stale definitions from active compatibility API dependencies.

## Scope and maintenance

This is a source search/audit, not a runtime API trace. Re-run the search if renderer/context creation code changes. Keep the core-profile request; do not switch to a compatibility context to accommodate CPU matrix helper names or unused enum definitions. The cleanup candidate is removal of the unused compatibility-era enum definitions after confirming no generated/platform-specific source depends on them.
