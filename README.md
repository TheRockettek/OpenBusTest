# OpenBus

OpenBus is a C++ bus simulator prototype built on the
[Open Dynamics Engine (ODE)](https://www.ode.org/). This first milestone is a
headless physics sandbox: it creates a simplified bus chassis, four wheels, a
ground plane, and advances the vehicle through ODE's collision and constraint
solver.

## Prerequisites

- CMake 3.20 or newer
- A C++17 compiler
- ODE
- GLFW and OpenGL

On Windows with vcpkg:

```powershell
vcpkg install ode:x64-windows glfw3:x64-windows
```

If `glfw3` is not available through `CMAKE_PREFIX_PATH`, the build now falls
back to downloading GLFW 3.4 during CMake configure. This still requires
internet access at configure time.

## Configure and build

Use a new build directory so old artifacts from previous experiments are not
reused:

```powershell
cmake -S . -B build-ode -DCMAKE_TOOLCHAIN_FILE=C:/vcpkg/scripts/buildsystems/vcpkg.cmake
cmake --build build-ode --config Release
```

## Linting

Install LLVM so `clang-tidy` is available on `PATH`, then run:

```powershell
.\lint.bat -DCMAKE_TOOLCHAIN_FILE=C:/vcpkg/scripts/buildsystems/vcpkg.cmake
```

This uses the separate `build-lint` directory and enables the project’s
`.clang-tidy` checks without changing the normal `build-ode` configuration.

To format the project’s C++ files with `clang-format`, install LLVM and run:

```powershell
.\format.bat
```

Run the prototype:

```powershell
.\build-ode\Release\OpenBus.exe
```

The executable opens a simple OpenGL preview window while the physics runs at
the configured fixed rate. While running, it writes a structured diagnostics
file to `OpenBus_physics.json`, sampled five times per second. The JSON contains
timing and control inputs, chassis position/velocity/yaw, and nested axle and
wheel positions, velocities, steering angles, suspension compression, wheel
angular speed, slip ratio, tyre forces, friction utilization, wheelspin, and
skid state.
Key press and release transitions for `W`, `A`, `S`, and `D` are buffered into
the next sample under `key_events`, including their wall-clock timestamps.

## Current physics model

- ODE world with gravity and a hash-space collision broad phase
- Box chassis with configurable mass and dimensions
- Configurable axle layout, including arbitrary axle count, track width,
  steering, and driven-axle flags
- ODE contact joints for chassis/ground and wheel/ground collisions
- Per-wheel ODE bodies with cylinder geometry, suspension slider joints,
  steering hinges, and wheel rotation hinges
- Per-corner spring-damper suspension, anti-roll forces, tyre friction,
  braking, rolling resistance, aerodynamic drag, steering, bounded chassis
  attitude stabilization, and automatic gearing
- Wheel torque, wheelspin, and skid telemetry derived from the ODE wheel bodies
- Fixed-rate physics (60 Hz by default) with a bounded catch-up budget; if a
  frame exceeds that budget, excess accumulated time is dropped to avoid a
  spiral of death

`BusConfiguration::lionCity12()` remains available as a two-axle example. The
default simulation uses `BusConfiguration::manDl05()` with three axles aligned
to the supplied MAN DL05 model. Add more entries to an `axles` vector for
other tri-axle or multi-axle vehicles. The
`articulated` flag is retained in the configuration so articulated buses can
share the same vehicle description; articulated multi-body sections will be
added as a separate ODE body/joint layer.

## Renderer controls

- `W`: throttle
- `S`: brake
- `A` / `D`: steer
- `0`: outside orbit camera
- `1`: driver viewpoint
- `2`-`9`: additional passenger, mirror, and exterior viewports
- Default camera: MAN DL05 driver viewpoint (`1`)
- Hold middle mouse and drag: orbit outside view `0`, or look around the
  selected non-third-person viewport
- Scroll up/down: zoom in/out
- VSync is enabled by default; set `OPENBUS_VSYNC=0`, `off`, or `false` before
  launching to run above the display refresh cadence
- Optional texture downscaling: set `OPENBUS_TEXTURE_SCALE=0.5` before
  launching to upload half-resolution textures; valid range is `0.25`-`1.0`
- The window title reports the currently visible/rendered triangle count
- Model display parts use conservative frustum culling, CFG-driven LOD
  selection, and sorted opaque/transparent batches to reduce off-screen work
  and state churn. Set `OPENBUS_FRUSTUM_CULLING=0` to disable culling for
  debugging. Opaque parts are front-to-back and transparent parts
  back-to-front; this is not hardware occlusion-query culling.
- Model geometry is uploaded to OpenGL vertex buffer objects (VBOs) and drawn
  with `glDrawArrays`; compatibility-profile VBO entry points are loaded at
  runtime.
- Window and door glass remains material-driven in the VBO path; the temporary
  flat glass fallback is no longer used for normal model materials.
- Normal MAN DL05 wheel meshes are removed from the static model pass and
  rendered from the live ODE wheel poses, including suspension, steering, and
  wheel rotation; the wire wheel fallback is used only when an asset is absent.
- The driver camera follows the ODE chassis position, pitch, roll, and heading
- When `MAN_DL05/Model/DL05.cfg` is available, the highest-detail `[LOD] 0.25`
  model block is rendered with driver-viewpoint mesh filtering
- OBJ texture coordinates follow the converter's existing V-axis conversion
  plus the final OpenGL image-origin correction;
- MAN DL05 OBJ lateral coordinates are mirrored into the simulator convention
  so the modeled right-side exit remains on the vehicle's right
  TGA, BMP, PNG/JPEG, and DXT1/DXT5 DDS texture data are decoded for model
  materials
- OBJ `mtllib`/`usemtl` groups are rendered with their material textures;
  converter-generated texture extensions are resolved by filename stem
- Window meshes use a depth-write-disabled, controlled translucent glass pass
  until the original script-driven OMSI material stack is implemented
- Script-controlled `WerbungInnen` advertising overlays are skipped until
  texture-state scripts are implemented, preventing opaque white overlays from
  covering the glass
- Door meshes retain their opaque panels while `fenster_*` door glass groups
  render through a separate static translucent pass
- Opaque DXT1 textures preserve color-index 3 as opaque, preventing roof
  materials from becoming transparent and exposing interior geometry
- Texture lookup diagnostics are written to `OpenBus_textures.log`; entries
  are deduplicated and record loaded files, missing source files, and decode
  failures
- The visible chassis and wheels use the live ODE body position and rotation,
  including chassis pitch/roll, suspension travel, steering, and wheel rotation
- A red three-axis marker shows the aggregate mass-weighted center of gravity
  at the configured 1.0 m bus CG height
- The configured 11,500 kg bus mass includes the wheel and suspension bodies;
  the chassis body mass is reduced accordingly so the complete ODE assembly
  totals 11,500 kg
- Close the window to exit

The ground uses a world-space grid with highlighted ten-metre lines.

## OMSI bus-file reference

The supplied OMSI examples are summarized in
[Docs/OMSI_BUS_FORMAT_REFERENCE.md](C:/Users/blane/Desktop/OpenBusTest/Docs/OMSI_BUS_FORMAT_REFERENCE.md).
It documents parser syntax, count-prefixed and terminated blocks, identity,
assets, scripts, cameras, physical data, axles, suspension, couplings, AI,
rail/trolley sections, and a minimal bus-file skeleton.

The CFG corpus is indexed in
[Docs/OMSI_CFG_FORMAT_REFERENCE.md](C:/Users/blane/Desktop/OpenBusTest/Docs/OMSI_CFG_FORMAT_REFERENCE.md).
Bus-only model construction is documented separately in
[Docs/OMSI_CFG_BUS_MODEL_REFERENCE.md](C:/Users/blane/Desktop/OpenBusTest/Docs/OMSI_CFG_BUS_MODEL_REFERENCE.md);
general surfaces, passenger cabins, paths, sounds, environment, AI/network,
controls, and metadata remain in
[Docs/OMSI_CFG_GENERAL_REFERENCE.md](C:/Users/blane/Desktop/OpenBusTest/Docs/OMSI_CFG_GENERAL_REFERENCE.md).
The original CFG files remain in place so their relative paths continue to
work.
