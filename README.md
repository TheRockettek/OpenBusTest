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

On Debian or Ubuntu, install the native development packages:

```bash
sudo apt install build-essential cmake libode-dev libglfw3-dev libglu1-mesa-dev
```

The Linux build uses the system GLFW/ODE packages when available and falls
back to downloading GLFW during CMake configuration when GLFW is not found.

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

On Linux, use a native build directory and the provided shell helpers:

```bash
chmod +x run.sh profile.sh screenshot.sh convert.sh
./run.sh
```

The helpers use `build-linux` by default. Set `BUILD_DIR` to choose another
build directory, and set `OPENBUS_VEHICLE=e400` to run the E400 model.

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
the configured fixed rate.

The Windows build writes a symbolized `game.crash` report with exception
details, registers, module names, and source lines when matching PDB files are
available. It also writes `game.crash.dmp`, which can be opened in Visual
Studio or WinDbg with the matching executable and PDB. The Linux build writes
a `game.crash` stack trace for fatal POSIX signals and unexpected termination.
Build with debug symbols when detailed function names are needed, for example
with `-DCMAKE_BUILD_TYPE=Debug`.

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

Vehicle physics is loaded from the selected OMSI `.bus` file. The default MAN
DL05 file is `MAN_DL05/MAN_DL05.bus`, and the supplied E400 MMC file is loaded
for the E400 configuration. Vehicle values are not duplicated in C++.
Articulated vehicles can set the `articulated` flag in the file; articulated
multi-body sections will be added as a separate ODE body/joint layer.
The parsed configuration is written to `OpenBus_configuration.json` at startup.
Parsed model parts, materials, visibility variables, animation variables, and
wheel bindings are written to `OpenBus_model_configuration.json` as well. For
parts with `[animparent]`, `parent_animations` shows the variables inherited
from the referenced mesh, which makes door-glass control variables visible.

The model variable store initializes OMSI-style system variables with the local
system date at midday: `Time` is `43200`, `Day`, `Month`, and `Year` come from
the current date, and `DayOfYear` is the zero-based day index. Defaults include
`NoSound=1`, `Pause=0`, `PrecipType=0`, `PrecipRate=0`,
`Weather_Temperature=20`, `Weather_AbsHum=10`, `Envir_Brightness=1`,
`AutoClutch=0`, and `SunAlt=60`.
`Timegap`, `GetTime`, `mouse_x`, and `mouse_y` are updated from the renderer
each frame.

Set `OPENBUS_BUS_CONFIG` to an OMSI `.bus` file to load physics data at startup
instead of selecting a built-in configuration, for example:

```powershell
$env:OPENBUS_BUS_CONFIG = "SP_E400MMC/E400MMC_ADL_10.9m_Voith_LowHeight.bus"
./build-ode/Release/OpenBus.exe
```

The loader uses `[mass]`, `[boundingbox]`, `[schwerpunkt]`, and `[newachse]`
records, including axle spring, damper, load, and driven flags. The
`bodyHalfLength`, `bodyHalfWidth`, `bodyHalfHeight`, collision dimensions, and
collision offsets are derived from `[boundingbox]`. Because standard OMSI
records do not define wheel thickness or articulation, each file also supplies
`[openbus_wheel_half_width]` and `[openbus_articulated]`. Axle steering is
derived from `Axle_Steering_X_L/R` variables in the referenced model.cfg.

## Renderer controls

- `W`: throttle
- `S`: brake
- `A` / `D`: steer
- `0`: outside orbit camera
- `1`-`9`: cameras from the selected `.bus` file in driver/passenger order
- Left/right arrow: step through driver/passenger cameras in file order
- Default camera: the driver camera selected by `[set_camera_std]`
- Hold middle mouse and drag: orbit outside view `0`, or look around the
  selected non-third-person viewport
- Hold right mouse and drag vertically: adjust FOV in configured views, or
  move closer/farther in the outside orbit view
- Scroll up/down: zoom in/out
- VSync is enabled by default; set `OPENBUS_VSYNC=0`, `off`, or `false` before
  launching to run above the display refresh cadence
- Reflection mirrors render at `256x256` by default. Set
  `OPENBUS_REFLECTION_SIZE=256`, `512`, or `1024` to choose the square mirror
  target resolution.
- Mirrors update every frame by default. Set `OPENBUS_REFLECTION_INTERVAL=4`
  to update them once every four rendered frames; values below `1` are treated
  as `1`.
- Mirror transparency is rendered by default. Set
  `OPENBUS_REFLECTION_TRANSPARENT=0` to render opaque geometry only and reduce
  mirror workload when transparent details are not needed.
- Scripts normally follow the render rate. Set `OPENBUS_SCRIPT_HZ=30` to run
  vehicle scripts at 30 Hz while rendering continues at the display rate;
  fixed-rate script ticks receive `Timegap=1/30` and catch up for short frame
  hitches with a bounded tick budget.
- Profiling retains at most `1,000,000` events by default. Set
  `OPENBUS_TRACE_MAX_EVENTS=250000` or another positive value to lower the
  memory ceiling. Trace events are flushed incrementally to the JSON file while
  profiling runs, rather than being held until shutdown.
- Trace scopes shorter than `1` microsecond are omitted by default. Set
  `OPENBUS_TRACE_MIN_US=5` to omit scopes shorter than five microseconds;
  values below `1` are treated as `1`.
- Optional texture downscaling: set `OPENBUS_TEXTURE_SCALE=0.5` before
  launching to upload half-resolution textures; valid range is `0.25`-`1.0`
  JPEG files. To create BC3 DDS siblings with AMD Compressonator, run
  `.\convert_textures.ps1 -Compressonator C:\path\to\CompressonatorCLI.exe`.
  selection, and sorted opaque/transparent batches to reduce off-screen work
  and state churn. Set `OPENBUS_FRUSTUM_CULLING=0` to disable culling for
  debugging. Opaque parts are front-to-back and transparent parts
  back-to-front; this is not hardware occlusion-query culling.
- Model geometry is uploaded to OpenGL vertex buffer objects (VBOs) and drawn
  with shader-backed `glDrawArrays` calls in an OpenGL 3.3 core profile.
- Window and door glass remains material-driven in the VBO path; the temporary
  flat glass fallback is no longer used for normal model materials.
- Normal MAN DL05 wheel meshes are removed from the static model pass and
  rendered from the live ODE wheel poses, including suspension, steering, and
  wheel rotation; the wire wheel fallback is used only when an asset is absent.
- The driver camera follows the ODE chassis position, pitch, roll, and heading
- OMSI coordinates use `x=lateral`, `y=longitudinal`, `z=vertical`; OpenBus
  converts them to engine `X=longitudinal`, `Y=lateral`, `Z=vertical` with the
  expected lateral reflection. This applies to CFG positions, camera centers,
  collision offsets, OBJ vertices, normals, and mesh pivots.
- O3D pivot matrices are animation metadata, not additional mesh placement
  transforms. Mirrored meshes are detected from pivot determinant and
  face/normal agreement; only those meshes have winding correction and
  clockwise back-face culling enabled.
- `[newanim]` records and their origin commands are processed in configuration
  order. Origin-only records are valid. `anim_rot`/`anim_trans` use the
  converted local animation frame, `delay` is rate-based smoothing, and
  `maxspeed` limits movement per second. `[animparent]` composes parent
  transforms for body and wheel/detail meshes.
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

## CPU flame graphs

Set `OPENBUS_TRACE=1` to write both the existing Chrome trace
(`openbus_trace.json`) and Brendan Gregg collapsed stacks
(`openbus_trace.collapsed`). Set `OPENBUS_TRACE_COLLAPSED=1` to write only the
collapsed stacks. `OPENBUS_TRACE_FILE` and `OPENBUS_TRACE_COLLAPSED_FILE`
override the respective output paths.

Use `profile.bat` for the standard Release profiled launch.

Collapsed values are elapsed microseconds. Each stack is prefixed with its
thread ID and includes frame, physics, renderer, OBJ, and texture scopes. With
the [FlameGraph](https://github.com/brendangregg/FlameGraph) scripts available:

```powershell
perl .\FlameGraph\flamegraph.pl .\openbus_trace.collapsed > .\openbus_trace.svg
```

The main frame path begins with `frame:main`; expand it to compare
`Renderer::beginFrame`, `BusSimulation::update`, `Renderer::draw`, and
`Renderer::endFrame` work.

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
