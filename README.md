# OpenBus

OpenBus is a C++ bus simulator prototype built on the
[Open Dynamics Engine (ODE)](https://www.ode.org/). This first milestone is a
headless physics sandbox: it creates a simplified bus chassis, four wheels, a
ground plane, and advances the vehicle through ODE's collision and constraint
solver.

## Prerequisites

- CMake 3.20 or newer
- A C++23 compiler
- ODE
- GLFW and OpenGL

On Windows with vcpkg, install standalone LLVM (for `clang-cl.exe`) and the
Visual Studio C++ build tools (for the MSVC headers, libraries, and linker):

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
.\run.bat
```

Native MSVC is rejected because its CMake C++23 mapping is
`/std:c++latest`, not an exact C++23 mode. The Windows helper initializes the
MSVC x64 environment and uses standalone Clang-cl with the NMake generator.
For a manual configure from `cmd.exe`:

```bat
call windows_clang_env.bat
cmake -S . -B build-ode -G "NMake Makefiles" -DCMAKE_CXX_COMPILER="%OPENBUS_CLANG_COMPILER%" -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=C:/vcpkg/scripts/buildsystems/vcpkg.cmake
cmake --build build-ode --target OpenBus
```

Release and RelWithDebInfo builds enable interprocedural/link-time optimization
for `OpenBus` by default. Disable it when comparing profiler call boundaries or
when using a toolchain that does not support IPO:

```bat
call windows_clang_env.bat
cmake -S . -B build-ode -G "NMake Makefiles" -DCMAKE_CXX_COMPILER="%OPENBUS_CLANG_COMPILER%" -DCMAKE_BUILD_TYPE=Release -DOPENBUS_ENABLE_IPO=OFF
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

### Optional binary mesh bundle

To reduce startup parsing and file-open overhead, generate an `.obx` bundle
from converted OBJ meshes:

```powershell
.\build-ode\Release\obj_bundle.exe MAN_DL05\Model\DL05_obj build-ode\DL05.obx
```

Copy or rename the resulting file to `openbus.obx` beside the model's
converted meshes. OpenBus uses matching bundle entries when available and
falls back to OBJ files for missing entries, so the bundle is optional.
The model CFG still controls mesh order, visibility, animations, and material
overrides.

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

Vehicle physics is loaded from the OMSI `.bus` file configured by
`OPENBUS_BUS_CONFIG`. Relative paths are resolved against the standard Steam
installation at `C:\Program Files (x86)\Steam\steamapps\common\OMSI 2`; set
`OPENBUS_OMSI_ROOT` when OMSI is installed elsewhere:

```powershell
$env:OPENBUS_OMSI_ROOT = "D:\SteamLibrary\steamapps\common\OMSI 2"
```

Vehicle values are not duplicated in C++.
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

`OPENBUS_BUS_CONFIG` accepts either an absolute path or a path relative to
`OPENBUS_OMSI_ROOT`. Set the optional `OPENBUS_MODEL_CONFIG` to select a `.cfg`
explicitly; otherwise the loader uses the `.bus` file's `[model]` entry. For
example, both paths below are relative to the OMSI installation root:

```powershell
$env:OPENBUS_BUS_CONFIG = "Vehicles/Caetano Levante/Caetano.bus"
$env:OPENBUS_MODEL_CONFIG = "Vehicles/Caetano Levante/Model/model_caetano.cfg"
./build-ode/Release/OpenBus.exe
```

Every rendered vehicle must provide a `VehiclePlacement` containing its world
`x/y/z` position and yaw in degrees. The renderer can load multiple vehicles
in one process by calling `RenderLoop::AddVehicle` once per `.bus`/`.cfg` pair.
The first vehicle is connected to the shared bus simulation; additional
vehicles are scripted AI models rendered at their declared placements. Set
`OPENBUS_AI_BUS_CONFIG` and `OPENBUS_AI_MODEL_CONFIG` together to add an
optional AI vehicle; `run.bat` uses them to load the VW Golf test vehicle.

The loader uses `[mass]`, `[boundingbox]`, `[schwerpunkt]`, and `[newachse]`
records, including axle spring, damper, load, and driven flags. The
`bodyHalfLength`, `bodyHalfWidth`, `bodyHalfHeight`, collision dimensions, and
collision offsets are derived from `[boundingbox]`. Because standard OMSI
records do not define wheel thickness or articulation, each file also supplies
`[openbus_wheel_half_width]` and `[openbus_articulated]`. Axle steering is
derived from `Axle_Steering_X_L/R` variables in the referenced model.cfg.

## Renderer controls

Vehicle script bindings use named keyboard keys rather than OMSI numeric key
codes. Pressing an action calls `trigger_<action>`; releasing it calls
`trigger_<action>_off`. At startup, matching vehicle actions inherit their key
and modifier assignments from `Inputs/keyboard.cfg` under `OPENBUS_OMSI_ROOT`;
the documented table below is the fallback when that file is unavailable.

| Key | Vehicle actions |
| --- | --- |
| Num8 | `throttle` |
| Num2 | `brake` |
| Num4 / Num5 / Num6 | `steering_left` / `steering_neutral` / `steering_right` |
| Tab | `clutch` |
| Num+ | `throttle_amplify`, `bus_doorfront5` |
| T | `ticket_give` |
| Shift+T / Ctrl+T | `change_take` / `change_give` |
| `.` | `parking_brake_toggle` |
| Num7 / Num9 / Num. | `blinker_left_set` / `blinker_right_set` / `blinker_off` |
| B | `blinker_warn_toggle` |
| L / Shift+L | `kw_scheinwerfer_toggle` / `kw_standlicht_toggle` |
| F / Ctrl+F | `kw_fernlicht_toggle` / `taster_nebelschluss` |
| M | `kw_m_enginestart` |
| W / Shift+W / Ctrl+W | `kw_wipermode_up` / `cp_wischer_intervall_toggle` / `cp_wischer_wascher_button` |
| H | `horn` |
| Q | `cp_microphone` |
| E | `cp_batterietrennschalter_toggle` |
| 6 / 7 / 8 / 9 | `cp_fahrerlicht_toggle` / `cp_licht_untenrechts_toggle` / `cp_licht_oberdeck_toggle` / `cp_licht_unterdeck_toggle` |
| R / N | `kw_s_R`, `automatic_R` / `kw_s_N`, `automatic_N` |
| 1 / 2 / 3 / 4 / 5 / 6 | `kw_s_1` / `kw_s_2` / `kw_s_3` / `kw_s_4` / `kw_s_5` / `kw_s_6` |
| D | `automatic_D` |
| Num/ / Num* / Num- / Scroll Lock | `bus_doorfront0` / `bus_doorfront1` / `bus_dooraft` / `bus_20h-switch` |
| F5 / F6 / F7 / F8 | `bus_linie_minus` / `bus_ziel_minus` / `bus_ziel_plus` / `bus_linie_plus` |
| F12 | `cp_schalter_kinderwagen` |
| `[` / `/` | `kw_s_plus` / `kw_s_minus` |

Cashdesk, IBIS, and rollband actions are intentionally not assigned yet.
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
- Set `OPENBUS_MATERIAL_BATCHING=1` to use the experimental material-table
  shader path. It combines compatible opaque geometry and supports up to 8
  ordinary 2D base textures per group; transparent, animated-material, and
  auxiliary-texture batches remain on the standard path. It is disabled in
  `profile.bat` because the current E400 workload is GPU-bound and the extra
  material texture sampling is slower than the additional draw calls.
- Native OSC bytecode is the default script backend. Scripts that cannot be
  compiled fall back to Lua; set `OPENBUS_SCRIPT_BACKEND=lua` to force Lua for
  debugging.
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
- Set `OPENBUS_WHEELS_FROM_ODE=1` to bypass wheel `newanim` transforms and
  place each wheel from its matching ODE body pose. The default is `0`.
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

## Repeatable render benchmark

Run `benchmark_rendering.bat` to build an isolated Release executable
with tracing enabled, then benchmark 720p, 1080p, 1440p, 4K, and 5120x1440.
The runner is `benchmark_rendering.py`, requires Python 3.9 or newer, and uses
only the standard library; set `OPENBUS_PYTHON` if `python` is not on PATH.
The runner performs five runs per resolution by default, in deterministic
seeded order; use `--runs 1` for a quick check or `--seed N` to change the order.
After building, the Python runner can also be launched directly with
`python benchmark_rendering.py`. Each run measures baseline rendering,
user-facing camera cycling, third-person zoom, driving controls, and dashboard
interaction. Benchmark mode creates a hidden GLFW window, uses a fixed 60 Hz
simulation step, and disables VSync. Hiding the window may avoid OS work-area
constraints; actual dimensions are still checked and mismatches remain flagged.
Normal simulator launches remain visible and retain the standard `build-ode`
build and `profile.bat` settings.

`--warmup-frames` is a total distributed across the five phases (60 by default,
12 per phase). Each phase is then measured twice per resolution run, with 120
fixed-step frames per visit by default (two seconds at 60 Hz), slowing camera,
zoom, steering, and dashboard-look progressions to half their previous rate.
Use `--phase-repeats` and `--phase-frames` to adjust those values. Results are written to
`render-benchmark-results/`. The `*_runs.csv` and `*_scopes.csv` files record
requested window, actual window, framebuffer sizes, framebuffer/window scale
factors, and flag any framebuffer that differs from the requested resolution.
Use `--require-exact-resolution` to
return a failure status if any run is clamped. The `*_frames.csv` file records
one row per main-frame or selected renderer-scope event, including timestamp,
inclusive duration, and self time. Nested calls stay separate so reflection
draws are not accidentally summed into a misleading per-frame total. High-volume
`Vehicle::drawBatch.prepareAndSubmit` events are included only when they last at
least 0.1 ms, keeping normal batch submissions from overwhelming the report.

The `*_scopes.csv` file groups Chrome trace scopes by phase and reports
occurrence count, summed inclusive elapsed wall time, summed self time, average,
and maximum duration. Durations are milliseconds converted from trace
microseconds; these are CPU-side elapsed scope timings, not GPU timings. Self
time subtracts the union of direct child intervals on the same thread, avoiding
double subtraction for overlaps. Raw per-run traces and logs are retained
alongside the CSV files.

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
