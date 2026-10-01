# OpenBus Code Review

**Review date:** 2026-10-01  
**Scope:** First-party C++ source, headers, tests, and CMake configuration. Third-party Lua/sol sources and build artifacts were excluded.

## Findings

### High Priority

#### 1. Normalized script variables are case-sensitive by mistake

`getNormalized`, `getStringNormalized`, and their setters bypass `keyFor()`. Shipped mixed-case E400 Lua variables can therefore create duplicate keys and fail to read canonical state.

**Location:** `src/Variables.cpp:61`

**Fix:** Apply the same lowercase normalization to all four normalized accessors, or rename the API if exact-case access is intentional. Add tests covering mixed-case reads and writes.

#### 2. `AddVehicle` ignores `spawnPosition`

The overload explicitly discards its spawn argument, so every added vehicle starts at the same location.

**Location:** `src/Renderer.cpp:2951`

**Fix:** Pass spawn coordinates into the vehicle/simulation state, or remove the overload until multi-vehicle spawning is supported.

#### 3. `nrspecrandom` uses the wrong denominator

Both native and Lua implementations divide `std::minstd_rand` output by C `RAND_MAX`. On Windows this can produce values far above `1.0`.

**Locations:** `src/ScriptRuntime.cpp:144` and `src/ScriptRuntime.cpp:931`

**Fix:** Divide by `std::minstd_rand::max()` or use `std::uniform_real_distribution<double>` and add a range test asserting `[0, 1]`.

### Medium Priority

#### 4. Texture loading has no bounded worker strategy

Texture requests use one `std::async` task per asset. OBJ loading also creates an asynchronous task even though startup immediately waits for it. Large models can incur excessive thread creation and scheduling overhead.

**Location:** `src/AssetRequestManager.cpp:67`

**Fix:** Use a bounded worker pool and queue, or make synchronous startup loading explicit. Keep visible-asset loading behavior deterministic.

#### 5. Texture alias keys can collide

The alias key contains only the texture root and filename stem. Two different subdirectories containing the same stem can resolve to the first cached texture.

**Location:** `src/AssetRequestManager.cpp:55`

**Fix:** Include the relative parent path in the alias key. Preserve extension aliasing without collapsing unrelated directories.

#### 6. Texture address modes are parsed but ignored

`matl_texadress_clamp`, border, mirror, and mirror-once values reach the configuration model, but texture uploads hard-code `GL_REPEAT`.

**Location:** `src/Renderer.cpp:1527`

**Fix:** Carry address modes into `Batch`/material state and apply them at upload time. Add representative CFG/material tests.

#### 7. Malformed asset and configuration sizes are insufficiently bounded

DDS block arithmetic can overflow around `image.width + 3`; BMP decoding uses `std::abs(INT_MIN)`; count-prefixed configuration blocks can request very large allocations.

**Locations:** `src/TextureLoader.cpp:199`, `src/TextureLoader.cpp:313`, and `src/VehicleConfigLoader.cpp:367`

**Fix:** Add checked arithmetic, maximum decoded-image sizes, maximum record counts, and graceful diagnostics for oversized inputs.

#### 8. Worker-thread COM cleanup is not exception-safe

`CoUninitialize()` is called only on normal texture-loading paths. Exceptions from texture resolution or decoding skip cleanup.

**Location:** `src/AssetRequestManager.cpp:135`

**Fix:** Use a small RAII COM initializer in the worker function.

#### 9. Script APIs still return placeholders

HOF/timetable/passenger lookups, ground height, ticketing, arrival boards, text rendering, and `stloadtex` still return zero or empty values.

**Location:** `src/ScriptRuntime.cpp:864`

**Fix:** Either implement these systems or clearly mark them unsupported before claiming broader OMSI compatibility. Add focused behavior tests as each subsystem is added.

#### 10. Probes are not registered with CTest

The project builds `OpenBusPhysicsProbe` and `OpenBusVariablesProbe`, but `ctest` reports that no tests are registered.

**Location:** `CMakeLists.txt:178`

**Fix:** Add `enable_testing()` and `add_test()` entries, then run them from a clean build in CI.

## Maintainability Recommendations

- Remove unused suspension constants `SUSP_SPRING_K` and `SUSP_DAMPER_C`; actual axle rates come from configuration, so these values misleadingly suggest tunable behavior.
  - `src/BusSimulation.cpp:24`
- Split the large renderer into asset resolution, model assembly, animation, reflection, and vehicle-drawing modules.
- Replace the ambiguous `getNormalized` API with one consistently case-insensitive variable API.
- Add focused parser tests, material tests, texture-decoder fuzz tests, and deterministic ODE stress tests.
- Treat the stepped road-bump boxes as a physics-test risk. Their discontinuous vertical faces can create high-speed wheel impulses.
  - `src/BusSimulation.cpp:319`
- Consider a continuous collision mesh for road bumps instead of discrete boxes.
- Make the Windows crash reporter best-effort. Logging, streams, mutexes, and symbol APIs can themselves fail while handling a crash.

## Verification

- Source diagnostics: no errors reported.
- Release physics and variables probes: passed.
- `ctest --test-dir build-ode -C Release --output-on-failure`: completed with no registered tests.

## Review Notes

The current checkout already contains transparency sorting and active environment-map rendering. Those were not counted as missing features. User/formatter changes in the worktree were preserved and not reverted.

## Suggested Fix Order

1. Normalize variable access and add mixed-case script tests.
2. Fix random-value normalization and the ignored spawn-position contract.
3. Register existing probes with CTest.
4. Replace unbounded asset-task creation and make COM cleanup RAII-safe.
5. Fix texture aliasing and address-mode propagation.
6. Add input-size limits and malformed-asset tests.
7. Decide the supported OMSI scope, then implement or explicitly exclude the remaining script APIs.
8. Split the largest renderer/runtime modules after behavior is covered by tests.
