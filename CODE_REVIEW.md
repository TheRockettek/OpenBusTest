# OpenBus Code Review

**Review date:** 2026-10-01  
**Scope:** First-party C++ source, headers, tests, and CMake configuration. Third-party Lua/sol sources and build artifacts were excluded.

## Findings

### Medium Priority

#### 1. Script APIs still return placeholders

HOF/timetable/passenger lookups, ground height, ticketing, arrival boards, text rendering, and `stloadtex` still return zero or empty values.

**Location:** `src/ScriptRuntime.cpp:864`

**Fix:** Either implement these systems or clearly mark them unsupported before claiming broader OMSI compatibility. Add focused behavior tests as each subsystem is added.

#### 2. Shared variables are unsynchronized

`SimulationState::sharedVariables()` is read by rendering while scripts and simulation updates can write the same maps. `Variables` currently uses unsynchronized `std::unordered_map` storage.

**Location:** `src/Variables.h`

**Impact:** Concurrent reads and writes can race and invalidate unordered-map access, causing incorrect animation state or process crashes.

**Fix:** Establish an explicit single-thread ownership rule, or protect all variable map operations with a synchronization strategy that does not hold locks across script callbacks.

## Maintainability Recommendations

- Remove unused suspension constants `SUSP_SPRING_K` and `SUSP_DAMPER_C`; actual axle rates come from configuration, so these values misleadingly suggest tunable behavior.
  - `src/BusSimulation.cpp:24`
- Split the large renderer into asset resolution, model assembly, animation, reflection, and vehicle-drawing modules.
- Add focused parser tests, material tests, texture-decoder fuzz tests, and deterministic ODE stress tests.
- Treat the stepped road-bump boxes as a physics-test risk. Their discontinuous vertical faces can create high-speed wheel impulses.
  - `src/BusSimulation.cpp:319`
- Consider a continuous collision mesh for road bumps instead of discrete boxes.
- Make the Windows crash reporter best-effort. Logging, streams, mutexes, and symbol APIs can themselves fail while handling a crash.

## Verification

- Source diagnostics: no errors reported.
- Release physics and variables probes: passed.
- `ctest --test-dir build-ode -C Release --output-on-failure`: 2/2 tests passed.

## Review Notes

The current checkout already contains transparency sorting and active environment-map rendering. Those were not counted as missing features. User/formatter changes in the worktree were preserved and not reverted.

## Suggested Fix Order

1. Define variable ownership or synchronization before enabling concurrent script/render updates.
2. Decide the supported OMSI scope, then implement or explicitly exclude the remaining script APIs.
3. Split the largest renderer/runtime modules after behavior is covered by tests.
