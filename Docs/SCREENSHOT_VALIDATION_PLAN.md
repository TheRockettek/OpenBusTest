# Screenshot Validation Plan

This plan validates features currently classified as **fully supported** in the
OpenBus runtime. It is focused on visual behavior that can be demonstrated by
repeatable screenshots. Runtime-only properties such as cache hit rates,
triangle counts, crash handling, and exact physics values require logs or
automated tests in addition to screenshots.

## Test setup

- Build: `build-ode\Release\OpenBus.exe`
- Vehicle: MAN DL05
- Asset entry point: `MAN_DL05\Model\DL05.cfg`
- Window size: keep the same size for every comparison capture
- VSync: leave enabled unless timing behavior is being tested
- Camera controls:
  - `0`: outside orbit camera
  - `1`: driver viewpoint
  - `2`-`9`: additional views
  - Middle mouse drag: orbit/look around
  - Scroll: zoom
- Manual capture format: lossless PNG at the native window size
- Naming convention: `<feature>-<view>-<condition>.png`

Before each test, start a fresh process and wait until the model is fully
visible. Record the executable build and asset revision with the capture set.

The repository smoke path is `screenshot.bat`. With
`OPENBUS_CAPTURE_VIEWS=1`, it captures four automated exterior PNG views to
`screenshots\three-quarter.png`, `front.png`, `left.png`, and `right.png`.
Set `OPENBUS_VEHICLE=e400` before launching to exercise the
SP E400 MMC configuration and its larger `[newanim]`/`[animparent]` set.

The automated capture is a smoke check for model orientation, mirrored-mesh
winding, materials, wheel binding, and loading completeness. It does not
replace interactive driver/passenger, LOD-distance, steering, motion, or
close-up alpha tests listed below.

## Validation matrix

### 1. Model assembly and OBJ material loading

**Goal:** prove that the configured model meshes, material groups, and texture
assignments are rendered together.

**Steps:**

1. Start the simulator with the MAN DL05 asset available.
2. Capture an exterior three-quarter view showing the complete bus.
3. Capture a side view showing separate body, window, door, wheel, and decal
   regions.
4. Capture the driver view to confirm that interior geometry is present.

**Expected screenshots:**

- The bus body is complete and positioned as one vehicle.
- Textures are assigned to the correct material regions rather than appearing
  as a single fallback color.
- Interior, doors, windows, and exterior geometry are visible in their
  appropriate views.

**Evidence:**

- `model-exterior-three-quarter.png`
- `model-exterior-side.png`
- `model-driver-interior.png`

### 2. LOD selection

**Goal:** prove that `[LOD]` content changes with camera distance without
  disappearing or visibly corrupting the model.

**Steps:**

1. Capture the bus at close range in the outside camera.
2. Move the camera farther away while keeping the bus centered and capture the
   same angle.
3. Repeat at a distance where the next LOD is expected to be selected.

**Expected screenshots:**

- Close range shows the highest-detail model.
- Farther captures show a stable lower-detail model.
- No abrupt blank frame, missing body section, or unrelated mesh appears at the
  transition.

**Evidence:**

- `lod-close.png`
- `lod-mid.png`
- `lod-far.png`

Screenshot comparison should be paired with the window triangle count where
possible; the count is stronger evidence that the LOD changed than visual
inspection alone.

### 3. Viewpoint filtering

**Goal:** prove that viewpoint masks select the expected exterior/interior
content.

**Steps:**

1. Capture an outside view with camera `0`.
2. Capture the driver view with camera `1`.
3. Capture a passenger or alternate interior view using one of cameras `2`-`9`.

**Expected screenshots:**

- Exterior-only parts remain appropriate to the outside view.
- Interior parts appear from the driver/passenger views.
- No major mesh layer incorrectly vanishes from the view in which it belongs.

**Evidence:**

- `viewpoint-exterior.png`
- `viewpoint-driver.png`
- `viewpoint-passenger.png`

### 4. Alpha materials and translucent glass

**Goal:** prove alpha-tested and blended materials render with correct depth
  behavior.

**Steps:**

1. Capture a side view containing windows, door glass, and body panels.
2. Capture an angled view where the opposite side or interior is visible
   through the glass.
3. Capture a door area at close range.

**Expected screenshots:**

- Opaque body panels remain solid.
- Glass is translucent rather than opaque white or black.
- Interior and background geometry can be seen through glass where expected.
- Transparent surfaces do not produce severe sorting, flickering, or depth
  fighting artifacts.

**Evidence:**

- `alpha-side.png`
- `alpha-angled-interior.png`
- `alpha-door-close.png`

### 5. Depth-write-disabled materials

**Goal:** prove `[matl_noZwrite]` materials do not incorrectly block geometry
  behind them.

**Steps:**

1. Capture a close view of overlapping translucent window or overlay geometry.
2. Repeat after orbiting slightly so the background geometry changes.

**Expected screenshots:**

- The overlay remains visible.
- Background geometry is not permanently occluded by the overlay.
- Coplanar layers do not flash or alternate between visible and invisible.

**Evidence:**

- `nozwrite-angle-a.png`
- `nozwrite-angle-b.png`

### 6. Environment-map materials

**Goal:** prove `[matl_envmap]` produces a visible reflection contribution on
  materials that use it.

**Steps:**

1. Capture reflective body or trim from two different outside angles.
2. Keep lighting and camera distance comparable between the captures.

**Expected screenshots:**

- The base material remains visible.
- The reflective contribution changes as the view changes.
- The environment layer is not rendered as an opaque replacement texture.

**Evidence:**

- `envmap-angle-a.png`
- `envmap-angle-b.png`

This is a visual smoke test, not proof that the reflection equation exactly
matches OMSI.

### 7. Static decals

**Goal:** prove decal meshes and their alpha textures load through the generic
  mesh/material path.

**Steps:**

1. Capture the side panel containing the door or body decals.
2. Zoom in until the decal texture and its alpha boundary are clear.
3. Capture the same region from a slight angle.

**Expected screenshots:**

- The decal texture is present and correctly located.
- Transparent decal pixels reveal the underlying bus paint.
- The decal does not render as a solid rectangle.
- The decal remains stable when the camera angle changes.

**Evidence:**

- `decals-side.png`
- `decals-close.png`
- `decals-angle.png`

This validates static decal composition only. Dynamic script-controlled decals,
lightmaps, and texture changes are outside this “full” screenshot claim.

### 8. Live wheel rendering

**Goal:** prove wheel meshes are bound to live physics poses rather than being
  left in the static model position.

**Steps:**

1. Capture a stationary side view showing all visible wheel groups.
2. Hold `A` or `D` and capture the steered front wheels.
3. Hold `W` briefly, then capture a moving or post-motion wheel view.
4. Capture a close view of a wheel and its wheel arch.

**Expected screenshots:**

- All configured wheels are present in their wheel arches.
- Steering changes the front-wheel orientation.
- Suspension movement keeps wheels attached to the chassis position.
- Wheel rotation follows movement rather than remaining a static mesh.
- No wire fallback appears when the configured wheel asset is available.

**Evidence:**

- `wheels-stationary.png`
- `wheels-steered.png`
- `wheels-after-motion.png`
- `wheels-close.png`

Screenshots demonstrate pose changes; wheel angular speed, slip, and force
values require a separate runtime or automated physics test.

### 9. Supported texture decoding

**Goal:** prove that representative texture families render correctly.

**Steps:**

1. Capture the bus using the normal MAN DL05 asset set.
2. Ensure the capture includes indexed/RLE TGA content, common raster textures,
   and DDS-backed materials where those assets are visible.
3. Capture a close-up of `body_ReflFF.tga`-backed geometry and a DXT5-backed
   material if both are visible in the selected model.

**Expected screenshots:**

- Textures contain expected colors and detail.
- Indexed/RLE TGA content is not blank or corrupted.
- DXT5 content is not rendered as a white fallback.
- Alpha channels remain meaningful on glass and decals.

**Evidence:**

- `textures-body-reflff.png`
- `textures-dds-dxt5.png`
- `textures-glass-and-decals.png`

Texture logs should be retained with the screenshots to prove which source
files were decoded and uploaded.

## Review checklist

- [ ] Every capture uses the same build and asset revision.
- [ ] Each expected visual result is visible at the chosen framing.
- [ ] No screenshot contains a missing-model, white-texture, or wire-fallback
      artifact unless the test explicitly expects it.
- [ ] Before/after or multi-angle evidence is included for behavior that cannot
      be demonstrated in one still image.
- [ ] `OpenBus_textures.log` is attached for texture-decoding validation.
- [ ] Any result that depends on a debug overlay or triangle count is labeled as
      supplementary evidence rather than screenshot-only proof.

## Limits of screenshot validation

Screenshots cannot establish exact parser coverage, material keyword semantics,
cache effectiveness, frame-time improvements, crash recovery, or numerical
physics correctness. Those require source inspection, logs, structured output,
or automated tests. A passing screenshot therefore validates the visible
behavior of the feature, not complete equivalence with OMSI.