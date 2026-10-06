# OMSI bus-model CFG reference

This document covers the CFG files referenced by a bus `[model]` entry. The
newer bus model files in the corpus use these sections to assemble the bus
from meshes, assign materials, animate parts, control visibility, and connect
script-driven displays and lights.

## 1. Model assembly

### `[mesh]`

Adds one `.o3d` mesh path. Repeated blocks are loaded in order. The order must
remain stable because animations, material changes, visibility entries, and
illumination groups refer to the assembled model.

### `[mesh_ident]`

Assigns a stable name to the preceding mesh. Later `[animparent]`,
`[mouseevent]`, visibility, and script references can use this name instead of
an implicit mesh index.

### `[LOD]`

Sets a level-of-detail threshold for the following model content. Preserve
ordering when multiple detail levels are present.

### `[collision_mesh]`

Selects a collision mesh, normally an `.o3d`, instead of using visible mesh
geometry for collision.

### `[viewpoint]`

Sets the view categories in which the following mesh is active. The value is a
bit mask made by adding:

- `1`: player-vehicle exterior
- `2`: player-vehicle interior
- `4`: non-player vehicle

`0` means all viewpoints. This is a mesh-level optimization and is separate
from selecting a camera.

### `[fixed]`

Marks the following object as fixed/static rather than attached to an animated
parent.

### `[absheight]`

Uses absolute/world height handling for the following object. Its exact
payload is model-family-specific.

### `[rendertype]`

Sets the render category, for example `surface`, for following geometry.

## 2. Material and texture assignment

### `[matl]`

Assigns a base texture/material to the preceding or following mesh context.
The common record is:

1. Texture filename
2. Material/texture slot index

### `[matl_item]`

Starts a material-item subrecord. The following material modifiers apply to
that item until the next material declaration.

### `[matl_change]`

Conditionally changes a material texture or slot:

1. Texture/material filename
2. Material slot/index
3. Script variable or condition name

### `[matl_bumpmap]`

Two values: bump/normal-map filename and strength.

### `[matl_envmap]`

Two values: environment-map filename and reflection strength.

### `[matl_freetex]`

Two values: texture filename and a free/script texture variable name.

### `[matl_lightmap]`

Two values: lightmap filename and the controlling light/brightness variable.

### `[matl_nightmap]`

One night-texture filename. The texture is used when the relevant night
lighting state is active. It is treated as a self-lit overlay: black texels
leave the base material visible, while brighter texels contribute the
nightmap's authored colour.

### `[matl_transmap]`

One transparency-map expression or texture reference, commonly a script
expression such as `\S:1`.

### `[matl_alpha]`

One alpha/rendering mode integer:

- `0`: ignore alpha; render opaque
- `1`: clip alpha at `0.5`
- `2`: blend alpha with the existing image

### `[matl_noZcheck]`

Marker disabling depth-buffer comparison for the material.

### `[matl_noZwrite]`

Marker disabling depth-buffer writes for the material. It does not itself
enable alpha blending; combine it with `[matl_alpha]` when both behaviors are
required.

### `[matl_texadress_border]`

Marker selecting border texture addressing.

### `[matl_texadress_clamp]`

Marker selecting clamped texture addressing.

### `[tex_detail_factor]`

One detail-texture strength/scaling value.

### `[CTC]`

Starts a colour/texture-change scheme. Common values identify the scheme,
texture directory/source, and a mode flag.

### `[CTCTexture]`

Maps a named CTC texture variable to a texture filename. Repeated records form
the complete paint-scheme mapping.

### `[texchanges]`

Starts a model texture-change table. Following entries identify replacement
textures and their controlling variables; exact record length varies by model
generation.

### `[newtexchangemaster]`

Defines a reusable texture-change master. It commonly supplies a base texture,
an identifier, and an `[entries]` count/list.

### `[entries]`

Count-prefixed filenames or replacement values belonging to the preceding
texture-change master.

## 3. Script textures and displays

### `[scripttexture]`

Creates a dynamic texture surface. The common leading values are width and
height in pixels, followed by display/alpha options.

### `[texttexture]`

Defines a named text-rendering surface. Common fields include name, font,
width, height, font-colour mode, and RGB colour. Basic text is centered
horizontally; OpenBus vertically centers the rendered line block in the surface.

### `[texttexture_enh]`

Enhanced text texture definition. It commonly adds font style/size, colour
channels, and formatting flags to the basic text texture record. Its horizontal
alignment value is `0` = center, `1` = left, and `2` = right. Font-colour mode
`0` uses the configured RGB colour; `1` preserves the font's colour image.

### `[useTextTexture]`

One text-texture identifier/index applied to the current mesh/material.

### `[VFDmaxmin]`

Six display bounds: minimum x/y/z followed by maximum x/y/z. A following
view-context flag may restrict the display to interior, exterior, or AI views.

## 4. Camera definitions

These records are normally defined in a BUS or vehicle CFG rather than in an
individual mesh section.

### `[add_camera_driver]`

Adds a driver camera. The record contains local position, orbit distance, field
of view, initial pan, and initial tilt. Pan is positive to the right and tilt
is positive upward. The left/right camera keys step through driver cameras in
file order.

### `[add_camera_pax]`

Adds a passenger camera with the same layout and angle conventions as
`[add_camera_driver]`.

### `[add_camera_reflexion]`

Adds a reflection/mirror camera. Its render target is a square virtual texture
named `reflexionN.bmp`, where `N` is the zero-based reflection-camera index.
The orbit distance should normally be zero.

### `[add_camera_reflexion_2]`

Adds a reflection camera with an additional culling value. The culling value
controls when rendering stops as the mirror leaves the view.

### `[set_camera_std]`

Sets the zero-based default driver-camera index.

### `[set_camera_outside_center]`

Sets the local point around which the exterior camera orbits.

### `[texcoordtransX]` and `[texcoordtransY]`

Each takes one script variable/expression controlling dynamic texture-coordinate
translation along the corresponding texture axis.

## 4. Animation and hierarchy

### Coordinate convention used by OpenBus

OMSI model CFG coordinates use `x` for lateral position, `y` for longitudinal
position, and `z` for height. OpenBus simulation/render coordinates use `X` for
longitudinal position, `Y` for lateral position, and `Z` for height. CFG
positions are therefore converted as:

```text
render X = CFG y
render Y = -CFG x
render Z = CFG z
```

Converted OBJ vertices use the equivalent importer mapping:

```text
render X = OBJ z
render Y = -OBJ x
render Z = OBJ y
```

Here `OBJ x/y/z` are the values written by the converter from the O3D file:
O3D `x` is right/lateral, O3D `y` is up/vertical, and O3D `z` is
forward/longitudinal. The negative lateral sign is intentional and is part of
the handedness conversion.

The converter writes O3D vertex positions unchanged and emits the O3D section
transform as `# openbus_transform` metadata. OpenBus therefore converts the
OBJ positions once using the net mapping above. The metadata matrix is not a
second mesh placement transform. It is retained as the mesh pivot for
`origin_from_mesh`; its converted translation and orientation are applied only
when evaluating an animation origin.

Mirrored mesh handling is separate from coordinate conversion. OpenBus compares
each face cross product with its summed vertex normals and with those normals
after the converted pivot rotation. If a converted pivot has a positive
determinant, the transformed-normal explanation does not account for the
disagreement, and at least 90% of the counted faces oppose their normals, the
OBJ triangle winding is reversed. Clockwise back-face culling is enabled only
for these mirrored meshes; ordinary meshes are left unculled so their authored
winding is preserved.

For `[newanim]` origin rotations, the converted axes are:

```text
origin_rot_x -> render -Y
origin_rot_y -> render +X
origin_rot_z -> render +Z
```

OpenOMSI evaluates these rotations with a negative angle. Because the
render-coordinate conversion is handed, the equivalent OpenBus operations are
positive angle around render `+Y`, negative angle around render `+X`, and
negative angle around render `+Z`, respectively. Both
`anim_rot` and `anim_trans` use the animation frame's source X axis; this is
render `-Y`, so animated translation is negative render Y.

Each `[newanim]` appends one animation record in configuration order. Its
`origin_trans`, `origin_rot_x`, `origin_rot_y`, `origin_rot_z`, and
`origin_from_mesh` commands are also retained and composed in the order in
which they appear. A record may contain only origin commands: it is valid and
remains an identity animation step, rather than being rejected for lacking
`anim_rot` or `anim_trans`.

When present, `anim_rot` and `anim_trans` operate on the animation frame's
local source X axis. Under OpenBus's handedness conversion that source X axis
is render `-Y`; rotation and translation are evaluated from the converted
origin matrix and then composed with the other `[newanim]` records. The first
record in the file acts first, so later records wrap the existing result.
Signed animation scales are preserved.

`origin_from_mesh` uses the converted pivot metadata. It does not move the
already-converted mesh a second time. `origin_trans` uses the normal CFG
coordinate conversion above.

`delay` is a rate, not a millisecond duration: each frame closes the remaining
amount by `delay * Timegap`, capped at one. `maxspeed` then limits the change
per second. `offset` is added to the controller value before smoothing.

Wheel meshes have one additional fixed basis: the ODE wheel body is initialized
with a `-90` degree rotation around `X`, and the renderer cancels that basis
before applying wheel animation variables. Rolling is excluded from the mount
pose and is applied from `Wheel_Rotation_*` around render `+Y` using the
magnitude of the CFG scale. The source O3D scale sign is not applied again
after the shared OBJ coordinate reflection; doing so reverses converted E400
wheels. Wheel models
are matched to physics wheels by their converted origin, not by file order;
the physics index convention is even index `+Y` (left) and odd index `-Y`
(right). Custom steering variables may animate any wheel. A variable named
`Axle_Steering_N_L/R` is applied only to physical axle `N`, preventing legacy
cross-axle records from steering the wrong wheel.

### `[newanim]`

Appends an ordered animation record. A record may contain only origin commands,
or it may also contain one driven operation. A common sequence is:

```text
[newanim]
origin_from_mesh
origin_rot_y
<angle>
anim_trans
<script variable>
<scale>
```

Other records use `anim_rot` for rotation. The origin mode, rotation axis,
animation type, controller variable, and scale are processed in their original
order. The animation type and controller are optional; origin-only records are
valid.

`delay` is a rate-based smoothing value, `maxspeed` is a per-second limit, and
`offset` is added to the driven value.

### Wheel animation timing in OpenBus

The physical and visual wheel paths are updated in this order each frame:

1. `BusSimulation::update()` advances ODE in fixed-size steps. Each step calls
  `refreshWheelTelemetry()` after `dWorldStep()`, which reads suspension from
  the wheel/chassis separation projected onto chassis-local Z and reads the
  wheel angular velocity from the ODE wheel body rotation and angular
  velocity. Spring and damper forces use bounded positive compression; the
  model animation uses the signed chassis-local wheel displacement so it
  follows the physical wheel pose through chassis pitch and roll. ODE's slider
  coordinate is also exposed by `wheelSuspensionSliderTravel()` for physics
  diagnostics.
2. The same fixed step reads the ODE wheel hinge angle, unwraps it across
  `+/-pi`, and publishes the accumulated value as
  `Wheel_Rotation_<axle>_<side>`. The accumulated value is sign-inverted at
  this boundary because ODE's hinge angle axis is opposite to the model CFG
  wheel-rotation convention. Suspension is published as
  `Axle_Suspension_<axle>_<side>`. Angular velocity is retained separately as
  `Wheel_RotationSpeed_<axle>_<side>`.
3. `main.cpp` calls `renderer.updatePlayerVariables()` immediately after
  `simulation.update()`. The values therefore reach the model variables in
  the same frame as the latest completed ODE step.
4. During `Vehicle::draw()`, `updateAnimationStates()` converts each variable
  to a `newanim` target. It then applies `delay` smoothing and `maxspeed`
  limiting before `animationTransformForPart()` builds the mesh transform.

Scripts can also tune the physical suspension through writable, per-wheel
`Axle_Springfactor_<axle>_<side>` variables. OpenBus multiplies the configured
axle spring rate by this factor before calculating elastic support force;
damper and anti-roll forces remain independently configured. A factor of `1`
preserves the configured rate, and `0` removes the spring force for that wheel.
Finite values are clamped to `[0, 10]`; missing or non-finite values use the
neutral factor `1`.

ODE wheel bodies start at the configured rest position with their tire bottoms
at the ground plane; the simulation does not apply a pre-compression based on
factor `1` before scripts provide their live spring factors. This avoids an
initial free-fall of the wheels when a script selects a softer spring. Wheel
contacts use the same ground plane as the rendered road, with increased error
recovery to limit transient penetration. A wheel-only low-factor physics probe
checks ground clearance and confirms that published animation displacement
matches the ODE wheel pose.

For each axle, the ODE tire radius is the larger of the configured bus-file
radius and the radial envelope of the model mesh carrying that axle's
`Wheel_Rotation` animation. This keeps a visibly larger tire from extending
through the ground while retaining the configured radius when model geometry
is unavailable or smaller.

Consequently, ODE debug geometry shows the latest physical wheel pose, while a
`newanim` wheel can intentionally show an earlier pose when its animation has
`delay` or `maxspeed`. A mismatch with no such limits must instead come from
the animation scale/sign, origin basis, or the fact that ODE rotation is
integrated from angular velocity while the model consumes the accumulated
`Wheel_Rotation` value. The rotation target is now derived from the ODE hinge
angle rather than integrating angular velocity a second time; this avoids a
step of phase error when ODE changes the wheel speed during a fixed step.
For suspension, ODE reports positive compression when the wheel moves upward.
The E400 wheel CFG uses `origin_rot_y -90` and positive `anim_trans` scale; in
the converted frame, a positive animation amount moves the mesh downward.
Therefore OpenBus publishes the negative of the wheel's projected ODE
displacement: compression moves the rendered wheel up, and droop moves it down.

For wheel debugging, compare these values in order: ODE `wheelPose()`;
`wheelRotation()` and `wheelSuspensionCompression()`; the published variable;
the animation target amount; and finally the smoothed `currentAmount`. The
first value that differs identifies the responsible layer.

### `[animparent]`

One parent attachment/name. The current mesh inherits the parent transform,
which is how doors, panels, wheels, and engine-bay parts follow another
animated component. OpenBus requires the referenced `[mesh_ident]` to have
already been declared and resolves parents across both body and wheel/detail
mesh collections. Parent and child animation transforms compose in the same
ordered matrix path.

### `[alphascale]`

One script variable controlling object alpha/visibility strength.

### `[visible]`

Two common values:

1. Visibility variable/name
2. Active/inactive value or mode

Use repeated blocks for script-controlled visibility states.

## 5. Lights and interior illumination

### `[illumination_interior]`

Assigns four slots on the current mesh to model-wide `[interiorlight]` records.
Each slot is a zero-based index into the ordered list of those records; `-1`
leaves a slot unused. A mesh can reference up to four lights, while a model can
define more than four.

### `[interiorlight]`

Defines an interior light with eight values: controller variable (or numeric
constant), intensity, red, green, blue, and local x/y/z position. Installed
vehicle CFGs use byte-scale RGB values (0–255). Positions use the model's local
coordinates. The format source available here does not specify physical units
for intensity, so renderers should treat it as an authored relative strength.

### `[light_enh_2]`

Enhanced light definition. The examples contain local position, direction,
colour, intensity/range, and mode values.

### `[spotlight]`

Defines a spot light. Common values include position, direction, RGB colour,
range, inner angle, and outer angle.

### `[isshadow]`

Marker or mode controlling shadow participation for the current object/light.

## 6. Interaction and visibility

### `[mouseevent]`

Starts a clickable mesh/event record. The following value identifies the event
or script action; its remaining layout is vehicle-specific.

### `[viewpoint]`

Restricts mesh visibility to a view category/index. This is distinct from the
driver/passenger camera definitions in a BUS or passenger-cabin CFG.

## 7. Model-file authoring order

A reliable model CFG order is:

1. Declare model-level texture/text definitions.
2. Declare each mesh and optional `[mesh_ident]`.
3. Apply base `[matl]` and material modifiers.
4. Add `[viewpoint]`, `[visible]`, and `[animparent]`.
5. Add `[newanim]`, mouse events, illumination, and lights.
6. Add CTC/texture-change definitions and display bindings.

The parser is keyword-driven, but model references are order-sensitive. Keep a
mesh's material, animation, and visibility records together unless the source
model explicitly separates them.

## 8. Safe bus-model layout

Do not rename or move referenced assets without updating all relative paths.
For new projects, this layout separates model concerns while retaining
relative-path clarity:

```text
vehicle/
  bus/
    citybus.bus
  model/
    citybus.cfg
    mesh/
    collision/
    texture/
    display/
    light/
  script/
    *.osc
    *.txt
```
