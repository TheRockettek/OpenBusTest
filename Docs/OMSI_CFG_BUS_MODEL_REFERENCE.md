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
lighting state is active.

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
width, height, alignment, colour, and transparency/format flags.

### `[texttexture_enh]`

Enhanced text texture definition. It commonly adds font style/size, colour
channels, and formatting flags to the basic text texture record.

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

The negative lateral sign is intentional. It is the reflection needed to keep
the original model winding and the renderer's culling convention consistent.

The converter writes O3D vertex positions unchanged and emits the O3D section
transform as `# openbus_transform` metadata. OpenBus therefore converts the
OBJ positions once using the mapping above. For `origin_from_mesh`, the
metadata translation supplies the pivot; its orientation is not applied again
to the already-converted vertices.

For `[newanim]` origin rotations, the converted axes are:

```text
origin_rot_x -> render -Y
origin_rot_y -> render +Z
origin_rot_z -> render +X
```

Generic `anim_rot` starts with the OMSI animation frame's source `Z` axis,
which maps to render `+X`. The `origin_rot_*` values then orient that frame.
The converted mesh transform is applied to the resulting axis, so the E400
steering-wheel block's `origin_rot_z 90` preserves its authored source `Z`
axis and the transform's third column supplies the tilted render axle. The
converted basis reverses the rotation handedness, so the signed animation
scale remains part of the model data. Wheel rolling is a separate special
case and uses render `+Y` after the wheel mount basis is applied.

Origin rotations are applied in `x`, `y`, `z` order, followed by the animation
transform, then undone in reverse order. The animation scale and sign are part
of the model data and must be preserved; for example, the DL05/E400 wheel
records use different signs for `Wheel_Rotation_*`.

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

Starts an animation record. A common sequence is:

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
animation type, controller variable, and scale must be kept in their original
order.

### `[animparent]`

One parent attachment/name. The current mesh inherits the parent transform,
which is how doors, panels, wheels, and engine-bay parts follow another
animated component.

### `[alphascale]`

One script variable controlling object alpha/visibility strength.

### `[visible]`

Two common values:

1. Visibility variable/name
2. Active/inactive value or mode

Use repeated blocks for script-controlled visibility states.

## 5. Lights and interior illumination

### `[illumination_interior]`

Assigns interior illumination groups to the current mesh. Repeated numeric
values identify lighting channels/material groups.

### `[interiorlight]`

Defines an interior light. Common fields include controlling variable, mode,
RGB colour, intensity, and local x/y/z position. Preserve all values because
the exact tail varies by vehicle.

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
