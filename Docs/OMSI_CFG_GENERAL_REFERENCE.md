# OMSI `.cfg` general configuration reference

This reference is inferred from the CFG files supplied with the OMSI examples.
The corpus contains several independent configuration languages that all use
the same keyword-oriented syntax. A CFG file must be interpreted according to
its role; a `[mesh]` in a vehicle model CFG does not have the same surrounding
context as `[mesh]` in an environment or object CFG.

The files should not be physically moved without rewriting references. BUS,
model, path, sound, and passenger-cabin files contain relative paths to one
another. The classifications below are therefore logical configuration types.

## 1. Shared syntax

CFG files use the same rules as the BUS files:

- A keyword must be the only content on its line and start at column zero.
- Values are read from following lines until the block's fixed layout, count,
  or terminator has been consumed.
- Unknown/non-keyword text is normally ignored while searching.
- Repeated keywords create ordered records.
- Some blocks use a count followed by that many records.
- Some blocks use `[end]` or a family-specific terminator.

See [OMSI_BUS_FORMAT_REFERENCE.md](C:/Users/blane/Desktop/OpenBusTest/Docs/OMSI_BUS_FORMAT_REFERENCE.md)
for the common parser rules and commenting conventions.

## 2. Vehicle/model CFGs

These are the large model files referenced by a BUS `[model]` section.

### `[mesh]`

Adds one `.o3d` mesh filename. Repeated `[mesh]` blocks add meshes in order.
The order is significant for later script, texture, animation, and collision
references.

### `[LOD]`

One LOD threshold or distance value. Multiple LOD records may select different
model detail levels.

### `[rendertype]`

One render classification such as `surface`. It determines how the following
model/object data is rendered.

### `[fixed]`

Marks geometry or an object as fixed/static rather than moving with an
animation transform.

### `[absheight]`

Marks height handling as absolute/world-referenced. The following layout is
model-family-specific.

### `[collision_mesh]`

One collision mesh filename, normally an `.o3d`. It is used instead of or in
addition to the visible meshes for collision detection.

### `[viewpoint]`

One viewpoint or visibility category/index. It is commonly used to restrict an
object to interior, exterior, AI, or other view contexts.

### `[mouseevent]`

Starts a mouse-interaction record. The following lines identify the clickable
mesh/event and its script or variable action; the exact record is model
specific.

### `[illumination_interior]`

A repeated interior-light assignment. The values identify an illumination
mode and affected mesh/vertex/material groups. Preserve the record order from
the source model.

### `[CTC]`

Starts a colour/texture-change configuration. The common values are a
scheme/name, a texture directory or source, and a mode/flag.

### `[CTCTexture]`

Maps one CTC texture variable/name to one texture filename. Repeated records
define the complete paint-scheme texture set.

### `[newtexchangemaster]`

Starts a texture-change master. Common leading values are a base texture
filename and a replacement/variable name, followed by an `[entries]` list.

### `[entries]`

Count-prefixed entries associated with the preceding texture-change master:

```text
[entries]
<count>
<entry filename or value 1>
...
```

### `[scripttexture]`

Defines a dynamic script-driven texture surface. Common leading values are
width and height in pixels, followed by script texture options. Repeated
blocks define displays such as LED/VFD signs.

### `[texttexture]`

Defines a named text-rendering surface. Common values include:

1. Texture identifier
2. Font identifier or font family
3. Width in pixels
4. Height in pixels
5. Alignment/format flags
6. Colour or transparency options

Exact trailing values vary by model version.

### `[texttexture_enh]`

Enhanced text texture definition. The examples commonly contain a texture
name, font name/size/style, dimensions, colour channels, and alignment/format
flags.

### `[VFDmaxmin]`

Six bounds for a variable display or viewable model region:

1. Minimum x/y/z
2. Maximum x/y/z

The file may include a view-context flag after the block; preserve it with the
source model.

### `[tex_detail_factor]`

One detail-texture strength/scaling factor.

### `[terrainmapping]`

Terrain/material mapping data for a model surface. Layout varies considerably;
it can contain a texture or mapping descriptor rather than a simple numeric
record.

## 3. Surface and texture CFGs

These are commonly named after an image, for example
`2_strasse2.bmp.cfg`, and describe how a texture behaves as a road or terrain
surface.

### `[puddles]`

Enables puddle/wetness behaviour for the texture. It is commonly a marker with
no value lines.

### `[moisture]`

Enables moisture behaviour for the texture. It is commonly a marker with no
value lines.

### `[surface]`

One integer surface type:

| Value | Surface |
| ---: | --- |
| 0 | Asphalt (default) |
| 1 | Concrete |
| 2 | Cobblestone |
| 3 | Dirt |
| 4 | Grass |
| 5 | Gravel |
| 6 | Snow |
| 7 | Deep snow |

The supplied `2_strasse2.bmp.cfg` uses:

```text
[puddles]

[moisture]

[surface]
0
```

This means the texture supports puddle/moisture effects and is classified as
asphalt.

### `[NightMapMode]`

Selects how a night texture/map is applied. The mode values are renderer
version dependent.

### `[scripttexture]`

Also occurs in model CFGs and describes a dynamic texture surface. When used
in a standalone texture/material context, interpret its dimensions and
following options as texture metadata rather than a mesh declaration.

## 4. Passenger-cabin CFGs

These files are referenced by BUS `[passengercabin]`.

### `[drivpos]`

Five values describing a driver position:

1. Local x
2. Local y
3. Local z
4. Camera/pivot distance
5. Driver/view flag

### `[passpos]`

Five values describing one passenger standing/seated position:

1. Local x
2. Local y
3. Local z
4. passenger/camera radius or interaction value
5. passenger/view flag

Repeated blocks create passenger positions.

### `[entry]`

One passenger entry-point or door/boarding index. Repeated blocks define
multiple entries.

### `[exit]`

One passenger exit-point or door/alighting index. Repeated blocks define
multiple exits.

### `[linkToNextVeh]` and `[linkToPrevVeh]`

One passenger-cabin connection index for an articulated or coupled vehicle.
The value links this cabin to the next or previous vehicle section.

### `[stamper]`

Defines a ticket-stamping or passenger interaction point. The common record
contains an index and local transform values.

### `[illumination_interior]`

Assigns interior illumination groups to cabin areas. It uses repeated numeric
records whose exact meaning depends on the model's interior mesh.

## 5. Path CFGs

These are referenced by BUS `[paths]`.

### `[pathpnt]`

Three local coordinates for a passenger path point:

1. x
2. y
3. z

Repeated points form walking/boarding paths. Additional values may follow for
direction, width, or path flags in newer files.

### `[stepsoundpack]`

Count-prefixed list of step-sound filenames:

```text
[stepsoundpack]
<count>
<sound 1>
...
<sound count>
```

Repeated blocks can assign different sound packs to different floors or path
regions.

## 6. Sound CFGs

### `[loopsound]`

Defines one looping sound. The common record contains:

1. Sound filename
2. Sample rate
3. Control variable, often `velocity`
4. Maximum/control range
5. Loop or mode flag

### `[3d]`

Defines 3D sound placement or attenuation parameters. The common values begin
with local coordinates and are followed by sound-distance or direction data.
Exact record length is sound-file specific.

## 7. AI, traffic, and network CFGs

### `[ailist]`

Defines an AI vehicle list. The common opening values identify a list/type and
count, followed by vehicle BUS filenames. Preserve vehicle order and all
weights/quotas following filenames.

### `[aigroup_2]`

Defines a named AI group and its weighted vehicle entries. A typical entry is:

```text
vehicles\SomeVehicle.bus    <weight>
```

### `[aigroup_depot]`

Defines a depot/group name and its associated operator or location. It is
usually followed by depot-specific vehicle and paint/registration records.

### `[busstop]`

Defines one bus stop. The common record begins with a stop name, station/index
identifiers, position or radius values, and flags. Repeated records create the
stop database.

### `[signalroute]`

Starts a traffic-signal route record. It is followed by signal identifiers and
entry/connection data.

### `[StnLink]`

Defines a station/link connection. The common values include distance or
timing, station identifiers, local offsets, and link/flag values. Preserve
the complete record because the layout is map-family-specific.

## 8. Environment and world CFGs

### `[sky_textures]`

List of sky texture filenames, normally one per line. Order maps to the
environment's sky/time-of-day stages.

### `[cloudtype]`

Four common values:

1. Cloud type name
2. Cloud texture filename
3. Distance/scale
4. Cloud classification

Repeated blocks define the available cloud types.

### `[startdate]`

One date in `YYYYMMDD` form used as the environment/calendar start date.

### `[currency]`

Defines currency name and a mode/precision flag, followed by repeated `[coin]`
records.

### `[coin]`

Two values:

1. Coin mesh filename
2. Coin denomination

## 9. Controls and game configuration

### `[game]`

Marker for game-level keyboard/controller bindings. It is commonly followed by
repeated `[entry]` blocks.

### `[ctrl]`

Defines one controller device. The common values are device name and an
enable/type flag.

### `[axis]`

Defines one controller axis mapping. The common record includes device/axis
indices, inversion, scale, dead zone, and range values.

### `[entry]`

In keyboard/game CFGs, `[entry]` defines an action binding rather than a
passenger-cabin entry. The common values are action name, key/button code, and
modifier or mode flags. Interpret it according to the containing CFG type.

## 10. Global and metadata CFGs

### `[friendlyname]`

One display name for the configuration.

### `[description] ... [end]`

Free-form description terminated by `[end]`.

### `[usrinfo]`

Installation/user metadata. This may contain serial numbers, usernames, local
installation paths, or other sensitive values. Do not copy it into source
control or new documentation; redact it when sharing diagnostics.

## 11. Recommended logical organization

Do not move the supplied files in place without updating every relative
reference. For a new bus project, use a logical layout such as:

```text
vehicle/
  bus/
    citybus.bus
  model/
    citybus.cfg
    meshes/
    textures/
    collision/
  passenger/
    citybus_passengercabin.cfg
  paths/
    citybus_paths.cfg
  sound/
    citybus_sound.cfg
  scripts/
    *.osc
    *.txt
  surfaces/
    asphalt.bmp.cfg
    wet_road.dds.cfg
  environment/
    sky.cfg
    clouds.cfg
  ai/
    ailist.cfg
    depots.cfg
  controls/
    keyboard.cfg
```

The existing corpus can be classified without moving it:

- Large CFGs containing `[mesh]`, `[CTC]`, `[LOD]`, and texture sections:
  **model CFGs**.
- Image-named CFGs containing `[surface]`, `[moisture]`, or `[puddles]`:
  **surface/material CFGs**.
- Files containing `[drivpos]`, `[passpos]`, `[entry]`, and `[exit]`:
  **passenger-cabin CFGs**.
- Files containing `[pathpnt]` or `[stepsoundpack]`: **path CFGs**.
- Files containing `[loopsound]`: **sound CFGs**.
- Files containing `[ailist]`, `[aigroup_*]`, `[busstop]`, `[signalroute]`,
  or `[StnLink]`: **AI/network CFGs**.
- Files containing `[sky_textures]`, `[cloudtype]`, or `[startdate]`:
  **environment CFGs**.

## 12. Conversion checklist

1. Identify the containing configuration type before interpreting a keyword.
2. Preserve relative paths and filename case.
3. Keep repeated records in their original order.
4. Validate count-prefixed blocks after adding or removing entries.
5. Keep mesh, collision mesh, texture, CTC, and script indices synchronized.
6. Keep passenger entries/exits aligned with the cabin and door scripts.
7. Keep path points ordered and continuous.
8. Keep surface classification files beside, or referenced relative to, the
   textures they describe.
9. Exclude `[usrinfo]` values from commits and shared diagnostics.
10. Add one configuration family at a time and test the vehicle after each
    family is connected.
