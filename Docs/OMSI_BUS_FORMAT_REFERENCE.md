# OMSI `.bus` configuration reference

This document describes the `.bus` format inferred from the `.bus` examples in
this folder and the supplied syntax notes. It is a practical authoring
reference, not a replacement for the OMSI executable's parser.

## 1. Parser rules

### 1.1 Keyword recognition

OMSI scans the file for keywords. A keyword is recognized only when it is the
complete contents of a line:

```text
[keyword]
```

The following are **not** recognized as the same keyword:

```text
    [keyword]
[keyword] comment
[keyword]##
```

Everything that is not a recognized keyword is ignored while the parser is
searching. This makes ordinary comments safe, but a comment must not be
appended to a keyword line.

Keywords are generally case-sensitive and values are read from the following
lines in order. After a block has consumed its values, scanning resumes at the
next keyword.

### 1.2 Block types

The examples use three block shapes:

1. **Fixed-count block**: a keyword is followed by a known number of values.
2. **Count-prefixed block**: the first value is a count, followed by that many
   filenames or records.
3. **Terminated block**: values continue until `[end]`.

Repeated keywords create arrays. The order of repeated records matters for
arrays such as scripts, meshes, cameras, and axles.

### 1.3 Disabling commands

For configuration types that support it, a keyword can be disabled by adding
any extra character to its line, or by surrounding multiple lines with:

```text
-<DISABLED>-
...
-<ENABLED>-
```

The disabled region is ignored. Do not indent a keyword unless the intent is
to disable it.

## 2. Top-level vehicle identity and metadata

### `[friendlyname]`

Three lines:

1. Manufacturer
2. Vehicle type/name
3. Default paint name used when no texture replacement is active

### `[description] ... [end]`

Free-form multi-line description. `[end]` terminates the block. The text is
shown as vehicle information and may contain blank lines and formatting.

### `[type]`

One integer identifying the vehicle category. Values in the examples include
normal buses and special/rail vehicles; preserve the value used by the
vehicle family being converted.

### `[fixed]`

A marker used by some files to select fixed/non-articulated behavior. It has no
value lines in the examples.

## 3. Registration and odometer

### `[number]`

One filename containing registration-number lists, normally an `.org` file.
The first line of a list may identify a paint scheme; following lines are
allowed vehicle numbers. An empty scheme name represents the main list.

### `[registration_automatic]`

Enables automatic registration generation. The next line is the registration
prefix, for example `B-V ` or `D-A `. Some files leave it empty.

### `[registration_list]`

One registration-list filename. The list index corresponds to the selected
number-list entry.

### `[registration_free]`

Enables free registration selection. It has no required value line in the
examples.

### `[kmcounter_init]`

Two values:

1. Initial year
2. Initial odometer value in kilometres

## 4. Assets and scripts

### `[sound]` and `[sound_ai]`

One sound configuration filename each. `[sound]` is used for the player
vehicle; `[sound_ai]` is used for AI operation.

### `[model]`

One model configuration filename.

### `[paths]`

One path/route configuration filename.

### `[passengercabin]`

One passenger-cabin configuration filename.

### `[varnamelist]`

Count-prefixed list of script variable-list files:

```text
[varnamelist]
<count>
<file 1>
...
<file count>
```

### `[stringvarnamelist]`

Same structure as `[varnamelist]`, but for string variables.

### `[script]`

Count-prefixed list of `.osc` script files. Script order is significant because
the scripts are loaded and updated in that order.

### `[constfile]`

Count-prefixed list of script constant files. Keep the order aligned with the
script/variable families that consume them.

### `[scriptshare]`

A marker used by some vehicle families to share script resources. The examples
do not show a universal value layout; treat its following lines as
family-specific.

## 5. Cameras and views

Coordinates are local vehicle coordinates. The common camera record has seven
values:

1. `x` lateral
2. `y` longitudinal
3. `z` vertical
4. eye/pivot distance
5. field of view in degrees
6. horizontal view angle in degrees
7. vertical view angle in degrees

### `[add_camera_driver]`

Adds one driver camera using the seven-value record above. Camera index is the
order of appearance.

### `[add_camera_pax]`

Adds one passenger camera using the same seven-value record.

### `[add_camera_reflexion]` and `[add_camera_reflexion_2]`

Adds a reflection/mirror camera. The record uses the same seven camera values,
with the two keywords allowing separate reflection-camera groups.

### `[set_camera_std]`

One integer selecting the default driver-camera index.

### `[set_camera_outside_center]`

Three values defining the outside-camera look-at center: local `x`, `y`, and
`z`.

### `[view_schedule]`

One or more labelled view entries used by schedule/route displays. The
following text is family-specific and is not a physics value.

### `[view_ticketselling]`

One or more labelled ticket-selling view entries. The leading number in the
examples is a view/attachment index followed by a human-readable label.

## 6. Basic physical data

### `[mass]`

One value: kerb/dry vehicle mass in tonnes. OMSI adds passenger mass at
runtime, so exclude passengers, driver, luggage, and consumables when possible.

### `[momentofintertia]`

Three mass moments of inertia in `t*m²`, in this order:

1. About the x axis (lateral/transverse)
2. About the y axis (longitudinal)
3. About the z axis (vertical)

The spelling `momentofintertia` is intentional and must be preserved.

### `[cog]`

Three local coordinates for the centre of gravity: `x`, `y`, and `z`.

### `[schwerpunkt]`

One value: height of the centre of gravity above the ground in metres. This is
the common German-language counterpart to the z component of `[cog]`.

Use one convention consistently in a vehicle. For the OpenBus model, the
current target is a 1.0 m aggregate CG height.

### `[boundingbox]`

Six values, one per line:

1. Width without mirrors, metres
2. Length, metres
3. Height, metres
4. Bounding-box centre offset on local x, metres
5. Bounding-box centre offset on local y, metres
6. Bounding-box centre offset on local z, metres

The box is a coarse collision volume, not the visual mesh.

### `[rollwiderstand]`

One rolling-resistance force in newtons. The examples describe it as a
constant force.

### `[rot_pnt_long]`

One longitudinal coordinate in metres for the point around which the vehicle
turns.

### `[inv_min_turnradius]`

One inverse minimum turning radius in `1/m`. The supplied calculation uses:

```text
inverse_radius = tan(maximum_steering_angle) / wheelbase
```

The source notes also describe first deriving the maximum steering angle from
wheelbase and the outer-wheel turning radius. Use radians for trigonometric
functions in code and metres for all distances.

### `[rowdy_factor]`

One or more family-specific roughness/ride-behaviour factors. Examples include
negative and positive values; preserve the source vehicle's intended range.

## 7. Axles, wheels, suspension, and AI vehicle height

### `[newachse]`

Creates one axle. The following named records define the axle:

| Name | Meaning | Typical unit |
| --- | --- | --- |
| `achse_long` | Longitudinal axle position | m |
| `achse_maxwidth` | Maximum tyre contact width across the axle | m |
| `achse_minwidth` | Minimum/inner tyre width or track limit | m |
| `achse_raddurchmesser` | Wheel diameter | m |
| `achse_feder` | Spring stiffness per side | kN/m |
| `achse_maxforce` | Maximum axle/suspension load | kN |
| `achse_daempfer` | Damper coefficient | kN*s/m |
| `achse_antrieb` | Driven-axle flag (`0`/`1`) | boolean |

The first axle can establish the wheel reference used by drivetrain speed
calculations. Define driven/wheel-reference axles in the order expected by the
vehicle's gearbox scripts.

### `[boogies]`

Defines bogie-related behaviour. Its first value is commonly a bogie
longitudinal/geometry parameter; later values are family-specific. Some files
place explanatory text and `[sinus]` blocks nearby, so parse only exact
keywords.

### `[ai_deltaheight]`

One or more AI height corrections, normally one per relevant axle or vehicle
section. Values are metres; comments may appear between records.

### `[ai_brakeperformance]`

AI braking parameters. The examples commonly contain three values describing
braking strength and related thresholds. Exact interpretation is script/model
dependent; preserve the source vehicle's sequence.

### `[ai_veh_type]`

One integer AI vehicle category.

### `[sinus]`

Sinusoidal body/ride oscillation parameters. Examples commonly contain three
values: amplitude, frequency/period-related value, and phase or damping-related
value. The exact interpretation is model/script dependent.

## 8. Couplings and attachments

### `[coupling_front]` and `[coupling_back]`

Define a coupling point. The common record contains local position/orientation
values for the coupling, usually including local `x`, longitudinal `y`, and a
coupling type/flag. Exact record length varies by vehicle generation.

### `[coupling_front_character]`

Defines front-coupling articulation limits. The common three values are:

1. Maximum/minimum alpha range in degrees
2. Minimum beta/down angle in degrees
3. Maximum beta/up angle in degrees

### `[couple_back]`

References or configures a vehicle coupled behind the current vehicle. A
filename and boolean are common, but optional family-specific lines may follow.

### `[couple_front_open_for_sound]`

Marker enabling front-coupling sound behaviour. Some files place explanatory
text after the keyword; it has no universal numeric payload.

### `[control_cable_front]` and `[control_cable_back]`

Create an electrical/control connection at the front or rear coupling. The
common five-line record is:

1. Side: `L`, `R`, or `C`
2. Cable number
3. Read variable identifier
4. Write variable identifier
5. Coupling variable identifier

`L` on one vehicle connects to `R` on the other; `C` connects centre cables.
The coupling variable determines whether the link remains active.

### `[new_attachment]`

Creates a named model attachment. Attachments commonly use a name followed by
translation/rotation attachment names and local transform values. The exact
record is model-family specific and should be read together with the referenced
model configuration.

### `[contact_shoe]`

Defines a trolley/rail contact shoe. The examples use local position and
orientation/height values; exact record length depends on the trolley vehicle.

### `[rail_body_osc]`

Rail-body oscillation parameters, commonly including a longitudinal reference,
mode/count, and strength/frequency values.

## 9. Rail/trolley and special vehicle data

The following sections occur mainly in trolleybuses, trams, and rail-capable
vehicles:

- `[contact_shoe]` — contact-shoe geometry and electrical/rail contact.
- `[rail_body_osc]` — rail-body oscillation.
- `[boogies]` — bogie geometry and ride behaviour.
- `[coupling_front_character]` — articulation limits.
- `[control_cable_front]` / `[control_cable_back]` — electrical coupling links.

Do not add these sections to a normal bus unless its model and scripts consume
them.

## 10. Minimal conventional bus skeleton

This is an ordering example, not a complete runnable vehicle:

```text
[friendlyname]
Manufacturer
Vehicle type
Default paint

[description]
Vehicle description
[end]

[number]
registrations.org

[sound]
sound\sound.cfg

[sound_ai]
sound\sound_ai.cfg

[model]
model\model.cfg

[paths]
model\paths.cfg

[passengercabin]
model\passengercabin.cfg

[varnamelist]
1
script\main_varlist.txt

[stringvarnamelist]
1
script\main_stringvarlist.txt

[script]
1
script\main.osc

[constfile]
1
script\main_constfile.txt

[mass]
11.5

[momentofintertia]
500
200
500

[boundingbox]
2.55
12.0
3.5
0
0
1.75

[schwerpunkt]
1.0

[rollwiderstand]
1000

[rot_pnt_long]
0

[inv_min_turnradius]
0.14

[newachse]
achse_long
3.45
achse_maxwidth
2.30
achse_minwidth
1.15
achse_raddurchmesser
1.01
achse_feder
250
achse_maxforce
90
achse_daempfer
16
achse_antrieb
1
```

## 11. Authoring checklist

1. Keep every active keyword alone at the start of its line.
2. Count every count-prefixed list exactly; a wrong count shifts parsing of the
   remainder of the file.
3. Terminate `[description]` with `[end]`.
4. Keep model, path, passenger-cabin, sound, script, and variable filenames
   relative to the vehicle directory.
5. Use metres, kilograms/tonnes, seconds, degrees, and radians exactly as
   specified for each section.
6. Define mass, CG, inertia, bounding box, rolling resistance, turning radius,
   and axles as one internally consistent physical system.
7. Define the driven/reference axle before dependent gearbox calculations.
8. Keep coupling cable numbering paired between front and rear vehicles.
9. Treat undocumented trailing values as vehicle-family-specific rather than
   guessing their meaning.
10. Test a new file first with identity, model, physics, and one script; add
    optional AI, coupling, rail, and camera sections incrementally.

