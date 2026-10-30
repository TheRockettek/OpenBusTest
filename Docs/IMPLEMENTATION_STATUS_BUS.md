# BUS and bus-model configuration: incomplete support

Scope: the vehicle `.bus` file, its model CFG, and the passenger-cabin/path files referenced by it. The lists below focus on known incomplete or absent behavior; they are not a list of all supported records. Core vehicle loading, model assembly, animations, wheel configuration, cameras, and the current MAN DL05/E400 paths are active.

## BUS file (`.bus`)

| Keyword / family | Status | Current boundary |
| --- | --- | --- |
| `[fixed]`, `[scriptshare]` | Not implemented | BUS-level fixed/static selection and shared-script semantics are not applied. |
| `[view_schedule]`, `[view_ticketselling]` | Parsed only | The loader diagnoses the unsupported selection behavior but does not retain/use the full view labels. |
| `[cog]` | Partial | Its z coordinate supplies the physics height when `[schwerpunkt]` is absent. OpenBus gives `[schwerpunkt]` precedence and warns on disagreement; nonzero x/y offsets are retained and warned about but not applied. Native precedence is unverified. |
| `[rollwiderstand]` | Active subset | A non-negative authored force in newtons opposes longitudinal motion; negative values are rejected, and when absent the previous mass-scaled OpenBus fallback is retained. Native OMSI physics parity remains unverified. |
| `[rot_pnt_long]` | Parsed only | The longitudinal rotation point is retained and diagnosed but does not affect vehicle physics. |
| `[ai_deltaheight]` | Active subset | Applied as a vertical offset to AI vehicle placement, including map-route updates; player vehicles are unchanged. Native OMSI sign/unit parity still needs live validation. |
| `[rowdy_factor]`, `[boogies]`, `[ai_brakeperformance]`, `[ai_veh_type]`, `[sinus]` | Unsupported | Recognized with explicit diagnostics; family-specific behavior is absent. |
| BUS/vehicle `[fixed]`, `[scriptshare]`, `[new_attachment]` | Unsupported | Recognized with explicit diagnostics; their vehicle-level runtime behavior is absent. |
| `[coupling_front]`, `[coupling_back]`, `[coupling_front_character]`, `[couple_back]`, `[couple_front_open_for_sound]` | Unsupported | Recognized with explicit diagnostics; coupling and multi-vehicle behavior are absent. |
| `[control_cable_front]`, `[control_cable_back]` | Unsupported | Recognized with explicit diagnostics; no cross-vehicle control/electrical transfer. |
| `[contact_shoe]`, `[rail_body_osc]` | Unsupported | Recognized with explicit diagnostics; rail/trolley-specific behavior is absent. |
| `[sound_ai]` | Partial | AI vehicle initialization selects the authored AI sound bank and falls back to `[sound]` when absent. AI ambient/spatial playback parity remains unverified. See [audio status](IMPLEMENTATION_STATUS_AUDIO.md). |
| `[registration_automatic]`, `[registration_free]`, `[kmcounter_init]` | Partial | Startup selection/number seeding and odometer initialization exist; persistence and the full OMSI selection UI do not. |

Other BUS gaps include the complete configurable input map and vehicle-family-specific view systems. The player runtime currently dispatches physical W/A/S/D transitions to scripts rather than implementing every OMSI input-context record.

Both script backends preserve the float-stack condition across `{if}` / `{else}` branches and macro calls. This is required by the DL05 front-door interlock: the door macro combines the caller's retained condition with the ramp interlock. The `bus_20h-switch` control enables the stop brake, not the separate parking brake; normal electrical, speed, and ramp interlocks still apply.

## Bus model CFG

DirectX `.x` meshes (including the DL05 Atron AFR4) provide vehicle-axis bounds for
frustum culling and picking after their frame transforms are applied. Their UVs use
the same internal V convention as OBJ/O3D, avoiding an inverted texture in the
vehicle renderer while preserving scenery texture orientation.
Static `.x` material diffuse colour, alpha, texture, specular highlight, and emissive
colour are carried into vehicle and scenery shading. Exact native lighting parity is
not verified.

Auxiliary material maps can reference live script textures with `\S:<slot>`
(including the DL05 VDV and Atron transparency masks), rather than treating
those references as disk filenames. Both script backends allow drawing between
`STLock` and `STUnlock`, retain RGBA channel order for rectangles, and implement
`GetFontIndex`, `TextLength`, and `STTextOut` using the installed OFT bitmap fonts.
`STTextOut`'s Full Colour argument selects font colours; zero uses `STSetColor`,
not a transparent packed colour. Font/layout parity beyond this subset remains
unverified. Font-directory indexes refresh when directory timestamps change,
including after definitions are added or removed. Timestamp resolution can
limit detection, in-place changes to OFT font names do not refresh the index,
and fonts already cached by a script runtime are not live-reloaded.

Vehicle `[matl_nightmap]` layers normally fade in with darkness (`nightlighta` /
`envir_brightness`). Nightmaps on script-selected material items, and on meshes gated
by `[visible]`, are drawn at full strength in daylight, because their visibility
variable is the switch. This lets the DL05 `LM_*` warning lamps (`panel_LM` nightmap)
light up when their variables become active.

The DL05 window panes are authored double-sided, and OpenBus draws the same glass
layers OMSI does. Measured from the driver seat, the glass reduces scene brightness
by only about 12–21%. From outside, the windows look dark mainly because the interior
behind them uses a fixed ambient level, not because of the glass. No glass-specific
override is applied.

| Keyword / family | Status | Current boundary |
| --- | --- | --- |
| `[matl_bumpmap]`, `[matl_envmap]`, `[matl_freetex]`, `[matl_lightmap]`, `[matl_nightmap]` | Partial | Loading/shader/material paths exist, but controller, blending, binding, or visual-parity details remain incomplete. Environment-map behavior still needs representative visual validation. |
| `[matl_transmap]` with `[matl_alpha]` mode `2` | Active subset | Transmap alpha sampling and mode-2 depth writes are active; `[matl_noZwrite]` suppresses writes. An OpenGL overlap probe covers both modes, but representative OMSI visual parity remains unverified. |
| `[matl_item]` | Partial | Item material states are selected from their controller through the vehicle runtime, including supported auxiliary material properties. Independent texture replacement and full native item scope are not implemented. |
| `[CTC]`, `[CTCTexture]` | Partial | `[CTCTexture]` substitutions are applied case-insensitively to base mesh texture names. `[CTC]` template/paint-scheme selection and full runtime substitution remain incomplete. |
| `[texchanges]`, `[newtexchangemaster]`, `[entries]` | Not implemented | Texture-change tables and replacement selection are not loaded/applied. |
| `[rendertype]` | Not supported for BUS models | The loader currently restricts this keyword to scenery-object configuration; BUS/vehicle model contexts are diagnosed as invalid. Scenery support is tracked in [scenery status](IMPLEMENTATION_STATUS_SCENERY.md). |
| `[fixed]` | Active subset | Suppresses inheritance of an `[animparent]` transform while retaining the part's own animations. OMSI parity for other `[fixed]` effects remains unverified. |
| `[absheight]` | Partial | Scenery height placement is active; broader model-family transform semantics are not implemented. |
| `[light_enh]`, `[light_enh_2]`, `[spotlight]` | Partial | Records are parsed/stored, including `[light_enh_2]`'s complete numeric payload with or without its optional bitmap, but full geometry and render-order behavior are missing. Basic `[illumination_interior]` / `[interiorlight]` point-light shading includes distance attenuation; a GPU probe covers color isolation, zero strength, and near/far attenuation, but OMSI intensity parity is unverified. |
| `[smoke]` | Parsed only | Fields are consumed for alignment; no exhaust/particle system is created. |
| `[VFDmaxmin]`, `[tex_detail_factor]` | Parsed only | Display bounds and detail scaling do not currently alter rendering. |
| `[origin_from_mesh]` | Partial | The origin operation is retained, but becomes identity when mesh transformation metadata is unavailable. |
| `[texttexture]`, `[texttexture_enh]` | Partial | Dynamic surfaces and horizontal alignment/grid spacing are active; fallback font rasterization and remaining enhanced-format flags/layout/alpha details differ from OMSI. |
| `[illumination_interior]`, `[interiorlight]` | Partial | Controller-driven point-light shading, mesh assignments, and distance attenuation are active. GPU coverage exercises color, zero strength, and attenuation; controller integration with representative assets and OMSI intensity parity remain unverified. |
| `[mouseevent]` | Partial | Transformed-triangle picking, captured press/held-drag/release, and player W/A/S/D event transitions are active. Handler lookup and dispatch normalize authored punctuation consistently (including DL05 `bus_20h-switch`); drag/release handlers are optional. Configurable key maps and scenery-object mouse dispatch are not complete. |

## Passenger-cabin and path CFGs

`[drivpos]`, `[passpos]`, `[entry]`, `[exit]`, basic `[pathpnt]` / `[pathlink]`, `[stepsoundpack]`, and room-height transitions are loaded. Passenger seats now have a deterministic per-vehicle occupancy service. Scheduled AI buses stop at matched timetable calls; station-ID-only trips can use complete directed station-link routes. Nearby player buses also service mapped calls when stopped and aligned with the route. Scheduled waiting queues render independently of the ambient pedestrian cap; a reserved boarder walks visually from the queue toward the bus flank during the boarding interval.

- `[linkToNextVeh]`, `[linkToPrevVeh]`, `[stamper]`, and `[noticketsale]` have no complete runtime behavior.
- Boarding into configured seat slots and next-stop alighting are active. Boarding movement is only a roadside visual approximation; cabin path-link traversal and alighting movement are absent. Passenger mass is not applied, and seated humans are not rendered.
- Map ticket-pack and currency coin records populate the `getticketname`, `getticketvalue`, and `givechangecoin` script macros for player and AI vehicles. Ticket requests, card/cash transactions, ticket-selling UI, and animated/audible/physical coin dispensing are absent.
- Extended path fields and actual step-sound playback/route use remain incomplete.
- HOF, stop-announcement, and arrival-board integration remains absent. Scheduled AI bus scripts receive timetable stop names, profile-selected arrival/departure times, and route-based delay; `GetTTTerminusIndex` returns the zero-based index of the last mapped stop on the bus route, or `-1` when no stops are mapped.

## Evidence and related pages

Implementation references: [VehicleConfigLoader.cpp](../src/config/VehicleConfigLoader.cpp), [ModelConfigLoader.cpp](../src/config/ModelConfigLoader.cpp), [BusSimulation.cpp](../src/physics/BusSimulation.cpp), and [RenderLoop.cpp](../src/app/RenderLoop.cpp). Regression coverage includes [VehicleConfigProbe.cpp](../tests/VehicleConfigProbe.cpp), [ModelConfigProbe.cpp](../tests/ModelConfigProbe.cpp), and [ViewpointProbe.cpp](../tests/ViewpointProbe.cpp). The layout reference is [OMSI_BUS_FORMAT_REFERENCE.md](OMSI_BUS_FORMAT_REFERENCE.md); this status is also summarized in the [configuration status index](CONFIGURATION_IMPLEMENTATION_STATUS.md).
