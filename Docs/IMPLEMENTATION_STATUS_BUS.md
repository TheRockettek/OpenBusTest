# BUS and bus-model configuration: incomplete support

Scope: the vehicle `.bus` file, its model CFG, and the passenger-cabin/path files referenced by it. The lists below focus on known incomplete or absent behavior; they are not a list of all supported records. Core vehicle loading, model assembly, animations, wheel configuration, cameras, and the current MAN DL05/E400 paths are active.

## BUS file (`.bus`)

| Keyword / family | Status | Current boundary |
| --- | --- | --- |
| `[fixed]`, `[scriptshare]` | Not implemented | BUS-level fixed/static selection and shared-script semantics are not applied. |
| `[view_schedule]`, `[view_ticketselling]` | Parsed only | The loader recognizes the records, but does not retain/use the full view labels and selection behavior. |
| `[cog]` | Partial | Not consistently reconciled with `[schwerpunkt]`; use of the two CG forms together is not established. |
| `[rollwiderstand]` | Active subset | A non-negative authored force in newtons opposes longitudinal motion; negative values are rejected, and when absent the previous mass-scaled OpenBus fallback is retained. Native OMSI physics parity remains unverified. |
| `[rot_pnt_long]` | Parsed only | The longitudinal rotation point is consumed but does not currently affect vehicle physics. |
| `[ai_deltaheight]` | Parsed only | No AI vehicle height correction is applied. |
| `[rowdy_factor]`, `[boogies]`, `[ai_brakeperformance]`, `[ai_veh_type]`, `[sinus]` | Not implemented | Family-specific AI/drivetrain behavior is absent. |
| `[coupling_front]`, `[coupling_back]`, `[coupling_front_character]`, `[couple_back]`, `[couple_front_open_for_sound]` | Not implemented | Coupling and multi-vehicle behavior are absent; articulated multi-body physics is not implemented. |
| `[control_cable_front]`, `[control_cable_back]` | Not implemented | No cross-vehicle control/electrical variable transfer. |
| `[contact_shoe]`, `[rail_body_osc]` | Not implemented | Rail/trolley-specific behavior is absent. |
| `[sound_ai]` | Parsed only | Its referenced path is retained, but there is no AI sound-bank selection/runtime. See [audio status](IMPLEMENTATION_STATUS_AUDIO.md). |
| `[registration_automatic]`, `[registration_free]`, `[kmcounter_init]` | Partial | Startup selection/number seeding and odometer initialization exist; persistence and the full OMSI selection UI do not. |

Other BUS gaps include the complete configurable input map and vehicle-family-specific view systems. The player runtime currently dispatches physical W/A/S/D transitions to scripts rather than implementing every OMSI input-context record.

Both script backends preserve the float-stack condition across `{if}` / `{else}` branches and macro calls. This is required by the DL05 front-door interlock: the door macro combines the caller's retained condition with the ramp interlock. The `bus_20h-switch` control enables the stop brake, not the separate parking brake; normal electrical, speed, and ramp interlocks still apply.

## Bus model CFG

DirectX `.x` meshes (including the DL05 Atron AFR4) provide vehicle-axis bounds for
frustum culling and picking after their frame transforms are applied. Their UVs use
the same internal V convention as OBJ/O3D, avoiding an inverted texture in the
vehicle renderer while preserving scenery texture orientation.

| Keyword / family | Status | Current boundary |
| --- | --- | --- |
| `[matl_bumpmap]`, `[matl_envmap]`, `[matl_freetex]`, `[matl_lightmap]`, `[matl_nightmap]` | Partial | Loading/shader/material paths exist, but controller, blending, binding, or visual-parity details remain incomplete. Environment-map behavior still needs representative visual validation. |
| `[matl_transmap]` with `[matl_alpha]` mode `2` | Partial | Transmap alpha sampling exists; blended depth behavior is not OMSI-verified and currently lacks the documented depth-write behavior. |
| `[matl_item]` | Parsed/no-op | It does not yet define a complete, independently applied material-item scope. |
| `[CTC]`, `[CTCTexture]` | Partial | Mappings are retained, but complete runtime paint-scheme selection/substitution is absent. |
| `[texchanges]`, `[newtexchangemaster]`, `[entries]` | Not implemented | Texture-change tables and replacement selection are not loaded/applied. |
| `[rendertype]` | Not supported for BUS models | The loader currently restricts this keyword to scenery-object configuration; BUS/vehicle model contexts are diagnosed as invalid. Scenery support is tracked in [scenery status](IMPLEMENTATION_STATUS_SCENERY.md). |
| `[fixed]` | Active subset | Suppresses inheritance of an `[animparent]` transform while retaining the part's own animations. OMSI parity for other `[fixed]` effects remains unverified. |
| `[absheight]` | Partial | Scenery height placement is active; broader model-family transform semantics are not implemented. |
| `[light_enh]`, `[light_enh_2]`, `[spotlight]` | Partial | Records are parsed/stored, including `[light_enh_2]`'s complete numeric payload with or without its optional bitmap, but full geometry, falloff, and render-order behavior is missing. Basic `[illumination_interior]` / `[interiorlight]` emission is active but does not establish full lighting parity. |
| `[smoke]` | Parsed only | Fields are consumed for alignment; no exhaust/particle system is created. |
| `[VFDmaxmin]`, `[tex_detail_factor]` | Parsed only | Display bounds and detail scaling do not currently alter rendering. |
| `[origin_from_mesh]` | Partial | The origin operation is retained, but becomes identity when mesh transformation metadata is unavailable. |
| `[texttexture]`, `[texttexture_enh]` | Partial | Dynamic surfaces and horizontal alignment/grid spacing are active; fallback font rasterization and remaining enhanced-format flags/layout/alpha details differ from OMSI. |
| `[illumination_interior]`, `[interiorlight]` | Partial | Controller-driven emissive color and mesh assignments are active; positional falloff and full OMSI behavior are not. |
| `[mouseevent]` | Partial | Transformed-triangle picking, captured press/held-drag/release, and player W/A/S/D event transitions are active. Handler lookup and dispatch normalize authored punctuation consistently (including DL05 `bus_20h-switch`); drag/release handlers are optional. Configurable key maps and scenery-object mouse dispatch are not complete. |

## Passenger-cabin and path CFGs

`[drivpos]`, `[passpos]`, `[entry]`, `[exit]`, basic `[pathpnt]` / `[pathlink]`, `[stepsoundpack]`, and room-height transitions are loaded. They do not constitute a passenger simulation:

- `[linkToNextVeh]`, `[linkToPrevVeh]`, `[stamper]`, and `[noticketsale]` have no complete runtime behavior.
- Boarding, alighting, passenger movement/mass, route/path selection, and ticket interactions are absent.
- Extended path fields and actual step-sound playback/route use remain incomplete.
- HOF, timetable, stop-announcement, and arrival-board integration is absent; documented route/timetable script lookups still use safe placeholder behavior.

## Evidence and related pages

Implementation references: [VehicleConfigLoader.cpp](../src/VehicleConfigLoader.cpp), [ModelConfigLoader.cpp](../src/ModelConfigLoader.cpp), [BusSimulation.cpp](../src/BusSimulation.cpp), and [RenderLoop.cpp](../src/RenderLoop.cpp). Regression coverage includes [VehicleConfigProbe.cpp](../tests/VehicleConfigProbe.cpp), [ModelConfigProbe.cpp](../tests/ModelConfigProbe.cpp), and [ViewpointProbe.cpp](../tests/ViewpointProbe.cpp). The layout reference is [OMSI_BUS_FORMAT_REFERENCE.md](OMSI_BUS_FORMAT_REFERENCE.md); this status is also summarized in the [configuration status index](CONFIGURATION_IMPLEMENTATION_STATUS.md).
