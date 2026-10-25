# Configuration keywords with incomplete support

This is a consolidated index of configuration keywords and record families
whose runtime behavior is known to be absent, parsed-only, partial, or
unverified in OpenBus. It covers the gaps documented in the family-specific
status pages; it is not a catalog of every keyword in OMSI.

**Partial** means that some behavior works but the listed keyword is not fully
implemented. **Parsed only** means that data is read or retained without the
corresponding runtime behavior. **Not implemented** means no runtime behavior
is provided. Details and validation boundaries are in the linked status pages.

## Bus model CFG (`OPENBUS_MODEL_CONFIG`)

These are the known bus-model keyword gaps:

| Keyword(s) | Status | Details |
| --- | --- | --- |
| `[matl_bumpmap]`, `[matl_envmap]`, `[matl_freetex]`, `[matl_lightmap]`, `[matl_nightmap]` | Partial | Material loading and shader paths exist, but some controller, blending, binding, or visual-parity behavior remains incomplete. |
| `[matl_transmap]`, `[matl_alpha]` | Partial | Transmap sampling and alpha mode 2 depth writes work; remaining native visual parity is unverified. |
| `[matl_item]` | Partial | Controller-based item selection works; independent texture replacement and full native item scope do not. |
| `[CTC]`, `[CTCTexture]` | Partial | Base-texture substitutions work; template/paint-scheme selection and full runtime replacement do not. |
| `[texchanges]`, `[newtexchangemaster]`, `[entries]` | Not implemented | Texture-change tables and replacement selection are not loaded or applied. |
| `[rendertype]` | Unsupported in bus models | The keyword is restricted to scenery-object model contexts. |
| `[fixed]`, `[absheight]` | Partial | Only documented subsets of their model-family transform behavior are active. |
| `[light_enh]`, `[light_enh_2]`, `[spotlight]` | Partial | Records are parsed, but full geometry and render-order behavior are missing. |
| `[smoke]` | Parsed only | No exhaust or particle system is created. |
| `[VFDmaxmin]`, `[tex_detail_factor]` | Parsed only | Display bounds and detail scaling do not affect rendering. |
| `[origin_from_mesh]` | Partial | The origin operation is retained, but is identity when mesh transform metadata is unavailable. |
| `[texttexture]`, `[texttexture_enh]` | Partial | Dynamic surfaces work; fallback font rasterization and some format/layout/alpha behavior differ from OMSI. |
| `[illumination_interior]`, `[interiorlight]` | Partial | Point-light shading works, but controller integration and native intensity parity remain incomplete or unverified. |
| `[mouseevent]` | Partial | Vehicle picking and basic event transitions work; configurable key maps and scenery-object dispatch are incomplete. |
| `[terrainmapping]` | Not implemented | Terrain/material mapping is not dispatched by the shared model-config loader. |

See [BUS and bus-model status](IMPLEMENTATION_STATUS_BUS.md#bus-model-cfg)
and [scenery status](IMPLEMENTATION_STATUS_SCENERY.md) for full limitations.

## Vehicle BUS file

| Keyword(s) | Status | Details |
| --- | --- | --- |
| `[fixed]`, `[scriptshare]` | Not implemented | BUS-level fixed/static selection and shared-script semantics are not applied. |
| `[view_schedule]`, `[view_ticketselling]` | Parsed only | Full view labels and selection behavior are not retained or used. |
| `[cog]` | Partial | Combined use with `[schwerpunkt]` is not consistently reconciled. |
| `[rollwiderstand]`, `[ai_deltaheight]`, `[sound_ai]` | Partial | Working subsets exist; native physics, placement, and AI audio parity remain unverified. |
| `[rot_pnt_long]` | Parsed only | It does not currently affect vehicle physics. |
| `[rowdy_factor]`, `[boogies]`, `[ai_brakeperformance]`, `[ai_veh_type]`, `[sinus]` | Not implemented | Family-specific AI/drivetrain behavior is absent. |
| `[coupling_front]`, `[coupling_back]`, `[coupling_front_character]`, `[couple_back]`, `[couple_front_open_for_sound]` | Not implemented | Coupling and articulated multi-vehicle behavior are absent. |
| `[control_cable_front]`, `[control_cable_back]` | Not implemented | Cross-vehicle control/electrical variable transfer is absent. |
| `[contact_shoe]`, `[rail_body_osc]` | Not implemented | Rail/trolley-specific behavior is absent. |
| `[registration_automatic]`, `[registration_free]`, `[kmcounter_init]` | Partial | Startup selection/number seeding and odometer initialization exist; persistence and the full selection UI do not. |

The complete configurable input map and vehicle-family-specific view systems
also remain incomplete. See [BUS status](IMPLEMENTATION_STATUS_BUS.md).

## Passenger-cabin and path CFGs

| Keyword(s) / behavior | Current gap |
| --- | --- |
| `[linkToNextVeh]`, `[linkToPrevVeh]`, `[stamper]`, `[noticketsale]` | No complete runtime behavior. |
| `[pathpnt]`, `[pathlink]` extended fields | Only basic paths and room-height transitions are loaded; complete cabin traversal is absent. |
| `[stepsoundpack]` | The record is loaded, but complete step-sound playback and route use are absent. |
| Passenger mass and seated-human rendering | Not implemented; boarding movement is a roadside visual approximation and alighting movement is absent. |
| Ticket requests, cash/card transactions, ticket-selling UI, coin dispensing | Not implemented; map ticket/coin records only feed the documented script macros. |
| HOF, timetable, stop-announcement, arrival-board integration | Not implemented; route/timetable script lookups retain placeholder behavior. |

See [BUS and passenger/path status](IMPLEMENTATION_STATUS_BUS.md#passenger-cabin-and-path-cfgs).

## Map, terrain, road, and traffic configuration

| Keyword(s) / data | Status | Details |
| --- | --- | --- |
| `[groundtex]` and per-tile `.map.<index>.dds` | Partial | Base/detail textures and alpha masks render; seasonal selection and some source-index conventions remain unverified. |
| Global metadata such as `[description]`, `[NextIDCode]`, `[worldcoordinates]`, `[dynhelperactive]`, `[realrail]`, `[LHT]`, `[backgroundimage]`, `[mapcam]`, `[moneysystem]`, `[ticketpack]`, `[repair_time_min]`, `[years]`, `[realyearoffset]`, `[standarddepot]`, and traffic-density records | Partial | Many values are retained or used, but complete native semantics, density units/interpolation, and related rendering/UI behavior are not implemented or verified. |
| `[aigroup_2]`, `[aigroup_depot]`, traffic groups, `.ttl`, `.ttp`, `.hum` and related AI data | Partial | Supported route/population subsets work; scheduled route coverage, human animation, and several human fields are incomplete. |
| `[links]`, `[walk_param]`, `[voice]`, `[mass]` in human configuration | Not implemented | Human skeletal animation, voice, walking parameters, and mass behavior are absent. |
| `[rule]`, `[kill_rule]` | Partial | Traffic restrictions and density subsets work; overtaking and complete native category/density behavior do not. |
| `[object]`, `[spline]`, `[spline_h]` | Partial | Supported placement and road subsets work; Chrono overrides, some placement semantics, and full geometry/material parity remain incomplete. |
| `[splineAttachement]`, `[splineAttachement_repeater]`, `[attachObj]`, `[new_attachment]` | Partial | Basic linked placement/attachment behavior works; unresolved chains, repeater-master lookup, script behavior, and some attachment cases remain unsupported or unverified. |
| `[varparent]` | Parsed/validated only | Editor references are checked, but scenery script-variable inheritance is not evaluated. |
| `[spline_terrain_align]`, `[spline_terrain_align_2]` | Provisional / parsed only | The first is an approximation; the second is retained but not applied to geometry. |
| `[variable_terrain]`, `[variable_terrainlightmap]` | Parsed only | Dynamic terrain/lightmap updates are not implemented. |
| Tile `[terrain]` / `.map.terrain` | Partial | Static terrain sidecars render; dynamic terrain variants are not implemented. A tile without its sidecar has no terrain mesh. |
| `[water]` / `.map.water` | Provisional visual rendering | Sidecar surfaces render as translucent tile-wide quads and low road lanes are excluded; vehicle collision, multiple surfaces per tile, exact material/opacity, and native visual parity are not implemented or verified. |
| Road-profile `.sli` records `[texture]`, `[profile]`, `[profilepnt]`, `[path]`, `[halfcantwidth]`, `[heightprofile]` | Partial | Static road-strip rendering, path metadata, and cant handling support subsets; profile surfaces and native visual/unit parity remain incomplete or unverified. |
| Chrono `[selobject]`, `[selspline]`, `[delete]`, `[typ]`, `[relabel]` | Partial / not implemented | Source data can be retained, but date selection and variant application are absent. |
| `[traffic_lights_group]`, `[traffic_light]`, `[phase]`, `[traffic_light_stop]`, `[traffic_light_jump]`, `[use_traffic_light]`, `[approachdist]` | Partial | Fixed-cycle/request-aware behavior exists; cycle origin, model lamp animation, and native stop-line fidelity are unverified. |
| `.map.LM.bmp`, `.map.terrain_0.rdy` | Inventoried only | Paths/presence are detected; payloads are not decoded or applied. |

See [map status](IMPLEMENTATION_STATUS_MAP.md) for individual behavior and
validation notes.

## Scenery-object configuration

| Keyword(s) / behavior | Status | Details |
| --- | --- | --- |
| `[LOD]` scenery model parts | Partial | Nonzero LODs render using the vehicle threshold mapping; unusual threshold layouts and installed-scene parity remain unverified. |
| `[tree]` | Partial | Crossed textured preview cards are used instead of full vegetation/helper-mesh behavior. |
| `[varparent]` | Parsed/validated only | Editor references are checked; scenery script-variable inheritance is not evaluated. |
| `[script]` and model animation/material directives | Partial | Basic script frames, visibility, and dynamic surfaces work; scripted mesh animation, full material changes, sound callbacks, and scenery mouse events do not. |
| `[spline_terrain_align]` | Provisional | Placement tilt follows sampled terrain normal; exact behavior is unverified. |
| `[collision_mesh]`, `[boundingbox]`, `[nocollision]` | Partial | O3D collision subsets work; attached/repeater collision and complete collision semantics remain incomplete. |
| `[surface]` | Parsed only | The flag is retained, but surface-type/friction behavior is not derived from it. |
| `[crashmode_pole]` | Partial | Kinematic bending/breaking exists; hinge/fall dynamics and broad asset parity are incomplete. |
| `[maplight]`, `[nightmapmode]` | Parsed/consumed only | These records do not generate map lighting/lightmaps. |
| `[rendertype]` | Partial | Render-pass routing works for supported values; precedence and native ordering parity are unverified. |
| `[terrainmapping]` | Not implemented | Terrain/material mapping is not handled by the shared model-config loader. |
| Scenery sound and mouse-event dispatch | Not implemented | Playback/callbacks and scenery-object interaction are not connected. |

See [scenery status](IMPLEMENTATION_STATUS_SCENERY.md).

## Sound configuration and audio data

| Keyword(s) / behavior | Status | Details |
| --- | --- | --- |
| `[sound]`, `[trigger]` | Active subset | Playback works for configured/known triggers; unknown authored triggers are logged and do not play. |
| `[loop]`, `[looped]`, `[noloop]`, `[viewpoint]` | Active subset | Basic loop and view masks work; full native channel/ownership behavior is not established. |
| `[3d]` | Partial | Source position and maximum distance are used; complete spatialization and attenuation parity are unverified. |
| `[volcurve]`, `[pnt]` | Partial | Curves affect supported trigger/ambient paths; complete controller and curve semantics are unverified. |
| `[conditionSingle]` | Active subset | Single-variable comparisons work; complex condition/group logic is absent. |
| `[loopsound]` | Partial | Ambient loops support a subset of the configured control/curve/condition fields. |
| Start/loop/end filename families and same-file retrigger | Heuristic / partial | Filename pairing is heuristic; exact native retrigger behavior is unverified. |
| Compressed audio such as Ogg Vorbis or MP3 | Not implemented | The current decoder supports WAV PCM and 32-bit IEEE-float data only. |

See [audio status](IMPLEMENTATION_STATUS_AUDIO.md).

## Runtime diagnostics

For a keyword encountered in a loaded file that is not recognized or has no
implemented behavior, OpenBus can log an `Unhandled configuration tag [...]`
warning with the format, file, and line. The exact set depends on the files
being loaded. Such a warning is useful for identifying a gap, but the absence
of a warning does not imply complete runtime or OMSI-compatible behavior: some
keywords are parsed or partially applied and are documented above instead.
