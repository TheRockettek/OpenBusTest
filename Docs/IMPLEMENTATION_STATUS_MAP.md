# Map configuration: incomplete support

Scope: `global.cfg`, tile `tile_<x>_<y>.map` records, terrain/ground-texture adjuncts, map scenery/attachments, Chrono variants, and road `.sli` profiles. OpenBus loads a useful static subset, but this is not full OMSI map behavior.

## Partial or unimplemented global/map records

| Keyword / data | Status | Current boundary |
| --- | --- | --- |
| `[map]`, `[entrypoints]` | Active subset | Tile inventory and spawn selection work; duplicate tile rows are warned and aliased to the first tile. |
| `[groundtex]` and per-tile `.map.<index>.dds` | Partial | Base textures, per-tile alpha masks, and each record's repeated detail texture are rendered. Suffixes 1..N-1 map directly to non-base records; terminal suffix N maps to record N-1 for maps such as Cotterell. Seasonal selection and other source index conventions remain unverified. Some add-on `.dds` files contain BMP data; texture loading detects this by file signature. |
| Global metadata (`[description]`, `[NextIDCode]`, `[worldcoordinates]`, `[dynhelperactive]`, `[realrail]`, `[LHT]`, `[backgroundimage]`, `[mapcam]`, `[moneysystem]`, `[ticketpack]`, `[repair_time_min]`, `[years]`, `[realyearoffset]`, `[standarddepot]`, traffic-density records) | Partial runtime | Values/flags are retained. `[LHT]` determines the signed lane convention when a non-zero actor offset is requested; automatic traffic follows the authored path with no extra lateral offset. Road and passenger density points are sampled with provisional linear interpolation and endpoint clamping, then used as multipliers for separate capped AI-vehicle and human populations. Each defaults to 8, controlled by `OPENBUS_MAP_AI_COUNT` and `OPENBUS_MAP_HUMAN_COUNT` (0..200); `OPENBUS_MAP_AI_HOUR` (0..24) selects the schedule hour for both. Absent density data falls back to factor 1.0. Native density units/interpolation and the default time-of-day policy are unverified. Other unknown global sections are warned and preserved. |
| `ailists.cfg`, `drivers.txt`, `.hum` | Partial runtime | `[aigroup_2]` `.bus`/`.ovh` references and complete group payloads, `[aigroup_depot]` depot/HOF associations with linked bus-type `.bus` references and assignment payloads, and map-local `.hum` driver model paths are retained. Startup combines `.bus` entries from vehicle groups and bus types, creates up to 200 vehicles on nearby road-path routes, retries a bounded set of candidates across available routes, and advances them from spline poses at a provisional 12 m/s; open routes recycle actors and wrap spawn positions before their endpoints. Human actors are spawned from valid `drivers.txt` `.hum` configs on nearby human-path routes using both parsed `[humangeom]` dimensions, move at a provisional 1.2 m/s, and use the same bounded candidate and endpoint-safe placement behavior. `OpenBusMapTrafficRuntimeProbe` covers repeated in-range AI/human placement, route movement, endpoint handling, and out-of-range rejection. Human rendering currently uses colored primitive proxies: `.hum` mesh/model configs, skeletal animation, and `[links]`, `[walk_param]`, `[voice]`, and `[mass]` runtime behavior are not implemented. Unsupported `.ovh` entries are skipped and logged. |
| Tile `[terrain]` / `.map.terrain` | Partial | Static terrain sidecars are loaded and rendered. Dynamic terrain variants are not. A listed tile without a terrain sidecar has no terrain mesh. |
| `[water]` / `.map.water` | Provisional visual rendering | Each sidecar surface is rendered as a translucent, tile-wide quad with SW/SE/NE/NW corner heights. Corner order is reverse-engineered from neighboring installed OMSI maps; multiple surfaces per tile, exact material/opacity, and native OMSI visual parity are not verified. Water does not participate in vehicle collision. |
| `[variable_terrain]`, `[variable_terrainlightmap]` | Parsed only | Marker presence is reported; dynamic terrain/lightmap updates are not implemented. |
| `[rule]`, `[kill_rule]` | Parsed only | Four-field records are retained but do not hide, select, or otherwise affect placements. |
| `.map.LM.bmp`, `.map.terrain_0.rdy` | Inventoried only | Paths/presence are detected; payloads are not decoded/applied. |

## Tile placement and attachment records

| Keyword / family | Status | Current boundary |
| --- | --- | --- |
| `[object]` | Partial | Static supported scenery placement is active. Counted placement labels feed label-oriented inline SCO text-texture slots, visible only on the authored front face; label/slot ordering is inferred and native OMSI visual parity remains unverified. Chrono overrides and some placement semantics are not implemented. |
| `[spline]`, `[spline_h]` | Partial | Road centerlines, elevation, cant, and a subset of profile rendering are active. Full OMSI geometry/material semantics and visual parity remain incomplete. |
| `[splineAttachement]` | Partial | Rows follow unique reciprocal spline links across tile boundaries. Missing/ambiguous IDs, invalid spline geometry, or chains without a usable local owner still produce no placement. |
| `[splineAttachement_repeater]` | Partial | Rows follow linked spline chains from their local owner; resolving the encoded repeater master tile/object index is still incomplete. |
| `[attachObj]` | Partial | Static parent-ID lookup and ordered `[new_attachment]` composition work across map tiles. Missing parents/anchors and cycles are skipped; script-variable behavior is not provided. |
| `[varparent]` | Parsed/validated only | Editor `Object Nr.` links are checked, but scenery script-variable inheritance is not implemented. This identifier is not treated as an attachment placement ID. |
| `[spline_terrain_align]` | Provisional | Marked scenery is tilted to the sampled terrain normal; exact OMSI behavior is unverified. |
| `[spline_terrain_align_2]` | Parsed only | The value is preserved but not applied to road/spline geometry. |
| Chrono `[selobject]`, `[selspline]`, `[delete]`, `[typ]`, `[relabel]` | Partial / not implemented | Selector IDs and raw tile content can be retained; date selection and variant application are not implemented. |

## Road profile `.sli` files

`[texture]`, `[profile]`, and `[profilepnt]` support a subset of static road-strip rendering, including straight and signed-radius centerlines. `.sli` `[path]` payloads are retained, and the leading integral field is exposed as a path-type code. Installed `invis_street.sli` and `invis_human.sli` use codes `0` and `1` respectively; this inferred mapping separates road-vehicle and human route networks. Remaining path-field semantics and native OMSI traffic behavior are unverified; human actors now traverse type-1 routes but use proxy geometry rather than their configured `.hum` models. `[nocollision]` is retained and suppresses spline collision geometry without removing its renderable sections. Known gaps include:

- Spline `skewStart` / `skewEnd` fields are applied as longitudinal shear per lateral distance in rendered/collision road geometry. At linked corners, when the authored endpoint axes already align (as in Cotterell splines 518/519 with approximately `+1`/`-1` skew), the seam now preserves that shared parallelogram axis instead of adding a second automatic miter. The exact native unit convention and broader OMSI visual parity remain unverified.
- `.sli` `[halfcantwidth]` explicitly sets cant clamping width; absent or invalid values fall back to valid `[heightprofile]` lateral extents. The four `[heightprofile]` fields are retained, but a complete authored height-profile surface is not implemented. Native visual parity remains unverified.
- Road markings, texture/material variations, chain continuity, curved cross-section seams, and full banking/cant parity still need validation.
- Editor-only profiles are intentionally omitted; profiles with missing texture files can render untextured.

## Not a keyword gap

A `.map` file listed in `global.cfg` but missing its `.terrain` sidecar is absent source data, not an unimplemented terrain keyword. Likewise, duplicate placement IDs and missing referenced textures/assets originate in the map/add-on data; the loader reports or skips them rather than rewriting the installed map. These cases should remain distinct from unsupported runtime semantics.

## Evidence and related pages

AI vehicles now use an out-and-back route built from the selected open road-spline chain. The reverse traversal keeps the vehicle on the authored car splines at endpoints instead of recycling its position to the chain start; [MapTrafficRuntimeProbe.cpp](../tests/MapTrafficRuntimeProbe.cpp) verifies spatial continuity through both turnarounds. This supplements the earlier startup description in the table above.

Implementation references: [MapConfigLoader.cpp](../src/map/MapConfigLoader.cpp), [MapRenderer.cpp](../src/render/map/MapRenderer.cpp), [MapRoadGeometry.cpp](../src/map/MapRoadGeometry.cpp), [MapSplineGeometry.cpp](../src/map/MapSplineGeometry.cpp), and [MapCollisionBuilder.cpp](../src/map/MapCollisionBuilder.cpp). Regression coverage includes [MapConfigLoaderProbe.cpp](../tests/MapConfigLoaderProbe.cpp), [MapSplineGeometryProbe.cpp](../tests/MapSplineGeometryProbe.cpp), [MapRoadGeometryProbe.cpp](../tests/MapRoadGeometryProbe.cpp), and [MapSceneryPlacementProbe.cpp](../tests/MapSceneryPlacementProbe.cpp). See [OMSI2_MAP_FORMAT_REFERENCE.md](OMSI2_MAP_FORMAT_REFERENCE.md) for observed record layouts and the [configuration status index](CONFIGURATION_IMPLEMENTATION_STATUS.md) for the other file families.
