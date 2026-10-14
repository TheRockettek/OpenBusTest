# Map configuration: incomplete support

Scope: `global.cfg`, tile `tile_<x>_<y>.map` records, terrain/ground-texture adjuncts, map scenery/attachments, Chrono variants, and road `.sli` profiles. OpenBus loads a useful static subset, but this is not full OMSI map behavior.

## Partial or unimplemented global/map records

| Keyword / data | Status | Current boundary |
| --- | --- | --- |
| `[map]`, `[entrypoints]` | Active subset | Tile inventory and spawn selection work; duplicate tile rows are warned and aliased to the first tile. |
| `[groundtex]` and per-tile `.map.<index>.dds` | Partial | Base texture and decoded alpha masks for later ground layers are supported. Detail layers, seasonal selection, and all source sidecar/index conventions are not covered. Some add-on `.dds` files contain BMP data; texture loading now detects this by file signature. |
| Global metadata (`[description]`, `[NextIDCode]`, `[worldcoordinates]`, `[dynhelperactive]`, `[realrail]`, `[backgroundimage]`, `[mapcam]`, `[moneysystem]`, `[ticketpack]`, `[repair_time_min]`, `[years]`, `[realyearoffset]`, `[standarddepot]`, `[addseason]`, traffic-density records) | Parsed/retained only | Values/flags are retained, but generally do not drive map behavior. `[LHT]` and other unknown global sections are warned and preserved without runtime semantics. |
| Tile `[terrain]` / `.map.terrain` | Partial | Static terrain sidecars are loaded and rendered. Dynamic terrain variants are not. A listed tile without a terrain sidecar has no terrain mesh. |
| `[water]` / `.map.water` | Parsed only | Sidecar records are decoded and validated, but surfaces are not rendered because spatial ordering/geometry semantics are unresolved. |
| `[variable_terrain]`, `[variable_terrainlightmap]` | Parsed only | Marker presence is reported; dynamic terrain/lightmap updates are not implemented. |
| `[rule]`, `[kill_rule]` | Parsed only | Four-field records are retained but do not hide, select, or otherwise affect placements. |
| `.map.LM.bmp`, `.map.terrain_0.rdy` | Inventoried only | Paths/presence are detected; payloads are not decoded/applied. |

## Tile placement and attachment records

| Keyword / family | Status | Current boundary |
| --- | --- | --- |
| `[object]` | Partial | Static supported scenery placement is active; labels, Chrono overrides, and some placement semantics are not. |
| `[spline]`, `[spline_h]` | Partial | Road centerlines, elevation, cant, and a subset of profile rendering are active. Full OMSI geometry/material semantics and visual parity remain incomplete. |
| `[splineAttachement]` | Partial | Rows follow unique reciprocal spline links across tile boundaries. Missing/ambiguous IDs, invalid spline geometry, or chains without a usable local owner still produce no placement. |
| `[splineAttachement_repeater]` | Partial | Rows follow linked spline chains from their local owner; resolving the encoded repeater master tile/object index is still incomplete. |
| `[attachObj]` | Partial | Static parent-ID lookup and ordered `[new_attachment]` composition work across map tiles. Missing parents/anchors and cycles are skipped; script-variable behavior is not provided. |
| `[varparent]` | Parsed/validated only | Editor `Object Nr.` links are checked, but scenery script-variable inheritance is not implemented. This identifier is not treated as an attachment placement ID. |
| `[spline_terrain_align]` | Provisional | Marked scenery is tilted to the sampled terrain normal; exact OMSI behavior is unverified. |
| `[spline_terrain_align_2]` | Parsed only | The value is preserved but not applied to road/spline geometry. |
| Chrono `[selobject]`, `[selspline]`, `[delete]`, `[typ]`, `[relabel]` | Partial / not implemented | Selector IDs and raw tile content can be retained; date selection and variant application are not implemented. |

## Road profile `.sli` files

`[texture]`, `[profile]`, and `[profilepnt]` support a subset of static road-strip rendering, including straight and signed-radius centerlines. Known gaps include:

- Spline `skewStart` / `skewEnd` fields are applied by a provisional best-guess model (longitudinal shear per lateral distance) in rendered/collision road geometry; OMSI unit and visual parity are unverified.
- `.sli` `[heightprofile]` records are parsed. Valid lateral extents currently inform cant-width clamping; a complete authored height-profile surface is not implemented.
- Road markings, texture/material variations, chain continuity, curved cross-section seams, and full banking/cant parity still need validation.
- Editor-only profiles are intentionally omitted; profiles with missing texture files can render untextured.

## Not a keyword gap

A `.map` file listed in `global.cfg` but missing its `.terrain` sidecar is absent source data, not an unimplemented terrain keyword. Likewise, duplicate placement IDs and missing referenced textures/assets originate in the map/add-on data; the loader reports or skips them rather than rewriting the installed map. These cases should remain distinct from unsupported runtime semantics.

## Evidence and related pages

Implementation references: [MapConfigLoader.cpp](../src/MapConfigLoader.cpp), [MapRenderer.cpp](../src/MapRenderer.cpp), [MapRoadGeometry.cpp](../src/MapRoadGeometry.cpp), [MapSplineGeometry.cpp](../src/MapSplineGeometry.cpp), and [MapCollisionBuilder.cpp](../src/MapCollisionBuilder.cpp). Regression coverage includes [MapConfigLoaderProbe.cpp](../tests/MapConfigLoaderProbe.cpp), [MapSplineGeometryProbe.cpp](../tests/MapSplineGeometryProbe.cpp), [MapRoadGeometryProbe.cpp](../tests/MapRoadGeometryProbe.cpp), and [MapSceneryPlacementProbe.cpp](../tests/MapSceneryPlacementProbe.cpp). See [OMSI2_MAP_FORMAT_REFERENCE.md](OMSI2_MAP_FORMAT_REFERENCE.md) for observed record layouts and the [configuration status index](CONFIGURATION_IMPLEMENTATION_STATUS.md) for the other file families.
