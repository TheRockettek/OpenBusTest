# Scenery configuration: incomplete support

Scope: map scenery-object `.sco` files, inline/external scenery model CFGs, their map placements, and scenery collision/script behavior. OpenBus can render a subset of static scenery; parsed SCO data is not automatically runtime support.

## Implemented subset

- `[model]` external model CFG references and inline model records can feed static scenery batches. O3D and static text, binary, and MSZIP-compressed `.x` meshes are supported; nonzero `[LOD]` model parts are selected by distance using the same threshold mapping as vehicle models. Visual OMSI parity still needs validation.
- `[tree]` produces a crossed-textured-plane preview, not the authored helper mesh/tree behavior.
- Model geometry, including terrain, is rendered one-sided by default; terrain triangle indices are wound to match the renderer's front-face convention. SCO `[helparrow]` is recognized as metadata but does not change the global face policy.
- `[onlyeditor]` is respected: those placements are omitted from runtime rendering (except tree preview handling).
- `[absheight]` affects scenery height placement. `[new_attachment]` transforms can be used to resolve static `[attachObj]` children.
- `[attachObj]` translations follow the decoded scenery mesh's local X/Y basis. This keeps the London traffic-light bracket's authored CFG-Y arm offset aligned with its O3D mesh; attachment rotation parity remains provisional.
- Repeated SCO `[matl]` overrides are associated with O3D material slots by matching texture basename and authored occurrence order when numeric slot IDs disagree. This handles the London underground entrance's repeated `Fence_3.dds` alpha modes without overwriting unrelated entrance and kerb textures.
- `[script]`, `[varnamelist]`, `[stringvarnamelist]`, and `[constfile]` resources initialize per-placement scenery scripts. Basic `init`/`frame`, `[visible]`, and per-placement `[usescripttexture]` / `[usetexttexture]` surfaces are active. Counted map `[object]` labels feed recognized label-oriented inline SCO `[texttexture]` slots by slot index; this format/ordering is inferred from installed assets and still needs native OMSI visual comparison.
- `[collision_mesh]` O3D geometry from SCOs and external model CFGs is spatially indexed and clipped into each overlapped streamed tile, including long bridges and custom junction meshes. `[nocollision]` filters the relevant collision; attached/scenery repeater collision and non-O3D formats remain unsupported.
- `[groups]`, `[friendlyname]`, and `[helparrow]` are accepted as SCO metadata/editor fields and ignored by static rendering.
- Text-texture glyphs (`[texttexture]`, `[texttexture_enh]`) are multiplied by the mesh's diffuse colour and receive the same ambient/sun, lightmap and interior lighting as ordinary textures; nightmap, transmap and bumpmap layers are not applied to them. `[matl_freetex]` surfaces are lit the same way.

### Cotterell sign investigation

The "One Way" marking beside spawn 0 is object 18 in `maps\Cotterell\tile_7_1.map`
(label `ONE@WAY`), using `Sceneryobjects\UKDT\Cotterell\Road-hog123\Lane_Destination_2lane.sco`.
Its `[texttexture]` uses colour mode 1, so the glyphs come from the light-grey paint
bitmap of `Lane_Destination.oft`. No SCO keyword marks it as self-lit or as a blend
override; it rendered bright white because text textures were previously drawn unlit.
They now follow scene lighting, so the paint matches the surrounding road markings.

The campus direction signs use `[texttexture_enh]` with RGB `65, 85, 112` and alpha
mode `2`; the logo sign uses `[matl_freetex]`. Their O3D materials have diffuse 1.0
and no emissive, so the off-shade blue and grey were the same unlit-rendering cause;
no sign-specific colour correction is applied.

## Partial or unimplemented records

| Keyword / family | Status | Current boundary |
| --- | --- | --- |
| `[LOD]` scenery model parts | Partial | Nonzero LODs now render based on camera distance and the existing vehicle threshold mapping. The mapping and transitions have unit coverage, but installed-scene visual parity and unusual/malformed threshold layouts remain unverified. |
| DirectX `.x` meshes | Partial | Text, binary, and MSZIP-compressed `tzip`/`bzip` static meshes can load. Skinning/animation, named external materials, and animated X content remain unsupported. |
| `[tree]` | Partial | Preview cards are crossed textured planes rather than full OMSI vegetation/helper-mesh behavior. |
| `[varparent]` | Parsed/validated only | Editor `Object Nr.` references are kept distinct from placement IDs and checked, but scenery script-variable inheritance is not evaluated. |
| `[script]` and model animation/material directives | Partial | Script `init`/`frame` and visibility/text/script texture paths are active. Script-driven mesh animation, full dynamic material/texture changes, sound callbacks, and scenery mouse events are not. |
| `[spline_terrain_align]` | Provisional | Marked placements are tilted to the sampled terrain normal. Exact OMSI semantics have not been verified. |
| `[collision_mesh]`, `[nocollision]` | Partial | Supported O3D collision meshes from SCOs and external model CFGs are split across overlapped streamed tiles; unsupported formats, attached/repeater collision, complete collision semantics, and broad contact validation remain. `[boundingbox]` is not used as a map scenery collision fallback. |
| `[surface]` | Parsed only | The SCO flag is retained, but no full surface-type/friction behavior is derived from it. |
| `[maplight]`, `[nightmapmode]` | Parsed/consumed only | These records do not generate map lighting/lightmaps. |
| `[rendertype]` | Partial | SCO and scenery-model CFGs accept numeric values plus `presurface`, `surface`, and `on_surface`. An explicit model-part value takes precedence over the SCO value; otherwise the SCO value is used. Render passes route these categories, but precedence and native OMSI parity remain unverified. |
| Dynamic sound and interaction fields | Not implemented | Scenery sound playback/callbacks and scenery-object mouse-event dispatch are not connected. |

Scenery model CFGs share material and display limitations with bus model CFGs: incomplete CTC/texture replacement, lighting, alpha-depth parity, terrain mapping, and text-format details. See [BUS model CFG status](IMPLEMENTATION_STATUS_BUS.md#bus-model-cfg) for those shared model keywords; their presence in an SCO does not imply better scenery support.

`[terrainmapping]` belongs to the model/scenery CFG dialect, not the vehicle `.bus` file. It is not dispatched by the shared model-config loader, so terrain/material mapping remains unimplemented for scenery (and is not a BUS-level feature either).

## Not a keyword gap

A missing mesh/texture file, an unsupported asset encoding, duplicate placement IDs, or malformed source placement is an input/asset issue rather than an unimplemented SCO keyword. Missing assets are skipped or rendered without the unavailable material; duplicate IDs can prevent a referenced attachment from resolving. `[onlyeditor]` skips are intentional.

## Evidence and related pages

Implementation references: [SceneryObjectConfigLoader.cpp](../src/config/SceneryObjectConfigLoader.cpp), [ModelConfigLoader.cpp](../src/config/ModelConfigLoader.cpp), [MapRenderer.cpp](../src/render/map/MapRenderer.cpp), and [MapCollisionBuilder.cpp](../src/map/MapCollisionBuilder.cpp). Regression coverage includes [ModelConfigProbe.cpp](../tests/ModelConfigProbe.cpp), [MapCollisionBuilderProbe.cpp](../tests/MapCollisionBuilderProbe.cpp), and [MapSceneryPlacementProbe.cpp](../tests/MapSceneryPlacementProbe.cpp). See the [OMSI general CFG reference](OMSI_CFG_GENERAL_REFERENCE.md) for observed file layouts and the [configuration status index](CONFIGURATION_IMPLEMENTATION_STATUS.md) for the other file families.
