# Scenery configuration: incomplete support

Scope: map scenery-object `.sco` files, inline/external scenery model CFGs, their map placements, and scenery collision/script behavior. OpenBus can render a subset of static scenery; parsed SCO data is not automatically runtime support.

## Implemented subset

- `[model]` external model CFG references and inline model records can feed static scenery batches. O3D and static text, binary, and MSZIP-compressed `.x` meshes are supported; nonzero `[LOD]` model parts are selected by distance using the same threshold mapping as vehicle models. Visual OMSI parity still needs validation.
- `[tree]` produces a crossed-textured-plane preview, not the authored helper mesh/tree behavior.
- Model geometry, including terrain, is rendered one-sided by default; terrain triangle indices are wound to match the renderer's front-face convention. SCO `[helparrow]` is recognized as metadata but does not change the global face policy.
- O3D and explicit DirectX `.x` normals retain their authored direction when
  converted to map axes, so upward junction surfaces receive the same
  directional lighting as upward spline surfaces. Live-map parity remains
  unverified.
- `[onlyeditor]` is respected: those placements are omitted from runtime rendering (except tree preview handling).
- `[absheight]` affects scenery height placement. `[new_attachment]` transforms can be used to resolve static `[attachObj]` children.
- `[attachObj]` translations follow the decoded scenery mesh's local X/Y basis. This keeps the London traffic-light bracket's authored CFG-Y arm offset aligned with its O3D mesh; attachment rotation parity remains provisional.
- Repeated SCO `[matl]` overrides are associated with O3D material slots by matching texture basename and authored occurrence order when numeric slot IDs disagree. This handles the London underground entrance's repeated `Fence_3.dds` alpha modes without overwriting unrelated entrance and kerb textures.
- `[script]`, `[varnamelist]`, `[stringvarnamelist]`, and `[constfile]` resources initialize per-placement scenery scripts. Basic `init`/`frame`, `[visible]`, and per-placement `[usescripttexture]` / `[usetexttexture]` surfaces are active. Counted map `[object]` labels feed recognized label-oriented inline SCO `[texttexture]` slots by slot index; this format/ordering is inferred from installed assets and still needs native OMSI visual comparison.
- `[texcoordtransX]` and `[texcoordtransY]` translate material UVs using each
  placement's current script variables, as for vehicle materials. Installed-scene
  visual parity remains unverified.
- `[matl_lightmap]` loads the material's emissive mask, including
  script-selected `[matl_item]` masks, and applies the selected map's optional
  per-placement controller; `[alphascale]` scales base material opacity. Both
  use the vehicle renderer's clamped controller semantics. Missing or undecodable
  lightmaps are reported and fall back to ordinary surface lighting. Other
  item modifiers and installed-scene visual parity remain unverified.
- `[matl_bumpmap]` loads the material's normal map and applies its configured
  strength using the same shader as vehicle materials. Missing or undecodable
  maps are reported and fall back to unperturbed shading. Native bump and
  parallax behavior and installed-scene visual parity remain unverified.
- `[matl_envmap]` applies a spherical environment reflection in the material
  shader, scaled by the vehicle renderer's opacity factor and respecting the
  material's alpha. Missing or undecodable maps are reported and fall back to
  the unreflected surface. Dynamic material items and native visual parity
  remain unverified.
- `[matl_nightmap]` uses the vehicle nightmap shader. Base maps follow the
  placement's clamped `nightlighta` script variable; visibility-gated meshes
  and script-selected `[matl_item]` nightmaps render at full strength. Missing
  or undecodable maps are reported and leave the underlying surface unchanged.
  `[NightMapMode]` does not yet set `nightlighta` or schedule lighting, and
  item selection currently changes only nightmap and lightmap properties, not
  other material modifiers. Native visual parity remains unverified.
- `[collision_mesh]` O3D geometry from SCOs and external model CFGs is spatially indexed and clipped into each overlapped streamed tile, including long bridges and custom junction meshes. Without an authored collision mesh, `[boundingbox]` supplies a box collider; when neither is present, first-LOD O3D model geometry is used. `[nocollision]` suppresses collision unless the SCO also declares `[surface]`. Attached/scenery repeater collision and non-O3D formats remain unsupported.
- `[crashmode_pole]` scenery is removed from static mesh collision and represented as a streamed capsule. Its shaft center and width are estimated from the lower collision-mesh bounds, avoiding offsets from lamp arms; authored bounding boxes use map x/y/z axes. Vehicle chassis impacts accumulate configured momentum-based rotation; physics and rendering bend around the same collision base and disable collision at the break angle. The response is a kinematic approximation: the capsule does not reproduce every model shape, bending is applied per contact episode, breakage does not produce a detached/falling body, and broad live-map behavior still needs validation.
- `[groups]`, `[friendlyname]`, and `[helparrow]` are accepted as SCO metadata/editor fields and ignored by static rendering.
- `[description]` blocks are ignored by scenery rendering and by the inline-model
  loader, even when their free-form text resembles model directives. An absent
  `[end]` is reported as an error.
- Text-texture glyphs (`[texttexture]`, `[texttexture_enh]`) are multiplied by the mesh's diffuse colour. Lightmap, nightmap, transmap and bumpmap layers are not applied to them. `[matl_freetex]` uses its own dynamic texture path.
- Material emission blends the shaded surface toward its sampled, unlit colour. White emission in route-arrow, route-text and DL05 display O3D materials therefore preserves texture RGB and black pixels instead of adding white and saturating the surface.

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
| DirectX `.x` meshes | Partial | Text, binary, and MSZIP-compressed `tzip`/`bzip` static meshes can load. Inline and same-file named material references (including forward references), per-vertex `MeshVertexColors`, and static material diffuse/alpha/texture/specular/emission values flow into vehicle/scenery rendering. Exact native lighting parity is unverified; skinning/animation and cross-file external materials remain unsupported. |
| `[tree]` | Partial | Preview cards are crossed textured planes rather than full OMSI vegetation/helper-mesh behavior. |
| `[varparent]` | Parsed/validated only | Editor `Object Nr.` references are kept distinct from placement IDs and checked, but scenery script-variable inheritance is not evaluated. |
| `[script]` and model animation/material directives | Partial | Script `init`/`frame` and visibility/text/script texture paths are active. Script-driven mesh animation, full dynamic material/texture changes, sound callbacks, and scenery mouse events are not. |
| `[spline_terrain_align]` | Provisional | Marked placements are tilted to the sampled terrain normal. Native refresh can register marked runtime objects for a gated terrain-hole-related polygon path; exact OMSI semantics remain unverified. |
| `[terrainhole]` | Not implemented | Native static tracing connects its model-side source and optional generated `.hole` cache to terrain-hole polygon processing. OpenBus warns on the keyword; terrain and collision effects remain unverified. |
| `[collision_mesh]`, `[boundingbox]`, `[nocollision]`, `[surface]` | Partial | Supported O3D collision meshes from SCOs and external model CFGs are split across overlapped streamed tiles. If no collision mesh is available, a configured bounding box is used; otherwise first-LOD O3D render geometry is the fallback. `[nocollision]` suppresses collision unless `[surface]` is also present. Unsupported formats, attached/repeater collision, complete collision semantics, and broad contact validation remain. |
| `[crashmode_pole]` | Partial runtime | Positive finite resistance and break angle are parsed. Lower O3D collision-mesh bounds or authored bounding boxes provide an approximate capsule; physics applies resistance-scaled chassis momentum once per contact episode, and rendering rotates about the same collision base. Poles disable collision on break; exact mesh collision, hinge/fall dynamics and broad live-map validation remain incomplete. |
| `[surface]` | Parsed only | The SCO flag is retained, but no full surface-type/friction behavior is derived from it. |
| `[complexity]` | Parsed/retained only | The authored integer is retained, but scenery detail filtering is not implemented. |
| `[nightmapmode]` | Parsed/retained only | The authored integer is retained, but its rendering behavior is not implemented. |
| `[maplight]` | Parsed/consumed only | Does not generate map lighting/lightmaps. |
| `[rendertype]` | Partial | SCO and scenery-model CFGs accept numeric values plus `presurface`, `surface`, and `on_surface`. An explicit model-part value takes precedence over the SCO value; otherwise the SCO value is used. Render passes route these categories, but precedence and native OMSI parity remain unverified. |
| Dynamic sound and interaction fields | Not implemented | Scenery sound playback/callbacks and scenery-object mouse-event dispatch are not connected. |

Scenery model CFGs share material and display limitations with bus model CFGs: incomplete CTC/texture replacement, lighting, alpha-depth parity, terrain mapping, and text-format details. See [BUS model CFG status](IMPLEMENTATION_STATUS_BUS.md#bus-model-cfg) for those shared model keywords; their presence in an SCO does not imply better scenery support.

Inline SCO texture-address markers use the shared material parser and renderer,
including border, clamp, mirror, and mirror-once modes. A GPU probe verifies
border sampling and shared-texture wrap changes; representative scenery visual
parity is not yet established.

`[terrainmapping]` belongs to the model/scenery CFG dialect, not the vehicle `.bus` file. The shared model-config loader retains its variable-length payload verbatim, but terrain/material mapping remains unimplemented for scenery (and is not a BUS-level feature either).

## Not a keyword gap

A missing mesh/texture file, an unsupported asset encoding, duplicate placement IDs, or malformed source placement is an input/asset issue rather than an unimplemented SCO keyword. Missing assets are skipped or rendered without the unavailable material; duplicate IDs can prevent a referenced attachment from resolving. `[onlyeditor]` skips are intentional.

## Evidence and related pages

Implementation references: [SceneryObjectConfigLoader.cpp](../src/config/SceneryObjectConfigLoader.cpp), [ModelConfigLoader.cpp](../src/config/ModelConfigLoader.cpp), [MapRenderer.cpp](../src/render/map/MapRenderer.cpp), and [MapCollisionBuilder.cpp](../src/map/MapCollisionBuilder.cpp). Regression coverage includes [ModelConfigProbe.cpp](../tests/ModelConfigProbe.cpp), [MapCollisionBuilderProbe.cpp](../tests/MapCollisionBuilderProbe.cpp), and [MapSceneryPlacementProbe.cpp](../tests/MapSceneryPlacementProbe.cpp). See the [OMSI general CFG reference](OMSI_CFG_GENERAL_REFERENCE.md) for observed file layouts and the [configuration status index](CONFIGURATION_IMPLEMENTATION_STATUS.md) for the other file families.
