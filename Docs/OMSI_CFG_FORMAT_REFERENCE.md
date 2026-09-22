# OMSI `.cfg` format index

OMSI uses several related CFG dialects. They share keyword parsing rules, but
the same keyword can have different meaning depending on the containing file.
Do not treat every CFG as a vehicle model file.

## Configuration types

| Reference | Use |
| --- | --- |
| [OMSI_CFG_BUS_MODEL_REFERENCE.md](C:/Users/blane/Desktop/OpenBusTest/Docs/OMSI_CFG_BUS_MODEL_REFERENCE.md) | Bus-only model construction: meshes, materials, animations, lights, visibility, displays, and texture changes |
| [OMSI_CFG_GENERAL_REFERENCE.md](C:/Users/blane/Desktop/OpenBusTest/Docs/OMSI_CFG_GENERAL_REFERENCE.md) | Shared syntax plus surfaces, passenger cabins, paths, sounds, AI/network, environment, controls, and metadata |

The original CFG files have not been moved. Their relative references are part
of the format and must remain valid.

## How to classify a CFG

- **Bus model CFG**: large file containing `[mesh]` plus model-only sections
  such as `[matl]`, `[newanim]`, `[visible]`, `[CTC]`, or `[illumination_interior]`.
- **Surface/material CFG**: image-named file containing `[surface]`,
  `[moisture]`, or `[puddles]`.
- **Passenger cabin CFG**: contains `[drivpos]`, `[passpos]`, `[entry]`, or
  `[exit]`.
- **Path CFG**: contains `[pathpnt]` or `[stepsoundpack]`.
- **Sound CFG**: contains `[loopsound]` or `[3d]`.
- **World/AI CFG**: contains `[ailist]`, `[aigroup_*]`, `[busstop]`,
  `[signalroute]`, `[StnLink]`, `[sky_textures]`, or `[cloudtype]`.
- **Controls/global CFG**: contains `[game]`, `[ctrl]`, `[axis]`,
  `[friendlyname]`, or `[usrinfo]`.

When a file matches multiple categories, classify it by the subsystem that
loads it. For example, `[entry]` means a passenger entry in a cabin CFG but a
keyboard binding in a game CFG.

## Shared syntax

1. A keyword must be the only text on its line and begin at column zero.
2. Values follow line-by-line until the block's fixed layout, count, or
   terminator is consumed.
3. Repeated keywords create ordered records.
4. Count-prefixed blocks must consume exactly the declared number of records.
5. `[description]` blocks end at `[end]`.
6. Do not physically reorganize referenced files without rewriting relative
   paths.

