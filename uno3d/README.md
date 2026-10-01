# UNO3D: OMSI O3D conversion helpers

This folder now contains a C++ converter from OMSI `.o3d` to `.obj/.mtl`.
It can also read OMSI `.cfg` model files (like `DL05.cfg`) and batch-convert referenced meshes.

OBJ is directly supported by Assimp, so this gives you a simple pipeline:

1. Convert `.o3d` -> `.obj`
2. Load `.obj` through Assimp in your engine

## Tool

- `o3d_to_obj.cpp` (built as `uno3d_converter`)

## Build

The converter is part of the main CMake project.

## Usage

### Convert one file

```powershell
./build/bin/uno3d_converter "C:/path/to/model.o3d"
```

Writes:
- `model.obj`
- `model.mtl`

### Convert one file to explicit output name

```powershell
./build/bin/uno3d_converter "C:/path/to/model.o3d" -o "C:/out/model.obj"
```

### Convert a folder

```powershell
./build/bin/uno3d_converter "C:/path/to/models"
```

### Convert a folder recursively

```powershell
./build/bin/uno3d_converter "C:/path/to/models" -r -o "C:/converted"
```

Directory O3D conversion uses up to four concurrent workers by default. Adjust
the limit with `--workers`, or set `OPENBUS_CONVERTER_WORKERS` to a value from
1 through 8. Converter profiling is enabled automatically and writes
`uno3d_trace.json`; set `OPENBUS_TRACE_FILE` to choose another path. The trace
contains conversion, O3D parsing, OBJ/MTL writing, and texture-conversion phases.

### Convert an OMSI model CFG (recommended for full bus models)

```powershell
./build/bin/uno3d_converter "C:/path/to/Model/DL05.cfg" -o "C:/converted"
```

Behavior in CFG mode:
- Reads `[mesh]` entries and converts referenced `.o3d` meshes.
- Reads `[matl]` and `[matl_change]` texture overrides and applies them to generated MTLs.
- Attempts to copy resolved textures next to each generated MTL for easy loading.
- Skips non-`.o3d` mesh entries (for example `.x`) with warnings.

### Build an optional OpenBus binary bundle

After conversion, pack the generated OBJ meshes into one `.obx` file:

```powershell
.\build-ode\Release\obj_bundle.exe MAN_DL05\Model\DL05_obj .\DL05.obx
```

Place the result beside the converted meshes and name it `openbus.obx`.
OpenBus uses matching bundle entries when available and falls back to the OBJ
files otherwise. The CFG remains responsible for mesh order, visibility,
animations, and material overrides.

### If winding is wrong in your renderer

```powershell
./build/bin/uno3d_converter "C:/path/to/model.o3d" --flip-winding
```

## Supported O3D data

- Vertex positions
- Vertex normals
- UV coordinates
- Triangle material IDs
- Embedded material data + diffuse texture filename
- Transform block (parsed, not baked into OBJ)

## Supported CFG data

- `[mesh]`
- `[matl]`
- `[matl_change]`

## Limitations

- Encrypted O3D files are currently not supported in this public converter.
- Bone weights are parsed but not exported to OBJ (OBJ has no skinning format).

## Assimp loading

Assimp can load the generated OBJ directly. If textures are not found, make sure texture paths in MTL are valid relative paths for your asset layout.
