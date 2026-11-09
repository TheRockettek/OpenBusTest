# OpenBus map bake archive, version 1

The archive is an optional cache container. It does not replace the OMSI map
source, and a reader must be able to fall back to normal map loading when the
archive is missing, stale, unsupported, or corrupt. Version 1 defines the file
container plus manifest and map-wide-index payload codecs; runtime tile map
data serialization is not implemented yet.

All integer fields are little-endian. Payloads are uncompressed in version 1.
The archive is seekable and has a fixed header, payload region, and trailing
chunk index.

## Header

The header is exactly 32 bytes:

| Offset | Size | Field |
| ---: | ---: | --- |
| 0 | 8 | Magic bytes `OBMAPBK\0` |
| 8 | 4 | Format version (`1`) |
| 12 | 4 | Header size (`32`) |
| 16 | 8 | Byte offset of the chunk index |
| 24 | 4 | Number of index entries |
| 28 | 4 | CRC-32 of the entire index |

## Payloads and index

Chunk payloads follow the header. Each index entry has a 40-byte fixed section
followed by its key bytes:

| Field | Size | Meaning |
| --- | ---: | --- |
| Type | 4 | Chunk type |
| Key size | 4 | Key byte length |
| Tile X | 4 | Signed tile coordinate, or zero when not applicable |
| Tile Y | 4 | Signed tile coordinate, or zero when not applicable |
| Payload offset | 8 | Absolute byte offset in the archive |
| Payload size | 8 | Uncompressed payload size |
| Payload CRC-32 | 4 | CRC-32 of the payload bytes |
| Flags | 4 | Must be zero in version 1 |
| Key | variable | Nonempty identifier bytes with no NUL; producers should encode UTF-8 |

The index is stored at the offset recorded in the header and runs to end of
file. Entries are unique by `(type, tile X, tile Y, key)`. Payload ranges must
be within the payload region and must not overlap. Readers validate the header,
index checksum, entry bounds, uniqueness, and each chunk checksum when read.
CRC-32 detects accidental corruption; it is not an authenticity mechanism.

Chunk types are `1` manifest, `2` map-wide index, `3` shared asset, and `4`
tile data. Suggested keys are `map` for the manifest, normalized asset paths
for shared assets, and `tile` plus signed coordinates for tile data. The
container codec rejects unknown types and nonzero flags.

`MapBakeArchiveWriter` accepts chunks incrementally and can copy a chunk from
an input stream using a fixed-size buffer, retaining only chunk-index metadata
in memory. `finish()` writes the index and patches the header. The
`writeMapBakeArchiveFile` callback overload combines this with temporary-file
publication, so a failed producer does not replace an existing archive.

Compression, complete source dependency collection, actual map data
serialization, runtime loading, stale-cache decisions, and fallback diagnostics
remain future work. `writeMapBakeArchiveFile` publishes the completed archive
through a temporary sibling file and replacement; no map bake builder invokes
it yet. The original source-map loading path is unchanged.

## Map-wide index payload, version 1

Chunk type `2` with key `map` stores the tile-coordinate index. All integers are
little-endian:

| Field | Size | Meaning |
| --- | ---: | --- |
| Magic | 4 | `OBMI` |
| Schema version | 4 | `1` |
| Tile count | 4 | Number of tile entries |
| Tile X | 4 | Signed tile coordinate |
| Tile Y | 4 | Signed tile coordinate |
| Chunk-key size | 4 | Byte length |
| Chunk key | variable | Key of the corresponding type-`4` tile-data chunk |

Each tile entry repeats the last four fields. Coordinates are unique and
serialized in ascending `(tile Y, tile X)` order. Chunk keys are nonempty,
NUL-free byte strings intended to be UTF-8. A tile-data chunk is located by
`(type 4, tile X, tile Y, chunk key)`. The codec limits the index to 100,000
tiles and 64 MiB; serialization sorts entries deterministically and
deserialization rejects unsorted or duplicate coordinates. The archive-level
index reader requires the `map` index chunk and verifies that every referenced
tile-data chunk identity exists; each tile payload checksum is verified when
that chunk is read.

## Manifest payload, version 1

Chunk type `1` stores a manifest payload. Its schema has its own version field,
independent of the archive-container version. All integers are little-endian.
Strings are length-prefixed byte sequences intended to be UTF-8; map keys and
setting keys must be nonempty, and strings cannot contain NUL bytes. Setting
values may be empty.

| Field | Size | Meaning |
| --- | ---: | --- |
| Magic | 4 | `OBMF` |
| Schema version | 4 | `1` |
| Map key size | 4 | Byte length |
| Map key | variable | Logical map identifier |
| Setting count | 4 | Number of settings |
| Dependency count | 4 | Number of source dependencies |
| Settings | variable | Repeated key size, key, value size, value |
| Dependencies | variable | Repeated key size, key, source size, SHA-256 digest |

Each setting's key and value sizes are 4-byte unsigned integers. Each dependency
key size is 4 bytes, its source size is an 8-byte unsigned integer, and its
SHA-256 digest is 32 bytes. Dependency keys are unique and serialized in
ascending order. The codec bounds the payload at 64 MiB and each string at
1 MiB.

The manifest comparison helper reads the `manifest` chunk with key `map` from
an archive, then compares it with a current manifest. An archive without that
chunk is reported as stale; archive checksum failures and malformed manifest
payloads are errors and should trigger the caller's normal-load fallback.

The current manifest API fingerprints an explicitly supplied list of source
files and records settings; it does not discover the complete map dependency
closure. It detects files changing during hashing and compares manifests for
added, removed, or changed dependencies and settings. The manifest codec is not
yet connected to a map-bake command, archive builder, or runtime stale-cache
check, so it does not yet enable baking or alter normal map loading. The asset
validation report now exposes a deduplicated path inventory of files reached by
its current scan, including map inputs and resolved assets. That inventory is
explicitly incomplete and cannot be used to certify a bake.

The inventory also includes side files discovered by the runtime's vehicle
asset search, including scanned `.cti` appearance files and `.hof` depot files.
This lets additions and removals in those discovered sets appear as manifest
dependency changes.

`makeMapBakeSourceFiles` maps paths under the configured map directory to
`map/<relative-path>` keys and paths under the OMSI root to
`omsi/<relative-path>` keys. The map root takes precedence if it is nested
inside the OMSI root. Keys use forward slashes; on Windows, ASCII letters in
keys are lowercased to match the loader's case-insensitive path behavior.
Sources outside both configured roots, including symlinks that resolve outside
them, are rejected instead of embedding machine-specific absolute paths.
