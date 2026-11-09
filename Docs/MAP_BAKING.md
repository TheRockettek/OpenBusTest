# Optional map bake workflow

## Status

Map baking is not yet available in-game. The archive, manifest, and map-index
code currently provide format primitives only; there is no production builder
or runtime baked-map loader. The asset validator also deliberately reports
incomplete dependency coverage. A map must not be baked or treated as
bake-ready until dependency discovery and serialization cover every supported
runtime input. Unknown or partially understood references must block readiness,
not be silently omitted.

The original source-map loading path remains authoritative and must continue to
work whether or not a bake exists. The in-game asset check currently reports
phase progress. Cancellation is not yet available.

## Intended in-game workflow

The planned flow is:

1. Select a map and run **Check assets**. The scan reports its current phase and
   progress, missing or inaccessible references, and unsupported content.
2. Enable **Bake map** only when dependency closure is complete, mandatory
   assets resolve, and all required data has a supported serializer.
3. Build the bake into a temporary file, report discovery/serialization/write
   progress, validate the completed archive, then atomically publish it beside
   the map.
4. On map load, use a bake only after checking its format, integrity, map
   identity, and source fingerprints. Missing, stale, corrupt, or unsupported
   bakes are logged and bypassed; the existing source loader runs normally.

Progress should be visible for both checking and baking. Bake cancellation
must leave the source map untouched and must not publish a partial archive.

## Compatibility and format evolution

The bake is an optional cache, never a replacement for OMSI source files.
Source files and the normal loader remain the compatibility fallback.

Archive-format and payload-schema versions are independent. Readers must reject
unknown required versions or payloads and fall back to source loading instead
of interpreting them as a known format. A future format revision may add
optional payloads only when older readers can safely ignore them; required
features need explicit capability/version checks. Existing version-1 bytes must
not be reinterpreted when the schema evolves. Keep old readers or provide an
explicit migration path for any published format change.

Before a bake is used, verify its manifest fingerprints against the inputs
recorded by a complete dependency scan. A failed fingerprint, checksum, archive
identity, or schema check invalidates the cache and triggers ordinary loading.

## Readiness gates

The builder and loader should remain disabled until all of these are true:

- Every map/runtime-reachable source is inventoried, including transitive
  configuration references and date-selected Chrono content.
- Each reference is resolved and classified as required, optional, or
  unsupported; unverified references prevent a complete result.
- Every required serialized structure has a tested decoder/encoder pair.
- Archive corruption, stale inputs, unsupported versions, interrupted writes,
  and absent caches have regression coverage for source-loader fallback.
- In-game progress and cancellation tests confirm that failed or cancelled
  operations never report success or publish a partial bake.
