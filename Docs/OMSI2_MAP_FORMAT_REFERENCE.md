# OMSI 2 map-format reference (reverse-engineered)

This document is an implementation-oriented reference for OMSI 2 map packages. It
combines the parser and serializer behavior of OMSI Map Merger v1.3 with
read-only observations from the installed **Grande Porto 2022** map. It is not
an official or complete specification of the OMSI executable.

> **Evidence rule:** a parser's field count and order are strong evidence of
> syntax accepted by that parser; they do not by themselves prove the meaning,
> units, or runtime behavior of a field. In particular, names such as `line4`,
> `num2`, and `object_on_tile_index` are implementation labels, not authoritative
> OMSI terminology. Where meaning was not independently established, this
> reference preserves the value as opaque text and says so.

## 1. Sources, scope, and confidence

The main structural reference used here is
[barteg77/omsi_map_merger v1.3](https://github.com/barteg77/omsi_map_merger/tree/00c2ceb189ab00ef11a59f7f3279dcfd7e7fd79a),
commit `00c2ceb189ab00ef11a59f7f3279dcfd7e7fd79a`. Its README explicitly says
that it expects editor-like syntax and lists parser/merge limitations. Its
ParGlare grammars, parser actions, models, serializers, and tests are useful
reverse-engineering evidence, but not a formal OMSI specification. The English
[merger manual](https://github.com/barteg77/omsi_map_merger/blob/00c2ceb189ab00ef11a59f7f3279dcfd7e7fd79a/MANUAL_en.md)
describes its workflow, not the full format.

A second source is the OMSI forum's historical
[AI-list tutorial](https://forum.omnibussimulator.de/forum/index.php?thread/11880-tut-changing-adjusting-the-ai-list-ailist/).
It is useful for recognizing older `[ailist]` / `[aigroup]` dialects, but it
predates current OMSI 2 maps and is not a normative format definition. The
Grande Porto 2022 files are the empirical validation corpus for the examples
and counts below.

No complete official, field-by-field map-format specification was established
from the sources consulted. This document therefore separates **parser-observed
structure**, **sample-observed facts**, and **unverified semantics**. Treat all
unparsed files and fields conservatively.

## 2. Package layout and file roles

A map is a directory package, not a single file. The principal entry point is
`global.cfg`; its repeated `[map]` records name the tile files. Paths in map
records commonly point outside the map directory to shared OMSI assets.

| Path/pattern | Role and evidence |
| --- | --- |
| `global.cfg` | Main map metadata, grid/tile manifest, global entrypoints, ground-texture and traffic settings. Parsed by OMSI Map Merger. |
| `tile_<x>_<y>.map` | Text data for a grid tile. `<x>` and `<y>` are integer coordinates and must agree with the corresponding `[map]` record in the merger's model. |
| `tile_<x>_<y>.map.terrain`, `.map.water`, `.map.LM.bmp`, `.map.terrain_0.rdy` | Tile adjuncts. In the sample, terrain/readiness/water payloads are binary; `LM.bmp` is a BMP. The merger copies some adjunct patterns but does not decode their payloads. |
| `tile_<x>_<y>.map.prt` | Per-tile adjunct observed in Grande Porto. A sampled nonempty file is text-like, but its structure and runtime role were not established; the v1.3 merger's tile-sidecar list does not include it. Preserve it unchanged. |
| `texture/map/tile_<x>_<y>.map.roadmap.bmp` and `texture/map/tile_<x>_<y>.map.<n>.dds` | Optional per-tile road-map/ground-texture assets recognized as copy patterns by the merger. Their image payloads are not parsed as map records. |
| `TTData/` | Standard timetable package: `*.ttl` line files, `*.ttr` track files, `*.ttp` trip files, plus optional `Busstops.cfg` and `StnLinks.cfg`. |
| `Chrono/<event>/` | Date-controlled map variant. Can contain `Chrono.cfg`, language descriptors, tile overrides named for existing tiles, and an event-specific `TTData/`. |
| `ailists.cfg` | Modern AI vehicle groups/depot assignments. The merger supports modern groups, not the older list syntax. |
| `*.hof` | HOF destination/stop data is commonly map/vehicle-facing, but the merger does not parse it. Treat its syntax as a separate format. |
| `signalroutes.cfg`, `unsched_trafficdens.txt`, `unsched_vehgroups.txt`, `drivers.txt`, `Holidays*.txt`, `humans.txt`, `registrations.txt`, images and other root files | Map-level companion data. Some are copied byte-for-byte by the merger; copying is not equivalent to understanding or merging their contents. |
| `timeline.prt.cfg` | Root file observed in Grande Porto with 15 Chrono paths and a trailing data block. It is not in the merger's registered parser/copy set; its schema is unknown here. |
| `laststn.osn`, `laststn.osn.owt`, `laststn.osn_*.dds` | Opaque companion files observed in the map folder, rather than tile geometry. They are outside the merger's registered parser/copy set; their schema and runtime role are unknown here. |

The current OpenBus manifest inventories numeric-suffix `.dds` sidecar paths
per listed tile and preserves the suffix text verbatim. It does not decode the
DDS payload or establish how the suffix maps to `[groundtex]` entries; no
sidecar is currently bound to rendered terrain. Tile `.map.LM.bmp` and
`.map.terrain_0.rdy` paths and presence are also inventoried, but their payloads
are not decoded or applied.

The map also depends on `Sceneryobjects\...`, `Splines\...`, vehicles,
textures, and other assets that may live elsewhere under the OMSI installation
or in add-ons. Do not assume that a map directory contains every referenced
asset.

### 2.1 The merger's actual read/write boundary

At the pinned v1.3 source, the merger structurally parses `global.cfg`, listed
tile `.map` files, selected timetable files, modern `ailists.cfg`, chrono tile
files, and selected chrono timetable files. It has generic copy rules for
additional files. Notable boundaries from its README/source:

- `unsched_trafficdens.txt` and `unsched_vehgroups.txt` are not semantically
  merged (they may be copied as opaque files).
- Generic map files are copied, not parsed; the merger's merge orchestration
  takes its registered opaque root-companion set from the first map rather
  than combining those sets from every input map. Arbitrary unregistered root
  files are not thereby guaranteed to be copied.
- `global_*.dsc` and `chrono_#upd.cfg` are not handled as structured data.
- `[worldcoordinates]` maps are reported as unsupported/unsafe for merging.
- Older AI-list syntax is not supported.
- Its grammar is intentionally strict about editor-style line order.
- Tile `.prt` files are present in the Grande Porto sample but are not in the
  merger's registered tile-sidecar patterns.

See the pinned [`omsi_map.py`](https://github.com/barteg77/omsi_map_merger/blob/00c2ceb189ab00ef11a59f7f3279dcfd7e7fd79a/omsi_map.py),
[`omsi_files.py`](https://github.com/barteg77/omsi_map_merger/blob/00c2ceb189ab00ef11a59f7f3279dcfd7e7fd79a/omsi_files.py),
[`omsi_map_merger.py`](https://github.com/barteg77/omsi_map_merger/blob/00c2ceb189ab00ef11a59f7f3279dcfd7e7fd79a/omsi_map_merger.py),
[`README.md`](https://github.com/barteg77/omsi_map_merger/blob/00c2ceb189ab00ef11a59f7f3279dcfd7e7fd79a/README.md),
and the per-file grammars/parsers referenced below.

## 3. Text encoding and line-oriented parsing

There is no single encoding for every file in a map. Do not choose one decoder
for the whole directory.

Observed in Grande Porto 2022:

- `global.cfg` and sampled tile `.map` files use UTF-16 little-endian with a
  BOM and CRLF lines.
- Sampled `ailists.cfg`, timetable files, and Chrono `.cfg` / `.dsc` files have
  no BOM and use CRLF. Many are ASCII-only, so their exact legacy encoding
  cannot be determined from those bytes alone.
- `GrandePorto2022.hof` contains non-ASCII bytes and is not strict UTF-8; its
  precise code page was not established.
- Binary tile adjuncts must not be passed through a text decoder.

The merger uses different decoding policies by parser: UTF-16 for global and
Chrono tile text; UTF-16 with an ASCII fallback for ordinary tiles; an encoding
detection library for modern AI lists; and ISO-8859-1 for timetable files.
That is implementation behavior, not proof that all valid maps use those exact
encodings. See its [`file_decoder.py`](https://github.com/barteg77/omsi_map_merger/blob/00c2ceb189ab00ef11a59f7f3279dcfd7e7fd79a/file_decoder.py)
and individual `parse()` functions.

The parser model is line-oriented:

1. A recognized tag occupies its own line, e.g. `[map]`; many tags are literal
   and case/spelling sensitive in the merger's grammar.
2. Values are positional lines after the tag, not key/value pairs.
3. Blank lines can be optional values/placeholders or separators. They must not
   be indiscriminately discarded before fixed-width records are parsed.
4. Some formats use exact dash/dot separator lines and human-readable header
   lines. They are part of the parser grammar.
5. Repeated records are ordered. Preserve their order unless a specific
   reference requires remapping.
6. Variable/unknown lines occur inside some records. Preserve them verbatim;
   do not treat every unrecognized line as a comment or safely discardable.

OpenBus preserves the raw payload lines and source section line for unsupported
bracketed sections in `global.cfg` and tile `.map` files while also emitting a
diagnostic. This is storage-only: it does not establish the section's meaning
or apply it to rendering/simulation. A robust parser should retain raw lines
and unknown records even when it also constructs typed fields; round-trip and
forward-compatibility are safer than normalizing undocumented data.

## 4. `global.cfg`

The merger's global grammar defines an ordered sequence of optional metadata
sections, repeated settings, entrypoints, then repeated map records. Implement
the order accepted by this parser; whether OMSI accepts other orderings has not
been established here. See [`global_config_grammar.pg`](https://github.com/barteg77/omsi_map_merger/blob/00c2ceb189ab00ef11a59f7f3279dcfd7e7fd79a/global_config_grammar.pg),
[`global_config_parser.py`](https://github.com/barteg77/omsi_map_merger/blob/00c2ceb189ab00ef11a59f7f3279dcfd7e7fd79a/global_config_parser.py),
and [`global_config.py`](https://github.com/barteg77/omsi_map_merger/blob/00c2ceb189ab00ef11a59f7f3279dcfd7e7fd79a/global_config.py).

| Section | Payload shape captured by the merger | Confidence/notes |
| --- | --- | --- |
| Initial comment | Lines before the first recognized section | Preserved as initial text. |
| `[name]`, `[friendlyname]` | One optional line each | Values are strings. |
| `[description]` | Free-form lines through `[end]` | Terminator is structural; do not stop at arbitrary blank lines. |
| `[version]` | One optional line | String in the merger model. Grande Porto contains `14`. |
| `[NextIDCode]` | One value | Converted to integer; the merger uses this value when offsetting IDs during a map merge. Exact engine semantics beyond that use are not specified here. |
| `[worldcoordinates]`, `[dynhelperactive]`, `[realrail]` | Marker sections | Presence is represented as a flag. `[worldcoordinates]` has a known merger limitation. |
| `[backgroundimage]` | 6 ordered lines | Stored as generic `num1`…`num6`; meanings are not established by the parser model. |
| `[mapcam]` | 8 ordered lines | Stored as generic values; meanings are not established here. |
| `[moneysystem]`, `[ticketpack]`, `[repair_time_min]`, `[realyearoffset]`, `[standarddepot]` | One line each in the grammar (some optional) | Preserve values as text unless a consumer-specific meaning has been verified. |
| `[years]` | 2 ordered lines | Generic values in the merger model. |
| `[groundtex]` | 5 lines per occurrence | Two texture/path strings and three generic values; repeatable. Grande Porto has 10 entries. Do not infer all numeric meanings from position alone. |
| `[addseason]` | A preceding line ending in `:`, then 3 lines | Repeatable. The preceding label and three values are represented separately by the merger. Grande Porto has five such blocks. |
| `[trafficdensity_road]`, `[trafficdensity_passenger]` | 2 lines per occurrence | Repeatable; numeric-looking values are kept as generic values. |
| `[entrypoints]` | One count followed by that many 12-line records | The parser names fields `object_on_tile_index`, `id`, `line3`…`line10`, `tile_index`, and `name`. It converts ID and tile index to integers; preserve opaque positions. |
| `[map]` | Repeated: integer x, integer y, filename | Filename is expected to be `tile_<x>_<y>.map` in the merger model. Record order also supplies the tile ordinal used by merger-maintained tile-index references. |

### 4.1 Tile coordinates versus tile indices

These are distinct concepts:

- `x`/`y` in a `[map]` record identify the tile's integer grid coordinates and
  appear in its filename.
- A **tile index** in entrypoint/timetable/station-link data is an ordinal
  reference to the map's tile list. The merger shifts these ordinals by the
  number of tiles preceding a merged map; it does not replace them with x/y.
- The merger rebuilds tile filenames when translating a map on the integer
  grid. Its code does not transform the tile-local placement fields in every
  tile object/spline record.

Grande Porto is consistent with filenames and `[map]` coordinate pairs. The
precise physical units and tile width are **not specified by the parser**. The
sample has values near 300 in background-image parameters and local positions,
which suggests a roughly 300-unit tile scale, but this is an inference, not a
field-level guarantee. Keep coordinates/units configurable until independently
verified.

## 5. Tile text file: `tile_<x>_<y>.map`

The pinned merger grammar defines this broad order:

1. Initial nonempty comment/editor line.
2. `[version]` and its version value.
3. Optional `[terrain]`, `[water]`, `[variable_terrainlightmap]`, and
   `[variable_terrain]` marker sections in that order.
4. Zero or more spline records (`[spline]` / `[spline_h]`).
5. Zero or more scenery/object records (`[object]`, `[attachObj]`,
   `[splineAttachement]`, `[splineAttachement_repeater]`).

Records can contain nested modifier sections (`[varparent]`, terrain alignment,
`[rule]`, `[kill_rule]`) and preserved object-specific lines. The raw spellings
`[splineAttachement]` and `[splineAttachement_repeater]` are intentional; retain
the extra `e` in `Attachement`.

The marker-only tile flags refer to optional adjunct data as well as text
state. Their payloads are not encoded as ordinary values in the tile text.

### 5.1 Spline records

Payload positions below are line positions after the tag. Empty lines count
when they occupy an optional slot. The field labels follow the merger's model;
only IDs/link fields have clearly typed handling in its parser.

| Tag | Payload positions in file order |
| --- | --- |
| `[spline]` | 19: `line1`, spline filename, ID, previous spline ID, next spline ID, `x`, `z`, `y`, rotation, length, radius, gradient start, gradient end, cant start, cant end, skew start, skew end, `line18`, then the literal `mirror` marker or an empty line. |
| `[spline_h]` | 20: same as `[spline]`, with `delta_h` inserted after gradient end and before cant start. |

After a spline's fixed fields, the grammar can accept an optional
`[spline_terrain_align_2]` block and repeated `[rule]` / `[kill_rule]` blocks.
Each rule block has four payload lines. Most geometry/numeric values are kept
as strings by the merger; do not assume radians/metres for every field without
separate evidence.

IDs and nonzero `id_previous` / `id_next` values are offset during a merge by
the merger. The literal zero connection values are preserved for ordinary
(non-Chrono) tiles. This is evidence that the two ID positions are treated as
spline links by the merger, but other runtime semantics still belong to OMSI.

### 5.2 Scenery and attachment records

For the object-family records, the grammar requires a preceding nonempty
label/description line. The field names below reflect the parser's data model;
unknown `lineN` slots should remain opaque. Optional extra lines are bounded by
the next recognized structural tag, not by a universal fixed length.

| Tag | Fixed payload positions in file order |
| --- | --- |
| `[object]` | 10: `line1`, asset filename, object ID, `x`, `y`, `z`, rotation, pitch, bank, `line10`. |
| `[attachObj]` | 10: `line1`, asset filename, object ID, attached-to object ID, `line5`, attach-point index, rotation, pitch, bank, label count. |
| `[splineAttachement]` | 14: `line1`, asset filename, ID, `line4`, `x`, `z`, `y`, rotation, pitch, bank, interval, distance, `line13`, `line14`. |
| `[splineAttachement_repeater]` | 16: `line1`, `line2`, `line3`, asset filename, ID, `line6`, `x`, `z`, `y`, rotation, pitch, bank, interval, distance, `line15`, `line16`. |

A significant ordering detail: the merger maps `[object]` disk positions as
`x,y,z`; its internal constructor happens to store them as `pos_x,pos_z,pos_y`
and the serializer writes disk order back. Several spline/attachment model
records use `x,z,y` internally and in their grammar positions. A new parser
should follow the file-specific order rather than imposing one universal
coordinate tuple order.

After the fixed positions, supported modifiers include:

- `[varparent]` plus one integer-like parent ID.
- `[spline_terrain_align]` as a marker (no fixed value in the grammar).
- Repeated `[rule]` or `[kill_rule]`, each followed by four lines.
- Optional object-specific lines not matching the next structural tag.

OpenBus preserves `[attachObj]`'s attach-point index, three rotation strings,
and label-count string verbatim. It also exposes numeric attach-point and
rotation values when those strings parse, with a validity flag; failed
conversions do not discard or rewrite the raw text. The label count remains
opaque and does not determine how many optional tail lines are consumed.
Neither typed values nor the retained fields imply that parent-anchor
transforms are implemented.

The merger offsets object IDs, `AttachObj.attached_to_object_id`, and present
`varparent` values during merge. That makes them reference-sensitive fields;
copying records without remapping can break relationships.

### 5.3 Chrono tile records

Chrono tile overrides use the same general tile assets/records but start with a
comment and `[version]`, followed by an ordered mixture of selectors and
object/spline records. The merger models `[selspline]` and `[selobject]` as a
selector type plus an integer ID and variable following lines. Grande Porto
examples place `[typ]` and an asset path after `[selobject]`; retain such lines
as raw override data rather than assuming that this is the only legal payload.

Selectors and overridden tile objects are ID-sensitive. The merger shifts
selector IDs and the associated object/spline IDs and links when merging maps.
See [`chrono_tile_grammar.pg`](https://github.com/barteg77/omsi_map_merger/blob/00c2ceb189ab00ef11a59f7f3279dcfd7e7fd79a/chrono_tile_grammar.pg),
[`chrono_tile_parser.py`](https://github.com/barteg77/omsi_map_merger/blob/00c2ceb189ab00ef11a59f7f3279dcfd7e7fd79a/chrono_tile_parser.py),
and [`chrono_tile.py`](https://github.com/barteg77/omsi_map_merger/blob/00c2ceb189ab00ef11a59f7f3279dcfd7e7fd79a/chrono_tile.py).

## 6. Tile adjuncts and binary/opaque data

The merger registers these optional tile-related copy patterns: `<tile>.terrain`,
`<tile>.water`, `<tile>.LM.bmp`, a road-map bitmap under `texture/map/`, and
per-ground-texture DDS files under `texture/map/`. It does not decode their
binary payloads. Some patterns vary with `[groundtex]` count. This list is not
an exhaustive inventory of files emitted or consumed by OMSI.

| Observed Grande Porto item | Sample evidence | Safe current handling |
| --- | --- | --- |
| `<tile>.terrain` | 1,204 files; sampled files are 14,888 bytes and binary | Preserve/copy bytes. Layout, dimensions, compression, and numeric encoding are not documented here. |
| `<tile>.terrain_0.rdy` | 606 files; sampled files are 220,003 bytes and binary | Preserve/copy bytes. Meaning of the `rdy` payload was not reverse-engineered. |
| `<tile>.water` | 212 files of 20 bytes; one-to-one with the 212 sampled tile files containing `[water]` | Preserve/copy bytes. Do not infer the 20-byte layout from one map. |
| `<tile>.LM.bmp` | 1,208 BMPs, each 196,662 bytes in this map | Ordinary BMP container observed; OMSI lighting/map semantics are outside the merger parser. |
| `<tile>.prt` | 1,208 files, including 213 zero-byte files; a sampled nonempty file has CRLF text beginning with an asset path and numeric lines | Preserve/copy unchanged. Its exact schema and whether it is generated/cache data are unknown; it is omitted from the merger's registered sidecar patterns. |

Binary adjuncts and apparently generated files should not be silently discarded
just because the text parser can regenerate some tile data. Validate any
regeneration against OMSI itself and preserve unknown files through round trips.

## 7. Chrono directory package

A typical structure is:

```text
Chrono/<event>/
    Chrono.cfg
    Chrono_<language>.dsc
    tile_<x>_<y>.map          # zero or more tile overrides
    TTData/                   # optional event-specific timetable overrides
```

Grande Porto has 15 event directories. `Chrono.cfg` samples contain
`[startdate]` and `[enddate]` followed by `YYYYMMDD` values. A language
description sample contains `[name]`, `[description]`, and `[end]`. Do not assume
the merger validates these files: at the pinned v1.3 source, `Chrono.cfg` and
language descriptors are registered for copying, not parsed into typed fields.

The merger discovers `Chrono/*/` directories, reads tile overrides matching
tiles in `global.cfg`, and scans event `TTData/` similarly to standard
`TTData/`. Its README specifically excludes `chrono_#upd.cfg` from support.
The date boundary inclusivity, precedence among overlapping events, and all
possible Chrono descriptor fields were not established by the consulted parser
source; confirm them with OMSI before implementing runtime selection.

## 8. Timetable package (`TTData`)

The merger expects files with editor-style headers, divider lines, fixed record
order, and (for some files) dot separators. Header text and blank lines are
structural in these grammars. Avoid trimming or rebuilding them from assumed
field semantics.

| File | Record layout established by the merger | Important unknowns/cautions |
| --- | --- | --- |
| `Busstops.cfg` | File banner and header lines, then repeated `[busstop]` groups. Model fields are name, tile index, ID, exiting-passenger value, `line4`, `line5`, subname (7 positions). | Parser converts tile index and ID; the exiting-passenger model annotation and parser conversion are not fully consistent. Keep raw text until checked against OMSI. |
| `StnLinks.cfg` | Repeated optional comment + `[StnLink]`, then 9 fixed positions: `line1`, start busstop ID, end busstop ID, `line4`…`line9`; nested `[StnLink_entry]` groups have 7 fixed values: ID, `line2`, tile index, length, `line5`…`line7`, then optional chrono-file lines. | Busstop IDs, entry IDs, and tile indices are integer-converted. Most other values remain strings. |
| `*.ttr` | Header/dividers, then repeated comment/index line + `[track_entry]` + up to 7 positions: ID, `line2`, tile index, `line4`, length, `line6`, optional `line7`. | IDs/tile indices are integers; other columns are largely opaque. The leading comment/index line is part of each entry. |
| `*.ttl` | Header/dividers, optional `[userallowed]`, `[priority]` + one value, then repeated divider + `[newtour]` + tour name, optional AI-group name and `line3`; each tour can contain repeated preceding comment + `[addtrip]` + trip name, `line2`, departure time. | File/name associations are not resolved into typed cross-file objects by this parser. Preserve the exact trip/tour names and any blank slots. |
| `*.ttp` | Header/dividers; `[trip]` plus 3 lines; optional `[trainreverse]`; dotted `Stations` section with `[station]` or `[station_typ2]` records; dotted `Profiles` section followed by lines retained as profile data. | `[station]` has 8 positions: integer ID, interval, name, integer tile index, then `line5`…`line8`. `[station_typ2]` contains one integer ID. Profile subrecords are not fully semantically decoded by the merger. |

Grande Porto profile-related tags include `[profile]`,
`[profile_man_arr_time]`, `[profile_man_dep_time]`, and
`[profile_otherstopping]`. The merger retains profile-section lines rather
than assigning a complete semantic schema to those records.

Specific parser sources: [`busstops_parser.py`](https://github.com/barteg77/omsi_map_merger/blob/00c2ceb189ab00ef11a59f7f3279dcfd7e7fd79a/busstops_parser.py),
[`station_links_parser.py`](https://github.com/barteg77/omsi_map_merger/blob/00c2ceb189ab00ef11a59f7f3279dcfd7e7fd79a/station_links_parser.py),
[`track_parser.py`](https://github.com/barteg77/omsi_map_merger/blob/00c2ceb189ab00ef11a59f7f3279dcfd7e7fd79a/track_parser.py),
[`time_table_line_parser.py`](https://github.com/barteg77/omsi_map_merger/blob/00c2ceb189ab00ef11a59f7f3279dcfd7e7fd79a/time_table_line_parser.py),
and [`trip_parser.py`](https://github.com/barteg77/omsi_map_merger/blob/00c2ceb189ab00ef11a59f7f3279dcfd7e7fd79a/trip_parser.py).

### 8.1 Typical shapes (illustrative, not normative)

A line file in Grande Porto begins with a dashed title and metadata, then
`[userallowed]`, `[priority]`, `[newtour]`, and `[addtrip]` records. A trip file
uses the literal title `Time Table Trip File`, a `[trip]` block, a dotted
`Stations` heading, and repeated `[station]` records. A track file uses
`Time Table Track File` and `[track_entry]`. Exact dash/dot lengths and all
surrounding lines should be taken from the source file/grammar, not these
summaries.

## 9. AI list: `ailists.cfg`

The modern syntax represented by OMSI Map Merger is:

- `[aigroup_2]`: group name, optional HOF name, vehicle/type lines, then
  `[end]`. Lines may include trailing values (for example, weights in the
  Grande Porto `NormalCars` group); other groups have plain path lines. Keep
  each complete line rather than assuming every vehicle row has the same number
  of whitespace-separated tokens.
- `[aigroup_depot]`: depot/group name and HOF name, followed by one or more
  `[aigroup_depot_typgroup_2]` groups.
- `[aigroup_depot_typgroup_2]`: vehicle type/config path, variable vehicle
  assignment/repaint lines, then `[end]`.

The parser preserves vehicle and repaint records mostly as line collections;
it does not prove the meaning of each field within a line. The sample contains
`Vehicles\...` references, group names such as `NormalCars` and `Trucks`, and a
depot named `STCP VN` associated with `GrandePorto2022`.

Older files use forms such as `[ailist]`, `[aigroup]`, and
`[aigroup_depot_typgroup]`; the forum tutorial describes those historical
forms. OMSI Map Merger v1.3 explicitly does not support old-type AI lists, so
do not treat its modern grammar as a universal parser for every OMSI map era.
The same forum thread's later OMSI 2-era reply shows `[aigroup_2]` and
`[aigroup_depot_typgroup_2]`, consistent with the modern syntax found in
Grande Porto.
See [`ailists_grammar.pg`](https://github.com/barteg77/omsi_map_merger/blob/00c2ceb189ab00ef11a59f7f3279dcfd7e7fd79a/ailists_grammar.pg)
and [`ailists_parser.py`](https://github.com/barteg77/omsi_map_merger/blob/00c2ceb189ab00ef11a59f7f3279dcfd7e7fd79a/ailists_parser.py).

## 10. IDs, indices, and cross-file references

The merger's implementation demonstrates that several numeric fields must be
updated together when combining maps. This is especially useful for future
format implementations, but it is still a description of the merger's
remapping logic, not a complete OMSI reference manual.

| Namespace/reference | Fields that the merger offsets |
| --- | --- |
| Numeric object/spline IDs | Tile object IDs, spline IDs, nonzero previous/next spline IDs, attachment target IDs, present `varparent` IDs, entrypoint IDs, timetable stop/link/track/trip-station IDs, and Chrono selector/record IDs. |
| Tile ordinal | Global entrypoint tile index, busstop tile index, station-link entry tile index, track-entry tile index, regular trip station tile index; tile-index offsets are based on the number/order of `[map]` entries. |
| Ground-texture ordinal | The merger combines `[groundtex]` records and adjusts registered per-tile DDS sidecar filename ordinals. Preserving or replacing an incoming map's base ground texture changes that adjustment; do not infer that every numeric texture-looking field is rewritten. |

Some fields that look like references are **not** adjusted by the merger's
mutators; for example, string names and several opaque line fields. Do not
assume every integer in a `.map`, `.ttl`, `.ttr`, or `.ttp` file is an object ID.
Use typed field knowledge and a reference graph, not blanket numeric rewrites.

The relevant implementation is in [`Tile.change_ids`](https://github.com/barteg77/omsi_map_merger/blob/00c2ceb189ab00ef11a59f7f3279dcfd7e7fd79a/tile.py),
[`GlobalConfig.change_ids_and_tile_indices`](https://github.com/barteg77/omsi_map_merger/blob/00c2ceb189ab00ef11a59f7f3279dcfd7e7fd79a/global_config.py),
[`Timetable.change_ids_and_tile_indices`](https://github.com/barteg77/omsi_map_merger/blob/00c2ceb189ab00ef11a59f7f3279dcfd7e7fd79a/timetable.py),
[`ChronoTile.change_ids`](https://github.com/barteg77/omsi_map_merger/blob/00c2ceb189ab00ef11a59f7f3279dcfd7e7fd79a/chrono_tile.py),
and the merge orchestration in [`omsi_map_merger.py`](https://github.com/barteg77/omsi_map_merger/blob/00c2ceb189ab00ef11a59f7f3279dcfd7e7fd79a/omsi_map_merger.py).

## 11. Grande Porto 2022 validation snapshot

The following observations were collected read-only from:

`C:\Program Files (x86)\Steam\steamapps\common\OMSI 2\maps\Grande Porto 2022`

They validate that the described file families and record types occur in a
real installed map; they do **not** prove every OMSI map uses the same versions
or optional sections.

| Check | Observed result |
| --- | --- |
| Package size/layout | 5,665 files at the map root; the recursive tree contains 10,818 files across `Chrono/`, `TTData/`, and `texture/`. |
| Map root and tile manifest | 1,208 base `tile_<x>_<y>.map` files and 1,208 `[map]` records in `global.cfg`; all listed tile files exist and no base tile is unlisted. |
| Tile grid | x coordinates range `-34…25`, y coordinates `-21…57`; the bounding rectangle is sparse (1,208 of 4,740 possible pairs). |
| Global metadata | `global.cfg` has 9,463 lines, `[version]` value `14`, 256 entrypoints, 10 `[groundtex]` records, five `[addseason]` records, repeated road/passenger density records. |
| Root tile records | `[version]` and `[terrain]` occur in all 1,208 base tiles; `[variable_terrainlightmap]` and `[variable_terrain]` occur in 1,204; `[water]` occurs in 212. Other records include `[spline]`, `[spline_h]`, `[object]`, `[attachObj]`, `[splineAttachement]`, `[splineAttachement_repeater]`, `[varparent]`, `[rule]`, and `[kill_rule]`. |
| Tile record frequencies | In base tiles: `[object]` 147,881 occurrences; `[rule]` 142,354; `[splineAttachement]` 131,136; `[spline]` 119,978; `[splineAttachement_repeater]` 8,614; `[spline_h]` 3,762; `[varparent]` 3,339; `[attachObj]` 210. Frequency counts include repeated records and are specific to this map. |
| Chrono | 15 event directories; 60 tile override files (four per event in this map), 15 `Chrono.cfg` files, and 75 language `.dsc` files (five per event). Sample date values are `20201003` and `20201010`; a sample override uses `[selobject]`, an ID, `[typ]`, and a scenery-object path. |
| Standard timetable | `TTData` contains 242 `.ttl`, 1,405 `.ttr`, 1,402 `.ttp`, and two `.cfg` files (`Busstops.cfg`, `StnLinks.cfg`). Sample title banners identify OMSI version 2.3.004. |
| AI list | `ailists.cfg` uses `[aigroup_2]`, `[aigroup_depot]`, and `[aigroup_depot_typgroup_2]`; vehicle and repaint entries show why rows should initially be preserved as raw lines. |
| Tile adjuncts | 1,208 `.map.LM.bmp`; 1,208 `.map.prt`; 1,204 `.map.terrain`; 606 `.map.terrain_0.rdy`; 212 `.map.water`. Counts/sizes are specific to this map. |
| Other root formats | `GrandePorto2022.hof` contains `[name]`, `[servicetrip]`, `[global_strings]`, `[addbusstop]`, `[addterminus]`, `[addterminus_allexit]`, `[infosystem_trip]`, and `[infosystem_busstop_list]`. `timeline.prt.cfg` lists 15 Chrono paths followed by an additional opaque block; neither is structurally parsed by the merger. |

Examples inspected include `global.cfg` entrypoints around lines 349–362,
`tile_-10_15.map` spline and rule data around lines 13–38,
`TTData\A. V. Pacense 64.ttl`, `TTData\ADPL Volta.ttp`,
`TTData\ADPL Volta.ttr`, and
`Chrono\Festas_S.VerissimoParanhos\tile_0_3.map`. These are validation
locations, not a promise that line numbers are stable across map releases.

### 11.1 AI-list and signal-route evidence

The installed map's `ailists.cfg` uses the modern group sections
`[aigroup_2]`, `[aigroup_depot]`, and `[aigroup_depot_typgroup_2]`; no legacy
`[ailist]` or `[aigroup]` sections were observed in this file. A sampled
`[aigroup_2]` named `NormalCars` contains vehicle paths such as
`vehicles\VW_Golf_2\AI_VW_Golf_2.bus` followed by a numeric value. Depot groups
associate vehicle groups with HOF names and include variable-length vehicle
and repaint data. These observations establish that the file selects vehicle
assets and depot/HOF combinations, but not a complete grammar for route
selection, spawn placement, dispatch timing, or vehicle movement. Preserve
unparsed rows rather than treating the trailing numeric value as a universally
understood weight or probability.

The map's `signalroutes.cfg` is only a seven-line header/template and contains
no actual signal-route records. This does not establish that the map has no
traffic signals: signal objects or scripts may carry additional behavior. It
does mean the inspected route file is not enough to derive a signal state
machine or AI stopping rules. Together with the unresolved timetable-to-route
and spawn scheduling behavior, the installed evidence is insufficient for a
dependable AI-traffic or signal-compliance implementation. Defer those runtime
features until route, signal, and scheduling data can be verified; the current
OpenBus static vehicle-grid option is a development aid, not OMSI AI traffic.

## 12. Known gaps and implementation cautions

The consulted sources do not establish:

- The meanings/units of many global numeric positions (`[backgroundimage]`,
  `[mapcam]`, `[groundtex]` trailing values, density values, entrypoint `lineN`
  fields) beyond their order and count.
- A complete, version-independent semantic definition of every tile object,
  spline, rule, attachment, or terrain-alignment field.
- The binary structure of `.terrain`, `.terrain_0.rdy`, or `.water`, the exact
  OMSI meaning of `.LM.bmp`, or the schema/runtime purpose of `.prt`.
- Full semantics of all `TTData` profile fields, `station_typ2`, station-link
  trailing fields, Chrono override lines, date inclusivity/precedence, and
  Chrono update files.
- A complete modern/legacy AI-list cross-version grammar, or the HOF,
  `signalroutes.cfg`, unscheduled-traffic, and global/chrono descriptor formats.
- A guaranteed encoding for every text file or the physical tile width/units
  for every map mode.

For safe future implementation, start with exact line-preserving parsing for
well-understood records, record the original version, retain unknown fields,
and treat binary/unsupported files as opaque assets. Add a field to a typed
model only when parser evidence, multiple real maps, or controlled OMSI tests
support it. Check source values and reference counts before writing; then test
both parse/serialize round trips and actual loading in OMSI.

## 13. Implementation checklist

- [ ] Identify the map root and parse `global.cfg` before resolving tiles.
- [ ] Decode each file family independently and retain BOM/newline provenance.
- [ ] Preserve blank lines, fixed separators, record order, unknown lines, and
      original asset paths.
- [ ] Validate `[map]` coordinates, filename, duplicate entries, missing tile
      files, and the ordinal tile-index mapping.
- [ ] Parse each tile using its version and ordered groups; preserve object
      modifiers and unknown record families.
- [ ] Track object/spline IDs and references explicitly; avoid global search and
      replace of numeric values.
- [ ] Parse `TTData` and Chrono as linked packages; do not assume filename
      uniqueness or a one-to-one relationship without checking.
- [ ] Preserve all unparsed sidecars and map-level files byte-for-byte.
- [ ] Validate outputs against a real OMSI load, and use Grande Porto plus other
      maps to prevent overfitting to a single map/version.

## References

- [OMSI Map Merger v1.3 README and limitations](https://github.com/barteg77/omsi_map_merger/blob/00c2ceb189ab00ef11a59f7f3279dcfd7e7fd79a/README.md)
- [English user manual](https://github.com/barteg77/omsi_map_merger/blob/00c2ceb189ab00ef11a59f7f3279dcfd7e7fd79a/MANUAL_en.md)
- [`global.cfg` grammar and parser](https://github.com/barteg77/omsi_map_merger/blob/00c2ceb189ab00ef11a59f7f3279dcfd7e7fd79a/global_config_grammar.pg), [parser actions](https://github.com/barteg77/omsi_map_merger/blob/00c2ceb189ab00ef11a59f7f3279dcfd7e7fd79a/global_config_parser.py)
- [Tile grammar](https://github.com/barteg77/omsi_map_merger/blob/00c2ceb189ab00ef11a59f7f3279dcfd7e7fd79a/tile_grammar.pg), [tile parser](https://github.com/barteg77/omsi_map_merger/blob/00c2ceb189ab00ef11a59f7f3279dcfd7e7fd79a/tile_parser.py), [tile model](https://github.com/barteg77/omsi_map_merger/blob/00c2ceb189ab00ef11a59f7f3279dcfd7e7fd79a/tile.py)
- [Chrono tile grammar](https://github.com/barteg77/omsi_map_merger/blob/00c2ceb189ab00ef11a59f7f3279dcfd7e7fd79a/chrono_tile_grammar.pg), [Chrono discovery/loading](https://github.com/barteg77/omsi_map_merger/blob/00c2ceb189ab00ef11a59f7f3279dcfd7e7fd79a/chrono.py)
- [Timetable parsers and models](https://github.com/barteg77/omsi_map_merger/tree/00c2ceb189ab00ef11a59f7f3279dcfd7e7fd79a)
- [Older AI-list tutorial (historical forum reference)](https://forum.omnibussimulator.de/forum/index.php?thread/11880-tut-changing-adjusting-the-ai-list-ailist/)
