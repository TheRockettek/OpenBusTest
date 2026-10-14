# Configuration implementation status

These pages describe OpenBus runtime support, not complete OMSI compatibility. A keyword being parsed, retained, or consumed for parser alignment does not mean its OMSI behavior is implemented.

## Status meanings

- **Partial** — a subset of the keyword's behavior is active; the limitations are stated.
- **Parsed only** — data is read or preserved, but it does not produce the relevant runtime behavior.
- **Not implemented** — there is no runtime support for the keyword/feature.
- **Unsupported input / not a keyword gap** — the input references missing or conflicting files/records; format support alone cannot repair it.

## By file family

- [BUS and bus-model configuration](IMPLEMENTATION_STATUS_BUS.md) — `.bus`, bus model CFG, and linked passenger/path data.
- [Scenery configuration](IMPLEMENTATION_STATUS_SCENERY.md) — `.sco` and its model CFG, placement, scripts, and collision.
- [Map configuration](IMPLEMENTATION_STATUS_MAP.md) — `global.cfg`, tile `.map`, terrain/texture sidecars, and `.sli` roads.
- [Audio configuration](IMPLEMENTATION_STATUS_AUDIO.md) — sound CFG records, BUS audio references, trigger behavior, and supported audio data.

The detailed OMSI format references describe observed record layouts; these status pages describe what OpenBus currently does with them. See [TODO.md](../TODO.md) for the broader project completion backlog.
