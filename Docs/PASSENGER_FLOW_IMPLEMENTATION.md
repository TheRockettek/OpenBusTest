# Passenger flow implementation status

Passenger service is represented by timetable stop/link data and a deterministic
service runtime. Legacy timetable `[station]` calls that match a scheduled AI
bus route now constrain its speed, stop the bus, and run the service dwell and
boarding flow. Waiting passengers are rendered as stationary humans near an
active scheduled stop, using the configured human models and the vehicle's
remaining queue count. This is still an early AI-only slice: station-link-only
trips, player-bus stop detection, seated passenger rendering, destination-aware
alighting, and ticket sales are not implemented. Ambient map pedestrians remain
a separate population.

## Timetable station data

The `.ttp` loader retains valid `[station]` records as an ordered station ID,
track-entry index, name, tile index, and four verbatim trailing fields, and reads
`[station_typ2]` IDs. It accepts station-based bus trips without a matching
`.ttr` as passenger timetables, including departure times from `.ttl`. It also
reads stop IDs from `Busstops.cfg` and directed endpoint IDs and path-entry
values from `StnLinks.cfg`. Trip station IDs are matched against unique stop
IDs; consecutive IDs are considered linked only when exactly one directed
station link joins them. Station-link path entries retain the source spline,
path index, tile index, and length fields, but they are not yet projected onto
the renderer's map-space route geometry.

The eight-position `[station]` record layout is documented in
[`OMSI2_MAP_FORMAT_REFERENCE.md`](OMSI2_MAP_FORMAT_REFERENCE.md). An invalid ID,
negative track-entry index, empty name, invalid tile index, or incomplete record
is warned about and skipped. The `Busstops.cfg` and `StnLinks.cfg` layouts follow the OMSI Map Merger
parsers and the openOMSI timetable reader; fields not used for identity remain
opaque:

* [OMSI Map Merger trip parser](https://github.com/barteg77/omsi_map_merger/blob/00c2ceb189ab00ef11a59f7f3279dcfd7e7fd79a/trip_parser.py)
* [OMSI Map Merger station-link parser](https://github.com/barteg77/omsi_map_merger/blob/00c2ceb189ab00ef11a59f7f3279dcfd7e7fd79a/station_links_parser.py)
* [openOMSI timetable parser](https://github.com/openOMSI-org/openOMSI/blob/main/crates/omsi-timetable/src/lib.rs)

Bus-stop numeric parameters, station-link length/parameters, and all seven
station-link entry values are validated before a record is accepted. The first
four station-link entry fields are typed as spline ID, path index, tile index,
and length, based on the openOMSI format reader; the last three values remain
verbatim. The entry does not itself provide a pedestrian-world position, so no
coordinates are fabricated from these values.

## Script ticket and cash macros

Vehicle scripts in both the native OSC and Lua backends can use the following
per-vehicle `Variables` convention:

* `ticket_name_<index>` (string) and `ticket_value_<index>` (number) provide
  the lookup values for `getticketname` and `getticketvalue`. Indices are
  zero-based, non-negative integers. Missing/invalid entries return an empty
  string or `0`.
* `givechangecoin` consumes one positive finite amount and updates
  `cash_dispensed_total`, `cash_last_coin_value`, and
  `cash_dispensed_coin_count` on that vehicle. Non-positive or non-finite
  amounts are ignored. These variables provide a script-visible accounting
  surface; this does not yet animate, sound, or physically model a coin.
* `gethumancountonseat` and `gethumancountonpathlink` read the numeric vehicle
  variables `human_count_on_seat_<index>` and
  `human_count_on_path_link_<index>`. These are macro surfaces for an owning
  passenger runtime. Vehicle scripts now receive per-seat counts from the
  service state initialized from `passengercabin.cfg`; path-link values are not
  populated yet.

The same macros are available to player and AI vehicle scripts because both
use `ScriptRuntime` with vehicle-local state. Ticket-pack and currency-file
parsing, passenger-seat/path occupancy queries, and the station service flow
remain unimplemented; their existing macro fallbacks should not be interpreted
as live passenger data.

## Service runtime

`MapPassengerServiceRuntime` maintains per-stop waiting queues, fixed seat
occupancy, alighting, a dwell timer, and boarding at a bounded rate. Its queue
scale helper is separate from the ambient pedestrian population target and
applies only to scheduled queue sizes. The component has deterministic probe coverage and is owned by each vehicle
using the configured passenger seats. Scheduled AI buses initialize stop
queues at departure, stop at mapped timetable calls, and board waiting
passengers into available seats during an eight-second dwell. Rendered queues
are limited to active scheduled AI-bus calls within 75 meters of the main
camera. The waiting count is a fixed nominal demand per mapped stop multiplied
by the scheduled-human scale; it is not a shared map-wide demand model.

## Remaining work

Resolve station-link path entries against map geometry, add passenger
destination/alighting logic, and share demand across trips rather than
initializing a separate queue per bus. Add player-bus stop detection, seated
passenger rendering, ticket-pack/currency loading, ticket sales, and audible or
physical coin output.
