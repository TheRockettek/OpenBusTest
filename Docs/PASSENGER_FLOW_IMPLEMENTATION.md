# Passenger flow implementation status

Passenger service is represented by timetable stop/link data and a deterministic
service runtime. Legacy timetable `[station]` calls that match a scheduled AI
bus route now constrain its speed, stop the bus, and run the service dwell and
boarding flow. Waiting passengers are rendered as stationary humans near an
active scheduled stop, using the configured human models and the vehicle's
remaining shared queue count. Station-ID-only trips can form scheduled bus
routes when every directed station link has a complete ordered path. Player
buses can board and alight at mapped stops when stationary and aligned with
their approach direction. Passengers board with the next stop as their
destination and alight there; any riders remaining at a route's final mapped
call alight before the scheduled bus ends its trip. Ambient map pedestrians
remain a separate population.

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
is warned about and skipped. The `Busstops.cfg` and `StnLinks.cfg` layouts
follow the OMSI Map Merger parsers and the openOMSI timetable reader; fields
not used for identity remain opaque:

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

* The map's global `[ticketpack]` path is loaded from the OMSI root. The loader
  reads ordered `[ticket]` blocks and publishes `ticket_name_<index>` and
  `ticket_value_<index>` for `getticketname` and `getticketvalue`. A vehicle-
  local value overrides the map value. Indices must be whole, non-negative
  integers; missing/invalid entries return an empty string or `0`.
* The map's `[moneysystem]` config is loaded for its ordered `[coin]` records.
  `givechangecoin` consumes a whole, non-negative coin index, looks up the
  configured denomination, and updates
  `cash_dispensed_total`, `cash_last_coin_value`, and
  `cash_dispensed_coin_count` on that vehicle, plus
  `cash_last_coin_index` and `cash_coin_dispensed_count_<index>`. Invalid
  indices or denominations are ignored. This is script-visible accounting;
  coin models are retained as `cash_coin_model_<index>` but not yet animated,
  sounded, or physically dispensed.
* `gethumancountonseat` and `gethumancountonpathlink` read the numeric vehicle
  variables `human_count_on_seat_<index>` and
  `human_count_on_path_link_<index>`. These are macro surfaces for an owning
  passenger runtime. Vehicle scripts now receive per-seat counts from the
  service state initialized from `passengercabin.cfg`; path-link values are not
  populated yet.

The same macros are available to player and AI vehicle scripts because both
use `ScriptRuntime` and fall back to map-shared ticket/currency values. Ticket
and coin configuration loading is active, but passenger ticket selection,
payment transactions, physical coin dispensing, and live path-link occupancy
remain unimplemented.

## Service runtime

`MapPassengerServiceRuntime` maintains per-stop waiting queues, fixed seat
occupancy, alighting, a dwell timer, and boarding at a bounded rate. A
passenger is reserved from the shared queue while the boarding interval runs,
then assigned to a seat on completion. During that interval, a human model is
drawn walking from the stop queue toward the bus's roadside flank. This is a
visual approach approximation, not yet a path through the authored cabin
entry/path-link graph. Its queue scale helper is separate from the ambient
pedestrian population target and applies only to scheduled queue sizes. The
component has deterministic probe coverage and is owned by each vehicle using
the configured passenger seats.
Scheduled AI buses stop at mapped timetable calls and board waiting
passengers into available seats during an eight-second dwell. Queue consumption
is serialized each update against the shared remaining count, preventing two
dwelling buses from boarding the same passenger. Bus iteration order currently
determines which bus boards first. Queues for registered scheduled AI-bus
routes' mapped stops are rendered within 75 meters of the main camera,
independent of the ambient AI pedestrian population cap. The waiting count is
a fixed nominal demand per mapped stop multiplied by the scheduled-human
scale; it is not a shared map-wide demand model.

## Remaining work

Station-link stop locations currently use the beginning of each matched
route-entry segment; accurate within-spline stop projection still needs
verification against map geometry. Queues are shared by stop ID across the
registered scheduled routes, but demand is still a fixed nominal count and bus
iteration order decides boarding priority. Waiting people remain stationary
until their boarding transition; boarded passengers do not yet traverse cabin
path links, and occupied seats are exposed to scripts but are not rendered as
seated humans. Ticket selection, payment transactions, and audible or physical
coin output remain incomplete.
