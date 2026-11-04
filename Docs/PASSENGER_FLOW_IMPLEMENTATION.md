# Passenger flow implementation status

Passenger service is represented by timetable stop/link data and a deterministic
service runtime. Legacy timetable `[station]` calls that match a scheduled AI
bus route now constrain its speed, stop the bus, and run the service dwell and
boarding flow. Station-ID-only trips can form scheduled bus routes when every
directed station link has a complete ordered path. Player buses can board and
alight at mapped calls when stationary, aligned with their approach direction,
and within 20 metres of the stop. Waiting passengers walk from their authored
waiting positions to the actual front entry and remain there until the entry
door's `pax_entryN_open` variable is set. Player-bus and scheduled-AI entry
waiting positions form a single line beside the bus. Exiting similarly requires
an open
`pax_exitN_open` door. The player stop remains active until the bus moves away;
when the bus starts moving, service is cancelled before passengers advance and
an interrupted boarder returns to the shared queue. Boarding riders receive a
deterministic service and downstream stop from the located timetable routes
available when their waiting record is first assigned. Only a bus with the
same registered route identity that serves the selected stop near the rider's
destination can board that rider; a matching timetable key and line alone do
not make separate route registrations interchangeable. Cancellation returns
the same journey to the shared queue; routes registered later do not change
assigned journeys. Journey records retain exact origin and destination call
indices, so repeated stop IDs do not match at the wrong call. Queue counts and
reservations remain shared by stop ID, but riders on a route with repeated IDs
are placed beside their assigned call. Before boarding, a rider is assigned a
destination 15–45 m forward on a nearby directed human spline where available.
If no suitable spline exists, the mapped stop is used instead and the fallback
is logged.
When the human lane network is still being prepared, an eligible scheduled bus
holds its dwell rather than departing solely because destination selection is
pending. If no located mapped journey is available, the rider waits and the
stop is reported in the log.
Any riders remaining at a route's final mapped call alight before the
scheduled bus ends its trip. Waiting queues and boarding/alighting positions
use the correct passenger side for left- and right-hand traffic maps, with
curb-side and queue offsets derived from the map's north-zero road heading.
Ambient map pedestrians remain a separate population.
Once within 200 m of the generated destination (or the mapped stop for the
fallback), each seated rider requests its assigned stop via `int_haltewunsch`
when the vehicle script supplies the trigger.
All remaining route calls are checked, so an intervening stop does not hide a
nearby later destination. Requests for later destinations are rearmed after
serving an intermediate stop, whose door cycle may have cleared the bell.
When the bus arrives, the configured exterior exit position for each seat is
checked against the rider's destination before normal destination-based
alighting. A rider left aboard because a repeated stop ID is too far away can
request a later call again. The mapped-stop fallback retains its originally
assigned world position. Player-bus exits are rechecked during alighting, so
a rider waits aboard if the stopped bus drifts out of range. Forced unloading
at terminal calls or on service cancellation can still override the limit;
those cases need further work.
After alighting, a separate pedestrian actor follows its assigned directed
route if the exit is within 0.5 m horizontally and vertically of a route point
from up to 25 m behind the lane start through the generated destination. The
short connector may not move toward the bus. Otherwise the rider disappears
and a warning is logged. This conservative proximity check does not establish
road safety or guarantee a handoff for every sidewalk; visual and end-to-end
validation remain outstanding.
Player stop discovery uses a timetable-route catalog registered independently
of scheduled AI bus instances, so setting the AI bus population to zero does
not disable player boarding. The catalog still requires a loadable bus
configuration and a routable, matched timetable route.

Waiting passengers render using the configured human models.
Cabin positions use the vehicle model's coordinate convention and model-root
height. Occupied seats use a skeletal sitting pose with the hips aligned to
the authored seat height; zero-height positions remain standing. Passengers
walk to the seat before sitting and stand up before alighting. Missing or
incomplete model skin weights use an approximate seated-only geometric rig,
while normal walking retains its existing animation.
Cabin traversal uses the length of the rendered path at a nominal 1.4 m/s,
with time for the sit/stand transition and a minimum 0.75-second interval.
The same duration drives simulation and rendering; visual validation remains
outstanding.
On multi-door buses, boarders request and use the first configured entry.
Alighters request and use the shortest reachable configured exit from their
seat's nearest cabin path point. If no exit is reachable, they use the nearest
known exit, or the first configured exit if its geometry is unavailable. The
selected entry must be open before boarding starts. An alighter with a
connected path can stand and walk to a waiting point up to 0.5 m short of the
selected exit along the path while it is closed, but cannot pass it until it
opens. Without a known
connected exit path, the alighter remains seated until the door opens.
Changing to the next passenger re-evaluates the door and waiting point. Only
the head of the FIFO queue can exit, but queued alighters now walk concurrently
toward spaced waiting positions along their seat-to-exit paths. Their progress
is retained when each rider is promoted to the active alighter. Door slots
beyond index 7 are not managed. Runtime probes cover closed-door waiting and
FIFO promotion; visual validation of the queue remains outstanding.
Waiting passengers request entry only when a seat is free or an alighter is
due to free one; a reserved boarder keeps the request until boarding completes.
Full buses leave the remaining queue in place without holding the entry request.
Queues are shared by stop ID across registered scheduled routes and replenished
to at least their initial size when a scheduled AI bus departs or the player bus
leaves a served stop. When multiple calls share an ID, their queue renders at
the active boarding call or otherwise at the nearest visible call. Queue
rendering depends on the registered timetable route, not on the AI bus model
finishing its asynchronous load, so passengers remain visible and available to
the player during loading.

## Timetable station data

The `.ttp` loader retains valid `[station]` records as an ordered station ID,
track-entry index, name, tile index, and four verbatim trailing fields, and reads
`[station_typ2]` IDs. It accepts station-based bus trips without a matching
`.ttr` as passenger timetables, including departure times from `.ttl`. It also
reads stop IDs from `Busstops.cfg` and directed endpoint IDs and path-entry
values from `StnLinks.cfg`. Trip station IDs are matched against unique stop
IDs; consecutive IDs are considered linked only when exactly one directed
station link joins them. Station-link path entries retain the source spline,
path index, tile index, and length fields. Legacy TTP station offsets project
to the corresponding position on their track entry, including reversed and
subdivided runtime route segments. Station-ID-only calls use the end of their
incoming station-link path.

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
verbatim. Station-link path entries do not themselves provide a
pedestrian-world position, so no coordinates are fabricated from those values.

## Script ticket and cash macros

Vehicle scripts in both the native OSC and Lua backends can use the following
per-vehicle `Variables` convention:

* The map's global `[ticketpack]` path is loaded from the OMSI root. The loader
  retains `stamper_prop`, `ticketbuy_prop`, `chattiness`, `whinge_prop`, and
  ordered ticket names, station limits, age limits, values, and display strings.
  It publishes `ticket_name_<index>` and `ticket_value_<index>` for
  `getticketname` and `getticketvalue`. A vehicle-local value overrides the map
  value. Indices must be whole, non-negative integers; missing/invalid entries
  return an empty string or `0`.
* The map's `[moneysystem]` config is loaded for its ordered `[coin]` records.
  `givechangecoin` consumes a whole, non-negative coin index, looks up the
  configured denomination, and updates
  `cash_dispensed_total`, `cash_last_coin_value`, and
  `cash_dispensed_coin_count` on that vehicle, plus
  `cash_last_coin_index` and `cash_coin_dispensed_count_<index>`. Invalid
  indices or denominations are ignored. This is script-visible accounting.
  `GiveChangeCoin` counter changes also trigger a short-lived generic coin
  visual at the configured change point, falling back to the money point or
  ticket machine. Configured coin models are retained as
  `cash_coin_model_<index>` but are not loaded or rendered; coin sound and
  physical dispensing are also absent.
* When ticket sales are enabled, a waiting passenger may buy the cheapest
  ticket matching the rider's `[age]` value and downstream station count, with
  purchase and stamping chances from the ticket pack. A purchaser's approach,
  machine wait, and return durations are simulated using the first configured
  ticket-machine position, and the passenger is rendered walking to the
  machine and back. Simulated tender uses the configured coin denominations.
  If change is owed, boarding remains blocked until driver scripts call
  `GiveChangeCoin` for enough change; the passenger then waits for ticket
  issuance and returns to the entry queue. Sale state is exposed through
  `passenger_ticket_sale_*` vehicle variables.
* `gethumancountonseat` and `gethumancountonpathlink` read the numeric vehicle
  variables `human_count_on_seat_<index>` and
  `human_count_on_path_link_<index>`. These are macro surfaces for an owning
  passenger runtime. Vehicle scripts receive per-seat counts and the active
  boarding or alighting passenger's cabin path-link occupancy. The simulation
  uses the same selected door and path as rendering, and updates occupancy
  without relying on a draw pass. Direct fallback paths have no authored
  path-link occupancy.

The same macros are available to player and AI vehicle scripts because both
use `ScriptRuntime` and fall back to map-shared ticket/currency values. Ticket
selection and the change/ticket wait gate are implemented, but passenger cash
is simulated, money is not physically transferred, configured coin models and
sound are not used, and scenery/world path-link occupancy remains unimplemented.
Passengers visit configured money points and, when change is due, change points
before returning to the entry queue. When stamping is selected, they visit the
first configured stamper, pause, and return. These actions do not animate the
hand, cash, ticket, or coins. Ticketing still needs in-game validation.

The player bus's physics mass includes seated passengers whose `.hum` files
specify `[mass]`. OMSI human mass values are interpreted as tonnes and converted
to kilograms. Each rider is represented by a fixed mass body at the authored
seat position, so occupied seats affect total mass and the combined center of
gravity. Human body inertia and native physics parity remain approximate and
need in-game validation.

## Service runtime

`MapPassengerServiceRuntime` maintains per-stop waiting queues, fixed seat
occupancy, alighting, a dwell timer, and boarding at a bounded rate. A
passenger is reserved from the shared queue while the boarding interval runs,
then assigned to a seat on completion. Shared waiting records retain their model,
service, destination, arrival order and physical queue slot. A bus selects the
oldest matching rider without removing or moving passengers for other services.
Reservations retain their slots until boarding finishes or is cancelled, and
returned riders retain their journeys even if another bus reserved a rider
concurrently. Replenished demand exceeding the stop's available queue slots is
held as pending demand until a slot is free. Model indices are deterministic
and are not globally unique passenger identifiers. Assignments use mapped stop
IDs, not exact timetable-call indices, and are limited to routes known at the
time of assignment.
During boarding and alighting, the
configured human model animates between the roadside and seat. If the authored
cabin entry/exit points and directed path-link graph connect the endpoints,
movement follows that graph; otherwise it uses a direct fallback path. Seated
passengers render at their configured cabin positions. Walking passengers face
the active path segment and retain the authored endpoint orientation when
movement completes. The queue scale helper is separate from the ambient
pedestrian population target and applies only to scheduled queue sizes. The
component has deterministic probe coverage and is owned by each vehicle using
the configured passenger seats.
Human gait phase, resting upper-arm pose, arm swing, hip sway, and torso bob
use the `.hum` `[walk_param]` step length, upper-arm beta, arm-swing multiplier,
hip setting, and waist-animation setting. The right-shoulder coordinates in
`[links]` set the approximate arm pivot and are mirrored for the other side;
the right-hip coordinates likewise set the hip-sway pivot, and the right-knee
coordinates set the lower-leg gait pivot. Right-elbow and right-hand
coordinates set approximate forearm and wrist flex pivots while walking; neck
Y/Z coordinates set an approximate head-stabilization pivot, and waist Y/Z
coordinates anchor gait-linked torso lean. These controls drive approximate
mesh deformations rather than native limb animation; human `[seatheight]` and
voice are not yet applied.
Scheduled AI buses stop at mapped timetable calls and board waiting
passengers into available seats during an eight-second dwell. Queue consumption
is serialized each update against the shared remaining count, preventing two
dwelling buses from boarding the same passenger. First boarding priority
rotates independently within each shared stop queue, so buses serving other
stops do not affect their order. Queues for registered timetable routes'
mapped stops are rendered within 75 meters of the main camera,
independent of the ambient AI pedestrian population cap. The waiting count is
a fixed nominal demand per mapped stop multiplied by the scheduled-human
scale; it is not a shared map-wide demand model.
Passengers are rendered only at authored passenger waiting-area positions
matched to their stop. Stops without a matching area do not generate fallback
waiting, boarding, or alighting actors or queue debug markers; a warning is
logged once when a visible queue has no authored area. Timetable passenger
demand and service counts remain active at those stops.
Passenger service and the script's departure/door-close handshake are legitimate
waits, including for following traffic, and do not trigger stuck-bus removal.

## Remaining work

Queue demand is a fixed nominal count per mapped stop rather than a shared
map-wide demand model. Waiting people use a subtle idle gait before boarding;
boarding and alighting use the full movement animation. Ticket selection and
payment transactions remain incomplete; configured coin models, audio, and
native dispensing animation remain unimplemented.
