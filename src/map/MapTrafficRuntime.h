#pragma once

#include "map/MapSceneryPlacement.h"
#include "map/MapTrafficSignal.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace openbus::map {

struct MapTrafficGroupRules {
    // Group index is defined by unsched_vehgroups.txt.
    std::size_t groupIndex = 0;
    std::optional<double> speedLimitKmh;
    std::optional<double> density;
    std::optional<double> priority;
    std::optional<bool> carsProhibited;
    std::optional<bool> unscheduledTrafficProhibited;
    std::optional<bool> busesAllowed;
    std::optional<bool> trucksAllowed;
    std::optional<bool> overtakingProhibited;
};

struct MapTrafficPathRules {
    std::optional<double> speedLimitKmh;
    std::optional<double> density;
    std::vector<MapTrafficGroupRules> groups;
    bool carsProhibited = false;
    bool busesAllowed = false;
    bool trucksAllowed = true;
    bool overtakingProhibited = false;
    bool roundabout = false;
    // Higher authored traffic-priority values have right-of-way.
    double priority = 128.0;
};

struct MapTrafficVehicleFilter {
    int vehicleType = 0;
    std::optional<std::size_t> unscheduledGroupIndex;
    bool scheduled = false;
    std::optional<std::size_t> trafficRuleGroupIndex;
};

struct MapTrafficRouteRequirement {
    int splineId = 0;
    std::optional<std::size_t> sceneryPathIndex;
};

struct MapTrafficRoutePrefixMatch {
    std::vector<MapTrafficRouteRequirement> routePrefix;
    std::size_t timetableStartIndex = 0;
    std::size_t timetableEntriesMatched = 0;
};

struct MapTrafficRouteSegment {
    MapSplinePlacementReference spline;
    bool reversed = false;
    double pathLateralMeters = 0.0;
    double pathHeightMeters = 0.0;
    MapTrafficPathRules pathRules;
    std::optional<MapTrafficSignalSchedule> trafficSignal;
    std::optional<std::size_t> sceneryPathIndex;
    // Scenery junction paths are sampled in world space, like the F2 overlay.
    std::vector<std::array<double, 3>> points;
    // Route-local start position on the original spline for safely subdivided segments.
    std::optional<double> splineDistanceAtRouteStartMeters;
};

MapTrafficRouteSegment reverseMapTrafficRouteSegment(MapTrafficRouteSegment segment,
                                                     double segmentLengthMeters);

struct MapTrafficRoute {
    std::vector<MapTrafficRouteSegment> segments;
    std::vector<double> cumulativeEndDistances;
    double totalLength = 0.0;
    bool closed = false;
    std::optional<double> loopStartDistance;
    bool startsAtEntryLane = false;
};

struct MapTrafficDrivingRouteMetadata {
    // Reuse only while the route is unchanged; sampling also depends on each actor's offset.
    std::size_t routeHash = 0;
    bool samplingCannotFail = false;
};

struct MapTrafficRoutePosition {
    std::size_t segmentIndex = 0;
    double distanceMeters = 0.0;
};

struct MapTrafficSpawnCandidate {
    double distanceMeters = 0.0;
    int tileX = 0;
    int tileY = 0;
    double locationPenaltyMeters = 0.0;
    double entryPreferenceMeters = 0.0;
};

// Last constraint that held a driving actor below cruise speed; used for stall diagnostics.
enum class MapTrafficBlockReason {
    None,
    Leader,
    Vehicle,
    Obstacle,
    CrossingReservation,
    JunctionTraffic,
    Signal,
    Collision,
};

const char* mapTrafficBlockReasonName(MapTrafficBlockReason reason) noexcept;

struct MapTrafficActor {
    std::string modelPath;
    std::optional<std::string> driverModelPath;
    MapTrafficRoute route;
    double distanceMeters = 0.0;
    double speedMetersPerSecond = 0.0;
    double targetSpeedMetersPerSecond = 0.0;
    double lateralOffsetMeters = 0.0;
    double cruiseSpeedMetersPerSecond = 0.0;
    double lengthMeters = 4.5;
    double widthMeters = 1.8;
    // Signed longitudinal offset from the model origin to the rear axle.
    // Zero retains the route-centered pose for actors without vehicle axle data.
    double rearAxleOffsetMeters = 0.0;
    double minimumTurnRadiusMeters = 4.5;
    double junctionWaitSeconds = 0.0;
    std::optional<double> scheduledStopDistanceMeters;
    int vehicleType = 0;
    std::optional<std::size_t> unscheduledGroupIndex;
    std::optional<std::size_t> trafficRuleGroupIndex;
    bool scheduled = false;
    bool isBus = false;
    MapTrafficBlockReason blockReason = MapTrafficBlockReason::None;
    // What blockReason refers to: another driving actor, or an index into the obstacle list
    // passed to the last advanceMapTrafficDriving call. Identity only; never dereferenced here.
    const MapTrafficActor* blockingActor = nullptr;
    std::optional<std::size_t> blockingObstacleIndex;
    // End of the rolling local junction reservation (also used by the F2 overlay).
    // This does not imply ownership of the entire remaining authored spline.
    std::optional<double> junctionClaimEndMeters;
    // Keep a too-late-to-brake signal decision until this signal entry is passed.
    std::optional<std::size_t> committedSignalSegmentIndex;
    // Distant actors retain full interaction checks but use sparser curve-speed sampling.
    bool reducedSpeedLookahead = false;
};

double mapTrafficSpawnSpeedMetersPerSecond(const MapTrafficActor& actor, double distanceMeters);

struct MapTrafficPose {
    std::array<double, 3> position = {};
    double headingRadians = 0.0;
    int splineId = 0;
    double pitchRadians = 0.0;
    double rollRadians = 0.0;
};

struct MapPassengerStopAnchor {
    int stopId = 0;
    std::array<double, 3> position = {};
};

struct MapPassengerWaitingArea {
    int objectId = 0;
    std::array<double, 3> position = {};
    std::vector<MapTrafficPose> positions;
    std::array<std::array<double, 3>, 8> bounds = {};
};

MapTrafficPose mapTrafficSceneryPassengerPose(const MapSceneryPose& sceneryPose,
                                              const std::array<double, 3>& localPosition,
                                              double seatHeight, double rotationDegrees);

// Assign each waiting area to its nearest stop, with a bounded distance and height difference.
std::map<int, std::vector<MapTrafficPose>>
mapTrafficAssignPassengerPositions(const std::vector<MapPassengerStopAnchor>& stops,
                                   const std::vector<MapPassengerWaitingArea>& areas,
                                   const std::map<int, std::size_t>& requestedCounts = {});

enum class MapTrafficTurnSignal {
    None,
    Left,
    Right,
};

struct MapTrafficCameraView {
    std::array<double, 3> position = {};
    std::array<double, 16> viewProjection = {};
};

struct MapTrafficSignalStopDebugInfo {
    // Center of the physical stopline, not the vehicle-center stopping position.
    MapTrafficPose pose;
    // Distance the vehicle center must travel to reach its safe stopping position.
    double distanceAheadMeters = 0.0;
    bool blocked = false;
};

struct MapTrafficSplineEndpointStatus {
    bool startConnected = false;
    bool endConnected = false;
};

bool mapTrafficPoseOnActiveTile(const MapTrafficPose& pose,
                                const std::vector<std::pair<int, int>>& activeTiles,
                                double tileSize = OMSI_TILE_SIZE_METERS);
bool mapTrafficSpawnInsideForwardExclusion(const MapTrafficPose& playerPose,
                                           double playerLengthMeters, double playerWidthMeters,
                                           const MapTrafficPose& spawnPose, double spawnWidthMeters,
                                           double forwardDistanceMeters) noexcept;
double mapTrafficSpawnPriorityDistanceMeters(const MapTrafficPose& playerPose,
                                             const MapTrafficPose& spawnPose,
                                             double predictedForwardDistanceMeters) noexcept;

// Returns a signed lateral offset relative to travel direction: positive is right, negative
// is left. The supplied distance is a non-negative magnitude.
double mapTrafficLaneOffsetMeters(bool leftHandTraffic, double distanceMeters);
double mapTrafficPassengerSideSign(bool leftHandTraffic) noexcept;
double mapTrafficPassengerCurbOffsetMeters(bool leftHandTraffic, double laneOffsetMeters,
                                           double vehicleWidthMeters, double clearanceMeters);
double mapTrafficRouteTravelHeadingRadians(const MapTrafficRoute& route, double distanceMeters,
                                           double lateralOffsetMeters);
double mapTrafficHeadingToVehicleYawDegrees(double headingRadians);
double mapTrafficHourAtElapsedSeconds(double startHour, double elapsedSeconds);
int mapTrafficWeekdayMaskAtElapsedSeconds(int startWeekday, double startHour,
                                          double elapsedSeconds);
double mapTrafficNextDepartureElapsedSeconds(double startHour, double elapsedSeconds,
                                             double departureMinutes);
std::size_t mapTrafficNextDepartureIndex(double startHour, double elapsedSeconds,
                                         const std::vector<double>& departureMinutes);
bool mapTrafficScheduledDepartureDue(double elapsedSeconds, double departureElapsedSeconds);
MapTrafficPathRules mapTrafficRulesForPath(const std::vector<MapTileRule>& rules,
                                           std::size_t pathIndex);
bool mapTrafficPathAllowsVehicle(const MapTrafficPathRules& rules,
                                 const MapTrafficVehicleFilter& vehicle);
double mapTrafficPathDensityForVehicle(const MapTrafficPathRules& rules,
                                       const MapTrafficVehicleFilter& vehicle) noexcept;
std::optional<double>
mapTrafficPathSpeedLimitForVehicle(const MapTrafficPathRules& rules,
                                   const MapTrafficVehicleFilter& vehicle) noexcept;
double mapTrafficPathPriorityForVehicle(const MapTrafficPathRules& rules,
                                        const MapTrafficVehicleFilter& vehicle) noexcept;
bool mapTrafficRoutePositionOnJunctionPath(const MapTrafficRoute& route, double distanceMeters);
bool mapTrafficActorOnJunctionPath(const MapTrafficActor& actor, double distanceMeters);
std::vector<std::vector<double>>
mapTrafficPartitionScheduledDepartures(const std::vector<double>& departureMinutes,
                                       std::size_t partitionCount);
std::vector<std::size_t>
mapTrafficAllocateScheduledBuses(const std::vector<std::size_t>& departureCounts,
                                 std::size_t busPoolLimit);

// Scheduled passengers are scaled independently from the ambient pedestrian cap.
std::size_t mapScheduledPassengerQueueSize(std::size_t nominalCount, double humanScale,
                                           std::size_t safetyLimit = 100000) noexcept;
std::size_t mapScheduledPassengerQueueAfterDeparture(std::size_t waitingPassengers,
                                                     std::size_t initialPassengers) noexcept;
std::size_t mapScheduledPassengerModelIdentity(int stopId, std::size_t queuePosition);
std::size_t mapScheduledPassengerQueueSlot(std::size_t initialCount, std::size_t passengerIndex);
std::array<double, 2> mapTrafficPassengerWorldOffset(bool leftHandTraffic, double headingRadians,
                                                     double lateralOffsetMeters,
                                                     double longitudinalOffsetMeters);
MapTrafficPose mapTrafficPassengerQueuePose(const MapTrafficPose& stopPose, bool leftHandTraffic,
                                            std::size_t queueSlot);
std::vector<std::size_t> mapPassengerServiceUpdateOrder(const std::vector<int>& serviceStopIds,
                                                        std::size_t rotation);

struct MapPassengerStopService {
    int stopId = 0;
    std::size_t waitingPassengers = 0;
    std::size_t passengersAlighting = 0;
    double dwellSeconds = 0.0;
    int nextStopId = 0;
};

struct MapTrafficPassengerStop {
    int stopId = 0;
    int splineId = 0;
    std::size_t pathIndex = 0;
    std::size_t trackEntryIndex = 0;
    std::size_t waitingPassengers = 0;
    std::size_t passengersAlighting = 0;
    double dwellSeconds = 0.0;
    double routeDistanceMeters = 0.0;
    std::optional<double> splinePositionMeters;
    std::string name;
    std::size_t timetableStationIndex = 0;
};

class MapPassengerServiceRuntime {
  public:
    explicit MapPassengerServiceRuntime(std::size_t seatCount = 0);

    void setWaitingPassengers(int stopId, std::size_t count);
    bool arrive(const MapPassengerStopService& stop);
    // Reserves a waiting passenger at the start of a boarding interval and seats them at
    // completion.
    void advance(double elapsedSeconds, bool allowBoarding = true, bool allowAlighting = true);
    bool dwelling() const noexcept;
    int currentStopId() const noexcept;
    bool boardingPassenger() const noexcept;
    double boardingProgress() const noexcept;
    std::optional<std::size_t> boardingSeatIndex() const noexcept;
    bool alightingPassenger() const noexcept;
    double alightingProgress() const noexcept;
    std::optional<std::size_t> alightingSeatIndex() const noexcept;
    std::optional<std::size_t> boardingPassengerModelIndex() const noexcept;
    std::uint64_t passengerStopRequestSequence() const noexcept;
    std::size_t waitingPassengers(int stopId) const noexcept;
    void setPassengerPathLinkProgressProfile(
        std::vector<std::pair<double, std::optional<std::size_t>>> profile);
    std::size_t passengerCountOnPathLink(std::size_t linkIndex) const noexcept;
    std::size_t passengerCountOnSeat(std::size_t seatIndex) const noexcept;
    std::optional<std::size_t> passengerModelIndexOnSeat(std::size_t seatIndex) const noexcept;
    std::size_t onboardPassengerCount() const noexcept;
    std::size_t seatCount() const noexcept;

  private:
    std::vector<std::optional<int>> seatDestinations_;
    std::vector<std::optional<std::size_t>> seatPassengerModelIndices_;
    std::unordered_map<int, std::size_t> waitingByStop_;
    int currentStopId_ = 0;
    int nextStopId_ = 0;
    double dwellRemainingSeconds_ = 0.0;
    double boardingAccumulatorSeconds_ = 0.0;
    std::optional<std::size_t> boardingSeat_;
    std::optional<std::size_t> boardingPassengerModelIndex_;
    std::vector<std::size_t> queuedAlightingSeats_;
    std::optional<std::size_t> alightingSeat_;
    double alightingAccumulatorSeconds_ = 0.0;
    std::optional<std::size_t> passengerPathLinkIndex_;
    std::vector<std::pair<double, std::optional<std::size_t>>> passengerPathLinkProgressProfile_;
    std::uint64_t passengerStopRequestSequence_ = 0;
    bool dwelling_ = false;

    void clearPassengerPathLinkProgressProfile() noexcept;
    void updatePassengerPathLinkIndex() noexcept;
};

void advanceMapPassengerServiceWithSharedQueue(MapPassengerServiceRuntime& service, int stopId,
                                               std::size_t& sharedWaitingPassengers,
                                               double elapsedSeconds, bool allowBoarding = true,
                                               bool allowAlighting = true);

enum class MapTrafficPassengerStopDepartureAction {
    None,
    RequestDeparture,
    ResumeRoute,
    RetireRoute
};

MapTrafficPassengerStopDepartureAction
mapTrafficPassengerStopDepartureAction(bool stopArrived, bool passengerServiceDwelling,
                                       bool departureRequested,
                                       double aiScheduledAtStation,
                                       bool retireAfterStop = false) noexcept;
bool mapTrafficShouldCancelScheduledRouteForPlayerTrip(std::string_view playerTimetableKey,
                                                       std::string_view scheduledTimetableKey,
                                                       bool driveAsAdditionalBus) noexcept;

MapTrafficActor makeMapTrafficActor(MapTrafficRoute route, std::string modelPath,
                                    double speedMetersPerSecond, double laneOffsetMagnitudeMeters,
                                    bool leftHandTraffic,
                                    std::optional<std::string> driverModelPath = std::nullopt);
void resetMapTrafficActorForEntry(MapTrafficActor& actor) noexcept;

// True when either complete ordered spline sequence occurs contiguously in the other.
bool mapTrafficRoutesShareCompleteOrderedPath(const std::vector<int>& firstSplineIds,
                                              const std::vector<int>& secondSplineIds);

// Returns the longest two-or-more-spline timetable match beginning at the route endpoint.
std::vector<int> mapTrafficRoutePrefixMatchingTimetable(const std::vector<int>& timetableSplineIds,
                                                        const std::vector<int>& routeSplineIds);
// Ordinary spline checkpoints can repeat within one lane; placed scenery paths retain selectors.
bool mapTrafficTimetablePathContainedInRoute(
    const std::vector<MapTrafficRouteRequirement>& timetablePath,
    const std::vector<MapTrafficRouteRequirement>& route);
bool mapTrafficTimetablePathContainedInRouteInOrder(
    const std::vector<MapTrafficRouteRequirement>& timetablePath,
    const std::vector<MapTrafficRouteRequirement>& route);
MapTrafficRoutePrefixMatch
mapTrafficRoutePrefixMatch(const std::vector<MapTrafficRouteRequirement>& timetablePath,
                           const std::vector<MapTrafficRouteRequirement>& route);
std::vector<MapTrafficRouteRequirement>
mapTrafficRoutePrefixMatchingTimetable(const std::vector<MapTrafficRouteRequirement>& timetablePath,
                                       const std::vector<MapTrafficRouteRequirement>& route);

// Places an actor at a route distance if the sampled pose is within the spawn radius.
// Open routes stop at totalLength - endpointMarginMeters; closed routes wrap.
bool placeMapTrafficActorNear(MapTrafficActor& actor, double startDistanceMeters, double centerX,
                              double centerY, double maximumDistanceMeters,
                              double endpointMarginMeters = 0.0);

// Places looping actors at a route distance, wrapping open routes before their endpoint margin.
// Returns false if an open route has no usable distance after applying that margin.
bool placeMapTrafficActorNearLooping(MapTrafficActor& actor, double startDistanceMeters,
                                     double centerX, double centerY, double maximumDistanceMeters,
                                     double endpointMarginMeters = 0.0);

// Resolves a route by following reciprocal next/previous spline links from a valid starting
// placement. Missing links and ambiguous IDs stop the route; a reciprocal loop closes it.
std::optional<MapTrafficRoute> buildMapTrafficRoute(const MapSplinePlacementReference& start,
                                                    const MapSplineChainResolver& chainResolver,
                                                    std::size_t segmentLimit = 1024);

// Splits eligible straight road segments while keeping a 5 m buffer at each segment join.
MapTrafficRoute subdivideMapTrafficRoute(const MapTrafficRoute& route);

std::optional<MapTrafficPose> sampleMapTrafficRoute(const MapTrafficRoute& route,
                                                    double distanceMeters,
                                                    double lateralOffsetMeters);
// The rear axle follows the route; the returned pose is the vehicle model origin.
std::optional<MapTrafficPose> sampleMapTrafficActorPose(const MapTrafficActor& actor,
                                                        double distanceMeters);
std::optional<MapTrafficPose> sampleMapTrafficActorPose(const MapTrafficRoute& route,
                                                        double distanceMeters,
                                                        double lateralOffsetMeters,
                                                        double rearAxleOffsetMeters);
std::optional<MapTrafficRoutePosition>
mapTrafficRoutePositionOnSpline(const MapTrafficRoute& route, int splineId,
                                std::size_t sceneryPathIndex, double splinePositionMeters,
                                std::size_t firstSegmentIndex = 0);
MapTrafficTurnSignal mapTrafficActorTurnSignal(const MapTrafficActor& actor,
                                               double activationDistanceMeters = 35.0);
std::vector<MapTrafficSpawnCandidate> mapTrafficRouteSpawnCandidates(const MapTrafficRoute& route,
                                                                     double vehicleLengthMeters);
bool mapTrafficSpawnCandidatesOverlapActiveTiles(
    const std::vector<MapTrafficSpawnCandidate>& candidates,
    const std::vector<std::pair<int, int>>& activeTiles);
bool mapTrafficRouteSegmentClearsWater(const MapTrafficRouteSegment& segment,
                                       const WaterData& water, double tileSize,
                                       double clearanceMeters = 0.25);

// Advances one actor. Returns false when it reaches the end of an open route.
bool advanceMapTrafficActor(MapTrafficActor& actor, double elapsedSeconds);

// Advances an actor and recycles it to the start when it reaches the end of an open route.
bool advanceMapTrafficActorLooping(MapTrafficActor& actor, double elapsedSeconds);

// True when the actor is stopped at a nearby non-permissive traffic signal.
bool mapTrafficActorWaitingAtSignal(const MapTrafficActor& actor, double signalClockSeconds);
std::optional<MapTrafficSignalStopDebugInfo>
mapTrafficNextSignalStopDebugInfo(const MapTrafficActor& actor, double signalClockSeconds);

// Advances a population and removes actors that reach the end of open routes.
// The population is unchanged if any actor has invalid runtime state.
std::size_t advanceMapTrafficActors(std::vector<MapTrafficActor>& actors, double elapsedSeconds);

struct MapTrafficObstacle {
    MapTrafficPose pose;
    double lengthMeters = 4.5;
    double widthMeters = 1.8;
};

// True when a route actor is aligned behind an obstacle within the supplied bumper gap.
bool mapTrafficActorBehindObstacle(const MapTrafficActor& actor, const MapTrafficObstacle& obstacle,
                                   double maximumGapMeters);

// OBB clearance is shared by entry admission and swept-path movement checks.
bool mapTrafficHasClearance(const MapTrafficObstacle& vehicle,
                            const std::vector<MapTrafficObstacle>& obstacles,
                            double clearanceMeters = 0.5);
// Advances in small collision-checked steps, looping open routes; returns false if blocked.
bool advanceMapTrafficActorLoopingAvoidingObstacles(
    MapTrafficActor& actor, const std::vector<MapTrafficObstacle>& obstacles, double lengthMeters,
    double widthMeters, double elapsedSeconds, double signalClockSeconds = 0.0);
bool canSpawnMapTrafficEntry(const MapTrafficActor& actor, bool entryHidden,
                             const std::vector<MapTrafficObstacle>& obstacles);
bool mapTrafficRouteEndReadyForUnload(const MapTrafficActor& actor);
// True when a bounding sphere is farther than minimumDistanceMeters (horizontally) from the
// camera and lies entirely outside the clip volume of the column-major projection * view.
bool mapTrafficEntryHiddenFromCamera(const std::array<double, 3>& position, double radiusMeters,
                                     const std::array<double, 3>& cameraPosition,
                                     const std::array<double, 16>& viewProjection,
                                     double minimumDistanceMeters);
bool mapTrafficEntryHiddenFromCameras(const std::array<double, 3>& position, double radiusMeters,
                                      const MapTrafficCameraView& mainCamera,
                                      const std::vector<MapTrafficCameraView>& additionalCameras,
                                      double minimumDistanceMeters);
double mapTrafficCurveSpeedLimit(const MapTrafficRoute& route, double distanceMeters,
                                 double cruiseSpeedMetersPerSecond, double minimumTurnRadiusMeters);
MapTrafficDrivingRouteMetadata analyzeMapTrafficDrivingRoute(const MapTrafficRoute& route);
// Uses small synchronous steps and swept footprints; never moves external obstacles.
void advanceMapTrafficDriving(const std::vector<MapTrafficActor*>& actors,
                              const std::vector<MapTrafficObstacle>& obstacles,
                              double elapsedSeconds, double signalClockSeconds = 0.0,
                              std::span<const MapTrafficDrivingRouteMetadata> routeMetadata = {});
void advanceMapTrafficSignalControllers(
    const std::vector<std::shared_ptr<MapTrafficSignalController>>& controllers,
    const std::vector<MapTrafficActor*>& actors, double elapsedSeconds, double initialClockSeconds,
    const std::vector<MapTrafficSignalBusPriorityRequest>& busPriorityRequests = {});

class MapTrafficRouteNetwork {
  public:
    void addTileSplines(const std::vector<MapSplinePlacement>& splines, int tileX, int tileY,
                        const MapCoordinates& coordinates = {});
    void finalize();
    std::optional<MapTrafficRoute> buildRoute(int splineId, std::size_t segmentLimit = 1024) const;
    std::vector<int> splineIds() const;
    std::vector<int> splineIdsInTiles(const std::vector<std::pair<int, int>>& tiles) const;
    std::vector<int> routeStartSplineIds() const;
    std::optional<MapTrafficSplineEndpointStatus> endpointStatus(int splineId) const;

  private:
    std::unordered_map<int, MapSplinePlacementReference> splinesById_;
    std::unordered_set<int> ambiguousSplineIds_;
    bool finalized_ = false;
};

struct MapTrafficLaneNetworkStats {
    std::size_t lanes = 0;
    std::size_t deadEnds = 0;
    // Dead ends within a few meters of an aligned lane start that did not join.
    std::size_t nearMissDeadEnds = 0;
};

struct MapTrafficLaneStart {
    int splineId = 0;
    std::size_t laneSelector = 0;
    bool operator==(const MapTrafficLaneStart&) const = default;
};

struct MapTrafficLaneConnectionGap {
    double horizontalMeters = 0.0;
    double verticalMeters = 0.0;
    double headingAlignment = 1.0;
};

// Directed authored lanes, joined by coincident endpoints (including scenery paths).
class MapTrafficLaneNetwork {
  public:
    void addSegment(MapTrafficRouteSegment segment);
    void finalize();
    MapTrafficLaneNetworkStats stats() const;
    // laneSelector chooses among the spline's authored lanes (modulo its lane count).
    std::optional<MapTrafficRoute>
    buildRoute(int splineId, std::size_t laneSelector = 0, std::size_t routeSeed = 0,
               double minimumTurnRadiusMeters = 0.0, MapTrafficVehicleFilter vehicle = {},
               const std::vector<int>& requiredSplineSequence = {},
               const std::vector<std::optional<std::size_t>>& requiredSceneryPathSequence = {},
               bool stopAtRequiredSequenceEnd = false,
               bool allowIntermediarySegmentsBetweenRequired = false,
               bool allowReverseTraversalForRequired = false) const;
    std::size_t laneCount(int splineId) const;
    std::vector<MapTrafficLaneStart>
    lanesForTimetableEntry(int splineId, std::optional<std::size_t> sceneryPathIndex) const;
    std::optional<MapTrafficLaneConnectionGap>
    nearestConnectionGap(int fromSplineId, std::optional<std::size_t> fromPathIndex, int toSplineId,
                         std::optional<std::size_t> toPathIndex) const;
    std::vector<MapTrafficLaneStart> entryLanes() const;
    std::optional<MapTrafficLaneStart> selectEntryLane(MapTrafficVehicleFilter vehicle,
                                                       std::size_t selectionSeed) const;
    // Connected road lanes with a forward continuation, including internal lanes.
    std::vector<MapTrafficLaneStart> spawnLanes() const;
    std::optional<MapTrafficLaneStart> selectSpawnLane(MapTrafficVehicleFilter vehicle,
                                                       std::size_t selectionSeed) const;
    std::optional<MapTrafficLaneStart>
    selectSpawnLaneInTiles(MapTrafficVehicleFilter vehicle, std::size_t selectionSeed,
                           const std::vector<std::pair<int, int>>& activeTiles) const;

  private:
    using Cell = std::pair<long long, long long>;

    std::optional<MapTrafficLaneStart>
    selectLane(MapTrafficVehicleFilter vehicle, std::size_t selectionSeed,
               const std::vector<MapTrafficLaneStart>& candidates) const;

    std::vector<MapTrafficRouteSegment> segments_;
    std::vector<double> lengths_;
    std::vector<std::vector<std::size_t>> next_;
    std::vector<std::vector<std::size_t>> timetableNext_;
    std::unordered_map<int, std::vector<std::size_t>> lanesBySpline_;
    std::vector<MapTrafficPose> laneStarts_;
    std::vector<MapTrafficPose> laneEnds_;
    std::vector<MapTrafficLaneStart> spawnLanes_;
    std::vector<std::size_t> incoming_;
    std::vector<double> maximumCurvatures_;
    MapTrafficLaneNetworkStats stats_;
    std::map<Cell, std::vector<std::size_t>> laneStartCells_;
    std::map<Cell, std::vector<std::size_t>> laneEndCells_;
    std::map<Cell, std::vector<std::size_t>> timetableStartCells_;
    std::vector<bool> nearMissByLane_;
    std::size_t finalizedSegmentCount_ = 0;
    bool finalized_ = false;

    void finalizeAppendedSegments();
};

} // namespace openbus::map
