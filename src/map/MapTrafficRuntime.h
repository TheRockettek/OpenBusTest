#pragma once

#include "map/MapSceneryPlacement.h"
#include "map/MapTrafficSignal.h"

#include <array>
#include <cstddef>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace openbus::map {

struct MapTrafficGroupRules {
    std::size_t groupIndex = 0;
    std::optional<double> speedLimitKmh;
    std::optional<double> density;
    std::optional<double> priority;
    std::optional<bool> carsProhibited;
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
    bool trucksAllowed = false;
    bool overtakingProhibited = false;
    // OMSI traffic priority: lower values have right-of-way.
    double priority = 128.0;
};

struct MapTrafficVehicleFilter {
    int vehicleType = 0;
    std::optional<std::size_t> unscheduledGroupIndex;
    bool scheduled = false;
};

struct MapTrafficRouteSegment {
    MapSplinePlacementReference spline;
    bool reversed = false;
    double pathLateralMeters = 0.0;
    double pathHeightMeters = 0.0;
    MapTrafficPathRules pathRules;
    std::optional<MapTrafficSignalSchedule> trafficSignal;
    // Scenery junction paths are sampled in world space, like the F2 overlay.
    std::vector<std::array<double, 3>> points;
};

struct MapTrafficRoute {
    std::vector<MapTrafficRouteSegment> segments;
    std::vector<double> cumulativeEndDistances;
    double totalLength = 0.0;
    bool closed = false;
    std::optional<double> loopStartDistance;
};

struct MapTrafficSpawnCandidate {
    double distanceMeters = 0.0;
    int tileX = 0;
    int tileY = 0;
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
    double minimumTurnRadiusMeters = 4.5;
    double junctionWaitSeconds = 0.0;
    int vehicleType = 0;
    std::optional<std::size_t> unscheduledGroupIndex;
    bool scheduled = false;
    MapTrafficBlockReason blockReason = MapTrafficBlockReason::None;
    // What blockReason refers to: another driving actor, or an index into the obstacle list
    // passed to the last advanceMapTrafficDriving call. Identity only; never dereferenced here.
    const MapTrafficActor* blockingActor = nullptr;
    std::optional<std::size_t> blockingObstacleIndex;
    // Route distance up to which this actor holds an admitted claim on its upcoming junction.
    std::optional<double> junctionClaimEndMeters;
};

struct MapTrafficPose {
    std::array<double, 3> position = {};
    double headingRadians = 0.0;
    int splineId = 0;
};

struct MapTrafficSignalStopDebugInfo {
    MapTrafficPose pose;
    double distanceAheadMeters = 0.0;
    bool blocked = false;
};

struct MapTrafficSplineEndpointStatus {
    bool startConnected = false;
    bool endConnected = false;
};

bool mapTrafficPoseOnActiveTile(const MapTrafficPose& pose,
                                const std::vector<std::pair<int, int>>& activeTiles);

// Returns a signed lateral offset relative to travel direction: positive is right, negative
// is left. The supplied distance is a non-negative magnitude.
double mapTrafficLaneOffsetMeters(bool leftHandTraffic, double distanceMeters);
double mapTrafficHeadingToVehicleYawDegrees(double headingRadians);
double mapTrafficHourAtElapsedSeconds(double startHour, double elapsedSeconds);
double mapTrafficNextDepartureElapsedSeconds(double startHour, double elapsedSeconds,
                                             double departureMinutes);
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

MapTrafficActor makeMapTrafficActor(MapTrafficRoute route, std::string modelPath,
                                    double speedMetersPerSecond, double laneOffsetMagnitudeMeters,
                                    bool leftHandTraffic,
                                    std::optional<std::string> driverModelPath = std::nullopt);
void resetMapTrafficActorForEntry(MapTrafficActor& actor) noexcept;

// True when either complete ordered spline sequence occurs contiguously in the other.
bool mapTrafficRoutesShareCompleteOrderedPath(const std::vector<int>& firstSplineIds,
                                              const std::vector<int>& secondSplineIds);

// Returns the longest two-or-more-spline timetable match beginning at the route endpoint.
std::vector<int> mapTrafficRoutePrefixMatchingTimetable(
    const std::vector<int>& timetableSplineIds, const std::vector<int>& routeSplineIds);

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

std::optional<MapTrafficPose> sampleMapTrafficRoute(const MapTrafficRoute& route,
                                                    double distanceMeters,
                                                    double lateralOffsetMeters);
std::vector<MapTrafficSpawnCandidate> mapTrafficRouteSpawnCandidates(const MapTrafficRoute& route,
                                                                     double vehicleLengthMeters);

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
bool mapTrafficActorBehindObstacle(const MapTrafficActor& actor,
                                   const MapTrafficObstacle& obstacle,
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
double mapTrafficCurveSpeedLimit(const MapTrafficRoute& route, double distanceMeters,
                                 double cruiseSpeedMetersPerSecond, double minimumTurnRadiusMeters);
// Uses small synchronous steps and swept footprints; never moves external obstacles.
void advanceMapTrafficDriving(const std::vector<MapTrafficActor*>& actors,
                              const std::vector<MapTrafficObstacle>& obstacles,
                              double elapsedSeconds, double signalClockSeconds = 0.0);
void advanceMapTrafficSignalControllers(
    const std::vector<std::shared_ptr<MapTrafficSignalController>>& controllers,
    const std::vector<MapTrafficActor*>& actors, double elapsedSeconds, double initialClockSeconds);

class MapTrafficRouteNetwork {
  public:
    void addTileSplines(const std::vector<MapSplinePlacement>& splines, int tileX, int tileY);
    void finalize();
    std::optional<MapTrafficRoute> buildRoute(int splineId, std::size_t segmentLimit = 1024) const;
    std::vector<int> splineIds() const;
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
               const std::vector<int>& requiredSplineSequence = {}) const;
    std::size_t laneCount(int splineId) const;
    std::vector<MapTrafficLaneStart> entryLanes() const;
    std::optional<MapTrafficLaneStart> selectEntryLane(MapTrafficVehicleFilter vehicle,
                                                       std::size_t selectionSeed) const;
    // Connected road lanes with a forward continuation, including internal lanes.
    std::vector<MapTrafficLaneStart> spawnLanes() const;
    std::optional<MapTrafficLaneStart> selectSpawnLane(MapTrafficVehicleFilter vehicle,
                                                       std::size_t selectionSeed) const;

  private:
    std::optional<MapTrafficLaneStart>
    selectLane(MapTrafficVehicleFilter vehicle, std::size_t selectionSeed,
               const std::vector<MapTrafficLaneStart>& candidates) const;

    std::vector<MapTrafficRouteSegment> segments_;
    std::vector<double> lengths_;
    std::vector<std::vector<std::size_t>> next_;
    std::unordered_map<int, std::vector<std::size_t>> lanesBySpline_;
    std::vector<MapTrafficLaneStart> spawnLanes_;
    std::vector<std::size_t> incoming_;
    std::vector<double> maximumCurvatures_;
    MapTrafficLaneNetworkStats stats_;
    bool finalized_ = false;
};

} // namespace openbus::map
