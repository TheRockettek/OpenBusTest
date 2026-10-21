#pragma once

#include "map/MapSceneryPlacement.h"

#include <array>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace openbus::map {

struct MapTrafficRouteSegment {
    MapSplinePlacementReference spline;
    bool reversed = false;
    double pathLateralMeters = 0.0;
    double pathHeightMeters = 0.0;
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

struct MapTrafficActor {
    std::string modelPath;
    std::optional<std::string> driverModelPath;
    MapTrafficRoute route;
    double distanceMeters = 0.0;
    double speedMetersPerSecond = 0.0;
    double lateralOffsetMeters = 0.0;
};

struct MapTrafficPose {
    std::array<double, 3> position = {};
    double headingRadians = 0.0;
    int splineId = 0;
};

struct MapTrafficSplineEndpointStatus {
    bool startConnected = false;
    bool endConnected = false;
};

// Returns a signed lateral offset relative to travel direction: positive is right, negative
// is left. The supplied distance is a non-negative magnitude.
double mapTrafficLaneOffsetMeters(bool leftHandTraffic, double distanceMeters);
double mapTrafficHeadingToVehicleYawDegrees(double headingRadians);
double mapTrafficHourAtElapsedSeconds(double startHour, double elapsedSeconds);

MapTrafficActor makeMapTrafficActor(MapTrafficRoute route, std::string modelPath,
                                    double speedMetersPerSecond, double laneOffsetMagnitudeMeters,
                                    bool leftHandTraffic,
                                    std::optional<std::string> driverModelPath = std::nullopt);

// Places an actor at a route distance if the sampled pose is within the spawn radius.
// Open routes stop at totalLength - endpointMarginMeters; closed routes wrap.
bool placeMapTrafficActorNear(MapTrafficActor& actor, double startDistanceMeters, double centerX,
                              double centerY, double maximumDistanceMeters,
                              double endpointMarginMeters = 0.0);

// Places looping actors at a route distance, wrapping open routes before their endpoint margin.
// Returns false if an open route has no usable distance after applying that margin.
bool placeMapTrafficActorNearLooping(MapTrafficActor& actor, double startDistanceMeters,
                                     double centerX, double centerY,
                                     double maximumDistanceMeters,
                                     double endpointMarginMeters = 0.0);

// Resolves a route by following reciprocal next/previous spline links from a valid starting
// placement. Missing links and ambiguous IDs stop the route; a reciprocal loop closes it.
std::optional<MapTrafficRoute> buildMapTrafficRoute(const MapSplinePlacementReference& start,
                                                    const MapSplineChainResolver& chainResolver,
                                                    std::size_t segmentLimit = 1024);

std::optional<MapTrafficPose> sampleMapTrafficRoute(const MapTrafficRoute& route,
                                                    double distanceMeters,
                                                    double lateralOffsetMeters);

// Advances one actor. Returns false when it reaches the end of an open route.
bool advanceMapTrafficActor(MapTrafficActor& actor, double elapsedSeconds);

// Advances an actor and recycles it to the start when it reaches the end of an open route.
bool advanceMapTrafficActorLooping(MapTrafficActor& actor, double elapsedSeconds);

// Advances a population and removes actors that reach the end of open routes.
// The population is unchanged if any actor has invalid runtime state.
std::size_t advanceMapTrafficActors(std::vector<MapTrafficActor>& actors, double elapsedSeconds);

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

// Directed authored lanes, joined by coincident endpoints (including scenery paths).
class MapTrafficLaneNetwork {
  public:
    void addSegment(MapTrafficRouteSegment segment);
    void finalize();
    MapTrafficLaneNetworkStats stats() const;
    // laneSelector chooses among the spline's authored lanes (modulo its lane count).
    std::optional<MapTrafficRoute> buildRoute(int splineId, std::size_t laneSelector = 0) const;
    std::size_t laneCount(int splineId) const;

  private:
    std::vector<MapTrafficRouteSegment> segments_;
    std::vector<double> lengths_;
    std::vector<std::optional<std::size_t>> next_;
    std::unordered_map<int, std::vector<std::size_t>> lanesBySpline_;
    MapTrafficLaneNetworkStats stats_;
    bool finalized_ = false;
};

} // namespace openbus::map
