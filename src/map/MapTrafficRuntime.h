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
};

struct MapTrafficRoute {
    std::vector<MapTrafficRouteSegment> segments;
    std::vector<double> cumulativeEndDistances;
    double totalLength = 0.0;
    bool closed = false;
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

// Returns a signed lateral offset relative to travel direction: positive is right, negative
// is left. The supplied distance is a non-negative magnitude.
double mapTrafficLaneOffsetMeters(bool leftHandTraffic, double distanceMeters);
double mapTrafficHeadingToVehicleYawDegrees(double headingRadians);

MapTrafficActor makeMapTrafficActor(MapTrafficRoute route, std::string modelPath,
                                    double speedMetersPerSecond, double laneOffsetMagnitudeMeters,
                                    bool leftHandTraffic,
                                    std::optional<std::string> driverModelPath = std::nullopt);

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

// Advances a population and removes actors that reach the end of open routes.
// The population is unchanged if any actor has invalid runtime state.
std::size_t advanceMapTrafficActors(std::vector<MapTrafficActor>& actors, double elapsedSeconds);

class MapTrafficRouteNetwork {
  public:
    void addTileSplines(const std::vector<MapSplinePlacement>& splines, int tileX, int tileY);
    void finalize();
    std::optional<MapTrafficRoute> buildRoute(int splineId, std::size_t segmentLimit = 1024) const;
    std::vector<int> splineIds() const;

  private:
    std::unordered_map<int, MapSplinePlacementReference> splinesById_;
    std::unordered_set<int> ambiguousSplineIds_;
    bool finalized_ = false;
};

} // namespace openbus::map
