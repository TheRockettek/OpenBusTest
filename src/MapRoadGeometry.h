#pragma once

#include "MapConfigLoader.h"
#include "MapSplineGeometry.h"
#include "MapSplineProfile.h"

#include <array>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace openbus::map {

struct MapRoadVertex {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double textureU = 0.0;
    double textureV = 0.0;
};

struct MapRoadSectionGeometry {
    std::vector<MapRoadVertex> vertices;
    std::vector<int> indices;
};

// Provisional shared seam frame: profile height remains world-up; lateralAxis is the
// world-space displacement per metre of right-positive (possibly mirrored) lateral offset.
struct MapRoadEndpointFrame {
    std::array<double, 3> center = {};
    std::array<double, 3> lateralAxis = {};
};

struct MapRoadSplineJoins {
    std::optional<MapRoadEndpointFrame> start;
    std::optional<MapRoadEndpointFrame> end;
};

// Build once from all map placements, then finalize before querying. Only nearby reciprocal
// links with unique nonzero IDs are joined. Values are owned; tile residency is irrelevant.
class MapRoadConnectionIndex {
  public:
    void addSpline(const MapSplinePlacement& spline, int tileX, int tileY);
    void finalize();
    MapRoadSplineJoins joinsFor(const MapSplinePlacement& spline, int tileX, int tileY) const;

  private:
    struct Placement {
        MapSplinePlacement spline;
        int tileX = 0;
        int tileY = 0;
    };
    std::unordered_map<int, Placement> placements_;
    std::unordered_set<int> ambiguousIds_;
    std::unordered_map<int, MapRoadSplineJoins> joins_;
};

// Builds one visible .sli cross-section strip in world coordinates. Rendering and collision
// consume this same geometry so road physics cannot drift from the displayed profile.
MapRoadSectionGeometry buildMapRoadSectionGeometry(const MapSplinePlacement& spline,
                                                   const std::vector<MapSplineSample>& centerline,
                                                   const MapSplineProfileSection& section,
                                                   const MapRoadSplineJoins& joins = {});

} // namespace openbus::map
