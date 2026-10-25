#pragma once

#include "map/MapConfigLoader.h"
#include "map/MapSplineGeometry.h"
#include "map/MapSplineProfile.h"

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

struct MapRoadCantContribution {
    double cantPercent = 0.0; // Weighted percentage in the frame's lateral direction.
    std::optional<double> halfWidth;
};

// Provisional shared seam frame: profile height remains world-up; lateralAxis is the
// non-cant displacement per metre of right-positive (possibly mirrored) lateral offset.
struct MapRoadEndpointFrame {
    std::array<double, 3> center = {};
    std::array<double, 3> lateralAxis = {};
    // A seam averages displacements, not just percentages: widths can differ.
    std::array<MapRoadCantContribution, 2> cantContributions = {};
};

struct MapRoadSplineJoins {
    std::optional<MapRoadEndpointFrame> start;
    std::optional<MapRoadEndpointFrame> end;
};

// Build once from all map placements, then finalize before querying. Only nearby reciprocal
// links with unique nonzero IDs are joined. Values are owned; tile residency is irrelevant.
class MapRoadConnectionIndex {
  public:
    explicit MapRoadConnectionIndex(MapCoordinates coordinates = {}) : coordinates_(coordinates) {}
    void addSpline(const MapSplinePlacement& spline, int tileX, int tileY,
                   std::optional<double> cantHalfWidth = std::nullopt);
    void finalize();
    // Frames are expressed in the requested tile's unscaled metric frame, with its grid
    // origin retained, ready for buildMapRoadSectionGeometry's final projection.
    MapRoadSplineJoins joinsFor(const MapSplinePlacement& spline, int tileX, int tileY) const;

  private:
    MapCoordinates coordinates_;
    struct Placement {
        MapSplinePlacement spline;
        int tileX = 0;
        int tileY = 0;
        std::optional<double> cantHalfWidth;
    };
    std::unordered_map<int, Placement> placements_;
    std::unordered_set<int> ambiguousIds_;
    std::unordered_map<int, MapRoadSplineJoins> joins_;
};

// Builds one visible .sli cross-section strip in world coordinates. Rendering and collision
// consume this same geometry so road physics cannot drift from the displayed profile.
// A supplied cant half-width clamps lateral cant displacement; nullopt keeps it unbounded.
// Centerline samples use authored metres plus the tile's grid origin. Apply transform only
// after constructing the profile and joining endpoints, so widths, cant and UVs stay authored.
MapRoadSectionGeometry buildMapRoadSectionGeometry(
    const MapSplinePlacement& spline, const std::vector<MapSplineSample>& centerline,
    const MapSplineProfileSection& section, const MapRoadSplineJoins& joins = {},
    std::optional<double> cantHalfWidth = std::nullopt, const MapTileTransform& transform = {});

// Computes smooth, map-up-facing normals from the final rendered strip triangles.
std::vector<std::array<double, 3>>
computeMapRoadVertexNormals(const MapRoadSectionGeometry& geometry);

} // namespace openbus::map
