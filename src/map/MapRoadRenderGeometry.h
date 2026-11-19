#pragma once

#include "map/MapRoadGeometry.h"
#include "render/ModelVertex.h"

#include <cstdint>
#include <filesystem>
#include <limits>
#include <map>
#include <span>
#include <string>
#include <vector>

namespace openbus::map {

struct RoadMeshGeometry {
    std::vector<openbus::rendering::ModelVertex> vertices;
    std::vector<std::uint32_t> indices;
    std::filesystem::path texturePath;
    double minX = std::numeric_limits<double>::infinity();
    double minY = std::numeric_limits<double>::infinity();
    double maxX = -std::numeric_limits<double>::infinity();
    double maxY = -std::numeric_limits<double>::infinity();
    double minZ = std::numeric_limits<double>::infinity();
    double maxZ = -std::numeric_limits<double>::infinity();
    int tileX = 0;
    int tileY = 0;
    int alphaMode = 0;
    bool textured = false;
    bool supportsMoisture = false;
    bool weatherClassified = false;
};

struct MapRoadSplineRenderInput {
    MapSplinePlacement spline;
    MapRoadSplineJoins joins;
    std::filesystem::path profilePath;
    const MapSplineProfile* profile = nullptr;
};

// Prepares render batches from CPU map/profile data without depending on a graphics API.
std::map<std::string, RoadMeshGeometry>
prepareMapRoadRenderBatches(int tileX, int tileY, const MapCoordinates& coordinates,
                            const std::filesystem::path& omsiRoot,
                            std::span<const MapRoadSplineRenderInput> splines);

// Appends a road strip in tile-local GPU vertex coordinates, retaining world-space bounds.
void appendMapRoadProfile(RoadMeshGeometry& road, const MapSplinePlacement& spline,
                          const std::vector<MapSplineSample>& centerline,
                          const MapSplineProfileSection& section, const MapRoadSplineJoins& joins,
                          std::optional<double> cantHalfWidth, const MapCoordinates& coordinates);

} // namespace openbus::map
