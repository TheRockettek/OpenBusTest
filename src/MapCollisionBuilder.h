#pragma once

#include "BusTypes.h"
#include "MapConfigLoader.h"
#include "MapSceneryPlacement.h"
#include "MapRoadGeometry.h"
#include "O3DLoader.h"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <unordered_map>
#include <vector>

namespace openbus::map {

struct MapRoadCollisionResult {
    std::vector<StaticCollisionMesh> meshes;
    std::size_t tilesVisited = 0;
    std::size_t splineSectionsAdded = 0;
    std::size_t sceneryCollisionObjectsAdded = 0;
    std::size_t sceneryCollisionMeshesMissing = 0;
    std::size_t skippedProfiles = 0;
    std::size_t skippedSplines = 0;
};

struct MapSceneryCollisionObject {
    MapSceneryPose pose;
    double worldZ = 0.0;
    double minX = 0.0;
    double minY = 0.0;
    double maxX = 0.0;
    double maxY = 0.0;
    std::vector<std::shared_ptr<openbus::rendering::ParsedObj>> meshes;
};

// Parsed once for streamed collision tiles. Large scenery meshes (bridges and custom
// junctions) are indexed into every tile their collision bounds overlap.
struct MapSceneryCollisionIndex {
    std::vector<MapSceneryCollisionObject> objects;
    std::unordered_map<std::uint64_t, std::vector<std::size_t>> objectsByTile;
};

std::shared_ptr<const MapSceneryCollisionIndex>
buildMapSceneryCollisionIndex(const MapDefinition& map, const std::filesystem::path& omsiRoot);

// Builds map collision from road-profile strips and authored SCO/model [collision_mesh]
// scenery throughout the selected map, partitioned into per-tile ODE mesh inputs.
MapRoadCollisionResult buildMapRoadCollision(const MapDefinition& map,
                                             const std::filesystem::path& omsiRoot);
// Builds only the requested tile; an absent tile returns an empty result.
MapRoadCollisionResult
buildMapRoadCollisionTile(const MapDefinition& map, const std::filesystem::path& omsiRoot,
                          int tileX, int tileY, const MapRoadConnectionIndex* connections = nullptr,
                          const MapSceneryCollisionIndex* scenery = nullptr);
// Builds spline-strip collision for one tile without constructing the map-wide scenery index.
MapRoadCollisionResult
buildMapSplineCollisionTile(const MapDefinition& map, const std::filesystem::path& omsiRoot,
                            int tileX, int tileY,
                            const MapRoadConnectionIndex* connections = nullptr);

// Metadata only, independent of geometry residency. Streaming workers retain this once per map.
// Resolve profile cant widths before finalizing joins; unavailable widths retain legacy cant.
MapRoadConnectionIndex loadMapRoadConnections(const MapDefinition& map,
                                              const std::filesystem::path& omsiRoot = {});

} // namespace openbus::map
