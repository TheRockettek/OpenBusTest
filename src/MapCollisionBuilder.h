#pragma once

#include "BusTypes.h"
#include "MapConfigLoader.h"
#include "MapRoadGeometry.h"

#include <cstddef>
#include <filesystem>
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

// Builds map collision from road-profile strips and authored SCO/model [collision_mesh]
// scenery throughout the selected map, partitioned into per-tile ODE mesh inputs.
MapRoadCollisionResult buildMapRoadCollision(const MapDefinition& map,
                                             const std::filesystem::path& omsiRoot);
// Builds only the requested tile; an absent tile returns an empty result.
MapRoadCollisionResult buildMapRoadCollisionTile(const MapDefinition& map,
                                                 const std::filesystem::path& omsiRoot, int tileX,
                                                 int tileY,
                                                 const MapRoadConnectionIndex* connections = nullptr);

// Metadata only, independent of geometry residency. Streaming workers retain this once per map.
MapRoadConnectionIndex loadMapRoadConnections(const MapDefinition& map);

} // namespace openbus::map
