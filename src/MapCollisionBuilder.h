#pragma once

#include "BusTypes.h"
#include "MapConfigLoader.h"

#include <cstddef>
#include <filesystem>
#include <vector>

namespace openbus::map {

struct MapRoadCollisionResult {
    std::vector<StaticCollisionMesh> meshes;
    std::size_t tilesVisited = 0;
    std::size_t splineSectionsAdded = 0;
    std::size_t skippedProfiles = 0;
    std::size_t skippedSplines = 0;
};

// Builds collision from runtime-visible .sli profile strips in the spawn tile and its
// eight immediate neighbours, matching the current terrain collision residency window.
MapRoadCollisionResult buildSpawnRoadCollision(const MapDefinition& map,
                                               std::size_t centerTileIndex,
                                               const std::filesystem::path& omsiRoot);

} // namespace openbus::map
