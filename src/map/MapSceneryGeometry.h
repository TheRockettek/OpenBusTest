#pragma once

#include "render/ModelVertex.h"

#include <cstddef>
#include <unordered_map>
#include <vector>

namespace openbus::rendering {
struct ParsedObj;
}

namespace openbus::map {

// CPU-side scenery geometry in map coordinates, independent of graphics API resources.
struct MapSceneryGeometry {
    std::unordered_map<int, std::vector<openbus::rendering::ModelVertex>> verticesByMaterial;
    std::size_t invalidPositionIndices = 0;
};

[[nodiscard]] MapSceneryGeometry
prepareMapSceneryGeometry(const openbus::rendering::ParsedObj& parsed);

} // namespace openbus::map
