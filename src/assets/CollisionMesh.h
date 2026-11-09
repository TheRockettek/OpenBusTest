#pragma once

#include "assets/ObjLoader.h"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace openbus::assets {

inline bool hasValidCollisionIndices(const rendering::ParsedObj& mesh,
                                     const rendering::ObjTriangle& triangle) {
    return std::all_of(triangle.indices.begin(), triangle.indices.end(), [&](const auto& index) {
        return index.position > 0 &&
               static_cast<std::size_t>(index.position) <= mesh.positions.size();
    });
}

inline bool isUsableCollisionTriangle(const rendering::ParsedObj& mesh,
                                      const rendering::ObjTriangle& triangle) {
    return hasValidCollisionIndices(mesh, triangle) &&
           std::all_of(triangle.indices.begin(), triangle.indices.end(), [&](const auto& index) {
               const auto& position = mesh.positions[static_cast<std::size_t>(index.position - 1)];
               return std::isfinite(position.x) && std::isfinite(position.y) &&
                      std::isfinite(position.z);
           });
}

} // namespace openbus::assets
