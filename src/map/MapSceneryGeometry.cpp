#include "map/MapSceneryGeometry.h"

#include "assets/ObjLoader.h"

#include <array>
#include <cmath>

namespace openbus::map {

MapSceneryGeometry prepareMapSceneryGeometry(const openbus::rendering::ParsedObj& parsed) {
    using namespace openbus::rendering;
    MapSceneryGeometry geometry;
    const auto validIndex = [](int value, std::size_t count) {
        return value > 0 && static_cast<std::size_t>(value) <= count;
    };
    for (const ObjTriangle& triangle : parsed.triangles) {
        const auto sourceMaterial = parsed.materials.find(triangle.material);
        const int materialIndex =
            sourceMaterial == parsed.materials.end() ? 0 : sourceMaterial->second.materialIndex;
        for (const ObjIndex& index : triangle.indices) {
            if (!validIndex(index.position, parsed.positions.size())) {
                ++geometry.invalidPositionIndices;
                continue;
            }
            const std::size_t positionIndex = static_cast<std::size_t>(index.position - 1);
            const ObjPosition& position = parsed.positions[positionIndex];
            const std::array<double, 4> color = positionIndex < parsed.vertexColors.size()
                                                    ? parsed.vertexColors[positionIndex]
                                                    : std::array<double, 4>{1.0, 1.0, 1.0, 1.0};
            std::array<double, 3> normal = {0.0, 0.0, 1.0};
            if (validIndex(index.normal, parsed.normals.size())) {
                const ObjNormal& sourceNormal =
                    parsed.normals[static_cast<std::size_t>(index.normal - 1)];
                normal = {sourceNormal.x, sourceNormal.y, sourceNormal.z};
                const double length = std::sqrt(normal[0] * normal[0] + normal[1] * normal[1] +
                                                normal[2] * normal[2]);
                if (length > 1.0e-8) {
                    for (double& component : normal) {
                        component /= length;
                    }
                }
            }
            float u = 0.0F;
            float v = 0.0F;
            if (validIndex(index.texCoord, parsed.texCoords.size())) {
                const ObjTexCoord& coordinate =
                    parsed.texCoords[static_cast<std::size_t>(index.texCoord - 1)];
                u = static_cast<float>(coordinate.u);
                v = static_cast<float>(1.0 - coordinate.v);
            }
            geometry.verticesByMaterial[materialIndex].push_back(
                {static_cast<float>(position.x), static_cast<float>(position.y),
                 static_cast<float>(position.z), u, v, 0.0F, static_cast<float>(normal[0]),
                 static_cast<float>(normal[1]), static_cast<float>(normal[2]),
                 static_cast<float>(color[0]), static_cast<float>(color[1]),
                 static_cast<float>(color[2]), static_cast<float>(color[3])});
        }
    }
    return geometry;
}

} // namespace openbus::map
