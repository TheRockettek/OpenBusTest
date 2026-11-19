#include "render/HumanMeshGeometry.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace openbus::rendering {

std::optional<HumanMeshBounds> humanRenderablePositionBounds(const ParsedObj& mesh) {
    HumanMeshBounds bounds;
    bounds.minimum.fill(std::numeric_limits<double>::infinity());
    bounds.maximum.fill(-std::numeric_limits<double>::infinity());
    bool hasPosition = false;
    for (const ObjTriangle& triangle : mesh.triangles) {
        for (const ObjIndex& index : triangle.indices) {
            if (index.position <= 0 ||
                static_cast<std::size_t>(index.position) > mesh.positions.size()) {
                continue;
            }
            const ObjPosition& position =
                mesh.positions[static_cast<std::size_t>(index.position - 1)];
            const std::array<double, 3> values = {position.x, position.y, position.z};
            if (!std::all_of(values.begin(), values.end(),
                             [](double value) { return std::isfinite(value); })) {
                throw std::invalid_argument(
                    "Human renderable mesh positions must contain finite values");
            }
            for (std::size_t axis = 0; axis < values.size(); ++axis) {
                bounds.minimum[axis] = std::min(bounds.minimum[axis], values[axis]);
                bounds.maximum[axis] = std::max(bounds.maximum[axis], values[axis]);
            }
            hasPosition = true;
        }
    }
    return hasPosition ? std::optional<HumanMeshBounds>(bounds) : std::nullopt;
}

HumanMeshGeometry prepareHumanMeshGeometry(const ParsedObj& mesh, const HumanSkinWeights& skin,
                                           const HumanJoints& seatedJoints, bool inferSeatedWeights,
                                           double height, double modelScale,
                                           const std::array<double, 2>& horizontalCenter,
                                           double minimumZ) {
    if (skin.hasWeights && skin.vertices.size() != mesh.positions.size()) {
        throw std::invalid_argument("Human skin weights must match the source position count");
    }
    if (!std::isfinite(modelScale) || modelScale <= 0.0 || !std::isfinite(horizontalCenter[0]) ||
        !std::isfinite(horizontalCenter[1]) || !std::isfinite(minimumZ) ||
        (inferSeatedWeights && (!std::isfinite(height) || height <= 0.0))) {
        throw std::invalid_argument("Human mesh geometry requires finite scaling and bounds");
    }
    HumanMeshGeometry geometry;
    const auto validIndex = [](int index, std::size_t count) {
        return index > 0 && static_cast<std::size_t>(index) <= count;
    };
    for (const ObjTriangle& triangle : mesh.triangles) {
        const auto sourceMaterial = mesh.materials.find(triangle.material);
        const int materialIndex =
            sourceMaterial == mesh.materials.end() ? 0 : sourceMaterial->second.materialIndex;
        for (const ObjIndex& index : triangle.indices) {
            if (!validIndex(index.position, mesh.positions.size())) {
                continue;
            }
            const std::size_t positionIndex = static_cast<std::size_t>(index.position - 1);
            const ObjPosition& sourcePosition = mesh.positions[positionIndex];
            const std::array<double, 4> color = positionIndex < mesh.vertexColors.size()
                                                    ? mesh.vertexColors[positionIndex]
                                                    : std::array<double, 4>{1.0, 1.0, 1.0, 1.0};
            std::array<double, 3> normal = {0.0, 0.0, 1.0};
            if (validIndex(index.normal, mesh.normals.size())) {
                const ObjNormal& sourceNormal =
                    mesh.normals[static_cast<std::size_t>(index.normal - 1)];
                normal = {sourceNormal.x, sourceNormal.y, sourceNormal.z};
                const double length = std::hypot(normal[0], normal[1], normal[2]);
                if (length > 1.0e-8) {
                    for (double& component : normal) {
                        component /= length;
                    }
                }
            }
            float u = 0.0F;
            float v = 0.0F;
            if (validIndex(index.texCoord, mesh.texCoords.size())) {
                const ObjTexCoord& coordinate =
                    mesh.texCoords[static_cast<std::size_t>(index.texCoord - 1)];
                u = static_cast<float>(coordinate.u);
                v = static_cast<float>(1.0 - coordinate.v);
            }
            const std::array<double, 3> position = {
                (sourcePosition.x - horizontalCenter[0]) * modelScale,
                (sourcePosition.y - horizontalCenter[1]) * modelScale,
                (sourcePosition.z - minimumZ) * modelScale};
            geometry.verticesByMaterial[materialIndex].push_back(
                {static_cast<float>(position[0]), static_cast<float>(position[1]),
                 static_cast<float>(position[2]), u, v, 0.0F, static_cast<float>(normal[0]),
                 static_cast<float>(normal[1]), static_cast<float>(normal[2]),
                 static_cast<float>(color[0]), static_cast<float>(color[1]),
                 static_cast<float>(color[2]), static_cast<float>(color[3])});
            if (skin.hasWeights) {
                geometry.weightsByMaterial[materialIndex].push_back(skin.vertices[positionIndex]);
            }
            if (inferSeatedWeights) {
                HumanVertexWeights seatedWeights = {};
                if (skin.hasWeights) {
                    seatedWeights = skin.vertices[positionIndex];
                }
                const bool weighted =
                    std::any_of(seatedWeights.begin(), seatedWeights.begin() + HUMAN_BONE_COUNT,
                                [](float weight) { return weight > 0.0F; });
                if (!weighted) {
                    seatedWeights = inferHumanSkinWeights(position, seatedJoints, height);
                }
                geometry.seatedWeightsByMaterial[materialIndex].push_back(seatedWeights);
            }
        }
    }
    return geometry;
}

} // namespace openbus::rendering
