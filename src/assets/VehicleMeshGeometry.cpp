#include "assets/VehicleMeshGeometry.h"

#include "assets/ObjLoader.h"

#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace openbus::rendering {
namespace {

bool validIndex(int index, std::size_t count) {
    return index > 0 && static_cast<std::size_t>(index) <= count;
}

std::array<double, 3> convertPosition(const ObjPosition& position) {
    return {position.y, -position.x, position.z};
}

std::array<double, 3> normalizeVector(std::array<double, 3> value) {
    const double length =
        std::sqrt(value[0] * value[0] + value[1] * value[1] + value[2] * value[2]);
    if (length <= 1.0e-8) {
        return {0.0, 0.0, 1.0};
    }
    for (double& component : value) {
        component /= length;
    }
    return value;
}

} // namespace

std::vector<ModelVertex> prepareVehicleMeshGeometry(const ParsedObj& model,
                                                    std::span<const ObjTriangle* const> triangles) {
    if (triangles.size() > std::numeric_limits<std::size_t>::max() / 3) {
        throw std::runtime_error("triangle vertex count overflow");
    }
    std::vector<ModelVertex> vertices;
    vertices.reserve(triangles.size() * 3);
    for (const ObjTriangle* triangle : triangles) {
        if (triangle == nullptr) {
            throw std::invalid_argument("vehicle triangle must not be null");
        }
        std::array<double, 3> fallbackNormal = {0.0, 0.0, 1.0};
        if (validIndex(triangle->indices[0].position, model.positions.size()) &&
            validIndex(triangle->indices[1].position, model.positions.size()) &&
            validIndex(triangle->indices[2].position, model.positions.size())) {
            const auto first = convertPosition(
                model.positions[static_cast<std::size_t>(triangle->indices[0].position - 1)]);
            const auto second = convertPosition(
                model.positions[static_cast<std::size_t>(triangle->indices[1].position - 1)]);
            const auto third = convertPosition(
                model.positions[static_cast<std::size_t>(triangle->indices[2].position - 1)]);
            const std::array<double, 3> edgeA = {second[0] - first[0], second[1] - first[1],
                                                 second[2] - first[2]};
            const std::array<double, 3> edgeB = {third[0] - first[0], third[1] - first[1],
                                                 third[2] - first[2]};
            fallbackNormal = normalizeVector({edgeA[1] * edgeB[2] - edgeA[2] * edgeB[1],
                                              edgeA[2] * edgeB[0] - edgeA[0] * edgeB[2],
                                              edgeA[0] * edgeB[1] - edgeA[1] * edgeB[0]});
        }
        for (const ObjIndex& index : triangle->indices) {
            if (!validIndex(index.position, model.positions.size())) {
                continue;
            }
            const std::size_t positionIndex = static_cast<std::size_t>(index.position - 1);
            const ObjPosition& position = model.positions[positionIndex];
            const std::array<double, 4> color = positionIndex < model.vertexColors.size()
                                                    ? model.vertexColors[positionIndex]
                                                    : std::array<double, 4>{1.0, 1.0, 1.0, 1.0};
            std::array<double, 3> normal = fallbackNormal;
            if (validIndex(index.normal, model.normals.size())) {
                const ObjNormal& sourceNormal =
                    model.normals[static_cast<std::size_t>(index.normal - 1)];
                normal = normalizeVector({sourceNormal.y, -sourceNormal.x, sourceNormal.z});
            }
            const ObjTexCoord* texCoord = nullptr;
            if (validIndex(index.texCoord, model.texCoords.size())) {
                texCoord = &model.texCoords[static_cast<std::size_t>(index.texCoord - 1)];
            }
            vertices.push_back({static_cast<float>(position.y), static_cast<float>(-position.x),
                                static_cast<float>(position.z),
                                texCoord ? static_cast<float>(texCoord->u) : 0.0f,
                                texCoord ? static_cast<float>(1.0 - texCoord->v) : 0.0f, 0.0f,
                                static_cast<float>(normal[0]), static_cast<float>(normal[1]),
                                static_cast<float>(normal[2]), static_cast<float>(color[0]),
                                static_cast<float>(color[1]), static_cast<float>(color[2]),
                                static_cast<float>(color[3])});
        }
    }
    return vertices;
}

} // namespace openbus::rendering
