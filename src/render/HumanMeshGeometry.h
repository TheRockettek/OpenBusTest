#pragma once

#include "render/HumanSkinning.h"
#include "render/ModelVertex.h"

#include <array>
#include <optional>
#include <unordered_map>
#include <vector>

namespace openbus::rendering {

struct HumanMeshGeometry {
    std::unordered_map<int, std::vector<ModelVertex>> verticesByMaterial;
    std::unordered_map<int, std::vector<HumanVertexWeights>> weightsByMaterial;
    std::unordered_map<int, std::vector<HumanVertexWeights>> seatedWeightsByMaterial;
};

struct HumanMeshBounds {
    std::array<double, 3> minimum = {};
    std::array<double, 3> maximum = {};
};

// Returns bounds over only source positions referenced by renderable triangle corners.
[[nodiscard]] std::optional<HumanMeshBounds> humanRenderablePositionBounds(const ParsedObj& mesh);

// Expands triangle corners into the production human coordinate frame and material groups.
// The supplied weights are indexed by parsed source position, not by triangle corner.
[[nodiscard]] HumanMeshGeometry
prepareHumanMeshGeometry(const ParsedObj& mesh, const HumanSkinWeights& skin,
                         const HumanJoints& seatedJoints, bool inferSeatedWeights, double height,
                         double modelScale, const std::array<double, 2>& horizontalCenter,
                         double minimumZ);

} // namespace openbus::rendering
