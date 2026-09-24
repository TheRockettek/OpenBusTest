#include "BusModelLoader.h"

#include "BusConfigLoader.h"
#include "Variables.h"

#include <algorithm>
#include <utility>

namespace openbus::rendering {

BusModelLoadResult loadBusModel(const std::filesystem::path& configPath,
                                const std::filesystem::path& modelRoot, Variables& variables) {
    const ModelConfig configuration = loadBusModelConfig(configPath, modelRoot, variables);
    BusModelLoadResult result;
    result.lodThresholds = configuration.lodThresholds;
    result.diagnostics = configuration.diagnostics;
    result.parts.reserve(configuration.parts.size());
    for (const ModelPart& source : configuration.parts) {
        if (source.objPath.empty()) {
            continue;
        }
        BusModelPart part;
        part.objPath = source.objPath;
        part.textureName = source.textureName;
        part.viewpoint = source.viewpoint;
        part.renderType = source.renderType;
        part.visibleVariable = source.visibleVariable;
        part.visibleValue = source.visibleValue;
        part.meshIdentifier = source.meshIdentifier;
        part.animationParent = source.animationParent;
        part.lodIndex = source.lodIndex;
        part.wheelAnimation.rotationVariable = source.wheelAnimation.rotationVariable;
        part.wheelAnimation.suspensionVariable = source.wheelAnimation.suspensionVariable;
        part.wheelAnimation.steeringVariable = source.wheelAnimation.steeringVariable;
        part.wheelAnimation.origin = source.wheelAnimation.origin;
        part.wheelAnimation.hasOrigin = source.wheelAnimation.hasOrigin;
        for (const auto& materialEntry : source.materialStates) {
            BusModelMaterialState state;
            state.texturePath = materialEntry.second.texturePath;
            state.textureName = materialEntry.second.textureName;
            state.environmentTextureName = materialEntry.second.environmentTextureName;
            state.environmentStrength = materialEntry.second.environmentStrength;
            state.alphaMode = materialEntry.second.alphaMode;
            state.noZwrite = materialEntry.second.noZwrite;
            state.alphaScaleVariable = materialEntry.second.alphaScaleVariable;
            state.textureChanges.reserve(materialEntry.second.textureChanges.size());
            for (const auto& change : materialEntry.second.textureChanges) {
                state.textureChanges.push_back({change.texturePath, change.textureName, change.layer,
                                                change.activationVariable});
            }
            part.materialStates.emplace(materialEntry.first, std::move(state));
        }
        result.parts.push_back(std::move(part));
    }
    for (BusModelPart& part : result.parts) {
        if (part.animationParent.empty()) {
            continue;
        }
        const auto parent = std::find_if(
            result.parts.begin(), result.parts.end(), [&](const BusModelPart& candidate) {
                return candidate.meshIdentifier == part.animationParent;
            });
        if (parent != result.parts.end() && !parent->wheelAnimation.rotationVariable.empty()) {
            part.wheelAnimation = parent->wheelAnimation;
        }
    }
    return result;
}

}  // namespace openbus::rendering
