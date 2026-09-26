#include "BusModelLoader.h"

#include "BusConfigLoader.h"
#include "PerfTrace.h"
#include "Variables.h"

#include <algorithm>
#include <utility>

namespace openbus::rendering {

BusModelLoadResult loadBusModel(const std::filesystem::path& configPath,
                                const std::filesystem::path& modelRoot, Variables& variables) {
    openbus::rendering::TraceScope trace("config", "loadBusModel");
    const ModelConfig configuration = loadBusModelConfig(configPath, modelRoot, variables);
    BusModelLoadResult result;
    result.lodThresholds = configuration.lodThresholds;
    result.diagnostics = configuration.diagnostics;
    result.parts.reserve(configuration.parts.size());
    for (const ModelPart& source : configuration.parts) {
        if (source.objPath.empty() || source.isShadow) {
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
        const auto copyMaterialState = [](const ModelMaterialState& sourceState) {
            BusModelMaterialState state;
            state.texturePath = sourceState.texturePath;
            state.textureName = sourceState.textureName;
            state.materialIndex = sourceState.materialIndex;
            state.environmentTextureName = sourceState.environmentTextureName;
            state.environmentStrength = sourceState.environmentStrength;
            state.alphaMode = sourceState.alphaMode;
            state.noZwrite = sourceState.noZwrite;
            state.noZcheck = sourceState.noZcheck;
            state.alphaScaleVariable = sourceState.alphaScaleVariable;
            state.transmapTextureName = sourceState.transmapTextureName;
            state.nightmapTextureName = sourceState.nightmapTextureName;
            state.lightmapTextureName = sourceState.lightmapTextureName;
            state.lightmapStrengthVariable = sourceState.lightmapStrengthVariable;
            state.texcoordTransXVariable = sourceState.texcoordTransXVariable;
            state.texcoordTransYVariable = sourceState.texcoordTransYVariable;
            state.bumpmapTextureName = sourceState.bumpmapTextureName;
            state.bumpmapStrength = sourceState.bumpmapStrength;
            state.textureAddressS = sourceState.textureAddressS;
            state.textureAddressT = sourceState.textureAddressT;
            state.textureChanges.reserve(sourceState.textureChanges.size());
            for (const auto& change : sourceState.textureChanges) {
                state.textureChanges.push_back({change.texturePath, change.textureName,
                                                change.layer, change.activationVariable});
            }
            return state;
        };
        for (const auto& materialEntry : source.materialStates) {
            part.materialStates.emplace(materialEntry.first,
                                        copyMaterialState(materialEntry.second));
        }
        part.materialStatesInOrder.reserve(source.materialStatesInOrder.size());
        for (const ModelMaterialState& materialState : source.materialStatesInOrder) {
            part.materialStatesInOrder.push_back(copyMaterialState(materialState));
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

} // namespace openbus::rendering
