#include "BusModelLoader.h"

#include "BusConfigLoader.h"
#include "PerfTrace.h"
#include "Variables.h"

#include <algorithm>
#include <cctype>
#include <utility>

namespace openbus::rendering {

BusModelLoadResult loadBusModel(const std::filesystem::path& configPath,
                                const std::filesystem::path& modelRoot,
                                openbus::scripting::Vehicle& variables) {
    openbus::rendering::TraceScope trace("config", "loadBusModel");
    const ModelConfig configuration = loadBusModelConfig(configPath, modelRoot, variables);
    BusModelLoadResult result;
    result.interiorLights = configuration.interiorLights;
    result.enhancedLights = configuration.enhancedLights;
    result.spotlights = configuration.spotlights;
    result.ctcTemplates = configuration.ctcTemplates;
    result.ctcTextures = configuration.ctcTextures;
    result.scriptTextures = configuration.scriptTextures;
    result.textTextures = configuration.textTextures;
    result.collisionMeshes = configuration.collisionMeshes;
    result.hasBoundingBox = configuration.hasBoundingBox;
    result.boundingBox = configuration.boundingBox;
    result.lodThresholds = configuration.lodThresholds;
    result.diagnostics = configuration.diagnostics;
    result.parts.reserve(configuration.parts.size());
    for (const ModelPart& source : configuration.parts) {
        if (source.objPath.empty()) {
            continue;
        }
        BusModelPart part;
        part.objPath = source.objPath;
        part.bundleEntry = source.bundleEntry;
        part.textureName = source.textureName;
        part.viewpoint = source.viewpoint;
        part.renderType = source.renderType;
        std::string meshStem = source.objPath.stem().string();
        std::transform(
            meshStem.begin(), meshStem.end(), meshStem.begin(),
            [](unsigned char character) { return static_cast<char>(std::tolower(character)); });
        part.isShadow = source.isShadow || meshStem == "shadow";
        part.noCollision = source.noCollision;
        part.visibleVariable = source.visibleVariable;
        part.visibleValue = source.visibleValue;
        part.interiorLightIndexes = source.interiorLightIndexes;
        part.meshIdentifier = source.meshIdentifier;
        part.animationParent = source.animationParent;
        part.mouseEvent = source.mouseEvent;
        part.animations = source.animations;
        part.lodIndex = source.lodIndex;
        part.wheelAnimation.rotationVariable = source.wheelAnimation.rotationVariable;
        part.wheelAnimation.rotationScale = source.wheelAnimation.rotationScale;
        part.wheelAnimation.suspensionVariable = source.wheelAnimation.suspensionVariable;
        part.wheelAnimation.suspensionScale = source.wheelAnimation.suspensionScale;
        part.wheelAnimation.steeringVariable = source.wheelAnimation.steeringVariable;
        part.wheelAnimation.steeringScale = source.wheelAnimation.steeringScale;
        part.wheelAnimation.origin = source.wheelAnimation.origin;
        part.wheelAnimation.hasOrigin = source.wheelAnimation.hasOrigin;
        const auto copyMaterialState = [](const ModelMaterialState& sourceState) {
            return sourceState;
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
    return result;
}

} // namespace openbus::rendering
