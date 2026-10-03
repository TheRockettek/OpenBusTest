#pragma once

#include "ConfigurationTypes.h"
#include "ModelConfigTypes.h"

#include <array>
#include <filesystem>
#include <string>
#include <unordered_map>
#include <vector>

namespace openbus::scripting {
class Vehicle;
}

namespace openbus::rendering {

struct WheelAnimation {
    std::string rotationVariable;
    double rotationScale = 0.0;
    std::string suspensionVariable;
    double suspensionScale = 0.0;
    std::string steeringVariable;
    double steeringScale = 0.0;
    std::array<double, 3> origin = {};
    bool hasOrigin = false;
};

using BusModelMaterialState = ModelMaterialState;

struct BusModelPart {
    std::filesystem::path objPath;
    std::string bundleEntry;
    std::filesystem::path texturePath;
    std::string textureName;
    std::array<double, 3> color = {0.65, 0.65, 0.65};
    int viewpoint = 0;
    int renderType = 2;
    std::string visibleVariable;
    int visibleValue = 0;
    std::array<int, 4> interiorLightIndexes = {-1, -1, -1, -1};
    std::string meshIdentifier;
    std::string animationParent;
    std::string mouseEvent;
    std::unordered_map<std::string, BusModelMaterialState> materialStates;
    std::vector<BusModelMaterialState> materialStatesInOrder;
    std::vector<ModelAnimation> animations;
    int lodIndex = -1;
    WheelAnimation wheelAnimation;
};

struct BusModelLoadResult {
    std::vector<BusModelPart> parts;
    std::vector<ModelInteriorLight> interiorLights;
    std::vector<ModelEnhancedLight> enhancedLights;
    std::vector<ModelSpotlight> spotlights;
    std::vector<ModelCtcTemplate> ctcTemplates;
    std::vector<ModelCtcTexture> ctcTextures;
    std::vector<ModelScriptTexture> scriptTextures;
    std::vector<ModelTextTexture> textTextures;
    std::vector<double> lodThresholds;
    ConfigurationDiagnostics diagnostics;
};

BusModelLoadResult loadBusModel(const std::filesystem::path& configPath,
                                const std::filesystem::path& modelRoot,
                                openbus::scripting::Vehicle& variables);

} // namespace openbus::rendering
