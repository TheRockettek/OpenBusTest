#pragma once

#include "ConfigurationTypes.h"
#include "ModelConfigTypes.h"

#include <array>
#include <filesystem>
#include <string>
#include <unordered_map>
#include <vector>

class Variables;

namespace openbus::rendering {

struct WheelAnimation {
    std::string rotationVariable;
    std::string suspensionVariable;
    std::string steeringVariable;
    std::array<double, 3> origin = {};
    bool hasOrigin = false;
};

struct BusModelMaterialState {
    std::filesystem::path texturePath;
    std::string textureName;
    std::string environmentTextureName;
    double environmentStrength = 0.0;
    int alphaMode = 0;
    bool noZwrite = false;
    std::string alphaScaleVariable;
    struct TextureChange {
        std::filesystem::path texturePath;
        std::string textureName;
        int layer = 0;
        std::string activationVariable;
    };
    std::vector<TextureChange> textureChanges;
};

struct BusModelPart {
    std::filesystem::path objPath;
    std::filesystem::path texturePath;
    std::string textureName;
    std::array<double, 3> color = {0.65, 0.65, 0.65};
    int viewpoint = 0;
    int renderType = 2;
    std::string visibleVariable;
    int visibleValue = 0;
    std::string meshIdentifier;
    std::string animationParent;
    std::unordered_map<std::string, BusModelMaterialState> materialStates;
    int lodIndex = -1;
    WheelAnimation wheelAnimation;
};

struct BusModelLoadResult {
    std::vector<BusModelPart> parts;
    std::vector<double> lodThresholds;
    ConfigurationDiagnostics diagnostics;
};

BusModelLoadResult loadBusModel(const std::filesystem::path& configPath,
                                const std::filesystem::path& modelRoot, Variables& variables);

}  // namespace openbus::rendering
