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

struct BusModelMaterialState {
    std::filesystem::path texturePath;
    std::string textureName;
    int materialIndex = -1;
    std::string environmentTextureName;
    double environmentStrength = 0.0;
    int alphaMode = 0;
    bool noZwrite = false;
    bool noZcheck = false;
    std::string alphaScaleVariable;
    std::string transmapTextureName;
    std::string nightmapTextureName;
    std::string lightmapTextureName;
    std::string lightmapStrengthVariable;
    std::string freeTextureVariable;
    std::string texcoordTransXVariable;
    std::string texcoordTransYVariable;
    std::string bumpmapTextureName;
    double bumpmapStrength = 0.0;
    TextureAddressMode textureAddressS = TextureAddressMode::Repeat;
    TextureAddressMode textureAddressT = TextureAddressMode::Repeat;
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
    std::vector<BusModelMaterialState> materialStatesInOrder;
    std::vector<ModelAnimation> animations;
    int lodIndex = -1;
    WheelAnimation wheelAnimation;
};

struct BusModelLoadResult {
    std::vector<BusModelPart> parts;
    std::vector<double> lodThresholds;
    ConfigurationDiagnostics diagnostics;
};

BusModelLoadResult loadBusModel(const std::filesystem::path& configPath,
                                const std::filesystem::path& modelRoot,
                                openbus::scripting::Vehicle& variables);

} // namespace openbus::rendering
