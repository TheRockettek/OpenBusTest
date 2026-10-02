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
    std::string freeTextureName;
    std::string freeTextureVariable;
    int scriptTextureIndex = -1;
    int textTextureIndex = -1;
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
