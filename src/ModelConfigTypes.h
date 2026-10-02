#pragma once

#include "ConfigurationTypes.h"

#include <array>
#include <filesystem>
#include <string>
#include <unordered_map>
#include <vector>

enum class ModelConfigKind { Vehicle, Bus, SceneryObject };

enum class TextureAddressMode { Repeat, Clamp, Border, Mirror, MirrorOnce };

struct ModelMaterialState {
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

struct ModelScriptTexture {
    int slot = -1;
    int width = 0;
    int height = 0;
    std::vector<std::string> options;
};

struct ModelTextTexture {
    int slot = -1;
    bool enhanced = false;
    std::vector<std::string> values;
};

struct ModelWheelAnimation {
    std::string rotationVariable;
    double rotationScale = 0.0;
    std::string suspensionVariable;
    double suspensionScale = 0.0;
    std::string steeringVariable;
    double steeringScale = 0.0;
    std::array<double, 3> origin = {};
    std::array<double, 3> originRotation = {};
    bool hasOrigin = false;
};

enum class ModelAnimationOriginType { Translation, RotationX, RotationY, RotationZ, FromMesh };

struct ModelAnimationOrigin {
    ModelAnimationOriginType type = ModelAnimationOriginType::Translation;
    std::array<double, 3> value = {};
};

struct ModelAnimation {
    std::string type;
    std::string variable;
    double scale = 0.0;
    double maxSpeed = 0.0;
    double delay = 0.0;
    double offset = 0.0;
    std::array<double, 3> origin = {};
    std::array<double, 3> originRotation = {};
    bool hasOrigin = false;
    bool originFromMesh = false;
    std::vector<ModelAnimationOrigin> originOperations;
    std::array<double, 9> meshRotation = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    bool hasMeshRotation = false;
    std::array<double, 16> meshTransform = {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                                            0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    bool hasMeshTransform = false;
};

struct ModelInteriorLight {
    std::string controller;
    std::array<double, 7> parameters = {};
};

struct ModelEnhancedLight {
    bool enhanced = false;
    std::string controller;
    std::vector<double> parameters;
    std::string textureName;
};

struct ModelSpotlight {
    std::array<double, 12> parameters = {};
};

struct ModelCtcTemplate {
    std::string name;
    std::string texturePath;
    int index = 0;
};

struct ModelCtcTexture {
    std::string slot;
    std::string textureName;
};

struct ModelPart {
    std::filesystem::path objPath;
    std::string bundleEntry;
    std::filesystem::path sourceMeshPath;
    std::string textureName;
    std::array<double, 3> color = {0.65, 0.65, 0.65};
    int viewpoint = 0;
    int renderType = 2;
    bool isShadow = false;
    std::string meshIdentifier;
    std::string animationParent;
    std::string mouseEvent;
    std::string visibleVariable;
    int visibleValue = 0;
    std::array<int, 4> interiorLightIndexes = {-1, -1, -1, -1};
    std::unordered_map<std::string, ModelMaterialState> materialStates;
    std::vector<ModelMaterialState> materialStatesInOrder;
    int lodIndex = -1;
    std::vector<ModelAnimation> animations;
    ModelWheelAnimation wheelAnimation;
};

struct ModelConfig {
    std::vector<ModelPart> parts;
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
