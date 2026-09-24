#pragma once

#include "ConfigurationTypes.h"

#include <array>
#include <filesystem>
#include <string>
#include <unordered_map>
#include <vector>

enum class ModelConfigKind { Vehicle, Bus, SceneryObject };

struct ModelMaterialState {
    std::filesystem::path texturePath;
    std::string textureName;
    std::string environmentTextureName;
    double environmentStrength = 0.0;
    int alphaMode = 0;
    bool noZwrite = false;
    std::string alphaScaleVariable;
};

struct ModelWheelAnimation {
    std::string rotationVariable;
    std::string suspensionVariable;
    std::string steeringVariable;
    std::array<double, 3> origin = {};
    bool hasOrigin = false;
};

struct ModelAnimation {
    std::string type;
    std::string variable;
    double scale = 0.0;
};

struct ModelPart {
    std::filesystem::path objPath;
    std::filesystem::path sourceMeshPath;
    std::string textureName;
    std::array<double, 3> color = {0.65, 0.65, 0.65};
    int viewpoint = 0;
    int renderType = 2;
    std::string meshIdentifier;
    std::string animationParent;
    std::string visibleVariable;
    int visibleValue = 0;
    std::array<int, 4> interiorLightIndexes = {-1, -1, -1, -1};
    std::unordered_map<std::string, ModelMaterialState> materialStates;
    int lodIndex = -1;
    std::vector<ModelAnimation> animations;
    ModelWheelAnimation wheelAnimation;
};

struct ModelConfig {
    std::vector<ModelPart> parts;
    std::vector<double> lodThresholds;
    ConfigurationDiagnostics diagnostics;
};
