#pragma once

#include "ModelConfigTypes.h"

#include <array>
#include <filesystem>
#include <string>
#include <vector>

namespace openbus::scripting {
class SceneryObject;
}

struct SceneryTreeDefinition {
    std::string texturePath;
    double minimumHeight = 0.0;
    double maximumHeight = 0.0;
    double minimumRatio = 1.0;
    double maximumRatio = 1.0;
};

struct SceneryObjectConfig {
    std::filesystem::path sourcePath;
    std::filesystem::path modelPath;
    std::filesystem::path collisionMesh;
    bool noCollision = false;
    bool surface = false;
    bool onlyEditor = false;
    bool absoluteHeight = false;
    bool hasBoundingBox = false;
    std::array<double, 6> boundingBox = {};
    std::vector<std::string> scripts;
    std::vector<std::string> variableLists;
    std::vector<std::string> stringVariableLists;
    std::vector<std::string> constantFiles;
    std::vector<SceneryTreeDefinition> trees;
    ConfigurationDiagnostics diagnostics;
};

SceneryObjectConfig loadSceneryObjectFile(const std::filesystem::path& configPath);
std::filesystem::path
resolveSceneryObjectModelConfigPath(const SceneryObjectConfig& configuration);

ModelConfig loadSceneryObjectConfig(const std::filesystem::path& configPath,
                                    const std::filesystem::path& modelRoot,
                                    openbus::scripting::SceneryObject& variables);
