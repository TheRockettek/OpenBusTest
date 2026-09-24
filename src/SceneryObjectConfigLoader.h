#pragma once

#include "ModelConfigTypes.h"

#include <array>
#include <filesystem>
#include <string>
#include <vector>

class Variables;

struct SceneryObjectConfig {
    std::filesystem::path sourcePath;
    std::filesystem::path modelPath;
    std::filesystem::path collisionMesh;
    bool noCollision = false;
    bool surface = false;
    bool hasBoundingBox = false;
    std::array<double, 6> boundingBox = {};
    std::vector<std::string> scripts;
    std::vector<std::string> variableLists;
    std::vector<std::string> stringVariableLists;
    std::vector<std::string> constantFiles;
    ConfigurationDiagnostics diagnostics;
};

SceneryObjectConfig loadSceneryObjectFile(const std::filesystem::path& configPath);

ModelConfig loadSceneryObjectConfig(const std::filesystem::path& configPath,
                                    const std::filesystem::path& modelRoot, Variables& variables);
