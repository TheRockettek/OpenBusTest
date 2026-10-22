#pragma once

#include "config/VehicleConfigLoader.h"
#include "map/MapTrafficSignal.h"

#include <array>
#include <cstddef>
#include <filesystem>
#include <optional>
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

struct SceneryObjectTrafficPath {
    // OMSI SCO [path] records are local spline segments, not start/end coordinate pairs.
    std::size_t pathIndex = 0;
    std::array<double, 3> startPosition = {};
    double headingDegrees = 0.0;
    double radius = 0.0;
    double length = 0.0;
    double gradientStartPercent = 0.0;
    double gradientEndPercent = 0.0;
    double pathTypeCode = 0.0;
    double widthMeters = 0.0;
    int trafficLightIndex = -1;
    std::optional<openbus::map::MapTrafficSignalSchedule> trafficSignal;
};

struct SceneryObjectTrafficLight {
    std::string name;
    std::vector<openbus::map::MapTrafficSignalPhase> phases;
    std::optional<double> approachDistanceMeters;
};

struct SceneryObjectConfig {
    std::filesystem::path sourcePath;
    std::filesystem::path modelPath;
    std::filesystem::path collisionMesh;
    bool noCollision = false;
    bool surface = false;
    bool onlyEditor = false;
    bool absoluteHeight = false;
    bool helpArrow = false;
    int renderType = 2;
    bool hasBoundingBox = false;
    std::array<double, 6> boundingBox = {};
    std::vector<std::string> scripts;
    std::vector<std::string> variableLists;
    std::vector<std::string> stringVariableLists;
    std::vector<std::string> constantFiles;
    VehicleConfig scriptConfiguration;
    std::vector<SceneryTreeDefinition> trees;
    std::vector<SceneryObjectTrafficPath> trafficPaths;
    double trafficLightsGroupDurationSeconds = 0.0;
    std::vector<SceneryObjectTrafficLight> trafficLights;
    std::vector<std::array<double, 3>> trafficLightStops;
    std::vector<std::array<double, 4>> trafficLightJumps;
    std::vector<ModelAttachmentPoint> attachmentPoints;
    ConfigurationDiagnostics diagnostics;
};

SceneryObjectConfig loadSceneryObjectFile(const std::filesystem::path& configPath);
std::filesystem::path resolveSceneryObjectModelConfigPath(const SceneryObjectConfig& configuration);

ModelConfig loadSceneryObjectConfig(const std::filesystem::path& configPath,
                                    const std::filesystem::path& modelRoot,
                                    openbus::scripting::SceneryObject& variables);
