#pragma once

#include "BusTypes.h"
#include "ConfigurationTypes.h"
#include "ModelConfigTypes.h"

#include <array>
#include <filesystem>
#include <string>
#include <vector>

enum class VehicleFileKind { Vehicle, Bus };

enum class VehicleCameraKind { Driver, Passenger, Reflexion, Reflexion2 };

struct VehicleCamera {
    VehicleCameraKind kind = VehicleCameraKind::Driver;
    std::array<double, 3> position = {};
    double orbitDistance = 0.0;
    double fieldOfView = 0.0;
    double pan = 0.0;
    double tilt = 0.0;
    double culling = 0.0;
};

struct VehicleConfig {
    VehicleFileKind kind = VehicleFileKind::Vehicle;
    std::filesystem::path sourcePath;
    std::filesystem::path modelPath;
    std::vector<VehicleCamera> cameras;
    int standardDriverCamera = -1;
    std::array<double, 3> outsideCameraCenter = {};
    bool hasOutsideCameraCenter = false;
    std::vector<BusAxle> axles;
    double massTonnes = 0.0;
    double centerOfGravityHeight = 0.0;
    bool hasCenterOfGravityHeight = false;
    double wheelHalfWidth = 0.0;
    bool hasWheelHalfWidth = false;
    bool articulated = false;
    bool hasArticulated = false;
    std::array<double, 3> momentOfInertia = {};
    std::array<double, 6> boundingBox = {};
    bool hasBoundingBox = false;
    std::vector<std::string> scripts;
    std::vector<std::string> variableLists;
    std::vector<std::string> stringVariableLists;
    std::vector<std::string> constantFiles;
    ConfigurationDiagnostics diagnostics;
};

VehicleConfig loadVehicleConfig(const std::filesystem::path& configPath, VehicleFileKind kind);

class Variables;

ModelConfig loadVehicleModelConfig(const std::filesystem::path& configPath,
                                   const std::filesystem::path& modelRoot, Variables& variables);
