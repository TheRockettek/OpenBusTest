#pragma once

#include "BusTypes.h"
#include "ConfigurationTypes.h"
#include "ModelConfigTypes.h"

#include <array>
#include <cstddef>
#include <filesystem>
#include <optional>
#include <string>
#include <unordered_map>
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
};

struct ConstantCurvePoint {
    float x = 0.0F;
    float y = 0.0F;
};

struct ConstantCurve {
    std::string name;
    std::vector<ConstantCurvePoint> points;
};

struct PassengerCabinPosition {
    std::array<double, 3> position = {};
    double seatHeight = 0.0;
    double rotationDegrees = 0.0;
    bool driver = false;
    std::array<int, 4> interiorLightIndexes = {-1, -1, -1, -1};
};

struct PassengerPathPoint {
    int index = -1;
    std::array<double, 3> position = {};
};

struct PassengerPathLink {
    int from = -1;
    int to = -1;
};

struct VehicleConfig {
    VehicleFileKind kind = VehicleFileKind::Vehicle;
    std::filesystem::path sourcePath;
    std::string friendlyManufacturer;
    std::string friendlyVehicleName;
    std::string friendlyDefaultPaint;
    std::string description;
    std::optional<int> vehicleType;
    std::filesystem::path modelPath;
    std::vector<VehicleCamera> cameras;
    int standardDriverCamera = -1;
    std::array<double, 3> outsideCameraCenter = {};
    bool hasOutsideCameraCenter = false;
    std::vector<BusAxle> axles;
    double massTonnes = 0.0;
    double centerOfGravityHeight = 0.0;
    bool hasCenterOfGravityHeight = false;
    double aiDeltaHeight = 0.0;
    bool hasAiDeltaHeight = false;
    double inverseMinimumTurnRadius = 0.0;
    bool hasInverseMinimumTurnRadius = false;
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
    std::filesystem::path soundConfigPath;
    std::filesystem::path soundAiConfigPath;
    std::filesystem::path pathsConfigPath;
    std::filesystem::path passengerCabinConfigPath;
    bool passengerCabinLoaded = false;
    std::vector<PassengerCabinPosition> passengerPositions;
    std::optional<PassengerCabinPosition> driverPosition;
    std::vector<int> passengerEntryPathPoints;
    std::vector<int> passengerExitPathPoints;
    bool passengerPathsLoaded = false;
    std::vector<PassengerPathPoint> passengerPathPoints;
    std::vector<PassengerPathLink> passengerPathLinks;
    std::vector<std::vector<std::string>> passengerStepSoundPacks;
    int passengerPathNextStepSound = -1;
    std::vector<double> passengerPathNextRoomHeights;
    std::filesystem::path numberConfigPath;
    std::filesystem::path registrationListConfigPath;
    bool registrationListsLoaded = false;
    // The [number] fleet identifiers and [registration_list] plate strings
    // are parallel, order-preserving lists.
    std::vector<std::string> vehicleNumbers;
    std::vector<std::string> registrationNumbers;
    std::size_t selectedRegistrationIndex = 0;
    std::string selectedVehicleNumber;
    std::string selectedRegistration;
    bool registrationAutomatic = false;
    std::string registrationPrefix;
    bool registrationFree = false;
    int odometerInitialYear = 0;
    double odometerInitialKilometres = 0.0;
    bool hasOdometerInitial = false;
    std::vector<std::string> floatVariables;
    std::vector<std::string> stringVariables;
    std::unordered_map<std::string, float> constants;
    std::unordered_map<std::string, ConstantCurve> curves;
    ConfigurationDiagnostics diagnostics;
};

VehicleConfig loadVehicleConfig(const std::filesystem::path& configPath, VehicleFileKind kind,
                                bool convertScripts = true);
// Loads a configuration's referenced variable/constant files and prepares its OSC script paths.
void prepareScriptConfiguration(VehicleConfig& configuration, bool convertScripts = true);
std::size_t registrationOptionCount(const VehicleConfig& configuration);
// Selects the parallel number/plate entry, wrapping within their paired range.
void selectRegistrationAtIndex(VehicleConfig& configuration, std::size_t index);

namespace openbus::scripting {
class Vehicle;
}

ModelConfig loadVehicleModelConfig(const std::filesystem::path& configPath,
                                   const std::filesystem::path& modelRoot,
                                   openbus::scripting::Vehicle& variables);
