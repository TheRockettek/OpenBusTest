#pragma once

#include "core/BusTypes.h"
#include "config/ConfigurationTypes.h"
#include "config/ModelConfigTypes.h"

#include <array>
#include <cstddef>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
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
    // Reflection cameras re-aim as the player's eye moves unless authored with a *_static keyword.
    bool followsViewer = true;
};

struct VehicleViewEntry {
    int viewIndex = -1;
    // Empty for the parameterless marker on a driver camera.
    std::string label;
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

struct PassengerTicketSaleMachine {
    int index = 0;
    std::array<double, 3> position = {};
};

struct PassengerCabinStamper {
    int index = 0;
    std::array<double, 3> position = {};
};

struct PassengerTicketSalePoint {
    std::array<double, 3> position = {};
    std::array<double, 2> size = {};
    std::string doorName;
};

struct PassengerPathPoint {
    int index = -1;
    std::array<double, 3> position = {};
};

struct PassengerPathLink {
    int from = -1;
    int to = -1;
    bool oneWay = false;
};

struct VehicleCouplingCharacter {
    double alphaDegrees = 0.0;
    double minimumBetaDegrees = 0.0;
    double maximumBetaDegrees = 0.0;
    int type = 0;
};

struct VehicleCouplingReference {
    std::filesystem::path vehicleConfigPath;
    bool flag = false;
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
    std::vector<VehicleViewEntry> scheduleViews;
    std::vector<VehicleViewEntry> ticketSellingViews;
    int standardDriverCamera = -1;
    std::array<double, 3> outsideCameraCenter = {};
    bool hasOutsideCameraCenter = false;
    std::vector<BusAxle> axles;
    double massTonnes = 0.0;
    // Optional constant rolling-resistance force from BUS [rollwiderstand], in newtons.
    std::optional<double> rollingResistanceForceNewtons;
    double centerOfGravityHeight = 0.0;
    bool hasCenterOfGravityHeight = false;
    std::optional<std::array<double, 3>> centerOfGravity;
    std::optional<double> longitudinalRotationPoint;
    double aiDeltaHeight = 0.0;
    bool hasAiDeltaHeight = false;
    double inverseMinimumTurnRadius = 0.0;
    bool hasInverseMinimumTurnRadius = false;
    std::optional<std::array<double, 3>> frontCouplingPoint;
    std::optional<std::array<double, 3>> rearCouplingPoint;
    std::optional<VehicleCouplingCharacter> frontCouplingCharacter;
    std::optional<VehicleCouplingReference> rearCouplingReference;
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
    std::optional<int> passengerPathPointToNextVehicle;
    std::optional<int> passengerPathPointToPreviousVehicle;
    bool passengerCabinNoTicketSale = false;
    std::vector<PassengerCabinStamper> passengerCabinStampers;
    std::vector<PassengerTicketSaleMachine> passengerTicketSaleMachines;
    std::optional<PassengerTicketSalePoint> passengerTicketMoneyPoint;
    std::optional<PassengerTicketSalePoint> passengerTicketChangePoint;
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

struct VehicleRegistrationSelection {
    std::optional<std::string> vehicleNumber;
    std::optional<std::string> registration;
};

VehicleConfig loadVehicleConfig(const std::filesystem::path& configPath, VehicleFileKind kind,
                                bool convertScripts = true);
// Loads passenger positions from a cabin file into the supplied configuration.
void loadPassengerCabinConfiguration(VehicleConfig& configuration);
// Loads a configuration's referenced variable/constant files and prepares its OSC script paths.
void prepareScriptConfiguration(VehicleConfig& configuration, bool convertScripts = true);
std::vector<int> passengerPathRoute(const std::vector<PassengerPathPoint>& points,
                                    const std::vector<PassengerPathLink>& links, int fromPoint,
                                    int toPoint);
std::vector<std::pair<double, std::optional<std::size_t>>>
passengerPathLinkProgressProfile(const std::vector<std::array<double, 3>>& path,
                                 const std::vector<std::optional<std::size_t>>& segmentLinks);
std::optional<std::size_t>
passengerPathLinkAtProgress(const std::vector<std::array<double, 3>>& path,
                            const std::vector<std::optional<std::size_t>>& segmentLinks,
                            double progress);
std::size_t registrationOptionCount(const VehicleConfig& configuration);
// Selects the parallel number/plate entry, wrapping within their paired range.
void selectRegistrationAtIndex(VehicleConfig& configuration, std::size_t index);
VehicleRegistrationSelection vehicleRegistrationSelectionAtIndex(const VehicleConfig& configuration,
                                                                 std::size_t index,
                                                                 std::string_view freeVehicleNumber,
                                                                 std::string_view freeRegistration);
std::optional<std::size_t>
vehicleCameraArrayIndexForDriverView(const std::vector<VehicleCamera>& cameras, int viewIndex);
std::filesystem::path soundConfigPathForVehicle(const VehicleConfig& configuration,
                                                bool isAiVehicle);
VehiclePlacement placementForVehicle(const VehicleConfig& configuration,
                                     const VehiclePlacement& placement, bool isAiVehicle);

namespace openbus::scripting {
class Vehicle;
}

ModelConfig loadVehicleModelConfig(const std::filesystem::path& configPath,
                                   const std::filesystem::path& modelRoot,
                                   openbus::scripting::Vehicle& variables);
