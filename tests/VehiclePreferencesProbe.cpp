#include "app/vehicle/VehiclePreferences.h"
#include "ProbeTempDirectory.h"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>

int main() {
    const std::filesystem::path root =
        openbus::probe::makeProbeDirectory("openbus_vehicle_preferences_probe");
    const std::filesystem::path preferencesPath = root / "vehicle-preferences.txt";
    if (!openbus::app::loadVehicleRegistrationPreferences(preferencesPath).empty()) {
        std::cerr << "A missing preferences file should load as empty\n";
        return 1;
    }

    const openbus::app::VehicleRegistrationPreferences expected = {
        {"/vehicles/Bus with spaces/MAN.bus", {"fleet \"12\"", "AB\\123"}},
        {"/vehicles/another.bus", {"", "CUSTOM"}},
    };
    openbus::app::saveVehicleRegistrationPreferences(preferencesPath, expected);
    if (openbus::app::loadVehicleRegistrationPreferences(preferencesPath) != expected) {
        std::cerr << "Vehicle preferences did not round-trip quoted values\n";
        return 1;
    }
    auto updatedPreferences = expected;
    updatedPreferences["/vehicles/Bus with spaces/MAN.bus"].registration = "updated";
    openbus::app::saveVehicleRegistrationPreferences(preferencesPath, updatedPreferences);
    if (openbus::app::loadVehicleRegistrationPreferences(preferencesPath) != updatedPreferences) {
        std::cerr << "Vehicle preferences did not atomically replace an existing file\n";
        return 1;
    }
    const auto odometerKey = openbus::app::vehicleOdometerPreferenceKey(
        root / "Bus.bus", "fleet 12", "AB123");
    const openbus::app::VehicleOdometerPreferences expectedOdometers = {
        {odometerKey, 123456.789},
        {openbus::app::vehicleOdometerPreferenceKey(root / "Other.bus", "", "CUSTOM"), 0.0},
    };
    const std::filesystem::path odometersPath = root / "vehicle-odometers.txt";
    openbus::app::saveVehicleOdometerPreferences(odometersPath, expectedOdometers);
    if (openbus::app::loadVehicleOdometerPreferences(odometersPath) != expectedOdometers) {
        std::cerr << "Vehicle odometers did not round-trip per vehicle identity\n";
        return 1;
    }
    auto updatedOdometers = expectedOdometers;
    updatedOdometers[odometerKey] = 123999.5;
    openbus::app::saveVehicleOdometerPreferences(odometersPath, updatedOdometers);
    if (openbus::app::loadVehicleOdometerPreferences(odometersPath) != updatedOdometers) {
        std::cerr << "Vehicle odometers did not atomically replace an existing file\n";
        return 1;
    }
    bool negativeOdometerRejected = false;
    try {
        openbus::app::saveVehicleOdometerPreferences(odometersPath, {{odometerKey, -1.0}});
    } catch (const std::invalid_argument&) {
        negativeOdometerRejected = true;
    }
    if (!negativeOdometerRejected ||
        openbus::app::loadVehicleOdometerPreferences(odometersPath) != updatedOdometers) {
        std::cerr << "Invalid odometer value was accepted or replaced valid saved state\n";
        return 1;
    }

    const std::filesystem::path malformedPath = root / "malformed.txt";
    {
        std::ofstream malformed(malformedPath);
        malformed << "wrong header\n";
    }
    try {
        (void)openbus::app::loadVehicleRegistrationPreferences(malformedPath);
    } catch (const std::runtime_error&) {
        return 0;
    }
    std::cerr << "Malformed vehicle preferences were not rejected\n";
    return 1;
}
