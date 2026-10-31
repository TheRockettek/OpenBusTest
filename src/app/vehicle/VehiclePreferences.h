#pragma once

#include <compare>
#include <filesystem>
#include <map>
#include <string>

namespace openbus::app {

struct VehicleRegistrationPreference {
    std::string vehicleNumber;
    std::string registration;

    bool operator==(const VehicleRegistrationPreference&) const = default;
};

using VehicleRegistrationPreferences = std::map<std::string, VehicleRegistrationPreference>;

struct VehicleOdometerKey {
    std::string configurationPath;
    std::string vehicleNumber;
    std::string registration;

    auto operator<=>(const VehicleOdometerKey&) const = default;
};

using VehicleOdometerPreferences = std::map<VehicleOdometerKey, double>;

VehicleRegistrationPreferences
loadVehicleRegistrationPreferences(const std::filesystem::path& path);
void saveVehicleRegistrationPreferences(const std::filesystem::path& path,
                                        const VehicleRegistrationPreferences& preferences);
std::string vehicleRegistrationPreferenceKey(const std::filesystem::path& configurationPath);
VehicleOdometerPreferences loadVehicleOdometerPreferences(const std::filesystem::path& path);
void saveVehicleOdometerPreferences(const std::filesystem::path& path,
                                    const VehicleOdometerPreferences& preferences);
VehicleOdometerKey vehicleOdometerPreferenceKey(const std::filesystem::path& configurationPath,
                                                std::string vehicleNumber,
                                                std::string registration);

} // namespace openbus::app
