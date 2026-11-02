#pragma once

#include <filesystem>
#include <map>
#include <string>
#include <string_view>

namespace openbus::app {

struct VehicleRegistrationPreference {
    std::string vehicleNumber;
    std::string registration;

    bool operator==(const VehicleRegistrationPreference&) const = default;
};

using VehicleRegistrationPreferences = std::map<std::string, VehicleRegistrationPreference>;

enum class VehicleOdometerMode { Global, Vehicle, VehicleDefault };

struct VehicleOdometerPreferences {
    double globalMetres = 0.0;
    std::map<std::string, double> modelMetres;

    bool operator==(const VehicleOdometerPreferences&) const = default;
};

VehicleOdometerMode parseVehicleOdometerMode(std::string_view value = "global");
double vehicleOdometerReading(const VehicleOdometerPreferences& preferences,
                              VehicleOdometerMode mode,
                              const std::filesystem::path& configurationPath,
                              double initialKilometres);
void setVehicleOdometerReading(VehicleOdometerPreferences& preferences, VehicleOdometerMode mode,
                               const std::filesystem::path& configurationPath,
                               double initialKilometres, double readingMetres);

VehicleRegistrationPreferences
loadVehicleRegistrationPreferences(const std::filesystem::path& path);
void saveVehicleRegistrationPreferences(const std::filesystem::path& path,
                                        const VehicleRegistrationPreferences& preferences);
std::string vehicleRegistrationPreferenceKey(const std::filesystem::path& configurationPath);
VehicleOdometerPreferences loadVehicleOdometerPreferences(const std::filesystem::path& path);
void saveVehicleOdometerPreferences(const std::filesystem::path& path,
                                    const VehicleOdometerPreferences& preferences);
std::string vehicleOdometerPreferenceKey(const std::filesystem::path& configurationPath);

} // namespace openbus::app
