#pragma once

#include <filesystem>
#include <string>
#include <unordered_map>
#include <vector>

namespace openbus::config {

struct VehicleCatalogEntry {
    std::filesystem::path configurationPath;
    std::string manufacturer;
    std::string type;
    std::string defaultAppearance;
};

struct VehicleAppearance {
    std::string name;
    std::filesystem::path sourcePath;
    std::unordered_map<std::string, std::string> textureReplacements;
};

struct VehicleDepot {
    std::filesystem::path path;
    std::string name;
};

std::vector<VehicleCatalogEntry>
discoverVehicleCatalog(const std::filesystem::path& vehiclesDirectory);
std::vector<VehicleAppearance>
discoverVehicleAppearances(const std::filesystem::path& vehicleConfigPath,
                           const std::filesystem::path& vehiclesDirectory);
std::vector<VehicleDepot> discoverVehicleDepots(const std::filesystem::path& vehicleConfigPath,
                                                const std::filesystem::path& vehiclesDirectory);
std::vector<VehicleDepot>
discoverAllVehicleDepots(const std::filesystem::path& vehiclesDirectory);

} // namespace openbus::config
