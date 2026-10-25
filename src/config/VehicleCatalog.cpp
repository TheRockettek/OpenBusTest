#include "config/VehicleCatalog.h"

#include "config/ConfigurationParser.h"
#include "core/AssetPath.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <iterator>
#include <stdexcept>
#include <system_error>
#include <unordered_set>

namespace openbus::config {

namespace {

std::string lowerExtension(const std::filesystem::path& path) {
    std::string extension = path.extension().string();
    std::transform(extension.begin(), extension.end(), extension.begin(),
                   [](unsigned char value) { return static_cast<char>(std::tolower(value)); });
    return extension;
}

std::string pathKey(const std::filesystem::path& path) {
    std::string key = std::filesystem::absolute(path).lexically_normal().generic_string();
    std::transform(key.begin(), key.end(), key.begin(),
                   [](unsigned char value) { return static_cast<char>(std::tolower(value)); });
    return key;
}

std::filesystem::path normalizePath(std::string value) {
    std::replace(value.begin(), value.end(), '\\', '/');
    return std::filesystem::path(value);
}

std::vector<std::filesystem::path> vehicleSideFiles(const std::filesystem::path& vehicleConfigPath,
                                                    const std::filesystem::path& vehiclesDirectory,
                                                    const std::string& extension) {
    const std::filesystem::path root =
        std::filesystem::absolute(vehiclesDirectory).lexically_normal();
    const std::filesystem::path vehicleDirectory =
        std::filesystem::absolute(vehicleConfigPath).lexically_normal().parent_path();
    const std::filesystem::path relativeDirectory = vehicleDirectory.lexically_relative(root);
    if (relativeDirectory.empty() || relativeDirectory.is_absolute() ||
        *relativeDirectory.begin() == "..") {
        throw std::invalid_argument("Vehicle configuration is outside the OMSI Vehicles folder: " +
                                    vehicleConfigPath.string());
    }

    std::vector<std::filesystem::path> files;
    std::unordered_set<std::string> seen;
    const auto appendFile = [&](const std::filesystem::directory_entry& entry) {
        std::error_code statusError;
        if (entry.is_regular_file(statusError) && !statusError &&
            lowerExtension(entry.path()) == extension &&
            seen.insert(pathKey(entry.path())).second) {
            files.push_back(entry.path());
        }
    };

    std::error_code error;
    std::filesystem::recursive_directory_iterator nested(
        vehicleDirectory, std::filesystem::directory_options::skip_permission_denied, error);
    const std::filesystem::recursive_directory_iterator end;
    for (; !error && nested != end; nested.increment(error)) {
        appendFile(*nested);
    }
    if (error) {
        throw std::runtime_error("Failed to list vehicle assets under '" +
                                 vehicleDirectory.string() + "': " + error.message());
    }

    for (std::filesystem::path parent = vehicleDirectory; !parent.empty();
         parent = parent.parent_path()) {
        std::filesystem::directory_iterator entry(
            parent, std::filesystem::directory_options::skip_permission_denied, error);
        for (const std::filesystem::directory_iterator endEntry; !error && entry != endEntry;
             entry.increment(error)) {
            appendFile(*entry);
        }
        if (error) {
            throw std::runtime_error("Failed to list vehicle assets under '" + parent.string() +
                                     "': " + error.message());
        }
        if (parent == root || parent == parent.parent_path()) {
            break;
        }
    }

    std::sort(files.begin(), files.end());
    return files;
}

std::filesystem::path resolveCtiTexture(const std::filesystem::path& ctiPath,
                                        const std::string& textureName) {
    const std::filesystem::path requested = normalizePath(textureName);
    const std::array<std::filesystem::path, 3> candidates = {
        requested.is_absolute() ? requested : ctiPath.parent_path() / requested,
        requested.is_absolute() ? requested : ctiPath.parent_path().parent_path() / requested,
        requested.is_absolute() ? requested
                                : ctiPath.parent_path().parent_path().parent_path() / requested};
    for (const std::filesystem::path& candidate : candidates) {
        const std::filesystem::path resolved =
            openbus::assets::resolveCaseInsensitivePath(candidate);
        std::error_code error;
        if (std::filesystem::is_regular_file(resolved, error) && !error) {
            return resolved;
        }
    }
    return std::filesystem::absolute(candidates.front()).lexically_normal();
}

std::string hofName(const std::filesystem::path& path) {
    Reader reader(path);
    if (!reader.isOpen()) {
        throw std::runtime_error("Could not open depot file: " + path.string());
    }
    Line line;
    while (reader.next(line)) {
        if (!line.isKeyword() || lower(line.keyword()) != "name") {
            continue;
        }
        Line value;
        ConfigurationDiagnostics diagnostics;
        if (reader.readPayload(value, diagnostics, "name") && !trim(value.text).empty()) {
            return trim(value.text);
        }
        break;
    }
    return path.stem().string();
}

} // namespace

std::vector<VehicleCatalogEntry>
discoverVehicleCatalog(const std::filesystem::path& vehiclesDirectory) {
    std::error_code error;
    if (!std::filesystem::is_directory(vehiclesDirectory, error) || error) {
        throw std::runtime_error("OMSI Vehicles directory is unavailable: " +
                                 vehiclesDirectory.string());
    }

    std::vector<VehicleCatalogEntry> vehicles;
    std::filesystem::recursive_directory_iterator entry(
        vehiclesDirectory, std::filesystem::directory_options::skip_permission_denied, error);
    const std::filesystem::recursive_directory_iterator end;
    for (; !error && entry != end; entry.increment(error)) {
        std::error_code statusError;
        if (!entry->is_regular_file(statusError) || statusError ||
            lowerExtension(entry->path()) != ".bus") {
            continue;
        }
        const std::filesystem::path path = entry->path();
        vehicles.push_back({path, "(Unknown manufacturer)", path.stem().string(), "Default"});
        VehicleCatalogEntry& vehicle = vehicles.back();
        Reader reader(path);
        if (!reader.isOpen()) {
            throw std::runtime_error("Could not open vehicle configuration: " + path.string());
        }
        Line line;
        while (reader.next(line)) {
            if (!line.isKeyword() || lower(line.keyword()) != "friendlyname") {
                continue;
            }
            std::vector<std::string> values;
            ConfigurationDiagnostics diagnostics;
            if (reader.readPayloads(3, values, diagnostics, "friendlyname")) {
                if (!trim(values[0]).empty()) {
                    vehicle.manufacturer = trim(values[0]);
                }
                if (!trim(values[1]).empty()) {
                    vehicle.type = trim(values[1]);
                }
                if (!trim(values[2]).empty()) {
                    vehicle.defaultAppearance = trim(values[2]);
                }
            }
            break;
        }
    }
    if (error) {
        throw std::runtime_error("Failed to list OMSI Vehicles directory '" +
                                 vehiclesDirectory.string() + "': " + error.message());
    }

    std::sort(vehicles.begin(), vehicles.end(),
              [](const VehicleCatalogEntry& left, const VehicleCatalogEntry& right) {
                  const std::string leftManufacturer = lower(left.manufacturer);
                  const std::string rightManufacturer = lower(right.manufacturer);
                  if (leftManufacturer != rightManufacturer) {
                      return leftManufacturer < rightManufacturer;
                  }
                  const std::string leftType = lower(left.type);
                  const std::string rightType = lower(right.type);
                  if (leftType != rightType) {
                      return leftType < rightType;
                  }
                  return left.configurationPath < right.configurationPath;
              });
    return vehicles;
}

std::vector<VehicleAppearance>
discoverVehicleAppearances(const std::filesystem::path& vehicleConfigPath,
                           const std::filesystem::path& vehiclesDirectory) {
    std::vector<VehicleAppearance> appearances;
    for (const std::filesystem::path& ctiPath :
         vehicleSideFiles(vehicleConfigPath, vehiclesDirectory, ".cti")) {
        Reader reader(ctiPath);
        if (!reader.isOpen()) {
            throw std::runtime_error("Could not open appearance configuration: " +
                                     ctiPath.string());
        }
        Line line;
        while (reader.next(line)) {
            if (!line.isKeyword() || lower(line.keyword()) != "item") {
                continue;
            }
            std::vector<std::string> values;
            ConfigurationDiagnostics diagnostics;
            if (!reader.readPayloads(3, values, diagnostics, "item")) {
                continue;
            }
            const std::string name = trim(values[0]);
            const std::string slot = lower(trim(values[1]));
            const std::string texture = trim(values[2]);
            if (name.empty() || slot.empty() || texture.empty()) {
                continue;
            }
            auto appearance = std::find_if(appearances.begin(), appearances.end(),
                                           [&](const VehicleAppearance& candidate) {
                                               return lower(candidate.name) == lower(name);
                                           });
            if (appearance == appearances.end()) {
                appearances.push_back({name, ctiPath, {}});
                appearance = std::prev(appearances.end());
            }
            appearance->textureReplacements[slot] =
                resolveCtiTexture(ctiPath, texture).generic_string();
        }
    }
    std::sort(appearances.begin(), appearances.end(),
              [](const VehicleAppearance& left, const VehicleAppearance& right) {
                  return lower(left.name) < lower(right.name);
              });
    return appearances;
}

std::vector<VehicleDepot> discoverVehicleDepots(const std::filesystem::path& vehicleConfigPath,
                                                const std::filesystem::path& vehiclesDirectory) {
    std::vector<VehicleDepot> depots;
    for (const std::filesystem::path& path :
         vehicleSideFiles(vehicleConfigPath, vehiclesDirectory, ".hof")) {
        depots.push_back({path, hofName(path)});
    }
    std::sort(depots.begin(), depots.end(),
              [](const VehicleDepot& left, const VehicleDepot& right) {
                  const std::string leftName = lower(left.name);
                  const std::string rightName = lower(right.name);
                  return leftName == rightName ? left.path < right.path : leftName < rightName;
              });
    return depots;
}

} // namespace openbus::config
