#include "config/VehicleCatalog.h"

#include "config/ConfigurationParser.h"
#include "core/AssetPath.h"
#include "core/Utf8Path.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <iterator>
#include <limits>
#include <optional>
#include <stdexcept>
#include <system_error>
#include <unordered_set>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

namespace openbus::config {

namespace {

std::string lowerExtension(const std::filesystem::path& path) {
    std::string extension = openbus::pathToUtf8(path.extension());
    std::transform(extension.begin(), extension.end(), extension.begin(),
                   [](unsigned char value) { return static_cast<char>(std::tolower(value)); });
    return extension;
}

std::string genericPathToUtf8(const std::filesystem::path& path) {
    std::string value = openbus::pathToUtf8(path);
    std::replace(value.begin(), value.end(), '\\', '/');
    return value;
}

std::string pathKey(const std::filesystem::path& path) {
    std::string key = genericPathToUtf8(std::filesystem::absolute(path).lexically_normal());
    std::transform(key.begin(), key.end(), key.begin(),
                   [](unsigned char value) { return static_cast<char>(std::tolower(value)); });
    return key;
}

std::filesystem::path pathFromVehicleConfigText(const std::string& value) {
#ifdef _WIN32
    if (value.size() > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
        throw std::length_error("Vehicle configuration path is too long");
    }
    const int byteCount = static_cast<int>(value.size());
    if (value.empty() || MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, value.data(), byteCount,
                                             nullptr, 0) > 0) {
        return openbus::pathFromUtf8(value);
    }

    const int wideCount =
        MultiByteToWideChar(CP_ACP, MB_ERR_INVALID_CHARS, value.data(), byteCount, nullptr, 0);
    if (wideCount == 0) {
        throw std::system_error(static_cast<int>(GetLastError()), std::system_category(),
                                "Could not decode vehicle configuration path");
    }
    std::wstring wideValue(static_cast<std::size_t>(wideCount), L'\0');
    if (MultiByteToWideChar(CP_ACP, MB_ERR_INVALID_CHARS, value.data(), byteCount, wideValue.data(),
                            wideCount) == 0) {
        throw std::system_error(static_cast<int>(GetLastError()), std::system_category(),
                                "Could not decode vehicle configuration path");
    }
    return std::filesystem::path(wideValue);
#else
    return openbus::pathFromUtf8(value);
#endif
}

std::filesystem::path normalizePath(std::string value) {
    std::replace(value.begin(), value.end(), '\\', '/');
    return pathFromVehicleConfigText(value);
}

std::filesystem::path resolveVehicleConfigReference(const std::filesystem::path& configPath,
                                                    std::string referencedPath) {
    std::replace(referencedPath.begin(), referencedPath.end(), '\\', '/');
    return openbus::assets::resolveCaseInsensitivePath(configPath.parent_path() /
                                                       pathFromVehicleConfigText(referencedPath));
}

std::filesystem::path pathForComparison(const std::filesystem::path& path) {
    std::string value = genericPathToUtf8(path);
#ifdef _WIN32
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
#endif
    return openbus::pathFromUtf8(value);
}

std::vector<std::filesystem::path> vehicleSideFiles(const std::filesystem::path& vehicleConfigPath,
                                                    const std::filesystem::path& vehiclesDirectory,
                                                    const std::string& extension) {
    const std::filesystem::path root =
        std::filesystem::absolute(vehiclesDirectory).lexically_normal();
    const std::filesystem::path vehicleDirectory =
        std::filesystem::absolute(vehicleConfigPath).lexically_normal().parent_path();
    const std::filesystem::path relativeDirectory =
        pathForComparison(vehicleDirectory).lexically_relative(pathForComparison(root));
    if (relativeDirectory.empty() || relativeDirectory.is_absolute() ||
        *relativeDirectory.begin() == "..") {
        throw std::invalid_argument("Vehicle configuration is outside the OMSI Vehicles folder: " +
                                    openbus::pathToUtf8(vehicleConfigPath));
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
                                 openbus::pathToUtf8(vehicleDirectory) + "': " + error.message());
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
            throw std::runtime_error("Failed to list vehicle assets under '" +
                                     openbus::pathToUtf8(parent) + "': " + error.message());
        }
        if (pathForComparison(parent) == pathForComparison(root) ||
            parent == parent.parent_path()) {
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
        const std::filesystem::path resolved = openbus::assets::resolveTexturePath(candidate);
        std::error_code error;
        if (std::filesystem::is_regular_file(resolved, error) && !error) {
            return resolved;
        }
    }
    return std::filesystem::absolute(candidates.front()).lexically_normal();
}

std::string resolveCtiTextureUtf8(const std::filesystem::path& ctiPath,
                                  const std::string& textureName) {
    try {
        return genericPathToUtf8(resolveCtiTexture(ctiPath, textureName));
    } catch (const std::exception& error) {
        throw std::runtime_error("Could not resolve CTI replacement texture '" + textureName +
                                 "' in '" + openbus::pathToUtf8(ctiPath) + "': " + error.what());
    }
}

bool pathIsWithin(const std::filesystem::path& path, const std::filesystem::path& directory) {
    std::string pathValue = pathKey(path);
    std::string directoryValue = pathKey(directory);
    if (directoryValue.empty()) {
        return false;
    }
    if (directoryValue.back() != '/') {
        directoryValue.push_back('/');
    }
    return pathValue.size() > directoryValue.size() &&
           pathValue.compare(0, directoryValue.size(), directoryValue) == 0;
}

std::string hofName(const std::filesystem::path& path) {
    Reader reader(path);
    if (!reader.isOpen()) {
        throw std::runtime_error("Could not open depot file: " + openbus::pathToUtf8(path));
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
    return openbus::pathToUtf8(path.stem());
}

} // namespace

std::vector<VehicleCatalogEntry>
discoverVehicleCatalog(const std::filesystem::path& vehiclesDirectory) {
    std::error_code error;
    if (!std::filesystem::is_directory(vehiclesDirectory, error) || error) {
        throw std::runtime_error("OMSI Vehicles directory is unavailable: " +
                                 openbus::pathToUtf8(vehiclesDirectory));
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
        vehicles.push_back(
            {path, "(Unknown manufacturer)", openbus::pathToUtf8(path.stem()), "Default"});
        VehicleCatalogEntry& vehicle = vehicles.back();
        Reader reader(path);
        if (!reader.isOpen()) {
            throw std::runtime_error("Could not open vehicle configuration: " +
                                     openbus::pathToUtf8(path));
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
                                 openbus::pathToUtf8(vehiclesDirectory) + "': " + error.message());
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
                           const std::filesystem::path& vehiclesDirectory,
                           const std::vector<ModelCtcTemplate>& ctcTemplates,
                           std::vector<std::filesystem::path>* sourceFiles) {
    std::vector<VehicleAppearance> appearances;
    std::vector<std::filesystem::path> ctiFiles =
        vehicleSideFiles(vehicleConfigPath, vehiclesDirectory, ".cti");
    if (!ctcTemplates.empty()) {
        std::vector<std::filesystem::path> templateFiles;
        for (const ModelCtcTemplate& ctc : ctcTemplates) {
            const std::filesystem::path templateDirectory =
                resolveVehicleConfigReference(vehicleConfigPath, ctc.texturePath);
            std::error_code directoryError;
            if (!std::filesystem::is_directory(templateDirectory, directoryError) ||
                directoryError) {
                continue;
            }
            for (const std::filesystem::path& ctiPath : ctiFiles) {
                if (pathIsWithin(ctiPath, templateDirectory) &&
                    std::ranges::find(templateFiles, ctiPath) == templateFiles.end()) {
                    templateFiles.push_back(ctiPath);
                }
            }
        }
        if (!templateFiles.empty()) {
            ctiFiles = std::move(templateFiles);
        }
    }
    if (sourceFiles) {
        *sourceFiles = ctiFiles;
    }
    for (const std::filesystem::path& ctiPath : ctiFiles) {
        Reader reader(ctiPath);
        if (!reader.isOpen()) {
            throw std::runtime_error("Could not open appearance configuration: " +
                                     openbus::pathToUtf8(ctiPath));
        }
        std::optional<std::size_t> currentAppearance;
        Line line;
        while (reader.next(line)) {
            if (!line.isKeyword()) {
                continue;
            }
            const std::string keyword = lower(line.keyword());
            if (keyword == "item") {
                std::vector<std::string> values;
                ConfigurationDiagnostics diagnostics;
                if (!reader.readPayloads(3, values, diagnostics, keyword)) {
                    currentAppearance.reset();
                    continue;
                }
                const std::string name = trim(values[0]);
                const std::string slot = lower(trim(values[1]));
                const std::string texture = trim(values[2]);
                if (name.empty() || slot.empty() || texture.empty()) {
                    currentAppearance.reset();
                    continue;
                }
                auto appearance = std::find_if(appearances.begin(), appearances.end(),
                                               [&](const VehicleAppearance& candidate) {
                                                   return lower(candidate.name) == lower(name);
                                               });
                if (appearance == appearances.end()) {
                    appearances.push_back({name, ctiPath, {}, {}});
                    currentAppearance = appearances.size() - 1;
                } else {
                    currentAppearance =
                        static_cast<std::size_t>(std::distance(appearances.begin(), appearance));
                }
                appearances[*currentAppearance].textureReplacements[slot] =
                    resolveCtiTextureUtf8(ctiPath, texture);
                continue;
            }
            if (keyword != "setvar") {
                continue;
            }
            std::vector<std::string> values;
            ConfigurationDiagnostics diagnostics;
            if (!reader.readPayloads(2, values, diagnostics, keyword)) {
                throw std::runtime_error("Invalid CTI [setvar] payload in '" +
                                         openbus::pathToUtf8(ctiPath) + "' at line " +
                                         std::to_string(line.number));
            }
            if (!currentAppearance) {
                throw std::runtime_error("CTI [setvar] has no preceding [item] in '" +
                                         openbus::pathToUtf8(ctiPath) + "' at line " +
                                         std::to_string(line.number));
            }
            const std::string variable = trim(values[0]);
            float value = 0.0F;
            if (variable.empty() || !parseFloat(values[1], value)) {
                throw std::runtime_error("Invalid CTI [setvar] in '" +
                                         openbus::pathToUtf8(ctiPath) + "' at line " +
                                         std::to_string(line.number));
            }
            appearances[*currentAppearance].variableReplacements[variable] = value;
        }
    }
    std::sort(appearances.begin(), appearances.end(),
              [](const VehicleAppearance& left, const VehicleAppearance& right) {
                  return lower(left.name) < lower(right.name);
              });
    return appearances;
}

const VehicleAppearance* findVehicleAppearance(const std::vector<VehicleAppearance>& appearances,
                                               const std::string& name) {
    const std::string requested = lower(trim(name));
    const auto appearance = std::find_if(
        appearances.begin(), appearances.end(),
        [&](const VehicleAppearance& candidate) { return lower(candidate.name) == requested; });
    return appearance == appearances.end() ? nullptr : &*appearance;
}

std::vector<VehicleDepot> discoverVehicleDepots(const std::filesystem::path& vehicleConfigPath,
                                                const std::filesystem::path& vehiclesDirectory) {
    std::vector<VehicleDepot> depots;
    for (const std::filesystem::path& path :
         discoverVehicleDepotSourceFiles(vehicleConfigPath, vehiclesDirectory)) {
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

std::vector<std::filesystem::path>
discoverVehicleDepotSourceFiles(const std::filesystem::path& vehicleConfigPath,
                                const std::filesystem::path& vehiclesDirectory) {
    return vehicleSideFiles(vehicleConfigPath, vehiclesDirectory, ".hof");
}

std::vector<VehicleDepot> discoverAllVehicleDepots(const std::filesystem::path& vehiclesDirectory) {
    std::error_code error;
    if (!std::filesystem::is_directory(vehiclesDirectory, error) || error) {
        throw std::runtime_error("OMSI Vehicles directory is unavailable: " +
                                 openbus::pathToUtf8(vehiclesDirectory));
    }

    std::vector<VehicleDepot> depots;
    std::filesystem::recursive_directory_iterator entry(
        vehiclesDirectory, std::filesystem::directory_options::skip_permission_denied, error);
    const std::filesystem::recursive_directory_iterator end;
    for (; !error && entry != end; entry.increment(error)) {
        std::error_code statusError;
        if (!entry->is_regular_file(statusError) || statusError ||
            lowerExtension(entry->path()) != ".hof") {
            continue;
        }
        depots.push_back({entry->path(), hofName(entry->path())});
    }
    if (error) {
        throw std::runtime_error("Failed to list OMSI Vehicles directory '" +
                                 openbus::pathToUtf8(vehiclesDirectory) + "': " + error.message());
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
