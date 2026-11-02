#include "app/vehicle/VehiclePreferences.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <utility>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

namespace openbus::app {
namespace {

constexpr const char* FILE_HEADER = "OpenBus vehicle preferences 1";
constexpr const char* ODOMETER_FILE_HEADER = "OpenBus vehicle odometers 3";
constexpr const char* MODEL_ODOMETER_FILE_HEADER = "OpenBus vehicle odometers 2";
constexpr const char* LEGACY_ODOMETER_FILE_HEADER = "OpenBus vehicle odometers 1";

void validateOdometerMetres(double metres) {
    if (!std::isfinite(metres) || metres < 0.0) {
        throw std::invalid_argument("Vehicle odometer must be finite and non-negative");
    }
}

double odometerBaseMetres(VehicleOdometerMode mode, double initialKilometres) {
    switch (mode) {
    case VehicleOdometerMode::Global:
    case VehicleOdometerMode::Vehicle:
        return 0.0;
    case VehicleOdometerMode::VehicleDefault: {
        const double metres = initialKilometres * 1000.0;
        validateOdometerMetres(metres);
        return metres;
    }
    }
    throw std::invalid_argument("Invalid vehicle odometer mode");
}

struct TemporaryFile {
    std::filesystem::path path;
    bool committed = false;

    ~TemporaryFile() {
        if (!committed) {
            std::error_code ignored;
            std::filesystem::remove(path, ignored);
        }
    }
};

template <typename WriteContent>
void writeAtomically(const std::filesystem::path& path, WriteContent&& writeContent) {
    static std::atomic<std::uint64_t> sequence = 0;
    std::filesystem::path temporaryPath = path;
    temporaryPath += ".tmp-" +
                     std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) +
                     "-" + std::to_string(sequence.fetch_add(1, std::memory_order_relaxed));
    TemporaryFile temporary{temporaryPath};

    std::ofstream output(temporaryPath, std::ios::binary | std::ios::trunc);
    if (!output) {
        throw std::runtime_error("Unable to write temporary preferences file: " +
                                 temporaryPath.string());
    }
    writeContent(output);
    output.flush();
    if (!output) {
        throw std::runtime_error("Failed while writing temporary preferences file: " +
                                 temporaryPath.string());
    }
    output.close();
    if (!output) {
        throw std::runtime_error("Failed to close temporary preferences file: " +
                                 temporaryPath.string());
    }

#ifdef _WIN32
    if (MoveFileExW(temporaryPath.c_str(), path.c_str(),
                    MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH) == 0) {
        throw std::system_error(static_cast<int>(GetLastError()), std::system_category(),
                                "Unable to replace preferences file " + path.string());
    }
#else
    std::error_code error;
    std::filesystem::rename(temporaryPath, path, error);
    if (error) {
        throw std::system_error(error, "Unable to replace preferences file " + path.string());
    }
#endif
    temporary.committed = true;
}

} // namespace

VehicleRegistrationPreferences
loadVehicleRegistrationPreferences(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::binary);
    if (!input) {
        if (!std::filesystem::exists(path)) {
            return {};
        }
        throw std::runtime_error("Unable to open vehicle preferences: " + path.string());
    }

    std::string line;
    if (!std::getline(input, line) || line != FILE_HEADER) {
        throw std::runtime_error("Invalid vehicle preferences header: " + path.string());
    }

    VehicleRegistrationPreferences preferences;
    std::size_t lineNumber = 1;
    while (std::getline(input, line)) {
        ++lineNumber;
        if (line.empty()) {
            continue;
        }
        std::istringstream record(line);
        std::string key;
        VehicleRegistrationPreference preference;
        std::string extra;
        if (!(record >> std::quoted(key) >> std::quoted(preference.vehicleNumber) >>
              std::quoted(preference.registration)) ||
            (record >> extra) || key.empty()) {
            throw std::runtime_error("Invalid vehicle preference at " + path.string() + ":" +
                                     std::to_string(lineNumber));
        }
        preferences[std::move(key)] = std::move(preference);
    }
    if (input.bad()) {
        throw std::runtime_error("Failed while reading vehicle preferences: " + path.string());
    }
    return preferences;
}

void saveVehicleRegistrationPreferences(const std::filesystem::path& path,
                                        const VehicleRegistrationPreferences& preferences) {
    writeAtomically(path, [&](std::ostream& output) {
        output << FILE_HEADER << '\n';
        for (const auto& [key, preference] : preferences) {
            output << std::quoted(key) << ' ' << std::quoted(preference.vehicleNumber) << ' '
                   << std::quoted(preference.registration) << '\n';
        }
    });
}

std::string vehicleRegistrationPreferenceKey(const std::filesystem::path& configurationPath) {
    return std::filesystem::absolute(configurationPath).lexically_normal().generic_string();
}

VehicleOdometerMode parseVehicleOdometerMode(std::string_view value) {
    if (value == "global") {
        return VehicleOdometerMode::Global;
    }
    if (value == "vehicle") {
        return VehicleOdometerMode::Vehicle;
    }
    if (value == "vehicle-default") {
        return VehicleOdometerMode::VehicleDefault;
    }
    throw std::invalid_argument(
        "OPENBUS_ODOMETER_MODE must be global, vehicle, or vehicle-default; got: " +
        std::string(value));
}

double vehicleOdometerReading(const VehicleOdometerPreferences& preferences,
                              VehicleOdometerMode mode,
                              const std::filesystem::path& configurationPath,
                              double initialKilometres) {
    const double base = odometerBaseMetres(mode, initialKilometres);
    if (mode == VehicleOdometerMode::Global) {
        validateOdometerMetres(preferences.globalMetres);
        return preferences.globalMetres;
    }
    const auto saved = preferences.modelMetres.find(vehicleOdometerPreferenceKey(configurationPath));
    const double raw = saved == preferences.modelMetres.end() ? 0.0 : saved->second;
    validateOdometerMetres(raw);
    const double reading = base + raw;
    validateOdometerMetres(reading);
    return reading;
}

void setVehicleOdometerReading(VehicleOdometerPreferences& preferences, VehicleOdometerMode mode,
                               const std::filesystem::path& configurationPath,
                               double initialKilometres, double readingMetres) {
    validateOdometerMetres(readingMetres);
    const double raw = readingMetres - odometerBaseMetres(mode, initialKilometres);
    validateOdometerMetres(raw);
    if (mode == VehicleOdometerMode::Global) {
        preferences.globalMetres = raw;
    } else {
        preferences.modelMetres[vehicleOdometerPreferenceKey(configurationPath)] = raw;
    }
}

VehicleOdometerPreferences loadVehicleOdometerPreferences(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::binary);
    if (!input) {
        if (!std::filesystem::exists(path)) {
            return {};
        }
        throw std::runtime_error("Unable to open vehicle odometers: " + path.string());
    }

    std::string line;
    std::getline(input, line);
    if (!line.empty() && line.back() == '\r') {
        line.pop_back();
    }
    if (!input || (line != ODOMETER_FILE_HEADER && line != MODEL_ODOMETER_FILE_HEADER &&
                   line != LEGACY_ODOMETER_FILE_HEADER)) {
        throw std::runtime_error("Invalid vehicle odometers header: " + path.string());
    }
    const bool legacyFormat = line == LEGACY_ODOMETER_FILE_HEADER;
    const bool hasGlobalCounter = line == ODOMETER_FILE_HEADER;

    VehicleOdometerPreferences preferences;
    std::size_t lineNumber = 1;
    if (hasGlobalCounter) {
        ++lineNumber;
        std::getline(input, line);
        std::istringstream record(line);
        std::string tag;
        std::string extra;
        if (!input || !(record >> tag >> preferences.globalMetres) || tag != "global" ||
            (record >> extra) || !std::isfinite(preferences.globalMetres) ||
            preferences.globalMetres < 0.0) {
            throw std::runtime_error("Invalid global vehicle odometer at " + path.string() + ":2");
        }
    }
    while (std::getline(input, line)) {
        ++lineNumber;
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }
        if (line.empty()) {
            continue;
        }
        std::istringstream record(line);
        std::string key;
        double metres = 0.0;
        std::string extra;
        record >> std::quoted(key);
        if (legacyFormat) {
            std::string vehicleNumber;
            std::string registration;
            record >> std::quoted(vehicleNumber) >> std::quoted(registration);
        }
        if (!(record >> metres) || (record >> extra) || key.empty() ||
            !std::isfinite(metres) || metres < 0.0) {
            throw std::runtime_error("Invalid vehicle odometer at " + path.string() + ":" +
                                     std::to_string(lineNumber));
        }
        // Per-vehicle mileage included authored initial values; shared model counters start at zero.
        preferences.modelMetres[std::move(key)] = legacyFormat ? 0.0 : metres;
    }
    if (input.bad()) {
        throw std::runtime_error("Failed while reading vehicle odometers: " + path.string());
    }
    return preferences;
}

void saveVehicleOdometerPreferences(const std::filesystem::path& path,
                                    const VehicleOdometerPreferences& preferences) {
    validateOdometerMetres(preferences.globalMetres);
    for (const auto& entry : preferences.modelMetres) {
        if (entry.first.empty()) {
            throw std::invalid_argument("Vehicle odometer configuration path must not be empty");
        }
        validateOdometerMetres(entry.second);
    }

    writeAtomically(path, [&](std::ostream& output) {
        output << ODOMETER_FILE_HEADER << '\n' << std::setprecision(17)
               << "global " << preferences.globalMetres << '\n';
        for (const auto& [key, metres] : preferences.modelMetres) {
            output << std::quoted(key) << ' ' << metres << '\n';
        }
    });
}

std::string vehicleOdometerPreferenceKey(const std::filesystem::path& configurationPath) {
    if (configurationPath.empty()) {
        throw std::invalid_argument("Vehicle odometer configuration path must not be empty");
    }
    return vehicleRegistrationPreferenceKey(configurationPath);
}

} // namespace openbus::app
