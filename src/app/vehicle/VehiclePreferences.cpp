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
constexpr const char* ODOMETER_FILE_HEADER = "OpenBus vehicle odometers 1";

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

VehicleOdometerPreferences loadVehicleOdometerPreferences(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::binary);
    if (!input) {
        if (!std::filesystem::exists(path)) {
            return {};
        }
        throw std::runtime_error("Unable to open vehicle odometers: " + path.string());
    }

    std::string line;
    if (!std::getline(input, line) || line != ODOMETER_FILE_HEADER) {
        throw std::runtime_error("Invalid vehicle odometers header: " + path.string());
    }

    VehicleOdometerPreferences preferences;
    std::size_t lineNumber = 1;
    while (std::getline(input, line)) {
        ++lineNumber;
        if (line.empty()) {
            continue;
        }
        std::istringstream record(line);
        VehicleOdometerKey key;
        double metres = 0.0;
        std::string extra;
        if (!(record >> std::quoted(key.configurationPath) >> std::quoted(key.vehicleNumber) >>
              std::quoted(key.registration) >> metres) ||
            (record >> extra) || key.configurationPath.empty() || !std::isfinite(metres) ||
            metres < 0.0) {
            throw std::runtime_error("Invalid vehicle odometer at " + path.string() + ":" +
                                     std::to_string(lineNumber));
        }
        preferences[std::move(key)] = metres;
    }
    if (input.bad()) {
        throw std::runtime_error("Failed while reading vehicle odometers: " + path.string());
    }
    return preferences;
}

void saveVehicleOdometerPreferences(const std::filesystem::path& path,
                                    const VehicleOdometerPreferences& preferences) {
    for (const auto& entry : preferences) {
        if (entry.first.configurationPath.empty()) {
            throw std::invalid_argument("Vehicle odometer configuration path must not be empty");
        }
        const double metres = entry.second;
        if (!std::isfinite(metres) || metres < 0.0) {
            throw std::invalid_argument("Vehicle odometer must be finite and non-negative");
        }
    }

    writeAtomically(path, [&](std::ostream& output) {
        output << ODOMETER_FILE_HEADER << '\n' << std::setprecision(17);
        for (const auto& [key, metres] : preferences) {
            output << std::quoted(key.configurationPath) << ' ' << std::quoted(key.vehicleNumber)
                   << ' ' << std::quoted(key.registration) << ' ' << metres << '\n';
        }
    });
}

VehicleOdometerKey vehicleOdometerPreferenceKey(const std::filesystem::path& configurationPath,
                                                std::string vehicleNumber,
                                                std::string registration) {
    return {vehicleRegistrationPreferenceKey(configurationPath), std::move(vehicleNumber),
            std::move(registration)};
}

} // namespace openbus::app
