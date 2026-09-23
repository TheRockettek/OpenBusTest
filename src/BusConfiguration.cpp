#include "BusConfiguration.h"

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace {

std::vector<BusAxle> loadBusAxles(const std::filesystem::path& busPath) {
    std::ifstream input(busPath);
    if (!input) {
        return {};
    }

    const auto trim = [](std::string value) {
        const std::size_t first = value.find_first_not_of(" \t\r");
        if (first == std::string::npos) {
            return std::string();
        }
        const std::size_t last = value.find_last_not_of(" \t\r");
        return value.substr(first, last - first + 1);
    };

    std::vector<BusAxle> axles;
    std::string line;
    while (std::getline(input, line)) {
        if (trim(line) != "[newachse]") {
            continue;
        }

        std::unordered_map<std::string, double> values;
        std::string key;
        const auto isAxleKey = [](const std::string& candidate) {
            return candidate == "achse_long" || candidate == "achse_maxwidth" ||
                   candidate == "achse_minwidth" || candidate == "achse_raddurchmesser" ||
                   candidate == "achse_feder" || candidate == "achse_maxforce" ||
                   candidate == "achse_daempfer" || candidate == "achse_antrieb";
        };
        while (true) {
            const std::streampos linePosition = input.tellg();
            if (!std::getline(input, line)) {
                break;
            }
            const std::string normalized = trim(line);
            if (normalized.empty() || normalized.front() == ';') {
                continue;
            }
            if (normalized.front() == '[') {
                input.clear();
                input.seekg(linePosition);
                break;
            }
            if (key.empty()) {
                if (isAxleKey(normalized)) {
                    key = normalized;
                }
                continue;
            }
            try {
                values[key] = std::stod(normalized);
            } catch (const std::exception&) {
                continue;
            }
            key.clear();
        }

        const auto value = [&](const char* name, double fallback) {
            const auto found = values.find(name);
            return found != values.end() ? found->second : fallback;
        };
        const double position = value("achse_long", 0.0);
        const double maxWidth = value("achse_maxwidth", 0.0);
        const double minWidth = value("achse_minwidth", 0.0);
        const double wheelDiameter = value("achse_raddurchmesser", 0.0);
        if (maxWidth <= 0.0 || wheelDiameter <= 0.0) {
            continue;
        }
        axles.push_back({position,
                         maxWidth,
                         maxWidth,
                         minWidth,
                         wheelDiameter,
                         value("achse_feder", 250.0) * 1000.0,
                         value("achse_maxforce", 0.0) * 1000.0,
                         value("achse_daempfer", 16.0) * 1000.0,
                         false,
                         value("achse_antrieb", 0.0) != 0.0});
    }
    return axles;
}

std::vector<BusAxle> loadE400Axles() {
    const std::array<std::filesystem::path, 3> candidates = {
        std::filesystem::path("SP_E400MMC") / "E400MMC_ADL_10.9m_Voith_LowHeight.bus",
        std::filesystem::current_path() / "SP_E400MMC" / "E400MMC_ADL_10.9m_Voith_LowHeight.bus",
        std::filesystem::current_path().parent_path() / "SP_E400MMC" /
            "E400MMC_ADL_10.9m_Voith_LowHeight.bus"};
    for (const auto& candidate : candidates) {
        if (std::filesystem::exists(candidate)) {
            const std::vector<BusAxle> axles = loadBusAxles(candidate);
            if (!axles.empty()) {
                return axles;
            }
        }
    }
    return {};
}

} // namespace

BusConfiguration BusConfiguration::lionCity12() {
    BusConfiguration configuration;
    configuration.axles = {{3.45, 2.30, 2.30, 0.0, 1.01, 250000.0, 0.0, 16000.0, true, true},
                           {-3.45, 2.30, 2.30, 0.0, 1.01, 250000.0, 0.0, 16000.0, false,
                            true}};
    return configuration;
}

BusConfiguration BusConfiguration::manDl05() {
    BusConfiguration configuration;
    configuration.axles = {{4.05018, 2.11836, 2.11836, 0.0, 1.01, 250000.0, 0.0, 16000.0, true,
                            true},
                           {-1.73317, 1.79100, 1.79100, 0.0, 1.01, 250000.0, 0.0, 16000.0, false,
                            true},
                           {-3.40552, 2.11836, 2.11836, 0.0, 1.01, 250000.0, 0.0, 16000.0, false,
                            true}};
    return configuration;
}

BusConfiguration BusConfiguration::spE400Mmc() {
    BusConfiguration configuration;
    configuration.mass = 12600.0;
    configuration.length = 10.90;
    configuration.width = 2.52;
    configuration.bodyHalfLength = 5.45;
    configuration.bodyHalfWidth = 1.26;
    configuration.wheelRadius = 0.483;
    configuration.wheelHalfWidth = 0.153;
    configuration.collisionLength = 10.84;
    configuration.collisionWidth = 2.52;
    configuration.collisionHeight = 3.96;
    configuration.collisionOffsetZ = 1.34;
    configuration.axles = {{2.80019, 2.20280, 2.20280, 0.0, 0.946602, 250000.0, 0.0,
                            16000.0, true, false},
                           {-2.92441, 1.93272, 1.93272, 0.0, 0.946602, 250000.0, 0.0, 16000.0,
                            false, true}};
    const std::vector<BusAxle> parsedAxles = loadE400Axles();
    if (!parsedAxles.empty()) {
        configuration.axles = parsedAxles;
        configuration.axles.front().steerable = true;
        configuration.wheelRadius = configuration.axles.front().wheelDiameter * 0.5;
    }
    return configuration;
}

BusVehicle busVehicleFromEnvironment() {
    const char* selected = std::getenv("OPENBUS_VEHICLE");
    if (selected != nullptr) {
        const std::string value(selected);
        if (value == "e400" || value == "sp_e400mmc" || value == "400mmc") {
            return BusVehicle::SpE400Mmc;
        }
    }
    return BusVehicle::ManDl05;
}

BusConfiguration busConfigurationFor(BusVehicle vehicle) {
    return vehicle == BusVehicle::SpE400Mmc ? BusConfiguration::spE400Mmc()
                                            : BusConfiguration::manDl05();
}