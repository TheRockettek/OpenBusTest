#pragma once

#include "config/ConfigurationTypes.h"

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace openbus::config {

struct HumanNumericRecord {
    std::string keyword;
    std::vector<std::string> rawFields;
    std::vector<double> values;
    bool valid = false;
};

struct HumanConfig {
    std::string modelConfigPath;
    std::optional<double> seatHeight;
    double walkStrideLengthMeters = 1.4;
    double walkUpperArmBetaDegrees = 66.0;
    double walkArmSwing = 1.0;
    double walkHipSway = 1.0;
    double walkWaistAnimation = 0.0;
    std::optional<double> mass;
    std::string voice;
    std::vector<HumanNumericRecord> numericRecords;
    ConfigurationDiagnostics diagnostics;
};

HumanConfig loadHumanConfig(const std::filesystem::path& configPath);

} // namespace openbus::config
