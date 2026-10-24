#include "config/HumanConfigLoader.h"

#include "config/ConfigurationParser.h"
#include "core/PerfTrace.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace openbus::config {

HumanConfig loadHumanConfig(const std::filesystem::path& configPath) {
    openbus::rendering::TraceScope trace("config", "HumanConfigLoader::load");
    Reader reader(configPath);
    if (!reader.isOpen()) {
        throw std::runtime_error("Could not open human configuration: " + configPath.string());
    }

    HumanConfig configuration;
    const auto readNumericRecord = [&](std::size_t lineNumber, const std::string& keyword,
                                       std::size_t fieldCount) {
        HumanNumericRecord record;
        record.keyword = keyword;
        bool complete =
            reader.readPayloads(fieldCount, record.rawFields, configuration.diagnostics, keyword);
        bool numeric = complete;
        for (const std::string& rawField : record.rawFields) {
            double value = 0.0;
            if (parseDouble(rawField, value)) {
                record.values.push_back(value);
            } else {
                numeric = false;
                configuration.diagnostics.error(lineNumber, keyword, "expected a numeric value");
            }
        }
        record.valid = complete && numeric && record.values.size() == fieldCount;
        configuration.numericRecords.push_back(std::move(record));
    };

    Line line;
    while (reader.next(line)) {
        if (!line.isKeyword()) {
            continue;
        }
        const std::string keyword = lower(line.keyword());
        if (keyword == "model") {
            Line value;
            if (!reader.readPayload(value, configuration.diagnostics, keyword)) {
                continue;
            }
            if (configuration.modelConfigPath.empty()) {
                configuration.modelConfigPath = trim(value.text);
            } else {
                configuration.diagnostics.warning(
                    line.number, keyword, "duplicate model reference ignored; first retained");
            }
        } else if (keyword == "seatheight") {
            Line value;
            if (!reader.readPayload(value, configuration.diagnostics, keyword)) {
                continue;
            }
            double seatHeight = 0.0;
            if (!parseDouble(value.text, seatHeight)) {
                configuration.diagnostics.error(value.number, keyword, "expected a numeric value");
            } else if (!configuration.seatHeight.has_value()) {
                configuration.seatHeight = seatHeight;
            } else {
                configuration.diagnostics.warning(line.number, keyword,
                                                  "duplicate seat height ignored; first retained");
            }
        } else if (keyword == "humangeom") {
            readNumericRecord(line.number, keyword, 2);
        } else if (keyword == "links") {
            readNumericRecord(line.number, keyword, 22);
        } else if (keyword == "walk_param") {
            readNumericRecord(line.number, keyword, 5);
            const HumanNumericRecord& record = configuration.numericRecords.back();
            if (record.valid) {
                if (record.values[0] <= 0.0) {
                    configuration.diagnostics.error(line.number, keyword,
                                                    "step length must be greater than zero");
                } else if (record.values[1] < 0.0 || record.values[1] > 180.0) {
                    configuration.diagnostics.error(line.number, keyword,
                                                    "upper-arm beta must be in [0, 180] degrees");
                } else {
                    configuration.walkStrideLengthMeters = record.values[0];
                    configuration.walkUpperArmBetaDegrees = record.values[1];
                    configuration.walkArmSwing = record.values[2];
                    configuration.walkHipSway = record.values[3];
                    configuration.walkWaistAnimation = record.values[4];
                }
            }
        } else if (keyword == "mass") {
            Line value;
            if (!reader.readPayload(value, configuration.diagnostics, keyword)) {
                continue;
            }
            double mass = 0.0;
            if (!parseDouble(value.text, mass)) {
                configuration.diagnostics.error(value.number, keyword, "expected a numeric value");
            } else if (!configuration.mass.has_value()) {
                configuration.mass = mass;
            } else {
                configuration.diagnostics.warning(line.number, keyword,
                                                  "duplicate mass ignored; first retained");
            }
        } else if (keyword == "voice") {
            Line value;
            if (!reader.readPayload(value, configuration.diagnostics, keyword)) {
                continue;
            }
            if (configuration.voice.empty()) {
                configuration.voice = trim(value.text);
            } else {
                configuration.diagnostics.warning(line.number, keyword,
                                                  "duplicate voice ignored; first retained");
            }
        }
    }

    if (configuration.modelConfigPath.empty()) {
        configuration.diagnostics.error(0, "model", "required model reference is missing");
    }
    const auto geometry =
        std::find_if(configuration.numericRecords.begin(), configuration.numericRecords.end(),
                     [](const HumanNumericRecord& record) {
                         return record.keyword == "humangeom" && record.valid &&
                                record.values.size() == 2 && record.values[1] > 0.0;
                     });
    const auto links = std::find_if(
        configuration.numericRecords.begin(), configuration.numericRecords.end(),
        [](const HumanNumericRecord& record) {
            return record.keyword == "links" && record.valid && record.values.size() == 22;
        });
    if (geometry != configuration.numericRecords.end() &&
        links != configuration.numericRecords.end()) {
        const double height = geometry->values[1];
        const double hipX = std::abs(links->values[0]);
        const double hipZ = links->values[2];
        if (hipX <= height * 0.5 && hipZ > 0.0 && hipZ <= height) {
            configuration.rightHipHeightFractions =
                std::array<double, 2>{hipX / height, hipZ / height};
        } else {
            configuration.diagnostics.warning(
                0, "links", "right-hip coordinates are outside the human geometry");
        }
        const double shoulderX = std::abs(links->values[8]);
        const double shoulderZ = links->values[10];
        if (shoulderX <= height * 0.5 && shoulderZ > 0.0 && shoulderZ <= height) {
            configuration.rightShoulderHeightFractions =
                std::array<double, 2>{shoulderX / height, shoulderZ / height};
        } else {
            configuration.diagnostics.warning(
                0, "links", "right-shoulder coordinates are outside the human geometry");
        }
    }
    return configuration;
}

} // namespace openbus::config
