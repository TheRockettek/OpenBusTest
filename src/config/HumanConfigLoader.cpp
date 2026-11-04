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
        } else if (keyword == "mass" || keyword == "age") {
            Line value;
            if (!reader.readPayload(value, configuration.diagnostics, keyword)) {
                continue;
            }
            double number = 0.0;
            if (!parseDouble(value.text, number)) {
                configuration.diagnostics.error(value.number, keyword, "expected a numeric value");
            } else if (number < 0.0) {
                configuration.diagnostics.error(value.number, keyword,
                                                keyword + " must be non-negative");
            } else {
                std::optional<double>& field =
                    keyword == "mass" ? configuration.mass : configuration.age;
                if (!field.has_value()) {
                    field = number;
                } else {
                    configuration.diagnostics.warning(
                        line.number, keyword, "duplicate " + keyword + " ignored; first retained");
                }
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
        const double kneeX = std::abs(links->values[3]);
        const double kneeY = links->values[4];
        const double kneeZ = links->values[5];
        if (kneeX <= height * 0.5 && std::abs(kneeY) <= height * 0.5 && kneeZ > 0.0 &&
            kneeZ < hipZ && kneeZ <= height) {
            configuration.rightKneePositionFractions =
                std::array<double, 3>{kneeX / height, kneeY / height, kneeZ / height};
        } else {
            configuration.diagnostics.warning(
                0, "links", "right-knee coordinates are outside the human geometry");
        }
        const double shoulderX = std::abs(links->values[8]);
        const double shoulderZ = links->values[10];
        const double elbowX = std::abs(links->values[11]);
        const double elbowY = links->values[12];
        const double elbowZ = links->values[13];
        if (elbowX <= height * 0.5 && std::abs(elbowY) <= height * 0.5 && elbowZ > 0.0 &&
            elbowZ < shoulderZ && elbowZ <= height) {
            configuration.rightElbowHeightFractions =
                std::array<double, 2>{elbowX / height, elbowZ / height};
        } else {
            configuration.diagnostics.warning(
                0, "links", "right-elbow coordinates are outside the human geometry");
        }
        const double handX = std::abs(links->values[16]);
        const double handY = links->values[17];
        const double handZ = links->values[18];
        if (handX <= height * 0.75 && std::abs(handY) <= height * 0.5 && handZ > 0.0 &&
            handZ <= height) {
            configuration.rightHandHeightFractions =
                std::array<double, 2>{handX / height, handZ / height};
        } else {
            configuration.diagnostics.warning(
                0, "links", "right-hand coordinates are outside the human geometry");
        }
        const double fingerX = std::abs(links->values[19]);
        const double fingerY = links->values[20];
        const double fingerZ = links->values[21];
        if (fingerX <= height && std::abs(fingerY) <= height * 0.5 && fingerZ > 0.0 &&
            fingerZ <= height) {
            configuration.rightFingerHeightFractions =
                std::array<double, 2>{fingerX / height, fingerZ / height};
        } else {
            configuration.diagnostics.warning(
                0, "links", "right-finger coordinates are outside the human geometry");
        }
        const double neckY = links->values[14];
        const double neckZ = links->values[15];
        if (std::abs(neckY) <= height * 0.5 && neckZ > shoulderZ && neckZ <= height) {
            configuration.neckHeightFractions =
                std::array<double, 2>{neckY / height, neckZ / height};
        } else {
            configuration.diagnostics.warning(0, "links",
                                              "neck coordinates are outside the human geometry");
        }
        const double waistY = links->values[6];
        const double waistZ = links->values[7];
        if (std::abs(waistY) <= height * 0.5 && waistZ > hipZ && waistZ < shoulderZ &&
            waistZ <= height) {
            configuration.waistHeightFractions =
                std::array<double, 2>{waistY / height, waistZ / height};
        } else {
            configuration.diagnostics.warning(0, "links",
                                              "waist coordinates are outside the human geometry");
        }
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
