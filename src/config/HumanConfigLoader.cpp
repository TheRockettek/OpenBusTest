#include "config/HumanConfigLoader.h"

#include "config/ConfigurationParser.h"
#include "core/PerfTrace.h"

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
            readNumericRecord(line.number, keyword, 21);
        } else if (keyword == "walk_param") {
            readNumericRecord(line.number, keyword, 5);
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
    return configuration;
}

} // namespace openbus::config
