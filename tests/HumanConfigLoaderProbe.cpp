#include "config/HumanConfigLoader.h"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

} // namespace

int main(int argc, char** argv) {
    try {
        if (argc != 2) {
            throw std::runtime_error("expected <human-config-fixture>");
        }
        const openbus::config::HumanConfig configuration =
            openbus::config::loadHumanConfig(argv[1]);
        require(configuration.modelConfigPath == "model\\driver.cfg",
                "human [model] path is retained");
        require(configuration.seatHeight.has_value() &&
                    std::abs(*configuration.seatHeight - 0.75) < 1.0e-9,
                "human [seatheight] is parsed");
        require(configuration.numericRecords.size() == 3 &&
                    configuration.numericRecords[0].keyword == "humangeom" &&
                    configuration.numericRecords[0].valid &&
                    configuration.numericRecords[0].values ==
                        std::vector<double>{0.2, 1.75} &&
                    configuration.numericRecords[1].keyword == "links" &&
                    configuration.numericRecords[1].valid &&
                    configuration.numericRecords[1].values.size() == 21 &&
                    configuration.numericRecords[2].keyword == "walk_param" &&
                    configuration.numericRecords[2].valid &&
                    configuration.numericRecords[2].values ==
                        std::vector<double>{1.4, 70.0, 0.7, 1.0, 0.0},
                "human geometry, link, and walk parameter records are parsed");
        require(configuration.voice == "M2" && configuration.mass.has_value() &&
                    std::abs(*configuration.mass - 0.07) < 1.0e-9,
                "human [voice] and [mass] fields are parsed");
        require(!configuration.diagnostics.hasErrors(),
                "valid human configuration has no errors");
    } catch (const std::exception& error) {
        std::cerr << "human configuration probe failed: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
