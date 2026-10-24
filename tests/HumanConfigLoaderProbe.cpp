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
                    configuration.numericRecords[1].values.size() == 22 &&
                    std::abs(configuration.numericRecords[1].values.back() - 1.34) < 1.0e-9 &&
                    configuration.numericRecords[2].keyword == "walk_param" &&
                    configuration.numericRecords[2].valid &&
                    configuration.numericRecords[2].values ==
                        std::vector<double>{1.4, 70.0, 0.7, 1.0, 0.0},
                "human geometry, link, and walk parameter records are parsed");
        require(std::abs(configuration.walkStrideLengthMeters - 1.4) < 1.0e-9,
                "human walking stride length is retained from walk_param");
        require(std::abs(configuration.walkUpperArmBetaDegrees - 70.0) < 1.0e-9,
                "human upper-arm beta angle is retained from walk_param");
        require(std::abs(configuration.walkArmSwing - 0.7) < 1.0e-9,
                "human arm swing multiplier is retained from walk_param");
        require(configuration.walkHipSway == 1.0,
                "human hip sway setting is retained from walk_param");
        require(configuration.walkWaistAnimation == 0.0,
                "human waist animation setting is retained from walk_param");
        require(configuration.rightShoulderHeightFractions.has_value() &&
                    std::abs((*configuration.rightShoulderHeightFractions)[0] -
                             (0.17 / 1.75)) < 1.0e-9 &&
                    std::abs((*configuration.rightShoulderHeightFractions)[1] -
                             (1.39 / 1.75)) < 1.0e-9,
                "right shoulder pivot is derived from authored links and normalized height");
        require(configuration.rightHipHeightFractions.has_value() &&
                    std::abs((*configuration.rightHipHeightFractions)[0] -
                             (0.10 / 1.75)) < 1.0e-9 &&
                    std::abs((*configuration.rightHipHeightFractions)[1] -
                             (0.84 / 1.75)) < 1.0e-9,
                "right hip pivot is derived from authored links and normalized height");
        require(configuration.rightKneePositionFractions.has_value() &&
                    std::abs((*configuration.rightKneePositionFractions)[0] -
                             (0.14 / 1.75)) < 1.0e-9 &&
                    std::abs((*configuration.rightKneePositionFractions)[1] -
                             (-0.02 / 1.75)) < 1.0e-9 &&
                    std::abs((*configuration.rightKneePositionFractions)[2] -
                             (0.43 / 1.75)) < 1.0e-9,
                "right knee pivot is derived from authored links and normalized height");
        require(configuration.rightElbowHeightFractions.has_value() &&
                    std::abs((*configuration.rightElbowHeightFractions)[0] -
                             (0.43 / 1.75)) < 1.0e-9 &&
                    std::abs((*configuration.rightElbowHeightFractions)[1] -
                             (1.34 / 1.75)) < 1.0e-9,
                "right elbow pivot is derived from authored links and normalized height");
        require(configuration.rightHandHeightFractions.has_value() &&
                    std::abs((*configuration.rightHandHeightFractions)[0] -
                             (0.69 / 1.75)) < 1.0e-9 &&
                    std::abs((*configuration.rightHandHeightFractions)[1] -
                             (1.34 / 1.75)) < 1.0e-9,
                "right hand pivot is derived from authored links and normalized height");
        require(configuration.rightFingerHeightFractions.has_value() &&
                    std::abs((*configuration.rightFingerHeightFractions)[0] -
                             (0.89 / 1.75)) < 1.0e-9 &&
                    std::abs((*configuration.rightFingerHeightFractions)[1] -
                             (1.34 / 1.75)) < 1.0e-9,
                "right finger pivot is derived from authored links and normalized height");
        require(configuration.neckHeightFractions.has_value() &&
                    std::abs((*configuration.neckHeightFractions)[0] -
                             (-0.05 / 1.75)) < 1.0e-9 &&
                    std::abs((*configuration.neckHeightFractions)[1] -
                             (1.49 / 1.75)) < 1.0e-9,
                "neck pivot is derived from authored links and normalized height");
        require(configuration.waistHeightFractions.has_value() &&
                    std::abs((*configuration.waistHeightFractions)[0] -
                             (-0.01 / 1.75)) < 1.0e-9 &&
                    std::abs((*configuration.waistHeightFractions)[1] -
                             (1.05 / 1.75)) < 1.0e-9,
                "waist pivot is derived from authored links and normalized height");
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
