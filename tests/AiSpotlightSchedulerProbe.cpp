#include "app/vehicle/AiSpotlightScheduler.h"

#include <iostream>

namespace {

bool expectInterval(const char* label, const openbus::app::vehicle::AiSpotlightScheduleInput& input,
                    const openbus::app::vehicle::AiSpotlightScheduleConfig& config,
                    int expected) {
    const int actual = openbus::app::vehicle::aiSpotlightUpdateIntervalFrames(input, config);
    if (actual == expected) {
        return true;
    }
    std::cerr << label << ": expected " << expected << " frames, got " << actual << '\n';
    return false;
}

} // namespace

int main() {
    using openbus::app::vehicle::AiSpotlightScheduleConfig;
    using openbus::app::vehicle::AiSpotlightScheduleInput;

    const AiSpotlightScheduleConfig config;
    bool passed = true;
    passed &= expectInterval("near", {20.0, 0.0, 0.0, false, false}, config, 2);
    passed &= expectInterval("medium", {100.0, 0.0, 0.0, false, false}, config, 6);
    passed &= expectInterval("far", {250.0, 0.0, 0.0, false, false}, config, 20);
    passed &= expectInterval("approaching", {100.0, 0.0, 6.0, false, false}, config, 3);
    passed &= expectInterval("fast", {20.0, 20.0, 0.0, false, false}, config, 1);
    passed &= expectInterval("state change", {250.0, 0.0, 0.0, true, false}, config, 1);
    passed &= expectInterval("maneuver", {250.0, 0.0, 0.0, false, true}, config, 1);
    passed &= expectInterval("invalid distance", {-1.0, 0.0, 0.0, false, false}, config, 1);
    using openbus::app::vehicle::aiSpotlightReturnedToView;
    if (aiSpotlightReturnedToView(false, 10, 0) || aiSpotlightReturnedToView(true, 10, 10) ||
        aiSpotlightReturnedToView(true, 10, 9) || !aiSpotlightReturnedToView(true, 10, 8)) {
        std::cerr << "AI spotlight view reentry scheduling failed\n";
        passed = false;
    }
    return passed ? 0 : 1;
}
