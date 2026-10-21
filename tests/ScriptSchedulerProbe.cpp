#include "app/ScriptTickScheduler.h"

#include <cmath>
#include <iostream>

namespace {

bool require(bool condition, const char* message) {
    if (!condition) {
        std::cerr << message << '\n';
        return false;
    }
    return true;
}

bool near(double actual, double expected) {
    return std::fabs(actual - expected) <= 1.0e-9;
}

} // namespace

int main() {
    using openbus::app::MAX_SCRIPT_CATCH_UP_TICKS;
    using openbus::app::scheduleScriptTicks;

    bool valid = true;
    constexpr double scriptStep = 1.0 / 60.0;

    const auto thirtyFps = scheduleScriptTicks(0.0, 1.0 / 30.0, scriptStep);
    valid &= require(thirtyFps.ticks == 2 && thirtyFps.droppedTicks == 0 &&
                         near(thirtyFps.remainderSeconds, 0.0),
                     "60 Hz scripts should execute two ticks at 30 FPS");

    const auto firstHighFpsFrame = scheduleScriptTicks(0.0, 1.0 / 120.0, scriptStep);
    const auto secondHighFpsFrame =
        scheduleScriptTicks(firstHighFpsFrame.remainderSeconds, 1.0 / 120.0, scriptStep);
    valid &= require(firstHighFpsFrame.ticks == 0 && secondHighFpsFrame.ticks == 1 &&
                         near(secondHighFpsFrame.remainderSeconds, 0.0),
                     "120 FPS should preserve fractional time between script ticks");

    const auto zeroElapsed = scheduleScriptTicks(0.0, 0.0, scriptStep);
    valid &= require(zeroElapsed.ticks == 0 && zeroElapsed.droppedTicks == 0 &&
                         near(zeroElapsed.remainderSeconds, 0.0),
                     "zero elapsed time should not schedule or drop ticks");

    const auto hitch = scheduleScriptTicks(0.0, 0.25, scriptStep);
    valid &= require(hitch.ticks == MAX_SCRIPT_CATCH_UP_TICKS && hitch.droppedTicks == 7 &&
                         near(hitch.droppedSeconds, 7.0 * scriptStep) &&
                         hitch.remainderSeconds < scriptStep,
                     "a 250 ms hitch should execute at most eight ticks and account for drops");

    const auto highRate = scheduleScriptTicks(0.0, 1.0 / 30.0, 1.0 / 1000.0);
    valid &= require(highRate.ticks == MAX_SCRIPT_CATCH_UP_TICKS &&
                         highRate.droppedTicks == 25 &&
                         near(highRate.droppedSeconds, 25.0 / 1000.0),
                     "high script rates should report ticks beyond the per-frame budget");

    const auto clampedElapsed = scheduleScriptTicks(0.0, 1.0, 1.0 / 1000.0);
    valid &= require(clampedElapsed.ticks == MAX_SCRIPT_CATCH_UP_TICKS &&
                         clampedElapsed.droppedTicks == 242 &&
                         near(clampedElapsed.droppedSeconds, 242.0 / 1000.0),
                     "elapsed time should be clamped before applying the tick budget");

    try {
        static_cast<void>(scheduleScriptTicks(0.0, 0.0, 0.0));
        valid &= require(false, "a non-positive script tick duration must be rejected");
    } catch (const std::invalid_argument&) {
    }

    return valid ? 0 : 1;
}
