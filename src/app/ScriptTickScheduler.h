#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace openbus::app {

inline constexpr int MAX_SCRIPT_CATCH_UP_TICKS = 8;
inline constexpr double MAX_SCRIPT_FRAME_DELTA_SECONDS = 0.25;

struct ScriptTickSchedule {
    int ticks = 0;
    int droppedTicks = 0;
    double remainderSeconds = 0.0;
    double droppedSeconds = 0.0;
};

inline ScriptTickSchedule scheduleScriptTicks(double accumulatorSeconds, double elapsedSeconds,
                                              double tickDurationSeconds) {
    if (!std::isfinite(accumulatorSeconds) || accumulatorSeconds < 0.0 ||
        !std::isfinite(elapsedSeconds) || elapsedSeconds < 0.0 ||
        !std::isfinite(tickDurationSeconds) || tickDurationSeconds <= 0.0 ||
        accumulatorSeconds >= tickDurationSeconds) {
        throw std::invalid_argument("Script tick scheduler received an invalid time value");
    }

    double accumulator = accumulatorSeconds +
                         std::clamp(elapsedSeconds, 0.0, MAX_SCRIPT_FRAME_DELTA_SECONDS);
    ScriptTickSchedule schedule;
    while (accumulator >= tickDurationSeconds &&
           schedule.ticks < MAX_SCRIPT_CATCH_UP_TICKS) {
        accumulator -= tickDurationSeconds;
        ++schedule.ticks;
    }

    if (accumulator >= tickDurationSeconds) {
        schedule.remainderSeconds = std::fmod(accumulator, tickDurationSeconds);
        const double remainderTolerance = tickDurationSeconds * 1.0e-12;
        if (schedule.remainderSeconds <= remainderTolerance ||
            tickDurationSeconds - schedule.remainderSeconds <= remainderTolerance) {
            schedule.remainderSeconds = 0.0;
        }
        schedule.droppedSeconds = accumulator - schedule.remainderSeconds;
        schedule.droppedTicks =
            static_cast<int>(std::round(schedule.droppedSeconds / tickDurationSeconds));
    } else {
        schedule.remainderSeconds = accumulator;
    }
    return schedule;
}

} // namespace openbus::app
