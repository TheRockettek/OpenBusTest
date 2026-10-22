#pragma once

#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>

namespace openbus::map {

struct MapTrafficSignalPhase {
    int state = 0;
    double durationSeconds = 0.0;
};

struct MapTrafficSignalSchedule {
    double cycleDurationSeconds = 0.0;
    std::vector<MapTrafficSignalPhase> phases;
    bool hasConditionalOverrides = false;
};

// Returns the active OMSI light phase, or no value when the schedule is invalid.
inline std::optional<int> mapTrafficSignalStateAtTime(
    const MapTrafficSignalSchedule& schedule, double elapsedSeconds) {
    if (!std::isfinite(schedule.cycleDurationSeconds) ||
        schedule.cycleDurationSeconds <= 0.0 || !std::isfinite(elapsedSeconds) ||
        elapsedSeconds < 0.0 || schedule.phases.empty()) {
        return std::nullopt;
    }

    double configuredDuration = 0.0;
    for (std::size_t index = 0; index < schedule.phases.size(); ++index) {
        const MapTrafficSignalPhase& phase = schedule.phases[index];
        if (phase.state < 0 || phase.state > 12 || !std::isfinite(phase.durationSeconds) ||
            phase.durationSeconds < 0.0 ||
            (phase.durationSeconds == 0.0 && index + 1 != schedule.phases.size())) {
            return std::nullopt;
        }
        configuredDuration += phase.durationSeconds;
    }
    const double durationTolerance =
        std::max(1.0, schedule.cycleDurationSeconds) * 1.0e-9;
    const bool finalPhaseExtendsToCycle = schedule.phases.back().durationSeconds == 0.0;
    if (configuredDuration > schedule.cycleDurationSeconds + durationTolerance ||
        (!finalPhaseExtendsToCycle &&
         std::abs(configuredDuration - schedule.cycleDurationSeconds) > durationTolerance) ||
        (finalPhaseExtendsToCycle &&
         configuredDuration >= schedule.cycleDurationSeconds - durationTolerance)) {
        return std::nullopt;
    }

    const double cycleTime = std::fmod(elapsedSeconds, schedule.cycleDurationSeconds);
    double phaseStart = 0.0;
    for (std::size_t index = 0; index < schedule.phases.size(); ++index) {
        const MapTrafficSignalPhase& phase = schedule.phases[index];
        if (phase.durationSeconds == 0.0 && index + 1 == schedule.phases.size()) {
            return phase.state;
        }
        const double phaseEnd = phaseStart + phase.durationSeconds;
        if (cycleTime < phaseEnd) {
            return phase.state;
        }
        phaseStart = phaseEnd;
    }
    return std::nullopt;
}

} // namespace openbus::map
