#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace openbus::app::vehicle {

struct AiSpotlightScheduleConfig {
    double nearDistanceMeters = 60.0;
    double mediumDistanceMeters = 180.0;
    int nearIntervalFrames = 2;
    int mediumIntervalFrames = 6;
    int farIntervalFrames = 20;
    double approachingSpeedMetersPerSecond = 5.0;
    double highSpeedMetersPerSecond = 18.0;
};

struct AiSpotlightScheduleInput {
    double distanceMeters = 0.0;
    double speedMetersPerSecond = 0.0;
    double approachSpeedMetersPerSecond = 0.0;
    bool stateChanged = false;
    bool maneuvering = false;
};

inline bool aiSpotlightReturnedToView(bool initialized, std::uint64_t currentFrame,
                                      std::uint64_t lastRelevantFrame) {
    return initialized && currentFrame > lastRelevantFrame && currentFrame - lastRelevantFrame > 1;
}

inline int aiSpotlightUpdateIntervalFrames(const AiSpotlightScheduleInput& input,
                                           const AiSpotlightScheduleConfig& config) {
    if (input.stateChanged || input.maneuvering) {
        return 1;
    }
    if (!std::isfinite(input.distanceMeters) || input.distanceMeters < 0.0) {
        return 1;
    }

    int interval = input.distanceMeters <= config.nearDistanceMeters ? config.nearIntervalFrames
                   : input.distanceMeters <= config.mediumDistanceMeters
                       ? config.mediumIntervalFrames
                       : config.farIntervalFrames;
    if (input.speedMetersPerSecond >= config.highSpeedMetersPerSecond ||
        input.approachSpeedMetersPerSecond >= config.approachingSpeedMetersPerSecond) {
        interval = std::max(1, interval / 2);
    }
    return interval;
}

} // namespace openbus::app::vehicle
