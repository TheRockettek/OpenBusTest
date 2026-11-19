#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace openbus::app::vehicle {

inline float advanceSpotlightShadowVisibility(float current, bool visible) {
    constexpr float step = 0.125F;
    return std::clamp(current + (visible ? step : -step), 0.0F, 1.0F);
}

inline bool spotlightShadowFresh(std::uint64_t age, int updateInterval) {
    return age <= std::max<std::uint64_t>(8, static_cast<std::uint64_t>(updateInterval) * 3);
}

inline bool reserveSpotlightShadowUpdateForAi(std::size_t budget, std::uint64_t frame) {
    return budget == 1 && frame % 8 == 0;
}

inline bool spotlightShadowUpdateBefore(bool firstPlayer, std::uint64_t firstUpdate,
                                        bool secondPlayer, std::uint64_t secondUpdate,
                                        std::size_t budget, std::uint64_t frame) {
    if (firstPlayer != secondPlayer)
        return reserveSpotlightShadowUpdateForAi(budget, frame) ? !firstPlayer : firstPlayer;
    return firstUpdate < secondUpdate;
}

inline std::size_t adaptSpotlightShadowUpdateBudget(std::size_t current, double framesPerSecond,
                                                    double frameRateCap = 0.0) {
    current = std::clamp<std::size_t>(current, 1, 8);
    if (!std::isfinite(framesPerSecond) || framesPerSecond <= 0.0)
        return current;
    const double target =
        std::isfinite(frameRateCap) && frameRateCap > 0.0 ? std::min(frameRateCap, 60.0) : 60.0;
    if (framesPerSecond < target * (2.0 / 3.0))
        return std::max<std::size_t>(1, current - 1);
    if (framesPerSecond > target * (11.0 / 12.0))
        return std::min<std::size_t>(8, current + 1);
    return current;
}

} // namespace openbus::app::vehicle
