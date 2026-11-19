#pragma once

#include <algorithm>
#include <array>
#include <optional>

namespace openbus::rendering {

struct MapPointLight {
    std::array<double, 3> position = {};
    std::array<double, 3> color = {};
    double radius = 0.0;
};

[[nodiscard]] inline double
mapLightStrengthForEnvironmentBrightness(std::optional<double> environmentBrightness) noexcept {
    if (!environmentBrightness.has_value()) {
        return 0.0;
    }
    return 1.0 - std::clamp(*environmentBrightness, 0.0, 1.0);
}

} // namespace openbus::rendering
