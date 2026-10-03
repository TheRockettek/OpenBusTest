#pragma once

#include <algorithm>
#include <cmath>

namespace openbus::input {

struct MouseControlInputs {
    double throttle = 0.0;
    double steering = 0.0;
    double brake = 0.0;
};

inline MouseControlInputs mouseControlInputs(double cursorX, double cursorY, double width,
                                             double height) {
    if (!std::isfinite(cursorX) || !std::isfinite(cursorY) || !std::isfinite(width) ||
        !std::isfinite(height) || width <= 0.0 || height <= 0.0) {
        return {};
    }

    const double horizontal = std::clamp(2.0 * cursorX / width - 1.0, -1.0, 1.0);
    const double vertical = std::clamp(2.0 * cursorY / height - 1.0, -1.0, 1.0);
    return {std::max(0.0, -vertical), horizontal, std::max(0.0, vertical)};
}

} // namespace openbus::input
