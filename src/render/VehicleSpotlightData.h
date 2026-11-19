#pragma once

#include <array>

namespace openbus::rendering {

struct VehicleSpotlight {
    std::array<double, 3> position = {};
    std::array<double, 3> direction = {};
    std::array<double, 3> color = {};
    double range = 0.0;
    double innerAngleDegrees = 0.0;
    double outerAngleDegrees = 0.0;
};

} // namespace openbus::rendering
