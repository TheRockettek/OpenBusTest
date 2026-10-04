#pragma once

#include "CameraMath.h"

#include <array>

namespace openbus::rendering {

// OMSI model coordinates are x=lateral, y=longitudinal, z=height. Mesh loading
// converts them to the renderer's longitudinal/lateral/height coordinate basis.
inline std::array<double, 3>
interiorLightPositionInViewSpace(const Matrix4& vehicleRootModelView,
                                 const std::array<double, 3>& omsiPosition) {
    const std::array<double, 4> modelPosition = {omsiPosition[1], -omsiPosition[0],
                                                  omsiPosition[2], 1.0};
    std::array<double, 4> viewPosition = {};
    for (int row = 0; row < 4; ++row) {
        for (int column = 0; column < 4; ++column) {
            viewPosition[row] += vehicleRootModelView[row + column * 4] * modelPosition[column];
        }
    }
    return {viewPosition[0], viewPosition[1], viewPosition[2]};
}

} // namespace openbus::rendering