#pragma once

#include "render/MatrixMath.h"

#include <array>

namespace openbus::app::vehicle {

enum class VehiclePartLoadPriority { Required, Prefetch, Skip };

// Parts outside every [LOD] section are drawn at all distances, so they are always required.
// Other LODs of the current view are prefetched: draw selects LOD per part from the real eye
// distance, which can differ from the load distance and must never find an unloaded LOD.
inline VehiclePartLoadPriority vehiclePartLoadPriority(bool viewpointMatches, int partLodIndex,
                                                       int selectedLodIndex) {
    if (!viewpointMatches) {
        return VehiclePartLoadPriority::Skip;
    }
    if (selectedLodIndex < 0 || partLodIndex < 0 || partLodIndex == selectedLodIndex) {
        return VehiclePartLoadPriority::Required;
    }
    return VehiclePartLoadPriority::Prefetch;
}

inline std::array<double, 3> cameraEyeFromModelView(const openbus::rendering::Matrix4& view) {
    return {-(view[0] * view[12] + view[1] * view[13] + view[2] * view[14]),
            -(view[4] * view[12] + view[5] * view[13] + view[6] * view[14]),
            -(view[8] * view[12] + view[9] * view[13] + view[10] * view[14])};
}

} // namespace openbus::app::vehicle
