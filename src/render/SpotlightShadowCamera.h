#pragma once

#include "render/CameraMath.h"
#include "render/CoreRenderer.h"

#include <optional>

namespace openbus::rendering {

struct SpotlightShadowCamera {
    Matrix4 view;
    Matrix4 projection;
    double nearPlane;
    double farPlane;
};

// Zero- and 180-degree cones, or values not representable by the GPU, cannot
// be covered by one perspective shadow map and return no camera.
[[nodiscard]] std::optional<SpotlightShadowCamera>
makeSpotlightShadowCamera(const VehicleSpotlight& worldLight);

} // namespace openbus::rendering
