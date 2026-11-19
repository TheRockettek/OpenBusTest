#include "render/SpotlightShadowCamera.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>

namespace openbus::rendering {

std::optional<SpotlightShadowCamera> makeSpotlightShadowCamera(const VehicleSpotlight& light) {
    const auto finite = [](double value) { return std::isfinite(value); };
    const double directionLength =
        std::hypot(light.direction[0], light.direction[1], light.direction[2]);
    if (!std::all_of(light.position.begin(), light.position.end(), finite) ||
        !std::all_of(light.direction.begin(), light.direction.end(), finite) ||
        !std::isfinite(directionLength) || directionLength < 1e-12 || !std::isfinite(light.range) ||
        light.range <= 0.0 || !std::isfinite(light.outerAngleDegrees) ||
        light.outerAngleDegrees < 0.0 || light.outerAngleDegrees > 180.0) {
        throw std::invalid_argument("Invalid spotlight shadow camera parameters");
    }
    if (light.outerAngleDegrees == 0.0 || light.outerAngleDegrees == 180.0) {
        return std::nullopt;
    }
    const double nearPlane = std::min(0.05, light.range * 0.01);
    const double farPlane = light.range * 4.0;
    const double scale = 1.0 / std::tan(light.outerAngleDegrees * std::numbers::pi / 360.0);
    if (nearPlane < std::numeric_limits<float>::min() || !std::isfinite(farPlane) ||
        farPlane > std::numeric_limits<float>::max() || !std::isfinite(scale) ||
        scale > std::numeric_limits<float>::max()) {
        return std::nullopt;
    }
    const Matrix4 view = makeLookDirectionMatrix(light.position,
                                                 {light.direction[0] / directionLength,
                                                  light.direction[1] / directionLength,
                                                  light.direction[2] / directionLength},
                                                 {0.0, 0.0, 1.0});
    const Matrix4 projection = {scale,
                                0.0,
                                0.0,
                                0.0,
                                0.0,
                                scale,
                                0.0,
                                0.0,
                                0.0,
                                0.0,
                                -(farPlane + nearPlane) / (farPlane - nearPlane),
                                -1.0,
                                0.0,
                                0.0,
                                -(2.0 * farPlane * nearPlane) / (farPlane - nearPlane),
                                0.0};
    const auto representable = [](double value) {
        return std::isfinite(value) && std::abs(value) <= std::numeric_limits<float>::max();
    };
    if (!std::all_of(view.begin(), view.end(), representable) ||
        !std::all_of(projection.begin(), projection.end(), representable)) {
        return std::nullopt;
    }
    return SpotlightShadowCamera{view, projection, nearPlane, farPlane};
}

} // namespace openbus::rendering
