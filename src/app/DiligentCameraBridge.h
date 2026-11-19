#pragma once

#include "map/MapTrafficRuntime.h"
#include "render/MatrixMath.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace openbus::app {

struct DiligentCameraView {
    openbus::rendering::Matrix4 view = {};
    openbus::rendering::Matrix4 projection = {};
    openbus::rendering::Matrix4 viewProjection = {};
    std::array<double, 3> cameraPosition = {};
    int viewportWidth = 0;
    int viewportHeight = 0;
};

// Diligent uses a [0, 1] clip-space depth range. The Vulkan backend flips the
// viewport to retain the Direct3D Y convention, so no additional Y conversion is needed.
inline openbus::rendering::Matrix4
convertOpenGLProjectionToDiligent(const openbus::rendering::Matrix4& projection) {
    if (!std::all_of(projection.begin(), projection.end(),
                     [](double value) { return std::isfinite(value); })) {
        throw std::invalid_argument("OpenGL projection matrix must contain finite values");
    }

    openbus::rendering::Matrix4 converted = projection;
    for (std::size_t column = 0; column < 4; ++column) {
        const std::size_t offset = column * 4;
        converted[offset + 2] = 0.5 * (projection[offset + 2] + projection[offset + 3]);
    }
    return converted;
}

inline DiligentCameraView makeDiligentCameraView(const openbus::rendering::Matrix4& view,
                                                 const openbus::rendering::Matrix4& projection,
                                                 const std::array<double, 3>& cameraPosition,
                                                 int viewportWidth, int viewportHeight) {
    const auto isFiniteMatrix = [](const openbus::rendering::Matrix4& matrix) {
        return std::all_of(matrix.begin(), matrix.end(),
                           [](double value) { return std::isfinite(value); });
    };
    if (viewportWidth <= 0 || viewportHeight <= 0 ||
        !std::all_of(cameraPosition.begin(), cameraPosition.end(),
                     [](double value) { return std::isfinite(value); }) ||
        !isFiniteMatrix(view) || !isFiniteMatrix(projection)) {
        throw std::invalid_argument("Diligent camera view is incomplete or invalid");
    }

    DiligentCameraView result;
    result.view = view;
    result.projection = convertOpenGLProjectionToDiligent(projection);
    result.viewProjection = openbus::rendering::multiplyMatrix4(result.projection, result.view);
    result.cameraPosition = cameraPosition;
    result.viewportWidth = viewportWidth;
    result.viewportHeight = viewportHeight;
    return result;
}

inline DiligentCameraView
makeDiligentCameraViewFromPose(const std::array<double, 3>& cameraPosition,
                               const std::array<double, 3>& direction,
                               const std::array<double, 3>& up, int viewportWidth,
                               int viewportHeight, double fieldOfViewDegrees) {
    const auto view = openbus::rendering::makeLookDirectionMatrix(cameraPosition, direction, up);
    const auto projection = openbus::rendering::makePerspectiveMatrix(
        static_cast<double>(viewportWidth), static_cast<double>(viewportHeight),
        fieldOfViewDegrees);
    return makeDiligentCameraView(view, projection, cameraPosition, viewportWidth, viewportHeight);
}

inline DiligentCameraView makeDiligentCameraView(const openbus::map::MapTrafficCameraView& source) {
    if (!std::all_of(source.viewProjection.begin(), source.viewProjection.end(),
                     [](double value) { return std::isfinite(value); })) {
        throw std::invalid_argument("Reflection camera snapshot has an invalid view-projection");
    }
    return makeDiligentCameraView(source.view, source.projection, source.position,
                                  source.viewportWidth, source.viewportHeight);
}

} // namespace openbus::app
