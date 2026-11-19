#include "render/SpotlightShadowCamera.h"
#include "render/MatrixMath.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <numbers>
#include <random>
#include <stdexcept>

namespace {
bool close(double actual, double expected, double tolerance = 1e-8) {
    return std::abs(actual - expected) <= tolerance;
}

bool checkCamera(const openbus::rendering::VehicleSpotlight& light) {
    using namespace openbus::rendering;
    const auto camera = makeSpotlightShadowCamera(light);
    if (!camera)
        return false;
    const double length = std::hypot(light.direction[0], light.direction[1], light.direction[2]);
    const std::array<double, 3> forward = {light.direction[0] / length,
                                            light.direction[1] / length,
                                            light.direction[2] / length};
    const std::array<double, 3> side = {camera->view[0], camera->view[4], camera->view[8]};
    const auto project = [&](double distance, double sideOffset) {
        std::array<double, 4> point = {0.0, 0.0, 0.0, 1.0};
        for (std::size_t i = 0; i < 3; ++i)
            point[i] = light.position[i] + distance * forward[i] + sideOffset * side[i];
        return transformPoint(camera->projection, transformPoint(camera->view, point));
    };
    const double middle = light.range;
    const auto center = project(middle, 0.0);
    const auto edge = project(
        middle, middle * std::tan(light.outerAngleDegrees * std::numbers::pi / 360.0));
    const auto nearPoint = project(camera->nearPlane, 0.0);
    const auto farPoint = project(camera->farPlane, 0.0);
    return close(center[0] / center[3], 0.0) && close(center[1] / center[3], 0.0) &&
           close(edge[0] / edge[3], 1.0) && close(edge[1] / edge[3], 0.0) &&
           close(nearPoint[2] / nearPoint[3], -1.0, 1e-7) &&
           close(farPoint[2] / farPoint[3], 1.0);
}
} // namespace

int main() {
    using namespace openbus::rendering;
    const Matrix4 previousView = modelViewMatrix();
    const Matrix4 previousProjection = projectionMatrix();
    VehicleSpotlight light;
    light.position = {14.0, -21.0, 3.0};
    light.direction = {0.0, 0.0, 1.0};
    light.range = 20.0;
    light.outerAngleDegrees = 60.0;
    bool ok = checkCamera(light);
    light.direction = {0.0, 0.0, -1.0};
    ok = checkCamera(light) && ok;
    light.direction = {1e300, -2e300, 3e300};
    ok = checkCamera(light) && ok;
    std::mt19937 generator(117);
    std::uniform_real_distribution<double> direction(-1.0, 1.0);
    std::uniform_real_distribution<double> angle(5.0, 170.0);
    for (int i = 0; i < 1000; ++i) {
        light.direction = {direction(generator), direction(generator), direction(generator)};
        light.outerAngleDegrees = angle(generator);
        ok = checkCamera(light) && ok;
    }
    light.outerAngleDegrees = 0.0;
    ok = !makeSpotlightShadowCamera(light) && ok;
    light.outerAngleDegrees = 180.0;
    ok = !makeSpotlightShadowCamera(light) && ok;
    light.outerAngleDegrees = 60.0;
    light.range = std::numeric_limits<float>::max();
    ok = !makeSpotlightShadowCamera(light) && ok;
    light.range = std::numeric_limits<double>::min();
    ok = !makeSpotlightShadowCamera(light) && ok;
    light.range = 20.0;
    light.direction = {0.0, 0.0, 0.0};
    try {
        static_cast<void>(makeSpotlightShadowCamera(light));
        ok = false;
    } catch (const std::invalid_argument&) {
    }
    light.direction = {1.0, 0.0, 0.0};
    light.outerAngleDegrees = std::numeric_limits<double>::quiet_NaN();
    try {
        static_cast<void>(makeSpotlightShadowCamera(light));
        ok = false;
    } catch (const std::invalid_argument&) {
    }
    ok = modelViewMatrix() == previousView && projectionMatrix() == previousProjection && ok;
    if (!ok)
        std::cerr << "Spotlight shadow camera projection regression\n";
    return ok ? 0 : 1;
}
