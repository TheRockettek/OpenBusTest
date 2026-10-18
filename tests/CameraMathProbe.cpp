#include "CameraMath.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>

namespace {

std::array<double, 3> rotateZ(const std::array<double, 3>& value, double angle) {
    const double cosine = std::cos(angle);
    const double sine = std::sin(angle);
    return {cosine * value[0] - sine * value[1], sine * value[0] + cosine * value[1],
            value[2]};
}

double dot(const std::array<double, 3>& left, const std::array<double, 3>& right) {
    return left[0] * right[0] + left[1] * right[1] + left[2] * right[2];
}

} // namespace

int main() {
    constexpr double pi = 3.14159265358979323846;
    openbus::rendering::setPerspective(1280.0, 720.0, 60.0);
    const openbus::rendering::Matrix4& projection = openbus::rendering::projectionMatrix();
    if (!std::all_of(projection.begin(), projection.end(), [](double value) {
            return std::isfinite(value);
        }) || projection[10] <= -1.0001 || projection[10] >= -1.0) {
        std::cerr << "Camera projection far clip is not configured for long map visibility\n";
        return 1;
    }
    const VehiclePlacement groundPlacement{{4.0, -7.0, 0.0}, 37.0};
    constexpr double modelOffsetZ = -0.4;
    constexpr double groundPlaneZ = 0.015;
    const VehiclePlacement modelRootPlacement =
        openbus::rendering::makeModelRootPlacement(groundPlacement, modelOffsetZ);
    const openbus::rendering::Matrix4 shadowTransform =
        openbus::rendering::makeGroundShadowTransform(modelRootPlacement, modelOffsetZ,
                                                       groundPlaneZ);
    const double shadowOriginWorldZ = modelRootPlacement.position[2] + modelOffsetZ +
                                      shadowTransform[14];
    if (std::abs(modelRootPlacement.position[2] - 0.4) > 1.0e-12 ||
        std::abs(shadowOriginWorldZ - groundPlaneZ) > 1.0e-12 ||
        shadowTransform[12] != 0.0 || shadowTransform[13] != 0.0) {
        std::cerr << "Static vehicle root or shadow was not aligned with its ground plane\n";
        return 1;
    }

    const double pan = 180.0 * pi / 180.0;
    const double tilt = -89.5 * pi / 180.0;
    const std::array<double, 3> direction = {
        std::cos(tilt) * std::cos(pan), std::cos(tilt) * std::sin(pan), std::sin(tilt)};
    const std::array<double, 3> up = {
        -std::sin(tilt) * std::cos(pan), -std::sin(tilt) * std::sin(pan), std::cos(tilt)};

    for (int step = 0; step < 32; ++step) {
        const double yaw = 2.0 * pi * step / 32.0;
        const std::array<double, 3> worldDirection = rotateZ(direction, yaw);
        const std::array<double, 3> worldUp = rotateZ(up, yaw);
        openbus::rendering::lookAt(0.0, 0.0, 0.0, worldDirection[0], worldDirection[1],
                                   worldDirection[2], worldUp[0], worldUp[1], worldUp[2]);
        const openbus::rendering::Matrix4& matrix = openbus::rendering::modelViewMatrix();
        const std::array<double, 3> actualUp = {matrix[1], matrix[5], matrix[9]};
        if (!std::all_of(matrix.begin(), matrix.end(), [](double value) {
                return std::isfinite(value);
            }) || dot(actualUp, worldUp) < 0.999999) {
            std::cerr << "Near-vertical camera basis changed during vehicle yaw\n";
            return 1;
        }
    }
    openbus::rendering::lookAt(4.0, -2.0, 7.0, 4.0, -2.0, 7.0);
    const openbus::rendering::Matrix4& coincidentView =
        openbus::rendering::modelViewMatrix();
    if (!std::all_of(coincidentView.begin(), coincidentView.end(), [](double value) {
            return std::isfinite(value);
        })) {
        std::cerr << "Coincident camera eye and target produced a non-finite view matrix\n";
        return 1;
    }

    using openbus::rendering::FreeCameraPose;
    const std::array<double, 3> forward = openbus::rendering::freeCameraForward(0.0, 0.0);
    if (std::abs(forward[0] - 1.0) > 1.0e-12 || std::abs(forward[1]) > 1.0e-12 ||
        std::abs(forward[2]) > 1.0e-12) {
        std::cerr << "Free camera yaw zero must face world +X\n";
        return 1;
    }
    const FreeCameraPose start{{1.0, 2.0, 3.0}, pi / 2.0, 0.0};
    const FreeCameraPose forwardStep =
        openbus::rendering::moveFreeCamera(start, 1.0, 0.0, 0.0, 0.1, 12.0);
    if (std::abs(forwardStep.position[0] - 1.0) > 1.0e-12 ||
        std::abs(forwardStep.position[1] - 3.2) > 1.0e-12 ||
        std::abs(forwardStep.position[2] - 3.0) > 1.0e-12) {
        std::cerr << "Free camera forward movement did not follow its world-space heading\n";
        return 1;
    }
    const FreeCameraPose strafeRight =
        openbus::rendering::moveFreeCamera(FreeCameraPose{{}, 0.0, 0.0}, 0.0, 1.0, 0.0,
                                           0.1, 12.0);
    const FreeCameraPose strafeLeft =
        openbus::rendering::moveFreeCamera(FreeCameraPose{{}, 0.0, 0.0}, 0.0, -1.0, 0.0,
                                           0.1, 12.0);
    if (std::abs(strafeRight.position[1] + 1.2) > 1.0e-12 ||
        std::abs(strafeLeft.position[1] - 1.2) > 1.0e-12) {
        std::cerr << "Free camera left/right strafe direction is reversed\n";
        return 1;
    }
    const FreeCameraPose diagonalStep =
        openbus::rendering::moveFreeCamera(start, 1.0, 1.0, 0.0, 0.1, 12.0);
    const double diagonalDistance = std::sqrt(
        std::pow(diagonalStep.position[0] - start.position[0], 2.0) +
        std::pow(diagonalStep.position[1] - start.position[1], 2.0) +
        std::pow(diagonalStep.position[2] - start.position[2], 2.0));
    if (std::abs(diagonalDistance - 1.2) > 1.0e-12) {
        std::cerr << "Free camera diagonal movement exceeded normal movement speed\n";
        return 1;
    }
    const FreeCameraPose clampedStep =
        openbus::rendering::moveFreeCamera(start, 0.0, 0.0, 1.0, 4.0, 12.0);
    if (std::abs(clampedStep.position[2] - 6.0) > 1.0e-12) {
        std::cerr << "Free camera movement did not cap long frame-time steps\n";
        return 1;
    }
    {
        using openbus::rendering::mirrorReflectionDirection;
        const auto near = [](const std::array<double, 3>& left,
                             const std::array<double, 3>& right) {
            return std::abs(left[0] - right[0]) < 1.0e-9 && std::abs(left[1] - right[1]) < 1.0e-9 &&
                   std::abs(left[2] - right[2]) < 1.0e-9;
        };
        // Mirror at the origin seen head-on from -X, so it shows straight back toward -X.
        const std::array<double, 3> mirror = {0.0, 0.0, 0.0};
        const std::array<double, 3> authored = {-1.0, 0.0, 0.0};
        const std::array<double, 3> referenceEye = {-1.0, 0.0, 0.0};
        if (!near(mirrorReflectionDirection(mirror, authored, referenceEye, referenceEye),
                  authored)) {
            std::cerr << "Mirror direction changed for the reference viewer\n";
            return 1;
        }
        // Moving the viewer to +Y swings the reflected view to -Y (angle of incidence equals
        // angle of reflection).
        const std::array<double, 3> shifted =
            mirrorReflectionDirection(mirror, authored, referenceEye, {-1.0, 1.0, 0.0});
        const double inverseRoot2 = 1.0 / std::sqrt(2.0);
        if (!near(shifted, {-inverseRoot2, -inverseRoot2, 0.0})) {
            std::cerr << "Mirror direction did not follow the viewer position\n";
            return 1;
        }
        // A viewer behind the mirror plane keeps the authored direction.
        if (!near(mirrorReflectionDirection(mirror, authored, referenceEye, {1.0, 0.5, 0.0}),
                  authored) ||
            !near(mirrorReflectionDirection(mirror, authored, mirror, {-1.0, 1.0, 0.0}),
                  authored)) {
            std::cerr << "Degenerate mirror inputs did not fall back to the authored direction\n";
            return 1;
        }
    }
    return 0;
}