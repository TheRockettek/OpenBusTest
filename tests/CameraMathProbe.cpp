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
    return 0;
}