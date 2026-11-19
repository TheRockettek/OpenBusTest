#include "render/MapLightTileBounds.h"

#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <numbers>
#include <random>
#include <stdexcept>

int main() {
    using namespace openbus::rendering;
    using namespace openbus::rendering::detail;
    Matrix4 projection = {};
    projection[0] = 1.0;
    projection[5] = 1.0;
    projection[10] = -1.02;
    projection[11] = -1.0;
    projection[14] = -0.202;
    constexpr int width = 193;
    constexpr int height = 129;
    const auto full = mapLightTileBounds({0, 0, -0.1F}, 1.0, projection, width, height);
    if (full.minimumX != 0 || full.minimumY != 0 || full.maximumX != 6 || full.maximumY != 4) {
        std::cerr << "near-plane crossing must cover all tiles\n";
        return 1;
    }
    const auto offscreen = mapLightTileBounds({1000, 0, -5}, 1.0, projection, width, height);
    if (offscreen.maximumY >= offscreen.minimumY) {
        std::cerr << "offscreen sphere should not cover any tiles\n";
        return 1;
    }
    const auto crossingOffscreen =
        mapLightTileBounds({1000, 0, -0.1F}, 1.0, projection, width, height);
    if (crossingOffscreen.maximumY >= crossingOffscreen.minimumY) {
        std::cerr << "near-plane crossing outside the view should not cover tiles\n";
        return 1;
    }
    const auto uncertain = mapLightTileBounds({0, 0, -5}, std::numeric_limits<double>::infinity(),
                                              projection, width, height);
    if (uncertain.maximumX != 6 || uncertain.maximumY != 4) {
        std::cerr << "unrepresentable radius must cover all tiles\n";
        return 1;
    }
    bool rejectedViewport = false;
    try {
        static_cast<void>(mapLightTileBounds({0, 0, -5}, 1.0, projection, 0, height));
    } catch (const std::invalid_argument&) {
        rejectedViewport = true;
    }
    if (!rejectedViewport) {
        std::cerr << "invalid viewport was not rejected\n";
        return 1;
    }
    std::mt19937 generator(20167);
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    std::size_t visibleSamples = 0;
    Matrix4 asymmetric = projection;
    asymmetric[0] = 1.3;
    asymmetric[5] = 0.8;
    asymmetric[8] = 0.25;
    asymmetric[9] = -0.2;
    Matrix4 orthographic = {};
    orthographic[0] = 0.1;
    orthographic[5] = 0.1;
    orthographic[10] = -0.02;
    orthographic[15] = 1.0;
    const std::array<Matrix4, 3> projections = {projection, asymmetric, orthographic};
    for (const auto& testProjection : projections) {
        for (int light = 0; light < 1000; ++light) {
            const std::array<float, 3> center = {static_cast<float>(unit(generator) * 30 - 15),
                                                 static_cast<float>(unit(generator) * 20 - 10),
                                                 static_cast<float>(-0.1 - unit(generator) * 40)};
            const double radius = 0.01 + unit(generator) * 4;
            const auto bounds = mapLightTileBounds(center, radius, testProjection, width, height);
            for (int sample = 0; sample < 256; ++sample) {
                const double azimuth = 2 * std::numbers::pi * unit(generator);
                const double z = 2 * unit(generator) - 1;
                const double radial = std::sqrt(1 - z * z);
                const double distance = radius * std::cbrt(unit(generator));
                const std::array<double, 3> point = {
                    center[0] + distance * radial * std::cos(azimuth),
                    center[1] + distance * radial * std::sin(azimuth), center[2] + distance * z};
                std::array<double, 4> clip = {};
                for (std::size_t component = 0; component < clip.size(); ++component) {
                    clip[component] = static_cast<float>(testProjection[component]) * point[0] +
                                      static_cast<float>(testProjection[4 + component]) * point[1] +
                                      static_cast<float>(testProjection[8 + component]) * point[2] +
                                      static_cast<float>(testProjection[12 + component]);
                }
                const double w = clip[3];
                const double clipZ = clip[2];
                if (w <= 0 || clipZ + w < 0)
                    continue;
                const double x = clip[0] / w;
                const double y = clip[1] / w;
                if (x < -1 || x >= 1 || y < -1 || y >= 1)
                    continue;
                ++visibleSamples;
                const int tileX =
                    static_cast<int>((x + 1) * 0.5 * width) / VEHICLE_SPOTLIGHT_TILE_SIZE;
                const int tileY =
                    static_cast<int>((y + 1) * 0.5 * height) / VEHICLE_SPOTLIGHT_TILE_SIZE;
                if (tileX < bounds.minimumX || tileX > bounds.maximumX || tileY < bounds.minimumY ||
                    tileY > bounds.maximumY) {
                    std::cerr << "sphere projection dropped a visible tile\n";
                    return 1;
                }
            }
        }
    }
    if (visibleSamples < 10000) {
        std::cerr << "not enough visible sphere samples were checked\n";
        return 1;
    }
    return 0;
}
