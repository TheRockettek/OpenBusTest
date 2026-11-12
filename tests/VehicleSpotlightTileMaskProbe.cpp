#include "render/VehicleSpotlightTileMask.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <numbers>
#include <random>
#include <vector>

namespace {

bool hasLight(const openbus::rendering::detail::VehicleSpotlightTileMask& mask, int tileX,
              int tileY, std::size_t lightIndex) {
    const std::size_t tile = static_cast<std::size_t>(tileY) * mask.columns + tileX;
    const std::size_t word = lightIndex / 32;
    const std::uint32_t bit = std::uint32_t{1} << (lightIndex % 32);
    return (mask.words[tile * static_cast<std::size_t>(mask.wordsPerTile) + word] & bit) != 0;
}

} // namespace

int main() {
    using namespace openbus::rendering;
    using openbus::rendering::detail::buildVehicleSpotlightTileMask;

    Matrix4 projection = {};
    projection[0] = 1.0;
    projection[5] = 1.0;
    projection[10] = -1.02;
    projection[11] = -1.0;
    projection[14] = -0.202;
    const std::array<int, 4> viewport = {0, 0, 64, 64};

    VehicleSpotlight centered;
    centered.position = {0.0, 0.0, -5.0};
    centered.direction = {0.0, 0.0, -1.0};
    centered.range = 0.25;
    centered.outerAngleDegrees = 40.0;
    VehicleSpotlight offscreen = centered;
    offscreen.position = {100.0, 0.0, -5.0};
    VehicleSpotlight behind = centered;
    behind.position = {0.0, 0.0, 5.0};
    const std::vector<VehicleSpotlight> lights = {centered, offscreen, behind};
    const auto mask = buildVehicleSpotlightTileMask(lights, projection, viewport, 16);
    if (!mask.enabled || mask.columns != 2 || mask.rows != 2 ||
        mask.maximumLightsPerTile != 1 || mask.assignedReferences != 4) {
        std::cerr << "unexpected spotlight tile mask dimensions or counts\n";
        return 1;
    }
    for (int tileY = 0; tileY < 2; ++tileY) {
        for (int tileX = 0; tileX < 2; ++tileX) {
            if (!hasLight(mask, tileX, tileY, 0) || hasLight(mask, tileX, tileY, 1) ||
                hasLight(mask, tileX, tileY, 2)) {
                std::cerr << "spotlight tile assignment was not conservative and selective\n";
                return 1;
            }
        }
    }

    const std::vector<VehicleSpotlight> singleLight = {centered};
    const auto sparseMask =
        buildVehicleSpotlightTileMask(singleLight, projection, {0, 0, 128, 128}, 16, 0.75);
    if (!sparseMask.enabled || sparseMask.assignedReferences >= 16) {
        std::cerr << "sparse spotlight mask was not retained under the occupancy threshold\n";
        return 1;
    }
    VehicleSpotlight narrowCone = centered;
    narrowCone.range = 1.0;
    narrowCone.outerAngleDegrees = 2.0;
    const auto narrowMask =
        buildVehicleSpotlightTileMask({&narrowCone, 1}, projection, {0, 0, 128, 128}, 64, 0.75);
    if (!narrowMask.enabled || narrowMask.assignedReferences >= 16) {
        std::cerr << "narrow spotlight cone did not retain sparse tile coverage\n";
        return 1;
    }

    std::vector<VehicleSpotlight> wordBoundaryLights(33, behind);
    wordBoundaryLights.back() = centered;
    const auto wordBoundaryMask =
        buildVehicleSpotlightTileMask(wordBoundaryLights, projection, viewport, 32);
    if (!wordBoundaryMask.enabled || wordBoundaryMask.wordsPerTile != 2 ||
        wordBoundaryMask.assignedReferences != 4) {
        std::cerr << "spotlight mask did not size multiple words per tile\n";
        return 1;
    }
    for (int tileY = 0; tileY < 2; ++tileY) {
        for (int tileX = 0; tileX < 2; ++tileX) {
            if (!hasLight(wordBoundaryMask, tileX, tileY, 32) ||
                hasLight(wordBoundaryMask, tileX, tileY, 0)) {
                std::cerr << "spotlight mask lost the first index in a new word\n";
                return 1;
            }
        }
    }

    VehicleSpotlight nearPlane = centered;
    nearPlane.position = {0.0, 0.0, -0.1};
    nearPlane.direction = {0.0, 0.0, 1.0};
    nearPlane.range = 0.05;
    const std::vector<VehicleSpotlight> nearPlaneLights = {nearPlane};
    const auto nearMask =
        buildVehicleSpotlightTileMask(nearPlaneLights, projection, viewport, 4);
    if (!nearMask.enabled || nearMask.assignedReferences != 4 ||
        nearMask.maximumLightsPerTile != 1) {
        std::cerr << "near-plane spotlight did not conservatively cover the viewport\n";
        return 1;
    }
    if (buildVehicleSpotlightTileMask(nearPlaneLights, projection, viewport, 4, 0.75).enabled) {
        std::cerr << "dense spotlight mask did not request the global evaluation path\n";
        return 1;
    }

    VehicleSpotlight crossing = centered;
    crossing.position = {-0.25, 0, 0.1};
    crossing.direction = {0, 0, -1};
    crossing.range = 2;
    crossing.outerAngleDegrees = 30;
    const auto crossingMask =
        buildVehicleSpotlightTileMask({&crossing, 1}, projection, {0, 0, 128, 128}, 16, 0.75);
    if (!crossingMask.enabled || crossingMask.assignedReferences >= 16) {
        std::cerr << "near-plane crossing did not retain a sparse conservative mask\n";
        return 1;
    }

    std::mt19937 generator(19371);
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    std::size_t visibleSamples = 0;
    for (int lightNumber = 0; lightNumber < 300; ++lightNumber) {
        VehicleSpotlight light;
        light.position = {2 * unit(generator) - 1, 2 * unit(generator) - 1,
                          2 * unit(generator) - 1};
        const double azimuth = 2 * std::numbers::pi * unit(generator);
        const double directionZ = 2 * unit(generator) - 1;
        const double radial = std::sqrt(1 - directionZ * directionZ);
        light.direction = {radial * std::cos(azimuth), radial * std::sin(azimuth), directionZ};
        light.range = 0.1 + 2 * unit(generator);
        light.outerAngleDegrees = 1 + 158 * unit(generator);
        const std::array<double, 3> right = {-std::sin(azimuth), std::cos(azimuth), 0};
        const std::array<double, 3> up = {-directionZ * std::cos(azimuth),
                                          -directionZ * std::sin(azimuth), radial};
        const auto candidate =
            buildVehicleSpotlightTileMask({&light, 1}, projection, {0, 0, 128, 128}, 16);
        if (!candidate.enabled) {
            std::cerr << "randomized spotlight mask unexpectedly fell back\n";
            return 1;
        }
        for (int sample = 0; sample < 256; ++sample) {
            const double angle =
                light.outerAngleDegrees * std::numbers::pi / 360.0 * std::sqrt(unit(generator));
            const double around = 2 * std::numbers::pi * unit(generator);
            const double distance = light.range * 4 * unit(generator);
            std::array<double, 3> point = {};
            for (std::size_t axis = 0; axis < 3; ++axis) {
                point[axis] = light.position[axis] +
                              distance * (light.direction[axis] * std::cos(angle) +
                                          std::sin(angle) * (right[axis] * std::cos(around) +
                                                             up[axis] * std::sin(around)));
            }
            const double w = -point[2];
            const double z = projection[10] * point[2] + projection[14];
            if (w <= 0 || z + w < 0)
                continue;
            const double x = point[0] / w;
            const double y = point[1] / w;
            if (x < -1 || x >= 1 || y < -1 || y >= 1)
                continue;
            ++visibleSamples;
            const int tileX = static_cast<int>((x + 1) * 2);
            const int tileY = static_cast<int>((y + 1) * 2);
            if (!hasLight(candidate, tileX, tileY, 0)) {
                std::cerr << "spotlight mask omitted a visible randomized cone sample\n";
                return 1;
            }
        }
    }
    if (visibleSamples < 100) {
        std::cerr << "randomized spotlight test did not cover enough visible samples\n";
        return 1;
    }

    const std::vector<VehicleSpotlight> centeredLights = {centered};
    const auto overflow = buildVehicleSpotlightTileMask(centeredLights, projection, viewport, 3);
    if (overflow.enabled || !overflow.words.empty()) {
        std::cerr << "spotlight tile-mask overflow did not request the global fallback\n";
        return 1;
    }
    const auto invalidViewport =
        buildVehicleSpotlightTileMask(centeredLights, projection, {0, 0, 0, 64}, 16);
    if (invalidViewport.enabled) {
        std::cerr << "invalid viewport enabled spotlight tile culling\n";
        return 1;
    }
    return 0;
}
