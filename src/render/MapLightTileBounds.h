#pragma once

#include "render/VehicleSpotlightTileMask.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace openbus::rendering::detail {

// The projected AABB encloses the sphere. Ambiguous numerical cases
// deliberately cover the whole viewport rather than dropping a light.
inline VehicleSpotlightTileBounds mapLightTileBounds(const std::array<float, 3>& center,
                                                     double radius, const Matrix4& projection,
                                                     int width, int height) {
    if (width <= 0 || height <= 0)
        throw std::invalid_argument("Invalid map-light tile viewport");
    const int columns = (width - 1) / VEHICLE_SPOTLIGHT_TILE_SIZE + 1;
    const int rows = (height - 1) / VEHICLE_SPOTLIGHT_TILE_SIZE + 1;
    const VehicleSpotlightTileBounds full{0, 0, columns - 1, rows - 1};
    if (!std::isfinite(radius) || radius < 0.0 ||
        !std::all_of(
            center.begin(), center.end(), [](float value) { return std::isfinite(value); }))
        return full;
    std::array<double, 16> matrix = {};
    for (std::size_t index = 0; index < matrix.size(); ++index) {
        matrix[index] = static_cast<float>(projection[index]);
        if (!std::isfinite(matrix[index]))
            return full;
    }
    // Cover rounding in the GPU's float position, radius and projection math.
    radius = std::nextafter(radius * (1.0 + 1.0e-5), std::numeric_limits<double>::infinity());
    if (!std::isfinite(radius))
        return full;
    std::array<std::array<double, 4>, 8> corners = {};
    double clipScale = 1.0;
    for (unsigned int corner = 0; corner < 8; ++corner) {
        std::array<double, 3> point = {};
        for (std::size_t axis = 0; axis < point.size(); ++axis)
            point[axis] =
                static_cast<double>(center[axis]) + ((corner & (1U << axis)) ? radius : -radius);
        auto& clip = corners[corner];
        for (std::size_t component = 0; component < clip.size(); ++component) {
            clip[component] = matrix[component] * point[0] + matrix[4 + component] * point[1] +
                              matrix[8 + component] * point[2] + matrix[12 + component];
            if (!std::isfinite(clip[component]))
                return full;
            clipScale = std::max(clipScale, std::abs(clip[component]));
        }
    }
    const double margin = clipScale * 1.0e-6;
    std::array<double, 8> distance = {};
    std::array<std::array<double, 4>, 20> clipped = {};
    std::size_t clippedCount = 0;
    for (std::size_t index = 0; index < corners.size(); ++index) {
        distance[index] = corners[index][2] + corners[index][3] + margin;
        if (distance[index] >= 0.0)
            clipped[clippedCount++] = corners[index];
    }
    for (unsigned int first = 0; first < 8; ++first) {
        for (unsigned int axis = 0; axis < 3; ++axis) {
            const unsigned int second = first ^ (1U << axis);
            if (first >= second || (distance[first] < 0.0) == (distance[second] < 0.0))
                continue;
            const double fraction = distance[first] / (distance[first] - distance[second]);
            auto& intersection = clipped[clippedCount++];
            for (std::size_t component = 0; component < intersection.size(); ++component)
                intersection[component] =
                    corners[first][component] +
                    fraction * (corners[second][component] - corners[first][component]);
        }
    }
    if (clippedCount == 0)
        return {0, 0, 0, -1};
    double minX = std::numeric_limits<double>::infinity();
    double minY = minX;
    double maxX = -minX;
    double maxY = -minX;
    for (std::size_t index = 0; index < clippedCount; ++index) {
        const auto& clip = clipped[index];
        if (clip[3] <= clipScale * 1.0e-8)
            return full;
        minX = std::min(minX, clip[0] / clip[3]);
        minY = std::min(minY, clip[1] / clip[3]);
        maxX = std::max(maxX, clip[0] / clip[3]);
        maxY = std::max(maxY, clip[1] / clip[3]);
    }
    if (!std::isfinite(minX) || !std::isfinite(minY) || !std::isfinite(maxX) ||
        !std::isfinite(maxY))
        return full;
    const double minPixelX = std::floor((minX * 0.5 + 0.5) * width) - 1.0;
    const double minPixelY = std::floor((minY * 0.5 + 0.5) * height) - 1.0;
    const double maxPixelX = std::ceil((maxX * 0.5 + 0.5) * width) + 1.0;
    const double maxPixelY = std::ceil((maxY * 0.5 + 0.5) * height) + 1.0;
    if (minPixelX > width - 1 || minPixelY > height - 1 || maxPixelX < 0 || maxPixelY < 0)
        return {0, 0, 0, -1};
    const auto tile = [](double pixel, int extent, int maximum) {
        return std::clamp(
            static_cast<int>(std::floor(std::clamp(pixel, 0.0, static_cast<double>(extent - 1)) /
                                        VEHICLE_SPOTLIGHT_TILE_SIZE)),
            0, maximum);
    };
    return {tile(minPixelX, width, columns - 1), tile(minPixelY, height, rows - 1),
            tile(maxPixelX, width, columns - 1), tile(maxPixelY, height, rows - 1)};
}

} // namespace openbus::rendering::detail
