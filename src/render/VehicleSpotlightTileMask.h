#pragma once

#include "render/CoreRenderer.h"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numbers>
#include <span>
#include <vector>

namespace openbus::rendering::detail {

inline constexpr int VEHICLE_SPOTLIGHT_TILE_SIZE = 32;

struct VehicleSpotlightTileBounds {
    std::int32_t minimumX = 0;
    std::int32_t minimumY = 0;
    std::int32_t maximumX = -1;
    std::int32_t maximumY = -1;
};
static_assert(sizeof(VehicleSpotlightTileBounds) == 4 * sizeof(std::int32_t));

struct VehicleSpotlightTileMask {
    bool enabled = false;
    int viewportX = 0;
    int viewportY = 0;
    int columns = 0;
    int rows = 0;
    int wordsPerTile = 0;
    std::vector<VehicleSpotlightTileBounds> bounds;
    std::vector<std::uint32_t> words;
    std::size_t assignedReferences = 0;
    std::size_t maximumLightsPerTile = 0;
};

inline VehicleSpotlightTileMask
buildVehicleSpotlightTileMask(std::span<const VehicleSpotlight> lights, const Matrix4& projection,
                              const std::array<int, 4>& viewport,
                              std::size_t maximumTextureBufferTexels,
                              double maximumAverageCoverage = 1.0, bool materializeWords = true) {
    using TileBounds = VehicleSpotlightTileBounds;

    VehicleSpotlightTileMask result;
    result.viewportX = viewport[0];
    result.viewportY = viewport[1];
    if (lights.empty() || viewport[2] <= 0 || viewport[3] <= 0 || maximumTextureBufferTexels == 0 ||
        !std::isfinite(maximumAverageCoverage) || maximumAverageCoverage <= 0.0 ||
        maximumAverageCoverage > 1.0) {
        return result;
    }

    const int width = viewport[2];
    const int height = viewport[3];
    result.columns = (width - 1) / VEHICLE_SPOTLIGHT_TILE_SIZE + 1;
    result.rows = (height - 1) / VEHICLE_SPOTLIGHT_TILE_SIZE + 1;
    if (static_cast<std::size_t>(result.columns) >
        std::numeric_limits<std::size_t>::max() / static_cast<std::size_t>(result.rows)) {
        return result;
    }
    const std::size_t tileCount =
        static_cast<std::size_t>(result.columns) * static_cast<std::size_t>(result.rows);
    const std::size_t wordsPerTile = lights.size() / 32 + (lights.size() % 32 != 0 ? 1 : 0);
    if (tileCount == 0 || wordsPerTile == 0 ||
        tileCount > maximumTextureBufferTexels / wordsPerTile ||
        wordsPerTile > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
        return result;
    }
    if (!std::all_of(projection.begin(), projection.end(),
                     [](double value) { return std::isfinite(value); })) {
        return result;
    }

    const TileBounds fullBounds{0, 0, result.columns - 1, result.rows - 1};
    const auto boundsFromNdc = [&](double minimumNdcX, double minimumNdcY, double maximumNdcX,
                                   double maximumNdcY) {
        if (!std::isfinite(minimumNdcX) || !std::isfinite(minimumNdcY) ||
            !std::isfinite(maximumNdcX) || !std::isfinite(maximumNdcY)) {
            return fullBounds;
        }
        if (maximumNdcX < -1.0 || minimumNdcX > 1.0 || maximumNdcY < -1.0 || minimumNdcY > 1.0) {
            return TileBounds{0, 0, 0, -1};
        }

        const double minimumPixelX = std::floor((minimumNdcX * 0.5 + 0.5) * width) - 1.0;
        const double minimumPixelY = std::floor((minimumNdcY * 0.5 + 0.5) * height) - 1.0;
        const double maximumPixelX = std::ceil((maximumNdcX * 0.5 + 0.5) * width) + 1.0;
        const double maximumPixelY = std::ceil((maximumNdcY * 0.5 + 0.5) * height) + 1.0;
        if (minimumPixelX > width - 1 || minimumPixelY > height - 1 || maximumPixelX < 0.0 ||
            maximumPixelY < 0.0) {
            return TileBounds{0, 0, 0, -1};
        }
        return TileBounds{std::clamp(static_cast<int>(std::floor(std::max(0.0, minimumPixelX) /
                                                                 VEHICLE_SPOTLIGHT_TILE_SIZE)),
                                     0, result.columns - 1),
                          std::clamp(static_cast<int>(std::floor(std::max(0.0, minimumPixelY) /
                                                                 VEHICLE_SPOTLIGHT_TILE_SIZE)),
                                     0, result.rows - 1),
                          std::clamp(static_cast<int>(std::floor(
                                         std::min(static_cast<double>(width - 1), maximumPixelX) /
                                         VEHICLE_SPOTLIGHT_TILE_SIZE)),
                                     0, result.columns - 1),
                          std::clamp(static_cast<int>(std::floor(
                                         std::min(static_cast<double>(height - 1), maximumPixelY) /
                                         VEHICLE_SPOTLIGHT_TILE_SIZE)),
                                     0, result.rows - 1)};
    };
    const auto projectedBounds = [&](const VehicleSpotlight& light) {
        const auto allTiles = [&] { return fullBounds; };
        const double length = light.range * 4.0;
        const double directionLength =
            std::hypot(light.direction[0], light.direction[1], light.direction[2]);
        const double halfAngle = light.outerAngleDegrees * std::numbers::pi / 360.0;
        if (!std::isfinite(length) || length <= 0.0 || !std::isfinite(directionLength) ||
            directionLength <= 1.0e-12 || !std::isfinite(halfAngle) ||
            light.outerAngleDegrees < 0.0 || light.outerAngleDegrees > 180.0 ||
            !std::all_of(light.position.begin(), light.position.end(),
                         [](double value) { return std::isfinite(value); })) {
            return allTiles();
        }
        if (halfAngle >= std::numbers::pi / 2.0) {
            return allTiles();
        }

        const double baseRadius = length * std::tan(halfAngle);
        const std::array<double, 3> direction = {light.direction[0] / directionLength,
                                                 light.direction[1] / directionLength,
                                                 light.direction[2] / directionLength};
        const std::array<double, 3> helperAxis = std::abs(direction[2]) < 0.9
                                                     ? std::array<double, 3>{0.0, 0.0, 1.0}
                                                     : std::array<double, 3>{0.0, 1.0, 0.0};
        std::array<double, 3> right = {direction[1] * helperAxis[2] - direction[2] * helperAxis[1],
                                       direction[2] * helperAxis[0] - direction[0] * helperAxis[2],
                                       direction[0] * helperAxis[1] - direction[1] * helperAxis[0]};
        const double rightLength = std::hypot(right[0], right[1], right[2]);
        if (!std::isfinite(baseRadius) || !std::isfinite(rightLength) || rightLength <= 1.0e-12) {
            return allTiles();
        }
        for (double& value : right) {
            value /= rightLength;
        }
        const std::array<double, 3> up = {right[1] * direction[2] - right[2] * direction[1],
                                          right[2] * direction[0] - right[0] * direction[2],
                                          right[0] * direction[1] - right[1] * direction[0]};
        std::array<std::array<double, 3>, 5> corners = {};
        corners[0] = light.position;
        for (std::size_t corner = 0; corner < 4; ++corner) {
            const double rightSign = (corner & 1U) == 0 ? -1.0 : 1.0;
            const double upSign = (corner & 2U) == 0 ? -1.0 : 1.0;
            for (std::size_t axis = 0; axis < 3; ++axis) {
                corners[corner + 1][axis] =
                    light.position[axis] + direction[axis] * length +
                    baseRadius * (rightSign * right[axis] + upSign * up[axis]);
            }
        }

        std::array<std::array<double, 4>, 5> clipCorners = {};
        double clipScale = 1.0;
        for (std::size_t corner = 0; corner < corners.size(); ++corner) {
            for (std::size_t component = 0; component < 4; ++component) {
                clipCorners[corner][component] = projection[component] * corners[corner][0] +
                                                 projection[4 + component] * corners[corner][1] +
                                                 projection[8 + component] * corners[corner][2] +
                                                 projection[12 + component];
                if (!std::isfinite(clipCorners[corner][component])) {
                    return allTiles();
                }
                clipScale = std::max(clipScale, std::abs(clipCorners[corner][component]));
            }
        }
        const double margin = clipScale * 1.0e-9;
        std::array<double, 5> planeDistance = {};
        for (std::size_t corner = 0; corner < corners.size(); ++corner) {
            planeDistance[corner] = clipCorners[corner][2] + clipCorners[corner][3] + margin;
        }
        constexpr std::array<std::array<std::size_t, 2>, 8> edges = {
            {{0, 1}, {0, 2}, {0, 3}, {0, 4}, {1, 2}, {1, 3}, {2, 4}, {3, 4}}};
        std::array<std::array<double, 4>, 13> clipped = {};
        std::size_t clippedCount = 0;
        for (std::size_t corner = 0; corner < corners.size(); ++corner) {
            if (planeDistance[corner] >= 0.0) {
                clipped[clippedCount++] = clipCorners[corner];
            }
        }
        for (const auto& edge : edges) {
            const double first = planeDistance[edge[0]];
            const double second = planeDistance[edge[1]];
            if ((first < 0.0) == (second < 0.0)) {
                continue;
            }
            const double fraction = first / (first - second);
            std::array<double, 4> intersection = {};
            for (std::size_t component = 0; component < 4; ++component) {
                intersection[component] =
                    clipCorners[edge[0]][component] +
                    fraction * (clipCorners[edge[1]][component] - clipCorners[edge[0]][component]);
            }
            clipped[clippedCount++] = intersection;
        }
        if (clippedCount == 0) {
            return TileBounds{0, 0, 0, -1};
        }
        double minimumNdcX = std::numeric_limits<double>::infinity();
        double minimumNdcY = std::numeric_limits<double>::infinity();
        double maximumNdcX = -std::numeric_limits<double>::infinity();
        double maximumNdcY = -std::numeric_limits<double>::infinity();
        for (std::size_t index = 0; index < clippedCount; ++index) {
            const auto& corner = clipped[index];
            if (corner[3] <= 1.0e-9) {
                return allTiles();
            }
            const double ndcX = corner[0] / corner[3];
            const double ndcY = corner[1] / corner[3];
            if (!std::isfinite(ndcX) || !std::isfinite(ndcY)) {
                return allTiles();
            }
            minimumNdcX = std::min(minimumNdcX, ndcX);
            minimumNdcY = std::min(minimumNdcY, ndcY);
            maximumNdcX = std::max(maximumNdcX, ndcX);
            maximumNdcY = std::max(maximumNdcY, ndcY);
        }
        return boundsFromNdc(minimumNdcX, minimumNdcY, maximumNdcX, maximumNdcY);
    };

    const long double maximumReferences =
        static_cast<long double>(tileCount) * lights.size() * maximumAverageCoverage;
    const auto coveredTileCount = [](const TileBounds& bounds) {
        if (bounds.maximumX < bounds.minimumX || bounds.maximumY < bounds.minimumY) {
            return std::size_t{0};
        }
        return static_cast<std::size_t>(bounds.maximumX - bounds.minimumX + 1) *
               static_cast<std::size_t>(bounds.maximumY - bounds.minimumY + 1);
    };
    result.bounds.reserve(lights.size());
    std::size_t estimatedReferences = 0;
    for (const VehicleSpotlight& light : lights) {
        const TileBounds lightBounds = projectedBounds(light);
        result.bounds.push_back(lightBounds);
        const std::size_t lightTileCount = coveredTileCount(lightBounds);
        if (estimatedReferences > std::numeric_limits<std::size_t>::max() - lightTileCount) {
            return result;
        }
        estimatedReferences += lightTileCount;
        if (estimatedReferences > maximumReferences) {
            return result;
        }
    }

    result.wordsPerTile = static_cast<int>(wordsPerTile);
    result.assignedReferences = estimatedReferences;
    result.enabled = true;
    if (!materializeWords) {
        return result;
    }
    result.words.assign(tileCount * wordsPerTile, 0);
    for (std::size_t lightIndex = 0; lightIndex < result.bounds.size(); ++lightIndex) {
        const TileBounds& lightBounds = result.bounds[lightIndex];
        if (lightBounds.maximumX < lightBounds.minimumX ||
            lightBounds.maximumY < lightBounds.minimumY) {
            continue;
        }
        const std::size_t wordIndex = lightIndex / 32;
        const std::uint32_t bit = std::uint32_t{1} << (lightIndex % 32);
        for (int tileY = lightBounds.minimumY; tileY <= lightBounds.maximumY; ++tileY) {
            for (int tileX = lightBounds.minimumX; tileX <= lightBounds.maximumX; ++tileX) {
                const std::size_t tile = static_cast<std::size_t>(tileY) * result.columns + tileX;
                result.words[tile * wordsPerTile + wordIndex] |= bit;
            }
        }
    }

    for (std::size_t tile = 0; tile < tileCount; ++tile) {
        std::size_t count = 0;
        for (std::size_t word = 0; word < wordsPerTile; ++word) {
            count += std::popcount(result.words[tile * wordsPerTile + word]);
        }
        result.maximumLightsPerTile = std::max(result.maximumLightsPerTile, count);
    }
    return result;
}

} // namespace openbus::rendering::detail
