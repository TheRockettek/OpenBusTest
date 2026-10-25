#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <vector>

namespace openbus::map {

inline constexpr double OMSI_TILE_SIZE_METERS = 300.0;

inline double mapWorldRowWidth(double row) {
    const double width =
        (40075016.69 / 65536.0) / std::cosh(2.0 * std::numbers::pi * row / 65536.0);
    if (!std::isfinite(width) || width <= 0.0) {
        throw std::invalid_argument("World-coordinate map row has no finite tile width");
    }
    return width;
}

struct MapTileTransform {
    double originX = 0.0;
    double originY = 0.0;
    double scaleX = 1.0;
    double scaleY = 1.0;

    std::array<double, 2> toWorld(double x, double y) const {
        return {originX + x * scaleX, originY + y * scaleY};
    }

    std::array<double, 2> toLocal(double x, double y) const {
        return {(x - originX) / scaleX, (y - originY) / scaleY};
    }
};

// Keep one continuous metric grid, scaling each real-world tile's authored frame onto it.
// Global tile indices are retained; this is not a floating-origin/rebasing mechanism.
struct MapCoordinates {
    double tileSize = OMSI_TILE_SIZE_METERS;
    bool worldCoordinates = false;

    static MapCoordinates forRows(bool worldCoordinates, std::vector<int> rows) {
        if (!worldCoordinates) {
            return {};
        }
        if (rows.empty()) {
            throw std::invalid_argument("World-coordinate map requires at least one tile");
        }
        std::sort(rows.begin(), rows.end());
        const double row = rows[rows.size() / 2];
        return {(mapWorldRowWidth(row) + mapWorldRowWidth(row + 1.0)) * 0.5, true};
    }

    MapTileTransform tile(int x, int y) const {
        if (!std::isfinite(tileSize) || tileSize <= 0.0) {
            throw std::invalid_argument("Map grid tile size must be finite and positive");
        }
        if (!worldCoordinates) {
            return {x * tileSize, y * tileSize};
        }
        const double lower = mapWorldRowWidth(y);
        const double upper = mapWorldRowWidth(static_cast<double>(y) + 1.0);
        return {x * tileSize, y * tileSize, tileSize / ((lower + upper) * 0.5), tileSize / upper};
    }
};

} // namespace openbus::map
