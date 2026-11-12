#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace openbus::rendering::map_detail {

std::uint64_t tileKey(int x, int y);

class StaticLightTileIndex {
  public:
    void add(std::uint64_t tile, std::size_t lightIndex) {
        byTile_[tile].push_back(lightIndex);
        allIndices_.push_back(lightIndex);
        allIndicesDirty_ = true;
        dirty_ = true;
    }

    const std::vector<std::size_t>& allIndices() {
        if (allIndicesDirty_) {
            std::sort(allIndices_.begin(), allIndices_.end());
            allIndices_.erase(std::unique(allIndices_.begin(), allIndices_.end()),
                              allIndices_.end());
            allIndicesDirty_ = false;
        }
        return allIndices_;
    }

    void addInfluence(std::size_t lightIndex, const std::array<double, 3>& position, double radius,
                      double tileSize) {
        if (!std::isfinite(position[0]) || !std::isfinite(position[1]) || !std::isfinite(radius) ||
            radius <= 0.0 || !std::isfinite(tileSize) || tileSize <= 0.0) {
            throw std::invalid_argument("Invalid static light influence");
        }
        const double minX = std::floor((position[0] - radius) / tileSize);
        const double maxX = std::floor((position[0] + radius) / tileSize);
        const double minY = std::floor((position[1] - radius) / tileSize);
        const double maxY = std::floor((position[1] + radius) / tileSize);
        constexpr double minTile = static_cast<double>(std::numeric_limits<int>::min());
        constexpr double maxTile = static_cast<double>(std::numeric_limits<int>::max());
        constexpr double maxIndexedTiles = 1024.0;
        if (!std::isfinite(minX) || !std::isfinite(maxX) || !std::isfinite(minY) ||
            !std::isfinite(maxY) || minX < minTile || maxX > maxTile || minY < minTile ||
            maxY > maxTile || (maxX - minX + 1.0) * (maxY - minY + 1.0) > maxIndexedTiles) {
            globalIndices_.push_back(lightIndex);
            dirty_ = true;
            return;
        }
        for (std::int64_t x = static_cast<int>(minX); x <= static_cast<int>(maxX); ++x) {
            for (std::int64_t y = static_cast<int>(minY); y <= static_cast<int>(maxY); ++y) {
                byTile_[tileKey(static_cast<int>(x), static_cast<int>(y))].push_back(lightIndex);
            }
        }
        dirty_ = true;
    }

    const std::vector<std::size_t>&
    activeIndices(const std::unordered_set<std::uint64_t>& activeTiles) {
        if (dirty_ || activeTiles_ != activeTiles) {
            std::vector<std::size_t> indices;
            if (!activeTiles.empty()) {
                indices = globalIndices_;
            }
            for (const std::uint64_t tile : activeTiles) {
                const auto found = byTile_.find(tile);
                if (found != byTile_.end()) {
                    indices.insert(indices.end(), found->second.begin(), found->second.end());
                }
            }
            std::sort(indices.begin(), indices.end());
            indices.erase(std::unique(indices.begin(), indices.end()), indices.end());
            activeTiles_ = activeTiles;
            activeIndices_ = std::move(indices);
            dirty_ = false;
        }
        return activeIndices_;
    }

  private:
    std::unordered_map<std::uint64_t, std::vector<std::size_t>> byTile_;
    std::vector<std::size_t> allIndices_;
    bool allIndicesDirty_ = false;
    std::vector<std::size_t> globalIndices_;
    std::unordered_set<std::uint64_t> activeTiles_;
    std::vector<std::size_t> activeIndices_;
    bool dirty_ = true;
};

} // namespace openbus::rendering::map_detail
