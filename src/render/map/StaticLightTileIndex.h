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
        maxIndex_ = std::max(maxIndex_, lightIndex);
        hasIndices_ = true;
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
        maxIndex_ = std::max(maxIndex_, lightIndex);
        hasIndices_ = true;
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
            // Sparse or unusually large identifiers use the exact sort/unique fallback.
            constexpr std::size_t maxDenseIndices = 4'000'000;
            const std::size_t denseLimit =
                allIndices_.size() <= (std::numeric_limits<std::size_t>::max() - 1024) / 4
                    ? allIndices_.size() * 4 + 1024
                    : std::numeric_limits<std::size_t>::max();
            const bool useDenseMarks =
                hasIndices_ && maxIndex_ < maxDenseIndices && maxIndex_ <= denseLimit;
            if (useDenseMarks) {
                if (marks_.size() <= maxIndex_) {
                    marks_.resize(maxIndex_ + 1, 0);
                }
                if (generation_ == std::numeric_limits<std::uint32_t>::max()) {
                    std::fill(marks_.begin(), marks_.end(), 0);
                    generation_ = 1;
                } else {
                    ++generation_;
                }
            }
            const auto append = [&](const std::vector<std::size_t>& values) {
                if (!useDenseMarks) {
                    indices.insert(indices.end(), values.begin(), values.end());
                    return;
                }
                for (const std::size_t index : values) {
                    if (marks_[index] != generation_) {
                        marks_[index] = generation_;
                        indices.push_back(index);
                    }
                }
            };
            if (!activeTiles.empty()) {
                append(globalIndices_);
            }
            for (const std::uint64_t tile : activeTiles) {
                const auto found = byTile_.find(tile);
                if (found != byTile_.end()) {
                    append(found->second);
                }
            }
            std::sort(indices.begin(), indices.end());
            if (!useDenseMarks) {
                indices.erase(std::unique(indices.begin(), indices.end()), indices.end());
            }
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
    std::size_t maxIndex_ = 0;
    bool hasIndices_ = false;
    std::vector<std::uint32_t> marks_;
    std::uint32_t generation_ = 0;
    std::vector<std::size_t> globalIndices_;
    std::unordered_set<std::uint64_t> activeTiles_;
    std::vector<std::size_t> activeIndices_;
    bool dirty_ = true;
};

} // namespace openbus::rendering::map_detail
