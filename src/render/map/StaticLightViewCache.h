#pragma once

#include "render/CoreRenderer.h"
#include "render/MatrixMath.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

namespace openbus::rendering::map_detail {

class StaticLightViewCache {
  public:
    bool prepare(const std::vector<MapPointLight>& sources,
                 const std::vector<std::size_t>& sourceIndices, const Matrix4& view,
                 const std::array<double, 3>& camera, std::size_t limit,
                 std::uint64_t revision = 0) {
        if (valid_ && view_ == view && camera_ == camera && sourceIndices_ == sourceIndices &&
            sourceCount_ == sources.size() && limit_ == limit && revision_ == revision) {
            return false;
        }

        const std::size_t count = std::min(sourceIndices.size(), limit);
        std::vector<StaticMapLightView> next;
        next.reserve(count);
        const auto append = [&](std::size_t index) {
            const auto& light = sources.at(sourceIndices[index]);
            const auto position = transformPoint(
                view, {light.position[0], light.position[1], light.position[2], 1.0});
            next.push_back({{position[0], position[1], position[2]}, index});
        };
        if (count == sourceIndices.size()) {
            for (std::size_t index = 0; index < count; ++index) {
                append(index);
            }
        } else if (count > 0) {
            std::vector<std::pair<double, std::size_t>> nearest;
            nearest.reserve(sourceIndices.size());
            for (std::size_t index = 0; index < sourceIndices.size(); ++index) {
                const auto& position = sources.at(sourceIndices[index]).position;
                const double dx = position[0] - camera[0];
                const double dy = position[1] - camera[1];
                const double dz = position[2] - camera[2];
                nearest.emplace_back(dx * dx + dy * dy + dz * dz, index);
            }
            std::partial_sort(nearest.begin(), nearest.begin() + count, nearest.end(),
                              [](const auto& a, const auto& b) { return a.first < b.first; });
            for (std::size_t index = 0; index < count; ++index) {
                append(nearest[index].second);
            }
        }
        sourceIndices_ = sourceIndices;
        view_ = view;
        camera_ = camera;
        sourceCount_ = sources.size();
        limit_ = limit;
        revision_ = revision;
        lights_ = std::move(next);
        valid_ = true;
        return true;
    }

    const std::vector<StaticMapLightView>& lights() const noexcept {
        return lights_;
    }

  private:
    Matrix4 view_ = {};
    std::array<double, 3> camera_ = {};
    std::vector<std::size_t> sourceIndices_;
    std::vector<StaticMapLightView> lights_;
    std::size_t sourceCount_ = 0;
    std::size_t limit_ = 0;
    std::uint64_t revision_ = 0;
    bool valid_ = false;
};

} // namespace openbus::rendering::map_detail
