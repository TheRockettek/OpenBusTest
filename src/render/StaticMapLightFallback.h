#pragma once

#include "render/CoreRenderer.h"

#include <cstddef>
#include <optional>
#include <stdexcept>
#include <vector>

namespace openbus::rendering::detail {

constexpr std::size_t mapLightTextureCapacity(std::size_t texelCapacity,
                                              std::size_t texelsPerLight) {
    return texelCapacity / texelsPerLight;
}

struct CompactStaticMapLightSelection {
    std::vector<std::size_t> sourceIndices;
    std::vector<StaticMapLightView> viewLights;
};

inline std::optional<CompactStaticMapLightSelection>
compactStaticMapLightSelection(const std::vector<MapPointLight>& sources,
                               const std::vector<std::size_t>& sourceIndices,
                               const std::vector<StaticMapLightView>& viewLights, std::size_t count,
                               std::size_t sourceCapacity) {
    if (sourceIndices.size() <= sourceCapacity) {
        return std::nullopt;
    }
    if (count > viewLights.size()) {
        throw std::invalid_argument("Invalid static map-light view count");
    }
    CompactStaticMapLightSelection compact;
    compact.sourceIndices.reserve(count);
    compact.viewLights.reserve(count);
    for (std::size_t index = 0; index < count; ++index) {
        const auto& view = viewLights[index];
        if (view.sourceIndex >= sourceIndices.size() ||
            sourceIndices[view.sourceIndex] >= sources.size()) {
            throw std::invalid_argument("Invalid static map-light source index");
        }
        compact.sourceIndices.push_back(sourceIndices[view.sourceIndex]);
        compact.viewLights.push_back({view.position, index});
    }
    return compact;
}

} // namespace openbus::rendering::detail
