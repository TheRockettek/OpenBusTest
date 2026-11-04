#pragma once

#include "config/ModelConfigTypes.h"

#include <algorithm>
#include <cctype>
#include <cstddef>
#include <string>
#include <unordered_map>
#include <vector>

namespace openbus::app::vehicle {

inline std::unordered_map<std::string, std::string>
buildCtcTextureReplacements(
    const std::vector<ModelCtcTexture>& modelTextures,
    const std::unordered_map<std::string, std::string>& appearanceReplacements) {
    const auto lowercase = [](std::string value) {
        std::transform(value.begin(), value.end(), value.begin(),
                       [](unsigned char character) {
                           return static_cast<char>(std::tolower(character));
                       });
        return value;
    };
    std::unordered_map<std::string, std::string> selectedBySlot;
    selectedBySlot.reserve(appearanceReplacements.size());
    for (const auto& [slot, replacement] : appearanceReplacements) {
        selectedBySlot[lowercase(slot)] = replacement;
    }
    std::unordered_map<std::string, std::string> replacements;
    replacements.reserve(modelTextures.size() * 2);
    for (const auto& texture : modelTextures) {
        const std::string slot = lowercase(texture.slot);
        const std::string textureName = lowercase(texture.textureName);
        if (slot.empty() || textureName.empty()) {
            continue;
        }
        const auto selected = selectedBySlot.find(slot);
        const std::string& replacement = selected == selectedBySlot.end()
                                             ? texture.textureName
                                             : selected->second;
        replacements[slot] = replacement;
        replacements[textureName] = replacement;
    }
    return replacements;
}

inline std::string
resolveCtcTextureName(const std::string& textureName,
                      const std::unordered_map<std::string, std::string>& replacements) {
    std::string key = textureName;
    std::transform(key.begin(), key.end(), key.begin(), [](unsigned char character) {
        return static_cast<char>(std::tolower(character));
    });
    const auto found = replacements.find(key);
    return found == replacements.end() ? textureName : found->second;
}

inline int resolveTextureArrayLayer(bool textureArray, int requestedLayer, std::size_t layerCount) {
    if (!textureArray || requestedLayer <= 0 || layerCount == 0) {
        return 0;
    }
    return static_cast<int>(std::min(static_cast<std::size_t>(requestedLayer), layerCount - 1));
}

} // namespace openbus::app::vehicle
