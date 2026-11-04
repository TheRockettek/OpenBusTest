#pragma once

#include "config/ModelConfigTypes.h"
#include "core/Variables.h"

#include <algorithm>
#include <cctype>
#include <cstddef>
#include <cmath>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace openbus::app::vehicle {

inline std::string normalizeCtcTextureKey(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char character) {
        return character == '\\' ? '/' : static_cast<char>(std::tolower(character));
    });
    return value;
}

inline std::unordered_map<std::string, std::string>
buildCtcTextureReplacements(
    const std::vector<ModelCtcTexture>& modelTextures,
    const std::unordered_map<std::string, std::string>& appearanceReplacements) {
    std::unordered_map<std::string, std::string> selectedBySlot;
    selectedBySlot.reserve(appearanceReplacements.size());
    for (const auto& [slot, replacement] : appearanceReplacements) {
        selectedBySlot[normalizeCtcTextureKey(slot)] = replacement;
    }
    std::unordered_map<std::string, std::string> replacements;
    replacements.reserve(modelTextures.size() * 2);
    for (const auto& texture : modelTextures) {
        const std::string slot = normalizeCtcTextureKey(texture.slot);
        const std::string textureName = normalizeCtcTextureKey(texture.textureName);
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

inline void applyCtiVariableReplacements(
    Variables& variables, const std::unordered_map<std::string, float>& replacements) {
    for (const auto& [name, value] : replacements) {
        if (name.empty() || !std::isfinite(value)) {
            throw std::invalid_argument("CTI variable replacements must be named and finite");
        }
        variables.set(name, value);
    }
}

inline std::string
resolveCtcTextureName(const std::string& textureName,
                      const std::unordered_map<std::string, std::string>& replacements) {
    const auto found = replacements.find(normalizeCtcTextureKey(textureName));
    return found == replacements.end() ? textureName : found->second;
}

inline int resolveTextureArrayLayer(bool textureArray, int requestedLayer, std::size_t layerCount) {
    if (!textureArray || requestedLayer <= 0 || layerCount == 0) {
        return 0;
    }
    return static_cast<int>(std::min(static_cast<std::size_t>(requestedLayer), layerCount - 1));
}

} // namespace openbus::app::vehicle
