#pragma once

#include <algorithm>
#include <cctype>
#include <cstddef>
#include <string>
#include <unordered_map>

namespace openbus::app::vehicle {

inline std::string resolveCtcTextureName(
    const std::string& textureName,
    const std::unordered_map<std::string, std::string>& replacements) {
    std::string key = textureName;
    std::transform(key.begin(), key.end(), key.begin(), [](unsigned char character) {
        return static_cast<char>(std::tolower(character));
    });
    const auto found = replacements.find(key);
    return found == replacements.end() ? textureName : found->second;
}

inline int resolveTextureArrayLayer(bool textureArray, int requestedLayer,
                                    std::size_t layerCount) {
    if (!textureArray || requestedLayer <= 0 || layerCount == 0) {
        return 0;
    }
    return static_cast<int>(
        std::min(static_cast<std::size_t>(requestedLayer), layerCount - 1));
}

} // namespace openbus::app::vehicle
