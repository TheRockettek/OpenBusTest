#pragma once

#include <cstddef>
#include <charconv>
#include <string_view>

namespace openbus::scripting {

inline constexpr int maxScriptTextureDimension = 4096;
inline constexpr int maxScriptTextureCount = 256;
inline constexpr std::size_t maxScriptTexturePixels = 4U * 1024U * 1024U;
inline constexpr std::size_t maxScriptTextureBytesPerSurface = maxScriptTexturePixels * 4U;
inline constexpr std::size_t maxScriptTextureBytesPerRuntime = 256U * 1024U * 1024U;
inline constexpr std::size_t maxScriptTextLength = 65U * 1024U;

constexpr bool validScriptTextureSize(int width, int height) {
    return width > 0 && height > 0 && width <= maxScriptTextureDimension &&
           height <= maxScriptTextureDimension &&
           static_cast<std::size_t>(width) <=
               maxScriptTexturePixels / static_cast<std::size_t>(height);
}

constexpr std::size_t scriptTextureByteSize(int width, int height) {
    return validScriptTextureSize(width, height)
               ? static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 4U
               : 0U;
}

constexpr bool validScriptTextureIndex(int index) {
    return index >= 0 && index < maxScriptTextureCount;
}

inline int scriptTextureReferenceIndex(std::string_view name) {
    if (name.size() <= 3 || name[0] != '\\' || (name[1] != 'S' && name[1] != 's') ||
        name[2] != ':') {
        return -1;
    }
    int index = -1;
    const auto parsed = std::from_chars(name.data() + 3, name.data() + name.size(), index);
    return parsed.ec == std::errc{} && parsed.ptr == name.data() + name.size() &&
                   validScriptTextureIndex(index)
               ? index
               : -1;
}

} // namespace openbus::scripting
