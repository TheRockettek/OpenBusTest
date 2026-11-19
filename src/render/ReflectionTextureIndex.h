#pragma once

#include <algorithm>
#include <cctype>
#include <exception>
#include <filesystem>
#include <string>
#include <string_view>

namespace openbus::rendering {

inline int reflectionTextureIndex(const std::string& textureName) {
    constexpr std::string_view prefix = "reflexion";
    const auto candidate = std::search(
        textureName.begin(), textureName.end(), prefix.begin(), prefix.end(),
        [](unsigned char character, char expected) { return std::tolower(character) == expected; });
    if (candidate == textureName.end()) {
        return -1;
    }

    std::string stem = std::filesystem::path(textureName).stem().string();
    std::transform(stem.begin(), stem.end(), stem.begin(), [](unsigned char character) {
        return static_cast<char>(std::tolower(character));
    });
    if (stem.rfind(prefix, 0) != 0 || stem.size() == prefix.size()) {
        return -1;
    }
    try {
        const int index = std::stoi(stem.substr(prefix.size()));
        return index >= 0 ? index : -1;
    } catch (const std::exception&) {
        return -1;
    }
}

} // namespace openbus::rendering
