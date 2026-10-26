#pragma once

#include <filesystem>
#include <string>

namespace openbus {

inline std::string pathToUtf8(const std::filesystem::path& path) {
    const std::u8string utf8 = path.u8string();
    return std::string(reinterpret_cast<const char*>(utf8.data()), utf8.size());
}

inline std::filesystem::path pathFromUtf8(const std::string& utf8) {
    const std::u8string value(reinterpret_cast<const char8_t*>(utf8.data()), utf8.size());
    return std::filesystem::path(value);
}

} // namespace openbus
