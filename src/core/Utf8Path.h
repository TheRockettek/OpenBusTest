#pragma once

#include <filesystem>
#include <string>

namespace openbus {

inline std::string pathToUtf8(const std::filesystem::path& path) {
    const std::u8string utf8 = path.u8string();
    return std::string(reinterpret_cast<const char*>(utf8.data()), utf8.size());
}

} // namespace openbus
