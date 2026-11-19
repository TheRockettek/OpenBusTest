#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <span>

namespace openbus::rendering {

bool saveFramebufferPng(const std::filesystem::path& path, int width, int height);
bool saveRgbaPng(const std::filesystem::path& path, int width, int height,
                 std::span<const std::uint8_t> pixels, std::size_t rowStride,
                 bool flipVertically = false);

} // namespace openbus::rendering
