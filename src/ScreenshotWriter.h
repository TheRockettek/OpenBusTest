#pragma once

#include <filesystem>

namespace openbus::rendering {

bool saveFramebufferPng(const std::filesystem::path& path, int width, int height);

} // namespace openbus::rendering
