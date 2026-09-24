#pragma once

#include <filesystem>

namespace openbus::rendering {

bool saveFramebufferBmp(const std::filesystem::path& path, int width, int height);

}  // namespace openbus::rendering
