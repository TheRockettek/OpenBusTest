#pragma once

#include "ObjLoader.h"

#include <filesystem>
#include <memory>

namespace openbus::rendering {

class O3DLoader {
  public:
    static std::shared_ptr<ParsedObj> parse(const std::filesystem::path& path);
};

} // namespace openbus::rendering