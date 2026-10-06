#pragma once

#include "ObjLoader.h"

#include <filesystem>
#include <memory>

namespace openbus::rendering {

// Loads static text-mode DirectX .x geometry into the common parsed-mesh form.
// Binary/compressed files, skinning, and named external materials are unsupported.
class XLoader {
  public:
    static std::shared_ptr<ParsedObj> parse(const std::filesystem::path& path);
};

} // namespace openbus::rendering