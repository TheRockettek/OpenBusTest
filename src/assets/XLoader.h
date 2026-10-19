#pragma once

#include "assets/ObjLoader.h"

#include <filesystem>
#include <memory>

namespace openbus::rendering {

// Loads static text, binary, and MSZIP-compressed DirectX .x geometry into the common parsed-mesh
// form. Skinning, animation, and named external materials are unsupported.
class XLoader {
  public:
    static std::shared_ptr<ParsedObj> parse(const std::filesystem::path& path);
};

} // namespace openbus::rendering