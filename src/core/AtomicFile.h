#pragma once

#include <filesystem>
#include <functional>
#include <iosfwd>

namespace openbus::core {

void writeFileAtomically(const std::filesystem::path& path,
                         const std::function<void(std::ostream&)>& writeContent);

} // namespace openbus::core
