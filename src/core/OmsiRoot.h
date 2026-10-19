#pragma once

#include <filesystem>

// Root of the OMSI 2 installation: $OPENBUS_OMSI_ROOT when set, otherwise the
// default Steam install location.
[[nodiscard]] std::filesystem::path omsiRootPath();
