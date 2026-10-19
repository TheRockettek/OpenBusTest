#include "core/OmsiRoot.h"

#include "core/Environment.h"

namespace {

constexpr const char* DEFAULT_OMSI_ROOT = R"(C:\Program Files (x86)\Steam\steamapps\common\OMSI 2)";

} // namespace

std::filesystem::path omsiRootPath() {
    if (const char* configuredRoot = openbus::getEnvironment("OPENBUS_OMSI_ROOT");
        configuredRoot != nullptr && *configuredRoot != '\0') {
        return configuredRoot;
    }
    return DEFAULT_OMSI_ROOT;
}
