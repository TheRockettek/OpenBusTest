#include "render/RendererReflection.h"

#include <cstdlib>
#include <iostream>
#include <string>

namespace {

bool setTransparency(const std::string& value) {
#ifdef _WIN32
    return _putenv_s("OPENBUS_REFLECTION_TRANSPARENT", value.c_str()) == 0;
#else
    return value.empty() ? unsetenv("OPENBUS_REFLECTION_TRANSPARENT") == 0
                         : setenv("OPENBUS_REFLECTION_TRANSPARENT", value.c_str(), 1) == 0;
#endif
}

} // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Expected default, enabled or disabled\n";
        return 1;
    }
    const std::string mode = argv[1];
    if (mode != "default" && mode != "enabled" && mode != "disabled") {
        std::cerr << "Unknown reflection settings probe mode: " << mode << '\n';
        return 1;
    }
    const bool expected = mode != "disabled";
    if (!setTransparency(mode == "default" ? "" : expected ? "1" : "0")) {
        std::cerr << "Could not configure mirror transparency environment\n";
        return 1;
    }
    if (openbus::rendering::reflectionTransparentEnabled() != expected) {
        std::cerr << "Mirror transparency did not match the initial environment\n";
        return 1;
    }
    if (!setTransparency(expected ? "0" : "1")) {
        std::cerr << "Could not change mirror transparency environment\n";
        return 1;
    }
    if (openbus::rendering::reflectionTransparentEnabled() != expected) {
        std::cerr << "Mirror transparency was not cached after the first lookup\n";
        return 1;
    }
    return 0;
}
