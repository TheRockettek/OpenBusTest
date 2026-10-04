#pragma once

namespace openbus::rendering {

enum class ViewpointContext : int {
    PlayerExterior = 1,
    PlayerInterior = 2,
    NonPlayer = 4,
};

constexpr int viewpointMask(ViewpointContext context) {
    return static_cast<int>(context);
}

constexpr bool viewpointMatches(int configuredMask, ViewpointContext context) {
    return configuredMask == 0 || (configuredMask > 0 && configuredMask <= 7 &&
                                   (configuredMask & viewpointMask(context)) != 0);
}

} // namespace openbus::rendering
