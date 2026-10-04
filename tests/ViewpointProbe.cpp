#include "Viewpoint.h"

#include <iostream>

int main() {
    using openbus::rendering::ViewpointContext;
    using openbus::rendering::viewpointMatches;

    const bool valid =
        viewpointMatches(0, ViewpointContext::PlayerExterior) &&
        viewpointMatches(0, ViewpointContext::PlayerInterior) &&
        viewpointMatches(0, ViewpointContext::NonPlayer) &&
        viewpointMatches(1, ViewpointContext::PlayerExterior) &&
        !viewpointMatches(1, ViewpointContext::PlayerInterior) &&
        !viewpointMatches(1, ViewpointContext::NonPlayer) &&
        !viewpointMatches(2, ViewpointContext::PlayerExterior) &&
        viewpointMatches(2, ViewpointContext::PlayerInterior) &&
        !viewpointMatches(2, ViewpointContext::NonPlayer) &&
        !viewpointMatches(4, ViewpointContext::PlayerExterior) &&
        !viewpointMatches(4, ViewpointContext::PlayerInterior) &&
        viewpointMatches(4, ViewpointContext::NonPlayer) &&
        viewpointMatches(5, ViewpointContext::PlayerExterior) &&
        !viewpointMatches(5, ViewpointContext::PlayerInterior) &&
        viewpointMatches(5, ViewpointContext::NonPlayer) &&
        !viewpointMatches(-1, ViewpointContext::PlayerExterior) &&
        !viewpointMatches(8, ViewpointContext::NonPlayer);

    if (!valid) {
        std::cerr << "viewpoint bit-mask filtering does not match the documented categories\n";
        return 1;
    }
    return 0;
}
