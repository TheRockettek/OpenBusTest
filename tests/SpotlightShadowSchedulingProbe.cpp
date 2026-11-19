#include "app/vehicle/SpotlightShadowScheduling.h"

#include <cmath>
#include <iostream>
#include <limits>

int main() {
    using namespace openbus::app::vehicle;
    bool ok = true;
    float visibility = 0.0F;
    for (int i = 0; i < 8; ++i) {
        visibility = advanceSpotlightShadowVisibility(visibility, true);
        ok = ok && std::abs(visibility - static_cast<float>(i + 1) / 8.0F) < 1e-6F;
    }
    ok = ok && advanceSpotlightShadowVisibility(visibility, true) == 1.0F;
    for (int i = 0; i < 8; ++i)
        visibility = advanceSpotlightShadowVisibility(visibility, false);
    ok = ok && visibility == 0.0F && advanceSpotlightShadowVisibility(0.0F, false) == 0.0F;
    ok = ok && spotlightShadowFresh(8, 2) && !spotlightShadowFresh(9, 2);
    ok = ok && spotlightShadowFresh(60, 20) && !spotlightShadowFresh(61, 20);
    ok = ok && reserveSpotlightShadowUpdateForAi(1, 8);
    ok = ok && !reserveSpotlightShadowUpdateForAi(1, 7);
    ok = ok && !reserveSpotlightShadowUpdateForAi(2, 8);
    ok = ok && spotlightShadowUpdateBefore(true, 7, false, 1, 1, 7);
    ok = ok && spotlightShadowUpdateBefore(false, 1, true, 7, 1, 8);
    ok = ok && spotlightShadowUpdateBefore(true, 7, false, 1, 2, 8);
    ok = ok && spotlightShadowUpdateBefore(false, 1, false, 2, 1, 8);
    ok = ok && !spotlightShadowUpdateBefore(false, 2, false, 2, 1, 8);
    ok = ok && adaptSpotlightShadowUpdateBudget(8, 30.0) == 7;
    ok = ok && adaptSpotlightShadowUpdateBudget(1, 30.0) == 1;
    ok = ok && adaptSpotlightShadowUpdateBudget(7, 60.0) == 8;
    ok = ok && adaptSpotlightShadowUpdateBudget(8, 60.0) == 8;
    ok = ok && adaptSpotlightShadowUpdateBudget(4, 45.0) == 4;
    ok = ok && adaptSpotlightShadowUpdateBudget(4, 30.0, 30.0) == 5;
    ok = ok && adaptSpotlightShadowUpdateBudget(4, 19.0, 30.0) == 3;
    ok = ok && adaptSpotlightShadowUpdateBudget(4, 30.0, 120.0) == 3;
    ok = ok && adaptSpotlightShadowUpdateBudget(4, std::numeric_limits<double>::quiet_NaN()) == 4;
    if (!ok)
        std::cerr << "Spotlight shadow scheduling regression\n";
    return ok ? 0 : 1;
}
