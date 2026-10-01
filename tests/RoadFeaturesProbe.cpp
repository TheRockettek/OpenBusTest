#include "RoadFeatures.h"

#include <cmath>
#include <iostream>

int main() {
    const RoadBump* hump = nullptr;
    for (const RoadBump& feature : defaultRoadBumps()) {
        if (feature.type == RoadFeatureType::Hump) {
            hump = &feature;
            break;
        }
    }
    if (hump == nullptr) {
        std::cerr << "default road features do not contain a hump\n";
        return 1;
    }

    const double tolerance = 1.0e-9;
    if (std::abs(roadFeatureHeightAt(*hump, -hump->length * 0.5)) > tolerance ||
        std::abs(roadFeatureHeightAt(*hump, hump->length * 0.5)) > tolerance) {
        std::cerr << "hump endpoints should meet the road surface\n";
        return 1;
    }
    if (std::abs(roadFeatureHeightAt(*hump, 0.0) - hump->height) > tolerance) {
        std::cerr << "hump midpoint does not reach the configured height\n";
        return 1;
    }
    return 0;
}