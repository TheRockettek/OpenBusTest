#include "RoadFeatures.h"

#include <algorithm>

double roadFeatureHeightAt(const RoadBump& feature, double localX) {
    if (feature.type != RoadFeatureType::Incline || feature.length <= 0.0) {
        return 0.0;
    }
    const double phase = std::clamp((localX + feature.length * 0.5) / feature.length, 0.0, 1.0);
    return feature.height * phase * phase * phase;
}

const std::vector<RoadBump>& defaultRoadBumps() {
    static const std::vector<RoadBump> bumps = [] {
        std::vector<RoadBump> result = {
            {7.0, 0.0, 0.90, 7.0, 0.10},
            {12.0, 0.0, 1.10, 7.0, 0.18},
            {17.0, 0.72, 1.00, 1.35, 0.16},
            {20.0, -0.72, 1.00, 1.35, 0.16},
            {48.0, 0.0, 16.0, 4.20, 0.55, RoadFeatureType::Bridge, 6.50, 0.55, 0.14},
            {59.0, 2.00, 0.55, 0.35, 0.75, RoadFeatureType::Barrier},
            {62.0, -2.00, 0.55, 0.35, 0.75, RoadFeatureType::Barrier},
            {65.0, 0.65, 0.80, 0.45, 0.30, RoadFeatureType::Barrier},
            {67.0, -0.65, 0.80, 0.45, 0.30, RoadFeatureType::Barrier},
            {90.0, 0.0, 120.0, 250, 50.0, RoadFeatureType::Incline},
        };
        for (int index = 0; index < 10; ++index) {
            result.push_back({24.0 + index * 0.62, 0.0, 0.36, 6.8,
                              index % 2 == 0 ? 0.055 : 0.085});
        }
        result.insert(result.end(), {
                                        {31.0, 0.82, 1.30, 1.15, 0.18},
                                        {32.6, -0.82, 1.30, 1.15, 0.24},
                                        {34.2, 0.82, 1.30, 1.15, 0.12},
                                        {35.8, -0.82, 1.30, 1.15, 0.28},
                                    });
        return result;
    }();
    return bumps;
}