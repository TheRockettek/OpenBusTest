#pragma once

#include "BusTypes.h"

double roadFeatureHeightAt(const RoadBump& feature, double localX);
const std::vector<RoadBump>& defaultRoadBumps();