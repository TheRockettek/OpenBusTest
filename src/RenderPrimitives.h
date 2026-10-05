#pragma once

#include "RoadFeatures.h"

#include <array>
#include <vector>

namespace openbus::rendering {

void drawBox(double length, double width, double height, double red, double green, double blue);
void drawRoadBox(double centerX, double centerY, double length, double width, double height,
                 double bottomZ, const std::array<double, 3>& topColor,
                 const std::array<double, 3>& sideColor);
void drawRoadIncline(const RoadBump& bump);
void drawGround(const std::vector<RoadBump>& bumps);

} // namespace openbus::rendering
