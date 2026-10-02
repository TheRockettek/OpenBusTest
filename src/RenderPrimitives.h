#pragma once

#include "BusSimulation.h"
#include "RoadFeatures.h"

#include <array>
#include <vector>

namespace openbus::rendering {

void drawBox(double length, double width, double height, double red, double green, double blue);
void drawSolidTriangles(const std::vector<std::array<double, 3>>& vertices, double red,
                        double green, double blue);
void drawWireframeTriangles(const std::vector<std::array<double, 3>>& vertices, double red,
                            double green, double blue);
void drawRoadBox(double centerX, double centerY, double length, double width, double height,
                 double bottomZ, const std::array<double, 3>& topColor,
                 const std::array<double, 3>& sideColor);
void drawRoadIncline(const RoadBump& bump);
void drawGround(const std::vector<RoadBump>& bumps);
void drawWheel(double radius, double halfWidth, double red = 0.04, double green = 0.04,
               double blue = 0.04);
void drawCenterOfGravityMarker(double size);
void drawCollisionWireframe(const BusSimulation& simulation);

} // namespace openbus::rendering
