#pragma once

#include <vector>

namespace openbus::map {

struct MapSplineSample {
    double x = 0.0;
    double y = 0.0;
    double distance = 0.0;
    double headingRadians = 0.0;
};

// Tessellates a spline centerline using OMSI's heading/radius/arc-length fields.
// An empty result indicates invalid or excessively curved geometry.
std::vector<MapSplineSample> tessellateMapSpline(double startX, double startY,
                                                 double headingDegrees, double length,
                                                 double radius);

} // namespace openbus::map
