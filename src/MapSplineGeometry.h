#pragma once

#include <array>
#include <optional>
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

// Samples a spline at one arc-length position. Returns no value for invalid geometry or a
// distance outside the spline.
std::optional<MapSplineSample> sampleMapSpline(double startX, double startY, double headingDegrees,
                                               double length, double radius, double distance);

// OMSI's [spline] gradient parabola and [spline_h] cubic height profile.
double mapSplineElevation(double startElevation, double length, double gradientStart,
                          double gradientEnd, std::optional<double> heightDelta, double distance);

// Lateral offset in the right-positive OMSI map frame (x east, y north).
std::array<double, 2> mapSplineRightOffset(double headingRadians, double distance);

} // namespace openbus::map
