#include "MapSplineGeometry.h"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace openbus::map {
namespace {

constexpr double STRAIGHT_RADIUS_EPSILON = 1.0e-8;
constexpr double MAX_SEGMENT_LENGTH_METERS = 2.0;
constexpr double MAX_HEADING_STEP_RADIANS = 5.0 * std::numbers::pi / 180.0;
constexpr std::size_t MAX_SPLINE_SEGMENTS = 4096;

} // namespace

std::vector<MapSplineSample> tessellateMapSpline(double startX, double startY,
                                                 double headingDegrees, double length,
                                                 double radius) {
    if (!std::isfinite(startX) || !std::isfinite(startY) ||
        !std::isfinite(headingDegrees) || !std::isfinite(length) ||
        !std::isfinite(radius) || length <= 0.0) {
        return {};
    }

    const bool straight = std::abs(radius) <= STRAIGHT_RADIUS_EPSILON;
    const double startHeading = headingDegrees * std::numbers::pi / 180.0;
    const double headingChange = straight ? 0.0 : length / radius;
    if (!std::isfinite(headingChange)) {
        return {};
    }
    const double lengthSegments = std::ceil(length / MAX_SEGMENT_LENGTH_METERS);
    const double headingSegments = std::ceil(std::abs(headingChange) / MAX_HEADING_STEP_RADIANS);
    const double segmentCountValue = std::max({1.0, lengthSegments, headingSegments});
    if (segmentCountValue > static_cast<double>(MAX_SPLINE_SEGMENTS)) {
        return {};
    }
    const std::size_t segmentCount = static_cast<std::size_t>(segmentCountValue);

    std::vector<MapSplineSample> samples;
    samples.reserve(segmentCount + 1);
    for (std::size_t index = 0; index <= segmentCount; ++index) {
        const double distance = length * static_cast<double>(index) /
                                static_cast<double>(segmentCount);
        const double heading = straight ? startHeading : startHeading + distance / radius;
        double x = 0.0;
        double y = 0.0;
        if (straight) {
            x = startX + std::sin(startHeading) * distance;
            y = startY + std::cos(startHeading) * distance;
        } else {
            x = startX + radius * (std::cos(startHeading) - std::cos(heading));
            y = startY + radius * (std::sin(heading) - std::sin(startHeading));
        }
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(heading)) {
            return {};
        }
        samples.push_back({x, y, distance, heading});
    }
    return samples;
}

} // namespace openbus::map
