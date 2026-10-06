#include "MapSplineGeometry.h"

#include "PerfTrace.h"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace openbus::map {
namespace {

constexpr double STRAIGHT_RADIUS_EPSILON = 1.0e-8;
constexpr double MAX_SEGMENT_LENGTH_METERS = 2.0;
constexpr double MAX_HEADING_STEP_RADIANS = 5.0 * std::numbers::pi / 180.0;
constexpr std::size_t MAX_SPLINE_SEGMENTS = 4096;

std::optional<MapSplineSample> sampleUnchecked(double startX, double startY, double headingDegrees,
                                               double radius, double distance) {
    const bool straight = std::abs(radius) <= STRAIGHT_RADIUS_EPSILON;
    const double startHeading = headingDegrees * std::numbers::pi / 180.0;
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
        return std::nullopt;
    }
    return MapSplineSample{x, y, distance, heading};
}

} // namespace

std::optional<MapSplineSample> sampleMapSpline(double startX, double startY, double headingDegrees,
                                               double length, double radius, double distance) {
    if (!std::isfinite(startX) || !std::isfinite(startY) || !std::isfinite(headingDegrees) ||
        !std::isfinite(length) || !std::isfinite(radius) || !std::isfinite(distance) ||
        length <= 0.0 || distance < 0.0 || distance > length) {
        return std::nullopt;
    }
    return sampleUnchecked(startX, startY, headingDegrees, radius, distance);
}

double mapSplineElevation(double startElevation, double length, double gradientStart,
                          double gradientEnd, std::optional<double> heightDelta, double distance) {
    if (!std::isfinite(startElevation) || !std::isfinite(length) || length <= 0.0 ||
        !std::isfinite(gradientStart) || !std::isfinite(gradientEnd) || !std::isfinite(distance) ||
        (heightDelta && !std::isfinite(*heightDelta))) {
        return startElevation;
    }
    const double t = std::clamp(distance / length, 0.0, 1.0);
    if (heightDelta) {
        const double m0 = gradientStart / 100.0 * length;
        const double m1 = gradientEnd / 100.0 * length;
        const double t2 = t * t;
        const double t3 = t2 * t;
        return startElevation + (t3 - 2.0 * t2 + t) * m0 + (3.0 * t2 - 2.0 * t3) * *heightDelta +
               (t3 - t2) * m1;
    }
    const double averageGradient = gradientStart + (gradientEnd - gradientStart) * t * 0.5;
    return startElevation + distance * averageGradient / 100.0;
}

std::array<double, 2> mapSplineRightOffset(double headingRadians, double distance) {
    return {distance * std::cos(headingRadians), -distance * std::sin(headingRadians)};
}

std::vector<MapSplineSample> tessellateMapSpline(double startX, double startY,
                                                 double headingDegrees, double length,
                                                 double radius) {
    openbus::rendering::TraceScope trace("map", "MapSplineGeometry::tessellate");
    if (!std::isfinite(startX) || !std::isfinite(startY) || !std::isfinite(headingDegrees) ||
        !std::isfinite(length) || !std::isfinite(radius) || length <= 0.0) {
        return {};
    }

    const bool straight = std::abs(radius) <= STRAIGHT_RADIUS_EPSILON;
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
        const double distance =
            length * static_cast<double>(index) / static_cast<double>(segmentCount);
        const auto sample = sampleUnchecked(startX, startY, headingDegrees, radius, distance);
        if (!sample) {
            return {};
        }
        samples.push_back(*sample);
    }
    return samples;
}

} // namespace openbus::map
