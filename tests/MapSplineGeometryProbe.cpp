#include "MapSplineGeometry.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

void require(bool condition, const char* description) {
    if (!condition) {
        throw std::runtime_error(description);
    }
}

void requireNear(double actual, double expected, double tolerance, const char* description) {
    if (std::abs(actual - expected) > tolerance) {
        throw std::runtime_error(std::string(description) + ": expected " +
                                 std::to_string(expected) + ", got " +
                                 std::to_string(actual));
    }
}

void verifySampling(const std::vector<openbus::map::MapSplineSample>& samples,
                    double expectedLength) {
    require(samples.size() >= 2, "spline produced at least two centerline samples");
    for (std::size_t index = 1; index < samples.size(); ++index) {
        const double distance = samples[index].distance - samples[index - 1].distance;
        const double headingChange = std::abs(samples[index].headingRadians -
                                              samples[index - 1].headingRadians);
        require(distance <= 2.0 + 1.0e-9, "centerline samples are at most 2 m apart");
        require(headingChange <= 5.0 * std::numbers::pi / 180.0 + 1.0e-9,
                "curved centerline samples turn at most five degrees");
    }
    requireNear(samples.back().distance, expectedLength, 1.0e-9,
                "last sample reaches the spline arc length");
}

void runProbe() {
    constexpr double tileOffsetX = -2100.0;
    constexpr double tileOffsetY = 3600.0;
    constexpr double tolerance = 2.0e-5;

    // Installed tile_-7_12.map IDs 30252 -> 30253: a +15 m, quarter-circle arc.
    const auto positiveCurve = openbus::map::tessellateMapSpline(
        tileOffsetX + 80.0672476244738, tileOffsetY + 33.0468149692465,
        466.951215243067, 23.5619397597659, 14.9999997646012);
    require(!positiveCurve.empty(), "positive-radius curve is tessellated");
    requireNear(positiveCurve.back().x, tileOffsetX + 90.0421820384018, tolerance,
                "positive-radius endpoint matches linked spline 30253 X");
    requireNear(positiveCurve.back().y, tileOffsetY + 14.3251633361388, tolerance,
                "positive-radius endpoint matches linked spline 30253 Y");
    requireNear(positiveCurve.back().headingRadians,
                556.951215243067 * std::numbers::pi / 180.0, tolerance,
                "positive-radius endpoint heading matches linked spline 30253");
    verifySampling(positiveCurve, 23.5619397597659);

    // Installed tile_-7_12.map IDs 30254 -> 30255: the mirrored -15 m arc.
    const auto negativeCurve = openbus::map::tessellateMapSpline(
        tileOffsetX + 114.981402704871, tileOffsetY + 22.4049666235302,
        286.951197452009, 23.5619397597659, -14.9999997646012);
    require(!negativeCurve.empty(), "negative-radius curve is tessellated");
    requireNear(negativeCurve.back().x, tileOffsetX + 96.2597347508879, tolerance,
                "negative-radius endpoint matches linked spline 30255 X");
    requireNear(negativeCurve.back().y, tileOffsetY + 12.4300313506087, tolerance,
                "negative-radius endpoint matches linked spline 30255 Y");
    requireNear(negativeCurve.back().headingRadians,
                196.951222916219 * std::numbers::pi / 180.0, tolerance,
                "negative-radius endpoint heading matches linked spline 30255");
    verifySampling(negativeCurve, 23.5619397597659);

    const auto straight =
        openbus::map::tessellateMapSpline(10.0, 20.0, 90.0, 50.0, 0.0);
    require(!straight.empty(), "zero-radius spline remains straight");
    requireNear(straight.back().x, 60.0, 1.0e-9, "straight endpoint X");
    requireNear(straight.back().y, 20.0, 1.0e-9, "straight endpoint Y");
    requireNear(straight.back().headingRadians, std::numbers::pi / 2.0, 1.0e-9,
                "straight endpoint heading");
    verifySampling(straight, 50.0);

    require(openbus::map::tessellateMapSpline(0.0, 0.0, 0.0, 0.0, 0.0).empty(),
            "zero-length spline is rejected");
    require(openbus::map::tessellateMapSpline(0.0, 0.0, 0.0, 10.0,
                                              std::numeric_limits<double>::infinity())
                .empty(),
            "non-finite radius is rejected");
}

} // namespace

int main() {
    try {
        runProbe();
    } catch (const std::exception& error) {
        std::cerr << "map spline geometry probe failed: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
