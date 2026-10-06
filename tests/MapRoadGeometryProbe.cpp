#include "MapRoadGeometry.h"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void requireNear(double actual, double expected, const char* message) {
    if (std::abs(actual - expected) > 1.0e-8) {
        throw std::runtime_error(std::string(message) + ": expected " +
                                 std::to_string(expected) + ", got " + std::to_string(actual));
    }
}

void runProbe() {
    openbus::map::MapSplinePlacement spline;
    spline.geometryValid = true;
    spline.localX = 100.0;
    spline.localY = 200.0;
    spline.elevation = 2.0;
    spline.length = 10.0;
    spline.gradientStart = 0.0;
    spline.gradientEnd = 10.0;

    openbus::map::MapSplineProfileSection section;
    section.textureIndex = 0;
    section.points = {{-2.0, 0.25, 0.0, 0.2}, {2.0, 0.25, 1.0, 0.2}};
    const auto straightCenterline =
        openbus::map::tessellateMapSpline(100.0, 200.0, 0.0, spline.length, 0.0);
    const auto straight =
        openbus::map::buildMapRoadSectionGeometry(spline, straightCenterline, section);
    require(straight.vertices.size() == straightCenterline.size() * 2,
            "straight cross-section keeps both profile points at every station");
    require(straight.indices.size() == (straightCenterline.size() - 1) * 6,
            "straight cross-section produces two triangles per segment");
    requireNear(straight.vertices[0].x, 98.0, "right-offset west profile edge");
    requireNear(straight.vertices[1].x, 102.0, "right-offset east profile edge");
    requireNear(straight.vertices.front().z, 2.25, "profile height is added to spline elevation");
    requireNear(straight.vertices[straight.vertices.size() - 1].z, 2.75,
                "spline gradient is reflected at the road endpoint");
    requireNear(straight.vertices.back().textureV, 2.0,
                "road texture V follows spline distance and profile scale");

    spline.chainOffsetValid = true;
    spline.chainOffset = spline.length;
    const auto continued =
        openbus::map::buildMapRoadSectionGeometry(spline, straightCenterline, section);
    requireNear(continued.vertices.front().textureV, straight.vertices.back().textureV,
                "chain offset continues texture phase at a linked spline start");
    requireNear(continued.vertices.back().textureV, 4.0,
                "texture phase advances from the authored chain offset");
    require(continued.indices == straight.indices &&
                continued.vertices.size() == straight.vertices.size(),
            "chain texture offset does not alter road topology");
    for (std::size_t vertexIndex = 0; vertexIndex < straight.vertices.size(); ++vertexIndex) {
        requireNear(continued.vertices[vertexIndex].x, straight.vertices[vertexIndex].x,
                    "chain texture offset leaves road x unchanged");
        requireNear(continued.vertices[vertexIndex].y, straight.vertices[vertexIndex].y,
                    "chain texture offset leaves road y unchanged");
        requireNear(continued.vertices[vertexIndex].z, straight.vertices[vertexIndex].z,
                    "chain texture offset leaves road height unchanged");
    }

    openbus::map::MapSplinePlacement bankedSpline = spline;
    bankedSpline.length = 10.0;
    bankedSpline.elevation = 0.0;
    bankedSpline.gradientStart = 0.0;
    bankedSpline.gradientEnd = 0.0;
    bankedSpline.cantStart = 2.0;
    bankedSpline.cantEnd = 4.0;
    openbus::map::MapSplineProfileSection bankedSection;
    bankedSection.points = {{-2.0, 0.0, 0.0, 0.0}, {2.0, 0.0, 1.0, 0.0}};
    const std::vector<openbus::map::MapSplineSample> bankSamples = {
        {0.0, 0.0, 0.0, 0.0}, {0.0, 5.0, 5.0, 0.0}, {0.0, 10.0, 10.0, 0.0}};
    const auto banked = openbus::map::buildMapRoadSectionGeometry(
        bankedSpline, bankSamples, bankedSection);
    requireNear(banked.vertices[0].z, 0.04,
                "positive cant raises the negative-lateral profile edge at spline start");
    requireNear(banked.vertices[1].z, -0.04,
                "positive cant lowers the positive-lateral profile edge at spline start");
    requireNear(banked.vertices[2].z, 0.06,
                "cant is interpolated linearly at the spline midpoint");
    requireNear(banked.vertices[3].z, -0.06,
                "interpolated cant is applied symmetrically across the section");
    requireNear(banked.vertices[4].z, 0.08,
                "end cant reaches the negative-lateral endpoint height");
    requireNear(banked.vertices[5].z, -0.08,
                "end cant reaches the positive-lateral endpoint height");

    spline.chainOffsetValid = false;
    spline.chainOffset = 0.0;
    spline.radius = 20.0;
    spline.length = 10.0;
    const auto curvedCenterline =
        openbus::map::tessellateMapSpline(100.0, 200.0, 0.0, spline.length, spline.radius);
    const auto curved =
        openbus::map::buildMapRoadSectionGeometry(spline, curvedCenterline, section);
    require(curved.vertices.size() == curvedCenterline.size() * 2,
            "curved cross-section follows each centerline sample");
    require(std::abs(curved.vertices.back().x - straight.vertices.back().x) > 0.1,
            "curved road vertices follow the curved centerline");

    spline.elevated = true;
    spline.heightDelta = 4.0;
    spline.gradientStart = 0.0;
    spline.gradientEnd = 0.0;
    const auto elevated =
        openbus::map::buildMapRoadSectionGeometry(spline, curvedCenterline, section);
    requireNear(elevated.vertices.back().z, 6.25,
                "elevated spline height profile is reflected in road geometry");

    spline.geometryValid = false;
    require(openbus::map::buildMapRoadSectionGeometry(spline, curvedCenterline, section)
                .vertices.empty(),
            "invalid spline placement does not create road geometry");
}

} // namespace

int main() {
    try {
        runProbe();
    } catch (const std::exception& error) {
        std::cerr << "map road geometry probe failed: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
