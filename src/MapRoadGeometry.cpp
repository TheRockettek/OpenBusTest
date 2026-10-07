#include "MapRoadGeometry.h"

#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>

namespace openbus::map {

MapRoadSectionGeometry buildMapRoadSectionGeometry(const MapSplinePlacement& spline,
                                                   const std::vector<MapSplineSample>& centerline,
                                                   const MapSplineProfileSection& section) {
    MapRoadSectionGeometry geometry;
    if (!spline.geometryValid || centerline.size() < 2 || section.points.size() < 2) {
        return geometry;
    }
    if (centerline.size() > std::numeric_limits<std::size_t>::max() / section.points.size()) {
        throw std::length_error("Map road section vertex count overflows size_t");
    }
    const std::size_t vertexCount = centerline.size() * section.points.size();
    if (vertexCount > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
        throw std::length_error("Map road section exceeds ODE's vertex index limit");
    }
    if (centerline.size() - 1 >
        std::numeric_limits<std::size_t>::max() / (section.points.size() - 1) / 6) {
        throw std::length_error("Map road section triangle count overflows size_t");
    }

    geometry.vertices.reserve(vertexCount);
    geometry.indices.reserve((centerline.size() - 1) * (section.points.size() - 1) * 6);
    const double chainOffset = spline.chainOffsetValid ? spline.chainOffset : 0.0;
    for (const MapSplineSample& sample : centerline) {
        const double cantT = std::clamp(sample.distance / spline.length, 0.0, 1.0);
        const double cant = spline.cantStart + (spline.cantEnd - spline.cantStart) * cantT;
        const double elevation = mapSplineElevation(
            spline.elevation, spline.length, spline.gradientStart, spline.gradientEnd,
            spline.elevated ? std::optional<double>(spline.heightDelta) : std::nullopt,
            sample.distance);
        for (const MapSplineProfilePoint& point : section.points) {
            const double profileLateral = spline.mirrored ? -point.lateral : point.lateral;
            const std::array<double, 2> lateral =
                mapSplineRightOffset(sample.headingRadians, profileLateral);
            // Cant is expressed as a crossfall percentage, matching the map attachment-row
            // placement convention: positive cant lowers the right-positive side.
            const double cantLift = -profileLateral * cant / 100.0;
            MapRoadVertex vertex{sample.x + lateral[0], sample.y + lateral[1],
                                 elevation + point.height + cantLift, point.textureU,
                                 (chainOffset + sample.distance) * point.textureVPerMeter};
            if (!std::isfinite(vertex.x) || !std::isfinite(vertex.y) || !std::isfinite(vertex.z) ||
                !std::isfinite(vertex.textureU) || !std::isfinite(vertex.textureV)) {
                throw std::invalid_argument("Map road section contains non-finite geometry");
            }
            geometry.vertices.push_back(vertex);
        }
    }

    for (std::size_t sampleIndex = 1; sampleIndex < centerline.size(); ++sampleIndex) {
        const std::size_t startBase = (sampleIndex - 1) * section.points.size();
        const std::size_t endBase = sampleIndex * section.points.size();
        for (std::size_t pointIndex = 1; pointIndex < section.points.size(); ++pointIndex) {
            const std::size_t startLeft = startBase + pointIndex - 1;
            const std::size_t endLeft = endBase + pointIndex - 1;
            const std::size_t endRight = endBase + pointIndex;
            const std::size_t startRight = startBase + pointIndex;
            if (spline.mirrored) {
                // Reflecting lateral coordinates reverses the cross-section orientation.
                // Reverse each triangle as well to keep the renderer's winding convention.
                geometry.indices.insert(geometry.indices.end(),
                                        {static_cast<int>(startLeft), static_cast<int>(endRight),
                                         static_cast<int>(endLeft), static_cast<int>(startLeft),
                                         static_cast<int>(startRight), static_cast<int>(endRight)});
            } else {
                geometry.indices.insert(geometry.indices.end(),
                                        {static_cast<int>(startLeft), static_cast<int>(endLeft),
                                         static_cast<int>(endRight), static_cast<int>(startLeft),
                                         static_cast<int>(endRight), static_cast<int>(startRight)});
            }
        }
    }
    return geometry;
}

} // namespace openbus::map
