#include "MapRoadGeometry.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <optional>
#include <stdexcept>

namespace openbus::map {
namespace {

constexpr double JOIN_DISTANCE_METERS = 0.25;
constexpr double MAX_MITER_SCALE = 4.0;

double frameCantHeightOffset(const MapRoadEndpointFrame& frame, double lateral) {
    double height = 0.0;
    for (const auto& contribution : frame.cantContributions) {
        height +=
            mapSplineCantHeightOffset(lateral, contribution.cantPercent, contribution.halfWidth);
    }
    return height;
}

double elevationAt(const MapSplinePlacement& spline, double distance) {
    const double clamped = std::clamp(distance, 0.0, spline.length);
    double elevation = mapSplineElevation(
        spline.elevation, spline.length, spline.gradientStart, spline.gradientEnd,
        spline.elevated ? std::optional<double>(spline.heightDelta) : std::nullopt, clamped);
    // A skewed cut can extend beyond the authored centerline. Continue the endpoint
    // tangent grade rather than flattening the overhang by clamping its height.
    if (distance != clamped) {
        elevation += (distance - clamped) *
                     (distance < 0.0 ? spline.gradientStart : spline.gradientEnd) / 100.0;
    }
    return elevation;
}

MapRoadEndpointFrame endpointFrame(const MapSplinePlacement& spline, const MapSplineSample& sample,
                                   bool end, std::optional<double> cantHalfWidth) {
    const double skew = end ? spline.skewEnd : spline.skewStart;
    const double grade = (end ? spline.gradientEnd : spline.gradientStart) / 100.0;
    const double cantPercent = end ? spline.cantEnd : spline.cantStart;
    const auto right = mapSplineRightOffset(sample.headingRadians, 1.0);
    MapRoadEndpointFrame frame;
    frame.center = {sample.x, sample.y, elevationAt(spline, sample.distance)};
    frame.lateralAxis = {right[0] + skew * std::sin(sample.headingRadians),
                         right[1] + skew * std::cos(sample.headingRadians), skew * grade};
    frame.cantContributions[0] = {cantPercent, cantHalfWidth};
    return frame;
}

bool finiteFrame(const MapRoadEndpointFrame& frame) {
    for (std::size_t axis = 0; axis < 3; ++axis) {
        if (!std::isfinite(frame.center[axis]) || !std::isfinite(frame.lateralAxis[axis])) {
            return false;
        }
    }
    for (const auto& contribution : frame.cantContributions) {
        if (!std::isfinite(contribution.cantPercent) ||
            (contribution.halfWidth.has_value() &&
             (!std::isfinite(*contribution.halfWidth) || *contribution.halfWidth < 0.0))) {
            return false;
        }
    }
    return true;
}

} // namespace

void MapRoadConnectionIndex::addSpline(const MapSplinePlacement& spline, int tileX, int tileY,
                                       std::optional<double> cantHalfWidth) {
    // Validate before changing the index so a rejected width cannot invalidate existing joins.
    static_cast<void>(mapSplineCantHeightOffset(0.0, 0.0, cantHalfWidth));
    joins_.clear();
    if (spline.splineId == 0 || ambiguousIds_.contains(spline.splineId)) {
        return;
    }
    if (!placements_.emplace(spline.splineId, Placement{spline, tileX, tileY, cantHalfWidth})
             .second) {
        placements_.erase(spline.splineId);
        ambiguousIds_.insert(spline.splineId);
    }
}

void MapRoadConnectionIndex::finalize() {
    joins_.clear();
    const auto sampleEndpoint = [](const Placement& placement, bool end) {
        const auto& spline = placement.spline;
        return spline.geometryValid
                   ? sampleMapSpline(placement.tileX * OMSI_TILE_SIZE_METERS + spline.localX,
                                     placement.tileY * OMSI_TILE_SIZE_METERS + spline.localY,
                                     spline.rotationDegrees, spline.length, spline.radius,
                                     end ? spline.length : 0.0)
                   : std::nullopt;
    };
    for (const auto& [id, placement] : placements_) {
        const auto& spline = placement.spline;
        for (const bool end : {false, true}) {
            const int otherId = end ? spline.nextSplineId : spline.previousSplineId;
            // Canonical ID order makes both frames independent of hash iteration/build order.
            if (otherId == 0 || id >= otherId) {
                continue;
            }
            const auto other = placements_.find(otherId);
            const auto sample = sampleEndpoint(placement, end);
            if (other == placements_.end() || !sample) {
                continue;
            }
            const auto ownFrame = endpointFrame(spline, *sample, end, placement.cantHalfWidth);
            if (!finiteFrame(ownFrame)) {
                continue;
            }
            const auto& neighbor = other->second.spline;
            std::optional<bool> neighborEnd;
            std::optional<MapSplineSample> neighborSample;
            MapRoadEndpointFrame neighborFrame;
            for (const bool candidateEnd : {false, true}) {
                if ((candidateEnd ? neighbor.nextSplineId : neighbor.previousSplineId) != id) {
                    continue;
                }
                const auto candidate = sampleEndpoint(other->second, candidateEnd);
                if (!candidate) {
                    continue;
                }
                const auto frame =
                    endpointFrame(neighbor, *candidate, candidateEnd, other->second.cantHalfWidth);
                if (!finiteFrame(frame)) {
                    continue;
                }
                const double gap = std::hypot(ownFrame.center[0] - frame.center[0],
                                              ownFrame.center[1] - frame.center[1],
                                              ownFrame.center[2] - frame.center[2]);
                if (gap > JOIN_DISTANCE_METERS) {
                    continue;
                }
                if (neighborEnd.has_value()) {
                    neighborSample.reset(); // Ambiguous loop: do not guess an endpoint.
                    break;
                }
                neighborEnd = candidateEnd;
                neighborSample = candidate;
                neighborFrame = frame;
            }
            if (!neighborSample) {
                continue;
            }
            // Require the endpoint pairing to be unambiguous in both directions. Very short
            // two-spline loops can otherwise map both of our endpoints to one neighbor end.
            const int oppositeLink = end ? spline.previousSplineId : spline.nextSplineId;
            if (oppositeLink == otherId) {
                const auto opposite = sampleEndpoint(placement, !end);
                if (opposite) {
                    const auto oppositeFrame =
                        endpointFrame(spline, *opposite, !end, placement.cantHalfWidth);
                    if (finiteFrame(oppositeFrame) &&
                        std::hypot(oppositeFrame.center[0] - neighborFrame.center[0],
                                   oppositeFrame.center[1] - neighborFrame.center[1],
                                   oppositeFrame.center[2] - neighborFrame.center[2]) <=
                            JOIN_DISTANCE_METERS) {
                        continue;
                    }
                }
            }
            const double sign = end == *neighborEnd ? -1.0 : 1.0;
            const auto right = mapSplineRightOffset(sample->headingRadians, 1.0);
            const auto otherRight = mapSplineRightOffset(neighborSample->headingRadians, sign);
            const double denominator = 1.0 + right[0] * otherRight[0] + right[1] * otherRight[1];
            if (denominator < 2.0 / (MAX_MITER_SCALE * MAX_MITER_SCALE)) {
                continue; // Near U-turns would create unbounded miter spikes.
            }
            MapRoadEndpointFrame seam;
            for (std::size_t axis = 0; axis < 3; ++axis) {
                seam.center[axis] =
                    std::midpoint(ownFrame.center[axis], neighborFrame.center[axis]);
            }
            // Automatic miter plus the mean authored shear in the common traversal frame.
            for (std::size_t axis = 0; axis < 2; ++axis) {
                seam.lateralAxis[axis] =
                    (right[axis] + otherRight[axis]) / denominator +
                    0.5 * (ownFrame.lateralAxis[axis] - right[axis] +
                           sign * neighborFrame.lateralAxis[axis] - otherRight[axis]);
            }
            const double along = seam.lateralAxis[0] * std::sin(sample->headingRadians) +
                                 seam.lateralAxis[1] * std::cos(sample->headingRadians);
            const double otherAlong =
                seam.lateralAxis[0] * std::sin(neighborSample->headingRadians) +
                seam.lateralAxis[1] * std::cos(neighborSample->headingRadians);
            seam.lateralAxis[2] =
                0.5 * (along * (end ? spline.gradientEnd : spline.gradientStart) / 100.0 +
                       otherAlong * (*neighborEnd ? neighbor.gradientEnd : neighbor.gradientStart) /
                           100.0);
            seam.cantContributions[0] = ownFrame.cantContributions[0];
            seam.cantContributions[0].cantPercent *= 0.5;
            seam.cantContributions[1] = neighborFrame.cantContributions[0];
            seam.cantContributions[1].cantPercent *= 0.5 * sign;
            if (!finiteFrame(seam)) {
                continue;
            }
            (end ? joins_[id].end : joins_[id].start) = seam;
            for (double& component : seam.lateralAxis) {
                component *= sign;
            }
            for (auto& contribution : seam.cantContributions) {
                contribution.cantPercent *= sign;
            }
            (*neighborEnd ? joins_[otherId].end : joins_[otherId].start) = seam;
        }
    }
}

MapRoadSplineJoins MapRoadConnectionIndex::joinsFor(const MapSplinePlacement& spline, int tileX,
                                                    int tileY) const {
    const auto placement = placements_.find(spline.splineId);
    if (placement == placements_.end() || placement->second.tileX != tileX ||
        placement->second.tileY != tileY) {
        return {};
    }
    const auto found = joins_.find(spline.splineId);
    return found == joins_.end() ? MapRoadSplineJoins{} : found->second;
}

MapRoadSectionGeometry buildMapRoadSectionGeometry(const MapSplinePlacement& spline,
                                                   const std::vector<MapSplineSample>& centerline,
                                                   const MapSplineProfileSection& section,
                                                   const MapRoadSplineJoins& joins,
                                                   std::optional<double> cantHalfWidth) {
    MapRoadSectionGeometry geometry;
    if (!spline.geometryValid || !std::isfinite(spline.length) || spline.length <= 0.0 ||
        centerline.size() < 2 || section.points.size() < 2) {
        return geometry;
    }
    if (cantHalfWidth.has_value() && (!std::isfinite(*cantHalfWidth) || *cantHalfWidth < 0.0)) {
        throw std::invalid_argument("Map road cant half-width must be finite and non-negative");
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
    // [scaleTexByLength] scales one texture span to this authored spline segment. Keep it
    // segment-local (do not apply chainOffset) so a long segment stretches the mapping instead
    // of producing one repeat for every metre as the ordinary V slope does.
    const double textureDistanceScale = section.scaleTextureByLength ? 1.0 / spline.length : 1.0;
    const auto startFrame = endpointFrame(spline, centerline.front(), false, cantHalfWidth);
    const auto endFrame = endpointFrame(spline, centerline.back(), true, cantHalfWidth);
    for (const MapSplineSample& sample : centerline) {
        // Use the sample's spline-distance parameter divided directly by the authored
        // segment length. Radius shapes the centerline only; it does not rescale the transition.
        const double segmentProgress = std::clamp(sample.distance / spline.length, 0.0, 1.0);
        const double cantPercent =
            spline.cantStart + (spline.cantEnd - spline.cantStart) * segmentProgress;
        const double skew = std::lerp(spline.skewStart, spline.skewEnd, segmentProgress);
        for (const MapSplineProfilePoint& point : section.points) {
            const double profileLateral = spline.mirrored ? -point.lateral : point.lateral;
            const std::array<double, 2> lateral =
                mapSplineRightOffset(sample.headingRadians, profileLateral);
            // Best-guess OMSI skew: metres forward per metre right, not degrees.
            const double skewShift = profileLateral * skew;
            const double elevation = elevationAt(spline, sample.distance + skewShift);
            // Runtime cant is a percentage: rise/run = percent / 100. Positive cant lowers
            // the right-positive side in the current map-axis convention.
            const double cantLift =
                mapSplineCantHeightOffset(profileLateral, cantPercent, cantHalfWidth);
            const double textureDistance =
                section.scaleTextureByLength ? sample.distance : chainOffset + sample.distance;
            MapRoadVertex vertex{
                sample.x + lateral[0] + skewShift * std::sin(sample.headingRadians),
                sample.y + lateral[1] + skewShift * std::cos(sample.headingRadians),
                elevation + point.height + cantLift, point.textureU,
                (textureDistance + skewShift) * point.textureVPerMeter * textureDistanceScale};
            const auto applyJoin = [&](const std::optional<MapRoadEndpointFrame>& join,
                                       const MapRoadEndpointFrame& original, double fromEnd,
                                       bool end) {
                if (!join) {
                    return;
                }
                std::array<double, 3> delta;
                for (std::size_t axis = 0; axis < 3; ++axis) {
                    delta[axis] =
                        join->center[axis] - original.center[axis] +
                        profileLateral * (join->lateralAxis[axis] - original.lateralAxis[axis]);
                }
                const double endpointDistance = end ? spline.length : 0.0;
                const double endpointSkew = end ? spline.skewEnd : spline.skewStart;
                const double endpointGrade = end ? spline.gradientEnd : spline.gradientStart;
                delta[2] -= elevationAt(spline, endpointDistance + profileLateral * endpointSkew) -
                            original.center[2] -
                            profileLateral * endpointSkew * endpointGrade / 100.0;
                delta[2] += frameCantHeightOffset(*join, profileLateral) -
                            frameCantHeightOffset(original, profileLateral);
                const double blendLength =
                    std::min(spline.length * 0.5,
                             std::max(2.0, 2.0 * std::hypot(delta[0], delta[1], delta[2])));
                const double t = std::clamp(fromEnd / blendLength, 0.0, 1.0);
                const double weight = 1.0 - t * t * (3.0 - 2.0 * t);
                vertex.x += delta[0] * weight;
                vertex.y += delta[1] * weight;
                vertex.z += delta[2] * weight;
                vertex.textureV += weight * point.textureVPerMeter * textureDistanceScale *
                                   (delta[0] * std::sin(sample.headingRadians) +
                                    delta[1] * std::cos(sample.headingRadians));
                if (fromEnd == 0.0) {
                    // Use the exact shared expression on both sides, including nonlinear grades.
                    vertex.x = join->center[0] + profileLateral * join->lateralAxis[0];
                    vertex.y = join->center[1] + profileLateral * join->lateralAxis[1];
                    vertex.z = join->center[2] + profileLateral * join->lateralAxis[2] +
                               frameCantHeightOffset(*join, profileLateral) + point.height;
                }
            };
            applyJoin(joins.start, startFrame, sample.distance, false);
            applyJoin(joins.end, endFrame, spline.length - sample.distance, true);
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

std::vector<std::array<double, 3>>
computeMapRoadVertexNormals(const MapRoadSectionGeometry& geometry) {
    // Road strips are wound clockwise from above for the renderer's GL_CW convention.
    // Accumulate the reversed triangle cross products so normals point toward map-up,
    // following cant, grade, curvature, skew, and the final seam corrections.
    std::vector<std::array<double, 3>> normals(geometry.vertices.size());
    for (std::size_t index = 0; index + 2 < geometry.indices.size(); index += 3) {
        const std::size_t firstIndex = static_cast<std::size_t>(geometry.indices[index]);
        const std::size_t secondIndex = static_cast<std::size_t>(geometry.indices[index + 1]);
        const std::size_t thirdIndex = static_cast<std::size_t>(geometry.indices[index + 2]);
        const MapRoadVertex& first = geometry.vertices[firstIndex];
        const MapRoadVertex& second = geometry.vertices[secondIndex];
        const MapRoadVertex& third = geometry.vertices[thirdIndex];
        const std::array<double, 3> firstEdge = {third.x - first.x, third.y - first.y,
                                                 third.z - first.z};
        const std::array<double, 3> secondEdge = {second.x - first.x, second.y - first.y,
                                                  second.z - first.z};
        const std::array<double, 3> faceNormal = {
            firstEdge[1] * secondEdge[2] - firstEdge[2] * secondEdge[1],
            firstEdge[2] * secondEdge[0] - firstEdge[0] * secondEdge[2],
            firstEdge[0] * secondEdge[1] - firstEdge[1] * secondEdge[0]};
        for (const std::size_t vertexIndex : {firstIndex, secondIndex, thirdIndex}) {
            for (std::size_t axis = 0; axis < faceNormal.size(); ++axis) {
                normals[vertexIndex][axis] += faceNormal[axis];
            }
        }
    }
    for (std::array<double, 3>& normal : normals) {
        const double normalLength = std::hypot(normal[0], normal[1], normal[2]);
        if (!std::isfinite(normalLength) ||
            normalLength <= std::numeric_limits<double>::epsilon()) {
            normal = {0.0, 0.0, 1.0};
            continue;
        }
        for (double& component : normal) {
            component /= normalLength;
        }
    }
    return normals;
}

} // namespace openbus::map
