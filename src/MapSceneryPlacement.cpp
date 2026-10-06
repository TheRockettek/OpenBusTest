#include "MapSceneryPlacement.h"

#include "MapSplineGeometry.h"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <unordered_set>

namespace openbus::map {

namespace {

using RotationMatrix = std::array<double, 9>;

RotationMatrix multiplyRotation(const RotationMatrix& left, const RotationMatrix& right) {
    RotationMatrix result = {};
    for (std::size_t column = 0; column < 3; ++column) {
        for (std::size_t row = 0; row < 3; ++row) {
            for (std::size_t inner = 0; inner < 3; ++inner) {
                result[column * 3 + row] += left[inner * 3 + row] * right[column * 3 + inner];
            }
        }
    }
    return result;
}

RotationMatrix rotationX(double degrees) {
    const double radians = degrees * std::numbers::pi / 180.0;
    const double cosine = std::cos(radians);
    const double sine = std::sin(radians);
    return {1.0, 0.0, 0.0, 0.0, cosine, sine, 0.0, -sine, cosine};
}

RotationMatrix rotationY(double degrees) {
    const double radians = degrees * std::numbers::pi / 180.0;
    const double cosine = std::cos(radians);
    const double sine = std::sin(radians);
    return {cosine, 0.0, -sine, 0.0, 1.0, 0.0, sine, 0.0, cosine};
}

RotationMatrix rotationZ(double degrees) {
    const double radians = degrees * std::numbers::pi / 180.0;
    const double cosine = std::cos(radians);
    const double sine = std::sin(radians);
    return {cosine, sine, 0.0, -sine, cosine, 0.0, 0.0, 0.0, 1.0};
}

RotationMatrix mapRotation(const std::array<double, 6>& degrees, std::size_t count) {
    RotationMatrix result = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    const std::size_t groups = count >= 6 ? 2 : 1;
    for (std::size_t group = 0; group < groups; ++group) {
        const std::size_t offset = group * 3;
        const RotationMatrix euler = multiplyRotation(
            multiplyRotation(rotationZ(-degrees[offset]), rotationY(degrees[offset + 2])),
            rotationX(degrees[offset + 1]));
        result = multiplyRotation(result, euler);
    }
    return result;
}

RotationMatrix poseRotation(const MapSceneryPose& pose) {
    return pose.hasOrientationMatrix ? pose.orientationMatrix
                                     : mapRotation(pose.rotationDegrees, pose.rotationCount);
}

std::array<double, 3> rotateVector(const RotationMatrix& matrix,
                                   const std::array<double, 3>& vector) {
    return {matrix[0] * vector[0] + matrix[3] * vector[1] + matrix[6] * vector[2],
            matrix[1] * vector[0] + matrix[4] * vector[1] + matrix[7] * vector[2],
            matrix[2] * vector[0] + matrix[5] * vector[1] + matrix[8] * vector[2]};
}

} // namespace

std::optional<MapSceneryPose> placeMapSceneryObject(const MapSceneryPlacement& object, int tileX,
                                                    int tileY) {
    if (!object.transformValid) {
        return std::nullopt;
    }
    MapSceneryPose pose;
    pose.x = static_cast<double>(tileX) * OMSI_TILE_SIZE_METERS + object.localPosition[0];
    pose.y = static_cast<double>(tileY) * OMSI_TILE_SIZE_METERS + object.localPosition[1];
    pose.z = object.localPosition[2];
    std::copy(object.rotationDegrees.begin(), object.rotationDegrees.end(),
              pose.rotationDegrees.begin());
    pose.rotationCount = object.rotationDegrees.size();
    if (!std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.z)) {
        return std::nullopt;
    }
    return pose;
}

double mapSceneryWorldHeight(double authoredZ, double terrainHeight, bool absoluteHeight) {
    return authoredZ + (absoluteHeight ? 0.0 : terrainHeight);
}

std::optional<std::array<double, 3>> sampleMapTerrainNormal(const TerrainGrid& terrain,
                                                            double localX, double localY) {
    if (terrain.intervals == 0 || !std::isfinite(localX) || !std::isfinite(localY)) {
        return std::nullopt;
    }
    const std::size_t last = terrain.intervals;
    const std::size_t side = last + 1;
    if (terrain.heights.size() < side * side) {
        return std::nullopt;
    }
    const double step = OMSI_TILE_SIZE_METERS / static_cast<double>(terrain.intervals);
    const double gridX = std::clamp(localX / step, 0.0, static_cast<double>(last));
    const double gridY = std::clamp(localY / step, 0.0, static_cast<double>(last));
    const std::size_t column = std::min(static_cast<std::size_t>(gridX), last - 1);
    const std::size_t row = std::min(static_cast<std::size_t>(gridY), last - 1);
    const double fractionX = gridX - static_cast<double>(column);
    const double fractionY = gridY - static_cast<double>(row);
    const auto heightAt = [&](std::size_t sampleRow, std::size_t sampleColumn) {
        return static_cast<double>(terrain.heights[sampleRow * side + sampleColumn]);
    };
    const double lowerX = heightAt(row, column + 1) - heightAt(row, column);
    const double upperX = heightAt(row + 1, column + 1) - heightAt(row + 1, column);
    const double lowerY = heightAt(row + 1, column) - heightAt(row, column);
    const double upperY = heightAt(row + 1, column + 1) - heightAt(row, column + 1);
    const double dzdx = std::lerp(lowerX, upperX, fractionY) / step;
    const double dzdy = std::lerp(lowerY, upperY, fractionX) / step;
    const double length = std::sqrt(dzdx * dzdx + dzdy * dzdy + 1.0);
    if (!std::isfinite(length) || length <= 1.0e-9) {
        return std::nullopt;
    }
    return std::array<double, 3>{-dzdx / length, -dzdy / length, 1.0 / length};
}

MapSceneryPose composeMapAttachedObjectPose(const MapSceneryPose& parent,
                                            const std::array<double, 3>& attachmentTranslation,
                                            const std::array<double, 3>& attachmentRotationXYZ,
                                            const std::array<double, 3>& childRotationDegrees) {
    const RotationMatrix parentRotation = poseRotation(parent);
    const std::array<double, 3> worldOffset = rotateVector(parentRotation, attachmentTranslation);

    const RotationMatrix attachmentRotation = multiplyRotation(
        multiplyRotation(rotationZ(-attachmentRotationXYZ[2]), rotationY(attachmentRotationXYZ[1])),
        rotationX(attachmentRotationXYZ[0]));
    const std::array<double, 6> childRotations = {
        childRotationDegrees[0], childRotationDegrees[1], childRotationDegrees[2], 0.0, 0.0, 0.0};

    MapSceneryPose result;
    result.x = parent.x + worldOffset[0];
    result.y = parent.y + worldOffset[1];
    result.z = parent.z + worldOffset[2];
    result.orientationMatrix = multiplyRotation(
        multiplyRotation(parentRotation, attachmentRotation), mapRotation(childRotations, 3));
    result.hasOrientationMatrix = true;
    return result;
}

std::array<double, 3> transformMapSceneryVector(const MapSceneryPose& pose,
                                                const std::array<double, 3>& localVector) {
    return rotateVector(poseRotation(pose), localVector);
}

bool alignMapSceneryPoseToTerrainNormal(MapSceneryPose& pose,
                                        const std::array<double, 3>& terrainNormal) {
    const double length =
        std::sqrt(terrainNormal[0] * terrainNormal[0] + terrainNormal[1] * terrainNormal[1] +
                  terrainNormal[2] * terrainNormal[2]);
    if (!std::isfinite(length) || length <= 1.0e-9) {
        return false;
    }
    const std::array<double, 3> normal = {terrainNormal[0] / length, terrainNormal[1] / length,
                                          terrainNormal[2] / length};
    // The shortest rotation from model-up (0,0,1) to the terrain normal. Terrain normals
    // are expected to point upward; the antiparallel fallback keeps malformed inputs finite.
    const double cosineHalfAngle = std::sqrt(std::max(0.0, (1.0 + normal[2]) * 0.5));
    double qx = 0.0;
    double qy = 0.0;
    double qz = 0.0;
    double qw = cosineHalfAngle;
    if (cosineHalfAngle <= 1.0e-9) {
        qx = 1.0;
    } else {
        const double scale = 0.5 / cosineHalfAngle;
        qx = -normal[1] * scale;
        qy = normal[0] * scale;
    }
    const RotationMatrix align = {1.0 - 2.0 * (qy * qy + qz * qz), 2.0 * (qx * qy + qz * qw),
                                  2.0 * (qx * qz - qy * qw),       2.0 * (qx * qy - qz * qw),
                                  1.0 - 2.0 * (qx * qx + qz * qz), 2.0 * (qy * qz + qx * qw),
                                  2.0 * (qx * qz + qy * qw),       2.0 * (qy * qz - qx * qw),
                                  1.0 - 2.0 * (qx * qx + qy * qy)};
    pose.orientationMatrix = multiplyRotation(align, poseRotation(pose));
    pose.hasOrientationMatrix = true;
    return true;
}

std::vector<MapSceneryPose> placeMapSplineAttachment(const MapSplineAttachment& attachment,
                                                     const std::vector<MapSplinePlacement>& splines,
                                                     int tileX, int tileY) {
    std::vector<MapSceneryPose> placements;
    if (!attachment.transformValid || attachment.splineIndex < 0 ||
        static_cast<std::size_t>(attachment.splineIndex) >= splines.size()) {
        return placements;
    }
    const std::size_t ownerIndex = static_cast<std::size_t>(attachment.splineIndex);
    const MapSplinePlacement& owner = splines[ownerIndex];
    if (!owner.geometryValid || owner.length <= 0.0) {
        return placements;
    }
    const double interval = std::max(0.0, attachment.interval);
    const double range = std::max(0.0, attachment.range);
    const double rowCountValue = interval > 0.0 ? std::floor(range / interval) + 1.0 : 1.0;
    const std::size_t rowCount = static_cast<std::size_t>(std::clamp(rowCountValue, 1.0, 4096.0));
    const std::size_t firstRowIndex = attachment.repeater ? attachment.repeaterFirstObjectIndex : 0;
    if (firstRowIndex >= rowCount) {
        return placements;
    }

    const double ownerChainOffset = owner.chainOffsetValid ? owner.chainOffset : 0.0;
    const double tileOriginX = static_cast<double>(tileX) * OMSI_TILE_SIZE_METERS;
    const double tileOriginY = static_cast<double>(tileY) * OMSI_TILE_SIZE_METERS;
    std::size_t currentIndex = ownerIndex;
    double accumulatedLength = 0.0;
    bool backwards = false;
    std::unordered_set<int> visited;
    visited.insert(owner.splineId);

    for (std::size_t segmentCount = 0; segmentCount < splines.size(); ++segmentCount) {
        const MapSplinePlacement& spline = splines[currentIndex];
        if (!spline.geometryValid || spline.length <= 0.0) {
            break;
        }
        for (std::size_t rowIndex = firstRowIndex; rowIndex < rowCount; ++rowIndex) {
            const double distanceFromChainStart =
                attachment.offset[2] + static_cast<double>(rowIndex) * interval;
            const double distanceOnChainDirection =
                distanceFromChainStart - ownerChainOffset - accumulatedLength;
            if (distanceOnChainDirection < -1.0e-6 ||
                distanceOnChainDirection > spline.length + 1.0e-6 ||
                (segmentCount != 0 && distanceOnChainDirection <= 1.0e-6)) {
                continue;
            }
            const double along = std::clamp(distanceOnChainDirection, 0.0, spline.length);
            const double curveDistance = backwards ? spline.length - along : along;
            const auto center = sampleMapSpline(tileOriginX + spline.localX,
                                                tileOriginY + spline.localY, spline.rotationDegrees,
                                                spline.length, spline.radius, curveDistance);
            if (!center) {
                continue;
            }

            const double lateral = backwards ? -attachment.offset[0] : attachment.offset[0];
            const double headingDegrees = center->headingRadians * 180.0 / std::numbers::pi;
            const auto lateralOffset = mapSplineRightOffset(center->headingRadians, lateral);
            const double t = std::clamp(curveDistance / spline.length, 0.0, 1.0);
            const double cant = spline.cantStart + (spline.cantEnd - spline.cantStart) * t;
            const double cantLift = -std::clamp(lateral, -10.0, 10.0) * cant / 100.0;
            const double elevation = mapSplineElevation(
                spline.elevation, spline.length, spline.gradientStart, spline.gradientEnd,
                spline.elevated ? std::optional<double>(spline.heightDelta) : std::nullopt,
                curveDistance);
            const double pitch = attachment.tilt
                                     ? std::atan((spline.gradientStart +
                                                  (spline.gradientEnd - spline.gradientStart) * t) /
                                                 100.0) *
                                           180.0 / std::numbers::pi
                                     : 0.0;
            const double bank = attachment.tilt && std::abs(lateral) <= 10.0
                                    ? std::atan(cant / 100.0) * 180.0 / std::numbers::pi
                                    : 0.0;
            const double reverseTurn = backwards ? 180.0 : 0.0;
            placements.push_back(
                {center->x + lateralOffset[0],
                 center->y + lateralOffset[1],
                 elevation + attachment.offset[1] + cantLift,
                 {headingDegrees, pitch, bank, attachment.rotationDegrees[0] + reverseTurn,
                  attachment.rotationDegrees[1], attachment.rotationDegrees[2]},
                 6});
        }

        accumulatedLength += spline.length;
        const int nextId = backwards ? spline.previousSplineId : spline.nextSplineId;
        if (nextId == 0 || visited.contains(nextId)) {
            break;
        }
        const auto next = std::find_if(
            splines.begin(), splines.end(),
            [nextId](const MapSplinePlacement& candidate) { return candidate.splineId == nextId; });
        if (next == splines.end()) {
            break; // A repeater in the next tile continues this chain.
        }
        if (next->previousSplineId == spline.splineId) {
            backwards = false;
        } else if (next->nextSplineId == spline.splineId) {
            backwards = true;
        } else {
            break;
        }
        currentIndex = static_cast<std::size_t>(std::distance(splines.begin(), next));
        visited.insert(nextId);
    }
    return placements;
}

} // namespace openbus::map
