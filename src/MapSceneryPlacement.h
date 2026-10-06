#pragma once

#include "MapConfigLoader.h"

#include <array>
#include <cstddef>
#include <optional>
#include <vector>

namespace openbus::map {

struct MapSceneryPose {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    // Renderer rotation components; spline poses use heading/pitch/bank before object angles.
    std::array<double, 6> rotationDegrees = {};
    std::size_t rotationCount = 0;
    // Column-major 3x3 orientation for composed [attachObj] placements.
    std::array<double, 9> orientationMatrix = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    bool hasOrientationMatrix = false;
};

// Converts a tile-local ordinary [object] placement to map coordinates without changing its
// authored rotation component order. Invalid parsed transforms produce no pose.
std::optional<MapSceneryPose> placeMapSceneryObject(const MapSceneryPlacement& object, int tileX,
                                                    int tileY);

// Absolute-height assets use their authored Z directly; other assets are terrain-relative.
double mapSceneryWorldHeight(double authoredZ, double terrainHeight, bool absoluteHeight);

std::optional<std::array<double, 3>> sampleMapTerrainNormal(const TerrainGrid& terrain,
                                                            double localX, double localY);

// Compose a child transform at an SCO attachment point. Attachment rotations are authored as
// local X/Y/Z axes; child rotations use the map renderer's heading/pitch/bank convention.
MapSceneryPose composeMapAttachedObjectPose(const MapSceneryPose& parent,
                                            const std::array<double, 3>& attachmentTranslation,
                                            const std::array<double, 3>& attachmentRotationXYZ,
                                            const std::array<double, 3>& childRotationDegrees);

std::array<double, 3> transformMapSceneryVector(const MapSceneryPose& pose,
                                                const std::array<double, 3>& localVector);

// Tilt a scenery pose so its local up axis follows a sampled map-terrain normal, preserving its
// authored heading and position. Returns false for a degenerate/non-finite normal.
bool alignMapSceneryPoseToTerrainNormal(MapSceneryPose& pose,
                                        const std::array<double, 3>& terrainNormal);

// Places a spline-attachment row on its spline and subsequent linked splines in the same tile.
// A repeater resumes the row at the encoded first-object index on its own spline.
std::vector<MapSceneryPose> placeMapSplineAttachment(const MapSplineAttachment& attachment,
                                                     const std::vector<MapSplinePlacement>& splines,
                                                     int tileX, int tileY);

} // namespace openbus::map
