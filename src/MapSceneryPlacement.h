#pragma once

#include "MapConfigLoader.h"

#include <array>
#include <cstddef>
#include <functional>
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

// Bilinearly samples a decoded map-terrain grid at tile-local coordinates.
std::optional<double> sampleMapTerrainHeight(const TerrainGrid& terrain, double localX,
                                             double localY);

// Map [attachObj] anchor numbers are zero-based offsets into the parent's ordered
// [new_attachment] list. A parent with no configured points can still use anchor zero as origin.
std::optional<std::size_t> mapSceneryAttachmentPointOffset(int attachmentPointNumber,
                                                           std::size_t attachmentPointCount);

std::optional<std::array<double, 3>> sampleMapTerrainNormal(const TerrainGrid& terrain,
                                                            double localX, double localY);

// Compose a child transform at an SCO attachment point. Attachment translations follow the
// decoded scenery-mesh local X/Y/Z basis; child rotations use the map renderer's
// heading/pitch/bank convention.
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

// Select a scenery model LOD using the threshold-to-distance mapping already used by vehicle
// models: boundary = 25 / max(threshold, 0.001), with the last level as the far fallback.
std::size_t selectMapSceneryLod(const std::vector<double>& thresholds, double distance) noexcept;

// Places a spline-attachment row on its spline and subsequent linked splines in the same tile.
// A repeater resumes the row at the encoded first-object index on its own spline.
// Resolve metadata for the actual host at each segment, not just the row's first spline.
using MapSplineCantWidthResolver = std::function<std::optional<double>(const MapSplinePlacement&)>;

struct MapSplinePlacementReference {
    MapSplinePlacement placement;
    int tileX = 0;
    int tileY = 0;
};

// Resolves unique spline IDs across the loaded map. Returning no value stops a chain at a
// missing/ambiguous ID; a returned record must have a reciprocal link to its predecessor.
using MapSplineChainResolver =
    std::function<std::optional<MapSplinePlacementReference>(int splineId)>;

std::vector<MapSceneryPose>
placeMapSplineAttachment(const MapSplineAttachment& attachment,
                         const std::vector<MapSplinePlacement>& splines, int tileX, int tileY,
                         const MapSplineCantWidthResolver& cantWidth = {});

// Map-wide overload used by the renderer so reciprocal spline links can continue through tile
// boundaries. The attachment's splineIndex still selects its owner within the source tile.
std::vector<MapSceneryPose>
placeMapSplineAttachment(const MapSplineAttachment& attachment,
                         const std::vector<MapSplinePlacement>& sourceTileSplines, int tileX,
                         int tileY, const MapSplineChainResolver& chainResolver,
                         const MapSplineCantWidthResolver& cantWidth = {});

// Returns the first generated pose for a spline-attachment record when it is used as an
// [attachObj] parent; its ID identifies the authored row, not every repeated child instance.
std::optional<MapSceneryPose>
placeMapSplineAttachmentParent(const MapSplineAttachment& attachment,
                               const std::vector<MapSplinePlacement>& splines, int tileX, int tileY,
                               const MapSplineCantWidthResolver& cantWidth = {});

std::optional<MapSceneryPose>
placeMapSplineAttachmentParent(const MapSplineAttachment& attachment,
                               const std::vector<MapSplinePlacement>& sourceTileSplines, int tileX,
                               int tileY, const MapSplineChainResolver& chainResolver,
                               const MapSplineCantWidthResolver& cantWidth = {});

} // namespace openbus::map
