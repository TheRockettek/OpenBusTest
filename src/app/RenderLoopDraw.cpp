// Frame rendering: reflections, scene draw, HUD overlay and capture.

#include "app/RenderLoopInternal.h"
#include "map/MapSplineProfile.h"
#include "render/WeatherEffects.h"

#include <cmath>
#include <map>
#include <numbers>

namespace {

class SceneryTrafficEndpointIndex {
  public:
    SceneryTrafficEndpointIndex(const openbus::rendering::MapRenderer& mapRenderer,
                                const std::vector<std::pair<int, int>>& activeTiles) {
        for (const openbus::rendering::TrafficPathDebugSegment& segment :
             mapRenderer.sceneryTrafficPathSegments()) {
            if (segment.points.empty() ||
                std::find(activeTiles.begin(), activeTiles.end(),
                          std::pair{segment.tileX, segment.tileY}) == activeTiles.end()) {
                continue;
            }
            addEndpoint(segment.points.front(), segment.pathTypeCode);
            addEndpoint(segment.points.back(), segment.pathTypeCode);
        }
        for (const int pathTypeCode :
             {openbus::map::MAP_SPLINE_PATH_TYPE_ROAD, openbus::map::MAP_SPLINE_PATH_TYPE_HUMAN}) {
            for (const int splineId : mapRenderer.trafficSplineIds(pathTypeCode)) {
                const std::optional<openbus::map::MapTrafficRoute> route =
                    mapRenderer.buildTrafficRoute(splineId, pathTypeCode);
                if (!route.has_value() || route->segments.empty()) {
                    continue;
                }
                const openbus::map::MapTrafficRouteSegment& firstSegment = route->segments.front();
                const openbus::map::MapSplinePlacementReference& splineReference =
                    firstSegment.spline;
                if (std::find(activeTiles.begin(), activeTiles.end(),
                              std::pair{splineReference.tileX, splineReference.tileY}) ==
                    activeTiles.end()) {
                    continue;
                }
                const double splineLength = splineReference.placement.length;
                if (!std::isfinite(splineLength) || splineLength <= 0.0) {
                    continue;
                }
                openbus::map::MapTrafficRoute singleSplineRoute;
                singleSplineRoute.segments.push_back(firstSegment);
                singleSplineRoute.cumulativeEndDistances.push_back(splineLength);
                singleSplineRoute.totalLength = splineLength;
                std::vector<openbus::map::MapSplinePathOffset> pathOffsets =
                    mapRenderer.trafficSplinePathOffsets(splineId, pathTypeCode);
                if (pathOffsets.empty()) {
                    pathOffsets.push_back({});
                }
                for (const openbus::map::MapSplinePathOffset& pathOffset : pathOffsets) {
                    for (const double distance : {0.0, splineLength}) {
                        const std::optional<openbus::map::MapTrafficPose> pose =
                            openbus::map::sampleMapTrafficRoute(singleSplineRoute, distance,
                                                                pathOffset.lateralMeters);
                        if (!pose.has_value()) {
                            continue;
                        }
                        std::array<double, 3> position = pose->position;
                        position[2] += pathOffset.heightMeters;
                        addEndpoint(position, static_cast<double>(pathTypeCode), splineId);
                    }
                }
            }
        }
    }

    std::optional<std::array<double, 3>>
    find(int pathTypeCode, const std::array<double, 3>& position, int excludeSplineId) const {
        constexpr double horizontalToleranceMeters = 0.5;
        constexpr double verticalToleranceMeters = 0.5;
        const double maximumDistanceSquared =
            horizontalToleranceMeters * horizontalToleranceMeters +
            verticalToleranceMeters * verticalToleranceMeters;
        double closestDistanceSquared = maximumDistanceSquared;
        std::optional<std::array<double, 3>> closestEndpoint;
        const Cell center = cellFor(position);
        for (long long xOffset = -1; xOffset <= 1; ++xOffset) {
            for (long long yOffset = -1; yOffset <= 1; ++yOffset) {
                const auto bucket =
                    endpoints_.find({center.first + xOffset, center.second + yOffset});
                if (bucket == endpoints_.end()) {
                    continue;
                }
                for (const Endpoint& candidate : bucket->second) {
                    if (candidate.pathTypeCode != static_cast<double>(pathTypeCode) ||
                        (candidate.splineId.has_value() &&
                         *candidate.splineId == excludeSplineId)) {
                        continue;
                    }
                    const double horizontalDistance = std::hypot(
                        position[0] - candidate.position[0], position[1] - candidate.position[1]);
                    const double verticalDistance = std::abs(position[2] - candidate.position[2]);
                    if (horizontalDistance > horizontalToleranceMeters ||
                        verticalDistance > verticalToleranceMeters) {
                        continue;
                    }
                    const double distanceSquared = horizontalDistance * horizontalDistance +
                                                   verticalDistance * verticalDistance;
                    if (distanceSquared < closestDistanceSquared) {
                        closestDistanceSquared = distanceSquared;
                        closestEndpoint = candidate.position;
                    }
                }
            }
        }
        return closestEndpoint;
    }

  private:
    using Cell = std::pair<long long, long long>;
    struct Endpoint {
        std::array<double, 3> position = {};
        double pathTypeCode = 0.0;
        std::optional<int> splineId;
    };

    static Cell cellFor(const std::array<double, 3>& position) {
        constexpr double cellSizeMeters = 0.5;
        return {static_cast<long long>(std::floor(position[0] / cellSizeMeters)),
                static_cast<long long>(std::floor(position[1] / cellSizeMeters))};
    }

    void addEndpoint(const std::array<double, 3>& position, double pathTypeCode,
                     std::optional<int> splineId = std::nullopt) {
        if (!std::isfinite(position[0]) || !std::isfinite(position[1]) ||
            !std::isfinite(position[2])) {
            return;
        }
        endpoints_[cellFor(position)].push_back({position, pathTypeCode, splineId});
    }

    std::map<Cell, std::vector<Endpoint>> endpoints_;
};

[[maybe_unused]] void appendHumanBox(std::vector<openbus::rendering::PrimitiveVertex>& vertices,
                                     const std::array<double, 3>& position, double headingRadians,
                                     double heightMeters, double bottomFraction,
                                     double partHeightFraction, double widthMeters,
                                     double depthMeters, const std::array<double, 3>& color) {
    const double forwardX = std::cos(headingRadians);
    const double forwardY = std::sin(headingRadians);
    const double rightX = -forwardY;
    const double rightY = forwardX;
    const double bottom = heightMeters * bottomFraction;
    const double partHeight = heightMeters * partHeightFraction;
    const double halfWidth = widthMeters * 0.5;
    const double halfDepth = depthMeters * 0.5;
    const std::array<std::array<double, 3>, 8> corners = {{
        {-halfDepth, -halfWidth, bottom},
        {halfDepth, -halfWidth, bottom},
        {halfDepth, halfWidth, bottom},
        {-halfDepth, halfWidth, bottom},
        {-halfDepth, -halfWidth, bottom + partHeight},
        {halfDepth, -halfWidth, bottom + partHeight},
        {halfDepth, halfWidth, bottom + partHeight},
        {-halfDepth, halfWidth, bottom + partHeight},
    }};
    constexpr std::array<std::array<int, 4>, 6> faces = {{
        {4, 5, 6, 7},
        {0, 1, 5, 4},
        {1, 2, 6, 5},
        {2, 3, 7, 6},
        {3, 0, 4, 7},
        {0, 3, 2, 1},
    }};
    for (const auto& face : faces) {
        for (int index : {face[0], face[1], face[2], face[0], face[2], face[3]}) {
            const auto& local = corners[static_cast<std::size_t>(index)];
            vertices.push_back(
                {static_cast<float>(position[0] + forwardX * local[0] + rightX * local[1]),
                 static_cast<float>(position[1] + forwardY * local[0] + rightY * local[1]),
                 static_cast<float>(position[2] + local[2]), static_cast<float>(color[0]),
                 static_cast<float>(color[1]), static_cast<float>(color[2])});
        }
    }
}

void appendTrafficBehaviorBox(std::vector<openbus::rendering::PrimitiveVertex>& vertices,
                              const openbus::map::MapTrafficPose& pose, double lengthMeters,
                              double widthMeters, double heightMeters,
                              const std::array<float, 3>& color) {
    if (!std::isfinite(lengthMeters) || lengthMeters <= 0.0 || !std::isfinite(widthMeters) ||
        widthMeters <= 0.0 || !std::isfinite(heightMeters) || heightMeters <= 0.0) {
        return;
    }
    const double forwardX = std::sin(pose.headingRadians);
    const double forwardY = std::cos(pose.headingRadians);
    const double rightX = std::cos(pose.headingRadians);
    const double rightY = -std::sin(pose.headingRadians);
    const double halfLength = lengthMeters * 0.5;
    const double halfWidth = widthMeters * 0.5;
    const double bottom = pose.position[2] + 0.04;
    std::array<std::array<double, 3>, 8> corners = {};
    constexpr std::array<std::array<int, 2>, 4> signs = {
        std::array<int, 2>{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};
    for (std::size_t corner = 0; corner < signs.size(); ++corner) {
        const double longitudinal = signs[corner][0] * halfLength;
        const double lateral = signs[corner][1] * halfWidth;
        const double x = pose.position[0] + forwardX * longitudinal + rightX * lateral;
        const double y = pose.position[1] + forwardY * longitudinal + rightY * lateral;
        corners[corner] = {x, y, bottom};
        corners[corner + 4] = {x, y, bottom + heightMeters};
    }
    constexpr std::array<std::array<std::size_t, 2>, 12> edges = {std::array<std::size_t, 2>{0, 1},
                                                                  {1, 2},
                                                                  {2, 3},
                                                                  {3, 0},
                                                                  {4, 5},
                                                                  {5, 6},
                                                                  {6, 7},
                                                                  {7, 4},
                                                                  {0, 4},
                                                                  {1, 5},
                                                                  {2, 6},
                                                                  {3, 7}};
    for (const auto& edge : edges) {
        for (const std::size_t corner : edge) {
            const auto& point = corners[corner];
            vertices.push_back({static_cast<float>(point[0]), static_cast<float>(point[1]),
                                static_cast<float>(point[2]), color[0], color[1], color[2]});
        }
    }
}

void appendTrafficSplineDebugVertices(const openbus::rendering::MapRenderer& mapRenderer,
                                      int pathTypeCode,
                                      const std::vector<std::pair<int, int>>& activeTiles,
                                      const std::array<float, 3>& color,
                                      const SceneryTrafficEndpointIndex& sceneryEndpoints,
                                      std::vector<openbus::rendering::PrimitiveVertex>& vertices) {
    for (const int splineId : mapRenderer.trafficSplineIds(pathTypeCode)) {
        const std::optional<openbus::map::MapTrafficRoute> route =
            mapRenderer.buildTrafficRoute(splineId, pathTypeCode);
        if (!route.has_value() || route->segments.empty()) {
            continue;
        }

        const openbus::map::MapTrafficRouteSegment& firstSegment = route->segments.front();
        const openbus::map::MapSplinePlacementReference& splineReference = firstSegment.spline;
        if (std::find(activeTiles.begin(), activeTiles.end(),
                      std::pair{splineReference.tileX, splineReference.tileY}) ==
            activeTiles.end()) {
            continue;
        }

        const double splineLength = splineReference.placement.length;
        if (!std::isfinite(splineLength) || splineLength <= 0.0) {
            continue;
        }
        const std::optional<openbus::map::MapTrafficSplineEndpointStatus> endpointStatus =
            mapRenderer.trafficSplineEndpointStatus(splineId, pathTypeCode);
        openbus::map::MapTrafficRoute singleSplineRoute;
        singleSplineRoute.segments.push_back(firstSegment);
        singleSplineRoute.cumulativeEndDistances.push_back(splineLength);
        singleSplineRoute.totalLength = splineLength;

        std::vector<openbus::map::MapSplinePathOffset> pathOffsets =
            mapRenderer.trafficSplinePathOffsets(splineId, pathTypeCode);
        if (pathOffsets.empty()) {
            pathOffsets.push_back({});
        }
        constexpr double sampleSpacingMeters = 2.0;
        constexpr std::size_t maximumSamplesPerSpline = 2048;
        const double requestedSamples = std::ceil(splineLength / sampleSpacingMeters);
        const std::size_t sampleCount =
            requestedSamples >= static_cast<double>(maximumSamplesPerSpline)
                ? maximumSamplesPerSpline
                : std::max<std::size_t>(1, static_cast<std::size_t>(requestedSamples));
        for (const openbus::map::MapSplinePathOffset& pathOffset : pathOffsets) {
            std::optional<openbus::map::MapTrafficPose> previousPose =
                openbus::map::sampleMapTrafficRoute(singleSplineRoute, 0.0,
                                                    pathOffset.lateralMeters);
            const std::optional<openbus::map::MapTrafficPose> startPose = previousPose;
            for (std::size_t sampleIndex = 1; sampleIndex <= sampleCount; ++sampleIndex) {
                const double distance = splineLength * static_cast<double>(sampleIndex) /
                                        static_cast<double>(sampleCount);
                const std::optional<openbus::map::MapTrafficPose> pose =
                    openbus::map::sampleMapTrafficRoute(singleSplineRoute, distance,
                                                        pathOffset.lateralMeters);
                if (previousPose.has_value() && pose.has_value()) {
                    vertices.push_back({static_cast<float>(previousPose->position[0]),
                                        static_cast<float>(previousPose->position[1]),
                                        static_cast<float>(previousPose->position[2] +
                                                           pathOffset.heightMeters + 0.08),
                                        color[0], color[1], color[2]});
                    vertices.push_back(
                        {static_cast<float>(pose->position[0]),
                         static_cast<float>(pose->position[1]),
                         static_cast<float>(pose->position[2] + pathOffset.heightMeters + 0.08),
                         color[0], color[1], color[2]});
                }
                previousPose = pose;
            }
            constexpr float endpointMarkerHalfSize = 0.45F;
            constexpr float endpointMarkerZOffset = 0.16F;
            constexpr std::array<float, 3> endpointMarkerColor = {1.0F, 0.48F, 0.02F};
            const auto appendEndpointJoinOrMarker =
                [&](const std::optional<openbus::map::MapTrafficPose>& pose,
                    double pathHeightOffset) {
                    if (!pose.has_value()) {
                        return;
                    }
                    std::array<double, 3> endpointPosition = pose->position;
                    endpointPosition[2] += pathHeightOffset;
                    if (const std::optional<std::array<double, 3>> joinedEndpoint =
                            sceneryEndpoints.find(pathTypeCode, endpointPosition, splineId)) {
                        const double horizontalGap =
                            std::hypot(endpointPosition[0] - (*joinedEndpoint)[0],
                                       endpointPosition[1] - (*joinedEndpoint)[1]);
                        const double verticalGap =
                            std::abs(endpointPosition[2] - (*joinedEndpoint)[2]);
                        if (horizontalGap > 0.01 || verticalGap > 0.01) {
                            vertices.push_back({static_cast<float>(endpointPosition[0]),
                                                static_cast<float>(endpointPosition[1]),
                                                static_cast<float>(endpointPosition[2] + 0.08),
                                                color[0], color[1], color[2]});
                            vertices.push_back({static_cast<float>((*joinedEndpoint)[0]),
                                                static_cast<float>((*joinedEndpoint)[1]),
                                                static_cast<float>((*joinedEndpoint)[2] + 0.08),
                                                color[0], color[1], color[2]});
                        }
                        return;
                    }
                    const float x = static_cast<float>(endpointPosition[0]);
                    const float y = static_cast<float>(endpointPosition[1]);
                    const float z = static_cast<float>(endpointPosition[2] + endpointMarkerZOffset);
                    vertices.push_back({x - endpointMarkerHalfSize, y - endpointMarkerHalfSize, z,
                                        endpointMarkerColor[0], endpointMarkerColor[1],
                                        endpointMarkerColor[2]});
                    vertices.push_back({x + endpointMarkerHalfSize, y + endpointMarkerHalfSize, z,
                                        endpointMarkerColor[0], endpointMarkerColor[1],
                                        endpointMarkerColor[2]});
                    vertices.push_back({x - endpointMarkerHalfSize, y + endpointMarkerHalfSize, z,
                                        endpointMarkerColor[0], endpointMarkerColor[1],
                                        endpointMarkerColor[2]});
                    vertices.push_back({x + endpointMarkerHalfSize, y - endpointMarkerHalfSize, z,
                                        endpointMarkerColor[0], endpointMarkerColor[1],
                                        endpointMarkerColor[2]});
                };
            if (endpointStatus.has_value()) {
                if (!endpointStatus->startConnected) {
                    appendEndpointJoinOrMarker(startPose, pathOffset.heightMeters);
                }
                if (!endpointStatus->endConnected) {
                    appendEndpointJoinOrMarker(previousPose, pathOffset.heightMeters);
                }
            }
        }
    }
}

void appendSceneryTrafficPathDebugVertices(
    const openbus::rendering::MapRenderer& mapRenderer,
    const std::vector<std::pair<int, int>>& activeTiles,
    std::vector<openbus::rendering::PrimitiveVertex>& vertices) {
    for (const openbus::rendering::TrafficPathDebugSegment& segment :
         mapRenderer.sceneryTrafficPathSegments()) {
        if (std::find(activeTiles.begin(), activeTiles.end(),
                      std::pair{segment.tileX, segment.tileY}) == activeTiles.end()) {
            continue;
        }
        const std::array<float, 3> color = segment.pathTypeCode == 1.0
                                               ? std::array<float, 3>{1.0F, 0.25F, 0.9F}
                                               : std::array<float, 3>{0.15F, 0.9F, 1.0F};
        for (std::size_t index = 1; index < segment.points.size(); ++index) {
            for (const std::array<double, 3>& point :
                 {segment.points[index - 1], segment.points[index]}) {
                vertices.push_back({static_cast<float>(point[0]), static_cast<float>(point[1]),
                                    static_cast<float>(point[2] + 0.08), color[0], color[1],
                                    color[2]});
            }
        }
    }
}

} // namespace

void RenderLoop::renderReflectionViews(const BusSimulation& simulation) {
    if (renderingReflection_ || reflectionRenderer_->empty()) {
        return;
    }
    const int previousCameraView = cameraView_;
    const double previousFovOffset = fieldOfViewOffset_;
    const double previousLookYaw = viewLookYaw_;
    const double previousLookPitch = viewLookPitch_;
    bool reflectionTrianglesReset = false;
    renderingReflection_ = true;
    openbus::rendering::setFrameTriangleCountReflectionPass(true);
    reflectionRenderer_->render(
        simulation, vehicleCameras_, viewport_,
        [this](std::size_t reflectionIndex) {
            openbus::rendering::ReflectionRequirement result;
            for (const std::unique_ptr<Vehicle>& vehicle : vehicles_) {
                if (!vehicle->mapTrafficRouteFinished &&
                    (!vehicle->followsMapTrafficRoute || vehicle->mapTrafficTileActive) &&
                    vehicle->needsReflectionTexture(reflectionIndex)) {
                    result.needed = true;
                    result.size =
                        std::max(result.size, vehicle->requiredReflectionSize(reflectionIndex));
                }
            }
            return result;
        },
        [this, &reflectionTrianglesReset](const BusSimulation& reflectionSimulation,
                                          std::size_t cameraIndex, int width, int height) {
            if (!reflectionTrianglesReset) {
                openbus::rendering::resetFrameReflectionTriangleCount();
                reflectionTrianglesReset = true;
            }
            framebufferWidth_ = std::max(width, 1);
            framebufferHeight_ = std::max(height, 1);
            cameraView_ = static_cast<int>(cameraIndex + 1);
            fieldOfViewOffset_ = 0.0;
            viewLookYaw_ = 0.0;
            viewLookPitch_ = 0.0;
            setPerspective(static_cast<double>(width), static_cast<double>(height),
                           currentFieldOfView());
            draw(reflectionSimulation);
        },
        [this, previousCameraView, previousFovOffset, previousLookYaw,
         previousLookPitch](int width, int height) {
            framebufferWidth_ = std::max(width, 1);
            framebufferHeight_ = std::max(height, 1);
            cameraView_ = previousCameraView;
            fieldOfViewOffset_ = previousFovOffset;
            viewLookYaw_ = previousLookYaw;
            viewLookPitch_ = previousLookPitch;
            setPerspective(static_cast<double>(width), static_cast<double>(height),
                           currentFieldOfView());
        });
    openbus::rendering::setFrameTriangleCountReflectionPass(false);
    renderingReflection_ = false;
}

void RenderLoop::draw(const BusSimulation& simulation) {
    TraceScope trace("frame", "RenderLoop::draw");
    if (!renderingReflection_) {
        openbus::rendering::resetFrameTriangleCount();
    }
    const BodyPose chassis = simulation.chassisPose();
    const ChassisCollisionBox collision = simulation.chassisCollisionBox();
    constexpr double radiansToDegrees = 180.0 / 3.14159265358979323846;
    const Matrix4 playerModelBase = multiplyMatrix4(
        poseMatrix(chassis),
        translationMatrix(
            {0.0, 0.0, playerVehicle_ != nullptr ? playerVehicle_->modelOffsetZ : 0.0}));
    double playerGroundHeight = [&]() {
        TraceScope phase("render", "RenderLoop::draw.groundHeight");
        return mapRenderer_ != nullptr
                   ? mapRenderer_->terrainHeightAt(chassis.position[0], chassis.position[1])
                         .value_or(playerVehicle_ != nullptr ? playerVehicle_->placement.position[2]
                                                             : 0.0)
                   : (playerVehicle_ != nullptr ? playerVehicle_->placement.position[2] : 0.0);
    }();
    double wheelGroundHeightSum = 0.0;
    std::size_t validWheelGroundHeights = 0;
    for (std::size_t wheelIndex = 0; wheelIndex < simulation.wheelCount(); ++wheelIndex) {
        const double wheelBottom =
            simulation.wheelPose(wheelIndex).position[2] - simulation.wheelRadius(wheelIndex);
        if (std::isfinite(wheelBottom)) {
            wheelGroundHeightSum += wheelBottom;
            ++validWheelGroundHeights;
        }
    }
    if (validWheelGroundHeights > 0) {
        // Wheel bottoms follow the actual contact surface, including elevated
        // roads and ramps that can differ from the underlying terrain grid.
        playerGroundHeight = wheelGroundHeightSum / static_cast<double>(validWheelGroundHeights);
    }
    const Matrix4 shadowGroundBase =
        multiplyMatrix4(translationMatrix({chassis.position[0], chassis.position[1],
                                           playerGroundHeight + GROUND_SHADOW_OFFSET_Z}),
                        rotationMatrix(simulation.yaw() * radiansToDegrees, 0.0, 0.0, 1.0));
    Matrix4 inversePlayerModelBase = identityMatrix();
    const bool canGroundClampShadow = invertAffineMatrix(playerModelBase, inversePlayerModelBase);
    const Matrix4 groundShadowTransform =
        canGroundClampShadow ? multiplyMatrix4(inversePlayerModelBase, shadowGroundBase)
                             : identityMatrix();
    {
        TraceScope phase("render", "RenderLoop::draw.camera");
        if (freeCameraEnabled_ && !renderingReflection_ && !captureMode_) {
            const std::array<double, 3> forward = openbus::rendering::freeCameraForward(
                freeCameraPose_.yawRadians, freeCameraPose_.pitchRadians);
            lookAt(freeCameraPose_.position[0], freeCameraPose_.position[1],
                   freeCameraPose_.position[2], freeCameraPose_.position[0] + forward[0],
                   freeCameraPose_.position[1] + forward[1],
                   freeCameraPose_.position[2] + forward[2]);
        } else if (cameraView_ == 0) {
            const std::array<double, 3> target =
                transformLocalPoint(chassis, simulation.outsideCameraCenter());
            const double targetX = target[0];
            const double targetY = target[1];
            const double targetZ = target[2];
            const double horizontalDistance = cameraDistance_ * std::cos(cameraPitch_);
            const double orbitYaw = simulation.yaw() + cameraYaw_;
            const double eyeX = targetX + horizontalDistance * std::cos(orbitYaw);
            const double eyeY = targetY + horizontalDistance * std::sin(orbitYaw);
            const double eyeZ = targetZ + cameraDistance_ * std::sin(cameraPitch_);
            lookAt(eyeX, eyeY, eyeZ, targetX, targetY, targetZ);
        } else if (const VehicleCamera* camera = currentVehicleCamera(); camera != nullptr) {
            constexpr double DEGREES_TO_RADIANS = 3.141592653589793 / 180.0;
            const bool reflectionCamera = camera->kind == VehicleCameraKind::Reflexion ||
                                          camera->kind == VehicleCameraKind::Reflexion2;
            const double panSign = reflectionCamera ? 1.0 : -1.0;
            double pan = panSign * camera->pan * DEGREES_TO_RADIANS + viewLookYaw_;
            double tilt = camera->tilt * DEGREES_TO_RADIANS + viewLookPitch_;
            const double modelOffsetZ =
                playerVehicle_ != nullptr ? playerVehicle_->modelOffsetZ : 0.0;
            const std::array<double, 3> centerLocal = {camera->position[1], -camera->position[0],
                                                       camera->position[2] + modelOffsetZ};
            if (reflectionCamera && camera->followsViewer && renderingReflection_ &&
                reflectionViewerEyeLocalValid_ && reflectionReferenceCamera_ >= 0 &&
                static_cast<std::size_t>(reflectionReferenceCamera_) < vehicleCameras_.size()) {
                const VehicleCamera& reference =
                    vehicleCameras_[static_cast<std::size_t>(reflectionReferenceCamera_)];
                const double referencePan = -reference.pan * DEGREES_TO_RADIANS;
                const double referenceTilt = reference.tilt * DEGREES_TO_RADIANS;
                const std::array<double, 3> referenceEye = {
                    reference.position[1] -
                        std::cos(referenceTilt) * std::cos(referencePan) * reference.orbitDistance,
                    -reference.position[0] -
                        std::cos(referenceTilt) * std::sin(referencePan) * reference.orbitDistance,
                    reference.position[2] + modelOffsetZ -
                        std::sin(referenceTilt) * reference.orbitDistance};
                const std::array<double, 3> reflected =
                    openbus::rendering::mirrorReflectionDirection(
                        centerLocal,
                        {std::cos(tilt) * std::cos(pan), std::cos(tilt) * std::sin(pan),
                         std::sin(tilt)},
                        referenceEye, reflectionViewerEyeLocal_);
                pan = std::atan2(reflected[1], reflected[0]);
                tilt = std::asin(std::clamp(reflected[2], -1.0, 1.0));
            }
            const std::array<double, 3> direction = {
                std::cos(tilt) * std::cos(pan), std::cos(tilt) * std::sin(pan), std::sin(tilt)};
            const std::array<double, 3> upLocal = {-std::sin(tilt) * std::cos(pan),
                                                   -std::sin(tilt) * std::sin(pan), std::cos(tilt)};
            const std::array<double, 3> eyeLocal = {
                centerLocal[0] - direction[0] * camera->orbitDistance,
                centerLocal[1] - direction[1] * camera->orbitDistance,
                centerLocal[2] - direction[2] * camera->orbitDistance};
            const std::array<double, 3> targetLocal = {eyeLocal[0] + direction[0] * 3.0,
                                                       eyeLocal[1] + direction[1] * 3.0,
                                                       eyeLocal[2] + direction[2] * 3.0};
            const std::array<double, 3> eye = transformLocalPoint(chassis, eyeLocal);
            const std::array<double, 3> target = transformLocalPoint(chassis, targetLocal);
            const std::array<double, 3> up = {
                chassis.rotation[0] * upLocal[0] + chassis.rotation[1] * upLocal[1] +
                    chassis.rotation[2] * upLocal[2],
                chassis.rotation[3] * upLocal[0] + chassis.rotation[4] * upLocal[1] +
                    chassis.rotation[5] * upLocal[2],
                chassis.rotation[6] * upLocal[0] + chassis.rotation[7] * upLocal[1] +
                    chassis.rotation[8] * upLocal[2]};
            lookAt(eye[0], eye[1], eye[2], target[0], target[1], target[2], up[0], up[1], up[2]);
        } else {
            std::array<double, 3> eyeLocal = {5.65, 0.70, 0.70};
            std::array<double, 3> targetLocal = {8.65, 0.70, 0.72};
            switch (cameraView_) {
            case 2:
                eyeLocal = {4.25, -0.55, 0.78};
                targetLocal = {7.25, -0.55, 0.80};
                break;
            case 3:
                eyeLocal = {-2.75, 0.0, 0.72};
                targetLocal = {0.25, 0.0, 0.74};
                break;
            case 4:
                eyeLocal = {5.35, 1.05, 0.86};
                targetLocal = {5.35, 4.0, 0.82};
                break;
            case 5:
                eyeLocal = {5.35, -1.05, 0.86};
                targetLocal = {5.35, -4.0, 0.82};
                break;
            case 6:
                eyeLocal = {6.85, 0.0, 0.80};
                targetLocal = {3.85, 0.0, 0.80};
                break;
            case 7:
                eyeLocal = {-6.15, 0.0, 0.80};
                targetLocal = {-3.15, 0.0, 0.80};
                break;
            case 8:
                eyeLocal = {0.0, 1.05, 1.30};
                targetLocal = {0.0, 4.0, 1.30};
                break;
            case 9:
                eyeLocal = {0.0, -1.05, 1.30};
                targetLocal = {0.0, -4.0, 1.30};
                break;
            default:
                break;
            }
            const std::array<double, 3> eye = transformLocalPoint(chassis, eyeLocal);
            const double directionX = targetLocal[0] - eyeLocal[0];
            const double directionY = targetLocal[1] - eyeLocal[1];
            const double directionZ = targetLocal[2] - eyeLocal[2];
            const double horizontalLength = std::hypot(directionX, directionY);
            const double distance = std::sqrt(directionX * directionX + directionY * directionY +
                                              directionZ * directionZ);
            const double baseYaw = std::atan2(directionY, directionX);
            const double basePitch = std::atan2(directionZ, horizontalLength);
            const double lookYaw = baseYaw + viewLookYaw_;
            const double lookPitch = basePitch + viewLookPitch_;
            const std::array<double, 3> target = transformLocalPoint(
                chassis, {eyeLocal[0] + distance * std::cos(lookPitch) * std::cos(lookYaw),
                          eyeLocal[1] + distance * std::cos(lookPitch) * std::sin(lookYaw),
                          eyeLocal[2] + distance * std::sin(lookPitch)});
            lookAt(eye[0], eye[1], eye[2], target[0], target[1], target[2]);
        }
        if (!renderingReflection_) {
            const auto& view = modelViewMatrix();
            const std::array<double, 3> eyeWorld = {
                -(view[0] * view[12] + view[1] * view[13] + view[2] * view[14]),
                -(view[4] * view[12] + view[5] * view[13] + view[6] * view[14]),
                -(view[8] * view[12] + view[9] * view[13] + view[10] * view[14])};
            const std::array<double, 3> forwardWorld = {-view[2], -view[6], -view[10]};
            const std::array<double, 3> upWorld = {view[1], view[5], view[9]};
            if (!captureMode_) {
                lastMainCameraPose_.position = eyeWorld;
                lastMainCameraPose_.yawRadians = std::atan2(forwardWorld[1], forwardWorld[0]);
                lastMainCameraPose_.pitchRadians =
                    std::atan2(forwardWorld[2], std::hypot(forwardWorld[0], forwardWorld[1]));
                lastMainCameraPoseValid_ = true;
                trafficCameraViewProjection_ =
                    multiplyMatrix4(openbus::rendering::projectionMatrix(), view);
            }
            const auto toLocalPoint = [&chassis](const std::array<double, 3>& world) {
                const double x = world[0] - chassis.position[0];
                const double y = world[1] - chassis.position[1];
                const double z = world[2] - chassis.position[2];
                return std::array<double, 3>{
                    chassis.rotation[0] * x + chassis.rotation[3] * y + chassis.rotation[6] * z,
                    chassis.rotation[1] * x + chassis.rotation[4] * y + chassis.rotation[7] * z,
                    chassis.rotation[2] * x + chassis.rotation[5] * y + chassis.rotation[8] * z};
            };
            const auto toLocalDirection = [&chassis](const std::array<double, 3>& world) {
                return std::array<double, 3>{
                    chassis.rotation[0] * world[0] + chassis.rotation[3] * world[1] +
                        chassis.rotation[6] * world[2],
                    chassis.rotation[1] * world[0] + chassis.rotation[4] * world[1] +
                        chassis.rotation[7] * world[2],
                    chassis.rotation[2] * world[0] + chassis.rotation[5] * world[1] +
                        chassis.rotation[8] * world[2]};
            };
            soundEngine_.setListenerPose(toLocalPoint(eyeWorld), toLocalDirection(forwardWorld),
                                         toLocalDirection(upWorld));
            reflectionViewerEyeLocal_ = toLocalPoint(eyeWorld);
            reflectionViewerEyeLocalValid_ = true;
        }
    }
    {
        TraceScope phase("render", "RenderLoop::draw.visibility");
        if (!renderingReflection_) {
            const RenderViewContext context = isExteriorView() ? RenderViewContext::PlayerExterior
                                                               : RenderViewContext::PlayerInterior;
            if (playerVehicle_ != nullptr) {
                playerVehicle_->setOdeSimulation(simulation);
            }
            for (const std::unique_ptr<Vehicle>& vehicle : vehicles_) {
                if (vehicle->mapTrafficRouteFinished ||
                    (vehicle->followsMapTrafficRoute && !vehicle->mapTrafficTileActive)) {
                    continue;
                }
                pushMatrix();
                if (vehicle.get() == playerVehicle_) {
                    applyPose(chassis);
                    vehicle->prepareFrameVisibility(context, framebufferWidth_, framebufferHeight_);
                } else {
                    applyVehiclePlacement(
                        makeModelRootPlacement(vehicle->placement, vehicle->modelOffsetZ));
                    vehicle->prepareFrameVisibility(RenderViewContext::NonPlayer, framebufferWidth_,
                                                    framebufferHeight_);
                }
                popMatrix();
            }
        }
    }
    if (!renderingReflection_ && !freeCameraEnabled_ && !mouseLookEnabled_ &&
        playerVehicle_ != nullptr && mouseInteractionPending_) {
        TraceScope phase("input", "RenderLoop::draw.interaction");
        // Reflections, captures and repeated draws are not additional input frames.
        mouseInteractionPending_ = false;
        double cursorX = 0.0;
        double cursorY = 0.0;
        glfwGetCursorPos(window_, &cursorX, &cursorY);
        int windowWidth = 1;
        int windowHeight = 1;
        glfwGetWindowSize(window_, &windowWidth, &windowHeight);
        const double framebufferScaleX =
            static_cast<double>(framebufferWidth_) / static_cast<double>(std::max(windowWidth, 1));
        const double framebufferScaleY = static_cast<double>(framebufferHeight_) /
                                         static_cast<double>(std::max(windowHeight, 1));
        const double framebufferCursorX = cursorX * framebufferScaleX;
        const double framebufferCursorY = cursorY * framebufferScaleY;
        pushMatrix();
        applyPose(chassis);
        const RenderViewContext interactionContext = isExteriorView()
                                                         ? RenderViewContext::PlayerExterior
                                                         : RenderViewContext::PlayerInterior;
        if (!mouseControlEnabled_ && benchmarkClickDiscoveryPending_) {
            TraceScope trace("benchmark", "Benchmark.discoverClickable");
            benchmarkClickDiscoveryPending_ = false;
            benchmarkClickDiscoveryComplete_ = true;
            if (playerVehicle_->hasVisibleClickable(interactionContext)) {
                constexpr int columns = 40;
                constexpr int rows = 24;
                for (int row = 1; row < rows && !benchmarkClickTargetFound_; ++row) {
                    const double y = static_cast<double>(framebufferHeight_) * row / rows;
                    for (int column = 1; column < columns; ++column) {
                        const double x = static_cast<double>(framebufferWidth_) * column / columns;
                        if (playerVehicle_
                                ->mouseEventAt(x, y, framebufferWidth_, framebufferHeight_,
                                               interactionContext)
                                .empty()) {
                            continue;
                        }
                        benchmarkClickFramebufferX_ = x;
                        benchmarkClickFramebufferY_ = y;
                        benchmarkClickTargetFound_ = true;
                        break;
                    }
                }
            }
        }
        if (!mouseControlEnabled_ && benchmarkClickQueued_ && benchmarkClickTargetFound_) {
            pendingMouseClick_ = true;
            pendingMouseClickX_ = benchmarkClickFramebufferX_ / framebufferScaleX;
            pendingMouseClickY_ = benchmarkClickFramebufferY_ / framebufferScaleY;
        }
        benchmarkClickQueued_ = false;
        if (!mouseControlEnabled_) {
            const int interactionContextValue = static_cast<int>(interactionContext);
            const std::uint64_t clickableRevision = playerVehicle_->clickableRevision();
            // Pose changes every simulation step; throttle stationary-pointer hover checks.
            const double currentTime = glfwGetTime();
            const bool hoverCacheMatches =
                clickableHoverCacheValid_ && clickableHoverCacheX_ == framebufferCursorX &&
                clickableHoverCacheY_ == framebufferCursorY &&
                clickableHoverCacheWidth_ == framebufferWidth_ &&
                clickableHoverCacheHeight_ == framebufferHeight_ &&
                clickableHoverCacheContext_ == interactionContextValue &&
                clickableHoverCacheCameraView_ == cameraView_ &&
                clickableHoverCacheCameraYaw_ == cameraYaw_ &&
                clickableHoverCacheCameraPitch_ == cameraPitch_ &&
                clickableHoverCacheLookYaw_ == viewLookYaw_ &&
                clickableHoverCacheLookPitch_ == viewLookPitch_ &&
                clickableHoverCacheRevision_ == clickableRevision &&
                currentTime - clickableHoverCacheTimestamp_ < clickableHoverRefreshInterval_;
            if (!hoverCacheMatches) {
                clickableHoverCacheValid_ = true;
                clickableHoverCacheX_ = framebufferCursorX;
                clickableHoverCacheY_ = framebufferCursorY;
                clickableHoverCacheWidth_ = framebufferWidth_;
                clickableHoverCacheHeight_ = framebufferHeight_;
                clickableHoverCacheContext_ = interactionContextValue;
                clickableHoverCacheCameraView_ = cameraView_;
                clickableHoverCacheCameraYaw_ = cameraYaw_;
                clickableHoverCacheCameraPitch_ = cameraPitch_;
                clickableHoverCacheLookYaw_ = viewLookYaw_;
                clickableHoverCacheLookPitch_ = viewLookPitch_;
                clickableHoverCacheRevision_ = clickableRevision;
                clickableHoverCacheTimestamp_ = currentTime;
                clickableHoverCacheHit_ =
                    playerVehicle_->hasVisibleClickable(interactionContext) &&
                    playerVehicle_->hasClickableAt(framebufferCursorX, framebufferCursorY,
                                                   framebufferWidth_, framebufferHeight_,
                                                   interactionContext);
            }
        } else {
            pendingMouseClick_ = false;
            clickableHoverCacheValid_ = false;
            clickableHoverCacheHit_ = false;
        }
        const bool hoveringClickable = !mouseControlEnabled_ && clickableHoverCacheHit_;
        GLFWcursor* cursor = mouseControlEnabled_ ? mouseSteeringCursor_
                             : hoveringClickable  ? clickableCursor_
                                                  : nullptr;
        glfwSetCursor(window_, cursor);
        const auto releaseMouse = [this](const std::string& event) {
            if (playerVehicle_->scripts) {
                playerVehicle_->scripts->invokeMouseRelease(event);
            }
        };
        if (mouseControlEnabled_) {
            pendingMouseClick_ = false;
            mouseInteraction_.cancel(releaseMouse);
        } else if (pendingMouseClick_) {
            std::string event = playerVehicle_->mouseEventAt(
                pendingMouseClickX_ * framebufferScaleX, pendingMouseClickY_ * framebufferScaleY,
                framebufferWidth_, framebufferHeight_, interactionContext);
            if (clickableDebugOverlay_) {
                gameLog.Log("Mouse press picked event=" +
                            (event.empty() ? std::string("none") : event));
            }
            mouseDragDiagnosticLogged_ = false;
            mouseInteraction_.press(
                std::move(event), pendingMouseClickX_, pendingMouseClickY_,
                [this](const std::string& pressedEvent) {
                    playerVehicle_->handleMouseClick(pressedEvent);
                },
                releaseMouse);
            pendingMouseClick_ = false;
        } else {
            mouseInteraction_.update(
                true, leftMousePressed_, cursorX, cursorY,
                [this](const std::string& event, double deltaX, double deltaY, double x, double y) {
                    if (playerVehicle_->scripts) {
                        if (clickableDebugOverlay_ && !mouseDragDiagnosticLogged_ &&
                            (deltaX != 0.0 || deltaY != 0.0)) {
                            std::string normalizedEvent = event;
                            std::transform(normalizedEvent.begin(), normalizedEvent.end(),
                                           normalizedEvent.begin(), [](unsigned char character) {
                                               return std::isalnum(character)
                                                          ? static_cast<char>(
                                                                std::tolower(character))
                                                          : '_';
                                           });
                            const std::string functionName = "trigger_" + normalizedEvent + "_drag";
                            const bool dragHandlerFound =
                                playerVehicle_->scripts->hasScriptEntryPoint(functionName);
                            mouseDragDiagnosticLogged_ = true;
                            playerVehicle_->scripts->invokeMouseDrag(event, deltaX, deltaY, x, y);
                            std::ostringstream description;
                            description
                                << "Mouse drag event=" << event << " dx=" << deltaX
                                << " dy=" << deltaY
                                << " drag_handler=" << (dragHandlerFound ? "found" : "missing");
                            if (normalizedEvent == "cp_rollo") {
                                description << " cp_rollo_pos="
                                            << playerVehicle_->variables.get("cp_rollo_pos");
                            } else if (normalizedEvent == "cp_rollof") {
                                description << " cp_rollof_pos="
                                            << playerVehicle_->variables.get("cp_rollof_pos");
                            }
                            gameLog.Log(description.str());
                            return;
                        }
                        playerVehicle_->scripts->invokeMouseDrag(event, deltaX, deltaY, x, y);
                    }
                },
                releaseMouse);
        }
        popMatrix();
    }
    {
        TraceScope phase("render", "RenderLoop::draw.reflections");
        renderReflectionViews(simulation);
    }
    openbus::rendering::setLightingViewMatrix(modelViewMatrix());
    {
        TraceScope phase("render", "RenderLoop::draw.ground");
        if (mapRenderer_ != nullptr) {
            glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
            glEnable(GL_DEPTH_TEST);
            glDepthFunc(GL_LESS);
            glDepthMask(GL_TRUE);
            glDisable(GL_BLEND);
            glDisable(GL_CULL_FACE);
            glDisable(GL_POLYGON_OFFSET_FILL);
            mapRenderer_->draw();
            for (const HumanTrafficState& human : humanTrafficActors_) {
                if (!human.populationEnabled) {
                    continue;
                }
                const auto pose = openbus::map::sampleMapTrafficRoute(
                    human.actor.route, human.actor.distanceMeters, human.actor.lateralOffsetMeters);
                if (!pose) {
                    throw std::runtime_error("Cannot sample a pedestrian render pose");
                }
                mapRenderer_->drawHumanTrafficActor(std::filesystem::path(human.actor.modelPath),
                                                    *pose, human.gait.phaseRadians,
                                                    human.gait.weight);
            }
        } else if (!captureMode_) {
            glEnable(GL_DEPTH_TEST);
            glDepthFunc(GL_LESS);
            glDepthMask(GL_TRUE);
            glDisable(GL_BLEND);
            glDisable(GL_CULL_FACE);
            glDisable(GL_POLYGON_OFFSET_FILL);
            drawGround(simulation.roadBumps());
        }
    }
    const RenderViewContext context = (renderingReflection_ || isExteriorView())
                                          ? RenderViewContext::PlayerExterior
                                          : RenderViewContext::PlayerInterior;
    {
        TraceScope phase("render", "RenderLoop::draw.model");
        bool playerDrawn = false;
        const auto modelLoadDeadline =
            std::chrono::steady_clock::now() + std::chrono::milliseconds(4);
        const auto drawVehicles = [&](VehicleRenderPass renderPass, bool playerVehicles) {
            for (const std::unique_ptr<Vehicle>& vehicle : vehicles_) {
                if (vehicle->mapTrafficRouteFinished ||
                    (vehicle->followsMapTrafficRoute && !vehicle->mapTrafficTileActive) ||
                    (vehicle.get() == playerVehicle_) != playerVehicles) {
                    continue;
                }
                const RenderViewContext vehicleContext =
                    vehicle.get() == playerVehicle_ ? context : RenderViewContext::NonPlayer;
                VehiclePlacement placedVehicle = vehicle->placement;
                const double dx = placedVehicle.position[0] - chassis.position[0];
                const double dy = placedVehicle.position[1] - chassis.position[1];
                const double dz = placedVehicle.position[2] - chassis.position[2];
                if (renderPass == VehicleRenderPass::Opaque) {
                    vehicle->ensureLoaded(vehicleContext, std::sqrt(dx * dx + dy * dy + dz * dz),
                                          modelLoadDeadline);
                }
                if (!vehicle->loaded || vehicle->displayLists.empty()) {
                    continue;
                }
                pushMatrix();
                if (vehicle.get() == playerVehicle_) {
                    applyPose(chassis);
                    if (renderPass == VehicleRenderPass::Opaque) {
                        playerDrawn = true;
                    }
                } else {
                    const std::optional<double> terrainHeight =
                        mapRenderer_ != nullptr && !vehicle->followsMapTrafficRoute
                            ? mapRenderer_->terrainHeightAt(placedVehicle.position[0],
                                                            placedVehicle.position[1])
                            : std::nullopt;
                    if (terrainHeight.has_value()) {
                        placedVehicle.position[2] = *terrainHeight;
                    }
                    applyVehiclePlacement(
                        makeModelRootPlacement(placedVehicle, vehicle->modelOffsetZ));
                }
                Matrix4 aiGroundShadowTransform = {};
                const Matrix4* shadowTransform = nullptr;
                if (vehicle.get() == playerVehicle_) {
                    if (canGroundClampShadow) {
                        shadowTransform = &groundShadowTransform;
                    }
                } else {
                    const VehiclePlacement rootPlacement =
                        makeModelRootPlacement(placedVehicle, vehicle->modelOffsetZ);
                    const double fallbackGroundHeight = placedVehicle.position[2];
                    const double shadowGroundHeight =
                        mapRenderer_ != nullptr ? mapRenderer_
                                                      ->terrainHeightAt(placedVehicle.position[0],
                                                                        placedVehicle.position[1])
                                                      .value_or(fallbackGroundHeight)
                                                : fallbackGroundHeight;
                    aiGroundShadowTransform =
                        makeGroundShadowTransform(rootPlacement, vehicle->modelOffsetZ,
                                                  shadowGroundHeight + GROUND_SHADOW_OFFSET_Z);
                    shadowTransform = &aiGroundShadowTransform;
                }
                vehicle->draw(vehicleContext, renderPass, shadowTransform);
                popMatrix();
            }
        };
        const auto drawTypeFourScenery = [&]() {
            if (mapRenderer_ == nullptr || renderingReflection_) {
                return;
            }
            glEnable(GL_DEPTH_TEST);
            glDepthFunc(GL_LESS);
            glDepthMask(GL_TRUE);
            glDisable(GL_BLEND);
            mapRenderer_->draw(true);
        };
        drawVehicles(VehicleRenderPass::Opaque, false);
        drawVehicles(VehicleRenderPass::Transparent, false);
        if (context == RenderViewContext::PlayerInterior) {
            drawTypeFourScenery();
        }
        drawVehicles(VehicleRenderPass::Opaque, true);
        drawVehicles(VehicleRenderPass::Transparent, true);
        if (context == RenderViewContext::PlayerExterior) {
            drawTypeFourScenery();
        }
        if (!playerDrawn && collision.enabled && !collision.mesh) {
            pushMatrix();
            applyPose(chassis);
            translate(collision.offsetX, collision.offsetY, collision.offsetZ);
            drawBox(collision.length, collision.width, collision.height, 0.85, 0.70, 0.08);
            popMatrix();
        }
        if (!renderingReflection_ && !mouseControlEnabled_ && clickableDebugOverlay_ &&
            playerVehicle_ != nullptr) {
            pushMatrix();
            applyPose(chassis);
            playerVehicle_->drawClickableDebug(context);
            popMatrix();
        }
    }
    if (!renderingReflection_ && collisionDebugOverlay_) {
        TraceScope phase("render", "RenderLoop::draw.collisionDebug");
        const std::uint64_t collisionRevision = simulation.collisionDebugRevision();
        if (!collisionWireframeBuilt_ || collisionWireframeRevision_ != collisionRevision) {
            TraceScope phase("render", "RenderLoop::draw.collisionDebug.build");
            collisionWireframeVertices_.clear();
            const std::vector<StaticCollisionMesh> meshes = simulation.collisionDebugMeshes();
            std::size_t triangleCount = 0;
            for (const StaticCollisionMesh& mesh : meshes) {
                if (mesh.vertices.size() % 3 != 0 || mesh.indices.size() % 3 != 0) {
                    continue;
                }
                triangleCount += mesh.indices.size() / 3;
                for (std::size_t index = 0; index + 2 < mesh.indices.size(); index += 3) {
                    const std::array<int, 3> triangle = {
                        mesh.indices[index], mesh.indices[index + 1], mesh.indices[index + 2]};
                    for (int edge = 0; edge < 3; ++edge) {
                        for (const int vertexIndex :
                             {triangle[static_cast<std::size_t>(edge)],
                              triangle[static_cast<std::size_t>((edge + 1) % 3)]}) {
                            if (vertexIndex < 0 || static_cast<std::size_t>(vertexIndex) * 3 + 2 >=
                                                       mesh.vertices.size()) {
                                continue;
                            }
                            const std::size_t vertexOffset =
                                static_cast<std::size_t>(vertexIndex) * 3;
                            collisionWireframeVertices_.push_back(
                                {static_cast<float>(mesh.vertices[vertexOffset]),
                                 static_cast<float>(mesh.vertices[vertexOffset + 1]),
                                 static_cast<float>(mesh.vertices[vertexOffset + 2] + 0.025), 1.0F,
                                 0.58F, 0.08F});
                        }
                    }
                }
            }
            collisionWireframeBuilt_ = true;
            collisionWireframeRevision_ = collisionRevision;
            gameLog.Log("Collision wireframe cached: " + std::to_string(triangleCount) +
                        " triangles across " + std::to_string(meshes.size()) +
                        " active terrain/road/scenery meshes");
        }

        GLboolean depthWasEnabled = GL_FALSE;
        GLboolean depthWriteWasEnabled = GL_TRUE;
        {
            TraceScope phase("render", "RenderLoop::draw.collisionDebug.queryState");
            depthWasEnabled = glIsEnabled(GL_DEPTH_TEST);
            glGetBooleanv(GL_DEPTH_WRITEMASK, &depthWriteWasEnabled);
        }
        glEnable(GL_DEPTH_TEST);
        glDepthMask(GL_FALSE);
        openbus::rendering::drawStaticPrimitives(collisionWireframeBuffer_,
                                                 collisionWireframeVertices_, GL_LINES, 1.5F,
                                                 collisionRevision);
        if (collision.enabled && !collision.mesh) {
            pushMatrix();
            applyPose(chassis);
            translate(collision.offsetX, collision.offsetY, collision.offsetZ);
            openbus::rendering::drawBox(collision.length, collision.width, collision.height, 1.0,
                                        0.15, 0.12);
            popMatrix();
        }
        glDepthMask(depthWriteWasEnabled);
        if (!depthWasEnabled) {
            glDisable(GL_DEPTH_TEST);
        }
    }
    if (!renderingReflection_ && trafficSplineDebugOverlay_ && mapRenderer_ != nullptr) {
        const std::vector<std::pair<int, int>>& activeTiles = mapRenderer_->activeTileCoordinates();
        if (!trafficSplineDebugBuilt_ || trafficSplineDebugTiles_ != activeTiles) {
            trafficSplineDebugVertices_.clear();
            const SceneryTrafficEndpointIndex sceneryEndpoints(*mapRenderer_, activeTiles);
            appendTrafficSplineDebugVertices(*mapRenderer_, openbus::map::MAP_SPLINE_PATH_TYPE_ROAD,
                                             activeTiles, {0.15F, 0.9F, 1.0F}, sceneryEndpoints,
                                             trafficSplineDebugVertices_);
            appendTrafficSplineDebugVertices(
                *mapRenderer_, openbus::map::MAP_SPLINE_PATH_TYPE_HUMAN, activeTiles,
                {1.0F, 0.25F, 0.9F}, sceneryEndpoints, trafficSplineDebugVertices_);
            appendSceneryTrafficPathDebugVertices(*mapRenderer_, activeTiles,
                                                  trafficSplineDebugVertices_);
            trafficSplineDebugTiles_ = activeTiles;
            trafficSplineDebugBuilt_ = true;
            ++trafficSplineDebugRevision_;
        }

        if (!trafficSplineDebugVertices_.empty()) {
            const GLboolean depthWasEnabled = glIsEnabled(GL_DEPTH_TEST);
            GLboolean depthWriteWasEnabled = GL_TRUE;
            glGetBooleanv(GL_DEPTH_WRITEMASK, &depthWriteWasEnabled);
            glDisable(GL_DEPTH_TEST);
            glDepthMask(GL_FALSE);
            openbus::rendering::drawStaticPrimitives(trafficSplineDebugBuffer_,
                                                     trafficSplineDebugVertices_, GL_LINES, 2.0F,
                                                     trafficSplineDebugRevision_);
            glDepthMask(depthWriteWasEnabled);
            if (depthWasEnabled) {
                glEnable(GL_DEPTH_TEST);
            }
        }

        std::vector<openbus::rendering::PrimitiveVertex> behaviorVertices;
        std::vector<openbus::rendering::PrimitiveVertex> vehicleBodyVertices;
        std::vector<openbus::map::MapTrafficPose> renderedSignalStops;
        const auto appendSignalStopBox = [&](const openbus::map::MapTrafficActor& actor) {
            const auto stop =
                openbus::map::mapTrafficNextSignalStopDebugInfo(actor, simulation.simulationTime());
            if (!stop.has_value() || stop->distanceAheadMeters > 50.0 ||
                !openbus::map::mapTrafficPoseOnActiveTile(stop->pose, activeTiles)) {
                return;
            }
            const bool alreadyRendered = std::any_of(
                renderedSignalStops.begin(), renderedSignalStops.end(),
                [&](const openbus::map::MapTrafficPose& rendered) {
                    return std::hypot(rendered.position[0] - stop->pose.position[0],
                                      rendered.position[1] - stop->pose.position[1]) < 0.5 &&
                           std::abs(
                               std::remainder(rendered.headingRadians - stop->pose.headingRadians,
                                              std::numbers::pi)) < 0.15;
                });
            if (alreadyRendered) {
                return;
            }
            renderedSignalStops.push_back(stop->pose);
            const std::array<float, 3> color = stop->blocked
                                                   ? std::array<float, 3>{1.0F, 0.12F, 0.08F}
                                                   : std::array<float, 3>{0.1F, 1.0F, 0.2F};
            appendTrafficBehaviorBox(behaviorVertices, stop->pose, 0.35,
                                     std::max(actor.widthMeters + 1.0, 2.5), 0.12, color);
        };
        for (const std::unique_ptr<Vehicle>& vehicle : vehicles_) {
            if (vehicle->followsMapTrafficRoute || vehicle->mapTrafficRouteFinished ||
                !vehicle->loaded || vehicle->displayLists.empty()) {
                continue;
            }
            openbus::map::MapTrafficPose pose;
            if (vehicle.get() == playerVehicle_) {
                const auto chassis = simulation.chassisPose();
                pose.position = chassis.position;
                pose.headingRadians = std::atan2(chassis.rotation[0], chassis.rotation[3]);
            } else {
                pose.position = vehicle->placement.position;
                pose.headingRadians =
                    (90.0 - vehicle->placement.yawDegrees) * std::numbers::pi / 180.0;
            }
            if (openbus::map::mapTrafficPoseOnActiveTile(pose, activeTiles)) {
                std::vector<openbus::rendering::PrimitiveVertex>& vertices =
                    vehicle.get() == playerVehicle_ ? behaviorVertices : vehicleBodyVertices;
                appendTrafficBehaviorBox(vertices, pose, vehicle->trafficLengthMeters,
                                         vehicle->trafficWidthMeters, vehicle->trafficHeightMeters,
                                         {0.95F, 0.95F, 0.95F});
            }
        }
        for (const TrafficVehicleState& state : trafficVehicles_) {
            if (!state.populationEnabled || state.vehicle == nullptr ||
                !state.vehicle->mapTrafficTileActive) {
                continue;
            }
            const auto pose = openbus::map::sampleMapTrafficRoute(
                state.actor.route, state.actor.distanceMeters, state.actor.lateralOffsetMeters);
            if (!pose.has_value()) {
                continue;
            }
            const bool onJunction = openbus::map::mapTrafficRoutePositionOnJunctionPath(
                state.actor.route, state.actor.distanceMeters);
            if (state.vehicle->loaded && !state.vehicle->displayLists.empty()) {
                appendTrafficBehaviorBox(vehicleBodyVertices, *pose, state.actor.lengthMeters,
                                         state.actor.widthMeters,
                                         state.vehicle->trafficHeightMeters,
                                         state.deadlocked ? std::array<float, 3>{1.0F, 0.1F, 0.1F}
                                         : onJunction ? std::array<float, 3>{0.15F, 0.45F, 1.0F}
                                                      : std::array<float, 3>{0.15F, 1.0F, 0.2F});
            }
            appendSignalStopBox(state.actor);

            const double reservationHorizon = state.actor.speedMetersPerSecond * 3.0;
            for (double ahead = 1.0; ahead < reservationHorizon; ahead += 2.0) {
                const double reservationDistance = state.actor.distanceMeters + ahead;
                if (!openbus::map::mapTrafficRoutePositionOnJunctionPath(state.actor.route,
                                                                         reservationDistance)) {
                    continue;
                }
                const auto reservationPose = openbus::map::sampleMapTrafficRoute(
                    state.actor.route, reservationDistance, state.actor.lateralOffsetMeters);
                if (!reservationPose.has_value() ||
                    !openbus::map::mapTrafficPoseOnActiveTile(*reservationPose, activeTiles)) {
                    continue;
                }
                appendTrafficBehaviorBox(behaviorVertices, *reservationPose,
                                         state.actor.lengthMeters, state.actor.widthMeters, 0.12,
                                         {1.0F, 0.5F, 0.05F});
            }
        }
        for (const HumanTrafficState& human : humanTrafficActors_) {
            if (!human.populationEnabled) {
                continue;
            }
            const auto pose = openbus::map::sampleMapTrafficRoute(
                human.actor.route, human.actor.distanceMeters, human.actor.lateralOffsetMeters);
            if (!pose.has_value() ||
                !openbus::map::mapTrafficPoseOnActiveTile(*pose, activeTiles)) {
                continue;
            }
            appendTrafficBehaviorBox(behaviorVertices, *pose, human.feetDistanceMeters * 1.25,
                                     human.feetDistanceMeters * 2.1, human.heightMeters,
                                     {1.0F, 0.25F, 0.9F});
            appendSignalStopBox(human.actor);
        }
        if (!vehicleBodyVertices.empty()) {
            const GLboolean depthWasEnabled = glIsEnabled(GL_DEPTH_TEST);
            GLboolean depthWriteWasEnabled = GL_TRUE;
            glGetBooleanv(GL_DEPTH_WRITEMASK, &depthWriteWasEnabled);
            glEnable(GL_DEPTH_TEST);
            glDepthMask(GL_FALSE);
            openbus::rendering::drawStaticPrimitives(trafficBehaviorDebugBuffer_,
                                                     vehicleBodyVertices, GL_LINES, 1.5F,
                                                     ++trafficBehaviorDebugRevision_);
            glDepthMask(depthWriteWasEnabled);
            if (!depthWasEnabled) {
                glDisable(GL_DEPTH_TEST);
            }
        }
        if (!behaviorVertices.empty()) {
            const GLboolean depthWasEnabled = glIsEnabled(GL_DEPTH_TEST);
            GLboolean depthWriteWasEnabled = GL_TRUE;
            glGetBooleanv(GL_DEPTH_WRITEMASK, &depthWriteWasEnabled);
            glDisable(GL_DEPTH_TEST);
            glDepthMask(GL_FALSE);
            openbus::rendering::drawStaticPrimitives(trafficBehaviorDebugBuffer_, behaviorVertices,
                                                     GL_LINES, 1.5F,
                                                     ++trafficBehaviorDebugRevision_);
            glDepthMask(depthWriteWasEnabled);
            if (depthWasEnabled) {
                glEnable(GL_DEPTH_TEST);
            }
        }
    }
    if (!renderingReflection_ && (cameraView_ == 0 || freeCameraEnabled_)) {
        const float precipitationTypeValue = simulationState_.sharedVariables().get("preciptype");
        const float precipitationRate = simulationState_.sharedVariables().get("preciprate");
        if (!std::isfinite(precipitationTypeValue) || precipitationTypeValue < 0.0F ||
            precipitationTypeValue > 2.0F ||
            std::floor(precipitationTypeValue) != precipitationTypeValue ||
            !std::isfinite(precipitationRate) || precipitationRate < 0.0F ||
            precipitationRate > 1.0F) {
            throw std::runtime_error("Weather system variables contain invalid precipitation data");
        }
        const auto precipitationType = static_cast<openbus::rendering::PrecipitationType>(
            static_cast<int>(precipitationTypeValue));
        const std::vector<openbus::rendering::PrimitiveVertex> precipitationVertices =
            openbus::rendering::makePrecipitationVertices(precipitationType, precipitationRate,
                                                          simulation.simulationTime());
        if (!precipitationVertices.empty()) {
            const openbus::rendering::Matrix4 previousProjection =
                openbus::rendering::projectionMatrix();
            const openbus::rendering::Matrix4 previousModelView =
                openbus::rendering::modelViewMatrix();
            const GLboolean depthWasEnabled = glIsEnabled(GL_DEPTH_TEST);
            const GLboolean blendWasEnabled = glIsEnabled(GL_BLEND);
            GLfloat previousLineWidth = 1.0F;
            GLfloat previousPointSize = 1.0F;
            glGetFloatv(GL_LINE_WIDTH, &previousLineWidth);
            glGetFloatv(GL_POINT_SIZE, &previousPointSize);
            openbus::rendering::setProjectionMatrix(openbus::rendering::identityMatrix());
            openbus::rendering::setModelViewMatrix(openbus::rendering::identityMatrix());
            glDisable(GL_DEPTH_TEST);
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            if (precipitationType == openbus::rendering::PrecipitationType::Rain) {
                openbus::rendering::drawPrimitives(precipitationVertices, GL_LINES, 1.0F);
            } else {
                glPointSize(2.0F);
                openbus::rendering::drawPrimitives(precipitationVertices, GL_POINTS);
            }
            glLineWidth(previousLineWidth);
            glPointSize(previousPointSize);
            openbus::rendering::setProjectionMatrix(previousProjection);
            openbus::rendering::setModelViewMatrix(previousModelView);
            if (depthWasEnabled) {
                glEnable(GL_DEPTH_TEST);
            }
            if (!blendWasEnabled) {
                glDisable(GL_BLEND);
            }
        }
    }
    if (!renderingReflection_ && statusHudTexture_ != 0) {
        TraceScope phase("render", "RenderLoop::draw.statusHud");
        {
            TraceScope phase("render", "RenderLoop::draw.statusHud.update");
            const openbus::rendering::MapRenderStats mapStats =
                mapRenderer_ != nullptr ? mapRenderer_->renderStats()
                                        : openbus::rendering::MapRenderStats{};
            const std::size_t spawnedAiVehicleCount = trafficVehicleCount();
            std::ostringstream statusText;
            statusText << "X " << std::fixed << std::setprecision(1) << simulation.positionX()
                       << " Y " << simulation.positionY() << " Z " << simulation.positionZ() << '\n'
                       << "CHUNKS " << mapStats.activeTiles << '/' << mapStats.totalTiles
                       << " TERRAIN " << mapStats.activeTerrainTiles << '/'
                       << mapStats.totalTerrainTiles << " OBJECTS " << mapStats.activeSceneryObjects
                       << '/' << mapStats.totalSceneryObjects << '\n'
                       << "ROADS " << mapStats.activeRoadBatches << '/' << mapStats.totalRoadBatches
                       << " COLLISION TILES " << simulation.loadedMapCollisionTileCount()
                       << " AI VEHICLES " << spawnedAiVehicleCount;
            const std::string currentStatus = statusText.str();
            if (currentStatus != statusHudText_) {
                const std::vector<std::uint8_t> pixels = [&]() {
                    TraceScope phase("render", "RenderLoop::draw.statusHud.rasterize");
                    return makeStatusHudPixels(currentStatus);
                }();
                TraceScope phase("render", "RenderLoop::draw.statusHud.upload");
                pglActiveTexture(GL_TEXTURE0);
                glBindTexture(GL_TEXTURE_2D, statusHudTexture_);
                glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, STATUS_HUD_TEXTURE_WIDTH,
                                STATUS_HUD_TEXTURE_HEIGHT, GL_RGBA, GL_UNSIGNED_BYTE,
                                pixels.data());
                openbus::rendering::invalidateTextureBindings();
                statusHudText_ = currentStatus;
            }
        }

        {
            TraceScope phase("render", "RenderLoop::draw.statusHud.submit");
            glDisable(GL_DEPTH_TEST);
            glDisable(GL_CULL_FACE);
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            const float scale = std::min(
                {1.5F,
                 static_cast<float>(std::max(framebufferWidth_ - 24, 1)) / STATUS_HUD_TEXTURE_WIDTH,
                 static_cast<float>(std::max(framebufferHeight_ - 24, 1)) /
                     STATUS_HUD_TEXTURE_HEIGHT});
            const float hudWidth = STATUS_HUD_TEXTURE_WIDTH * scale;
            const float hudHeight = STATUS_HUD_TEXTURE_HEIGHT * scale;
            const float leftPixels = 12.0F;
            const float bottomPixels = framebufferHeight_ - 12.0F - hudHeight;
            openbus::rendering::drawTextureQuad(
                statusHudTexture_, -1.0F + 2.0F * leftPixels / framebufferWidth_,
                -1.0F + 2.0F * bottomPixels / framebufferHeight_,
                2.0F * hudWidth / framebufferWidth_, 2.0F * hudHeight / framebufferHeight_);
            // The final overlay restores the scene pass's depth/blend state.
            // Face culling stays disabled; scene shaders handle sidedness.
            glEnable(GL_DEPTH_TEST);
            glDisable(GL_BLEND);
        }
    }
    {
        TraceScope phase("render", "RenderLoop::draw.windowTitle");
        const double titleUpdateTime = glfwGetTime();
        if (!renderingReflection_ && !captureMode_) {
            if (!fpsSampleInitialized_) {
                fpsSampleStartTime_ = titleUpdateTime;
                fpsSampleFrameCount_ = 0;
                fpsSampleInitialized_ = true;
            }
            ++fpsSampleFrameCount_;
            const double sampleDuration = titleUpdateTime - fpsSampleStartTime_;
            if (sampleDuration >= 1.0) {
                displayedFps_ = static_cast<double>(fpsSampleFrameCount_) / sampleDuration;
                fpsSampleFrameCount_ = 0;
                fpsSampleStartTime_ = titleUpdateTime;
            }
        }
        if (!renderingReflection_ && playerVehicle_ &&
            titleUpdateTime - lastStatsTitleTime_ > 0.25) {
            const double speedMetresPerSecond = simulation.speed();
            const long speedMph = std::lround(speedMetresPerSecond * 2.2369362921);
            const long speedKmh = std::lround(speedMetresPerSecond * 3.6);
            const long throttlePercent = std::lround(
                std::clamp(playerVehicle_->variables.get("throttle"), 0.0F, 1.0F) * 100.0F);
            const long brakePercent = std::lround(
                std::clamp(playerVehicle_->variables.get("brake"), 0.0F, 1.0F) * 100.0F);
            std::ostringstream title;
            title << "OpenBus - " << speedMph << " mph / " << speedKmh << " km/h - Throttle "
                  << throttlePercent << "% - Brake " << brakePercent << "% - "
                  << openbus::rendering::frameTriangleCount() << " main + "
                  << openbus::rendering::frameReflectionTriangleCount()
                  << " reflection triangles/refresh - " << std::fixed << std::setprecision(1)
                  << displayedFps_ << " FPS";
            {
                TraceScope phase("render", "RenderLoop::draw.windowTitle.set");
                glfwSetWindowTitle(window_, title.str().c_str());
            }
            lastStatsTitleTime_ = titleUpdateTime;
        }
    }
}

void RenderLoop::endFrame() {
    TraceScope trace("frame", "RenderLoop::endFrame.swapBuffers");
    glfwSwapBuffers(window_);
}

void RenderLoop::captureViews(const BusSimulation& simulation,
                              const std::filesystem::path& directory) {
    TraceScope trace("capture", "RenderLoop::captureViews");
    std::filesystem::create_directories(directory);
    int width = 1;
    int height = 1;
    glfwGetFramebufferSize(window_, &width, &height);
    framebufferWidth_ = std::max(width, 1);
    framebufferHeight_ = std::max(height, 1);

    const int previousCameraView = cameraView_;
    const double previousCameraYaw = cameraYaw_;
    const double previousCameraPitch = cameraPitch_;
    const double previousCameraDistance = cameraDistance_;
    const double previousViewLookYaw = viewLookYaw_;
    const double previousViewLookPitch = viewLookPitch_;
    captureMode_ = true;
    cameraView_ = 0;
    cameraDistance_ = 24.0;
    viewLookYaw_ = 0.0;
    viewLookPitch_ = 0.0;

    struct CaptureView {
        const char* name;
        int cameraView;
        double yaw;
        double pitch;
        double distance;
    };
    const int driverCameraView =
        reflectionReferenceCamera_ >= 0 ? reflectionReferenceCamera_ + 1 : 1;
    const std::array<CaptureView, 5> views = {{{"dashboard", driverCameraView, 0.0, 0.0, 0.0},
                                               {"three-quarter", 0, 0.55, 0.45, 11.0},
                                               {"front", 0, 0.0, 0.45, 10.5},
                                               {"left", 0, 1.5707963267948966, 0.45, 10.5},
                                               {"right", 0, -1.5707963267948966, 0.45, 10.5}}};
    for (const CaptureView& view : views) {
        TraceScope viewTrace("capture", "RenderLoop::captureViews.view");
        cameraView_ = view.cameraView;
        cameraYaw_ = view.yaw;
        cameraPitch_ = view.pitch;
        cameraDistance_ = view.distance;
        {
            TraceScope phase("capture", "RenderLoop::captureViews.render");
            viewport_ = {0, 0, width, height};
            glViewport(viewport_[0], viewport_[1], viewport_[2], viewport_[3]);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            setPerspective(static_cast<double>(width), static_cast<double>(height), 60.0);
            draw(simulation);
            glFinish();
        }
        const std::filesystem::path path = directory / (std::string(view.name) + ".png");
        {
            TraceScope phase("capture", "RenderLoop::captureViews.save");
            if (!openbus::rendering::saveFramebufferPng(path, width, height)) {
                gameLog.Log("Failed to save screenshot: " + path.string());
            } else {
                gameLog.Log("Saved screenshot: " + path.string());
            }
        }
    }

    cameraView_ = previousCameraView;
    cameraYaw_ = previousCameraYaw;
    cameraPitch_ = previousCameraPitch;
    cameraDistance_ = previousCameraDistance;
    viewLookYaw_ = previousViewLookYaw;
    viewLookPitch_ = previousViewLookPitch;
    captureMode_ = false;
}
