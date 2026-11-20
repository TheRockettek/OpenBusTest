#include "app/RenderLoopInternal.h"

#include "app/DiligentFrameRenderer.h"
#include "map/MapTrafficRuntime.h"
#include "core/PerfTrace.h"
#include "render/MatrixMath.h"
#include "render/RendererFramePresenter.h"

#include <imgui.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <limits>
#include <numbers>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

namespace {

std::optional<ImVec2> projectDebugOverlayClipPoint(const std::array<double, 4>& clip,
                                                   const ImVec2& displaySize) {
    if (!std::all_of(clip.begin(), clip.end(), [](double value) { return std::isfinite(value); }) ||
        clip[3] <= 1.0e-6 || clip[0] < -clip[3] || clip[0] > clip[3] || clip[1] < -clip[3] ||
        clip[1] > clip[3] || clip[2] < -clip[3] || clip[2] > clip[3]) {
        return std::nullopt;
    }

    const double ndcX = clip[0] / clip[3];
    const double ndcY = clip[1] / clip[3];
    if (!std::isfinite(ndcX) || !std::isfinite(ndcY)) {
        return std::nullopt;
    }
    return ImVec2(static_cast<float>((ndcX * 0.5 + 0.5) * displaySize.x),
                  static_cast<float>((0.5 - ndcY * 0.5) * displaySize.y));
}

std::optional<std::pair<ImVec2, ImVec2>>
clipDebugOverlaySegment(const openbus::rendering::Matrix4& viewProjection,
                        const std::array<double, 3>& start, const std::array<double, 3>& end,
                        const ImVec2& displaySize) {
    std::array<double, 4> first =
        openbus::rendering::transformPoint(viewProjection, {start[0], start[1], start[2], 1.0});
    std::array<double, 4> second =
        openbus::rendering::transformPoint(viewProjection, {end[0], end[1], end[2], 1.0});
    if (!std::all_of(first.begin(), first.end(),
                     [](double value) { return std::isfinite(value); }) ||
        !std::all_of(second.begin(), second.end(),
                     [](double value) { return std::isfinite(value); })) {
        return std::nullopt;
    }

    constexpr std::array<std::array<double, 4>, 6> clipPlanes = {{
        {1.0, 0.0, 0.0, 1.0},
        {-1.0, 0.0, 0.0, 1.0},
        {0.0, 1.0, 0.0, 1.0},
        {0.0, -1.0, 0.0, 1.0},
        {0.0, 0.0, 1.0, 1.0},
        {0.0, 0.0, -1.0, 1.0},
    }};
    const auto planeDistance = [](const std::array<double, 4>& plane,
                                  const std::array<double, 4>& point) {
        return plane[0] * point[0] + plane[1] * point[1] + plane[2] * point[2] +
               plane[3] * point[3];
    };
    double firstFraction = 0.0;
    double lastFraction = 1.0;
    for (const auto& plane : clipPlanes) {
        const double firstDistance = planeDistance(plane, first);
        const double secondDistance = planeDistance(plane, second);
        if (firstDistance < 0.0 && secondDistance < 0.0) {
            return std::nullopt;
        }
        if ((firstDistance < 0.0) != (secondDistance < 0.0)) {
            const double fraction = firstDistance / (firstDistance - secondDistance);
            if (firstDistance < 0.0) {
                firstFraction = std::max(firstFraction, fraction);
            } else {
                lastFraction = std::min(lastFraction, fraction);
            }
            if (firstFraction > lastFraction) {
                return std::nullopt;
            }
        }
    }
    const auto interpolate = [&first, &second](double fraction) {
        std::array<double, 4> point = {};
        for (std::size_t component = 0; component < point.size(); ++component) {
            point[component] = first[component] + (second[component] - first[component]) * fraction;
        }
        return point;
    };
    const auto projectedFirst =
        projectDebugOverlayClipPoint(interpolate(firstFraction), displaySize);
    const auto projectedSecond =
        projectDebugOverlayClipPoint(interpolate(lastFraction), displaySize);
    if (!projectedFirst || !projectedSecond) {
        return std::nullopt;
    }
    return std::make_pair(*projectedFirst, *projectedSecond);
}

std::optional<ImVec2> projectDebugOverlayPoint(const openbus::rendering::Matrix4& viewProjection,
                                               const std::array<double, 3>& point,
                                               const ImVec2& displaySize) {
    const std::array<double, 4> clip =
        openbus::rendering::transformPoint(viewProjection, {point[0], point[1], point[2], 1.0});
    return projectDebugOverlayClipPoint(clip, displaySize);
}

void drawDebugOverlayLine(ImDrawList& drawList, const openbus::rendering::Matrix4& viewProjection,
                          const ImVec2& displaySize, const std::array<double, 3>& start,
                          const std::array<double, 3>& end, ImU32 color, float thickness = 1.5F) {
    const auto projected = clipDebugOverlaySegment(viewProjection, start, end, displaySize);
    if (projected) {
        drawList.AddLine(projected->first, projected->second, color, thickness);
    }
}

void appendDebugOverlayLine(std::vector<openbus::rendering::PrimitiveVertex>& vertices,
                            const std::array<double, 3>& start, const std::array<double, 3>& end,
                            const std::array<float, 3>& color) {
    for (const double coordinate : start) {
        if (!std::isfinite(coordinate)) {
            return;
        }
    }
    for (const double coordinate : end) {
        if (!std::isfinite(coordinate)) {
            return;
        }
    }
    vertices.push_back({static_cast<float>(start[0]), static_cast<float>(start[1]),
                        static_cast<float>(start[2]), color[0], color[1], color[2]});
    vertices.push_back({static_cast<float>(end[0]), static_cast<float>(end[1]),
                        static_cast<float>(end[2]), color[0], color[1], color[2]});
}

void drawDebugOverlayArrow(ImDrawList& drawList, const ImVec2& start, const ImVec2& end,
                           ImU32 color);

void drawDebugOverlayVertices(ImDrawList& drawList,
                              const openbus::rendering::Matrix4& viewProjection,
                              const ImVec2& displaySize,
                              const std::vector<openbus::rendering::PrimitiveVertex>& vertices,
                              bool drawDirectionArrows = false) {
    for (std::size_t index = 0; index + 1 < vertices.size(); index += 2) {
        const auto& start = vertices[index];
        const auto& end = vertices[index + 1];
        const std::array<double, 3> startPosition = {start.x, start.y, start.z};
        const std::array<double, 3> endPosition = {end.x, end.y, end.z};
        const auto projected =
            clipDebugOverlaySegment(viewProjection, startPosition, endPosition, displaySize);
        if (!projected) {
            continue;
        }
        const ImU32 color =
            IM_COL32(static_cast<int>(std::clamp(start.r, 0.0F, 1.0F) * 255.0F),
                     static_cast<int>(std::clamp(start.g, 0.0F, 1.0F) * 255.0F),
                     static_cast<int>(std::clamp(start.b, 0.0F, 1.0F) * 255.0F), 235);
        drawList.AddLine(projected->first, projected->second, color, 1.5F);
        if (drawDirectionArrows && (index / 2) % 8 == 0) {
            drawDebugOverlayArrow(drawList, projected->first, projected->second, color);
        }
    }
}

void drawDebugOverlayArrow(ImDrawList& drawList, const ImVec2& start, const ImVec2& end,
                           ImU32 color) {
    const float dx = end.x - start.x;
    const float dy = end.y - start.y;
    const float length = std::sqrt(dx * dx + dy * dy);
    if (length < 1.0F) {
        return;
    }
    const float directionX = dx / length;
    const float directionY = dy / length;
    constexpr float arrowLength = 6.0F;
    constexpr float arrowHalfWidth = 3.0F;
    const ImVec2 base(end.x - directionX * arrowLength, end.y - directionY * arrowLength);
    const ImVec2 perpendicular(-directionY * arrowHalfWidth, directionX * arrowHalfWidth);
    drawList.AddLine(end, ImVec2(base.x + perpendicular.x, base.y + perpendicular.y), color, 1.5F);
    drawList.AddLine(end, ImVec2(base.x - perpendicular.x, base.y - perpendicular.y), color, 1.5F);
}

} // namespace

std::optional<openbus::map::MapTrafficPose>
RenderLoop::passengerQueuePose(int stopId, std::size_t queueSlot) const {
    if (mapRenderer_ == nullptr) {
        return std::nullopt;
    }
    const auto authored = authoredPassengerPositions_.find(stopId);
    if (authored != authoredPassengerPositions_.end() && !authored->second.empty()) {
        return authored->second[queueSlot % authored->second.size()];
    }
    const auto pedestrian = passengerPedestrianQueuePositions_.find(stopId);
    if (pedestrian != passengerPedestrianQueuePositions_.end() && !pedestrian->second.empty()) {
        return pedestrian->second[queueSlot % pedestrian->second.size()];
    }
    return std::nullopt;
}

std::optional<openbus::map::MapTrafficPose>
RenderLoop::passengerQueuePose(int stopId, const openbus::app::WaitingPassenger& passenger) const {
    const auto held = passengerHeldQueuePositions_.find({stopId, passenger.modelIndex});
    if (held != passengerHeldQueuePositions_.end()) {
        return held->second;
    }
    return passengerQueuePose(stopId, passenger.queueSlot);
}

void RenderLoop::draw(const BusSimulation& simulation, const World& world) {
    openbus::rendering::TraceScope trace("render", "RenderLoop::draw");
    if (framePresenter_ == nullptr || dynamic_cast<openbus::rendering::DiligentFrameRenderer*>(
                                          framePresenter_.get()) == nullptr) {
        throw std::logic_error("The production scene requires the Diligent frame presenter");
    }
    openbus::rendering::TraceScope assetTrace("render", "RenderLoop::prepareDiligentSceneAssets");
    prepareDiligentSceneAssets(simulation);
    assetTrace.finish();
    openbus::rendering::TraceScope sceneTrace("render", "RenderLoop::drawDiligentScene");
    drawDiligentScene(simulation, world);
}

void RenderLoop::drawDiligentDebugOverlays(const World& world,
                                           const openbus::rendering::Matrix4& viewProjection) {
    if (!imguiEnabled_ ||
        (!collisionDebugOverlay_ && !tileBoundaryDebugOverlay_ && !trafficSplineDebugOverlay_)) {
        return;
    }

    ImDrawList& drawList = *ImGui::GetForegroundDrawList();
    const ImVec2 displaySize = ImGui::GetIO().DisplaySize;
    const ImVec2 overlayLabelPosition(12.0F, 36.0F);
    float overlayLabelOffset = 0.0F;
    const auto addOverlayLabel = [&](const char* label, ImU32 color) {
        drawList.AddText(
            ImVec2(overlayLabelPosition.x, overlayLabelPosition.y + overlayLabelOffset), color,
            label);
        overlayLabelOffset += ImGui::GetTextLineHeightWithSpacing();
    };
    if (mapRenderer_ != nullptr && debugOverlayMapRenderer_ != mapRenderer_.get()) {
        tileBoundaryDebugBuilt_ = false;
        trafficSplineDebugBuilt_ = false;
        debugOverlayMapRenderer_ = mapRenderer_.get();
    }
    const auto drawCachedLines = [&](const std::vector<openbus::rendering::PrimitiveVertex>& lines,
                                     bool drawDirectionArrows = false) {
        drawDebugOverlayVertices(drawList, viewProjection, displaySize, lines, drawDirectionArrows);
    };

    if (collisionDebugOverlay_) {
        constexpr std::array<float, 3> collisionColor = {0.20F, 0.86F, 1.0F};
        constexpr std::size_t maximumCollisionSegments = 50000;
        const std::uint64_t collisionRevision = world.collisionDebugRevision();
        if (!collisionWireframeBuilt_ || collisionWireframeWorld_ != &world ||
            collisionWireframeRevision_ != collisionRevision) {
            collisionWireframeVertices_.clear();
            for (const StaticCollisionMesh& mesh : world.collisionDebugMeshes()) {
                if (mesh.vertices.size() % 3 != 0) {
                    continue;
                }
                const auto vertex = [&mesh](int index) -> std::optional<std::array<double, 3>> {
                    if (index < 0 || static_cast<std::size_t>(index) >= mesh.vertices.size() / 3) {
                        return std::nullopt;
                    }
                    const std::size_t offset = static_cast<std::size_t>(index) * 3;
                    return std::array<double, 3>{mesh.vertices[offset], mesh.vertices[offset + 1],
                                                 mesh.vertices[offset + 2]};
                };
                for (std::size_t index = 0; index + 2 < mesh.indices.size(); index += 3) {
                    const auto first = vertex(mesh.indices[index]);
                    const auto second = vertex(mesh.indices[index + 1]);
                    const auto third = vertex(mesh.indices[index + 2]);
                    if (!first || !second || !third) {
                        continue;
                    }
                    appendDebugOverlayLine(collisionWireframeVertices_, *first, *second,
                                           collisionColor);
                    appendDebugOverlayLine(collisionWireframeVertices_, *second, *third,
                                           collisionColor);
                    appendDebugOverlayLine(collisionWireframeVertices_, *third, *first,
                                           collisionColor);
                    if (collisionWireframeVertices_.size() / 2 >= maximumCollisionSegments) {
                        break;
                    }
                }
                if (collisionWireframeVertices_.size() / 2 >= maximumCollisionSegments) {
                    break;
                }
            }
            collisionWireframeBuilt_ = true;
            collisionWireframeRevision_ = collisionRevision;
            collisionWireframeWorld_ = &world;
        }
        drawCachedLines(collisionWireframeVertices_);
        const ImU32 collisionLabelColor = IM_COL32(50, 220, 255, 220);

        for (const MapPoleState& pole : world.mapPoleStates()) {
            const ImU32 color = pole.broken        ? IM_COL32(150, 150, 150, 230)
                                : pole.touchingBus ? IM_COL32(255, 70, 60, 240)
                                                   : collisionLabelColor;
            drawDebugOverlayLine(drawList, viewProjection, displaySize, pole.basePosition,
                                 pole.topPosition, color, 2.0F);
            if (!std::isfinite(pole.radiusMeters) || pole.radiusMeters <= 0.0) {
                continue;
            }
            constexpr std::size_t ringSegments = 12;
            for (std::size_t segment = 0; segment < ringSegments; ++segment) {
                const double angle0 =
                    2.0 * std::numbers::pi * static_cast<double>(segment) / ringSegments;
                const double angle1 =
                    2.0 * std::numbers::pi * static_cast<double>(segment + 1) / ringSegments;
                for (const auto& center : {pole.basePosition, pole.topPosition}) {
                    const std::array<double, 3> start = {
                        center[0] + std::cos(angle0) * pole.radiusMeters,
                        center[1] + std::sin(angle0) * pole.radiusMeters, center[2]};
                    const std::array<double, 3> end = {
                        center[0] + std::cos(angle1) * pole.radiusMeters,
                        center[1] + std::sin(angle1) * pole.radiusMeters, center[2]};
                    drawDebugOverlayLine(drawList, viewProjection, displaySize, start, end, color);
                }
            }
        }
        addOverlayLabel("Collision wireframe (C)", collisionLabelColor);
        if (collisionWireframeVertices_.size() / 2 >= maximumCollisionSegments) {
            addOverlayLabel("Collision overlay truncated at 50,000 segments", collisionLabelColor);
        }
    }

    if (tileBoundaryDebugOverlay_ && mapRenderer_ != nullptr) {
        constexpr ImU32 tileColor = IM_COL32(70, 255, 100, 235);
        const auto& activeTiles = mapRenderer_->activeTileCoordinates();
        const std::uint64_t mapRevision = mapRenderer_->trafficDebugGeometryRevision();
        if (!tileBoundaryDebugBuilt_ || tileBoundaryDebugRevision_ != mapRevision ||
            tileBoundaryDebugTiles_ != activeTiles) {
            tileBoundaryDebugVertices_.clear();
            const auto vertices = mapRenderer_->activeTileBoundaryVertices(0.0);
            constexpr std::array<float, 3> tileRgb = {0.27F, 1.0F, 0.39F};
            for (std::size_t index = 0; index + 1 < vertices.size(); index += 2) {
                appendDebugOverlayLine(tileBoundaryDebugVertices_, vertices[index],
                                       vertices[index + 1], tileRgb);
            }
            tileBoundaryDebugTiles_ = activeTiles;
            tileBoundaryDebugRevision_ = mapRevision;
            tileBoundaryDebugBuilt_ = true;
        }
        drawCachedLines(tileBoundaryDebugVertices_);
        addOverlayLabel("Active map tile boundaries (F11)", tileColor);
    }

    if (trafficSplineDebugOverlay_ && mapRenderer_ != nullptr) {
        constexpr ImU32 humanColor = IM_COL32(255, 80, 220, 235);
        constexpr std::array<float, 3> humanRgb = {1.0F, 0.31F, 0.86F};
        constexpr std::size_t maximumTrafficSegments = 20000;
        constexpr std::size_t maximumTrafficLabels = 256;
        const auto& activeTiles = mapRenderer_->activeTileCoordinates();
        const std::uint64_t mapRevision = mapRenderer_->trafficDebugGeometryRevision();
        if (!trafficSplineDebugBuilt_ || trafficSplineDebugMapRevision_ != mapRevision ||
            trafficSplineDebugTiles_ != activeTiles) {
            trafficSplineDebugVertices_.clear();
            trafficSplineDebugLabels_.clear();
            const std::set<std::pair<int, int>> activeTileSet(activeTiles.begin(),
                                                              activeTiles.end());
            std::set<std::tuple<int, int, std::size_t, int, int, double>> visitedSplines;
            std::size_t segmentCount = 0;
            for (const int pathType : {openbus::map::MAP_SPLINE_PATH_TYPE_ROAD,
                                       openbus::map::MAP_SPLINE_PATH_TYPE_HUMAN}) {
                const auto splineIds = mapRenderer_->trafficSplineIdsInTiles(pathType, activeTiles);
                for (const int splineId : splineIds) {
                    const auto route = mapRenderer_->buildTrafficRoute(splineId, pathType);
                    if (!route) {
                        continue;
                    }
                    double segmentStart = 0.0;
                    for (std::size_t segmentIndex = 0;
                         segmentIndex < route->segments.size() &&
                         segmentIndex < route->cumulativeEndDistances.size();
                         ++segmentIndex) {
                        const auto& segment = route->segments[segmentIndex];
                        const double segmentEnd = route->cumulativeEndDistances[segmentIndex];
                        const double segmentLength = segmentEnd - segmentStart;
                        const double currentStart = segmentStart;
                        segmentStart = segmentEnd;
                        if (segmentLength <= 0.0 ||
                            !activeTileSet.contains({segment.spline.tileX, segment.spline.tileY})) {
                            continue;
                        }
                        const std::size_t pathIndex = segment.sceneryPathIndex.value_or(
                            std::numeric_limits<std::size_t>::max());
                        const auto splineKey = std::make_tuple(
                            pathType, segment.spline.placement.splineId, pathIndex,
                            segment.spline.tileX, segment.spline.tileY,
                            segment.splineDistanceAtRouteStartMeters.value_or(currentStart));
                        if (!visitedSplines.insert(splineKey).second) {
                            continue;
                        }

                        const std::array<float, 3> color =
                            pathType == openbus::map::MAP_SPLINE_PATH_TYPE_HUMAN
                                ? humanRgb
                                : std::array<float, 3>{
                                      static_cast<float>(
                                          std::clamp(segment.pathRules.priority / 255.0, 0.0, 1.0)),
                                      static_cast<float>(std::clamp(
                                          1.0 - segment.pathRules.priority / 255.0, 0.0, 1.0)),
                                      0.16F};
                        const std::size_t sampleCount = static_cast<std::size_t>(
                            std::clamp(std::ceil(segmentLength / 5.0), 1.0, 256.0));
                        std::optional<openbus::map::MapTrafficPose> previousPose;
                        std::optional<std::array<double, 3>> labelPosition;
                        for (std::size_t sample = 0; sample <= sampleCount; ++sample) {
                            const double fraction =
                                static_cast<double>(sample) / static_cast<double>(sampleCount);
                            const auto pose = openbus::map::sampleMapTrafficRoute(
                                *route, currentStart + fraction * segmentLength, 0.0);
                            if (!pose) {
                                previousPose.reset();
                                continue;
                            }
                            if (previousPose) {
                                appendDebugOverlayLine(trafficSplineDebugVertices_,
                                                       previousPose->position, pose->position,
                                                       color);
                                ++segmentCount;
                            }
                            if (sample == sampleCount / 2) {
                                labelPosition = pose->position;
                            }
                            previousPose = pose;
                            if (segmentCount >= maximumTrafficSegments) {
                                break;
                            }
                        }
                        if (labelPosition &&
                            trafficSplineDebugLabels_.size() < maximumTrafficLabels) {
                            trafficSplineDebugLabels_.push_back({segment.spline.placement.splineId,
                                                                 pathType, *labelPosition, false});
                        }
                        if (segmentCount >= maximumTrafficSegments) {
                            break;
                        }
                    }
                    if (segmentCount >= maximumTrafficSegments) {
                        break;
                    }
                }
                if (segmentCount >= maximumTrafficSegments) {
                    break;
                }
            }
            trafficSplineDebugTiles_ = activeTiles;
            trafficSplineDebugMapRevision_ = mapRevision;
            trafficSplineDebugBuilt_ = true;
            ++trafficSplineDebugRevision_;
        }
        drawCachedLines(trafficSplineDebugVertices_, true);
        for (const TrafficSplineDebugLabel& label : trafficSplineDebugLabels_) {
            const auto position =
                projectDebugOverlayPoint(viewProjection, label.position, displaySize);
            if (!position) {
                continue;
            }
            const std::string text =
                (label.pathTypeCode == openbus::map::MAP_SPLINE_PATH_TYPE_HUMAN ? "H " : "R ") +
                std::to_string(label.splineId);
            const ImU32 color = label.pathTypeCode == openbus::map::MAP_SPLINE_PATH_TYPE_HUMAN
                                    ? humanColor
                                    : IM_COL32(90, 255, 90, 235);
            drawList.AddText(*position, color, text.c_str());
        }
        addOverlayLabel("AI / human traffic splines (F2)", humanColor);
        if (trafficSplineDebugVertices_.size() / 2 >= maximumTrafficSegments) {
            addOverlayLabel("Traffic overlay truncated at 20,000 segments", humanColor);
        }
    }
}

void RenderLoop::endFrame() {
    if (framePresenter_ == nullptr || dynamic_cast<openbus::rendering::DiligentFrameRenderer*>(
                                          framePresenter_.get()) == nullptr) {
        throw std::logic_error("Frame pacing requires the Diligent frame presenter");
    }
    if (const auto deadline = frameTiming_.deadlineAfterFrame(std::chrono::steady_clock::now())) {
        std::this_thread::sleep_until(*deadline);
    }
}

void RenderLoop::captureViews(const BusSimulation& simulation, const World& world,
                              const std::filesystem::path& directory) {
    if (framePresenter_ == nullptr) {
        throw std::logic_error("Screenshot capture requires an initialized renderer");
    }
    auto* diligentRenderer =
        dynamic_cast<openbus::rendering::DiligentFrameRenderer*>(framePresenter_.get());
    if (diligentRenderer == nullptr) {
        throw std::logic_error("Screenshot capture requires the Diligent frame presenter");
    }

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
    const bool previousCaptureMode = captureMode_;
    const auto restoreCaptureState = [&] {
        cameraView_ = previousCameraView;
        cameraYaw_ = previousCameraYaw;
        cameraPitch_ = previousCameraPitch;
        cameraDistance_ = previousCameraDistance;
        viewLookYaw_ = previousViewLookYaw;
        viewLookPitch_ = previousViewLookPitch;
        captureMode_ = previousCaptureMode;
    };

    struct CaptureView {
        const char* name;
        int cameraView;
        double yaw;
        double pitch;
        double distance;
    };
    const int driverCameraView =
        reflectionReferenceCamera_ >= 0 ? reflectionReferenceCamera_ + 1 : 1;
    std::vector<CaptureView> views = {{"dashboard", driverCameraView, 0.0, 0.0, 0.0},
                                      {"three-quarter", 0, 0.55, 0.45, 11.0},
                                      {"front", 0, 0.0, 0.45, 10.5},
                                      {"left", 0, 1.5707963267948966, 0.45, 10.5},
                                      {"right", 0, -1.5707963267948966, 0.45, 10.5}};
    if (const auto panelZoomCamera = vehicleCameraArrayIndexForDriverView(vehicleCameras_, 7)) {
        views.push_back({"driver-view-7", static_cast<int>(*panelZoomCamera) + 1, 0.0, 0.0, 0.0});
    }

    captureMode_ = true;
    try {
        for (const CaptureView& view : views) {
            cameraView_ = view.cameraView;
            cameraYaw_ = view.yaw;
            cameraPitch_ = view.pitch;
            cameraDistance_ = view.distance;
            const std::filesystem::path path = directory / (std::string(view.name) + ".png");
            invalidateTemporalHistory();
            diligentRenderer->captureNextFrame(path);
            draw(simulation, world);
            if (diligentRenderer->hasPendingCapture()) {
                throw std::runtime_error(
                    "Diligent screenshot frame was suspended before composition");
            }
            gameLog.Log("Saved screenshot: " + path.string());
        }
    } catch (...) {
        diligentRenderer->cancelPendingCapture();
        restoreCaptureState();
        throw;
    }
    restoreCaptureState();
}
