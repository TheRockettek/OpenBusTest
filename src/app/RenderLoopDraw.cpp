// Frame rendering: reflections, scene draw, HUD overlay and capture.

#include "app/RenderLoopInternal.h"
#include "app/ImGuiLayer.h"
#include "app/TrafficDebugDirection.h"

#include <imgui.h>
#include "app/vehicle/VehiclePartLoading.h"
#include "map/MapTileWindow.h"
#include "map/MapSplineProfile.h"
#include "render/WeatherEffects.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <map>
#include <numbers>
#include <sstream>
#include <thread>
#include <utility>

namespace {

constexpr double waitingPassengerIdleGaitWeight = 0.08;
constexpr double waitingPassengerIdleCycleRadiansPerSecond = 0.7;

struct ActiveSceneryTrafficSegment {
    std::size_t index;
    const openbus::rendering::TrafficPathDebugSegment* segment;
};

struct ActiveTrafficSplineRoute {
    int splineId;
    int pathTypeCode;
    openbus::map::MapTrafficRouteSegment firstSegment;
};

std::optional<ImVec2> projectTrafficDebugLabel(const openbus::rendering::Matrix4& viewProjection,
                                               const std::array<double, 3>& position,
                                               const ImVec2& displaySize) {
    const double x = position[0];
    const double y = position[1];
    const double z = position[2];
    const double clipX =
        viewProjection[0] * x + viewProjection[4] * y + viewProjection[8] * z + viewProjection[12];
    const double clipY =
        viewProjection[1] * x + viewProjection[5] * y + viewProjection[9] * z + viewProjection[13];
    const double clipZ =
        viewProjection[2] * x + viewProjection[6] * y + viewProjection[10] * z + viewProjection[14];
    const double clipW =
        viewProjection[3] * x + viewProjection[7] * y + viewProjection[11] * z + viewProjection[15];
    if (!std::isfinite(clipW) || clipW <= 1.0e-6) {
        return std::nullopt;
    }
    const double normalizedX = clipX / clipW;
    const double normalizedY = clipY / clipW;
    const double normalizedZ = clipZ / clipW;
    if (!std::isfinite(normalizedX) || !std::isfinite(normalizedY) || !std::isfinite(normalizedZ) ||
        normalizedX < -1.0 || normalizedX > 1.0 || normalizedY < -1.0 || normalizedY > 1.0 ||
        normalizedZ < -1.0 || normalizedZ > 1.0) {
        return std::nullopt;
    }
    return ImVec2(static_cast<float>((normalizedX * 0.5 + 0.5) * displaySize.x),
                  static_cast<float>((-normalizedY * 0.5 + 0.5) * displaySize.y));
}

void appendPoleCollisionOutline(std::vector<openbus::rendering::PrimitiveVertex>& vertices,
                                const MapPoleState& pole) {
    const std::array<double, 3> axis = {pole.topPosition[0] - pole.basePosition[0],
                                        pole.topPosition[1] - pole.basePosition[1],
                                        pole.topPosition[2] - pole.basePosition[2]};
    const double length = std::hypot(axis[0], axis[1], axis[2]);
    if (!std::isfinite(length) || length <= 1.0e-6 || !std::isfinite(pole.radiusMeters) ||
        pole.radiusMeters <= 0.0) {
        return;
    }
    const std::array<double, 3> up = {axis[0] / length, axis[1] / length, axis[2] / length};
    const std::array<double, 3> reference = std::abs(up[2]) < 0.9
                                                ? std::array<double, 3>{0.0, 0.0, 1.0}
                                                : std::array<double, 3>{1.0, 0.0, 0.0};
    std::array<double, 3> across = {up[1] * reference[2] - up[2] * reference[1],
                                    up[2] * reference[0] - up[0] * reference[2],
                                    up[0] * reference[1] - up[1] * reference[0]};
    const double acrossLength = std::hypot(across[0], across[1], across[2]);
    across = {across[0] / acrossLength, across[1] / acrossLength, across[2] / acrossLength};
    const std::array<double, 3> around = {up[1] * across[2] - up[2] * across[1],
                                          up[2] * across[0] - up[0] * across[2],
                                          up[0] * across[1] - up[1] * across[0]};
    const std::array<float, 3> color = pole.broken        ? std::array<float, 3>{0.5F, 0.5F, 0.5F}
                                       : pole.touchingBus ? std::array<float, 3>{1.0F, 0.12F, 0.08F}
                                                          : std::array<float, 3>{0.1F, 0.9F, 1.0F};
    constexpr std::size_t AXIAL_SEGMENTS = 16;
    constexpr std::size_t CIRCUMFERENTIAL_SEGMENTS = 16;
    const auto pointAt = [&](std::size_t axialStep, std::size_t aroundStep) {
        const double along =
            length * static_cast<double>(axialStep) / static_cast<double>(AXIAL_SEGMENTS);
        const double radius = std::min(pole.radiusMeters, length * 0.5);
        double radialDistance = radius;
        if (along < radius) {
            radialDistance =
                std::sqrt(std::max(0.0, radius * radius - (radius - along) * (radius - along)));
        } else if (along > length - radius) {
            const double fromTop = length - along;
            radialDistance =
                std::sqrt(std::max(0.0, radius * radius - (radius - fromTop) * (radius - fromTop)));
        }
        const double angle = 2.0 * std::numbers::pi * static_cast<double>(aroundStep) /
                             static_cast<double>(CIRCUMFERENTIAL_SEGMENTS);
        const double radialAcross = radialDistance * std::cos(angle);
        const double radialAround = radialDistance * std::sin(angle);
        return std::array<double, 3>{pole.basePosition[0] + up[0] * along +
                                         across[0] * radialAcross + around[0] * radialAround,
                                     pole.basePosition[1] + up[1] * along +
                                         across[1] * radialAcross + around[1] * radialAround,
                                     pole.basePosition[2] + up[2] * along +
                                         across[2] * radialAcross + around[2] * radialAround};
    };
    const auto appendLine = [&](const std::array<double, 3>& start,
                                const std::array<double, 3>& end) {
        vertices.push_back({static_cast<float>(start[0]), static_cast<float>(start[1]),
                            static_cast<float>(start[2]), color[0], color[1], color[2]});
        vertices.push_back({static_cast<float>(end[0]), static_cast<float>(end[1]),
                            static_cast<float>(end[2]), color[0], color[1], color[2]});
    };
    for (std::size_t axialStep = 0; axialStep <= AXIAL_SEGMENTS; ++axialStep) {
        for (std::size_t aroundStep = 0; aroundStep < CIRCUMFERENTIAL_SEGMENTS; ++aroundStep) {
            appendLine(pointAt(axialStep, aroundStep),
                       pointAt(axialStep, (aroundStep + 1) % CIRCUMFERENTIAL_SEGMENTS));
            if (axialStep < AXIAL_SEGMENTS) {
                appendLine(pointAt(axialStep, aroundStep), pointAt(axialStep + 1, aroundStep));
            }
        }
    }
}

std::vector<ActiveTrafficSplineRoute>
activeTrafficSplineRoutes(const openbus::rendering::MapRenderer& mapRenderer,
                          const std::vector<std::pair<int, int>>& activeTiles) {
    std::vector<ActiveTrafficSplineRoute> activeRoutes;
    for (const int pathTypeCode :
         {openbus::map::MAP_SPLINE_PATH_TYPE_ROAD, openbus::map::MAP_SPLINE_PATH_TYPE_HUMAN}) {
        for (const int splineId : mapRenderer.trafficSplineIdsInTiles(pathTypeCode, activeTiles)) {
            std::optional<openbus::map::MapTrafficRoute> route =
                mapRenderer.buildTrafficRoute(splineId, pathTypeCode);
            if (!route.has_value() || route->segments.empty()) {
                continue;
            }
            openbus::map::MapTrafficRouteSegment& firstSegment = route->segments.front();
            activeRoutes.push_back({splineId, pathTypeCode, std::move(firstSegment)});
        }
    }
    return activeRoutes;
}

class SceneryTrafficEndpointIndex {
  public:
    SceneryTrafficEndpointIndex(const openbus::rendering::MapRenderer& mapRenderer,
                                const std::vector<ActiveSceneryTrafficSegment>& activeSegments,
                                const std::vector<ActiveTrafficSplineRoute>& activeRoutes) {
        for (const ActiveSceneryTrafficSegment& activeSegment : activeSegments) {
            const openbus::rendering::TrafficPathDebugSegment& segment = *activeSegment.segment;
            if (segment.points.empty()) {
                continue;
            }
            addEndpoint(segment.points.front(), segment.pathTypeCode);
            addEndpoint(segment.points.back(), segment.pathTypeCode);
        }
        for (const ActiveTrafficSplineRoute& activeRoute : activeRoutes) {
            const int pathTypeCode = activeRoute.pathTypeCode;
            const int splineId = activeRoute.splineId;
            const openbus::map::MapSplinePlacementReference& splineReference =
                activeRoute.firstSegment.spline;
            const double splineLength = splineReference.placement.length;
            if (!std::isfinite(splineLength) || splineLength <= 0.0) {
                continue;
            }
            std::vector<openbus::map::MapSplinePathOffset> pathOffsets =
                mapRenderer.trafficSplinePathOffsets(splineId, pathTypeCode);
            if (pathOffsets.empty()) {
                pathOffsets.push_back({});
            }
            for (const openbus::map::MapSplinePathOffset& pathOffset : pathOffsets) {
                const auto singleSplineRoute =
                    openbus::app::trafficDebugSplineRoute(activeRoute.firstSegment, pathOffset);
                for (const double distance : {0.0, splineLength}) {
                    const std::optional<openbus::map::MapTrafficPose> pose =
                        openbus::map::sampleMapTrafficRoute(singleSplineRoute, distance, 0.0);
                    if (!pose.has_value()) {
                        continue;
                    }
                    addEndpoint(pose->position, static_cast<double>(pathTypeCode), splineId);
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

openbus::rendering::PrimitiveVertex trafficDebugVertex(const std::array<double, 3>& position,
                                                       const std::array<float, 3>& color,
                                                       const std::array<double, 2>& origin) {
    return {static_cast<float>(position[0] - origin[0]),
            static_cast<float>(position[1] - origin[1]),
            static_cast<float>(position[2]),
            color[0],
            color[1],
            color[2]};
}

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
                              const std::array<float, 3>& color,
                              const std::array<double, 2>& origin) {
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
            vertices.push_back(trafficDebugVertex(point, color, origin));
        }
    }
}

void appendTrafficBehaviorLine(std::vector<openbus::rendering::PrimitiveVertex>& vertices,
                               const std::array<double, 3>& from, const std::array<double, 3>& to,
                               const std::array<float, 3>& color,
                               const std::array<double, 2>& origin) {
    for (const auto& point : {from, to}) {
        vertices.push_back(trafficDebugVertex(point, color, origin));
    }
}

void appendTrafficDirectionArrow(std::vector<openbus::rendering::PrimitiveVertex>& vertices,
                                 const std::array<double, 3>& center, double headingRadians,
                                 double length, const std::array<float, 3>& color,
                                 const std::array<double, 2>& origin) {
    const auto points = openbus::app::trafficDirectionArrow(center, headingRadians, length);
    if (points) {
        for (const auto& point : *points) {
            vertices.push_back(trafficDebugVertex(point, color, origin));
        }
    } else {
        gameLog.Warning("Cannot draw traffic direction arrow with invalid geometry");
    }
}

std::vector<std::string>
trafficSignalDebugLabel(const openbus::rendering::TrafficPathDebugSegment& segment,
                        std::size_t fallbackId, double signalClockSeconds) {
    if (!segment.trafficSignal.has_value()) {
        return {};
    }
    const openbus::map::MapTrafficSignalSchedule& schedule = *segment.trafficSignal;
    const std::optional<int> currentState =
        openbus::map::mapTrafficSignalStateAtTime(schedule, signalClockSeconds);
    std::ostringstream heading;
    heading << 'J' << segment.mapObjectId.value_or(static_cast<int>(fallbackId)) << " P"
            << segment.pathIndex << " L" << schedule.lightIndex << " PH";
    if (currentState.has_value()) {
        heading << *currentState;
    } else {
        heading << '-';
    }
    if (schedule.controller != nullptr) {
        heading << " T" << std::fixed << std::setprecision(2)
                << schedule.controller->clockSeconds();
    } else {
        heading << " T" << std::fixed << std::setprecision(2) << signalClockSeconds;
    }
    std::vector<std::string> lines = {heading.str()};

    const std::vector<openbus::map::MapTrafficSignalPhase>* phases = &schedule.phases;
    if (schedule.program != nullptr && schedule.lightIndex < schedule.program->lights.size()) {
        phases = &schedule.program->lights[schedule.lightIndex];
    }
    std::ostringstream phaseLine;
    phaseLine << 'C' << std::fixed << std::setprecision(2) << schedule.cycleDurationSeconds << " S";
    for (std::size_t index = 0; index < phases->size(); ++index) {
        if (index > 0) {
            phaseLine << ' ';
        }
        phaseLine << (*phases)[index].state << '/' << std::fixed << std::setprecision(2)
                  << (*phases)[index].durationSeconds;
    }
    lines.push_back(phaseLine.str());

    if ((schedule.hasConditionalOverrides || schedule.hasBusPriority) &&
        schedule.program != nullptr) {
        for (const openbus::map::MapTrafficSignalControlPoint& point :
             schedule.program->controlPoints) {
            if (point.lightIndex != schedule.lightIndex) {
                continue;
            }
            std::ostringstream controlLine;
            controlLine << "CP" << std::fixed << std::setprecision(2) << point.timeSeconds;
            controlLine << 'H';
            if (point.ifRequest) {
                controlLine << 'R';
            }
            lines.push_back(controlLine.str());
        }
        for (const openbus::map::MapTrafficSignalBusPriorityRule& rule :
             schedule.program->busPriorityRules) {
            std::ostringstream priorityLine;
            priorityLine << "BP" << rule.phaseToInterrupt << "D" << std::fixed
                         << std::setprecision(2) << rule.triggerDistanceMeters << "T"
                         << rule.targetPhase << "G" << rule.minimumGreenSeconds;
            lines.push_back(priorityLine.str());
        }
    }
    return lines;
}

std::array<float, 3> trafficPriorityDebugColor(double priority) {
    const double boundedPriority = std::clamp(priority, 0.0, 255.0);
    constexpr std::array<double, 5> stops = {0.0, 64.0, 128.0, 192.0, 255.0};
    constexpr std::array<std::array<float, 3>, 5> colors = {
        std::array<float, 3>{0.55F, 0.12F, 1.0F},
        {0.05F, 0.38F, 1.0F},
        {0.12F, 1.0F, 0.12F},
        {1.0F, 0.82F, 0.05F},
        {1.0F, 0.08F, 0.05F},
    };
    for (std::size_t index = 1; index < stops.size(); ++index) {
        if (boundedPriority <= stops[index]) {
            const double amount =
                (boundedPriority - stops[index - 1]) / (stops[index] - stops[index - 1]);
            return {
                static_cast<float>(colors[index - 1][0] +
                                   amount * (colors[index][0] - colors[index - 1][0])),
                static_cast<float>(colors[index - 1][1] +
                                   amount * (colors[index][1] - colors[index - 1][1])),
                static_cast<float>(colors[index - 1][2] +
                                   amount * (colors[index][2] - colors[index - 1][2])),
            };
        }
    }
    return colors.back();
}

double trafficPriorityDebugValue(const openbus::map::MapTrafficPathRules& rules) {
    const openbus::map::MapTrafficGroupRules* selectedGroup = nullptr;
    for (const openbus::map::MapTrafficGroupRules& group : rules.groups) {
        if (!group.priority.has_value() || *group.priority == 128.0) {
            continue;
        }
        if (selectedGroup == nullptr || group.groupIndex < selectedGroup->groupIndex) {
            selectedGroup = &group;
        }
    }
    return selectedGroup != nullptr && selectedGroup->priority.has_value()
               ? *selectedGroup->priority
               : rules.priority;
}

void appendTrafficSplineDebugVertices(const openbus::rendering::MapRenderer& mapRenderer,
                                      int pathTypeCode,
                                      const std::vector<ActiveTrafficSplineRoute>& activeRoutes,
                                      const std::array<float, 3>& color,
                                      const SceneryTrafficEndpointIndex& sceneryEndpoints,
                                      std::vector<openbus::rendering::PrimitiveVertex>& vertices,
                                      const std::array<double, 2>& origin) {
    for (const ActiveTrafficSplineRoute& activeRoute : activeRoutes) {
        if (activeRoute.pathTypeCode != pathTypeCode) {
            continue;
        }

        const int splineId = activeRoute.splineId;
        const openbus::map::MapTrafficRouteSegment& firstSegment = activeRoute.firstSegment;
        const openbus::map::MapSplinePlacementReference& splineReference = firstSegment.spline;
        const double splineLength = splineReference.placement.length;
        if (!std::isfinite(splineLength) || splineLength <= 0.0) {
            continue;
        }
        const std::optional<openbus::map::MapTrafficSplineEndpointStatus> endpointStatus =
            mapRenderer.trafficSplineEndpointStatus(splineId, pathTypeCode);
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
            const auto singleSplineRoute =
                openbus::app::trafficDebugSplineRoute(firstSegment, pathOffset);
            std::optional<openbus::map::MapTrafficRoute> aiRouteSegments;
            if (pathTypeCode == openbus::map::MAP_SPLINE_PATH_TYPE_ROAD) {
                aiRouteSegments = openbus::map::subdivideMapTrafficRoute(singleSplineRoute);
            }
            const std::array<float, 3> pathColor =
                pathTypeCode == openbus::map::MAP_SPLINE_PATH_TYPE_ROAD
                    ? trafficPriorityDebugColor(
                          trafficPriorityDebugValue(openbus::map::mapTrafficRulesForPath(
                              splineReference.placement.rules, pathOffset.pathIndex)))
                    : color;
            std::optional<openbus::map::MapTrafficPose> previousPose =
                openbus::map::sampleMapTrafficRoute(singleSplineRoute, 0.0, 0.0);
            const std::optional<openbus::map::MapTrafficPose> startPose = previousPose;
            for (std::size_t sampleIndex = 1; sampleIndex <= sampleCount; ++sampleIndex) {
                const double distance = splineLength * static_cast<double>(sampleIndex) /
                                        static_cast<double>(sampleCount);
                const std::optional<openbus::map::MapTrafficPose> pose =
                    openbus::map::sampleMapTrafficRoute(singleSplineRoute, distance, 0.0);
                if (previousPose.has_value() && pose.has_value()) {
                    std::array<double, 3> from = previousPose->position;
                    std::array<double, 3> to = pose->position;
                    from[2] += 0.08;
                    to[2] += 0.08;
                    vertices.push_back(trafficDebugVertex(from, pathColor, origin));
                    vertices.push_back(trafficDebugVertex(to, pathColor, origin));
                }
                previousPose = pose;
            }
            const std::size_t arrowCount = std::max<std::size_t>(
                1, static_cast<std::size_t>(std::min(std::ceil(splineLength / 15.0), 256.0)));
            for (std::size_t index = 0; index < arrowCount; ++index) {
                const double distance =
                    splineLength * (static_cast<double>(index) + 0.5) / arrowCount;
                const auto pose =
                    openbus::map::sampleMapTrafficRoute(singleSplineRoute, distance, 0.0);
                if (!pose) {
                    continue;
                }
                auto center = pose->position;
                center[2] += 0.2;
                const double heading = openbus::map::mapTrafficRouteTravelHeadingRadians(
                    singleSplineRoute, distance, 0.0);
                appendTrafficDirectionArrow(vertices, center, heading, 2.0, pathColor, origin);
                if (pathOffset.bidirectional) {
                    appendTrafficDirectionArrow(vertices, center, heading + std::numbers::pi, 2.0,
                                                pathColor, origin);
                }
            }
            constexpr float endpointMarkerHalfSize = 0.45F;
            constexpr float endpointMarkerZOffset = 0.16F;
            constexpr std::array<float, 3> unconnectedStartMarkerColor = {1.0F, 0.48F, 0.02F};
            constexpr std::array<float, 3> unconnectedEndMarkerColor = {1.0F, 0.1F, 0.1F};
            const auto appendEndpointMarker =
                [&](const std::optional<openbus::map::MapTrafficPose>& pose,
                    double pathHeightOffset, bool isStart, bool splineConnected) {
                    if (!pose.has_value()) {
                        return;
                    }
                    std::array<double, 3> endpointPosition = pose->position;
                    endpointPosition[2] += pathHeightOffset;
                    if (const std::optional<std::array<double, 3>> joinedEndpoint =
                            sceneryEndpoints.find(pathTypeCode, endpointPosition, splineId)) {
                        splineConnected = true;
                        const double horizontalGap =
                            std::hypot(endpointPosition[0] - (*joinedEndpoint)[0],
                                       endpointPosition[1] - (*joinedEndpoint)[1]);
                        const double verticalGap =
                            std::abs(endpointPosition[2] - (*joinedEndpoint)[2]);
                        if (horizontalGap > 0.01 || verticalGap > 0.01) {
                            std::array<double, 3> from = endpointPosition;
                            std::array<double, 3> to = *joinedEndpoint;
                            from[2] += 0.08;
                            to[2] += 0.08;
                            vertices.push_back(trafficDebugVertex(from, pathColor, origin));
                            vertices.push_back(trafficDebugVertex(to, pathColor, origin));
                        }
                    }
                    if (splineConnected) {
                        return;
                    }
                    const float x = static_cast<float>(endpointPosition[0] - origin[0]);
                    const float y = static_cast<float>(endpointPosition[1] - origin[1]);
                    const float z = static_cast<float>(endpointPosition[2] + endpointMarkerZOffset);
                    const auto& markerColor =
                        isStart ? unconnectedStartMarkerColor : unconnectedEndMarkerColor;
                    if (isStart) {
                        vertices.push_back({x - endpointMarkerHalfSize, y - endpointMarkerHalfSize,
                                            z, markerColor[0], markerColor[1], markerColor[2]});
                        vertices.push_back({x + endpointMarkerHalfSize, y - endpointMarkerHalfSize,
                                            z, markerColor[0], markerColor[1], markerColor[2]});
                        vertices.push_back({x + endpointMarkerHalfSize, y - endpointMarkerHalfSize,
                                            z, markerColor[0], markerColor[1], markerColor[2]});
                        vertices.push_back({x + endpointMarkerHalfSize, y + endpointMarkerHalfSize,
                                            z, markerColor[0], markerColor[1], markerColor[2]});
                        vertices.push_back({x + endpointMarkerHalfSize, y + endpointMarkerHalfSize,
                                            z, markerColor[0], markerColor[1], markerColor[2]});
                        vertices.push_back({x - endpointMarkerHalfSize, y + endpointMarkerHalfSize,
                                            z, markerColor[0], markerColor[1], markerColor[2]});
                        vertices.push_back({x - endpointMarkerHalfSize, y + endpointMarkerHalfSize,
                                            z, markerColor[0], markerColor[1], markerColor[2]});
                        vertices.push_back({x - endpointMarkerHalfSize, y - endpointMarkerHalfSize,
                                            z, markerColor[0], markerColor[1], markerColor[2]});
                    } else {
                        vertices.push_back({x - endpointMarkerHalfSize, y - endpointMarkerHalfSize,
                                            z, markerColor[0], markerColor[1], markerColor[2]});
                        vertices.push_back({x + endpointMarkerHalfSize, y + endpointMarkerHalfSize,
                                            z, markerColor[0], markerColor[1], markerColor[2]});
                        vertices.push_back({x - endpointMarkerHalfSize, y + endpointMarkerHalfSize,
                                            z, markerColor[0], markerColor[1], markerColor[2]});
                        vertices.push_back({x + endpointMarkerHalfSize, y - endpointMarkerHalfSize,
                                            z, markerColor[0], markerColor[1], markerColor[2]});
                    }
                };
            const bool startConnected = endpointStatus.has_value() &&
                                        (pathOffset.reversed ? endpointStatus->endConnected
                                                               : endpointStatus->startConnected);
            const bool endConnected = endpointStatus.has_value() &&
                                      (pathOffset.reversed ? endpointStatus->startConnected
                                                             : endpointStatus->endConnected);
            appendEndpointMarker(startPose, 0.0, true, startConnected);
            appendEndpointMarker(previousPose, 0.0, false, endConnected);

            if (aiRouteSegments.has_value()) {
                constexpr float splitMarkerHalfWidth = 0.35F;
                constexpr float splitMarkerZOffset = 0.12F;
                constexpr std::array<float, 3> splitMarkerColor = {1.0F, 0.9F, 0.1F};
                for (std::size_t pieceIndex = 0;
                     pieceIndex + 1 < aiRouteSegments->cumulativeEndDistances.size();
                     ++pieceIndex) {
                    const auto splitPose = openbus::map::sampleMapTrafficRoute(
                        singleSplineRoute, aiRouteSegments->cumulativeEndDistances[pieceIndex],
                        0.0);
                    if (!splitPose.has_value()) {
                        continue;
                    }
                    const double rightX = std::cos(splitPose->headingRadians);
                    const double rightY = -std::sin(splitPose->headingRadians);
                    const double x = splitPose->position[0];
                    const double y = splitPose->position[1];
                    const double z = splitPose->position[2] + splitMarkerZOffset;
                    vertices.push_back(trafficDebugVertex(
                        {x - rightX * splitMarkerHalfWidth, y - rightY * splitMarkerHalfWidth, z},
                        splitMarkerColor, origin));
                    vertices.push_back(trafficDebugVertex(
                        {x + rightX * splitMarkerHalfWidth, y + rightY * splitMarkerHalfWidth, z},
                        splitMarkerColor, origin));
                }
            }
        }
    }
}

void appendSceneryTrafficPathDebugVertices(
    const std::vector<ActiveSceneryTrafficSegment>& activeSegments,
    std::vector<openbus::rendering::PrimitiveVertex>& vertices,
    const std::array<double, 2>& origin) {
    for (const ActiveSceneryTrafficSegment& activeSegment : activeSegments) {
        const openbus::rendering::TrafficPathDebugSegment& segment = *activeSegment.segment;
        const bool roadPath = segment.pathTypeCode == openbus::map::MAP_SPLINE_PATH_TYPE_ROAD;
        const std::array<float, 3> color =
            roadPath ? trafficPriorityDebugColor(trafficPriorityDebugValue(segment.pathRules))
            : segment.pathTypeCode == openbus::map::MAP_SPLINE_PATH_TYPE_HUMAN
                ? std::array<float, 3>{1.0F, 0.25F, 0.9F}
                : std::array<float, 3>{0.15F, 0.9F, 1.0F};
        double sinceArrow = 0.0;
        for (std::size_t index = 1; index < segment.points.size(); ++index) {
            const auto& from = segment.points[index - 1];
            const auto& to = segment.points[index];
            for (const std::array<double, 3>& point :
                 {from, to}) {
                std::array<double, 3> elevatedPoint = point;
                elevatedPoint[2] += 0.08;
                vertices.push_back(trafficDebugVertex(elevatedPoint, color, origin));
            }
            const double dx = to[0] - from[0];
            const double dy = to[1] - from[1];
            sinceArrow += std::hypot(dx, dy);
            if ((sinceArrow >= 15.0 || index == segment.points.size() / 2) &&
                std::hypot(dx, dy) > 1.0e-6) {
                const std::array<double, 3> center = {
                    (from[0] + to[0]) * 0.5, (from[1] + to[1]) * 0.5,
                    (from[2] + to[2]) * 0.5 + 0.2};
                appendTrafficDirectionArrow(vertices, center, std::atan2(dx, dy), 2.0, color,
                                            origin);
                sinceArrow = 0.0;
            }
        }
    }
}

openbus::map::MapTrafficPose passengerCabinSeatPose(const openbus::map::MapTrafficPose& vehiclePose,
                                                    const PassengerCabinPosition& seat) {
    constexpr double degreesToRadians = std::numbers::pi / 180.0;
    const double vehicleYaw =
        openbus::map::mapTrafficHeadingToVehicleYawDegrees(vehiclePose.headingRadians) *
        degreesToRadians;
    const double cosine = std::cos(vehicleYaw);
    const double sine = std::sin(vehicleYaw);
    openbus::map::MapTrafficPose passengerPose;
    passengerPose.position = {
        vehiclePose.position[0] + seat.position[0] * cosine - seat.position[1] * sine,
        vehiclePose.position[1] + seat.position[0] * sine + seat.position[1] * cosine,
        vehiclePose.position[2] + seat.position[2] + seat.seatHeight};
    passengerPose.headingRadians = vehicleYaw + seat.rotationDegrees * degreesToRadians;
    return passengerPose;
}

std::array<double, 3> passengerCabinLocalPosition(const PassengerCabinPosition& seat) {
    return {seat.position[0], seat.position[1], seat.position[2] + seat.seatHeight};
}

std::array<double, 3> passengerCabinLocalPoint(const openbus::map::MapTrafficPose& vehiclePose,
                                               const std::array<double, 3>& worldPosition) {
    constexpr double degreesToRadians = std::numbers::pi / 180.0;
    const double yaw =
        openbus::map::mapTrafficHeadingToVehicleYawDegrees(vehiclePose.headingRadians) *
        degreesToRadians;
    const double cosine = std::cos(yaw);
    const double sine = std::sin(yaw);
    const double offsetX = worldPosition[0] - vehiclePose.position[0];
    const double offsetY = worldPosition[1] - vehiclePose.position[1];
    return {offsetX * cosine + offsetY * sine, -offsetX * sine + offsetY * cosine,
            worldPosition[2] - vehiclePose.position[2]};
}

std::optional<std::size_t> passengerCabinPathPointIndex(const Vehicle& bus, int pointId) {
    const auto point = std::find_if(
        bus.passengerPathPoints.begin(), bus.passengerPathPoints.end(),
        [&](const PassengerPathPoint& candidate) { return candidate.index == pointId; });
    if (point == bus.passengerPathPoints.end()) {
        return std::nullopt;
    }
    return static_cast<std::size_t>(std::distance(bus.passengerPathPoints.begin(), point));
}

std::vector<std::array<double, 3>>
passengerCabinTravelPath(const Vehicle& bus, const openbus::map::MapTrafficPose& vehiclePose,
                         const std::array<double, 3>& startWorld,
                         const std::array<double, 3>& endLocal, bool boarding) {
    if (bus.passengerPathPoints.empty() || bus.passengerPathLinks.empty()) {
        return {};
    }
    constexpr double degreesToRadians = std::numbers::pi / 180.0;
    const double yaw =
        openbus::map::mapTrafficHeadingToVehicleYawDegrees(vehiclePose.headingRadians) *
        degreesToRadians;
    const double cosine = std::cos(yaw);
    const double sine = std::sin(yaw);
    const std::array<double, 3> startLocal = passengerCabinLocalPoint(vehiclePose, startWorld);
    const auto closestPoint =
        [&](const std::array<double, 3>& position,
            const std::vector<int>* candidates) -> std::optional<std::size_t> {
        std::optional<std::size_t> closest;
        double closestDistance = std::numeric_limits<double>::infinity();
        const auto consider = [&](std::size_t index) {
            const auto& candidate = bus.passengerPathPoints[index].position;
            const double dx = candidate[0] - position[0];
            const double dy = candidate[1] - position[1];
            const double dz = candidate[2] - position[2];
            const double distance = dx * dx + dy * dy + dz * dz;
            if (distance < closestDistance) {
                closest = index;
                closestDistance = distance;
            }
        };
        if (candidates != nullptr) {
            for (const int pointId : *candidates) {
                if (const auto index = passengerCabinPathPointIndex(bus, pointId)) {
                    consider(*index);
                }
            }
        } else {
            for (std::size_t index = 0; index < bus.passengerPathPoints.size(); ++index) {
                consider(index);
            }
        }
        return closest;
    };
    const std::vector<int>* startCandidates =
        boarding && !bus.passengerEntryPathPoints.empty() ? &bus.passengerEntryPathPoints : nullptr;
    const std::vector<int>* endCandidates =
        !boarding && !bus.passengerExitPathPoints.empty() ? &bus.passengerExitPathPoints : nullptr;
    auto startNode = closestPoint(startLocal, startCandidates);
    auto endNode = closestPoint(endLocal, endCandidates);
    if (!startNode && startCandidates != nullptr) {
        startNode = closestPoint(startLocal, nullptr);
    }
    if (!endNode && endCandidates != nullptr) {
        endNode = closestPoint(endLocal, nullptr);
    }
    if (!startNode || !endNode) {
        return {};
    }
    const std::vector<int> pointRoute = passengerPathRoute(
        bus.passengerPathPoints, bus.passengerPathLinks, bus.passengerPathPoints[*startNode].index,
        bus.passengerPathPoints[*endNode].index);
    if (pointRoute.empty()) {
        return {};
    }
    std::vector<std::array<double, 3>> localPath;
    localPath.reserve(pointRoute.size() + 2);
    localPath.push_back(startLocal);
    for (const int pointId : pointRoute) {
        const auto pointIndex = passengerCabinPathPointIndex(bus, pointId);
        if (!pointIndex) {
            return {};
        }
        const auto& point = bus.passengerPathPoints[*pointIndex].position;
        if (localPath.back() != point) {
            localPath.push_back(point);
        }
    }
    if (localPath.back() != endLocal) {
        localPath.push_back(endLocal);
    }
    std::vector<std::array<double, 3>> worldPath;
    worldPath.reserve(localPath.size());
    for (const auto& point : localPath) {
        worldPath.push_back({vehiclePose.position[0] + point[0] * cosine - point[1] * sine,
                             vehiclePose.position[1] + point[0] * sine + point[1] * cosine,
                             vehiclePose.position[2] + point[2]});
    }
    return worldPath;
}

openbus::map::MapTrafficPose
samplePassengerTravelPath(const std::vector<std::array<double, 3>>& path, double progress,
                          double startHeading, double endHeading) {
    openbus::map::MapTrafficPose pose;
    const openbus::rendering::HumanPathPose humanPose =
        openbus::rendering::sampleHumanPath(path, progress, startHeading, endHeading);
    pose.position = humanPose.position;
    pose.headingRadians = humanPose.headingRadians;
    return pose;
}

} // namespace

void RenderLoop::renderReflectionViews(const BusSimulation& simulation, const World& world) {
    if (renderingReflection_ || reflectionRenderer_->empty()) {
        return;
    }
    const int previousCameraView = cameraView_;
    const double previousFovOffset = fieldOfViewOffset_;
    const double previousLookYaw = viewLookYaw_;
    const double previousLookPitch = viewLookPitch_;
    bool reflectionTrianglesReset = false;
    bool reflectionTrafficViewsReset = false;
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
        [this, &world, &reflectionTrianglesReset,
         &reflectionTrafficViewsReset](const BusSimulation& reflectionSimulation,
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
            draw(reflectionSimulation, world);
            if (!reflectionTrafficViewsReset) {
                trafficReflectionCameraViews_.clear();
                reflectionTrafficViewsReset = true;
            }
            const auto& view = modelViewMatrix();
            const std::array<double, 3> cameraPosition = {
                -(view[0] * view[12] + view[1] * view[13] + view[2] * view[14]),
                -(view[4] * view[12] + view[5] * view[13] + view[6] * view[14]),
                -(view[8] * view[12] + view[9] * view[13] + view[10] * view[14])};
            trafficReflectionCameraViews_.push_back(
                {cameraPosition, multiplyMatrix4(openbus::rendering::projectionMatrix(), view)});
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

void RenderLoop::draw(const BusSimulation& simulation, const World& world) {
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
                    vehicle->prepareFrameVisibility(context, framebufferWidth_, framebufferHeight_,
                                                    mainViewMinimumObjectViewportFraction_);
                } else {
                    applyVehiclePlacement(
                        makeModelRootPlacement(vehicle->placement, vehicle->modelOffsetZ));
                    vehicle->prepareFrameVisibility(RenderViewContext::NonPlayer, framebufferWidth_,
                                                    framebufferHeight_,
                                                    mainViewMinimumObjectViewportFraction_);
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
                gameLog.Debug("Mouse press picked event=" +
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
                            gameLog.Debug(description.str());
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
        renderReflectionViews(simulation, world);
    }
    openbus::rendering::setLightingViewMatrix(modelViewMatrix());
    if (mapRenderer_ != nullptr) {
        const auto& objects = mapRenderer_->editorPassengerDebugObjects();
        std::map<int, std::size_t> requestedPassengerCounts;
        for (const auto& [stopId, count] : scheduledPassengerInitialQueues_) {
            requestedPassengerCounts.emplace(stopId, count);
        }
        for (const auto& [stopId, count] : scheduledPassengerQueues_) {
            requestedPassengerCounts[stopId] =
                std::max(requestedPassengerCounts[stopId], count);
        }
        if (objects.size() != authoredPassengerObjectCount_ ||
            requestedPassengerCounts != authoredPassengerQueueCounts_) {
            std::vector<openbus::map::MapPassengerStopAnchor> stops;
            std::vector<openbus::map::MapPassengerWaitingArea> areas;
            for (const auto& object : objects) {
                if (object.busStop && object.objectId.has_value()) {
                    stops.push_back({*object.objectId, object.position});
                }
                if (object.passengerCabin && !object.passengerPositions.empty()) {
                    areas.push_back({object.objectId.value_or(0), object.position,
                                     object.passengerPositions, object.bounds});
                }
            }
            authoredPassengerPositions_ = openbus::map::mapTrafficAssignPassengerPositions(
                stops, areas, requestedPassengerCounts);
            authoredPassengerQueueCounts_ = std::move(requestedPassengerCounts);
            authoredPassengerObjectCount_ = objects.size();
        }
    }
    const auto queuePose = [&](int stopId, const openbus::map::MapTrafficPose& stopPose,
                               std::size_t queueSlot) {
        const auto authored = authoredPassengerPositions_.find(stopId);
        if (authored != authoredPassengerPositions_.end() && !authored->second.empty()) {
            return authored->second[queueSlot % authored->second.size()];
        }
        return openbus::map::mapTrafficPassengerQueuePose(
            stopPose, mapRenderer_->leftHandTraffic(), queueSlot);
    };
    struct ScheduledStopRenderCandidate {
        openbus::map::MapTrafficPose pose;
        const openbus::map::MapTrafficPassengerStop* stop = nullptr;
        double cameraDistance = std::numeric_limits<double>::infinity();
        bool activelyServed = false;
    };
    std::map<int, ScheduledStopRenderCandidate> stopCandidates;
    if (mapRenderer_ != nullptr && lastMainCameraPoseValid_ &&
        (!scheduledPassengerHumanModels_.empty() ||
         (!renderingReflection_ && trafficSplineDebugOverlay_))) {
        for (std::size_t routeIndex = 0; routeIndex < passengerStopRoutes_.size(); ++routeIndex) {
            const PassengerStopRouteState& passengerRoute = passengerStopRoutes_[routeIndex];
            for (std::size_t stopIndex = 0; stopIndex < passengerRoute.stops.size(); ++stopIndex) {
                const openbus::map::MapTrafficPassengerStop& stop = passengerRoute.stops[stopIndex];
                const auto queue = scheduledPassengerQueues_.find(stop.stopId);
                if (queue == scheduledPassengerQueues_.end() ||
                    (queue->second == 0 && (renderingReflection_ || !trafficSplineDebugOverlay_))) {
                    continue;
                }
                const auto stopPose = openbus::map::sampleMapTrafficRoute(
                    passengerRoute.actor.route, stop.routeDistanceMeters, 0.0);
                if (!stopPose) {
                    throw std::runtime_error(
                        "Cannot sample a scheduled passenger stop for rendering");
                }
                const double cameraDistance =
                    std::hypot(stopPose->position[0] - lastMainCameraPose_.position[0],
                               stopPose->position[1] - lastMainCameraPose_.position[1]);
                if (cameraDistance > 75.0) {
                    continue;
                }
                const bool activeAiCall = std::any_of(
                    trafficVehicles_.begin(), trafficVehicles_.end(),
                    [&](const TrafficVehicleState& state) {
                        return state.passengerRouteIndex == routeIndex &&
                               state.vehicle != nullptr && state.passengerStopArrived &&
                               state.nextPassengerStopIndex == stopIndex &&
                               state.vehicle->passengerService.currentStopId() == stop.stopId;
                    });
                const bool activePlayerCall = activePlayerPassengerStop_ == stop.stopId &&
                                              activePlayerPassengerStopRouteIndex_ == routeIndex &&
                                              activePlayerPassengerStopIndex_ == stopIndex;
                const bool activelyServed = activeAiCall || activePlayerCall;
                const auto candidate = stopCandidates.find(stop.stopId);
                if (candidate != stopCandidates.end() &&
                    (candidate->second.activelyServed > activelyServed ||
                     (candidate->second.activelyServed == activelyServed &&
                      candidate->second.cameraDistance <= cameraDistance))) {
                    continue;
                }
                stopCandidates[stop.stopId] = {*stopPose, &stop, cameraDistance, activelyServed};
            }
        }
    }
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
            mapRenderer_->draw(false, renderingReflection_
                                          ? reflectionMinimumObjectViewportFraction_
                                          : mainViewMinimumObjectViewportFraction_);
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
            if (!scheduledPassengerHumanModels_.empty() && lastMainCameraPoseValid_) {
                for (const auto& [stopId, candidate] : stopCandidates) {
                    const auto queue = scheduledPassengerQueues_.find(stopId);
                    if (queue == scheduledPassengerQueues_.end() || queue->second == 0) {
                        continue;
                    }
                    const openbus::map::MapTrafficPose& stopPose = candidate.pose;
                    const auto initialQueue = scheduledPassengerInitialQueues_.find(stopId);
                    const std::size_t initialCount =
                        initialQueue == scheduledPassengerInitialQueues_.end()
                            ? queue->second
                            : std::max(initialQueue->second, queue->second);
                    const auto authored = authoredPassengerPositions_.find(stopId);
                    const bool hasAuthored = authored != authoredPassengerPositions_.end() &&
                                             !authored->second.empty();
                    if (!hasAuthored && reportedPassengerFallbackStops_.insert(stopId).second) {
                        gameLog.Warning("No authored passenger waiting area matched stop " +
                                        std::to_string(stopId) +
                                        "; using the generated roadside queue");
                    }
                    for (std::size_t passenger = 0; passenger < queue->second; ++passenger) {
                        const std::size_t queueSlot =
                            openbus::map::mapScheduledPassengerQueueSlot(initialCount, passenger);
                        const auto passengerPose = queuePose(stopId, stopPose, queueSlot);
                        const std::size_t modelIndex =
                            openbus::map::mapScheduledPassengerModelIdentity(stopId, passenger) %
                            scheduledPassengerHumanModels_.size();
                        mapRenderer_->drawHumanTrafficActor(
                            scheduledPassengerHumanModels_[modelIndex], passengerPose,
                            simulation.simulationTime() *
                                    waitingPassengerIdleCycleRadiansPerSecond +
                                static_cast<double>(passenger) * 1.7,
                            waitingPassengerIdleGaitWeight);
                    }
                }
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
        openbus::rendering::IndirectCullingScope indirectScope;
        bool playerDrawn = false;
        const auto modelLoadDeadline =
            std::chrono::steady_clock::now() + std::chrono::milliseconds(4);
        const auto drawVehicles = [&](VehicleRenderPass renderPass, bool playerVehicles) {
            if (renderPass == VehicleRenderPass::Opaque) {
                // Draw picks LODs from eye distance, so loading must use the same viewpoint.
                const std::array<double, 3> eye =
                    openbus::app::vehicle::cameraEyeFromModelView(modelViewMatrix());
                std::vector<std::pair<double, Vehicle*>> loadOrder;
                loadOrder.reserve(vehicles_.size());
                for (const std::unique_ptr<Vehicle>& vehicle : vehicles_) {
                    if (vehicle->mapTrafficRouteFinished ||
                        (vehicle->followsMapTrafficRoute && !vehicle->mapTrafficTileActive) ||
                        (vehicle.get() == playerVehicle_) != playerVehicles) {
                        continue;
                    }
                    const std::array<double, 3>& position = vehicle.get() == playerVehicle_
                                                                ? chassis.position
                                                                : vehicle->placement.position;
                    const double dx = position[0] - eye[0];
                    const double dy = position[1] - eye[1];
                    const double dz = position[2] - eye[2];
                    loadOrder.emplace_back(dx * dx + dy * dy + dz * dz, vehicle.get());
                }
                std::sort(
                    loadOrder.begin(), loadOrder.end(),
                    [](const auto& left, const auto& right) { return left.first < right.first; });
                for (const auto& [distanceSquared, vehicle] : loadOrder) {
                    const RenderViewContext vehicleContext =
                        vehicle == playerVehicle_ ? context : RenderViewContext::NonPlayer;
                    vehicle->ensureLoaded(vehicleContext, std::sqrt(distanceSquared),
                                          modelLoadDeadline);
                }
            }
            for (const std::unique_ptr<Vehicle>& vehicle : vehicles_) {
                if (vehicle->mapTrafficRouteFinished ||
                    (vehicle->followsMapTrafficRoute && !vehicle->mapTrafficTileActive) ||
                    (vehicle.get() == playerVehicle_) != playerVehicles) {
                    continue;
                }
                const RenderViewContext vehicleContext =
                    vehicle.get() == playerVehicle_ ? context : RenderViewContext::NonPlayer;
                VehiclePlacement placedVehicle = vehicle->placement;
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
                vehicle->draw(vehicleContext, renderPass, shadowTransform,
                              renderingReflection_ ? reflectionMinimumObjectViewportFraction_
                                                   : mainViewMinimumObjectViewportFraction_);
                popMatrix();
            }
        };
        const auto drawBusPassengerActors =
            [&](Vehicle& bus, const openbus::map::MapTrafficPose& busPose, std::size_t modelOffset,
                const openbus::map::MapTrafficActor* routeActor, const Vehicle* routeVehicle,
                const openbus::map::MapTrafficPassengerStop* boardingStop) {
                if (mapRenderer_ == nullptr || scheduledPassengerHumanModels_.empty() ||
                    !lastMainCameraPoseValid_) {
                    return;
                }
                const double busCameraDistance =
                    std::hypot(busPose.position[0] - lastMainCameraPose_.position[0],
                               busPose.position[1] - lastMainCameraPose_.position[1]);
                if (busCameraDistance > 75.0) {
                    return;
                }
                const std::size_t seatCount =
                    std::min(bus.passengerPositions.size(), bus.passengerService.seatCount());
                const auto alightingSeat = bus.passengerService.alightingSeatIndex();
                for (std::size_t seatIndex = 0; seatIndex < seatCount; ++seatIndex) {
                    if (bus.passengerService.passengerCountOnSeat(seatIndex) == 0 ||
                        (alightingSeat && *alightingSeat == seatIndex)) {
                        continue;
                    }
                    const auto passengerPose =
                        passengerCabinSeatPose(busPose, bus.passengerPositions[seatIndex]);
                    const std::size_t modelIndex =
                        bus.passengerService.passengerModelIndexOnSeat(seatIndex).value_or(
                            modelOffset + seatIndex) %
                        scheduledPassengerHumanModels_.size();
                    mapRenderer_->drawHumanTrafficActor(scheduledPassengerHumanModels_[modelIndex],
                                                        passengerPose, 0.0, 0.0);
                }
                const auto& service = bus.passengerService;
                if (boardingStop == nullptr || service.currentStopId() != boardingStop->stopId ||
                    routeActor == nullptr) {
                    return;
                }
                const auto stopPose = openbus::map::sampleMapTrafficRoute(
                    routeActor->route, boardingStop->routeDistanceMeters, 0.0);
                if (!stopPose) {
                    throw std::runtime_error("Cannot sample a bus passenger boarding pose");
                }
                const double stopCameraDistance =
                    std::hypot(stopPose->position[0] - lastMainCameraPose_.position[0],
                               stopPose->position[1] - lastMainCameraPose_.position[1]);
                if (stopCameraDistance > 75.0) {
                    return;
                }
                const double passengerSide =
                    openbus::map::mapTrafficPassengerSideSign(mapRenderer_->leftHandTraffic());
                const double laneOffset =
                    routeVehicle == &bus ? routeActor->lateralOffsetMeters : 0.0;
                const double vehicleWidth =
                    routeVehicle == &bus ? routeActor->widthMeters : bus.trafficWidthMeters;
                const double curbOffset = openbus::map::mapTrafficPassengerCurbOffsetMeters(
                    mapRenderer_->leftHandTraffic(), laneOffset, vehicleWidth, 0.35);
                const auto curbPose = [&] {
                    openbus::map::MapTrafficPose pose = *stopPose;
                    const auto offset = openbus::map::mapTrafficPassengerWorldOffset(
                        mapRenderer_->leftHandTraffic(), stopPose->headingRadians,
                        curbOffset + passengerSide * 1.8, 0.0);
                    pose.position[0] += offset[0];
                    pose.position[1] += offset[1];
                    pose.headingRadians -= passengerSide * std::numbers::pi * 0.5;
                    return pose;
                };
                const auto queuePoseForPassenger = [&] {
                    const std::size_t waiting = service.waitingPassengers(boardingStop->stopId);
                    const auto initialQueue =
                        scheduledPassengerInitialQueues_.find(boardingStop->stopId);
                    const std::size_t initialCount =
                        initialQueue == scheduledPassengerInitialQueues_.end()
                            ? waiting + 1
                            : std::max(initialQueue->second, waiting + 1);
                    const std::size_t queueSlot =
                        openbus::map::mapScheduledPassengerQueueSlot(initialCount, waiting);
                    openbus::map::MapTrafficPose pose = queuePose(boardingStop->stopId,
                                                                  *stopPose, queueSlot);
                    if (!authoredPassengerPositions_.contains(boardingStop->stopId)) {
                        pose.headingRadians -= passengerSide * std::numbers::pi * 0.5;
                    }
                    return pose;
                };
                const auto drawPassengerBetweenPoses =
                    [&](const std::vector<std::array<double, 3>>& path, double progress,
                        double startHeading, double endHeading, std::size_t modelIndex) {
                        const double easedProgress = progress * progress * (3.0 - 2.0 * progress);
                        const auto passengerPose = samplePassengerTravelPath(
                            path, easedProgress, startHeading, endHeading);
                        mapRenderer_->drawHumanTrafficActor(
                            scheduledPassengerHumanModels_[modelIndex], passengerPose,
                            simulation.simulationTime() * 2.0, 1.0);
                    };
                if (service.alightingPassenger()) {
                    const auto seat = service.alightingSeatIndex();
                    if (seat && *seat < bus.passengerPositions.size()) {
                        const auto seatPose =
                            passengerCabinSeatPose(busPose, bus.passengerPositions[*seat]);
                        const auto outsidePose = curbPose();
                        std::vector<std::array<double, 3>> path = passengerCabinTravelPath(
                            bus, busPose, seatPose.position,
                            passengerCabinLocalPoint(busPose, outsidePose.position), false);
                        if (path.empty()) {
                            path = {seatPose.position, outsidePose.position};
                        }
                        const std::size_t modelIndex =
                            bus.passengerService.passengerModelIndexOnSeat(*seat).value_or(
                                modelOffset + *seat) %
                            scheduledPassengerHumanModels_.size();
                        drawPassengerBetweenPoses(path, service.alightingProgress(),
                                                  seatPose.headingRadians,
                                                  outsidePose.headingRadians, modelIndex);
                    }
                }
                if (service.boardingPassenger()) {
                    const auto boardingSeat = service.boardingSeatIndex();
                    if (boardingSeat && *boardingSeat < bus.passengerPositions.size()) {
                        const auto seatPose =
                            passengerCabinSeatPose(busPose, bus.passengerPositions[*boardingSeat]);
                        const auto outsidePose = queuePoseForPassenger();
                        std::vector<std::array<double, 3>> path = passengerCabinTravelPath(
                            bus, busPose, outsidePose.position,
                            passengerCabinLocalPosition(bus.passengerPositions[*boardingSeat]),
                            true);
                        if (path.empty()) {
                            path = {outsidePose.position, curbPose().position, seatPose.position};
                        }
                        const std::size_t modelIndex =
                            service.boardingPassengerModelIndex().value_or(
                                modelOffset + static_cast<std::size_t>(boardingStop->stopId) +
                                *boardingSeat) %
                            scheduledPassengerHumanModels_.size();
                        drawPassengerBetweenPoses(path, service.boardingProgress(),
                                                  outsidePose.headingRadians,
                                                  seatPose.headingRadians, modelIndex);
                    } else {
                        const std::size_t modelIndex =
                            service.boardingPassengerModelIndex().value_or(
                                modelOffset + static_cast<std::size_t>(boardingStop->stopId)) %
                            scheduledPassengerHumanModels_.size();
                        mapRenderer_->drawHumanTrafficActor(
                            scheduledPassengerHumanModels_[modelIndex], curbPose(),
                            simulation.simulationTime() * 2.0, 1.0);
                    }
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
            mapRenderer_->draw(true, mainViewMinimumObjectViewportFraction_);
        };
        drawVehicles(VehicleRenderPass::Opaque, false);
        for (TrafficVehicleState& trafficVehicle : trafficVehicles_) {
            if (!trafficVehicle.isBus || trafficVehicle.vehicle == nullptr ||
                !trafficVehicle.populationEnabled ||
                !trafficVehicle.vehicle->mapTrafficTileActive) {
                continue;
            }
            auto busPose = openbus::map::sampleMapTrafficActorPose(
                trafficVehicle.actor, trafficVehicle.actor.distanceMeters);
            if (!busPose) {
                throw std::runtime_error("Cannot sample a scheduled AI bus passenger pose");
            }
            busPose->position[2] += trafficVehicle.vehicle->aiDeltaHeight;
            const openbus::map::MapTrafficPassengerStop* boardingStop = nullptr;
            if (trafficVehicle.passengerStopArrived &&
                trafficVehicle.nextPassengerStopIndex < trafficVehicle.passengerStops.size()) {
                const auto& stop =
                    trafficVehicle.passengerStops[trafficVehicle.nextPassengerStopIndex];
                if (stop.stopId != trafficVehicle.vehicle->passengerService.currentStopId()) {
                    throw std::logic_error("Scheduled bus passenger service stop index does not "
                                           "match its active call");
                }
                boardingStop = &stop;
            }
            const std::size_t modelOffset =
                trafficVehicle.passengerStops.empty()
                    ? 0
                    : static_cast<std::size_t>(trafficVehicle.passengerStops.front().stopId);
            drawBusPassengerActors(*trafficVehicle.vehicle, *busPose, modelOffset,
                                   &trafficVehicle.actor, trafficVehicle.vehicle, boardingStop);
        }
        drawVehicles(VehicleRenderPass::Transparent, false);
        if (context == RenderViewContext::PlayerInterior) {
            drawTypeFourScenery();
        }
        drawVehicles(VehicleRenderPass::Opaque, true);
        if (playerVehicle_ != nullptr && activePlayerPassengerStop_) {
            if (!activePlayerPassengerStopRouteIndex_ ||
                *activePlayerPassengerStopRouteIndex_ >= passengerStopRoutes_.size() ||
                !activePlayerPassengerStopIndex_) {
                throw std::logic_error("Active player passenger stop has no mapped route call");
            }
            const PassengerStopRouteState& serviceRoute =
                passengerStopRoutes_[*activePlayerPassengerStopRouteIndex_];
            if (*activePlayerPassengerStopIndex_ >= serviceRoute.stops.size()) {
                throw std::logic_error("Active player passenger stop index is out of range");
            }
            const openbus::map::MapTrafficPassengerStop& stop =
                serviceRoute.stops[*activePlayerPassengerStopIndex_];
            if (stop.stopId != *activePlayerPassengerStop_) {
                throw std::logic_error("Active player passenger stop index does not match its ID");
            }
            openbus::map::MapTrafficPose playerPose;
            playerPose.position = chassis.position;
            playerPose.headingRadians = std::atan2(chassis.rotation[0], chassis.rotation[3]);
            drawBusPassengerActors(*playerVehicle_, playerPose, 0, &serviceRoute.actor, nullptr,
                                   &stop);
        } else if (playerVehicle_ != nullptr) {
            openbus::map::MapTrafficPose playerPose;
            playerPose.position = chassis.position;
            playerPose.headingRadians = std::atan2(chassis.rotation[0], chassis.rotation[3]);
            drawBusPassengerActors(*playerVehicle_, playerPose, 0, nullptr, nullptr, nullptr);
        }
        drawVehicles(VehicleRenderPass::Transparent, true);
        if (context == RenderViewContext::PlayerExterior) {
            drawTypeFourScenery();
        }
        if (playerVehicle_ != nullptr && !playerDrawn && collision.enabled && !collision.mesh) {
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
        const std::uint64_t collisionRevision = world.collisionDebugRevision();
        if (!collisionWireframeBuilt_ || collisionWireframeRevision_ != collisionRevision) {
            TraceScope phase("render", "RenderLoop::draw.collisionDebug.build");
            collisionWireframeVertices_.clear();
            collisionSurfaceWireframeVertices_.clear();
            const std::vector<StaticCollisionMesh> meshes = world.collisionDebugMeshes();
            std::size_t triangleCount = 0;
            std::size_t surfaceTriangleCount = 0;
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
                if (mesh.surfaceHeightMapIndices.size() % 3 != 0) {
                    continue;
                }
                surfaceTriangleCount += mesh.surfaceHeightMapIndices.size() / 3;
                for (std::size_t index = 0; index + 2 < mesh.surfaceHeightMapIndices.size();
                     index += 3) {
                    const std::array<int, 3> triangle = {mesh.surfaceHeightMapIndices[index],
                                                         mesh.surfaceHeightMapIndices[index + 1],
                                                         mesh.surfaceHeightMapIndices[index + 2]};
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
                            collisionSurfaceWireframeVertices_.push_back(
                                {static_cast<float>(mesh.vertices[vertexOffset]),
                                 static_cast<float>(mesh.vertices[vertexOffset + 1]),
                                 static_cast<float>(mesh.vertices[vertexOffset + 2] + 0.03), 0.10F,
                                 0.90F, 1.0F});
                        }
                    }
                }
            }
            collisionWireframeBuilt_ = true;
            collisionWireframeRevision_ = collisionRevision;
            gameLog.Log("Collision wireframe cached: " + std::to_string(triangleCount) +
                        " triangles across " + std::to_string(meshes.size()) +
                        " active terrain/road/scenery meshes; " +
                        std::to_string(surfaceTriangleCount) +
                        " road triangles use .surf (cyan outline)");
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
        openbus::rendering::drawStaticPrimitives(collisionSurfaceWireframeBuffer_,
                                                 collisionSurfaceWireframeVertices_, GL_LINES, 2.5F,
                                                 collisionRevision);
        std::vector<openbus::rendering::PrimitiveVertex> poleCollisionVertices;
        const std::vector<MapPoleState> poles = world.mapPoleStates();
        for (const MapPoleState& pole : poles) {
            appendPoleCollisionOutline(poleCollisionVertices, pole);
        }
        openbus::rendering::drawPrimitives(poleCollisionVertices, GL_LINES, 2.0F);
        if (playerVehicle_ != nullptr && collision.enabled && !collision.mesh) {
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
    std::vector<ActiveSceneryTrafficSegment> activeSceneryTrafficSegments;
    std::vector<const openbus::rendering::EditorPassengerDebugObject*> activeEditorPassengerObjects;
    if (!renderingReflection_ && trafficSplineDebugOverlay_ && mapRenderer_ != nullptr) {
        const std::vector<std::pair<int, int>>& activeTiles = mapRenderer_->activeTileCoordinates();
        for (const auto& object : mapRenderer_->editorPassengerDebugObjects()) {
            if (std::find(activeTiles.begin(), activeTiles.end(),
                          std::pair{object.tileX, object.tileY}) != activeTiles.end()) {
                activeEditorPassengerObjects.push_back(&object);
            }
        }
        const std::vector<openbus::rendering::TrafficPathDebugSegment>& scenerySegments =
            mapRenderer_->sceneryTrafficPathSegments();
        activeSceneryTrafficSegments.reserve(scenerySegments.size());
        for (std::size_t index = 0; index < scenerySegments.size(); ++index) {
            const openbus::rendering::TrafficPathDebugSegment& segment = scenerySegments[index];
            if (std::find(activeTiles.begin(), activeTiles.end(),
                          std::pair{segment.tileX, segment.tileY}) != activeTiles.end()) {
                activeSceneryTrafficSegments.push_back({index, &segment});
            }
        }
        const double tileSize = mapRenderer_->tileSize();
        // Keep world-space debug vertices close to zero before converting them to floats.
        const std::array<double, 2> trafficDebugOrigin = {
            std::floor(lastMainCameraPose_.position[0] / tileSize) * tileSize,
            std::floor(lastMainCameraPose_.position[1] / tileSize) * tileSize};
        const std::uint64_t mapTrafficGeometryRevision =
            mapRenderer_->trafficDebugGeometryRevision();
        if (!trafficSplineDebugBuilt_ || trafficSplineDebugTiles_ != activeTiles ||
            trafficSplineDebugMapRevision_ != mapTrafficGeometryRevision ||
            trafficSplineDebugOriginX_ != trafficDebugOrigin[0] ||
            trafficSplineDebugOriginY_ != trafficDebugOrigin[1]) {
            trafficSplineDebugVertices_.clear();
            const std::vector<ActiveTrafficSplineRoute> activeTrafficRoutes =
                activeTrafficSplineRoutes(*mapRenderer_, activeTiles);
            trafficSplineDebugLabels_.clear();
            trafficSplineDebugLabels_.reserve(activeTrafficRoutes.size());
            for (const ActiveTrafficSplineRoute& activeRoute : activeTrafficRoutes) {
                const double splineLength = activeRoute.firstSegment.spline.placement.length;
                if (!std::isfinite(splineLength) || splineLength <= 0.0) {
                    continue;
                }
                openbus::map::MapTrafficRoute singleSplineRoute;
                singleSplineRoute.segments.push_back(activeRoute.firstSegment);
                singleSplineRoute.cumulativeEndDistances.push_back(splineLength);
                singleSplineRoute.totalLength = splineLength;
                const auto pose =
                    openbus::map::sampleMapTrafficRoute(singleSplineRoute, splineLength * 0.5, 0.0);
                if (pose.has_value()) {
                    std::array<double, 3> labelPosition = pose->position;
                    labelPosition[2] += 2.2;
                    const auto pathOffsets = mapRenderer_->trafficSplinePathOffsets(
                        activeRoute.splineId, activeRoute.pathTypeCode);
                    trafficSplineDebugLabels_.push_back(
                        {activeRoute.splineId, activeRoute.pathTypeCode, labelPosition,
                         openbus::app::trafficDebugHasBidirectionalPath(pathOffsets)});
                }
            }
            const SceneryTrafficEndpointIndex sceneryEndpoints(
                *mapRenderer_, activeSceneryTrafficSegments, activeTrafficRoutes);
            appendTrafficSplineDebugVertices(*mapRenderer_, openbus::map::MAP_SPLINE_PATH_TYPE_ROAD,
                                             activeTrafficRoutes, {0.15F, 0.9F, 1.0F},
                                             sceneryEndpoints, trafficSplineDebugVertices_,
                                             trafficDebugOrigin);
            appendTrafficSplineDebugVertices(
                *mapRenderer_, openbus::map::MAP_SPLINE_PATH_TYPE_HUMAN, activeTrafficRoutes,
                {1.0F, 0.25F, 0.9F}, sceneryEndpoints, trafficSplineDebugVertices_,
                trafficDebugOrigin);
            appendSceneryTrafficPathDebugVertices(activeSceneryTrafficSegments,
                                                  trafficSplineDebugVertices_, trafficDebugOrigin);
            trafficSplineDebugTiles_ = activeTiles;
            trafficSplineDebugMapRevision_ = mapTrafficGeometryRevision;
            trafficSplineDebugOriginX_ = trafficDebugOrigin[0];
            trafficSplineDebugOriginY_ = trafficDebugOrigin[1];
            trafficSplineDebugBuilt_ = true;
            ++trafficSplineDebugRevision_;
        }

        if (!trafficSplineDebugVertices_.empty()) {
            const GLboolean depthWasEnabled = glIsEnabled(GL_DEPTH_TEST);
            GLboolean depthWriteWasEnabled = GL_TRUE;
            glGetBooleanv(GL_DEPTH_WRITEMASK, &depthWriteWasEnabled);
            glDisable(GL_DEPTH_TEST);
            glDepthMask(GL_FALSE);
            pushMatrix();
            multiplyMatrix(translationMatrix({trafficDebugOrigin[0], trafficDebugOrigin[1], 0.0}));
            openbus::rendering::drawStaticPrimitives(trafficSplineDebugBuffer_,
                                                     trafficSplineDebugVertices_, GL_LINES, 2.0F,
                                                     trafficSplineDebugRevision_);
            popMatrix();
            glDepthMask(depthWriteWasEnabled);
            if (depthWasEnabled) {
                glEnable(GL_DEPTH_TEST);
            }
        }

        std::vector<openbus::rendering::PrimitiveVertex> behaviorVertices;
        std::vector<openbus::rendering::PrimitiveVertex> vehicleBodyVertices;
        std::vector<openbus::map::MapTrafficPose> renderedSignalStops;
        constexpr std::array<std::array<std::size_t, 2>, 12> editorEdges = {{
            {0, 1}, {1, 3}, {3, 2}, {2, 0}, {4, 5}, {5, 7},
            {7, 6}, {6, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7}}};
        for (const auto* object : activeEditorPassengerObjects) {
            const std::array<float, 3> color =
                object->busStop ? std::array<float, 3>{1.0F, 0.75F, 0.1F}
                                : std::array<float, 3>{0.2F, 0.8F, 1.0F};
            for (const auto& edge : editorEdges) {
                appendTrafficBehaviorLine(behaviorVertices, object->bounds[edge[0]],
                                          object->bounds[edge[1]], color, trafficDebugOrigin);
            }
            for (const auto& passenger : object->passengerPositions) {
                appendTrafficBehaviorBox(behaviorVertices, passenger, 0.25, 0.25, 1.8,
                                         {0.2F, 0.8F, 1.0F}, trafficDebugOrigin);
            }
        }
        for (const auto& [stopId, candidate] : stopCandidates) {
            const std::size_t waiting = scheduledPassengerQueues_.at(stopId);
            const auto initialQueue = scheduledPassengerInitialQueues_.find(stopId);
            const std::size_t initialCount =
                initialQueue == scheduledPassengerInitialQueues_.end()
                    ? waiting
                    : std::max(initialQueue->second, waiting);
            const std::size_t markerCount = std::max<std::size_t>(waiting, 1);
            for (std::size_t passenger = 0; passenger < markerCount; ++passenger) {
                const std::size_t queueSlot =
                    waiting == 0
                        ? 0
                        : openbus::map::mapScheduledPassengerQueueSlot(initialCount, passenger);
                const auto pose = queuePose(stopId, candidate.pose, queueSlot);
                appendTrafficBehaviorBox(behaviorVertices, pose, 0.4, 0.4, 1.8,
                                         {0.25F, 1.0F, 0.45F}, trafficDebugOrigin);
            }
        }
        for (const ActiveSceneryTrafficSegment& activeSegment : activeSceneryTrafficSegments) {
            const openbus::rendering::TrafficPathDebugSegment& segment = *activeSegment.segment;
            if (!segment.trafficSignal.has_value() ||
                segment.pathTypeCode != openbus::map::MAP_SPLINE_PATH_TYPE_ROAD ||
                segment.points.size() < 2) {
                continue;
            }
            const auto& start = segment.points.front();
            const auto& next = segment.points[1];
            const double dx = next[0] - start[0];
            const double dy = next[1] - start[1];
            const double tangentLength = std::hypot(dx, dy);
            if (!std::isfinite(tangentLength) || tangentLength <= 1.0e-6 ||
                !std::isfinite(segment.widthMeters) || segment.widthMeters <= 0.0) {
                continue;
            }
            const double forwardX = dx / tangentLength;
            const double forwardY = dy / tangentLength;
            const double lateralX = forwardY;
            const double lateralY = -forwardX;
            const std::array<double, 3> stopCenter = {start[0] - forwardX * 0.75,
                                                      start[1] - forwardY * 0.75, start[2] + 0.16};
            const std::optional<int> phase = openbus::map::mapTrafficSignalStateAtTime(
                *segment.trafficSignal, simulation.simulationTime());
            const std::array<float, 3> stopColor =
                !phase.has_value() || *phase < 6 || *phase > 8
                    ? std::array<float, 3>{1.0F, 0.12F, 0.08F}
                    : (*phase >= 9 && *phase <= 11 ? std::array<float, 3>{1.0F, 0.72F, 0.05F}
                                                   : std::array<float, 3>{0.08F, 1.0F, 0.18F});
            std::array<double, 3> lineStart = stopCenter;
            std::array<double, 3> lineEnd = stopCenter;
            for (std::size_t axis = 0; axis < 2; ++axis) {
                const double lateral = axis == 0 ? lateralX : lateralY;
                lineStart[axis] -= lateral * segment.widthMeters * 0.5;
                lineEnd[axis] += lateral * segment.widthMeters * 0.5;
            }
            appendTrafficBehaviorLine(behaviorVertices, lineStart, lineEnd, stopColor,
                                      trafficDebugOrigin);
        }
        const auto appendSignalStopBox = [&](const openbus::map::MapTrafficActor& actor) {
            const auto stop =
                openbus::map::mapTrafficNextSignalStopDebugInfo(actor, simulation.simulationTime());
            if (!stop.has_value() || stop->distanceAheadMeters > 50.0 ||
                !openbus::map::mapTrafficPoseOnActiveTile(stop->pose, activeTiles, tileSize)) {
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
            const double halfWidth = std::max(actor.widthMeters + 1.0, 2.5) * 0.5;
            const double rightX = std::cos(stop->pose.headingRadians);
            const double rightY = -std::sin(stop->pose.headingRadians);
            std::array<double, 3> lineStart = stop->pose.position;
            std::array<double, 3> lineEnd = stop->pose.position;
            lineStart[2] += 0.16;
            lineEnd[2] += 0.16;
            lineStart[0] -= rightX * halfWidth;
            lineStart[1] -= rightY * halfWidth;
            lineEnd[0] += rightX * halfWidth;
            lineEnd[1] += rightY * halfWidth;
            appendTrafficBehaviorLine(behaviorVertices, lineStart, lineEnd, color,
                                      trafficDebugOrigin);
        };
        const auto behaviorColor = [](openbus::map::MapTrafficBlockReason reason, bool onJunction,
                                      bool deadlocked) {
            if (deadlocked) {
                return std::array<float, 3>{1.0F, 0.05F, 0.05F};
            }
            switch (reason) {
            case openbus::map::MapTrafficBlockReason::Leader:
                return std::array<float, 3>{1.0F, 0.85F, 0.05F};
            case openbus::map::MapTrafficBlockReason::CrossingReservation:
                return std::array<float, 3>{1.0F, 0.5F, 0.05F};
            case openbus::map::MapTrafficBlockReason::JunctionTraffic:
                return std::array<float, 3>{0.9F, 0.15F, 1.0F};
            case openbus::map::MapTrafficBlockReason::Signal:
            case openbus::map::MapTrafficBlockReason::Collision:
                return std::array<float, 3>{1.0F, 0.1F, 0.1F};
            case openbus::map::MapTrafficBlockReason::Vehicle:
            case openbus::map::MapTrafficBlockReason::Obstacle:
                return std::array<float, 3>{1.0F, 0.3F, 0.05F};
            case openbus::map::MapTrafficBlockReason::None:
                return onJunction ? std::array<float, 3>{0.15F, 0.45F, 1.0F}
                                  : std::array<float, 3>{0.15F, 1.0F, 0.2F};
            }
            return std::array<float, 3>{1.0F, 1.0F, 1.0F};
        };
        const auto appendJunctionStopBar = [&](const openbus::map::MapTrafficActor& actor,
                                               const std::array<float, 3>& color) {
            if (openbus::map::mapTrafficActorOnJunctionPath(actor, actor.distanceMeters)) {
                return;
            }
            double junctionStart = std::numeric_limits<double>::infinity();
            const double searchEnd = std::min(actor.route.totalLength, actor.distanceMeters + 80.0);
            for (double distance = actor.distanceMeters; distance <= searchEnd; distance += 0.5) {
                if (openbus::map::mapTrafficRoutePositionOnJunctionPath(actor.route, distance)) {
                    junctionStart = distance;
                    break;
                }
            }
            if (!std::isfinite(junctionStart)) {
                return;
            }
            const double stopDistance =
                std::max(actor.distanceMeters, junctionStart - actor.lengthMeters * 0.5 - 0.5);
            const auto stopPose = openbus::map::sampleMapTrafficRoute(actor.route, stopDistance,
                                                                      actor.lateralOffsetMeters);
            if (stopPose.has_value() &&
                openbus::map::mapTrafficPoseOnActiveTile(*stopPose, activeTiles, tileSize)) {
                appendTrafficBehaviorBox(behaviorVertices, *stopPose, 0.3,
                                         std::max(actor.widthMeters + 1.0, 2.5), 0.12, color,
                                         trafficDebugOrigin);
            }
        };
        std::unordered_map<const openbus::map::MapTrafficActor*, openbus::map::MapTrafficPose>
            activeActorPoses;
        for (const TrafficVehicleState& state : trafficVehicles_) {
            if (!state.populationEnabled || state.vehicle == nullptr ||
                !state.vehicle->mapTrafficTileActive) {
                continue;
            }
            const auto pose = openbus::map::sampleMapTrafficActorPose(
                state.actor, state.actor.distanceMeters);
            if (pose.has_value()) {
                activeActorPoses.emplace(&state.actor, *pose);
            }
        }
        std::vector<openbus::map::MapTrafficPose> externalVehiclePoses;
        for (const std::unique_ptr<Vehicle>& vehicle : vehicles_) {
            if (vehicle->followsMapTrafficRoute || vehicle->mapTrafficRouteFinished) {
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
            externalVehiclePoses.push_back(pose);
        }
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
            if (openbus::map::mapTrafficPoseOnActiveTile(pose, activeTiles, tileSize)) {
                std::vector<openbus::rendering::PrimitiveVertex>& vertices =
                    vehicle.get() == playerVehicle_ ? behaviorVertices : vehicleBodyVertices;
                appendTrafficBehaviorBox(vertices, pose, vehicle->trafficLengthMeters,
                                         vehicle->trafficWidthMeters, vehicle->trafficHeightMeters,
                                         {0.95F, 0.95F, 0.95F}, trafficDebugOrigin);
                appendTrafficBehaviorBox(behaviorVertices, pose, vehicle->trafficLengthMeters,
                                         vehicle->trafficWidthMeters, 0.1, {1.0F, 1.0F, 1.0F},
                                         trafficDebugOrigin);
            }
        }
        for (const TrafficVehicleState& state : trafficVehicles_) {
            if (!state.populationEnabled || state.vehicle == nullptr ||
                !state.vehicle->mapTrafficTileActive) {
                continue;
            }
            const auto foundPose = activeActorPoses.find(&state.actor);
            if (foundPose == activeActorPoses.end()) {
                continue;
            }
            const auto* pose = &foundPose->second;
            const bool onJunction = openbus::map::mapTrafficRoutePositionOnJunctionPath(
                state.actor.route, state.actor.distanceMeters);
            const std::array<float, 3> color =
                behaviorColor(state.actor.blockReason, onJunction, state.deadlocked);
            if (state.vehicle->loaded && !state.vehicle->displayLists.empty()) {
                appendTrafficBehaviorBox(
                    vehicleBodyVertices, *pose, state.actor.lengthMeters, state.actor.widthMeters,
                    state.vehicle->trafficHeightMeters, color, trafficDebugOrigin);
            }
            appendTrafficBehaviorBox(behaviorVertices, *pose, state.actor.lengthMeters,
                                     state.actor.widthMeters, 0.1, color, trafficDebugOrigin);
            auto arrowCenter = state.vehicle->placement.position;
            arrowCenter[2] += std::max(state.vehicle->trafficHeightMeters, 1.0) + 0.3;
            const double vehicleHeading =
                (90.0 - state.vehicle->placement.yawDegrees) * std::numbers::pi / 180.0;
            appendTrafficDirectionArrow(behaviorVertices, arrowCenter, vehicleHeading,
                                        std::clamp(state.actor.lengthMeters * 0.6, 2.0, 6.0),
                                        {1.0F, 1.0F, 1.0F}, trafficDebugOrigin);
            appendSignalStopBox(state.actor);

            const double reservationHorizon = state.actor.speedMetersPerSecond * 3.0;
            for (double ahead = 2.0; ahead < reservationHorizon; ahead += 2.0) {
                const auto predictedPose = openbus::map::sampleMapTrafficActorPose(
                    state.actor, state.actor.distanceMeters + ahead);
                if (predictedPose.has_value() && openbus::map::mapTrafficPoseOnActiveTile(
                                                     *predictedPose, activeTiles, tileSize)) {
                    appendTrafficBehaviorBox(behaviorVertices, *predictedPose, 2.0,
                                             state.actor.widthMeters + 0.25, 0.08,
                                             {0.05F, 0.85F, 1.0F}, trafficDebugOrigin);
                }
                const double reservationDistance = state.actor.distanceMeters + ahead;
                if (!openbus::map::mapTrafficRoutePositionOnJunctionPath(state.actor.route,
                                                                         reservationDistance)) {
                    continue;
                }
                if (!predictedPose.has_value() || !openbus::map::mapTrafficPoseOnActiveTile(
                                                      *predictedPose, activeTiles, tileSize)) {
                    continue;
                }
                appendTrafficBehaviorBox(behaviorVertices, *predictedPose,
                                         state.actor.lengthMeters, state.actor.widthMeters, 0.12,
                                         {1.0F, 0.5F, 0.05F}, trafficDebugOrigin);
            }
            if (state.actor.junctionClaimEndMeters.has_value()) {
                const double claimEnd =
                    std::min(*state.actor.junctionClaimEndMeters, state.actor.route.totalLength);
                for (double distance = state.actor.distanceMeters + 1.0; distance < claimEnd;
                     distance += 2.0) {
                    const auto claimPose = openbus::map::sampleMapTrafficActorPose(
                        state.actor, distance);
                    if (claimPose.has_value() && openbus::map::mapTrafficPoseOnActiveTile(
                                                     *claimPose, activeTiles, tileSize)) {
                        appendTrafficBehaviorBox(behaviorVertices, *claimPose,
                                                 state.actor.lengthMeters,
                                                 state.actor.widthMeters + 0.3, 0.08,
                                                 {0.05F, 0.75F, 0.65F}, trafficDebugOrigin);
                    }
                }
            }
            if (state.actor.blockReason == openbus::map::MapTrafficBlockReason::JunctionTraffic ||
                state.actor.blockReason ==
                    openbus::map::MapTrafficBlockReason::CrossingReservation) {
                appendJunctionStopBar(state.actor, color);
            }
            if (state.actor.blockingActor != nullptr) {
                const auto blockerPose = activeActorPoses.find(state.actor.blockingActor);
                if (blockerPose != activeActorPoses.end()) {
                    std::array<double, 3> from = pose->position;
                    std::array<double, 3> to = blockerPose->second.position;
                    from[2] += 0.25;
                    to[2] += 0.25;
                    appendTrafficBehaviorLine(behaviorVertices, from, to, color,
                                              trafficDebugOrigin);
                }
            } else if (state.actor.blockingObstacleIndex.has_value() &&
                       *state.actor.blockingObstacleIndex < externalVehiclePoses.size()) {
                std::array<double, 3> from = pose->position;
                std::array<double, 3> to =
                    externalVehiclePoses[*state.actor.blockingObstacleIndex].position;
                from[2] += 0.25;
                to[2] += 0.25;
                appendTrafficBehaviorLine(behaviorVertices, from, to, color, trafficDebugOrigin);
            }
        }
        for (const HumanTrafficState& human : humanTrafficActors_) {
            if (!human.populationEnabled) {
                continue;
            }
            const auto pose = openbus::map::sampleMapTrafficRoute(
                human.actor.route, human.actor.distanceMeters, human.actor.lateralOffsetMeters);
            if (!pose.has_value() ||
                !openbus::map::mapTrafficPoseOnActiveTile(*pose, activeTiles, tileSize)) {
                continue;
            }
            appendTrafficBehaviorBox(behaviorVertices, *pose, human.feetDistanceMeters * 1.25,
                                     human.feetDistanceMeters * 2.1, human.heightMeters,
                                     {1.0F, 0.25F, 0.9F}, trafficDebugOrigin);
            appendTrafficBehaviorBox(behaviorVertices, *pose, human.feetDistanceMeters * 1.25,
                                     human.feetDistanceMeters * 2.1, 0.08, {1.0F, 0.25F, 0.9F},
                                     trafficDebugOrigin);
            appendSignalStopBox(human.actor);
        }
        if (!vehicleBodyVertices.empty()) {
            const GLboolean depthWasEnabled = glIsEnabled(GL_DEPTH_TEST);
            GLboolean depthWriteWasEnabled = GL_TRUE;
            glGetBooleanv(GL_DEPTH_WRITEMASK, &depthWriteWasEnabled);
            glEnable(GL_DEPTH_TEST);
            glDepthMask(GL_FALSE);
            pushMatrix();
            multiplyMatrix(translationMatrix({trafficDebugOrigin[0], trafficDebugOrigin[1], 0.0}));
            openbus::rendering::drawStaticPrimitives(trafficBehaviorDebugBuffer_,
                                                     vehicleBodyVertices, GL_LINES, 1.5F,
                                                     ++trafficBehaviorDebugRevision_);
            popMatrix();
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
            pushMatrix();
            multiplyMatrix(translationMatrix({trafficDebugOrigin[0], trafficDebugOrigin[1], 0.0}));
            openbus::rendering::drawStaticPrimitives(trafficBehaviorDebugBuffer_, behaviorVertices,
                                                     GL_LINES, 1.5F,
                                                     ++trafficBehaviorDebugRevision_);
            popMatrix();
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
    if (!renderingReflection_ && !captureMode_ && imguiEnabled_) {
        TraceScope phase("render", "RenderLoop::draw.imguiOverlays");
        {
            const openbus::rendering::MapRenderStats mapStats =
                mapRenderer_ != nullptr ? mapRenderer_->renderStats()
                                        : openbus::rendering::MapRenderStats{};
            const std::size_t spawnedAiVehicleCount = trafficVehicleCount();
            const std::size_t activeHumanCount = static_cast<std::size_t>(std::count_if(
                humanTrafficActors_.begin(), humanTrafficActors_.end(),
                [](const HumanTrafficState& human) { return human.populationEnabled; }));
            const SystemVariables& systemVariables = simulationState_.sharedVariables();
            const int year = static_cast<int>(systemVariables.get("year"));
            const int month = static_cast<int>(systemVariables.get("month"));
            const int day = static_cast<int>(systemVariables.get("day"));
            const int secondsSinceMidnight =
                std::clamp(static_cast<int>(std::floor(systemVariables.get("time"))), 0, 86399);
            const auto playerPosition = streamingPosition(simulation);
            int currentTileX = 0;
            int currentTileY = 0;
            bool currentTileActive = false;
            if (mapRenderer_ != nullptr) {
                const double tileSize = mapRenderer_->tileSize();
                currentTileX = openbus::map::mapWorldTile(playerPosition[0], tileSize);
                currentTileY = openbus::map::mapWorldTile(playerPosition[1], tileSize);
                const auto& activeTiles = mapRenderer_->activeTileCoordinates();
                currentTileActive =
                    std::find(activeTiles.begin(), activeTiles.end(),
                              std::pair{currentTileX, currentTileY}) != activeTiles.end();
            }
            std::ostringstream statusText;
            statusText << std::setfill('0') << std::setw(4) << year << '-' << std::setw(2) << month
                       << '-' << std::setw(2) << day << ' ' << std::setw(2)
                       << secondsSinceMidnight / 3600 << ':' << std::setw(2)
                       << (secondsSinceMidnight / 60) % 60 << ':' << std::setw(2)
                       << secondsSinceMidnight % 60 << std::setfill(' ') << '\n'
                       << "X " << std::fixed << std::setprecision(1) << playerPosition[0] << " Y "
                       << playerPosition[1] << " Z " << playerPosition[2];
            if (mapRenderer_ != nullptr) {
                statusText << " TILE " << currentTileX << ',' << currentTileY << ' '
                           << (currentTileActive ? "ACTIVE" : "NOT ACTIVE");
            }
            statusText << '\n'
                       << "CHUNKS " << mapStats.activeTiles << '/' << mapStats.totalTiles
                       << " TERRAIN " << mapStats.activeTerrainTiles << '/'
                       << mapStats.totalTerrainTiles << " OBJECTS " << mapStats.activeSceneryObjects
                       << '/' << mapStats.totalSceneryObjects << '\n'
                       << "ROADS " << mapStats.activeRoadBatches << '/' << mapStats.totalRoadBatches
                       << " COLLISION TILES " << world.loadedMapCollisionTileCount()
                       << " AI VEHICLES " << spawnedAiVehicleCount << " HUMANS " << activeHumanCount
                       << " AI " << std::fixed << std::setprecision(1) << currentAiSimulationRateHz_
                       << " HZ / " << currentAiWorkerTickMilliseconds_ << " MS";
            if (trafficSplineDebugOverlay_) {
                const double debugDeltaSeconds = std::clamp(frameTimeStep_, 0.0, 0.25);
                std::size_t clearCount = 0;
                std::size_t followingCount = 0;
                std::size_t yieldCount = 0;
                std::size_t reservationCount = 0;
                std::size_t signalCount = 0;
                std::size_t blockedCount = 0;
                std::size_t junctionCount = 0;
                std::size_t claimCount = 0;
                std::size_t deadlockedCount = 0;
                TrafficVehicleState* hottestBlock = nullptr;
                TrafficVehicleState* heldHotBlock = nullptr;
                int hottestRank = 0;
                for (TrafficVehicleState& state : trafficVehicles_) {
                    if (!state.populationEnabled || state.vehicle == nullptr ||
                        !state.vehicle->mapTrafficTileActive) {
                        state.debugDisplayedBlockReason = openbus::map::MapTrafficBlockReason::None;
                        state.debugCandidateBlockReason = openbus::map::MapTrafficBlockReason::None;
                        state.debugCandidateBlockSeconds = 0.0;
                        state.debugBlockReasonHoldSeconds = 0.0;
                        continue;
                    }
                    const openbus::map::MapTrafficBlockReason currentReason =
                        state.actor.blockReason;
                    if (currentReason == state.debugDisplayedBlockReason) {
                        state.debugCandidateBlockReason = openbus::map::MapTrafficBlockReason::None;
                        state.debugCandidateBlockSeconds = 0.0;
                        if (currentReason != openbus::map::MapTrafficBlockReason::None) {
                            state.debugBlockReasonHoldSeconds = 0.6;
                        }
                    } else if (currentReason == openbus::map::MapTrafficBlockReason::None) {
                        state.debugCandidateBlockReason = openbus::map::MapTrafficBlockReason::None;
                        state.debugCandidateBlockSeconds = 0.0;
                        state.debugBlockReasonHoldSeconds =
                            std::max(0.0, state.debugBlockReasonHoldSeconds - debugDeltaSeconds);
                        if (state.debugBlockReasonHoldSeconds == 0.0) {
                            state.debugDisplayedBlockReason =
                                openbus::map::MapTrafficBlockReason::None;
                        }
                    } else {
                        if (state.debugCandidateBlockReason == currentReason) {
                            state.debugCandidateBlockSeconds += debugDeltaSeconds;
                        } else {
                            state.debugCandidateBlockReason = currentReason;
                            state.debugCandidateBlockSeconds = 0.0;
                        }
                        if (state.debugCandidateBlockSeconds >= 0.15) {
                            state.debugDisplayedBlockReason = currentReason;
                            state.debugBlockReasonHoldSeconds = 0.6;
                            state.debugCandidateBlockReason =
                                openbus::map::MapTrafficBlockReason::None;
                            state.debugCandidateBlockSeconds = 0.0;
                        }
                    }
                    switch (state.actor.blockReason) {
                    case openbus::map::MapTrafficBlockReason::None:
                        ++clearCount;
                        break;
                    case openbus::map::MapTrafficBlockReason::Leader:
                        ++followingCount;
                        break;
                    case openbus::map::MapTrafficBlockReason::JunctionTraffic:
                        ++yieldCount;
                        break;
                    case openbus::map::MapTrafficBlockReason::CrossingReservation:
                        ++reservationCount;
                        break;
                    case openbus::map::MapTrafficBlockReason::Signal:
                        ++signalCount;
                        break;
                    case openbus::map::MapTrafficBlockReason::Vehicle:
                    case openbus::map::MapTrafficBlockReason::Obstacle:
                    case openbus::map::MapTrafficBlockReason::Collision:
                        ++blockedCount;
                        break;
                    }
                    junctionCount += openbus::map::mapTrafficActorOnJunctionPath(
                                         state.actor, state.actor.distanceMeters)
                                         ? 1U
                                         : 0U;
                    claimCount += state.actor.junctionClaimEndMeters.has_value() ? 1U : 0U;
                    deadlockedCount += state.deadlocked ? 1U : 0U;
                    const bool stoppedWithoutConstraint =
                        state.actor.speedMetersPerSecond <= 0.1 &&
                        state.actor.targetSpeedMetersPerSecond <= 0.1;
                    const int diagnosticRank = state.deadlocked ? 3
                                               : state.debugDisplayedBlockReason !=
                                                       openbus::map::MapTrafficBlockReason::None
                                                   ? 2
                                               : stoppedWithoutConstraint ? 1
                                                                          : 0;
                    if (state.vehicle == trafficDebugHotVehicle_ && diagnosticRank > 0) {
                        heldHotBlock = &state;
                    }
                    if (diagnosticRank > hottestRank ||
                        (diagnosticRank > 0 && diagnosticRank == hottestRank &&
                         (hottestBlock == nullptr ||
                          state.actor.speedMetersPerSecond <
                              hottestBlock->actor.speedMetersPerSecond))) {
                        hottestBlock = &state;
                        hottestRank = diagnosticRank;
                    }
                }
                if (heldHotBlock != nullptr && trafficDebugHotHoldSeconds_ > 0.0) {
                    hottestBlock = heldHotBlock;
                    trafficDebugHotHoldSeconds_ =
                        std::max(0.0, trafficDebugHotHoldSeconds_ - debugDeltaSeconds);
                } else if (hottestBlock != nullptr) {
                    trafficDebugHotVehicle_ = hottestBlock->vehicle;
                    trafficDebugHotHoldSeconds_ = 1.5;
                } else {
                    trafficDebugHotVehicle_ = nullptr;
                    trafficDebugHotHoldSeconds_ = 0.0;
                }
                statusText << '\n'
                           << "F2 C" << clearCount << " F" << followingCount << " Y" << yieldCount
                           << " R" << reservationCount << " S" << signalCount << " B"
                           << blockedCount << " J" << junctionCount << " L" << claimCount << " D"
                           << deadlockedCount << " HOT~ ";
                if (hottestBlock == nullptr) {
                    statusText << "NONE";
                } else {
                    const auto pose = openbus::map::sampleMapTrafficRoute(
                        hottestBlock->actor.route, hottestBlock->actor.distanceMeters,
                        hottestBlock->actor.lateralOffsetMeters);
                    const char* reason = "DEAD";
                    if (!hottestBlock->deadlocked) {
                        reason = hottestBlock->debugDisplayedBlockReason ==
                                         openbus::map::MapTrafficBlockReason::None
                                     ? "HOLD"
                                     : "BLOCK";
                        switch (hottestBlock->debugDisplayedBlockReason) {
                        case openbus::map::MapTrafficBlockReason::Leader:
                            reason = "FLW";
                            break;
                        case openbus::map::MapTrafficBlockReason::JunctionTraffic:
                            reason = "YLD";
                            break;
                        case openbus::map::MapTrafficBlockReason::CrossingReservation:
                            reason = "RSV";
                            break;
                        case openbus::map::MapTrafficBlockReason::Signal:
                            reason = "SIG";
                            break;
                        case openbus::map::MapTrafficBlockReason::None:
                            break;
                        case openbus::map::MapTrafficBlockReason::Vehicle:
                        case openbus::map::MapTrafficBlockReason::Obstacle:
                        case openbus::map::MapTrafficBlockReason::Collision:
                            reason = "BLK";
                            break;
                        }
                    }
                    statusText << 'S' << (pose ? pose->splineId : 0) << ' ' << reason << ' '
                               << std::fixed << std::setprecision(0)
                               << hottestBlock->actor.speedMetersPerSecond * 3.6 << "KMH T"
                               << hottestBlock->actor.targetSpeedMetersPerSecond * 3.6 << "KMH W"
                               << std::setprecision(1) << hottestBlock->actor.junctionWaitSeconds
                               << "S";
                    if (hottestBlock->actor.blockingActor != nullptr) {
                        const auto blockerPose = openbus::map::sampleMapTrafficRoute(
                            hottestBlock->actor.blockingActor->route,
                            hottestBlock->actor.blockingActor->distanceMeters,
                            hottestBlock->actor.blockingActor->lateralOffsetMeters);
                        statusText << " BY S" << (blockerPose ? blockerPose->splineId : 0) << ' '
                                   << std::fixed << std::setprecision(0)
                                   << hottestBlock->actor.blockingActor->speedMetersPerSecond * 3.6
                                   << "KMH";
                    } else if (hottestBlock->actor.blockingObstacleIndex.has_value()) {
                        statusText << " BY OBS#" << *hottestBlock->actor.blockingObstacleIndex;
                    }
                }
            } else {
                trafficDebugHotVehicle_ = nullptr;
                trafficDebugHotHoldSeconds_ = 0.0;
                for (TrafficVehicleState& state : trafficVehicles_) {
                    state.debugDisplayedBlockReason = openbus::map::MapTrafficBlockReason::None;
                    state.debugCandidateBlockReason = openbus::map::MapTrafficBlockReason::None;
                    state.debugCandidateBlockSeconds = 0.0;
                    state.debugBlockReasonHoldSeconds = 0.0;
                }
            }
            if (trafficSplineDebugOverlay_ && mapRenderer_ != nullptr) {
                const openbus::rendering::TrafficSignalDebugStats signalStats =
                    mapRenderer_->trafficSignalDebugStats();
                statusText << '\n'
                           << "TL " << signalStats.activeSignalModels << " C"
                           << signalStats.controllerDrivenModels << " PH";
                std::size_t shownPhases = 0;
                std::size_t additionalPhases = 0;
                for (std::size_t phase = 0; phase < signalStats.modelsByPhase.size(); ++phase) {
                    if (signalStats.modelsByPhase[phase] == 0) {
                        continue;
                    }
                    if (shownPhases == 6) {
                        ++additionalPhases;
                        continue;
                    }
                    statusText << ' ' << phase << ':' << signalStats.modelsByPhase[phase];
                    ++shownPhases;
                }
                if (additionalPhases > 0) {
                    statusText << " +";
                }
                if (signalStats.modelsWithoutPhase > 0) {
                    statusText << " ?:" << signalStats.modelsWithoutPhase;
                }
            }
            const std::string currentStatus = statusText.str();
            ImGui::SetNextWindowPos(ImVec2(12.0F, 12.0F), ImGuiCond_Always);
            ImGui::SetNextWindowBgAlpha(0.72F);
            constexpr ImGuiWindowFlags statusWindowFlags =
                ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_NoNav |
                ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_AlwaysAutoResize;
            ImGui::Begin("OpenBus status", nullptr, statusWindowFlags);
            ImGui::TextUnformatted(currentStatus.c_str());
            if (trafficSplineDebugOverlay_) {
                const std::size_t roadSplineCount = static_cast<std::size_t>(std::count_if(
                    trafficSplineDebugLabels_.begin(), trafficSplineDebugLabels_.end(),
                    [](const TrafficSplineDebugLabel& label) {
                        return label.pathTypeCode == openbus::map::MAP_SPLINE_PATH_TYPE_ROAD;
                    }));
                const std::size_t humanSplineCount = trafficSplineDebugLabels_.size() -
                                                     roadSplineCount;
                const std::size_t signalPathCount = static_cast<std::size_t>(std::count_if(
                    activeSceneryTrafficSegments.begin(), activeSceneryTrafficSegments.end(),
                    [](const ActiveSceneryTrafficSegment& activeSegment) {
                        return activeSegment.segment->trafficSignal.has_value();
                    }));
                ImGui::Text("F2 paths: road splines=%zu, human splines=%zu, SCO paths=%zu, "
                            "signal paths=%zu",
                            roadSplineCount, humanSplineCount, activeSceneryTrafficSegments.size(),
                            signalPathCount);
                std::ostringstream splineIds;
                splineIds << "Spline IDs:";
                std::size_t shownSplineIds = 0;
                for (const TrafficSplineDebugLabel& label : trafficSplineDebugLabels_) {
                    if (shownSplineIds == 12) {
                        break;
                    }
                    splineIds << ' '
                              << (label.pathTypeCode == openbus::map::MAP_SPLINE_PATH_TYPE_HUMAN
                                      ? 'H'
                                      : 'S')
                              << label.splineId;
                    ++shownSplineIds;
                }
                if (shownSplineIds < trafficSplineDebugLabels_.size()) {
                    splineIds << " +" << trafficSplineDebugLabels_.size() - shownSplineIds;
                }
                ImGui::TextUnformatted(splineIds.str().c_str());
                ImGui::TextUnformatted("ROAD PRIORITY");
                ImGui::SameLine();
                constexpr std::array<double, 5> priorityLegendValues = {0.0, 64.0, 128.0, 192.0,
                                                                        255.0};
                for (const double priority : priorityLegendValues) {
                    const std::array<float, 3> color = trafficPriorityDebugColor(priority);
                    ImGui::TextColored(ImVec4(color[0], color[1], color[2], 1.0F), "%.0f",
                                       priority);
                    ImGui::SameLine();
                }
                ImGui::NewLine();
                ImGui::TextUnformatted("Path arrows = travel direction; white arrows = AI facing");
                ImGui::TextUnformatted("Green boxes = passenger queue slots (within 75 m)");
                ImGui::TextUnformatted("Amber = editor bus stop; cyan = editor passenger area");
            }
            ImGui::End();
        }

        if (trafficSplineDebugOverlay_ && mapRenderer_ != nullptr) {
            const openbus::rendering::TrafficSignalDebugStats signalStats =
                mapRenderer_->trafficSignalDebugStats();
            const ImGuiIO& io = ImGui::GetIO();
            ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x - 12.0F, 12.0F), ImGuiCond_Always,
                                    ImVec2(1.0F, 0.0F));
            ImGui::SetNextWindowSize(ImVec2(440.0F, 360.0F), ImGuiCond_Always);
            ImGui::SetNextWindowBgAlpha(0.82F);
            constexpr ImGuiWindowFlags signalWindowFlags =
                ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse |
                ImGuiWindowFlags_NoSavedSettings;
            ImGui::Begin("F2 traffic lights", nullptr, signalWindowFlags);
            ImGui::Text("Signal models: %zu active, %zu controller-driven, %zu without phase",
                        signalStats.activeSignalModels, signalStats.controllerDrivenModels,
                        signalStats.modelsWithoutPhase);
            ImGui::Text("Models by phase:");
            ImGui::SameLine();
            for (std::size_t phaseIndex = 0; phaseIndex < signalStats.modelsByPhase.size();
                 ++phaseIndex) {
                if (signalStats.modelsByPhase[phaseIndex] == 0) {
                    continue;
                }
                ImGui::Text("%zu:%zu", phaseIndex, signalStats.modelsByPhase[phaseIndex]);
                ImGui::SameLine();
            }
            ImGui::NewLine();
            ImGui::Separator();
            if (ImGui::BeginChild("F2 traffic light details", ImVec2(0.0F, 0.0F), false)) {
                for (const ActiveSceneryTrafficSegment& activeSegment :
                     activeSceneryTrafficSegments) {
                    const openbus::rendering::TrafficPathDebugSegment& segment =
                        *activeSegment.segment;
                    const std::size_t fallbackId = activeSegment.index + 1;
                    if (!segment.trafficSignal.has_value() ||
                        segment.pathTypeCode != openbus::map::MAP_SPLINE_PATH_TYPE_ROAD ||
                        segment.points.size() < 2) {
                        continue;
                    }
                    const auto lines =
                        trafficSignalDebugLabel(segment, fallbackId, simulation.simulationTime());
                    if (lines.empty()) {
                        continue;
                    }
                    const std::optional<int> currentState =
                        openbus::map::mapTrafficSignalStateAtTime(*segment.trafficSignal,
                                                                  simulation.simulationTime());
                    const ImVec4 phaseColor =
                        !currentState.has_value() || *currentState < 6 || *currentState > 8
                            ? ImVec4(1.0F, 0.2F, 0.16F, 1.0F)
                            : (*currentState >= 9 && *currentState <= 11
                                   ? ImVec4(1.0F, 0.75F, 0.15F, 1.0F)
                                   : ImVec4(0.2F, 1.0F, 0.3F, 1.0F));
                    ImGui::PushID(static_cast<int>(fallbackId));
                    ImGui::TextColored(phaseColor, "%s", lines.front().c_str());
                    for (std::size_t lineIndex = 1; lineIndex < lines.size(); ++lineIndex) {
                        ImGui::TextDisabled("  %s", lines[lineIndex].c_str());
                    }
                    ImGui::Separator();
                    ImGui::PopID();
                }
            }
            ImGui::EndChild();
            ImGui::End();

            ImDrawList* debugLabelDrawList = ImGui::GetForegroundDrawList();
            const ImVec2 displaySize = ImGui::GetIO().DisplaySize;
            const auto drawWorldLabel = [&](const std::array<double, 3>& worldPosition,
                                            const std::string& label, ImU32 textColor,
                                            ImU32 backgroundColor) {
                const std::optional<ImVec2> screenPosition = projectTrafficDebugLabel(
                    trafficCameraViewProjection_, worldPosition, displaySize);
                if (!screenPosition.has_value()) {
                    return;
                }
                const ImVec2 textSize = ImGui::CalcTextSize(label.c_str());
                const ImVec2 topLeft(screenPosition->x - textSize.x * 0.5F - 4.0F,
                                     screenPosition->y - textSize.y - 4.0F);
                const ImVec2 bottomRight(screenPosition->x + textSize.x * 0.5F + 4.0F,
                                         screenPosition->y + 2.0F);
                debugLabelDrawList->AddRectFilled(topLeft, bottomRight, backgroundColor, 3.0F);
                debugLabelDrawList->AddText(ImVec2(screenPosition->x - textSize.x * 0.5F,
                                                   screenPosition->y - textSize.y - 2.0F),
                                            textColor, label.c_str());
            };
            for (const TrafficSplineDebugLabel& label : trafficSplineDebugLabels_) {
                const char prefix =
                    label.pathTypeCode == openbus::map::MAP_SPLINE_PATH_TYPE_HUMAN ? 'H' : 'S';
                drawWorldLabel(label.position,
                               std::string(1, prefix) + std::to_string(label.splineId) +
                                   (label.hasBidirectionalPath ? " [BIDIR]" : ""),
                               IM_COL32(255, 255, 255, 255), IM_COL32(15, 30, 38, 210));
            }
            for (const ActiveSceneryTrafficSegment& activeSegment :
                 activeSceneryTrafficSegments) {
                const openbus::rendering::TrafficPathDebugSegment& segment =
                    *activeSegment.segment;
                if (!segment.trafficSignal.has_value() || segment.points.size() < 2) {
                    continue;
                }
                std::array<double, 3> labelPosition = segment.points[segment.points.size() / 2];
                labelPosition[2] += 2.2;
                const std::string label =
                    "J" + std::to_string(segment.mapObjectId.value_or(
                              static_cast<int>(activeSegment.index + 1))) +
                    " P" + std::to_string(segment.pathIndex) + " L" +
                    std::to_string(segment.trafficSignal->lightIndex);
                drawWorldLabel(labelPosition, label, IM_COL32(255, 255, 255, 255),
                               IM_COL32(32, 24, 16, 220));
            }
            for (const auto& [stopId, candidate] : stopCandidates) {
                const std::size_t waiting = scheduledPassengerQueues_.at(stopId);
                const auto initialQueue = scheduledPassengerInitialQueues_.find(stopId);
                const std::size_t initialCount =
                    initialQueue == scheduledPassengerInitialQueues_.end()
                        ? waiting
                        : std::max(initialQueue->second, waiting);
                const auto pose = queuePose(stopId, candidate.pose,
                                            waiting == 0
                                                ? 0
                                                : openbus::map::mapScheduledPassengerQueueSlot(
                                                      initialCount, waiting - 1));
                std::array<double, 3> labelPosition = pose.position;
                labelPosition[2] += 2.1;
                const std::string name = candidate.stop->name.empty()
                                             ? std::string("Unnamed stop")
                                             : candidate.stop->name;
                const std::string label = name + " [stop " + std::to_string(stopId) +
                                          "] waiting: " +
                                          std::to_string(waiting) +
                                          (authoredPassengerPositions_.contains(stopId)
                                               ? ""
                                               : " [fallback]");
                drawWorldLabel(labelPosition, label, IM_COL32(255, 255, 255, 255),
                               IM_COL32(22, 100, 45, 230));
            }
            for (const auto* object : activeEditorPassengerObjects) {
                std::array<double, 3> labelPosition = {};
                for (std::size_t index = 4; index < 8; ++index) {
                    for (std::size_t axis = 0; axis < 3; ++axis) {
                        labelPosition[axis] += object->bounds[index][axis] * 0.25;
                    }
                }
                labelPosition[2] += 0.3;
                std::string label = object->busStop ? "EDITOR BUS STOP" : "EDITOR PASSENGERS";
                if (object->objectId.has_value()) {
                    label += " #" + std::to_string(*object->objectId);
                }
                if (!object->name.empty()) {
                    label += " " + object->name;
                }
                if (object->passengerCabin) {
                    label += " (" + std::to_string(object->passengerPositions.size()) + " positions)";
                }
                drawWorldLabel(labelPosition, label, IM_COL32(255, 255, 255, 255),
                               object->busStop ? IM_COL32(115, 77, 12, 230)
                                               : IM_COL32(14, 89, 113, 230));
            }
            for (const TrafficVehicleState& state : trafficVehicles_) {
                if (!state.populationEnabled || state.vehicle == nullptr ||
                    !state.vehicle->mapTrafficTileActive) {
                    continue;
                }
                const auto pose = openbus::map::sampleMapTrafficActorPose(
                    state.actor, state.actor.distanceMeters);
                if (!pose.has_value()) {
                    continue;
                }
                const char* status = "GO";
                if (state.deadlocked) {
                    status = "DEAD";
                } else if (state.actor.blockReason != openbus::map::MapTrafficBlockReason::None) {
                    switch (state.actor.blockReason) {
                    case openbus::map::MapTrafficBlockReason::Leader:
                        status = "FOLLOW";
                        break;
                    case openbus::map::MapTrafficBlockReason::Vehicle:
                    case openbus::map::MapTrafficBlockReason::Obstacle:
                    case openbus::map::MapTrafficBlockReason::Collision:
                        status = "BLOCK";
                        break;
                    case openbus::map::MapTrafficBlockReason::CrossingReservation:
                        status = "RESV";
                        break;
                    case openbus::map::MapTrafficBlockReason::JunctionTraffic:
                        status = "YIELD";
                        break;
                    case openbus::map::MapTrafficBlockReason::Signal:
                        status = "SIGNAL";
                        break;
                    case openbus::map::MapTrafficBlockReason::None:
                        break;
                    }
                } else if (state.actor.speedMetersPerSecond <= 0.1 &&
                           state.actor.targetSpeedMetersPerSecond <= 0.1) {
                    status = "HOLD";
                }
                std::ostringstream vehicleLabel;
                vehicleLabel << status << ' ' << std::fixed << std::setprecision(0)
                             << state.actor.speedMetersPerSecond * 3.6 << " km/h";
                if (state.actor.blockingActor != nullptr) {
                    const auto* blocker = state.actor.blockingActor;
                    const auto blockerPose = openbus::map::sampleMapTrafficActorPose(
                        *blocker, blocker->distanceMeters);
                    if (blockerPose.has_value()) {
                        vehicleLabel << " BY S" << blockerPose->splineId;
                        const auto segment = std::upper_bound(
                            blocker->route.cumulativeEndDistances.begin(),
                            blocker->route.cumulativeEndDistances.end(),
                            blocker->distanceMeters);
                        const std::size_t segmentIndex = static_cast<std::size_t>(
                            std::distance(blocker->route.cumulativeEndDistances.begin(), segment));
                        if (segmentIndex < blocker->route.segments.size() &&
                            blocker->route.segments[segmentIndex].sceneryPathIndex.has_value()) {
                            vehicleLabel << " P"
                                         << *blocker->route.segments[segmentIndex].sceneryPathIndex;
                        }
                    } else {
                        vehicleLabel << " BY ACTOR";
                    }
                } else if (state.actor.blockingObstacleIndex.has_value()) {
                    vehicleLabel << " BY OBS#" << *state.actor.blockingObstacleIndex;
                }
                const bool isHot = state.vehicle == trafficDebugHotVehicle_;
                if (isHot) {
                    vehicleLabel << " HOT";
                }
                std::array<double, 3> labelPosition = pose->position;
                labelPosition[2] += std::max(state.vehicle->trafficHeightMeters, 2.0) + 0.5;
                drawWorldLabel(labelPosition, vehicleLabel.str(), IM_COL32(255, 255, 255, 255),
                               isHot ? IM_COL32(185, 72, 8, 235) : IM_COL32(20, 24, 30, 220));
            }
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
    if (imguiEnabled_) {
        drawOnFootUi();
        imguiLayer_->render();
    }
    glfwSwapBuffers(window_);
    if (renderFpsCapHz_ <= 0.0) {
        return;
    }
    const auto framePeriod = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(1.0 / renderFpsCapHz_));
    const auto now = std::chrono::steady_clock::now();
    if (!frameDeadlineInitialized_) {
        nextFrameDeadline_ = now;
        frameDeadlineInitialized_ = true;
    }
    nextFrameDeadline_ += framePeriod;
    if (nextFrameDeadline_ > now) {
        std::this_thread::sleep_until(nextFrameDeadline_);
    } else {
        nextFrameDeadline_ = now;
    }
}

void RenderLoop::captureViews(const BusSimulation& simulation, const World& world,
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
            draw(simulation, world);
            glFinish();
        }
        const std::filesystem::path path = directory / (std::string(view.name) + ".png");
        {
            TraceScope phase("capture", "RenderLoop::captureViews.save");
            if (!openbus::rendering::saveFramebufferPng(path, width, height)) {
                gameLog.Error("Failed to save screenshot: " + path.string());
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
