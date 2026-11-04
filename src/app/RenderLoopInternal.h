#pragma once

// Shared includes and tuning constants for the RenderLoop implementation files.

#include "app/RenderLoop.h"
#include "app/ScriptTickScheduler.h"
#include "app/traffic/PassengerApproach.h"
#include "app/traffic/PassengerCabinPath.h"
#include "app/vehicle/Vehicle.h"
#include "input/VehicleKeyBindings.h"

#ifndef GLFW_INCLUDE_NONE
#define GLFW_INCLUDE_NONE
#endif
#include <GLFW/glfw3.h>

#include <algorithm>
#include <cmath>
#include <numbers>

namespace openbus::app {

inline constexpr double DEFAULT_FIELD_OF_VIEW = 60.0;
inline constexpr double MIN_FIELD_OF_VIEW = 20.0;
inline constexpr double MAX_FIELD_OF_VIEW = 120.0;
inline constexpr double GROUND_SHADOW_OFFSET_Z = 0.015;
inline constexpr double PLAYER_TIMETABLE_STOP_VISIT_RADIUS_METERS = 50.0;
inline constexpr double PLAYER_PASSENGER_STOP_ARRIVAL_RADIUS_METERS =
    playerPassengerStopArrivalRadiusMeters;
inline constexpr double PLAYER_PASSENGER_STOP_RELEASE_RADIUS_METERS =
    playerPassengerStopReleaseRadiusMeters;

inline std::optional<openbus::map::MapTrafficPose>
passengerConfiguredDoorQueuePose(const Vehicle& bus, const openbus::map::MapTrafficPose& pose,
                                 bool leftHandTraffic, const std::vector<int>& doors,
                                 std::optional<std::size_t> doorIndex, std::size_t queueSlot) {
    if (!doorIndex || *doorIndex >= doors.size()) {
        return std::nullopt;
    }
    const auto& cabin = bus.passengers->configuration();
    const int doorId = doors[*doorIndex];
    const auto point = std::find_if(cabin.passengerPathPoints.begin(),
                                    cabin.passengerPathPoints.end(),
                                    [doorId](const auto& candidate) {
                                        return candidate.index == doorId;
                                    });
    if (point == cabin.passengerPathPoints.end()) {
        return std::nullopt;
    }
    if (std::abs(point->position[0]) > 1.0e-6) {
        leftHandTraffic = point->position[0] < 0.0;
    }
    return passengerDoorQueuePose(pose, point->position[1], bus.trafficWidthMeters, leftHandTraffic,
                                  queueSlot);
}

inline openbus::map::MapTrafficPose
passengerVehicleLocalPose(const openbus::map::MapTrafficPose& busPose,
                          const std::array<double, 3>& localPosition) {
    auto pose = busPose;
    const double cosine = std::cos(busPose.headingRadians);
    const double sine = std::sin(busPose.headingRadians);
    pose.position[0] += localPosition[0] * cosine + localPosition[1] * sine;
    pose.position[1] += -localPosition[0] * sine + localPosition[1] * cosine;
    pose.position[2] += localPosition[2];
    return pose;
}

inline openbus::map::MapTrafficPose
playerPassengerDoorQueuePose(const Vehicle& bus, const BodyPose& chassis, bool leftHandTraffic,
                             std::size_t queueSlot) {
    openbus::map::MapTrafficPose pose;
    pose.position = transformLocalPoint(chassis, {0.0, 0.0, bus.modelOffsetZ});
    pose.headingRadians = std::atan2(chassis.rotation[0], chassis.rotation[3]);
    const auto configured = passengerConfiguredDoorQueuePose(
        bus, pose, leftHandTraffic, bus.passengers->configuration().passengerEntryPathPoints,
        bus.passengers->entryDoor(), queueSlot);
    return configured.value_or(passengerDoorQueuePose(
        pose, bus.trafficLengthMeters * 0.3, bus.trafficWidthMeters, leftHandTraffic, queueSlot));
}

inline openbus::map::MapTrafficPose passengerCurbPose(const openbus::map::MapTrafficPose& stopPose,
                                                      bool leftHandTraffic, double laneOffset,
                                                      double vehicleWidth) {
    const double passengerSide = openbus::map::mapTrafficPassengerSideSign(leftHandTraffic);
    const double curbOffset = openbus::map::mapTrafficPassengerCurbOffsetMeters(
        leftHandTraffic, laneOffset, vehicleWidth, 0.35);
    const auto offset = openbus::map::mapTrafficPassengerWorldOffset(
        leftHandTraffic, stopPose.headingRadians, curbOffset + passengerSide * 1.8, 0.0);
    auto pose = stopPose;
    pose.position[0] += offset[0];
    pose.position[1] += offset[1];
    pose.headingRadians -= passengerSide * std::numbers::pi * 0.5;
    return pose;
}

inline std::vector<std::array<double, 3>>
passengerAlightingPositions(const Vehicle& bus, const openbus::map::MapTrafficPose& busPose,
                            const openbus::map::MapTrafficPose& stopPose, bool leftHandTraffic,
                            double laneOffset, double vehicleWidth) {
    const auto fallback = passengerCurbPose(stopPose, leftHandTraffic, laneOffset, vehicleWidth);
    std::vector<std::array<double, 3>> positions;
    positions.reserve(bus.passengers->exitDoorsBySeat.size());
    for (const auto door : bus.passengers->exitDoorsBySeat) {
        positions.push_back(passengerConfiguredDoorQueuePose(
                                bus, busPose, leftHandTraffic,
                                bus.passengers->configuration().passengerExitPathPoints, door, 0)
                                .value_or(fallback)
                                .position);
    }
    return positions;
}

inline double mapTrafficSignalClockSeconds(double elapsedSeconds, double startHour) noexcept {
    return startHour * 3600.0 + elapsedSeconds;
}

} // namespace openbus::app

using namespace openbus::app;
using openbus::input::findVehicleKeyBinding;
using openbus::input::kBindingAlt;
using openbus::input::kBindingControl;
using openbus::input::kBindingHeld;
using openbus::input::kBindingShift;
using openbus::input::vehicleBindingPressed;
using openbus::input::VehicleKeyBinding;
using openbus::input::vehicleKeyBindings;
