#pragma once

// Shared includes and tuning constants for the RenderLoop implementation files.

#include "app/RenderLoop.h"
#include "app/ScriptTickScheduler.h"
#include "app/traffic/PassengerApproach.h"
#include "app/vehicle/Vehicle.h"
#include "input/VehicleKeyBindings.h"

#ifndef GLFW_INCLUDE_NONE
#define GLFW_INCLUDE_NONE
#endif
#include <GLFW/glfw3.h>

#include <algorithm>
#include <cmath>

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

inline openbus::map::MapTrafficPose
playerPassengerDoorQueuePose(const Vehicle& bus, const BodyPose& chassis, bool leftHandTraffic,
                             std::size_t queueSlot) {
    openbus::map::MapTrafficPose pose;
    pose.position = transformLocalPoint(chassis, {0.0, 0.0, bus.modelOffsetZ});
    pose.headingRadians = std::atan2(chassis.rotation[0], chassis.rotation[3]);
    double entryForward = bus.trafficLengthMeters * 0.3;
    const auto& cabin = bus.passengers->configuration();
    if (!cabin.passengerEntryPathPoints.empty()) {
        const int entryId = cabin.passengerEntryPathPoints.front();
        const auto entry = std::find_if(
            cabin.passengerPathPoints.begin(), cabin.passengerPathPoints.end(),
            [entryId](const auto& point) { return point.index == entryId; });
        if (entry != cabin.passengerPathPoints.end()) {
            entryForward = entry->position[1];
            if (std::abs(entry->position[0]) > 1.0e-6) {
                leftHandTraffic = entry->position[0] < 0.0;
            }
        }
    }
    return passengerDoorQueuePose(pose, entryForward, bus.trafficWidthMeters, leftHandTraffic,
                                  queueSlot);
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
