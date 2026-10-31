#include "app/RenderLoopInternal.h"
#include "app/traffic/PlayerTripPolicy.h"

namespace {

std::vector<openbus::app::PlayerTripPolicyTarget>
playerTripPolicyTargets(openbus::app::TrafficFleet& fleet) {
    std::vector<openbus::app::PlayerTripPolicyTarget> targets;
    for (auto& state : fleet.vehicles) {
        if (state.bus) {
            targets.push_back({*state.bus, state.workerGeneration});
        }
    }
    return targets;
}

} // namespace

void RenderLoop::SetPlayerTimetableSelection(
    std::optional<SpawnBusRequest::TimetableSelection> selection) {
    clearPlayerTripAiPolicy();
    const auto* trip =
        playerTimetable_.select(selection, passengerNetwork_.routes, passengerTimetableCourses_);
    if (trip == nullptr) {
        return;
    }
    playerPassengerServiceEnabled_ = true;
    applyPlayerTripAiPolicy(*trip, selection->driveAsAdditionalBus);
}

void RenderLoop::clearPlayerTripAiPolicy() {
    openbus::app::clearPlayerTripPolicy(playerTripPolicyTargets(trafficFleet_));
}

void RenderLoop::applyPlayerTripAiPolicy(const openbus::map::MapAiTimetableCourseTrip& trip,
                                         bool additionalBus) {
    openbus::app::applyPlayerTripPolicy(playerTripPolicyTargets(trafficFleet_), trip,
                                        additionalBus);
}

void RenderLoop::advancePlayerTimetableTrip() {
    if (!playerTimetable_.state) {
        return;
    }
    clearPlayerTripAiPolicy();
    const auto* trip =
        playerTimetable_.advance(passengerNetwork_.routes, passengerTimetableCourses_);
    if (trip != nullptr) {
        applyPlayerTripAiPolicy(*trip, playerTimetable_.state->selection.driveAsAdditionalBus);
        return;
    }
    playerPassengerServiceEnabled_ = false;
}
