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
    if (selection && (playerVehicle_ == nullptr || !playerVehicle_->scripts ||
                      selection->passengerRouteIndex >= passengerNetwork_.routes.size() ||
                      !openbus::app::passengerRouteMatchesHof(
                          passengerNetwork_.routes[selection->passengerRouteIndex],
                          playerVehicle_->scripts->hofData()))) {
        throw std::invalid_argument("Player timetable route does not match the spawned bus's HOF");
    }
    clearPlayerTripAiPolicy();
    if (playerVehicle_ && playerVehicle_->passengers && activePlayerPassengerStop_) {
        playerVehicle_->passengers->service.endServiceAtCurrentStop();
    }
    // Keep the mapped stop alive while riders finish exiting, but do not let
    // that old stop advance a newly selected timetable trip.
    activePlayerPassengerStopCompletesTimetable_ = false;
    if (!activePlayerPassengerStop_) {
        playerPassengerStopLatch_.reset();
        playerPassengerStopLatchRouteIndex_.reset();
        playerPassengerStopLatchIndex_.reset();
    }
    playerPassengerServiceEnabled_ = selection.has_value();
    const auto* trip =
        playerTimetable_.select(selection, passengerNetwork_.routes, passengerTimetableCourses_);
    if (trip == nullptr) {
        return;
    }
    playerPassengerServiceEnabled_ = true;
    playerTimetable_.state->lineName = openbus::app::passengerRouteHofRoute(
                                           passengerNetwork_.routes[selection->passengerRouteIndex],
                                           playerVehicle_->scripts->hofData())
                                           ->line;
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
    const auto* trip = playerTimetable_.advance(
        passengerNetwork_.routes, passengerTimetableCourses_, &playerVehicle_->scripts->hofData());
    if (trip != nullptr) {
        applyPlayerTripAiPolicy(*trip, playerTimetable_.state->selection.driveAsAdditionalBus);
        return;
    }
    playerPassengerServiceEnabled_ = false;
}
