#include "app/traffic/PlayerTimetableController.h"

#include <algorithm>
#include <stdexcept>

namespace openbus::app {

const openbus::map::MapAiTimetableCourseTrip*
PlayerTimetableController::select(std::optional<PlayerTimetableSelection> selection,
                                  const std::vector<PassengerStopRouteState>& routes,
                                  const std::vector<openbus::map::MapAiTimetableCourse>& courses) {
    state.reset();
    if (!selection) {
        return nullptr;
    }
    if (selection->passengerRouteIndex >= routes.size() ||
        selection->courseIndex >= courses.size()) {
        throw std::out_of_range("Player timetable selection is outside the available catalog");
    }
    const auto& course = courses[selection->courseIndex];
    const auto& route = routes[selection->passengerRouteIndex];
    if (selection->courseTripIndex >= course.trips.size() ||
        selection->firstStopIndex >= route.stops.size()) {
        throw std::out_of_range("Player timetable selection has an invalid trip or first stop");
    }
    const auto& trip = course.trips[selection->courseTripIndex];
    if (trip.timetableKey != route.timetableKey) {
        throw std::invalid_argument("Player timetable trip does not match its passenger route");
    }
    state = PlayerTimetableState{*selection, selection->firstStopIndex, route.lineName};
    return &trip;
}

const openbus::map::MapAiTimetableCourseTrip*
PlayerTimetableController::advance(const std::vector<PassengerStopRouteState>& routes,
                                   const std::vector<openbus::map::MapAiTimetableCourse>& courses) {
    if (!state) {
        return nullptr;
    }
    PlayerTimetableState& current = *state;
    if (current.stopAfterCurrentTrip || current.selection.courseIndex >= courses.size()) {
        state.reset();
        return nullptr;
    }
    const auto& course = courses[current.selection.courseIndex];
    std::vector<std::string> availableTimetableKeys;
    for (const PassengerStopRouteState& route : routes) {
        if (!route.stops.empty() && std::ranges::find(availableTimetableKeys, route.timetableKey) ==
                                        availableTimetableKeys.end()) {
            availableTimetableKeys.push_back(route.timetableKey);
        }
    }
    const auto nextTripIndex = openbus::map::mapAiTimetableNextCourseTripIndex(
        course, current.selection.courseTripIndex, availableTimetableKeys);
    if (nextTripIndex) {
        const auto& trip = course.trips[*nextTripIndex];
        const auto route =
            std::ranges::find_if(routes, [&](const PassengerStopRouteState& candidate) {
                return candidate.timetableKey == trip.timetableKey && !candidate.stops.empty();
            });
        if (route != routes.end()) {
            current.selection.courseTripIndex = *nextTripIndex;
            current.selection.passengerRouteIndex =
                static_cast<std::size_t>(std::distance(routes.begin(), route));
            current.selection.firstStopIndex = 0;
            current.nextStopIndex = 0;
            current.lineName = route->lineName;
            return &trip;
        }
    }
    state.reset();
    return nullptr;
}

} // namespace openbus::app
