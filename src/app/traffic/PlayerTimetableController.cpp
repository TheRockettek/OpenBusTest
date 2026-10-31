#include "app/traffic/PlayerTimetableController.h"

#include <algorithm>
#include <cctype>
#include <stdexcept>

namespace openbus::app {

namespace {

std::string normalizedHofName(const std::string& value) {
    std::string result;
    for (const unsigned char character : value) {
        if (!std::isspace(character)) {
            result.push_back(static_cast<char>(std::tolower(character)));
        }
    }
    return result;
}

std::string normalizedLine(const std::string& value) {
    std::string result = normalizedHofName(value);
    if (!result.empty() && std::ranges::all_of(result, [](unsigned char character) {
            return std::isdigit(character);
        })) {
        const auto first = result.find_first_not_of('0');
        return first == std::string::npos ? "0" : result.substr(first);
    }
    return result;
}

} // namespace

bool passengerRouteMatchesHof(const PassengerStopRouteState& route,
                              const openbus::config::HofData& hof) {
    return passengerRouteHofRoute(route, hof) != nullptr;
}

const openbus::config::HofRoute* passengerRouteHofRoute(const PassengerStopRouteState& route,
                                                        const openbus::config::HofData& hof) {
    if (route.stops.size() < 2) {
        return nullptr;
    }
    const auto matchesStops = [&](const openbus::config::HofRoute& candidate) {
        if (candidate.busstopIdents.size() < 2) {
            return false;
        }
        const auto matchesStop = [&](const std::string& stopIdent,
                                     const openbus::map::MapTrafficPassengerStop& mapStop) {
            const std::string ident = normalizedHofName(stopIdent);
            if (ident.empty()) {
                return false;
            }
            const auto busstop =
                std::ranges::find_if(hof.busstops, [&](const openbus::config::HofBusstop& stop) {
                    return normalizedHofName(stop.ident) == ident;
                });
            const std::string stopName = normalizedHofName(mapStop.name);
            return !stopName.empty() &&
                   (stopName == ident ||
                    (busstop != hof.busstops.end() &&
                     std::ranges::any_of(busstop->strings, [&](const std::string& name) {
                         return !name.empty() && normalizedHofName(name) == stopName;
                     })));
        };
        std::size_t routeStopIndex = 0;
        const bool hofSequenceInMapRoute =
            std::ranges::all_of(candidate.busstopIdents, [&](const std::string& stopIdent) {
                const auto mapStop =
                    std::find_if(route.stops.begin() + static_cast<std::ptrdiff_t>(routeStopIndex),
                                 route.stops.end(), [&](const auto& mapStop) {
                                     return matchesStop(stopIdent, mapStop);
                                 });
                if (mapStop == route.stops.end()) {
                    return false;
                }
                routeStopIndex =
                    static_cast<std::size_t>(std::distance(route.stops.begin(), mapStop)) + 1;
                return true;
            });
        if (hofSequenceInMapRoute) {
            return true;
        }

        std::size_t hofStopIndex = 0;
        return std::ranges::all_of(route.stops, [&](const auto& mapStop) {
            const auto hofStop = std::find_if(
                candidate.busstopIdents.begin() + static_cast<std::ptrdiff_t>(hofStopIndex),
                candidate.busstopIdents.end(),
                [&](const std::string& stopIdent) { return matchesStop(stopIdent, mapStop); });
            if (hofStop == candidate.busstopIdents.end()) {
                return false;
            }
            hofStopIndex =
                static_cast<std::size_t>(std::distance(candidate.busstopIdents.begin(), hofStop)) +
                1;
            return true;
        });
    };
    const auto matchesDestination = [&](const openbus::config::HofRoute& candidate) {
        if (!candidate.busstopIdents.empty() || candidate.line.empty() ||
            normalizedLine(candidate.line) != normalizedLine(route.lineName)) {
            return false;
        }
        std::vector<std::string> destinations{normalizedHofName(route.destinationName)};
        const std::size_t separator = route.destinationName.find_first_of(" \t");
        if (separator != std::string::npos &&
            normalizedLine(route.destinationName.substr(0, separator)) ==
                normalizedLine(route.lineName)) {
            destinations.push_back(normalizedHofName(route.destinationName.substr(separator + 1)));
        }
        const auto matchesDestinationName = [&](const std::string& name) {
            const std::string normalizedName = normalizedHofName(name);
            return !normalizedName.empty() &&
                   std::ranges::find(destinations, normalizedName) != destinations.end();
        };
        if (matchesDestinationName(candidate.name)) {
            return true;
        }
        if (!candidate.hasTerminusIndex) {
            return false;
        }
        const auto terminus = std::ranges::find(hof.termini, candidate.terminusIndex,
                                                &openbus::config::HofTerminus::code);
        return terminus != hof.termini.end() &&
               std::ranges::any_of(terminus->strings, matchesDestinationName);
    };
    const openbus::config::HofRoute* match = nullptr;
    bool ambiguousLine = false;
    for (const auto& candidate : hof.routes) {
        if (matchesDestination(candidate)) {
            return &candidate;
        }
        if (!matchesStops(candidate)) {
            continue;
        }
        if (normalizedLine(candidate.line) == normalizedLine(route.lineName)) {
            return &candidate;
        }
        if (match && normalizedLine(match->line) != normalizedLine(candidate.line)) {
            ambiguousLine = true;
        }
        if (!match) {
            match = &candidate;
        }
    }
    // Passenger-only map timetables use their trip key, not a public line number.
    return ambiguousLine ? nullptr : match;
}

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
                                   const std::vector<openbus::map::MapAiTimetableCourse>& courses,
                                   const openbus::config::HofData* hof) {
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
        if (!route.stops.empty() && (!hof || passengerRouteMatchesHof(route, *hof)) &&
            std::ranges::find(availableTimetableKeys, route.timetableKey) ==
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
                return candidate.timetableKey == trip.timetableKey && !candidate.stops.empty() &&
                       (!hof || passengerRouteMatchesHof(candidate, *hof));
            });
        if (route != routes.end()) {
            current.selection.courseTripIndex = *nextTripIndex;
            current.selection.passengerRouteIndex =
                static_cast<std::size_t>(std::distance(routes.begin(), route));
            current.selection.firstStopIndex = 0;
            current.nextStopIndex = 0;
            current.lineName = route->lineName;
            if (hof) {
                current.lineName = passengerRouteHofRoute(*route, *hof)->line;
            }
            return &trip;
        }
    }
    state.reset();
    return nullptr;
}

} // namespace openbus::app
