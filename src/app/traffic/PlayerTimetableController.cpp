#include "app/traffic/PlayerTimetableController.h"

#include <algorithm>
#include <cctype>
#include <stdexcept>
#include <unordered_map>

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

struct PreparedHofStop {
    std::string ident;
    std::vector<std::string> names;
};

struct PreparedHofRoute {
    std::string line;
    std::string name;
    std::vector<std::string> terminusNames;
    std::vector<PreparedHofStop> stops;
};

struct PreparedHof {
    explicit PreparedHof(const openbus::config::HofData& data) : source(data) {
        std::unordered_map<std::string, const openbus::config::HofBusstop*> stopsByIdent;
        for (const auto& stop : data.busstops)
            stopsByIdent.try_emplace(normalizedHofName(stop.ident), &stop);
        routes.reserve(data.routes.size());
        for (const auto& route : data.routes) {
            PreparedHofRoute prepared;
            prepared.line = normalizedLine(route.line);
            prepared.name = normalizedHofName(route.name);
            if (route.hasTerminusIndex) {
                const auto terminus = std::ranges::find(
                    data.termini, route.terminusIndex, &openbus::config::HofTerminus::code);
                if (terminus != data.termini.end()) {
                    for (const auto& name : terminus->strings)
                        prepared.terminusNames.push_back(normalizedHofName(name));
                }
            }
            prepared.stops.reserve(route.busstopIdents.size());
            for (const auto& ident : route.busstopIdents) {
                PreparedHofStop stop;
                stop.ident = normalizedHofName(ident);
                const auto found = stopsByIdent.find(stop.ident);
                if (found != stopsByIdent.end()) {
                    for (const auto& name : found->second->strings)
                        stop.names.push_back(normalizedHofName(name));
                }
                prepared.stops.push_back(std::move(stop));
            }
            routes.push_back(std::move(prepared));
        }
    }

    const openbus::config::HofData& source;
    std::vector<PreparedHofRoute> routes;
};

PassengerHofMatch analyzePreparedPassengerRouteHof(const PassengerStopRouteState& route,
                                                   const PreparedHof& hof);

} // namespace

bool passengerRouteMatchesHof(const PassengerStopRouteState& route,
                              const openbus::config::HofData& hof) {
    return passengerRouteHofRoute(route, hof) != nullptr;
}

const openbus::config::HofRoute* passengerRouteHofRoute(const PassengerStopRouteState& route,
                                                        const openbus::config::HofData& hof) {
    return analyzePassengerRouteHof(route, hof).route;
}

PassengerHofMatch analyzePassengerRouteHof(const PassengerStopRouteState& route,
                                           const openbus::config::HofData& hof) {
    return analyzePreparedPassengerRouteHof(route, PreparedHof(hof));
}

namespace {

PassengerHofMatch analyzePreparedPassengerRouteHof(const PassengerStopRouteState& route,
                                                   const PreparedHof& hof) {
    if (route.stops.size() < 2) {
        return {nullptr, PassengerHofMatchStatus::TooFewStops, 0, 0, std::nullopt};
    }
    std::vector<std::string> mapStops;
    mapStops.reserve(route.stops.size());
    for (const auto& stop : route.stops)
        mapStops.push_back(normalizedHofName(stop.name));
    const std::string mapLine = normalizedLine(route.lineName);
    const auto matchesStops = [&](const PreparedHofRoute& candidate) {
        if (candidate.stops.size() < 2) {
            return false;
        }
        const auto matchesStop = [](const PreparedHofStop& stop, const std::string& mapStop) {
            return !stop.ident.empty() && !mapStop.empty() &&
                   (mapStop == stop.ident ||
                    std::ranges::find(stop.names, mapStop) != stop.names.end());
        };
        std::size_t routeStopIndex = 0;
        const bool hofSequenceInMapRoute =
            std::ranges::all_of(candidate.stops, [&](const PreparedHofStop& stop) {
                const auto mapStop =
                    std::find_if(mapStops.begin() + static_cast<std::ptrdiff_t>(routeStopIndex),
                                 mapStops.end(), [&](const auto& mapStop) {
                                     return matchesStop(stop, mapStop);
                                 });
                if (mapStop == mapStops.end()) {
                    return false;
                }
                routeStopIndex =
                    static_cast<std::size_t>(std::distance(mapStops.begin(), mapStop)) + 1;
                return true;
            });
        if (hofSequenceInMapRoute) {
            return true;
        }

        std::size_t hofStopIndex = 0;
        return std::ranges::all_of(mapStops, [&](const auto& mapStop) {
            const auto hofStop = std::find_if(
                candidate.stops.begin() + static_cast<std::ptrdiff_t>(hofStopIndex),
                candidate.stops.end(),
                [&](const PreparedHofStop& stop) { return matchesStop(stop, mapStop); });
            if (hofStop == candidate.stops.end()) {
                return false;
            }
            hofStopIndex =
                static_cast<std::size_t>(std::distance(candidate.stops.begin(), hofStop)) +
                1;
            return true;
        });
    };
    const auto matchesDestination = [&](const PreparedHofRoute& candidate,
                                        const openbus::config::HofRoute& source) {
        if (!candidate.stops.empty() || source.line.empty() || candidate.line != mapLine) {
            return false;
        }
        std::vector<std::string> destinations{normalizedHofName(route.destinationName)};
        const std::size_t separator = route.destinationName.find_first_of(" \t");
        if (separator != std::string::npos &&
            normalizedLine(route.destinationName.substr(0, separator)) == mapLine) {
            destinations.push_back(normalizedHofName(route.destinationName.substr(separator + 1)));
        }
        const auto matchesDestinationName = [&](const std::string& normalizedName) {
            return !normalizedName.empty() &&
                   std::ranges::find(destinations, normalizedName) != destinations.end();
        };
        if (matchesDestinationName(candidate.name)) {
            return true;
        }
        return std::ranges::any_of(candidate.terminusNames, matchesDestinationName);
    };
    const openbus::config::HofRoute* match = nullptr;
    bool ambiguousLine = false;
    std::size_t matchingSequences = 0;
    std::size_t sameLineCandidates = 0;
    std::optional<std::size_t> exampleSameLineRoute;
    for (std::size_t index = 0; index < hof.routes.size(); ++index) {
        const auto& candidate = hof.routes[index];
        const auto& source = hof.source.routes[index];
        if (!source.line.empty() && !route.lineName.empty() && candidate.line == mapLine) {
            ++sameLineCandidates;
            if (!exampleSameLineRoute)
                exampleSameLineRoute = index;
        }
        if (matchesDestination(candidate, source)) {
            return {&source, PassengerHofMatchStatus::Matched, matchingSequences,
                    sameLineCandidates, exampleSameLineRoute};
        }
        if (!matchesStops(candidate)) {
            continue;
        }
        ++matchingSequences;
        if (candidate.line == mapLine) {
            return {&source, PassengerHofMatchStatus::Matched, matchingSequences,
                    sameLineCandidates, exampleSameLineRoute};
        }
        if (match && normalizedLine(match->line) != candidate.line) {
            ambiguousLine = true;
        }
        if (!match) {
            match = &source;
        }
    }
    // Passenger-only map timetables use their trip key, not a public line number.
    if (ambiguousLine) {
        return {nullptr, PassengerHofMatchStatus::AmbiguousLines, matchingSequences,
                sameLineCandidates, exampleSameLineRoute};
    }
    return {match, match ? PassengerHofMatchStatus::Matched
                         : PassengerHofMatchStatus::NoMatchingSequence,
            matchingSequences, sameLineCandidates, exampleSameLineRoute};
}

} // namespace

bool PlayerTimetableCatalog::update(const std::vector<PassengerStopRouteState>& passengerRoutes,
                                   const std::vector<openbus::map::MapAiTimetableCourse>& courses,
                                   const openbus::config::HofData& hof) {
    if (valid_) {
        return false;
    }
    std::vector<PlayerTimetableCatalogRoute> newRoutes;
    std::vector<PlayerTimetableCatalogLine> newLines;
    std::unordered_map<std::string, std::unordered_map<std::string, std::size_t>> routeByLineAndKey;
    const PreparedHof prepared(hof);
    newRoutes.reserve(passengerRoutes.size());
    for (const auto& passengerRoute : passengerRoutes) {
        const PassengerHofMatch match = analyzePreparedPassengerRouteHof(passengerRoute, prepared);
        newRoutes.push_back({match.status, match.matchingSequences, match.sameLineCandidates,
                             match.exampleSameLineRoute,
                             match.route ? match.route->line : std::string{},
                             match.route ? match.route->code : std::string{}});
        if (match.route) {
            routeByLineAndKey[match.route->line].try_emplace(passengerRoute.timetableKey,
                                                             newRoutes.size() - 1);
        }
        if (match.route &&
            std::ranges::find(newLines, match.route->line, &PlayerTimetableCatalogLine::name) ==
                newLines.end()) {
            newLines.push_back({match.route->line, {}});
        }
    }
    std::ranges::sort(newLines, {}, &PlayerTimetableCatalogLine::name);
    for (PlayerTimetableCatalogLine& line : newLines) {
        const auto& routeByKey = routeByLineAndKey.at(line.name);
        for (std::size_t courseIndex = 0; courseIndex < courses.size(); ++courseIndex) {
            PlayerTimetableCatalogCourse course{courseIndex, {}};
            for (std::size_t tripIndex = 0; tripIndex < courses[courseIndex].trips.size();
                 ++tripIndex) {
                const auto& trip = courses[courseIndex].trips[tripIndex];
                const auto route = routeByKey.find(trip.timetableKey);
                if (route != routeByKey.end())
                    course.trips.push_back({tripIndex, route->second});
            }
            if (!course.trips.empty()) {
                line.courses.push_back(std::move(course));
            }
        }
    }
    routes = std::move(newRoutes);
    lines = std::move(newLines);
    valid_ = true;
    return true;
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
