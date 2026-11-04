#include "app/traffic/PlayerTimetableController.h"

#include <algorithm>
#include <cctype>
#include <stdexcept>
#include <string_view>
#include <unordered_map>
#include <unordered_set>

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

std::string normalizedStopName(std::string value) {
    if (const auto suffix = value.find('*'); suffix != std::string::npos)
        value.erase(suffix);
    while (!value.empty() && std::isspace(static_cast<unsigned char>(value.back())))
        value.pop_back();
    const auto terminusSeparator = value.find_last_of(" \t");
    if (terminusSeparator != std::string::npos &&
        normalizedHofName(value.substr(terminusSeparator + 1)) == "terminus") {
        value.erase(terminusSeparator);
        while (!value.empty() && std::isspace(static_cast<unsigned char>(value.back())))
            value.pop_back();
    }
    const auto lastSpace = value.find_last_of(" \t");
    if (lastSpace != std::string::npos && value.size() - lastSpace <= 3 &&
        std::ranges::all_of(value.substr(lastSpace + 1),
                            [](unsigned char character) { return std::isdigit(character); })) {
        value.erase(lastSpace);
    }

    std::string result;
    for (std::size_t index = 0; index < value.size(); ++index) {
        unsigned char character = static_cast<unsigned char>(value[index]);
        if (character == 0xc3 && index + 1 < value.size()) {
            const unsigned char next = static_cast<unsigned char>(value[index + 1]);
            if (next >= 0x80 && next <= 0xbf) {
                character = static_cast<unsigned char>(next + 0x40);
                ++index;
            }
        }
        switch (character) {
        case 0xc4:
        case 0xe4:
            result += "ae";
            break;
        case 0xd6:
        case 0xf6:
            result += "oe";
            break;
        case 0xdc:
        case 0xfc:
            result += "ue";
            break;
        case 0xdf:
            result += "ss";
            break;
        default:
            if (character < 0x80 && std::isalnum(character))
                result.push_back(static_cast<char>(std::tolower(character)));
            else if (character >= 0x80)
                result.push_back(static_cast<char>(character));
        }
    }
    constexpr std::string_view terminus = "terminus";
    if (result.size() > terminus.size() && result.ends_with(terminus))
        result.erase(result.size() - terminus.size());
    return result;
}

struct PreparedHofStop {
    std::string ident;
    std::vector<std::string> names;
    std::vector<std::string> relaxedNames;
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
                const auto terminus = std::ranges::find(data.termini, route.terminusIndex,
                                                        &openbus::config::HofTerminus::code);
                if (terminus != data.termini.end()) {
                    for (const auto& name : terminus->strings)
                        prepared.terminusNames.push_back(normalizedHofName(name));
                }
            }
            prepared.stops.reserve(route.busstopIdents.size());
            for (const auto& ident : route.busstopIdents) {
                PreparedHofStop stop;
                stop.ident = normalizedHofName(ident);
                stop.relaxedNames.push_back(normalizedStopName(ident));
                auto found = stopsByIdent.find(stop.ident);
                if (found == stopsByIdent.end()) {
                    const auto suffix = stop.ident.find('*');
                    if (suffix != std::string::npos)
                        found = stopsByIdent.find(stop.ident.substr(0, suffix));
                }
                if (found != stopsByIdent.end()) {
                    for (const auto& name : found->second->strings) {
                        stop.names.push_back(normalizedHofName(name));
                        stop.relaxedNames.push_back(normalizedStopName(name));
                    }
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

std::size_t matchingPlayerTimetableRouteCount(
    const std::vector<PassengerStopRouteState>& routes, const openbus::config::HofData& hof) {
    const PreparedHof prepared(hof);
    std::unordered_set<std::string> matchedTimetables;
    for (const PassengerStopRouteState& route : routes) {
        if (!route.stops.empty() &&
            analyzePreparedPassengerRouteHof(route, prepared).route != nullptr) {
            matchedTimetables.insert(route.timetableKey);
        }
    }
    return matchedTimetables.size();
}

std::string passengerRouteDisplayLine(const PassengerStopRouteState& route,
                                      const openbus::config::HofRoute& hofRoute) {
    if (normalizedLine(hofRoute.line) == "xxx" && !route.lineName.empty() &&
        normalizedLine(route.lineName) != normalizedLine(route.timetableKey)) {
        return route.lineName;
    }
    return hofRoute.line;
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
        if (candidate.stops.size() == 2 && mapStops.size() > 2 &&
            (!matchesStop(candidate.stops.front(), mapStops.front()) ||
             !matchesStop(candidate.stops.back(), mapStops.back()))) {
            return false;
        }
        std::size_t routeStopIndex = 0;
        const bool hofSequenceInMapRoute =
            std::ranges::all_of(candidate.stops, [&](const PreparedHofStop& stop) {
                const auto mapStop = std::find_if(
                    mapStops.begin() + static_cast<std::ptrdiff_t>(routeStopIndex), mapStops.end(),
                    [&](const auto& mapStop) { return matchesStop(stop, mapStop); });
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
            const auto hofStop =
                std::find_if(candidate.stops.begin() + static_cast<std::ptrdiff_t>(hofStopIndex),
                             candidate.stops.end(), [&](const PreparedHofStop& stop) {
                                 return matchesStop(stop, mapStop);
                             });
            if (hofStop == candidate.stops.end()) {
                return false;
            }
            hofStopIndex =
                static_cast<std::size_t>(std::distance(candidate.stops.begin(), hofStop)) + 1;
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
    if (!match && !mapLine.empty() && mapLine != normalizedLine(route.timetableKey) &&
        mapStops.size() >= 4) {
        std::vector<std::string> relaxedMapStops;
        relaxedMapStops.reserve(route.stops.size());
        for (std::size_t index = 0; index < route.stops.size(); ++index) {
            std::string name = route.stops[index].name;
            if (index == 0 || index + 1 == route.stops.size()) {
                const auto separator = name.find_last_of(" \t");
                if (separator != std::string::npos &&
                    normalizedLine(name.substr(separator + 1)) == mapLine)
                    name.erase(separator);
            }
            relaxedMapStops.push_back(normalizedStopName(std::move(name)));
        }
        std::size_t bestMatches = 0;
        std::size_t bestLength = 1;
        for (std::size_t index = 0; index < hof.routes.size(); ++index) {
            const auto& candidate = hof.routes[index];
            if (candidate.stops.size() < 4 ||
                (candidate.line != mapLine && candidate.line != "xxx")) {
                continue;
            }
            const auto matchesStop = [&](std::size_t mapIndex, std::size_t hofIndex) {
                const auto& mapName = relaxedMapStops[mapIndex];
                const auto& names = candidate.stops[hofIndex].relaxedNames;
                if (mapName.empty())
                    return false;
                if (std::ranges::find(names, mapName) != names.end())
                    return true;
                if (mapIndex + 1 == relaxedMapStops.size() &&
                    hofIndex + 1 == candidate.stops.size()) {
                    if (std::ranges::find(names, "terminus") != names.end() &&
                        normalizedHofName(route.stops.values()[mapIndex].name)
                            .ends_with("terminus"))
                        return true;
                    const auto separator = route.stops.values()[mapIndex].name.find('/');
                    if (separator != std::string::npos) {
                        const std::string prefix = normalizedStopName(
                            route.stops.values()[mapIndex].name.substr(0, separator));
                        return prefix.size() >= 8 &&
                               std::ranges::find(names, prefix) != names.end();
                    }
                }
                return false;
            };
            const std::size_t lastMapStop = relaxedMapStops.size() - 1;
            const std::size_t lastHofStop = candidate.stops.size() - 1;
            const auto isTerminalVariant = [&](std::size_t mapIndex, std::size_t adjacentIndex) {
                const auto& adjacent = relaxedMapStops[adjacentIndex];
                return normalizedHofName(route.stops[mapIndex].name).find("terminus") !=
                           std::string::npos ||
                       (adjacent.size() >= 8 && relaxedMapStops[mapIndex].starts_with(adjacent));
            };
            const bool startsAtHofStop =
                matchesStop(0, 0) || (isTerminalVariant(0, 1) && matchesStop(1, 0));
            const bool endsAtHofStop = matchesStop(lastMapStop, lastHofStop) ||
                                       (isTerminalVariant(lastMapStop, lastMapStop - 1) &&
                                        matchesStop(lastMapStop - 1, lastHofStop));
            if (!startsAtHofStop || !endsAtHofStop)
                continue;
            std::vector<std::size_t> previous(candidate.stops.size() + 1);
            std::vector<std::size_t> current(candidate.stops.size() + 1);
            for (std::size_t mapIndex = 0; mapIndex < relaxedMapStops.size(); ++mapIndex) {
                for (std::size_t hofIndex = 0; hofIndex < candidate.stops.size(); ++hofIndex) {
                    current[hofIndex + 1] =
                        matchesStop(mapIndex, hofIndex)
                            ? previous[hofIndex] + 1
                            : std::max(previous[hofIndex + 1], current[hofIndex]);
                }
                std::swap(previous, current);
            }
            const std::size_t count = previous.back();
            const std::size_t shorter = std::min(relaxedMapStops.size(), candidate.stops.size());
            const std::size_t longer = std::max(relaxedMapStops.size(), candidate.stops.size());
            if (count < 4 || count * 5 < shorter * 4 || count * 20 < longer * 11)
                continue;
            ++matchingSequences;
            if (!match || count * bestLength > bestMatches * longer ||
                (count * bestLength == bestMatches * longer && candidate.line == mapLine &&
                 normalizedLine(match->line) != mapLine)) {
                match = &hof.source.routes[index];
                bestMatches = count;
                bestLength = longer;
            }
        }
    }
    return {match,
            match ? PassengerHofMatchStatus::Matched : PassengerHofMatchStatus::NoMatchingSequence,
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
        const std::string line =
            match.route ? passengerRouteDisplayLine(passengerRoute, *match.route) : std::string{};
        newRoutes.push_back({match.status, match.matchingSequences, match.sameLineCandidates,
                             match.exampleSameLineRoute, line,
                             match.route ? match.route->code : std::string{}});
        if (match.route) {
            routeByLineAndKey[line].try_emplace(passengerRoute.timetableKey, newRoutes.size() - 1);
        }
        if (match.route && std::ranges::find(newLines, line, &PlayerTimetableCatalogLine::name) ==
                               newLines.end()) {
            newLines.push_back({line, {}});
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
            std::ranges::stable_sort(course.trips, [&](const PlayerTimetableCatalogTrip& first,
                                                       const PlayerTimetableCatalogTrip& second) {
                return courses[courseIndex].trips[first.tripIndex].departureMinutes <
                       courses[courseIndex].trips[second.tripIndex].departureMinutes;
            });
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
                current.lineName =
                    passengerRouteDisplayLine(*route, *passengerRouteHofRoute(*route, *hof));
            }
            return &trip;
        }
    }
    state.reset();
    return nullptr;
}

} // namespace openbus::app
