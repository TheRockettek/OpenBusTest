#include "app/traffic/PlayerTimetableDiagnostics.h"
#include "core/Logger.h"

#include <algorithm>

namespace openbus::app {

void logPlayerTimetableCatalog(const openbus::app::PlayerTimetableCatalog& catalog,
                               const std::vector<openbus::app::PassengerStopRouteState>& routes,
                               const std::vector<openbus::map::MapAiTimetableCourse>& courses,
                               const std::vector<std::string>& mapKeys, const std::string& mapPath,
                               const std::string& hofPath, const openbus::config::HofData& hof,
                               const std::vector<std::string>& scriptErrors) {
    using openbus::app::PassengerHofMatchStatus;
    std::size_t matched = 0;
    std::size_t ambiguous = 0;
    std::size_t tooFew = 0;
    std::size_t noSequence = 0;
    std::size_t matchedWithLocatedStops = 0;
    std::size_t matchedMissingStops = 0;
    for (std::size_t index = 0; index < catalog.routes.size(); ++index) {
        const auto& route = catalog.routes[index];
        switch (route.status) {
        case PassengerHofMatchStatus::Matched:
            ++matched;
            if (routes[index].stopPoses.empty() ||
                std::ranges::any_of(routes[index].stopPoses,
                                    [](const auto& pose) { return pose.has_value(); })) {
                ++matchedWithLocatedStops;
            }
            matchedMissingStops += static_cast<std::size_t>(std::ranges::count_if(
                routes[index].stopPoses, [](const auto& pose) { return !pose; }));
            break;
        case PassengerHofMatchStatus::TooFewStops:
            ++tooFew;
            break;
        case PassengerHofMatchStatus::NoMatchingSequence:
            ++noSequence;
            break;
        case PassengerHofMatchStatus::AmbiguousLines:
            ++ambiguous;
            break;
        }
    }
    std::size_t selectableCourses = 0;
    std::size_t selectableTrips = 0;
    for (const auto& line : catalog.lines) {
        selectableCourses += line.courses.size();
        for (const auto& course : line.courses)
            selectableTrips += course.trips.size();
    }
    gameLog.Log("Player timetable catalog: map=" + mapPath + " HOF=" + hofPath +
                " map_scheduled_routes=" + std::to_string(mapKeys.size()) + " registered_routes=" +
                std::to_string(routes.size()) + " courses=" + std::to_string(courses.size()) +
                " HOF_routes=" + std::to_string(hof.routes.size()) + " HOF_stops=" +
                std::to_string(hof.busstops.size()) + " matched=" + std::to_string(matched) +
                " matched_with_located_stops=" + std::to_string(matchedWithLocatedStops) +
                " matched_missing_stops=" + std::to_string(matchedMissingStops) + " no_sequence=" +
                std::to_string(noSequence) + " ambiguous_lines=" + std::to_string(ambiguous) +
                " too_few_stops=" + std::to_string(tooFew) +
                " selectable_lines=" + std::to_string(catalog.lines.size()) +
                " selectable_courses=" + std::to_string(selectableCourses) +
                " selectable_trips=" + std::to_string(selectableTrips));
    if (selectableTrips != 0) {
        return;
    }
    gameLog.Warning("Player timetable has no selectable trips matching the selected HOF.");
    for (std::size_t index = 0; index < std::min<std::size_t>(scriptErrors.size(), 4); ++index)
        gameLog.Warning("Player timetable HOF script error: " + scriptErrors[index]);
    std::size_t unregistered = 0;
    std::string examples;
    for (const auto& key : mapKeys) {
        if (std::ranges::any_of(routes,
                                [&](const auto& route) { return route.timetableKey == key; })) {
            continue;
        }
        ++unregistered;
        if (unregistered <= 8)
            examples += (examples.empty() ? "" : ", ") + key;
    }
    if (unregistered != 0) {
        gameLog.Warning("Player timetable unregistered map routes=" + std::to_string(unregistered) +
                        " examples=" + examples);
    }
    for (std::size_t index = 0; index < std::min<std::size_t>(routes.size(), 12); ++index) {
        const auto& route = routes[index];
        const auto& result = catalog.routes[index];
        const char* status =
            result.status == PassengerHofMatchStatus::Matched          ? "matched"
            : result.status == PassengerHofMatchStatus::TooFewStops    ? "too-few-stops"
            : result.status == PassengerHofMatchStatus::AmbiguousLines ? "ambiguous-lines"
                                                                       : "no-matching-sequence";
        std::string stops;
        for (std::size_t stop = 0; stop < std::min<std::size_t>(route.stops.size(), 6); ++stop)
            stops += (stops.empty() ? "" : " | ") + route.stops[stop].name;
        gameLog.Warning("Player timetable route: key=" + route.timetableKey +
                        " line=" + route.lineName + " destination=" + route.destinationName +
                        " stops=" + std::to_string(route.stops.size()) + " status=" + status +
                        " matching_sequences_seen=" + std::to_string(result.matchingSequences) +
                        " same_line_candidates_seen=" + std::to_string(result.sameLineCandidates) +
                        " first_stops=" + stops +
                        (result.status == PassengerHofMatchStatus::Matched
                             ? " HOF_code=" + result.hofCode + " HOF_line=" + result.hofLine
                             : ""));
        if (result.exampleSameLineRoute) {
            const auto& candidate = hof.routes[*result.exampleSameLineRoute];
            std::string candidateStops;
            for (std::size_t stop = 0;
                 stop < std::min<std::size_t>(candidate.busstopIdents.size(), 6); ++stop) {
                candidateStops +=
                    (candidateStops.empty() ? "" : " | ") + candidate.busstopIdents[stop];
            }
            gameLog.Warning(
                "Player timetable same-line HOF example: code=" + candidate.code + " name=" +
                candidate.name + " stops=" + std::to_string(candidate.busstopIdents.size()) +
                (candidate.busstopIdents.empty() ? ""
                                                 : " first_stops=" + candidateStops +
                                                       " last=" + candidate.busstopIdents.back()));
        }
    }
    for (std::size_t index = 0; index < std::min<std::size_t>(hof.routes.size(), 8); ++index) {
        const auto& route = hof.routes[index];
        gameLog.Warning("Player timetable HOF route: code=" + route.code + " line=" + route.line +
                        " name=" + route.name +
                        " stops=" + std::to_string(route.busstopIdents.size()) +
                        (route.busstopIdents.empty() ? ""
                                                     : " first=" + route.busstopIdents.front() +
                                                           " last=" + route.busstopIdents.back()));
    }
}

} // namespace openbus::app
