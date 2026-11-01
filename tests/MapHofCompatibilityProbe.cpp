#include "app/traffic/PassengerStops.h"
#include "app/traffic/PlayerTimetableController.h"
#include "core/Variables.h"
#include "map/MapConfigLoader.h"
#include "scripting/ScriptRuntime.h"

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

int main(int argc, char** argv) {
    if (argc < 3 || argc > 5) {
        std::cerr << "usage: OpenBusMapHofCompatibilityProbe <map-directory> <hof-file> "
                     "[timetable-key|--catalog] [YYYYMMDD]\n";
        return 2;
    }

    try {
        const std::filesystem::path mapDirectory(argv[1]);
        const std::filesystem::path hofPath(argv[2]);
        openbus::map::MapDefinition map = openbus::map::loadMapDefinition(mapDirectory);
        const bool catalogMode = argc >= 4 && std::string(argv[3]) == "--catalog";
        if (argc == 5) {
            std::size_t parsed = 0;
            const std::string date(argv[4]);
            const int value = std::stoi(date, &parsed);
            if (parsed != date.size())
                throw std::invalid_argument("invalid Chrono timetable date");
            std::cout << "chrono_timetable_scenarios="
                      << openbus::map::applyMapChronoTimetables(map, value) << '\n';
        }

        VehicleConfig configuration;
        configuration.sourcePath = hofPath;
        openbus::scripting::Vehicle localState;
        localState.setString("hof_file", hofPath.string());
        SimulationState simulation;
        ScriptRuntime scripts(configuration, localState, simulation);
        const openbus::config::HofData& hof = scripts.hofData();
        if (hof.routes.empty()) {
            throw std::runtime_error("selected HOF contains no parsed timetable routes");
        }

        std::vector<std::string> matchedTimetables;
        std::vector<std::string> timetableSamples;
        std::vector<std::string> matchingLineSamples;
        std::vector<std::string> matchingHofLineSamples;
        std::vector<std::string> mapLines;
        std::vector<std::string> hofLines;
        std::vector<openbus::app::PassengerStopRouteState> passengerRoutes;
        openbus::app::PassengerNetwork passengerNetwork;
        passengerNetwork.registerTimetables(
            map.aiTraffic, 4, [](const auto&) -> std::optional<openbus::map::MapTrafficPose> {
                return openbus::map::MapTrafficPose{};
            });
        const auto scheduledKeys = openbus::app::scheduledPlayerTimetableKeys(map.aiTraffic);
        const std::size_t scheduledRoutes = scheduledKeys.size();
        const std::size_t routesWithPassengerStops = passengerNetwork.routes.size();
        for (const auto& passengerRoute : passengerNetwork.routes) {
            const auto& route = passengerRoute;
            const auto& stops = passengerRoute.stops;
            if (std::ranges::find(mapLines, route.lineName) == mapLines.end()) {
                mapLines.push_back(route.lineName);
            }
            if (timetableSamples.size() < 4) {
                std::string sample = route.timetableKey + " line=" + route.lineName +
                                     " destination=" + route.destinationName + " stops=";
                for (std::size_t index = 0; index < stops.size(); ++index) {
                    if (index > 0) {
                        sample += "|";
                    }
                    sample += stops[index].name;
                }
                timetableSamples.push_back(std::move(sample));
            }
            if (matchingLineSamples.size() < 4 &&
                std::ranges::any_of(hof.routes, [&](const openbus::config::HofRoute& hofRoute) {
                    return hofRoute.line == route.lineName;
                })) {
                std::string sample = route.timetableKey + " line=" + route.lineName +
                                     " destination=" + route.destinationName + " stops=";
                for (std::size_t index = 0; index < stops.size(); ++index) {
                    if (index > 0) {
                        sample += "|";
                    }
                    sample += stops[index].name;
                }
                matchingLineSamples.push_back(std::move(sample));
            }
            for (const openbus::config::HofRoute& hofRoute : hof.routes) {
                if (std::ranges::find(hofLines, hofRoute.line) == hofLines.end()) {
                    hofLines.push_back(hofRoute.line);
                }
                if (matchingHofLineSamples.size() < 6 && hofRoute.line == route.lineName) {
                    std::string sample = hofRoute.code + " line=" + hofRoute.line +
                                         " destination=" + hofRoute.name;
                    if (hofRoute.hasTerminusIndex) {
                        sample += " terminus=" + std::to_string(hofRoute.terminusIndex);
                    }
                    sample += " stop_count=" +
                              std::to_string(hofRoute.busstopIdents.size());
                    matchingHofLineSamples.push_back(std::move(sample));
                }
            }
            if (argc >= 4 && !catalogMode && route.timetableKey == argv[3]) {
                std::cout << "timetable " << route.timetableKey << " line=" << route.lineName
                          << " destination=" << route.destinationName << " stops=";
                for (const auto& stop : passengerRoute.stops)
                    std::cout << stop.name << '|';
                std::cout << '\n';
                if (const auto* matched = openbus::app::passengerRouteHofRoute(passengerRoute, hof))
                    std::cout << "matched_HOF=" << matched->code << '\n';
                for (const auto& candidate : hof.routes) {
                    if (candidate.line.empty() || route.timetableKey.find(candidate.line) != 0)
                        continue;
                    std::cout << "HOF " << candidate.code << " line=" << candidate.line
                              << " name=" << candidate.name << " stops=";
                    for (const auto& ident : candidate.busstopIdents)
                        std::cout << ident << '|';
                    std::cout << '\n';
                }
            }
            if (openbus::app::passengerRouteMatchesHof(passengerRoute, hof)) {
                matchedTimetables.push_back(route.timetableKey);
            }
            if (catalogMode) {
                passengerRoutes.push_back(passengerRoute);
            }
        }

        if (catalogMode) {
            openbus::app::PlayerTimetableCatalog catalog;
            const auto start = std::chrono::steady_clock::now();
            if (!catalog.update(passengerRoutes, map.aiTraffic.timetableCourses, hof))
                throw std::runtime_error("initial timetable catalog was not built");
            const auto built = std::chrono::steady_clock::now();
            for (int frame = 0; frame < 300; ++frame) {
                if (catalog.update(passengerRoutes, map.aiTraffic.timetableCourses, hof))
                    throw std::runtime_error("timetable catalog was rebuilt on an unchanged frame");
            }
            const auto reused = std::chrono::steady_clock::now();
            const auto catalogMatches =
                std::ranges::count(catalog.routes, openbus::app::PassengerHofMatchStatus::Matched,
                                   &openbus::app::PlayerTimetableCatalogRoute::status);
            if (static_cast<std::size_t>(catalogMatches) != matchedTimetables.size())
                throw std::runtime_error("cached HOF matches differ from direct matching");
            std::size_t selectableCourses = 0;
            std::size_t selectableTrips = 0;
            for (const auto& line : catalog.lines) {
                selectableCourses += line.courses.size();
                std::size_t lineTrips = 0;
                for (const auto& course : line.courses)
                    lineTrips += course.trips.size();
                selectableTrips += lineTrips;
                std::cout << "selectable_line=" << line.name
                          << " courses=" << line.courses.size()
                          << " trips=" << lineTrips << '\n';
            }
            std::cout << "catalog_build_ms="
                      << std::chrono::duration<double, std::milli>(built - start).count()
                      << " 300_cached_updates_ms="
                      << std::chrono::duration<double, std::milli>(reused - built).count()
                      << " selectable_lines=" << catalog.lines.size()
                      << " selectable_courses=" << selectableCourses
                      << " selectable_trips=" << selectableTrips << '\n';
            if (selectableTrips == 0)
                throw std::runtime_error("no scheduled HOF-matched course trips are selectable");
        }
        std::cout << "map=" << map.metadata.name << " scheduled_routes=" << scheduledRoutes
                  << " routes_with_passenger_stops=" << routesWithPassengerStops
                  << " HOF_matches=" << matchedTimetables.size() << '\n';
        for (const std::string& timetable : matchedTimetables) {
            std::cout << "  " << timetable << '\n';
        }
        if (matchedTimetables.empty()) {
            std::cout << "sample map routes:\n";
            for (const std::string& sample : timetableSamples) {
                std::cout << "  " << sample << '\n';
            }
            std::cout << "sample routes with an exact HOF line:\n";
            for (const std::string& sample : matchingLineSamples) {
                std::cout << "  " << sample << '\n';
            }
            std::cout << "map lines:";
            for (const std::string& line : mapLines) {
                std::cout << ' ' << line;
            }
            std::cout << "\nHOF lines:";
            for (const std::string& line : hofLines) {
                std::cout << ' ' << line;
            }
            std::cout << "\nHOF routes with an exact map line:\n";
            for (const std::string& sample : matchingHofLineSamples) {
                std::cout << "  " << sample << '\n';
            }
            std::cout << "sample HOF routes:\n";
            for (std::size_t index = 0; index < std::min<std::size_t>(hof.routes.size(), 4);
                 ++index) {
                const openbus::config::HofRoute& route = hof.routes[index];
                std::cout << "  " << route.code << " line=" << route.line << " stops=";
                for (std::size_t stopIndex = 0; stopIndex < route.busstopIdents.size();
                     ++stopIndex) {
                    if (stopIndex > 0) {
                        std::cout << '|';
                    }
                    std::cout << route.busstopIdents[stopIndex];
                }
                std::cout << '\n';
            }
        }
        return matchedTimetables.empty() ? 1 : 0;
    } catch (const std::exception& error) {
        std::cerr << "map/HOF compatibility probe failed: " << error.what() << '\n';
        return 1;
    }
}
