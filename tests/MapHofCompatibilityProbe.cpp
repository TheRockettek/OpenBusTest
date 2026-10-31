#include "app/traffic/PassengerStops.h"
#include "app/traffic/PlayerTimetableController.h"
#include "core/Variables.h"
#include "map/MapConfigLoader.h"
#include "scripting/ScriptRuntime.h"

#include <algorithm>
#include <filesystem>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "usage: OpenBusMapHofCompatibilityProbe <map-directory> <hof-file>\n";
        return 2;
    }

    try {
        const std::filesystem::path mapDirectory(argv[1]);
        const std::filesystem::path hofPath(argv[2]);
        const openbus::map::MapDefinition map = openbus::map::loadMapDefinition(mapDirectory);

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
        std::size_t scheduledRoutes = 0;
        std::size_t routesWithPassengerStops = 0;
        for (const openbus::map::MapAiTimetableRoute& route :
             map.aiTraffic.timetableRoutes) {
            if (route.scheduledDepartureMinutes.empty()) {
                continue;
            }
            ++scheduledRoutes;
            openbus::app::PassengerStops::Container stops;
            for (const openbus::map::MapAiTimetableRoute::Station& station : route.stations) {
                const auto position = openbus::map::mapAiTimetableStationPosition(route, station);
                if (!position || position->trackEntryIndex >= route.trackEntries.size() ||
                    route.trackEntries[position->trackEntryIndex].pathIndex < 0) {
                    continue;
                }
                openbus::map::MapTrafficPassengerStop stop;
                stop.stopId = station.id;
                stop.name = station.name;
                stops.push_back(std::move(stop));
            }
            if (stops.empty()) {
                continue;
            }
            ++routesWithPassengerStops;
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
            const openbus::app::PassengerStopRouteState passengerRoute{
                {}, openbus::app::PassengerStops(std::move(stops)), route.timetableKey,
                route.lineName, route.destinationName, route.tripName};
            if (openbus::app::passengerRouteMatchesHof(passengerRoute, hof)) {
                matchedTimetables.push_back(route.timetableKey);
            }
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
