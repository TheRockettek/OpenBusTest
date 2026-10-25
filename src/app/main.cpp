#include "physics/BusConfiguration.h"
#include "physics/BusSimulation.h"
#include "app/CrashHandler.h"
#include "core/Environment.h"
#include "core/Logger.h"
#include "map/MapCollisionStreamer.h"
#include "map/MapConfigLoader.h"
#include "map/MapSplineProfile.h"
#include "config/ModelConfigLoader.h"
#include "config/HumanConfigLoader.h"
#include "core/AssetPath.h"
#include "core/PerfTrace.h"
#include "app/RenderLoop.h"

#include <GLFW/glfw3.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace {

std::string pathToUtf8(const std::filesystem::path& path) {
    const std::u8string utf8 = path.u8string();
    return std::string(reinterpret_cast<const char*>(utf8.data()), utf8.size());
}

std::string environmentValue(const char* name) {
    const char* value = openbus::getEnvironment(name);
    return value == nullptr ? std::string{} : std::string(value);
}

bool booleanEnvironmentValue(const char* name, bool fallback) {
    std::string value = environmentValue(name);
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char character) {
        return static_cast<char>(std::tolower(character));
    });
    if (value.empty()) {
        return fallback;
    }
    if (value == "1" || value == "true" || value == "yes" || value == "on") {
        return true;
    }
    if (value == "0" || value == "false" || value == "no" || value == "off") {
        return false;
    }
    throw std::runtime_error(std::string(name) +
                             " must be one of 1/0, true/false, yes/no, or on/off");
}

std::pair<int, double> weatherSettingsFromEnvironment() {
    std::string weather = environmentValue("OPENBUS_WEATHER");
    std::transform(weather.begin(), weather.end(), weather.begin(), [](unsigned char character) {
        return static_cast<char>(std::tolower(character));
    });
    int precipitationType = 0;
    if (weather.empty() || weather == "clear") {
        precipitationType = 0;
    } else if (weather == "rain") {
        precipitationType = 1;
    } else if (weather == "snow") {
        precipitationType = 2;
    } else {
        throw std::runtime_error("OPENBUS_WEATHER must be clear, rain, or snow");
    }

    const std::string configuredRate = environmentValue("OPENBUS_WEATHER_RATE");
    if (configuredRate.empty()) {
        return {precipitationType, precipitationType == 0 ? 0.0 : 0.5};
    }
    char* end = nullptr;
    const double rate = std::strtod(configuredRate.c_str(), &end);
    if (end == configuredRate.c_str() || *end != '\0' || !std::isfinite(rate) || rate < 0.0 ||
        rate > 1.0 || (precipitationType == 0 && rate != 0.0)) {
        throw std::runtime_error(
            "OPENBUS_WEATHER_RATE must be in [0, 1] and non-zero only for rain or snow");
    }
    return {precipitationType, rate};
}

int positiveEnvironmentInt(const char* name, int fallback, int maximum) {
    const char* value = openbus::getEnvironment(name);
    if (value == nullptr || *value == '\0') {
        return fallback;
    }
    char* end = nullptr;
    const long parsed = std::strtol(value, &end, 10);
    if (end == value || *end != '\0' || parsed <= 0) {
        return fallback;
    }
    return static_cast<int>(std::min(parsed, static_cast<long>(maximum)));
}

int nonnegativeEnvironmentInt(const char* name, int fallback, int maximum) {
    const char* value = openbus::getEnvironment(name);
    if (value == nullptr || *value == '\0') {
        return fallback;
    }
    char* end = nullptr;
    const long parsed = std::strtol(value, &end, 10);
    if (end == value || *end != '\0' || parsed < 0) {
        return fallback;
    }
    return static_cast<int>(std::min(parsed, static_cast<long>(maximum)));
}

double nonnegativeEnvironmentDouble(const char* name, double fallback, double maximum) {
    const char* value = openbus::getEnvironment(name);
    if (value == nullptr || *value == '\0') {
        return fallback;
    }
    char* end = nullptr;
    const double parsed = std::strtod(value, &end);
    if (end == value || *end != '\0' || !std::isfinite(parsed) || parsed < 0.0) {
        return fallback;
    }
    return std::min(parsed, maximum);
}

std::size_t
maximumMapTrafficPopulation(std::size_t basePopulation,
                            const std::vector<openbus::map::MapTrafficDensityPoint>& schedule,
                            double densityMultiplier) {
    constexpr std::size_t populationCap = 200;
    const auto populationForDensity = [&](std::optional<double> density) {
        return openbus::map::mapTrafficPopulationForDensity(
            basePopulation, density.value_or(1.0) * densityMultiplier, populationCap);
    };
    const std::size_t fallback = populationForDensity(std::nullopt);
    if (schedule.empty() || !std::isfinite(schedule.front().hour) || schedule.front().hour < 0.0 ||
        schedule.front().hour > 24.0) {
        return fallback;
    }
    const std::optional<double> firstDensity =
        openbus::map::mapTrafficDensityAtHour(schedule, schedule.front().hour);
    if (!firstDensity.has_value()) {
        return fallback;
    }
    std::size_t maximum = populationForDensity(firstDensity);
    for (const openbus::map::MapTrafficDensityPoint& point : schedule) {
        const std::optional<double> density =
            openbus::map::mapTrafficDensityAtHour(schedule, point.hour);
        if (!density.has_value()) {
            return maximum;
        }
        maximum = std::max(maximum, populationForDensity(density));
    }
    return maximum;
}

std::size_t mapTrafficPopulationForTestDensity(std::size_t basePopulation,
                                               std::optional<double> density,
                                               double densityMultiplier) {
    return openbus::map::mapTrafficPopulationForDensity(
        basePopulation, density.value_or(1.0) * densityMultiplier, 200);
}

struct MapBusLineRouteStart {
    openbus::map::MapTrafficLaneStart entry;
    std::size_t routeSeed = 0;
    std::size_t timetableRouteIndex = 0;
    std::vector<int> requiredSplineSequence;
    std::vector<std::optional<std::size_t>> requiredSceneryPathSequence;
    bool stopAtRequiredSequenceEnd = false;
    bool allowIntermediarySegmentsBetweenRequired = false;
    bool allowReverseTraversalForRequired = false;
};

struct MapBusLineRouteSearch {
    std::vector<MapBusLineRouteStart> starts;
    std::vector<std::string> timetableOriginDiagnostics;
    std::size_t feasibleRoutesExamined = 0;
    std::size_t sharedTimetableSplineIds = 0;
    std::size_t longestCommonOrderedRun = 0;
    std::string bestTimetableLineName;
    std::string bestTimetableDestinationName;
    openbus::map::MapTrafficLaneStart bestEntry;
    std::size_t bestRouteSeed = 0;
    std::size_t bestRouteSplineCount = 0;
    std::size_t bestTimetableSplineCount = 0;
    std::size_t bestTimetablePathOffset = 0;
    std::size_t bestRoutePathOffset = 0;
    std::optional<int> nextTimetableSplineId;
    std::optional<int> nextRouteSplineId;
    std::vector<int> bestCommonSplineIds;
};

struct OrderedSplineRun {
    std::size_t length = 0;
    std::size_t firstStartIndex = 0;
    std::size_t secondStartIndex = 0;
};

std::vector<openbus::map::MapTrafficRouteRequirement> timetableRouteRequirements(
    const std::vector<openbus::map::MapAiTimetableRoute::TrackEntry>& timetable) {
    std::vector<openbus::map::MapTrafficRouteRequirement> requirements;
    requirements.reserve(timetable.size());
    for (const auto& entry : timetable) {
        requirements.push_back({entry.splineId, entry.pathIndex >= 0
                                                    ? std::optional<std::size_t>(
                                                          static_cast<std::size_t>(entry.pathIndex))
                                                    : std::nullopt});
    }
    return requirements;
}

bool timetablePathContainedInRoute(
    const std::vector<openbus::map::MapAiTimetableRoute::TrackEntry>& timetable,
    const std::vector<openbus::map::MapTrafficRouteRequirement>& route) {
    return openbus::map::mapTrafficTimetablePathContainedInRoute(
        timetableRouteRequirements(timetable), route);
}

std::size_t orderedTimetableCheckpointCount(
    const std::vector<openbus::map::MapTrafficRouteRequirement>& timetable,
    const std::vector<openbus::map::MapTrafficRouteRequirement>& route) {
    std::size_t timetableIndex = 0;
    for (const auto& routeEntry : route) {
        if (timetableIndex >= timetable.size()) {
            break;
        }
        const auto& timetableEntry = timetable[timetableIndex];
        if (routeEntry.splineId != timetableEntry.splineId ||
            (routeEntry.sceneryPathIndex.has_value() &&
             routeEntry.sceneryPathIndex != timetableEntry.sceneryPathIndex)) {
            continue;
        }
        const int matchedSplineId = timetableEntry.splineId;
        const auto matchedPathIndex = routeEntry.sceneryPathIndex;
        ++timetableIndex;
        while (
            timetableIndex < timetable.size() &&
            timetable[timetableIndex].splineId == matchedSplineId &&
            (!matchedPathIndex || timetable[timetableIndex].sceneryPathIndex == matchedPathIndex)) {
            ++timetableIndex;
        }
    }
    return timetableIndex;
}

OrderedSplineRun longestCommonOrderedSplineRun(const std::vector<int>& first,
                                               const std::vector<int>& second) {
    std::vector<std::size_t> previous(second.size() + 1);
    std::vector<std::size_t> current(second.size() + 1);
    OrderedSplineRun longest;
    for (std::size_t firstIndex = 0; firstIndex < first.size(); ++firstIndex) {
        for (std::size_t index = 1; index <= second.size(); ++index) {
            current[index] = first[firstIndex] == second[index - 1] ? previous[index - 1] + 1 : 0;
            if (current[index] > longest.length) {
                longest = {current[index], firstIndex + 1 - current[index], index - current[index]};
            }
        }
        std::swap(previous, current);
        std::fill(current.begin(), current.end(), 0);
    }
    return longest;
}

MapBusLineRouteSearch mapBusLineRouteStarts(const RenderLoop& renderer,
                                            const openbus::map::MapAiTrafficData& traffic,
                                            double minimumTurnRadiusMeters) {
    constexpr std::size_t minimumLineSplineCount = 2;
    constexpr std::size_t routeSeedCount = 16;
    MapBusLineRouteSearch result;
    std::unordered_set<int> timetableSplineIds;
    std::unordered_set<int> candidateSplineIds;
    std::size_t eligibleTimetableRouteCount = 0;
    for (const openbus::map::MapAiTimetableRoute& line : traffic.timetableRoutes) {
        if (line.stationCount > 0 && !line.destinationName.empty() && !line.lineName.empty() &&
            line.trackEntries.size() >= minimumLineSplineCount &&
            !line.scheduledDepartureMinutes.empty()) {
            timetableSplineIds.insert(line.splineIds.begin(), line.splineIds.end());
            ++eligibleTimetableRouteCount;
        }
    }
    std::unordered_set<std::size_t> matchedTimetableRoutes;
    const auto recordCommonRun = [&](const openbus::map::MapTrafficLaneStart& entry,
                                     std::size_t routeSeed, const std::vector<int>& routeSplineIds,
                                     const openbus::map::MapAiTimetableRoute& line,
                                     const OrderedSplineRun& commonRun) {
        if (commonRun.length <= result.longestCommonOrderedRun) {
            return;
        }
        result.longestCommonOrderedRun = commonRun.length;
        result.bestTimetableLineName = line.lineName;
        result.bestTimetableDestinationName = line.destinationName;
        result.bestEntry = entry;
        result.bestRouteSeed = routeSeed;
        result.bestRouteSplineCount = routeSplineIds.size();
        result.bestTimetableSplineCount = line.splineIds.size();
        result.bestTimetablePathOffset = commonRun.firstStartIndex;
        result.bestRoutePathOffset = commonRun.secondStartIndex;
        const std::size_t timetableNextIndex = commonRun.firstStartIndex + commonRun.length;
        const std::size_t routeNextIndex = commonRun.secondStartIndex + commonRun.length;
        result.nextTimetableSplineId = timetableNextIndex < line.splineIds.size()
                                           ? std::optional<int>(line.splineIds[timetableNextIndex])
                                           : std::nullopt;
        result.nextRouteSplineId = routeNextIndex < routeSplineIds.size()
                                       ? std::optional<int>(routeSplineIds[routeNextIndex])
                                       : std::nullopt;
        result.bestCommonSplineIds.assign(line.splineIds.begin() + commonRun.firstStartIndex,
                                          line.splineIds.begin() + timetableNextIndex);
    };
    std::vector<openbus::map::MapTrafficLaneStart> candidateEntries = renderer.trafficEntryLanes();
    for (const openbus::map::MapAiTimetableRoute& line : traffic.timetableRoutes) {
        if (line.trackEntries.empty() || line.scheduledDepartureMinutes.empty()) {
            continue;
        }
        const auto& firstEntry = line.trackEntries.front();
        const std::optional<std::size_t> pathIndex =
            firstEntry.pathIndex >= 0
                ? std::optional<std::size_t>(static_cast<std::size_t>(firstEntry.pathIndex))
                : std::nullopt;
        for (const auto& lane :
             renderer.trafficLanesForTimetableEntry(firstEntry.splineId, pathIndex)) {
            if (std::find(candidateEntries.begin(), candidateEntries.end(), lane) ==
                candidateEntries.end()) {
                candidateEntries.push_back(lane);
            }
        }
    }
    for (const auto& entry : candidateEntries) {
        if (matchedTimetableRoutes.size() == eligibleTimetableRouteCount) {
            break;
        }
        for (std::size_t lineIndex = 0; lineIndex < traffic.timetableRoutes.size(); ++lineIndex) {
            const openbus::map::MapAiTimetableRoute& line = traffic.timetableRoutes[lineIndex];
            if (line.stationCount == 0 || line.destinationName.empty() || line.lineName.empty() ||
                line.trackEntries.size() < minimumLineSplineCount ||
                line.scheduledDepartureMinutes.empty() ||
                matchedTimetableRoutes.contains(lineIndex)) {
                continue;
            }
            if (line.trackEntries.front().splineId != entry.splineId) {
                continue;
            }
            std::vector<int> requiredSplineSequence;
            std::vector<std::optional<std::size_t>> requiredSceneryPathSequence;
            for (const auto& trackEntry : line.trackEntries) {
                requiredSplineSequence.push_back(trackEntry.splineId);
                requiredSceneryPathSequence.push_back(
                    trackEntry.pathIndex >= 0
                        ? std::optional<std::size_t>(static_cast<std::size_t>(trackEntry.pathIndex))
                        : std::nullopt);
            }
            const auto routeSegments = renderer.trafficRouteSegments(
                entry.splineId, openbus::map::MAP_SPLINE_PATH_TYPE_ROAD, entry.laneSelector, 0.0, 0,
                requiredSplineSequence, {2, std::nullopt, true, std::nullopt},
                requiredSceneryPathSequence, true);
            if (routeSegments.size() < minimumLineSplineCount) {
                continue;
            }
            ++result.feasibleRoutesExamined;
            std::vector<int> routeSplineIds;
            routeSplineIds.reserve(routeSegments.size());
            for (const auto& segment : routeSegments) {
                routeSplineIds.push_back(segment.splineId);
                candidateSplineIds.insert(segment.splineId);
            }
            const OrderedSplineRun commonRun =
                longestCommonOrderedSplineRun(line.splineIds, routeSplineIds);
            recordCommonRun(entry, 0, routeSplineIds, line, commonRun);
            if (!timetablePathContainedInRoute(line.trackEntries, routeSegments)) {
                continue;
            }
            MapBusLineRouteStart routeStart;
            routeStart.entry = entry;
            routeStart.routeSeed = 0;
            routeStart.timetableRouteIndex = lineIndex;
            routeStart.requiredSplineSequence = std::move(requiredSplineSequence);
            routeStart.requiredSceneryPathSequence = std::move(requiredSceneryPathSequence);
            routeStart.stopAtRequiredSequenceEnd = true;
            result.starts.push_back(std::move(routeStart));
            matchedTimetableRoutes.insert(lineIndex);
            break;
        }
    }
    for (std::size_t lineIndex = 0; lineIndex < traffic.timetableRoutes.size(); ++lineIndex) {
        if (matchedTimetableRoutes.contains(lineIndex)) {
            continue;
        }
        const auto& line = traffic.timetableRoutes[lineIndex];
        if (line.trackEntries.size() < minimumLineSplineCount ||
            line.scheduledDepartureMinutes.empty()) {
            continue;
        }
        const auto requirements = timetableRouteRequirements(line.trackEntries);
        std::vector<openbus::map::MapTrafficRouteRequirement> representedRequirements;
        std::vector<int> requiredSplineSequence;
        std::vector<std::optional<std::size_t>> requiredPathSequence;
        std::size_t missingCheckpointCount = 0;
        for (const auto& requirement : requirements) {
            if (renderer
                    .trafficLanesForTimetableEntry(requirement.splineId,
                                                   requirement.sceneryPathIndex)
                    .empty()) {
                ++missingCheckpointCount;
                continue;
            }
            representedRequirements.push_back(requirement);
            requiredSplineSequence.push_back(requirement.splineId);
            requiredPathSequence.push_back(requirement.sceneryPathIndex);
        }
        if (missingCheckpointCount == 0 ||
            representedRequirements.size() < minimumLineSplineCount) {
            continue;
        }
        const auto origins = renderer.trafficLanesForTimetableEntry(
            requirements.front().splineId, requirements.front().sceneryPathIndex);
        std::size_t bestUnconstrainedCoverage = 0;
        std::optional<openbus::map::MapTrafficLaneStart> bestUnconstrainedOrigin;
        std::size_t bestUnconstrainedSeed = 0;
        for (const auto& origin : origins) {
            for (std::size_t routeSeed = 0; routeSeed < routeSeedCount; ++routeSeed) {
                const auto routeSegments = renderer.trafficRouteSegments(
                    origin.splineId, openbus::map::MAP_SPLINE_PATH_TYPE_ROAD, origin.laneSelector,
                    0.0, routeSeed, requiredSplineSequence, {2, std::nullopt, true, std::nullopt},
                    requiredPathSequence, true, true, true);
                if (!openbus::map::mapTrafficTimetablePathContainedInRouteInOrder(
                        representedRequirements, routeSegments)) {
                    continue;
                }
                matchedTimetableRoutes.insert(lineIndex);
                break;
            }
            if (!matchedTimetableRoutes.contains(lineIndex)) {
                for (std::size_t routeSeed = 0; routeSeed < routeSeedCount; ++routeSeed) {
                    const auto routeSegments = renderer.trafficRouteSegments(
                        origin.splineId, openbus::map::MAP_SPLINE_PATH_TYPE_ROAD,
                        origin.laneSelector, 0.0, routeSeed, {},
                        {2, std::nullopt, true, std::nullopt});
                    std::vector<openbus::map::MapTrafficRouteRequirement> routeRequirements;
                    routeRequirements.reserve(routeSegments.size());
                    for (const auto& segment : routeSegments) {
                        routeRequirements.push_back({segment.splineId, segment.sceneryPathIndex});
                    }
                    const std::size_t coverage =
                        orderedTimetableCheckpointCount(representedRequirements, routeRequirements);
                    if (coverage > bestUnconstrainedCoverage) {
                        bestUnconstrainedCoverage = coverage;
                        bestUnconstrainedOrigin = origin;
                        bestUnconstrainedSeed = routeSeed;
                    }
                }
            }
            if (matchedTimetableRoutes.contains(lineIndex)) {
                break;
            }
        }
        if (!matchedTimetableRoutes.contains(lineIndex) && bestUnconstrainedOrigin) {
            result.timetableOriginDiagnostics.push_back(
                "Scheduled timetable graph coverage " + line.lineName + " (" + line.tripName +
                ") from spline=" + std::to_string(bestUnconstrainedOrigin->splineId) +
                ", lane=" + std::to_string(bestUnconstrainedOrigin->laneSelector) +
                ", route seed=" + std::to_string(bestUnconstrainedSeed) +
                ": unconstrained "
                "route follows " +
                std::to_string(bestUnconstrainedCoverage) + " of " +
                std::to_string(representedRequirements.size()) +
                " represented checkpoints in order, including " +
                std::to_string(missingCheckpointCount) +
                " timetable checkpoint(s) with no lane data.");
        }
    }
    for (std::size_t lineIndex = 0; lineIndex < traffic.timetableRoutes.size(); ++lineIndex) {
        const auto& line = traffic.timetableRoutes[lineIndex];
        if (line.trackEntries.size() < minimumLineSplineCount ||
            line.scheduledDepartureMinutes.empty() || matchedTimetableRoutes.contains(lineIndex)) {
            continue;
        }
        const auto requirements = timetableRouteRequirements(line.trackEntries);
        std::vector<openbus::map::MapTrafficRouteRequirement> representedRequirements;
        representedRequirements.reserve(requirements.size());
        for (const auto& requirement : requirements) {
            if (!renderer
                     .trafficLanesForTimetableEntry(requirement.splineId,
                                                    requirement.sceneryPathIndex)
                     .empty()) {
                representedRequirements.push_back(requirement);
            }
        }
        if (representedRequirements.size() < 2) {
            continue;
        }
        const auto& firstEntry = representedRequirements.front();
        const auto origins = renderer.trafficLanesForTimetableEntry(firstEntry.splineId,
                                                                    firstEntry.sceneryPathIndex);
        std::size_t bestMatchedEntries = 0;
        std::optional<openbus::map::MapTrafficLaneStart> bestOrigin;
        for (const auto& origin : origins) {
            std::size_t lower = 2;
            std::size_t upper = representedRequirements.size();
            std::size_t matchedEntries = 1;
            while (lower <= upper) {
                const std::size_t middle = lower + (upper - lower) / 2;
                std::vector<int> requiredSplineSequence;
                std::vector<std::optional<std::size_t>> requiredPathSequence;
                requiredSplineSequence.reserve(middle);
                requiredPathSequence.reserve(middle);
                for (std::size_t index = 0; index < middle; ++index) {
                    requiredSplineSequence.push_back(representedRequirements[index].splineId);
                    requiredPathSequence.push_back(representedRequirements[index].sceneryPathIndex);
                }
                const auto routeSegments = renderer.trafficRouteSegments(
                    origin.splineId, openbus::map::MAP_SPLINE_PATH_TYPE_ROAD, origin.laneSelector,
                    0.0, 0, requiredSplineSequence, {2, std::nullopt, true, std::nullopt},
                    requiredPathSequence, true, true, true);
                const std::vector<openbus::map::MapTrafficRouteRequirement> prefix(
                    representedRequirements.begin(),
                    representedRequirements.begin() + static_cast<std::ptrdiff_t>(middle));
                if (openbus::map::mapTrafficTimetablePathContainedInRouteInOrder(prefix,
                                                                                 routeSegments)) {
                    matchedEntries = middle;
                    lower = middle + 1;
                } else {
                    upper = middle - 1;
                }
            }
            if (matchedEntries > bestMatchedEntries) {
                bestMatchedEntries = matchedEntries;
                bestOrigin = origin;
            }
        }
        if (!bestOrigin) {
            result.timetableOriginDiagnostics.push_back(
                "Scheduled timetable origin diagnostic " + line.lineName + " (" + line.tripName +
                "): no lane matches first checkpoint spline=" +
                std::to_string(firstEntry.splineId) +
                ", path=" + std::to_string(firstEntry.sceneryPathIndex.value_or(0)) + ".");
            continue;
        }
        std::string diagnostic =
            "Scheduled timetable origin diagnostic " + line.lineName + " (" + line.tripName +
            ") at spline=" + std::to_string(firstEntry.splineId) +
            ", lane=" + std::to_string(bestOrigin->laneSelector) + ": graph matches " +
            std::to_string(bestMatchedEntries) + " of " +
            std::to_string(representedRequirements.size()) + " represented ordered checkpoints " +
            "from the authored origin (" +
            std::to_string(line.trackEntries.size() - representedRequirements.size()) +
            " checkpoint(s) have no lane data).";
        if (bestMatchedEntries < representedRequirements.size()) {
            const auto& previous = representedRequirements[bestMatchedEntries - 1];
            const auto& next = representedRequirements[bestMatchedEntries];
            const auto previousLanes = renderer.trafficLanesForTimetableEntry(
                previous.splineId, previous.sceneryPathIndex);
            const auto nextLanes =
                renderer.trafficLanesForTimetableEntry(next.splineId, next.sceneryPathIndex);
            const auto nearestConnectionGap = renderer.trafficLaneConnectionGap(
                previous.splineId, previous.sceneryPathIndex, next.splineId, next.sceneryPathIndex);
            bool pairConnectsWithoutTurnLimit = false;
            bool pairConnectsWithTurnLimit = false;
            const std::vector<int> pairSplineSequence = {previous.splineId, next.splineId};
            const std::vector<std::optional<std::size_t>> pairPathSequence = {
                previous.sceneryPathIndex, next.sceneryPathIndex};
            for (const auto& lane : previousLanes) {
                const auto connects = [&](double turnRadius) {
                    const auto routeSegments = renderer.trafficRouteSegments(
                        lane.splineId, openbus::map::MAP_SPLINE_PATH_TYPE_ROAD, lane.laneSelector,
                        turnRadius, 0, pairSplineSequence, {2, std::nullopt, true, std::nullopt},
                        pairPathSequence, true, true, true);
                    return openbus::map::mapTrafficTimetablePathContainedInRouteInOrder(
                        {previous, next}, routeSegments);
                };
                pairConnectsWithoutTurnLimit = pairConnectsWithoutTurnLimit || connects(0.0);
                pairConnectsWithTurnLimit =
                    pairConnectsWithTurnLimit || connects(minimumTurnRadiusMeters);
            }
            bool graphBridgesUnrepresentedCheckpoints = false;
            bool unconstrainedGraphReachesNextRepresented = false;
            if (bestMatchedEntries < representedRequirements.size()) {
                const auto& nextRepresented = representedRequirements[bestMatchedEntries];
                const std::vector<int> bridgeSplineSequence = {previous.splineId,
                                                               nextRepresented.splineId};
                const std::vector<std::optional<std::size_t>> bridgePathSequence = {
                    previous.sceneryPathIndex, nextRepresented.sceneryPathIndex};
                for (const auto& lane : previousLanes) {
                    const auto routeSegments = renderer.trafficRouteSegments(
                        lane.splineId, openbus::map::MAP_SPLINE_PATH_TYPE_ROAD, lane.laneSelector,
                        0.0, 0, bridgeSplineSequence, {2, std::nullopt, true, std::nullopt},
                        bridgePathSequence, true, true, true);
                    if (openbus::map::mapTrafficTimetablePathContainedInRouteInOrder(
                            {previous, nextRepresented}, routeSegments)) {
                        graphBridgesUnrepresentedCheckpoints = true;
                        break;
                    }
                }
                for (const auto& lane : previousLanes) {
                    for (std::size_t seed = 0; seed < routeSeedCount; ++seed) {
                        const auto routeSegments = renderer.trafficRouteSegments(
                            lane.splineId, openbus::map::MAP_SPLINE_PATH_TYPE_ROAD,
                            lane.laneSelector, 0.0, seed, {},
                            {2, std::nullopt, true, std::nullopt});
                        const auto reached = std::find_if(
                            routeSegments.begin() + std::min<std::size_t>(1, routeSegments.size()),
                            routeSegments.end(), [&](const auto& segment) {
                                return segment.splineId == nextRepresented.splineId &&
                                       (!segment.sceneryPathIndex.has_value() ||
                                        segment.sceneryPathIndex ==
                                            nextRepresented.sceneryPathIndex);
                            });
                        if (reached != routeSegments.end()) {
                            unconstrainedGraphReachesNextRepresented = true;
                            break;
                        }
                    }
                    if (unconstrainedGraphReachesNextRepresented) {
                        break;
                    }
                }
            }
            diagnostic +=
                " First unconnected checkpoint: spline=" + std::to_string(next.splineId) +
                ", path=" + std::to_string(next.sceneryPathIndex.value_or(0)) +
                " (previous spline=" + std::to_string(previous.splineId) + " has " +
                std::to_string(previousLanes.size()) + " lane(s); next has " +
                std::to_string(nextLanes.size()) + "; direct pair connects without turn limit=" +
                (pairConnectsWithoutTurnLimit ? "yes" : "no") +
                ", with bus turn limit=" + (pairConnectsWithTurnLimit ? "yes" : "no") +
                "; nearest lane-end gap=" +
                (nearestConnectionGap
                     ? std::to_string(nearestConnectionGap->horizontalMeters) + "m horizontal, " +
                           std::to_string(nearestConnectionGap->verticalMeters) +
                           "m vertical, heading alignment=" +
                           std::to_string(nearestConnectionGap->headingAlignment)
                     : std::string("unavailable")) +
                "; next represented checkpoint=" + std::to_string(next.splineId) +
                "; graph bridges to it=" + (graphBridgesUnrepresentedCheckpoints ? "yes" : "no") +
                "; unconstrained routes reach it=" +
                (unconstrainedGraphReachesNextRepresented ? "yes" : "no") + ").";
        }
        result.timetableOriginDiagnostics.push_back(std::move(diagnostic));
    }
    for (const int splineId : candidateSplineIds) {
        if (timetableSplineIds.contains(splineId)) {
            ++result.sharedTimetableSplineIds;
        }
    }
    return result;
}

std::string formatSplineIds(const std::vector<int>& splineIds) {
    std::ostringstream formatted;
    formatted << '[';
    for (std::size_t index = 0; index < splineIds.size(); ++index) {
        if (index > 0) {
            formatted << ", ";
        }
        formatted << splineIds[index];
    }
    formatted << ']';
    return formatted.str();
}

std::string formatPlacement(const VehiclePlacement& placement) {
    std::ostringstream formatted;
    formatted << std::fixed << std::setprecision(3) << '(' << placement.position[0] << ", "
              << placement.position[1] << ", " << placement.position[2] << ')';
    return formatted.str();
}

VehiclePlacement aiVehiclePlacement(int index, int vehicleCount) {
    if (vehicleCount == 1) {
        return {{5.0, 5.0, 0.0}, 0.0};
    }
    constexpr int columns = 20;
    constexpr double columnSpacing = 5.5;
    constexpr double rowSpacing = 8.0;
    const int column = index % columns;
    const int row = index / columns;
    const double centeredColumn = static_cast<double>(column) - (columns - 1) / 2.0;
    return {{centeredColumn * columnSpacing, 12.0 + row * rowSpacing, 0.0}, 0.0};
}

const char* benchmarkPhaseName(RenderBenchmarkPhase phase) {
    switch (phase) {
    case RenderBenchmarkPhase::Baseline:
        return "Benchmark.phase.baseline";
    case RenderBenchmarkPhase::CameraCycle:
        return "Benchmark.phase.cameraCycle";
    case RenderBenchmarkPhase::ThirdPersonZoom:
        return "Benchmark.phase.thirdPersonZoom";
    case RenderBenchmarkPhase::DrivingControls:
        return "Benchmark.phase.drivingControls";
    case RenderBenchmarkPhase::DashboardInteraction:
        return "Benchmark.phase.dashboardInteraction";
    }
    return "Benchmark.phase.unknown";
}

} // namespace

int main() {
    installCrashHandler();
    Logger applicationLog("Application");
    try {
        applicationLog.Log("Starting OpenBus");
#if OPENBUS_ENABLE_PERF_TRACE
        const std::string traceEnabled = environmentValue("OPENBUS_TRACE");
        const std::string tracePath = environmentValue("OPENBUS_TRACE_FILE");
        applicationLog.Log(
            "Performance tracing instrumentation is compiled in; runtime=" +
            (traceEnabled.empty() ? std::string("disabled") : traceEnabled) +
            ", output=" + (tracePath.empty() ? std::string("openbus_trace.json") : tracePath) +
            ". Press F10 to toggle recording.");
#else
        applicationLog.Log("Performance tracing instrumentation is not compiled in.");
#endif

        const bool benchmarkMode =
            openbus::rendering::parseEnabledFlag(openbus::getEnvironment("OPENBUS_BENCHMARK"));
        const int windowWidth =
            benchmarkMode ? positiveEnvironmentInt("OPENBUS_BENCHMARK_WIDTH", 1280, 16384) : 1920;
        const int windowHeight =
            benchmarkMode ? positiveEnvironmentInt("OPENBUS_BENCHMARK_HEIGHT", 720, 16384) : 1080;
        applicationLog.Log("Loading: initializing renderer and OpenGL window.");
        RenderLoop renderer(windowWidth, windowHeight, "OpenBus");

        const std::filesystem::path omsiRoot = omsiRootPath();
        StartupSelection startupSelection;
        if (benchmarkMode) {
            const std::string configuredMap = environmentValue("OPENBUS_MAP_PATH");
            startupSelection.mapDirectory =
                omsiRoot / "maps" / (configuredMap.empty() ? "Cotterell" : configuredMap);
            startupSelection.mapDefinition =
                openbus::map::loadMapDefinition(startupSelection.mapDirectory);
            SystemVariables currentCalendar;
            startupSelection.year = static_cast<int>(currentCalendar.get("year"));
            startupSelection.month = static_cast<int>(currentCalendar.get("month"));
            startupSelection.day = static_cast<int>(currentCalendar.get("day"));
            const double configuredHour =
                nonnegativeEnvironmentDouble("OPENBUS_MAP_AI_HOUR", 12.0, 24.0);
            const double normalizedHour = std::fmod(configuredHour, 24.0);
            startupSelection.timeSeconds = normalizedHour * 3600.0;
            const std::string configuredSpawn = environmentValue("OPENBUS_MAP_SPAWN");
            const std::string configuredEntryPoint = environmentValue("OPENBUS_MAP_ENTRY");
            startupSelection.spawnEntryPoint =
                !configuredSpawn.empty()
                    ? openbus::map::selectMapSpawnPoint(startupSelection.mapDefinition,
                                                        configuredSpawn)
                    : (!configuredEntryPoint.empty()
                           ? openbus::map::selectMapEntryPoint(startupSelection.mapDefinition,
                                                               configuredEntryPoint)
                           : openbus::map::selectMapSpawnPoint(startupSelection.mapDefinition, {}));
        } else if (!renderer.runStartupMenu(omsiRoot, startupSelection)) {
            applicationLog.Log("Startup menu cancelled.");
            return 0;
        }

        // Resolve configuration paths for the bus and model.
        std::filesystem::path busConfigPath;
        std::filesystem::path modelConfigPath;
        std::filesystem::path aiBusConfigPath;
        std::filesystem::path aiModelConfigPath;
        std::filesystem::path mapDirectory;
        int aiVehicleCount = 0;
        {
            applicationLog.Log("Loading: resolving bus and model configuration paths.");
            openbus::rendering::TraceScope trace("config", "main.resolveConfigurationPaths");
            if (benchmarkMode) {
                busConfigPath = busConfigurationPathFor();
                modelConfigPath = modelConfigurationPathForBus(busConfigPath);
            } else {
                busConfigPath = startupSelection.vehicleConfigPath;
                modelConfigPath = modelConfigurationPathForBusFromConfig(busConfigPath);
            }
            const std::string configuredAiBusPath = environmentValue("OPENBUS_AI_BUS_CONFIG");
            const std::string configuredAiModelPath = environmentValue("OPENBUS_AI_MODEL_CONFIG");
            const bool hasAiBusPath = !configuredAiBusPath.empty();
            const bool hasAiModelPath = !configuredAiModelPath.empty();
            if (hasAiBusPath != hasAiModelPath) {
                throw std::runtime_error(
                    "OPENBUS_AI_BUS_CONFIG and OPENBUS_AI_MODEL_CONFIG must be set together");
            }
            if (hasAiBusPath) {
                aiBusConfigPath = busConfigurationPathFor(configuredAiBusPath);
                aiModelConfigPath =
                    modelConfigurationPathForBus(aiBusConfigPath, configuredAiModelPath);
                aiVehicleCount = positiveEnvironmentInt("OPENBUS_AI_COUNT", 1, 200);
            }
            mapDirectory = startupSelection.mapDirectory;
        }

        applicationLog.Log("Using selected map manifest: " + pathToUtf8(mapDirectory));
        openbus::map::MapDefinition mapDefinition = std::move(startupSelection.mapDefinition);
        SystemVariables mapCalendar;
        const double mapAiStartHour = startupSelection.timeSeconds / 3600.0;
        mapCalendar.configureSimulationClock(startupSelection.year, startupSelection.month,
                                             startupSelection.day, startupSelection.timeSeconds);
        const int mapCalendarYear = static_cast<int>(mapCalendar.get("year"));
        const int mapCalendarMonth = static_cast<int>(mapCalendar.get("month"));
        const int mapCalendarDay = static_cast<int>(mapCalendar.get("day"));
        const int mapCalendarDayOfYear = static_cast<int>(mapCalendar.get("dayofyear"));
        const std::chrono::year_month_day mapCalendarDate{
            std::chrono::year{mapCalendarYear},
            std::chrono::month{static_cast<unsigned>(mapCalendarMonth)},
            std::chrono::day{static_cast<unsigned>(mapCalendarDay)}};
        const std::chrono::sys_days firstDayOfYear{std::chrono::year_month_day{
            mapCalendarDate.year(), std::chrono::January, std::chrono::day{1}}};
        if (!mapCalendarDate.ok() ||
            (std::chrono::sys_days{mapCalendarDate} - firstDayOfYear).count() !=
                mapCalendarDayOfYear) {
            throw std::runtime_error("Map calendar system variables contain an invalid date");
        }
        const int mapAiStartWeekdayIndex = static_cast<int>(std::chrono::weekday{
            firstDayOfYear +
            std::chrono::days{mapCalendarDayOfYear}}.c_encoding());
        applicationLog.Log("Chrono map tile overrides are disabled; using base map tile files.");
        const std::size_t terrainSidecars = static_cast<std::size_t>(std::count_if(
            mapDefinition.tiles.begin(), mapDefinition.tiles.end(),
            [](const openbus::map::MapTileReference& tile) { return tile.hasTerrainFile; }));
        std::size_t groundTextureSidecars = 0;
        for (const openbus::map::MapTileReference& tile : mapDefinition.tiles) {
            groundTextureSidecars += tile.groundTextureSidecars.size();
        }
        const std::string displayMapName = !mapDefinition.metadata.friendlyName.empty()
                                               ? mapDefinition.metadata.friendlyName
                                               : mapDefinition.metadata.name;
        const std::string mapNameSummary =
            displayMapName.empty() ? std::string{} : ", name=" + displayMapName;
        applicationLog.Log(
            "Map manifest loaded: tiles=" + std::to_string(mapDefinition.tiles.size()) +
            ", terrain sidecars=" + std::to_string(terrainSidecars) +
            ", ground-texture sidecars=" + std::to_string(groundTextureSidecars) +
            ", ground textures=" + std::to_string(mapDefinition.groundTextures.size()) +
            ", entrypoints=" + std::to_string(mapDefinition.entryPoints.size()) + mapNameSummary +
            ".");
        for (const ConfigurationDiagnostic& diagnostic : mapDefinition.diagnostics.entries) {
            applicationLog.Log("Map global.cfg line " + std::to_string(diagnostic.line) + " [" +
                               diagnostic.keyword + "]: " + diagnostic.message);
        }
        const std::size_t spawnEntryPoint = startupSelection.spawnEntryPoint;
        const std::size_t groundTextureIndex = openbus::map::selectMapGroundTextureIndex(
            mapDefinition, environmentValue("OPENBUS_MAP_GROUNDTEX"));
        const VehiclePlacement busPlacement = mapDefinition.entryPoints[spawnEntryPoint].placement;
        if (!benchmarkMode) {
            renderer.SetNoBusSpawnPoint(busPlacement);
        }
        applicationLog.Log("Map " + pathToUtf8(mapDirectory) + " spawn: [" +
                           std::to_string(spawnEntryPoint) + "] " +
                           mapDefinition.entryPoints[spawnEntryPoint].name + " at " +
                           formatPlacement(busPlacement) + " m, yaw " +
                           std::to_string(busPlacement.yawDegrees) + " deg");

        BusConfiguration configuration;
        {
            applicationLog.Log("Loading bus physics configuration: " + busConfigPath.string());
            openbus::rendering::TraceScope trace("config", "main.loadBusConfiguration");
            configuration = loadBusConfiguration(busConfigPath, modelConfigPath);
            applicationLog.Log("Bus physics configuration loaded.");
        }

        ModelConfig modelConfiguration;
        {
            applicationLog.Log("Loading bus model configuration: " + modelConfigPath.string());
            openbus::rendering::TraceScope trace("config", "main.loadBusModelConfiguration");
            modelConfiguration = loadBusModelConfigurationFromModelConfig(modelConfigPath);
            applicationLog.Log("Bus model configuration loaded.");
        }

        {
            applicationLog.Log("Loading: writing configuration snapshots.");
            openbus::rendering::TraceScope trace("config", "main.writeConfigurationSnapshots");
            std::ofstream configurationOutput("OpenBus_configuration.json", std::ios::trunc);
            if (!configurationOutput) {
                applicationLog.Log("Failed to open OpenBus_configuration.json");
                throw std::runtime_error("Failed to open OpenBus_configuration.json");
            }
            writeBusConfigurationJson(configurationOutput, configuration);
            configurationOutput.flush();
            std::ofstream modelConfigurationOutput("OpenBus_model_configuration.json",
                                                   std::ios::trunc);
            if (!modelConfigurationOutput) {
                applicationLog.Log("Failed to open OpenBus_model_configuration.json");
                throw std::runtime_error("Failed to open OpenBus_model_configuration.json");
            }
            writeModelConfigurationJson(modelConfigurationOutput, modelConfigPath,
                                        modelConfiguration);
            modelConfigurationOutput.flush();
        }

        const int spawnTileX = static_cast<int>(
            std::floor(busPlacement.position[0] / mapDefinition.coordinates.tileSize));
        const int spawnTileY = static_cast<int>(
            std::floor(busPlacement.position[1] / mapDefinition.coordinates.tileSize));
        std::vector<TerrainCollisionGrid> spawnTerrain;
        std::vector<std::pair<int, int>> preloadedTerrainTiles;
        applicationLog.Log("Loading spawn terrain collision.");
        for (const openbus::map::MapTileReference& tile : mapDefinition.tiles) {
            if (!tile.hasTerrainFile || std::abs(tile.x - spawnTileX) > 2 ||
                std::abs(tile.y - spawnTileY) > 2) {
                continue;
            }
            try {
                const openbus::map::TerrainGrid grid =
                    openbus::map::loadTerrainGrid(tile.terrainPath);
                spawnTerrain.push_back({tile.x, tile.y, mapDefinition.coordinates.tileSize,
                                        grid.intervals, grid.heights});
                preloadedTerrainTiles.emplace_back(tile.x, tile.y);
            } catch (const std::exception& error) {
                applicationLog.Log("Spawn terrain collision unavailable for tile (" +
                                   std::to_string(tile.x) + "," + std::to_string(tile.y) +
                                   "): " + error.what());
            }
        }
        if (spawnTerrain.empty()) {
            throw std::runtime_error("No terrain collision could be loaded around the spawn point");
        }
        applicationLog.Log("Spawn terrain collision preloaded: " +
                           std::to_string(spawnTerrain.size()) + " tiles.");
        applicationLog.Log("Loading: initializing bus simulation and collision streaming.");
        BusSimulation simulation(configuration, busPlacement, 60.0, 8, busPlacement.position[2],
                                 std::move(spawnTerrain), {}, false);
        simulation.setFrozen(!benchmarkMode);
        openbus::map::MapCollisionStreamer collisionStreamer(
            mapDefinition, omsiRootPath(), simulation, 2, 2, std::move(preloadedTerrainTiles));

        const auto [precipitationType, precipitationRate] = weatherSettingsFromEnvironment();
        renderer.SetWeather(precipitationType, precipitationRate);
        const char* weatherName = precipitationType == 1   ? "rain"
                                  : precipitationType == 2 ? "snow"
                                                           : "clear";
        applicationLog.Log("Weather initialized: " + std::string(weatherName) +
                           ", precipitation rate=" + std::to_string(precipitationRate) + ".");
        applicationLog.Log("Loading map scene: terrain, tile index, scenery and roads.");
        renderer.SetMap(mapDefinition, spawnEntryPoint, groundTextureIndex, omsiRootPath());
        applicationLog.Log("Map scene loaded; initializing spawn render window.");
        renderer.updateMapStreaming(busPlacement.position[0], busPlacement.position[1]);
        const std::vector<std::pair<int, int>> spawnLoadedGeometryTiles = renderer.loadedMapTiles();
        std::vector<std::pair<int, int>> roadTrafficPoolLoadedTiles = spawnLoadedGeometryTiles;
        collisionStreamer.update(busPlacement.position[0], busPlacement.position[1],
                                 spawnLoadedGeometryTiles);
        applicationLog.Log("Map collision streaming started asynchronously with spawn terrain "
                           "already active: resident tiles=" +
                           std::to_string(collisionStreamer.loadedTileCount()) +
                           "; map manifest tiles=" + std::to_string(mapDefinition.tiles.size()) +
                           ".");
        std::size_t reportedCollisionTileCount = collisionStreamer.loadedTileCount();
        Vehicle* playerVehicle = nullptr;
        if (benchmarkMode || startupSelection.startInBus) {
            applicationLog.Log("Loading player vehicle: " + busConfigPath.string());
            playerVehicle =
                renderer.AddVehicle(busConfigPath, modelConfigPath, busPlacement,
                                    {AssetLoadingMode::Deferred, AssetLoadingMode::Eager}, false,
                                    startupSelection.vehicleSetup);
            simulation.setFrozen(false);
            renderer.SetPlayerVehicle(playerVehicle);
            applicationLog.Log("Player vehicle initialized; preparing AI traffic and pedestrians.");
        } else {
            applicationLog.Log("Starting on foot; use the in-game panel to spawn a bus.");
        }
        if (!aiBusConfigPath.empty()) {
            std::cout << "AI_VEHICLES=" << aiVehicleCount << '\n';
            for (int index = 0; index < aiVehicleCount; ++index) {
                renderer.AddVehicle(aiBusConfigPath, aiModelConfigPath,
                                    aiVehiclePlacement(index, aiVehicleCount),
                                    {AssetLoadingMode::Deferred, AssetLoadingMode::Deferred}, true);
            }
        }
        const int mapAiBaseCount = nonnegativeEnvironmentInt("OPENBUS_MAP_AI_COUNT", 200, 1000);
        const int mapAiBusCount = nonnegativeEnvironmentInt("OPENBUS_MAP_BUS_COUNT", 20, 200);
        const int mapHumanBaseCount = nonnegativeEnvironmentInt("OPENBUS_MAP_HUMAN_COUNT", 8, 200);
        const double scheduledHumanScale =
            nonnegativeEnvironmentDouble("OPENBUS_MAP_SCHEDULED_HUMAN_SCALE", 1.0, 100.0);
        const double mapTrafficDensityMultiplier =
            nonnegativeEnvironmentDouble("OPENBUS_MAP_TRAFFIC_DENSITY_MULTIPLIER", 1.0, 100.0);
        const std::size_t mapAiBusPoolCount = mapTrafficPopulationForTestDensity(
            static_cast<std::size_t>(mapAiBusCount), 1.0, mapTrafficDensityMultiplier);
        if (mapTrafficDensityMultiplier != 1.0) {
            applicationLog.Log("Map traffic density multiplier is " +
                               std::to_string(mapTrafficDensityMultiplier) +
                               "x "
                               "(OPENBUS_MAP_TRAFFIC_DENSITY_MULTIPLIER; active population cap "
                               "200).");
        }
        const bool mapTrafficSpawnInView =
            booleanEnvironmentValue("OPENBUS_MAP_AI_SPAWN_VISIBLE", false);
        const std::vector<openbus::map::MapAiRoadVehicle> availableAiRoadVehicles =
            openbus::map::mapAiRoadVehicles(mapDefinition.aiTraffic);
        std::function<void()> queueAdditionalMapRoadTraffic;
        renderer.SetMapCalendar(mapCalendarYear, mapCalendarMonth, mapCalendarDay,
                                mapAiStartHour * 3600.0);
        renderer.SetMapTrafficSpawnInView(mapTrafficSpawnInView);
        if (mapTrafficSpawnInView) {
            applicationLog.Log(
                "Map AI traffic may spawn in camera view "
                "(OPENBUS_MAP_AI_SPAWN_VISIBLE=1); active-tile and collision-clearance checks "
                "remain enabled.");
        }
        if (mapAiBaseCount == 0) {
            applicationLog.Log("Map-driven AI traffic is disabled by OPENBUS_MAP_AI_COUNT=0.");
        } else {
            const int startWeekdayMask =
                openbus::map::mapTrafficWeekdayMaskAtElapsedSeconds(mapAiStartWeekdayIndex, 0.0);
            const std::optional<double> groupDensity = openbus::map::mapAiRoadTrafficDensityAtHour(
                mapDefinition.aiTraffic, availableAiRoadVehicles, mapAiStartHour, startWeekdayMask);
            const std::optional<double> configuredDensity =
                groupDensity.has_value() ? groupDensity
                                         : openbus::map::mapTrafficDensityAtHour(
                                               mapDefinition.aiTraffic.roadDensity, mapAiStartHour);
            const double roadDensity =
                configuredDensity.value_or(1.0) * mapTrafficDensityMultiplier;
            const std::size_t targetRoadVehicleCount =
                mapTrafficPopulationForTestDensity(static_cast<std::size_t>(mapAiBaseCount),
                                                   configuredDensity, mapTrafficDensityMultiplier);
            const bool hasGroupDensitySchedule =
                mapDefinition.aiTraffic.hasUnscheduledTrafficDensityFile;
            const std::vector<openbus::map::MapAiRoadTrafficGroupPopulation>
                roadVehiclePoolByGroup =
                    hasGroupDensitySchedule
                        ? openbus::map::mapAiMaximumRoadTrafficPoolByGroup(
                              mapDefinition.aiTraffic, availableAiRoadVehicles,
                              static_cast<std::size_t>(mapAiBaseCount), mapTrafficDensityMultiplier,
                              200)
                        : std::vector<openbus::map::MapAiRoadTrafficGroupPopulation>{};
            std::size_t roadVehiclePoolCount = 0;
            if (hasGroupDensitySchedule) {
                for (const auto& groupPopulation : roadVehiclePoolByGroup) {
                    roadVehiclePoolCount += groupPopulation.populationCount;
                }
            } else {
                roadVehiclePoolCount = maximumMapTrafficPopulation(
                    static_cast<std::size_t>(mapAiBaseCount), mapDefinition.aiTraffic.roadDensity,
                    mapTrafficDensityMultiplier);
            }
            const std::vector<openbus::map::MapAiRoadVehicle> aiRoadVehicles =
                openbus::map::mapAiRoadVehiclesForPool(mapDefinition.aiTraffic);
            std::unordered_map<std::size_t, std::vector<openbus::map::MapAiRoadVehicle>>
                poolVehiclesByGroup;
            for (const auto& vehicle : aiRoadVehicles) {
                if (vehicle.selectionWeight > 0.0) {
                    poolVehiclesByGroup[vehicle.groupIndex].push_back(vehicle);
                }
            }
            if (!configuredDensity.has_value()) {
                applicationLog.Log("Map road traffic density is unavailable at " +
                                   std::to_string(mapAiStartHour) + "h; using density factor 1.0.");
            }
            if (aiRoadVehicles.empty()) {
                applicationLog.Log(
                    "Map road traffic not spawned: supported NormalCar(s), Trucks, Commercial(s), "
                    "and Ambulance groups have no loadable .ovh/.bus entries.");
            } else if (!hasGroupDensitySchedule &&
                       !openbus::map::selectMapAiRoadVehicle(aiRoadVehicles, 1).has_value()) {
                applicationLog.Log(
                    "Map road traffic not spawned: unsched_trafficdens.txt has no positive "
                    "density for a supported group at the selected weekday and hour.");
            } else if (roadVehiclePoolCount == 0) {
                applicationLog.Log("Map road traffic schedule has no positive population.");
            } else {
                queueAdditionalMapRoadTraffic =
                    [&, roadVehiclePoolCount, targetRoadVehicleCount, roadDensity,
                     hasGroupDensitySchedule, roadVehiclePoolByGroup, aiRoadVehicles,
                     poolVehiclesByGroup, addedTrafficVehicles = std::size_t{0},
                     noCompatibleSpawnLaneCount = std::size_t{0},
                     rejectedTrafficRouteCount = std::size_t{0}, poolGroupCursor = std::size_t{0},
                     nextCandidateSeed = std::size_t{0},
                     lastSpawnLanes = std::vector<openbus::map::MapTrafficLaneStart>{},
                     hasObservedSpawnLanes = false, hasLoggedPoolState = false,
                     unavailableAiVehicles = std::unordered_set<std::string>{},
                     queuedTrafficByGroup =
                         std::unordered_map<std::size_t, std::size_t>{}]() mutable {
                        constexpr double trafficSpeedMetersPerSecond = 12.0;
                        if (addedTrafficVehicles >= roadVehiclePoolCount) {
                            return;
                        }
                        const auto roadSpawnLanes = renderer.trafficSpawnLanes();
                        if (hasObservedSpawnLanes && roadSpawnLanes == lastSpawnLanes) {
                            return;
                        }
                        hasObservedSpawnLanes = true;
                        lastSpawnLanes = roadSpawnLanes;
                        if (roadSpawnLanes.empty()) {
                            applicationLog.Log(
                                "Map road traffic not queued: no connected road lane with a "
                                "forward continuation is available yet.");
                            return;
                        }

                        const std::size_t previouslyAdded = addedTrafficVehicles;
                        const std::size_t remainingReservations =
                            roadVehiclePoolCount - addedTrafficVehicles;
                        for (std::size_t attempt = 0; attempt < remainingReservations * 4 &&
                                                      addedTrafficVehicles < roadVehiclePoolCount;
                             ++attempt) {
                            const std::size_t candidateSeed = ++nextCandidateSeed;
                            const std::vector<openbus::map::MapAiRoadVehicle>* selectionPool =
                                &aiRoadVehicles;
                            std::optional<std::size_t> requiredGroupIndex;
                            if (hasGroupDensitySchedule) {
                                for (std::size_t offset = 0; offset < roadVehiclePoolByGroup.size();
                                     ++offset) {
                                    const std::size_t index =
                                        (poolGroupCursor + offset) % roadVehiclePoolByGroup.size();
                                    const auto& groupTarget = roadVehiclePoolByGroup[index];
                                    if (queuedTrafficByGroup[groupTarget.groupIndex] <
                                        groupTarget.populationCount) {
                                        requiredGroupIndex = groupTarget.groupIndex;
                                        poolGroupCursor =
                                            (index + 1) % roadVehiclePoolByGroup.size();
                                        break;
                                    }
                                }
                                if (!requiredGroupIndex.has_value()) {
                                    break;
                                }
                                const auto groupVehicles =
                                    poolVehiclesByGroup.find(*requiredGroupIndex);
                                if (groupVehicles == poolVehiclesByGroup.end()) {
                                    continue;
                                }
                                selectionPool = &groupVehicles->second;
                            }
                            const std::optional<std::size_t> selectedVehicle =
                                openbus::map::selectMapAiRoadVehicle(*selectionPool, candidateSeed);
                            if (!selectedVehicle.has_value()) {
                                if (!hasGroupDensitySchedule) {
                                    break;
                                }
                                continue;
                            }
                            const openbus::map::MapAiRoadVehicle& roadVehicle =
                                (*selectionPool)[*selectedVehicle];
                            const std::string& vehiclePath = roadVehicle.assetPath;
                            if (unavailableAiVehicles.contains(vehiclePath)) {
                                continue;
                            }
                            const std::optional<std::string> driverPath =
                                mapDefinition.aiTraffic.driverModelPaths.empty()
                                    ? std::nullopt
                                    : std::optional<std::string>(
                                          mapDefinition.aiTraffic.driverModelPaths
                                              [addedTrafficVehicles %
                                               mapDefinition.aiTraffic.driverModelPaths.size()]);
                            try {
                                const int vehicleType = renderer.trafficVehicleType(
                                    vehiclePath, roadVehicle.defaultVehicleType);
                                const auto lane = renderer.selectTrafficSpawnLane(
                                    {vehicleType, roadVehicle.groupIndex, false,
                                     roadVehicle.trafficRuleGroupIndex},
                                    candidateSeed);
                                if (!lane.has_value()) {
                                    ++noCompatibleSpawnLaneCount;
                                    continue;
                                }
                                if (renderer.AddTrafficVehicle(
                                        lane->splineId, openbus::map::MAP_SPLINE_PATH_TYPE_ROAD,
                                        vehiclePath, driverPath, trafficSpeedMetersPerSecond,
                                        {AssetLoadingMode::Deferred, AssetLoadingMode::Deferred},
                                        false, lane->laneSelector, candidateSeed, {},
                                        roadVehicle.groupIndex, roadVehicle.defaultVehicleType, {},
                                        {}, false, false, false, {}, {}, {},
                                        roadVehicle.trafficRuleGroupIndex)) {
                                    ++addedTrafficVehicles;
                                    ++queuedTrafficByGroup[roadVehicle.groupIndex];
                                } else {
                                    ++rejectedTrafficRouteCount;
                                }
                            } catch (const std::exception& error) {
                                unavailableAiVehicles.insert(vehiclePath);
                                applicationLog.Log("Skipping map road vehicle '" + vehiclePath +
                                                   "': " + error.what());
                            }
                        }
                        if (previouslyAdded == addedTrafficVehicles && hasLoggedPoolState) {
                            return;
                        }
                        hasLoggedPoolState = true;
                        applicationLog.Log(
                            "Map road AI vehicle pool queued: " +
                            std::to_string(addedTrafficVehicles) + " of " +
                            std::to_string(roadVehiclePoolCount) +
                            " reserved, initial target active=" +
                            std::to_string(targetRoadVehicleCount) +
                            ", density=" + std::to_string(roadDensity) +
                            ", connected spawn lanes=" + std::to_string(roadSpawnLanes.size()) +
                            ", attempts without a compatible lane=" +
                            std::to_string(noCompatibleSpawnLaneCount) +
                            ", selected lanes with rejected routes=" +
                            std::to_string(rejectedTrafficRouteCount) +
                            ". Models and scripts load asynchronously; road vehicles wait for a "
                            "clear, off-screen position in a currently active tile. Scheduled "
                            "buses retain endpoint-only admission.");
                    };
                queueAdditionalMapRoadTraffic();
            }
        }

        if (mapAiBusPoolCount == 0) {
            applicationLog.Log("Map AI buses have no active population.");
        }
        {
            const std::vector<std::string> aiBusConfigurations =
                openbus::map::mapAiBusAssetPaths(mapDefinition.aiTraffic);
            if (aiBusConfigurations.empty()) {
                applicationLog.Log(
                    "Map AI buses not spawned: ailists.cfg has no loadable depot bus "
                    "configurations.");
            } else {
                constexpr double busTrafficSpeedMetersPerSecond = 11.0;
                std::size_t addedBuses = 0;
                std::size_t eligibleTimetableRoutes = 0;
                std::size_t feasibleRoutesExamined = 0;
                std::size_t sharedTimetableSplineIds = 0;
                std::size_t longestCommonOrderedRun = 0;
                std::string bestOverlapTimetableLine;
                std::string bestOverlapDestination;
                openbus::map::MapTrafficLaneStart bestOverlapEntry;
                std::size_t bestOverlapRouteSeed = 0;
                std::size_t bestOverlapRouteSplineCount = 0;
                std::size_t bestOverlapTimetableSplineCount = 0;
                std::size_t bestOverlapTimetablePathOffset = 0;
                std::size_t bestOverlapRoutePathOffset = 0;
                std::optional<int> nextOverlapTimetableSplineId;
                std::optional<int> nextOverlapRouteSplineId;
                std::vector<int> bestOverlapSplineIds;
                std::unordered_set<std::string> unavailableAiBuses;
                std::unordered_set<std::string> loggedTimetableOriginDiagnostics;
                std::unordered_map<std::string, MapBusLineRouteSearch> busEntryLanes;
                struct MatchedBusRoute {
                    std::string busPath;
                    MapBusLineRouteStart routeStart;
                };
                std::vector<MatchedBusRoute> matchedBusRoutes;
                std::unordered_set<std::size_t> assignedTimetableRoutes;
                for (const std::string& busPath : aiBusConfigurations) {
                    if (unavailableAiBuses.contains(busPath)) {
                        continue;
                    }
                    auto entries = busEntryLanes.find(busPath);
                    if (entries == busEntryLanes.end()) {
                        try {
                            const double turnRadius =
                                renderer.trafficMinimumTurnRadius(busPath, true);
                            auto search = mapBusLineRouteStarts(renderer, mapDefinition.aiTraffic,
                                                                turnRadius);
                            for (const std::string& diagnostic :
                                 search.timetableOriginDiagnostics) {
                                if (loggedTimetableOriginDiagnostics.insert(diagnostic).second) {
                                    applicationLog.Log(diagnostic);
                                }
                            }
                            feasibleRoutesExamined += search.feasibleRoutesExamined;
                            sharedTimetableSplineIds += search.sharedTimetableSplineIds;
                            if (search.longestCommonOrderedRun > longestCommonOrderedRun) {
                                longestCommonOrderedRun = search.longestCommonOrderedRun;
                                bestOverlapTimetableLine = search.bestTimetableLineName;
                                bestOverlapDestination = search.bestTimetableDestinationName;
                                bestOverlapEntry = search.bestEntry;
                                bestOverlapRouteSeed = search.bestRouteSeed;
                                bestOverlapRouteSplineCount = search.bestRouteSplineCount;
                                bestOverlapTimetableSplineCount = search.bestTimetableSplineCount;
                                bestOverlapTimetablePathOffset = search.bestTimetablePathOffset;
                                bestOverlapRoutePathOffset = search.bestRoutePathOffset;
                                nextOverlapTimetableSplineId = search.nextTimetableSplineId;
                                nextOverlapRouteSplineId = search.nextRouteSplineId;
                                bestOverlapSplineIds = std::move(search.bestCommonSplineIds);
                            }
                            entries = busEntryLanes.emplace(busPath, std::move(search)).first;
                        } catch (const std::exception& error) {
                            unavailableAiBuses.insert(busPath);
                            applicationLog.Log("Skipping map AI bus '" + busPath +
                                               "': " + error.what());
                            continue;
                        }
                    }
                    if (entries->second.starts.empty()) {
                        continue;
                    }
                    for (const MapBusLineRouteStart& routeStart : entries->second.starts) {
                        if (assignedTimetableRoutes.insert(routeStart.timetableRouteIndex).second) {
                            matchedBusRoutes.push_back({busPath, routeStart});
                        }
                    }
                }
                if (!matchedBusRoutes.empty()) {
                    applicationLog.Log("Scheduled bus route search found " +
                                       std::to_string(matchedBusRoutes.size()) +
                                       " complete timetable route(s).");
                }
                for (const MatchedBusRoute& matchedRoute : matchedBusRoutes) {
                    const auto& timetable =
                        mapDefinition.aiTraffic
                            .timetableRoutes[matchedRoute.routeStart.timetableRouteIndex];
                    applicationLog.Log("Scheduled bus route " + timetable.lineName + " to " +
                                       timetable.destinationName + " (" + timetable.tripName +
                                       ") starts at " +
                                       std::to_string(matchedRoute.routeStart.entry.splineId) +
                                       " with a complete timetable match.");
                }
                eligibleTimetableRoutes = matchedBusRoutes.size();
                const std::size_t busPoolLimit = mapAiBusPoolCount;
                std::vector<std::size_t> departureCounts;
                departureCounts.reserve(matchedBusRoutes.size());
                for (const MatchedBusRoute& matchedRoute : matchedBusRoutes) {
                    departureCounts.push_back(
                        mapDefinition.aiTraffic
                            .timetableRoutes[matchedRoute.routeStart.timetableRouteIndex]
                            .scheduledDepartureMinutes.size());
                }
                const std::vector<std::size_t> busesPerRoute =
                    openbus::map::mapTrafficAllocateScheduledBuses(departureCounts, busPoolLimit);
                for (std::size_t routeIndex = 0; routeIndex < matchedBusRoutes.size();
                     ++routeIndex) {
                    const MatchedBusRoute& matchedRoute = matchedBusRoutes[routeIndex];
                    const auto& timetable =
                        mapDefinition.aiTraffic
                            .timetableRoutes[matchedRoute.routeStart.timetableRouteIndex];
                    std::vector<openbus::map::MapTrafficPassengerStop> passengerStops;
                    passengerStops.reserve(timetable.stations.size());
                    for (const openbus::map::MapAiTimetableRoute::Station& station :
                         timetable.stations) {
                        const auto stationPosition =
                            openbus::map::mapAiTimetableStationPosition(timetable, station);
                        if (!stationPosition) {
                            applicationLog.Log("Skipping passenger stop with an invalid timetable "
                                               "track position: " +
                                               station.name + " in " + timetable.timetableKey);
                            continue;
                        }
                        const auto& trackEntry =
                            timetable.trackEntries[stationPosition->trackEntryIndex];
                        if (trackEntry.pathIndex < 0) {
                            continue;
                        }
                        passengerStops.push_back(
                            {station.id, trackEntry.splineId,
                             static_cast<std::size_t>(trackEntry.pathIndex),
                             stationPosition->trackEntryIndex,
                             openbus::map::mapScheduledPassengerQueueSize(4, scheduledHumanScale),
                             0, 8.0, 0.0,
                             station.positionOnPreviousTrackEntryMeters
                                 ? std::optional<double>(stationPosition->offsetMeters)
                                 : std::nullopt});
                    }
                    std::size_t passengerRouteIndex = 0;
                    try {
                        if (!renderer.AddTrafficVehicle(
                                matchedRoute.routeStart.entry.splineId,
                                openbus::map::MAP_SPLINE_PATH_TYPE_ROAD, matchedRoute.busPath,
                                std::nullopt, busTrafficSpeedMetersPerSecond,
                                {AssetLoadingMode::Deferred, AssetLoadingMode::Deferred}, true,
                                matchedRoute.routeStart.entry.laneSelector,
                                matchedRoute.routeStart.routeSeed, {}, std::nullopt, 0,
                                matchedRoute.routeStart.requiredSplineSequence,
                                matchedRoute.routeStart.requiredSceneryPathSequence,
                                matchedRoute.routeStart.stopAtRequiredSequenceEnd,
                                matchedRoute.routeStart.allowIntermediarySegmentsBetweenRequired,
                                matchedRoute.routeStart.allowReverseTraversalForRequired,
                                passengerStops, timetable.lineName, timetable.destinationName,
                                std::nullopt, true, std::nullopt, &passengerRouteIndex)) {
                            applicationLog.Log(
                                "Player passenger stops are unavailable for timetable route " +
                                timetable.lineName +
                                " because its mapped road route could not "
                                "be registered.");
                            continue;
                        }
                    } catch (const std::exception& error) {
                        applicationLog.Log(
                            "Player passenger stops are unavailable for timetable route " +
                            timetable.lineName + ": " + error.what());
                        continue;
                    }
                    if (busesPerRoute[routeIndex] == 0) {
                        continue;
                    }
                    const auto departurePartitions =
                        openbus::map::mapTrafficPartitionScheduledDepartures(
                            timetable.scheduledDepartureMinutes, busesPerRoute[routeIndex]);
                    for (const auto& departures : departurePartitions) {
                        if (addedBuses >= busPoolLimit) {
                            break;
                        }
                        const std::optional<std::string> driverPath =
                            mapDefinition.aiTraffic.driverModelPaths.empty()
                                ? std::nullopt
                                : std::optional<std::string>(
                                      mapDefinition.aiTraffic.driverModelPaths
                                          [addedBuses %
                                           mapDefinition.aiTraffic.driverModelPaths.size()]);
                        try {
                            if (renderer.AddTrafficVehicle(
                                    matchedRoute.routeStart.entry.splineId,
                                    openbus::map::MAP_SPLINE_PATH_TYPE_ROAD, matchedRoute.busPath,
                                    driverPath, busTrafficSpeedMetersPerSecond,
                                    {AssetLoadingMode::Deferred, AssetLoadingMode::Deferred}, true,
                                    matchedRoute.routeStart.entry.laneSelector,
                                    matchedRoute.routeStart.routeSeed, departures, std::nullopt, 0,
                                    matchedRoute.routeStart.requiredSplineSequence,
                                    matchedRoute.routeStart.requiredSceneryPathSequence,
                                    matchedRoute.routeStart.stopAtRequiredSequenceEnd,
                                    matchedRoute.routeStart
                                        .allowIntermediarySegmentsBetweenRequired,
                                    matchedRoute.routeStart.allowReverseTraversalForRequired,
                                    passengerStops, timetable.lineName, timetable.destinationName,
                                    std::nullopt, false, passengerRouteIndex)) {
                                ++addedBuses;
                            } else {
                                applicationLog.Log(
                                    "Skipping scheduled map AI bus: matched timetable route could "
                                    "not be rebuilt from its endpoint entry.");
                            }
                        } catch (const std::exception& error) {
                            applicationLog.Log("Skipping map AI bus '" + matchedRoute.busPath +
                                               "': " + error.what());
                            break;
                        }
                    }
                }
                if (eligibleTimetableRoutes == 0) {
                    std::string routeSearchSummary =
                        "Map AI buses not spawned: no timetable trip with station data maps to "
                        "a feasible route of at least two road splines from an endpoint. "
                        "Searched " +
                        std::to_string(feasibleRoutesExamined) + " feasible endpoint routes; " +
                        std::to_string(sharedTimetableSplineIds) +
                        " distinct spline IDs overlap with timetable paths; the longest ordered "
                        "contiguous match was " +
                        std::to_string(longestCommonOrderedRun) + " spline IDs.";
                    if (!bestOverlapTimetableLine.empty()) {
                        routeSearchSummary +=
                            " Best overlap: line '" + bestOverlapTimetableLine + "' to '" +
                            bestOverlapDestination +
                            "', endpoint spline=" + std::to_string(bestOverlapEntry.splineId) +
                            ", lane=" + std::to_string(bestOverlapEntry.laneSelector) +
                            ", route seed=" + std::to_string(bestOverlapRouteSeed) +
                            ", timetable/route lengths=" +
                            std::to_string(bestOverlapTimetableSplineCount) + "/" +
                            std::to_string(bestOverlapRouteSplineCount) + ", common-path offsets=" +
                            std::to_string(bestOverlapTimetablePathOffset) + "/" +
                            std::to_string(bestOverlapRoutePathOffset) +
                            ", common splines=" + formatSplineIds(bestOverlapSplineIds) + ".";
                        if (nextOverlapTimetableSplineId.has_value() ||
                            nextOverlapRouteSplineId.has_value()) {
                            routeSearchSummary +=
                                " Next spline after overlap (timetable/route)=" +
                                (nextOverlapTimetableSplineId.has_value()
                                     ? std::to_string(*nextOverlapTimetableSplineId)
                                     : std::string("end")) +
                                "/" +
                                (nextOverlapRouteSplineId.has_value()
                                     ? std::to_string(*nextOverlapRouteSplineId)
                                     : std::string("end")) +
                                ".";
                        }
                    }
                    applicationLog.Log(routeSearchSummary);
                } else {
                    applicationLog.Log(
                        "Map AI buses queued at endpoint entries: " + std::to_string(addedBuses) +
                        " of " + std::to_string(mapAiBusPoolCount) + " requested across " +
                        std::to_string(eligibleTimetableRoutes) +
                        " matched timetable routes with departures. Multiple buses on a route "
                        "receive disjoint departure subsets; models and scripts load "
                        "asynchronously.");
                }
            }
        }
        const std::vector<std::string>& configuredHumanModelPaths =
            mapDefinition.aiTraffic.humanModelPaths;
        std::vector<std::filesystem::path> scheduledPassengerHumanModels;
        for (const std::string& configuredPath : configuredHumanModelPaths) {
            std::string normalizedPath = configuredPath;
            std::replace(normalizedPath.begin(), normalizedPath.end(), '\\', '/');
            const std::filesystem::path humanConfigPath =
                openbus::assets::resolveCaseInsensitivePath(omsiRootPath() / normalizedPath);
            if (std::filesystem::is_regular_file(humanConfigPath)) {
                scheduledPassengerHumanModels.push_back(humanConfigPath);
            } else {
                applicationLog.Log("Skipping missing scheduled passenger human model "
                                   "configuration '" +
                                   humanConfigPath.generic_string() + "'.");
            }
        }
        renderer.SetScheduledPassengerHumanModels(std::move(scheduledPassengerHumanModels));
        if (mapHumanBaseCount == 0) {
            applicationLog.Log("Map human traffic is disabled by OPENBUS_MAP_HUMAN_COUNT=0.");
        } else if (configuredHumanModelPaths.empty()) {
            applicationLog.Log(
                "Map human traffic not spawned: humans.txt contains no .hum model paths.");
        } else {
            const std::optional<double> configuredDensity = openbus::map::mapTrafficDensityAtHour(
                mapDefinition.aiTraffic.passengerDensity, mapAiStartHour);
            const std::size_t targetHumanCount =
                mapTrafficPopulationForTestDensity(static_cast<std::size_t>(mapHumanBaseCount),
                                                   configuredDensity, mapTrafficDensityMultiplier);
            const std::size_t humanPoolCount = maximumMapTrafficPopulation(
                static_cast<std::size_t>(mapHumanBaseCount),
                mapDefinition.aiTraffic.passengerDensity, mapTrafficDensityMultiplier);
            const std::vector<int> humanSplineIds =
                renderer.trafficSplineIds(openbus::map::MAP_SPLINE_PATH_TYPE_HUMAN);
            struct HumanModel {
                std::filesystem::path configPath;
                double feetDistanceMeters = 0.0;
                double heightMeters = 0.0;
                double walkStrideLengthMeters = 1.4;
            };
            std::vector<HumanModel> humanModels;
            for (const std::string& configuredPath : configuredHumanModelPaths) {
                std::string normalizedPath = configuredPath;
                std::replace(normalizedPath.begin(), normalizedPath.end(), '\\', '/');
                const std::filesystem::path humanConfigPath =
                    openbus::assets::resolveCaseInsensitivePath(omsiRootPath() / normalizedPath);
                if (!std::filesystem::is_regular_file(humanConfigPath)) {
                    applicationLog.Log("Skipping missing human model configuration '" +
                                       configuredPath + "'.");
                    continue;
                }
                try {
                    const openbus::config::HumanConfig humanConfig =
                        openbus::config::loadHumanConfig(humanConfigPath);
                    if (humanConfig.diagnostics.hasErrors()) {
                        applicationLog.Log("Skipping invalid human configuration '" +
                                           humanConfigPath.generic_string() + "'.");
                        continue;
                    }
                    const auto geometry = std::find_if(
                        humanConfig.numericRecords.begin(), humanConfig.numericRecords.end(),
                        [](const openbus::config::HumanNumericRecord& record) {
                            return record.keyword == "humangeom" && record.valid &&
                                   record.values.size() == 2;
                        });
                    if (geometry == humanConfig.numericRecords.end() ||
                        !std::isfinite(geometry->values[0]) || geometry->values[0] <= 0.0 ||
                        !std::isfinite(geometry->values[1]) || geometry->values[1] <= 0.0) {
                        applicationLog.Log("Skipping human configuration without valid "
                                           "[humangeom] dimensions: " +
                                           humanConfigPath.generic_string() + ".");
                        continue;
                    }
                    humanModels.push_back({humanConfigPath, geometry->values[0],
                                           geometry->values[1],
                                           humanConfig.walkStrideLengthMeters});
                } catch (const std::exception& error) {
                    applicationLog.Log("Skipping human configuration '" +
                                       humanConfigPath.generic_string() + "': " + error.what());
                }
            }
            if (!configuredDensity.has_value()) {
                applicationLog.Log("Map passenger traffic density is unavailable at " +
                                   std::to_string(mapAiStartHour) + "h; using density factor 1.0.");
            }
            if (humanSplineIds.empty()) {
                applicationLog.Log("Map human traffic not spawned: no usable human-path splines "
                                   "([path] type 1) were found.");
            } else if (humanModels.empty()) {
                applicationLog.Log("Map human traffic not spawned: no valid .hum configurations "
                                   "were available.");
            } else if (humanPoolCount == 0) {
                applicationLog.Log("Map passenger traffic schedule has no positive population.");
            } else {
                constexpr double humanSpeedMetersPerSecond = 1.2;
                constexpr double humanSpacingMeters = 8.0;
                constexpr double trafficRadiusMeters = 750.0;
                std::size_t addedHumans = 0;
                for (std::size_t candidate = 0;
                     candidate < humanPoolCount * 4 && addedHumans < humanPoolCount; ++candidate) {
                    const HumanModel& humanModel = humanModels[addedHumans % humanModels.size()];
                    try {
                        if (renderer.AddHumanTrafficActor(
                                humanSplineIds[candidate % humanSplineIds.size()],
                                humanModel.configPath, humanModel.feetDistanceMeters,
                                humanModel.heightMeters, humanModel.walkStrideLengthMeters,
                                humanSpeedMetersPerSecond,
                                static_cast<double>(candidate) * humanSpacingMeters,
                                busPlacement.position[0], busPlacement.position[1],
                                trafficRadiusMeters)) {
                            ++addedHumans;
                        }
                    } catch (const std::exception& error) {
                        applicationLog.Log("Skipping human traffic actor '" +
                                           humanModel.configPath.generic_string() +
                                           "': " + error.what());
                    }
                }
                applicationLog.Log(
                    "Map human traffic pool queued: " + std::to_string(addedHumans) + " of " +
                    std::to_string(humanPoolCount) + " maximum scheduled, active at startup=" +
                    std::to_string(targetHumanCount) + ", density=" +
                    std::to_string(configuredDensity.value_or(1.0) * mapTrafficDensityMultiplier) +
                    ", human paths=" + std::to_string(humanSplineIds.size()) +
                    ", model roster=humans.txt" +
                    ". Model meshes parse asynchronously; graphics upload occurs on the render "
                    "thread.");
            }
        }

        const auto updateMapTrafficPopulation = [&] {
            const double trafficHour = openbus::map::mapTrafficHourAtElapsedSeconds(
                mapAiStartHour, simulation.simulationTime());
            const int trafficWeekdayMask = openbus::map::mapTrafficWeekdayMaskAtElapsedSeconds(
                mapAiStartWeekdayIndex, simulation.simulationTime());
            const std::optional<double> groupDensity = openbus::map::mapAiRoadTrafficDensityAtHour(
                mapDefinition.aiTraffic, availableAiRoadVehicles, trafficHour, trafficWeekdayMask);
            const std::optional<double> roadDensity =
                groupDensity.has_value() ? groupDensity
                                         : openbus::map::mapTrafficDensityAtHour(
                                               mapDefinition.aiTraffic.roadDensity, trafficHour);
            const std::optional<double> passengerDensity = openbus::map::mapTrafficDensityAtHour(
                mapDefinition.aiTraffic.passengerDensity, trafficHour);
            const std::size_t targetRoadVehicleCount = mapTrafficPopulationForTestDensity(
                static_cast<std::size_t>(mapAiBaseCount), roadDensity, mapTrafficDensityMultiplier);
            renderer.SetMapTrafficPopulation(
                targetRoadVehicleCount, mapAiBusPoolCount,
                groupDensity.has_value()
                    ? openbus::map::mapAiRoadTrafficPopulationByGroup(
                          mapDefinition.aiTraffic, targetRoadVehicleCount, trafficHour,
                          trafficWeekdayMask)
                    : std::vector<openbus::map::MapAiRoadTrafficGroupPopulation>{},
                groupDensity.has_value());
            renderer.SetHumanTrafficPopulation(
                mapTrafficPopulationForTestDensity(static_cast<std::size_t>(mapHumanBaseCount),
                                                   passengerDensity, mapTrafficDensityMultiplier));
        };
        applicationLog.Log("Map traffic clock starts at " + std::to_string(mapAiStartHour) +
                           "h on " + std::to_string(mapCalendarYear) + "-" +
                           std::to_string(mapCalendarMonth) + "-" + std::to_string(mapCalendarDay) +
                           "; time and weekday advance from the map system calendar.");
        updateMapTrafficPopulation();

        double previousTime = glfwGetTime();
        if (benchmarkMode) {
            constexpr double benchmarkTimeStep = 1.0 / 60.0;
            constexpr std::array<RenderBenchmarkPhase, 5> phases = {
                RenderBenchmarkPhase::Baseline, RenderBenchmarkPhase::CameraCycle,
                RenderBenchmarkPhase::ThirdPersonZoom, RenderBenchmarkPhase::DrivingControls,
                RenderBenchmarkPhase::DashboardInteraction};
            const int warmupFrames =
                positiveEnvironmentInt("OPENBUS_BENCHMARK_WARMUP_FRAMES", 60, 10000);
            const int phaseFrames =
                positiveEnvironmentInt("OPENBUS_BENCHMARK_PHASE_FRAMES", 120, 100000);
            const int phaseRepeats =
                positiveEnvironmentInt("OPENBUS_BENCHMARK_PHASE_REPEATS", 5, 100);
            const int readyTimeoutSeconds =
                positiveEnvironmentInt("OPENBUS_BENCHMARK_READY_TIMEOUT", 180, 3600);
            const auto renderBenchmarkFrame = [&](RenderBenchmarkPhase phase, int frameInPhase,
                                                  int framesInPhase) {
                openbus::rendering::TraceScope frameTrace("frame", "main");
                renderer.beginFrame(benchmarkTimeStep);
                const RenderBenchmarkInput input =
                    renderer.setBenchmarkFrame(phase, frameInPhase, framesInPhase);
                renderer.updatePlayerVariables(simulation, input.throttle, input.steering,
                                               input.brake);
                simulation.updateWithWheelTorqueAndBrakeForces(
                    benchmarkTimeStep, renderer.physicsWheelTorque(), renderer.physicsSteering(),
                    renderer.physicsWheelBrakeForces(simulation.axleCount()),
                    renderer.physicsAxleSpringFactors(simulation.axleCount()));
                updateMapTrafficPopulation();
                renderer.updatePostPhysicsVariables(simulation);
                renderer.updateMapStreaming(simulation.positionX(), simulation.positionY());
                const auto loadedTiles = renderer.loadedMapTiles();
                if (loadedTiles != roadTrafficPoolLoadedTiles) {
                    roadTrafficPoolLoadedTiles = loadedTiles;
                    if (queueAdditionalMapRoadTraffic) {
                        queueAdditionalMapRoadTraffic();
                    }
                }
                collisionStreamer.update(simulation.positionX(), simulation.positionY(),
                                         loadedTiles);
                renderer.updateMapPoleStates(simulation.mapPoleStates());
                const std::size_t collisionTileCount = collisionStreamer.loadedTileCount();
                if (collisionTileCount != reportedCollisionTileCount) {
                    applicationLog.Log("Map collision streaming progress: resident tiles=" +
                                       std::to_string(collisionTileCount) + ".");
                    reportedCollisionTileCount = collisionTileCount;
                }
                renderer.draw(simulation);
                renderer.endFrame();
            };

            const auto readyDeadline =
                std::chrono::steady_clock::now() + std::chrono::seconds(readyTimeoutSeconds);
            int readinessFrames = 0;
            while (!renderer.isCaptureReady() && !renderer.shouldClose() &&
                   std::chrono::steady_clock::now() < readyDeadline) {
                renderBenchmarkFrame(RenderBenchmarkPhase::Baseline, 0, 1);
                ++readinessFrames;
            }
            if (!renderer.isCaptureReady()) {
                throw std::runtime_error("Benchmark timed out waiting for vehicle assets");
            }

            const auto windowDimensions = renderer.windowSize();
            const auto framebufferDimensions = renderer.framebufferSize();
            std::cout << "BENCHMARK_REQUESTED=" << windowWidth << 'x' << windowHeight << '\n';
            std::cout << "BENCHMARK_WINDOW=" << windowDimensions[0] << 'x' << windowDimensions[1]
                      << '\n';
            std::cout << "BENCHMARK_FRAMEBUFFER=" << framebufferDimensions[0] << 'x'
                      << framebufferDimensions[1] << '\n';
            std::cout << "BENCHMARK_PHASE_REPEATS=" << phaseRepeats << '\n';
            std::cout << "BENCHMARK_READINESS_FRAMES=" << readinessFrames << '\n';

            const bool requireExactResolution = openbus::rendering::parseEnabledFlag(
                openbus::getEnvironment("OPENBUS_BENCHMARK_REQUIRE_EXACT_RESOLUTION"));
            if (requireExactResolution && (framebufferDimensions[0] != windowWidth ||
                                           framebufferDimensions[1] != windowHeight)) {
                std::cout << "BENCHMARK_SKIPPED=resolution_mismatch\n";
                std::cout << "BENCHMARK_COMPLETE=1\n";
                applicationLog.Log("Benchmark skipped because the requested framebuffer size "
                                   "is unavailable");
                Logger::Flush();
                openbus::rendering::Flush();
                return 0;
            }

            const int warmupFramesPerPhase = warmupFrames / static_cast<int>(phases.size());
            const int extraWarmupFrames = warmupFrames % static_cast<int>(phases.size());
            for (std::size_t phaseIndex = 0; phaseIndex < phases.size(); ++phaseIndex) {
                const int framesThisPhase =
                    warmupFramesPerPhase +
                    (static_cast<int>(phaseIndex) < extraWarmupFrames ? 1 : 0);
                for (int frame = 0; frame < framesThisPhase; ++frame) {
                    renderBenchmarkFrame(phases[phaseIndex], frame, framesThisPhase);
                }
            }

            {
                openbus::rendering::TraceScope measurement("benchmark", "Benchmark.measure");
                for (int repeat = 0; repeat < phaseRepeats; ++repeat) {
                    for (const RenderBenchmarkPhase phase : phases) {
                        openbus::rendering::TraceScope phaseTrace("benchmark",
                                                                  benchmarkPhaseName(phase));
                        for (int frame = 0; frame < phaseFrames; ++frame) {
                            renderBenchmarkFrame(phase, frame, phaseFrames);
                        }
                    }
                }
            }
            std::cout << "BENCHMARK_CLICK_TARGET="
                      << (renderer.benchmarkClickTargetFound() ? "found" : "none") << '\n';
            std::cout << "BENCHMARK_COMPLETE=1\n";
        } else {
            bool captureOnStartup = openbus::rendering::parseEnabledFlag(
                openbus::getEnvironment("OPENBUS_CAPTURE_VIEWS"));
            bool pendingCaptureRequest = captureOnStartup;
            std::vector<KeyEvent> pendingKeyEvents;

            while (!renderer.shouldClose()) {
                openbus::rendering::TraceScope frameTrace("frame", "main");

                // Each frame updates input-backed variables first, advances physics,
                // then renders using the resulting simulation and variable state.
                const double currentTime = glfwGetTime();
                const double elapsed = currentTime - previousTime;
                previousTime = currentTime;

                renderer.beginFrame();
                renderer.updateOnFoot(simulation, elapsed);
                if (renderer.consumeSpawnBusRequest() && playerVehicle == nullptr) {
                    applicationLog.Log("Spawning player bus: " + busConfigPath.string());
                    playerVehicle =
                        renderer.AddVehicle(busConfigPath, modelConfigPath, busPlacement,
                                            {AssetLoadingMode::Deferred, AssetLoadingMode::Eager},
                                            false, startupSelection.vehicleSetup);
                    renderer.SetPlayerVehicle(playerVehicle);
                    simulation.setFrozen(false);
                    applicationLog.Log("Player bus spawned; physics simulation resumed.");
                }
                if (renderer.consumeRaisePlayerRequest()) {
                    simulation.translateVertically(5.0);
                    applicationLog.Log("Raised player vehicle 5 m above its current location");
                }

                const std::vector<KeyEvent> frameKeyEvents = renderer.consumeKeyEvents();
                pendingKeyEvents.insert(pendingKeyEvents.end(), frameKeyEvents.begin(),
                                        frameKeyEvents.end());
                renderer.updatePlayerVariables(simulation, renderer.throttle(), renderer.steering(),
                                               renderer.brake());
                simulation.updateWithWheelTorqueAndBrakeForces(
                    elapsed, renderer.physicsWheelTorque(), renderer.physicsSteering(),
                    renderer.physicsWheelBrakeForces(simulation.axleCount()),
                    renderer.physicsAxleSpringFactors(simulation.axleCount()));
                updateMapTrafficPopulation();
                renderer.updatePostPhysicsVariables(simulation);
                const auto streamingPosition = renderer.streamingPosition(simulation);
                renderer.updateMapStreaming(streamingPosition[0], streamingPosition[1]);
                const auto loadedTiles = renderer.loadedMapTiles();
                if (loadedTiles != roadTrafficPoolLoadedTiles) {
                    roadTrafficPoolLoadedTiles = loadedTiles;
                    if (queueAdditionalMapRoadTraffic) {
                        queueAdditionalMapRoadTraffic();
                    }
                }
                collisionStreamer.update(streamingPosition[0], streamingPosition[1], loadedTiles);
                renderer.updateMapPoleStates(simulation.mapPoleStates());
                const std::size_t collisionTileCount = collisionStreamer.loadedTileCount();
                if (collisionTileCount != reportedCollisionTileCount) {
                    applicationLog.Log("Map collision streaming progress: resident tiles=" +
                                       std::to_string(collisionTileCount) + ".");
                    reportedCollisionTileCount = collisionTileCount;
                }
                renderer.draw(simulation);

                if (renderer.consumeCaptureRequest()) {
                    pendingCaptureRequest = true;
                }

                if (pendingCaptureRequest && renderer.isCaptureReady()) {
                    pendingCaptureRequest = false;
                    captureOnStartup = false;
                    renderer.captureViews(simulation, "screenshots");
                    renderer.requestClose();
                }

                renderer.endFrame();
            }
        }
        applicationLog.Log("OpenBus shut down cleanly");
    } catch (const std::exception& error) {
        applicationLog.Log(std::string("OpenBus failed: ") + error.what());
        std::cerr << "OpenBus failed: " << error.what() << '\n';
        Logger::Flush();
        openbus::rendering::Flush();
        return 1;
    }
    Logger::Flush();
    openbus::rendering::Flush();
    return 0;
}
