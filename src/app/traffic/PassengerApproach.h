#pragma once

#include "map/MapTrafficRuntime.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace openbus::app {

inline constexpr double passengerApproachSpeedMetersPerSecond = 1.4;
inline constexpr double passengerSeatTransitionPortion = 0.2;
inline constexpr double playerPassengerStopArrivalRadiusMeters = 20.0;
inline constexpr double playerPassengerStopReleaseRadiusMeters = 25.0;

inline std::size_t passengerCoinVisualBurstCount(double previousCount, double currentCount) {
    if (!std::isfinite(previousCount) || !std::isfinite(currentCount) || previousCount < 0.0 ||
        currentCount < 0.0) {
        throw std::invalid_argument("Passenger coin counters must be finite and non-negative");
    }
    if (currentCount < previousCount) {
        return 0;
    }
    constexpr double maximumVisualBurst = 4.0;
    const double addedCoins = std::floor(currentCount) - std::floor(previousCount);
    return static_cast<std::size_t>(std::min(addedCoins, maximumVisualBurst));
}

inline double passengerCabinHeading(double vehicleHeadingRadians, double seatRotationDegrees) {
    return vehicleHeadingRadians + seatRotationDegrees * std::numbers::pi / 180.0;
}

struct PassengerSeatTransition {
    double pathProgress = 0.0;
    double seatedFraction = 0.0;
};

inline PassengerSeatTransition passengerSeatTransition(double animationProgress, bool boarding,
                                                       bool hasSeat) {
    const double progress = std::clamp(animationProgress, 0.0, 1.0);
    if (!hasSeat) {
        return {progress, 0.0};
    }
    constexpr double transitionPortion = passengerSeatTransitionPortion;
    if (boarding) {
        return {std::min(progress / (1.0 - transitionPortion), 1.0),
                std::clamp((progress - (1.0 - transitionPortion)) / transitionPortion, 0.0, 1.0)};
    }
    return {std::clamp((progress - transitionPortion) / (1.0 - transitionPortion), 0.0, 1.0),
            std::clamp(1.0 - progress / transitionPortion, 0.0, 1.0)};
}

inline std::vector<std::pair<double, std::optional<std::size_t>>>
passengerSeatPathLinkProfile(std::vector<std::pair<double, std::optional<std::size_t>>> profile,
                             bool boarding, bool hasSeat) {
    if (!hasSeat || profile.empty()) {
        return profile;
    }
    constexpr double transitionPortion = passengerSeatTransitionPortion;
    for (auto& segment : profile) {
        segment.first = boarding ? segment.first * (1.0 - transitionPortion)
                                 : transitionPortion + segment.first * (1.0 - transitionPortion);
    }
    if (boarding) {
        profile.emplace_back(1.0, std::nullopt);
    } else {
        profile.back().first = 1.0;
        profile.insert(profile.begin(), {transitionPortion, std::nullopt});
    }
    return profile;
}

inline std::array<double, 3> passengerCabinModelPoint(const std::array<double, 3>& point) {
    return {point[1], -point[0], point[2]};
}

inline std::array<double, 3> passengerCabinStandingPoint(const std::array<double, 3>& seat,
                                                         double seatHeight) {
    return {seat[0], seat[1], seat[2] - seatHeight};
}

inline openbus::map::MapTrafficPose
passengerDoorQueuePose(const openbus::map::MapTrafficPose& busPose, double entryForward,
                       double vehicleWidth, bool leftHandTraffic, std::size_t queueSlot) {
    const double side = openbus::map::mapTrafficPassengerSideSign(leftHandTraffic);
    const double lateral = vehicleWidth * 0.5 + 0.7;
    const double forward = entryForward - 0.7 * static_cast<double>(queueSlot);
    const auto offset = openbus::map::mapTrafficPassengerWorldOffset(
        leftHandTraffic, busPose.headingRadians, lateral, forward);
    auto result = busPose;
    result.position[0] += offset[0];
    result.position[1] += offset[1];
    if (queueSlot == 0) {
        result.headingRadians -= side * std::numbers::pi * 0.5;
    }
    return result;
}

inline openbus::map::MapTrafficPose
passengerStopCallQueuePose(openbus::map::MapTrafficPose stopPose, bool leftHandTraffic,
                           std::size_t queueSlot, double vehicleWidth = 2.4) {
    const double passengerSide = openbus::map::mapTrafficPassengerSideSign(leftHandTraffic);
    const double curbOffset = openbus::map::mapTrafficPassengerCurbOffsetMeters(
        leftHandTraffic, 0.0, vehicleWidth, 0.35);
    const auto offset = openbus::map::mapTrafficPassengerWorldOffset(
        leftHandTraffic, stopPose.headingRadians, curbOffset + passengerSide * 0.8,
        -0.8 * static_cast<double>(queueSlot));
    stopPose.position[0] += offset[0];
    stopPose.position[1] += offset[1];
    stopPose.headingRadians -= passengerSide * std::numbers::pi * 0.5;
    return stopPose;
}

inline double passengerApproachDuration(const openbus::map::MapTrafficPose& start,
                                        const openbus::map::MapTrafficPose& end) {
    return std::hypot(end.position[0] - start.position[0], end.position[1] - start.position[1]) /
           passengerApproachSpeedMetersPerSecond;
}

inline openbus::map::MapTrafficPose passengerApproachPose(const openbus::map::MapTrafficPose& start,
                                                          const openbus::map::MapTrafficPose& end,
                                                          double elapsedSeconds) {
    const double duration = passengerApproachDuration(start, end);
    const double progress =
        duration > 1.0e-6 ? std::clamp(elapsedSeconds / duration, 0.0, 1.0) : 1.0;
    auto result = end;
    for (std::size_t axis = 0; axis < 3; ++axis) {
        result.position[axis] =
            start.position[axis] + (end.position[axis] - start.position[axis]) * progress;
    }
    if (progress < 1.0 && duration > 1.0e-6) {
        result.headingRadians =
            std::atan2(end.position[0] - start.position[0], end.position[1] - start.position[1]);
    }
    return result;
}

inline bool passengerApproachTargetChanged(const openbus::map::MapTrafficPose& previous,
                                          const openbus::map::MapTrafficPose& current) {
    constexpr double positionThresholdMeters = 0.75;
    constexpr double heightThresholdMeters = 0.25;
    constexpr double headingThresholdRadians = 0.15;
    const double horizontalDistance =
        std::hypot(current.position[0] - previous.position[0],
                   current.position[1] - previous.position[1]);
    const double headingDifference =
        std::remainder(current.headingRadians - previous.headingRadians,
                       2.0 * std::numbers::pi);
    return horizontalDistance > positionThresholdMeters ||
           std::abs(current.position[2] - previous.position[2]) > heightThresholdMeters ||
           std::abs(headingDifference) > headingThresholdRadians;
}

struct PassengerRoadApproach {
    std::shared_ptr<const openbus::map::MapTrafficRoute> route;
    openbus::map::MapTrafficPose originPose;
    openbus::map::MapTrafficPose destinationPose;
    openbus::map::MapTrafficPose laneStartPose;
    openbus::map::MapTrafficPose laneEndPose;
    double startRouteDistanceMeters = 0.0;
    double endRouteDistanceMeters = 0.0;
    double startConnectorMeters = 0.0;
    double endConnectorMeters = 0.0;
};

inline std::optional<PassengerRoadApproach>
makePassengerRoadApproach(const openbus::map::MapPassengerPedestrianJourney& journey,
                          const openbus::map::MapTrafficPose& origin,
                          const openbus::map::MapTrafficPose& destination,
                          double maximumConnectorMeters = 2.0,
                          double maximumRouteMeters = 50.0) {
    if (!journey.route || !std::isfinite(journey.startDistanceMeters) ||
        journey.startDistanceMeters < 0.0 ||
        journey.startDistanceMeters > journey.route->totalLength ||
        !std::isfinite(maximumConnectorMeters) || maximumConnectorMeters < 0.0 ||
        !std::isfinite(maximumRouteMeters) || maximumRouteMeters <= 0.0) {
        return std::nullopt;
    }

    const auto laneStart = openbus::map::sampleMapTrafficRoute(
        *journey.route, journey.startDistanceMeters, 0.0);
    if (!laneStart ||
        std::abs(laneStart->position[2] - origin.position[2]) > 0.5) {
        return std::nullopt;
    }
    const double startConnector =
        std::hypot(laneStart->position[0] - origin.position[0],
                   laneStart->position[1] - origin.position[1]);
    if (startConnector > maximumConnectorMeters) {
        return std::nullopt;
    }

    const double lastRouteDistance =
        std::min(journey.route->totalLength,
                 journey.startDistanceMeters + maximumRouteMeters);
    std::optional<double> endRouteDistance;
    std::optional<openbus::map::MapTrafficPose> laneEnd;
    double endConnector = maximumConnectorMeters;
    const std::size_t sampleCount = std::max<std::size_t>(
        1, static_cast<std::size_t>(
               std::ceil((lastRouteDistance - journey.startDistanceMeters) / 0.5)));
    for (std::size_t index = 0; index <= sampleCount; ++index) {
        const double routeDistance =
            journey.startDistanceMeters +
            (lastRouteDistance - journey.startDistanceMeters) *
                static_cast<double>(index) / static_cast<double>(sampleCount);
        const auto candidate =
            openbus::map::sampleMapTrafficRoute(*journey.route, routeDistance, 0.0);
        if (!candidate) {
            throw std::runtime_error("Cannot sample pedestrian approach route");
        }
        if (std::abs(candidate->position[2] - destination.position[2]) > 0.5) {
            continue;
        }
        const double separation =
            std::hypot(candidate->position[0] - destination.position[0],
                       candidate->position[1] - destination.position[1]);
        if (separation <= endConnector) {
            endConnector = separation;
            endRouteDistance = routeDistance;
            laneEnd = candidate;
        }
    }
    if (!endRouteDistance || !laneEnd) {
        return std::nullopt;
    }

    return PassengerRoadApproach{journey.route,
                                 origin,
                                 destination,
                                 *laneStart,
                                 *laneEnd,
                                 journey.startDistanceMeters,
                                 *endRouteDistance,
                                 startConnector,
                                 endConnector};
}

inline double passengerRoadApproachDuration(const PassengerRoadApproach& approach) {
    if (!approach.route || approach.endRouteDistanceMeters <
                               approach.startRouteDistanceMeters) {
        throw std::invalid_argument("Passenger road approach has an invalid route");
    }
    const double routeDistance =
        approach.endRouteDistanceMeters - approach.startRouteDistanceMeters;
    return (approach.startConnectorMeters + routeDistance + approach.endConnectorMeters) /
           passengerApproachSpeedMetersPerSecond;
}

inline openbus::map::MapTrafficPose
passengerRoadApproachPose(const PassengerRoadApproach& approach, double elapsedSeconds) {
    if (!approach.route || !std::isfinite(elapsedSeconds) || elapsedSeconds < 0.0) {
        throw std::invalid_argument("Passenger road approach progress is invalid");
    }
    const double travelledMeters =
        std::min(elapsedSeconds * passengerApproachSpeedMetersPerSecond,
                 passengerRoadApproachDuration(approach) *
                     passengerApproachSpeedMetersPerSecond);
    if (travelledMeters <= approach.startConnectorMeters) {
        return passengerApproachPose(approach.originPose, approach.laneStartPose,
                                     travelledMeters / passengerApproachSpeedMetersPerSecond);
    }

    const double routeDistance = approach.endRouteDistanceMeters -
                                 approach.startRouteDistanceMeters;
    const double travelledOnRoute =
        travelledMeters - approach.startConnectorMeters;
    if (travelledOnRoute <= routeDistance) {
        const auto pose = openbus::map::sampleMapTrafficRoute(
            *approach.route,
            approach.startRouteDistanceMeters + travelledOnRoute, 0.0);
        if (!pose) {
            throw std::runtime_error("Cannot sample passenger road approach pose");
        }
        return *pose;
    }
    return passengerApproachPose(
        approach.laneEndPose, approach.destinationPose,
        (travelledOnRoute - routeDistance) / passengerApproachSpeedMetersPerSecond);
}

inline double passengerQueueApproachDuration(
    const std::optional<PassengerRoadApproach>& roadApproach,
    const openbus::map::MapTrafficPose& start, const openbus::map::MapTrafficPose& destination) {
    return roadApproach ? passengerRoadApproachDuration(*roadApproach)
                        : passengerApproachDuration(start, destination);
}

inline openbus::map::MapTrafficPose passengerQueueApproachPose(
    const std::optional<PassengerRoadApproach>& roadApproach,
    const openbus::map::MapTrafficPose& start, const openbus::map::MapTrafficPose& destination,
    double elapsedSeconds) {
    return roadApproach ? passengerRoadApproachPose(*roadApproach, elapsedSeconds)
                        : passengerApproachPose(start, destination, elapsedSeconds);
}

struct PassengerStamperActorPose {
    openbus::map::MapTrafficPose pose;
    bool walking = false;
};

struct PassengerTicketTransactionActorPose {
    openbus::map::MapTrafficPose pose;
    bool walking = false;
};

inline PassengerTicketTransactionActorPose passengerTicketTransactionActorPose(
    const openbus::map::MapTrafficPose& origin,
    const openbus::map::MapTrafficPose& moneyPoint,
    const openbus::map::MapTrafficPose& changePoint, bool paymentRequired, double changeDue,
    bool moneyPointReached, bool changePointReached, double moneyPointApproachSeconds,
    double changePointApproachSeconds) {
    if (paymentRequired && !moneyPointReached) {
        return {passengerApproachPose(origin, moneyPoint, moneyPointApproachSeconds), true};
    }
    if (changeDue > 0.0 && !changePointReached) {
        return {passengerApproachPose(moneyPoint, changePoint, changePointApproachSeconds), true};
    }
    return {changePoint, false};
}

inline PassengerStamperActorPose passengerStamperActorPose(
    const openbus::map::MapTrafficPose& start, const openbus::map::MapTrafficPose& stamper,
    const openbus::map::MapTrafficPose& destination, double approachSeconds,
    double waitSeconds, double returnSeconds) {
    if (approachSeconds < passengerApproachDuration(start, stamper)) {
        return {passengerApproachPose(start, stamper, approachSeconds), true};
    }
    if (waitSeconds < 0.5) {
        return {stamper, false};
    }
    return {passengerApproachPose(stamper, destination, returnSeconds),
            returnSeconds < passengerApproachDuration(stamper, destination)};
}

struct PassengerPedestrianHandoff {
    openbus::map::MapPassengerPedestrianJourney journey;
    openbus::map::MapTrafficPose exitPose;
    double laneStartMeters = 0.0;
    double connectorMeters = 0.0;
    double travelledMeters = 0.0;
    std::size_t modelIndex = 0;
};

inline std::optional<PassengerPedestrianHandoff> makePassengerPedestrianHandoff(
    const openbus::map::MapPassengerPedestrianJourney& journey,
    const openbus::map::MapTrafficPose& exitPose,
    const openbus::map::MapTrafficPose& busPose, bool leftHandTraffic, std::size_t modelIndex) {
    if (!journey.route || journey.destinationDistanceMeters <= journey.startDistanceMeters) {
        return std::nullopt;
    }
    constexpr double backwardSearchRangeMeters = 25.0;
    constexpr double maximumConnectorMeters = 0.5;
    const double first =
        std::max(0.0, journey.startDistanceMeters - backwardSearchRangeMeters);
    const double last =
        std::min(journey.destinationDistanceMeters - 0.01, journey.route->totalLength);
    if (last < first) {
        return std::nullopt;
    }
    const auto side = openbus::map::mapTrafficPassengerWorldOffset(
        leftHandTraffic, busPose.headingRadians, 1.0, 0.0);
    const double exitLateral = (exitPose.position[0] - busPose.position[0]) * side[0] +
                               (exitPose.position[1] - busPose.position[1]) * side[1];
    std::optional<double> nearest;
    double nearestDistance = maximumConnectorMeters;
    const std::size_t samples =
        std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil((last - first) / 0.25)));
    for (std::size_t index = 0; index <= samples; ++index) {
        const double distance = first + (last - first) * static_cast<double>(index) / samples;
        const auto pose = openbus::map::sampleMapTrafficRoute(*journey.route, distance, 0.0);
        if (!pose) {
            throw std::runtime_error("Cannot sample passenger pedestrian handoff route");
        }
        const double lateral = (pose->position[0] - busPose.position[0]) * side[0] +
                               (pose->position[1] - busPose.position[1]) * side[1];
        const double separation = std::hypot(pose->position[0] - exitPose.position[0],
                                              pose->position[1] - exitPose.position[1]);
        if (lateral >= exitLateral - 0.05 &&
            std::abs(pose->position[2] - exitPose.position[2]) <= 0.5 &&
            separation <= nearestDistance) {
            nearest = distance;
            nearestDistance = separation;
        }
    }
    if (!nearest) {
        return std::nullopt;
    }
    return PassengerPedestrianHandoff{journey, exitPose, *nearest, nearestDistance, 0.0,
                                     modelIndex};
}

inline bool advancePassengerPedestrianHandoff(PassengerPedestrianHandoff& passenger,
                                               double elapsedSeconds) {
    if (!std::isfinite(elapsedSeconds) || elapsedSeconds < 0.0) {
        throw std::invalid_argument("Passenger pedestrian elapsed time is invalid");
    }
    passenger.travelledMeters += elapsedSeconds * passengerApproachSpeedMetersPerSecond;
    return passenger.travelledMeters <
           passenger.connectorMeters +
               passenger.journey.destinationDistanceMeters - passenger.laneStartMeters;
}

inline openbus::map::MapTrafficPose
passengerPedestrianHandoffPose(const PassengerPedestrianHandoff& passenger) {
    if (!passenger.journey.route) {
        throw std::logic_error("Passenger pedestrian handoff has no route");
    }
    const double routeDistance =
        passenger.laneStartMeters + std::max(0.0, passenger.travelledMeters - passenger.connectorMeters);
    const auto lanePose = openbus::map::sampleMapTrafficRoute(
        *passenger.journey.route, std::min(routeDistance, passenger.journey.destinationDistanceMeters),
        0.0);
    if (!lanePose) {
        throw std::runtime_error("Cannot sample passenger pedestrian route");
    }
    if (passenger.travelledMeters < passenger.connectorMeters) {
        return passengerApproachPose(passenger.exitPose, *lanePose,
                                     passenger.travelledMeters / passengerApproachSpeedMetersPerSecond);
    }
    return *lanePose;
}

} // namespace openbus::app
