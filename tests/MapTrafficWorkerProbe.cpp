#include "app/MapTrafficWorker.h"

#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <thread>

namespace {

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

openbus::map::MapTrafficActor makeActor() {
    openbus::map::MapSplinePlacement spline;
    spline.splineId = 1;
    spline.localX = 0.0;
    spline.localY = 0.0;
    spline.length = 100.0;
    spline.elevation = 0.0;
    spline.geometryValid = true;

    openbus::map::MapTrafficRouteSegment segment;
    segment.spline = {spline, 0, 0};

    openbus::map::MapTrafficActor actor;
    actor.modelPath = "worker-probe";
    actor.route.segments.push_back(std::move(segment));
    actor.route.cumulativeEndDistances = {100.0};
    actor.route.totalLength = 100.0;
    actor.distanceMeters = 10.0;
    actor.speedMetersPerSecond = 5.0;
    actor.cruiseSpeedMetersPerSecond = 5.0;
    actor.targetSpeedMetersPerSecond = 5.0;
    return actor;
}

openbus::app::MapTrafficWorker::ActorInput makeVehicleInput(
    std::size_t stateIndex, std::uint64_t generation, openbus::map::MapTrafficActor actor) {
    using Worker = openbus::app::MapTrafficWorker;
    Worker::ActorMotion motion;
    motion.distanceMeters = actor.distanceMeters;
    motion.speedMetersPerSecond = actor.speedMetersPerSecond;
    motion.targetSpeedMetersPerSecond = actor.targetSpeedMetersPerSecond;
    motion.lateralOffsetMeters = actor.lateralOffsetMeters;
    motion.junctionWaitSeconds = actor.junctionWaitSeconds;
    motion.scheduledStopDistanceMeters = actor.scheduledStopDistanceMeters;
    motion.blockReason = actor.blockReason;
    motion.blockingObstacleIndex = actor.blockingObstacleIndex;
    motion.junctionClaimEndMeters = actor.junctionClaimEndMeters;
    motion.committedSignalSegmentIndex = actor.committedSignalSegmentIndex;
    return {stateIndex, generation,
            std::make_shared<const openbus::map::MapTrafficActor>(std::move(actor)), motion,
            std::nullopt};
}

openbus::app::MapTrafficWorker::HumanInput makeHumanInput(
    std::size_t stateIndex, std::uint64_t generation, openbus::map::MapTrafficActor actor,
    double feetDistanceMeters, bool populationEnabled, bool advanceWhileInactive) {
    using Worker = openbus::app::MapTrafficWorker;
    const double distanceMeters = actor.distanceMeters;
    const double speedMetersPerSecond = actor.speedMetersPerSecond;
    return {stateIndex, generation,
            std::make_shared<const openbus::map::MapTrafficActor>(std::move(actor)),
            distanceMeters, speedMetersPerSecond, feetDistanceMeters, populationEnabled,
            advanceWhileInactive};
}

} // namespace

int main() {
    try {
        openbus::app::MapTrafficWorker worker;
        openbus::map::MapTrafficActor vehicle = makeActor();
        openbus::map::MapTrafficActor pedestrian = makeActor();
        pedestrian.distanceMeters = 30.0;
        openbus::map::MapTrafficActor inactivePedestrian = makeActor();
        inactivePedestrian.distanceMeters = 50.0;
        auto signalProgram = std::make_shared<openbus::map::MapTrafficSignalProgram>();
        signalProgram->cycleDurationSeconds = 10.0;
        signalProgram->lights = {{{0, 10.0}}};
        signalProgram->approachDistancesMeters = {std::nullopt};
        auto signalController =
            std::make_shared<openbus::map::MapTrafficSignalController>(signalProgram);
        worker.submit({makeVehicleInput(0, 0, std::move(vehicle))},
                       {makeHumanInput(0, 0, std::move(pedestrian), 0.2, true, false),
                        makeHumanInput(1, 0, std::move(inactivePedestrian), 0.2, false, true)},
                       {}, {}, {signalController}, {},
                       0.05, 0.05);

        const auto waitForResults = [&] {
            std::optional<openbus::app::MapTrafficWorker::UpdateResults> results;
            const auto resultDeadline =
                std::chrono::steady_clock::now() + std::chrono::seconds(2);
            while (!results && std::chrono::steady_clock::now() < resultDeadline) {
                results = worker.takeLatestResults();
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            return results;
        };
        auto results = waitForResults();
        require(results.has_value() && results->actors.size() == 1 &&
                    results->actors[0].generation == 0 &&
                    results->humans.size() == 2 && results->humans[0].generation == 0,
                "the dedicated traffic loop publishes vehicle and pedestrian updates");
        require(std::abs(results->actors[0].motion.distanceMeters - 10.25) < 0.05 &&
                    results->humans[0].distanceMeters > 30.0 &&
                    results->humans[1].distanceMeters > 50.0,
                "the dedicated loop advances vehicle, active pedestrian, and off-screen pedestrian AI");
        const auto expectedActor = makeActor();
        const auto expectedVehiclePose = openbus::map::sampleMapTrafficRoute(
            expectedActor.route, results->actors[0].motion.distanceMeters,
            results->actors[0].motion.lateralOffsetMeters);
        require(expectedVehiclePose.has_value() &&
                    std::abs(results->actors[0].pose.position[0] -
                             expectedVehiclePose->position[0]) < 1.0e-6 &&
                    std::abs(results->actors[0].pose.position[1] -
                             expectedVehiclePose->position[1]) < 1.0e-6 &&
                    std::isfinite(results->actors[0].travelHeadingRadians),
                "the AI worker publishes a pose and travel heading for its advanced vehicle");
        require(signalController->clockSeconds() > 0.0,
                "the dedicated loop advances traffic signals on the AI worker");
        require(std::isfinite(worker.averageTickProcessingSeconds()) &&
                    worker.averageTickProcessingSeconds() > 0.0,
                "the worker reports a positive moving average of AI tick processing time");

        vehicle = makeActor();
        vehicle.distanceMeters = 20.0;
        pedestrian = makeActor();
        pedestrian.distanceMeters = 40.0;
        inactivePedestrian = makeActor();
        inactivePedestrian.distanceMeters = 60.0;
        worker.submit({makeVehicleInput(0, 1, std::move(vehicle))},
                      {makeHumanInput(0, 1, std::move(pedestrian), 0.2, true, false),
                       makeHumanInput(1, 1, std::move(inactivePedestrian), 0.2, false, true)},
                      {}, {}, {}, {},
                      0.05, 0.10);
        results = waitForResults();
        require(results.has_value() && results->actors.size() == 1 &&
                    results->actors[0].generation == 1 &&
                    std::abs(results->actors[0].motion.distanceMeters - 20.25) < 0.05 &&
                    results->humans.size() == 2 && results->humans[0].generation == 1 &&
                    results->humans[0].distanceMeters > 40.0 &&
                    results->humans[1].generation == 1 &&
                    results->humans[1].distanceMeters > 60.0,
                "new generations do not inherit stale actor movement state");

        worker.submitPopulationPlan(
            {{0, 0, 3, false, false, true, false, true},
             {1, 0, 3, false, false, false, true, false},
             {2, 0, std::nullopt, false, true, false, true, false}},
            0, 1, 0, {{3, 1}}, true);
        std::optional<std::vector<openbus::app::MapTrafficWorker::PopulationDecision>>
            populationDecisions;
        const auto populationDeadline =
            std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (!populationDecisions && std::chrono::steady_clock::now() < populationDeadline) {
            populationDecisions = worker.takePopulationDecisions();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        require(populationDecisions.has_value() && populationDecisions->size() == 3 &&
                    !(*populationDecisions)[0].despawn &&
                    !(*populationDecisions)[1].spawn &&
                    (*populationDecisions)[2].spawn,
                "the worker selects population candidates against per-group and bus targets");

        worker.submitPopulationPlan(
            {{4, 1, 7, false, false, true, false, true}}, 0, 0, 0, {}, true);
        populationDecisions.reset();
        const auto despawnDeadline =
            std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (!populationDecisions && std::chrono::steady_clock::now() < despawnDeadline) {
            populationDecisions = worker.takePopulationDecisions();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        require(populationDecisions.has_value() && populationDecisions->size() == 1 &&
                    (*populationDecisions)[0].despawn &&
                    (*populationDecisions)[0].generation == 1,
                "the worker publishes generation-tagged off-screen despawn decisions");

        worker.submitPopulationPlan(
            {{4, 2, std::nullopt, true, false, true, false, true},
             {5, 2, std::nullopt, true, false, true, false, true}},
            0, 0, 1, {}, false);
        populationDecisions.reset();
        const auto humanPopulationDeadline =
            std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (!populationDecisions &&
               std::chrono::steady_clock::now() < humanPopulationDeadline) {
            populationDecisions = worker.takePopulationDecisions();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        require(populationDecisions.has_value() && populationDecisions->size() == 2 &&
                    !(*populationDecisions)[0].despawn &&
                    (*populationDecisions)[1].despawn &&
                    (*populationDecisions)[1].isHuman,
                "the worker plans human population and unloads the newest eligible actor first");

        auto conditionalSignalProgram =
            std::make_shared<openbus::map::MapTrafficSignalProgram>();
        conditionalSignalProgram->cycleDurationSeconds = 10.0;
        conditionalSignalProgram->lights = {{{6, 5.0}, {0, 5.0}}};
        conditionalSignalProgram->approachDistancesMeters = {std::nullopt};
        conditionalSignalProgram->controlPoints = {{0, 3.0, true}};
        auto conditionalSignalController =
            std::make_shared<openbus::map::MapTrafficSignalController>(
                conditionalSignalProgram);
        openbus::map::MapTrafficSignalSchedule conditionalSchedule;
        conditionalSchedule.cycleDurationSeconds = 10.0;
        conditionalSchedule.phases = conditionalSignalProgram->lights[0];
        conditionalSchedule.hasConditionalOverrides = true;
        conditionalSchedule.approachDistanceMeters = 0.0;
        conditionalSchedule.program = conditionalSignalProgram;
        conditionalSchedule.controller = conditionalSignalController;
        openbus::map::MapTrafficActor inactiveSignalPedestrian = makeActor();
        inactiveSignalPedestrian.distanceMeters = 0.0;
        inactiveSignalPedestrian.speedMetersPerSecond = 0.0;
        inactiveSignalPedestrian.route.segments[0].trafficSignal = conditionalSchedule;
        openbus::app::MapTrafficWorker signalWorker;
        signalWorker.submit(
            {}, {makeHumanInput(0, 0, std::move(inactiveSignalPedestrian), 0.2, false, false)},
            {}, {}, {conditionalSignalController}, {}, 3.05, 3.05);
        double signalWorkerElapsedSeconds = 0.0;
        const auto signalDeadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (signalWorkerElapsedSeconds < 3.05 &&
               std::chrono::steady_clock::now() < signalDeadline) {
            if (auto signalResults = signalWorker.takeLatestResults()) {
                signalWorkerElapsedSeconds += signalResults->elapsedSeconds;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        const double conditionalSignalClock = conditionalSignalController->clockSeconds();
        if (signalWorkerElapsedSeconds + 1.0e-6 < 3.05 ||
            std::abs(conditionalSignalClock - 3.0) >= 1.0e-5) {
            throw std::runtime_error(
                "inactive pedestrian signal probe did not hold at the conditional control point: "
                "elapsed=" +
                std::to_string(signalWorkerElapsedSeconds) +
                ", clock=" + std::to_string(conditionalSignalClock));
        }

        openbus::app::MapTrafficWorker rateWorker;
        rateWorker.setSimulationRateHz(10.0);
        rateWorker.submit({makeVehicleInput(0, 0, makeActor())}, {}, {}, {}, {}, {}, 0.1, 0.1);
        std::optional<openbus::app::MapTrafficWorker::UpdateResults> rateResults;
        const auto rateDeadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (!rateResults && std::chrono::steady_clock::now() < rateDeadline) {
            rateResults = rateWorker.takeLatestResults();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        require(rateResults.has_value() && rateResults->actors.size() == 1 &&
                    rateResults->elapsedSeconds >= 0.1 - 1.0e-6 &&
                    std::abs(rateResults->actors[0].motion.distanceMeters - 10.5) < 1.0e-5,
                "the worker advances actors using its configured simulation rate");
        std::cout << "Map traffic worker probe passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Map traffic worker probe failed: " << error.what() << '\n';
        return 1;
    }
}
