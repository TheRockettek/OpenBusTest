#include "app/MapTrafficWorker.h"
#include "map/MapTileWindow.h"

#include <atomic>
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
        using Worker = openbus::app::MapTrafficWorker;
        const std::vector<Worker::PopulationCandidate> balanceCandidates = {
            {0, 0, 1, false, false, true},
            {1, 0, 1, false, false, true},
            {2, 0, 2, false, false, false},
            {3, 0, 3, false, false, false},
            {4, 0, std::nullopt, false, false, false},
            {5, 0, std::nullopt, false, true, true},
            {6, 0, std::nullopt, true, false, true},
        };
        const Worker::PopulationBalance groupedBalance(balanceCandidates, 2, 1, 2,
                                                        {{1, 1}, {2, 1}}, true);
        require(groupedBalance.countFor(balanceCandidates[0]) == 2 &&
                    groupedBalance.targetFor(balanceCandidates[0]) == 1 &&
                    groupedBalance.needsDespawn(balanceCandidates[0]) &&
                    groupedBalance.countFor(balanceCandidates[2]) == 0 &&
                    groupedBalance.targetFor(balanceCandidates[2]) == 1 &&
                    groupedBalance.needsSpawn(balanceCandidates[2]) &&
                    groupedBalance.targetFor(balanceCandidates[3]) == 0 &&
                    !groupedBalance.needsSpawn(balanceCandidates[3]) &&
                    groupedBalance.targetFor(balanceCandidates[4]) == 0 &&
                    groupedBalance.countFor(balanceCandidates[5]) == 1 &&
                    groupedBalance.targetFor(balanceCandidates[5]) == 1 &&
                    !groupedBalance.needsDespawn(balanceCandidates[5]) &&
                    groupedBalance.countFor(balanceCandidates[6]) == 1 &&
                    groupedBalance.targetFor(balanceCandidates[6]) == 2,
                "population demand respects separate road group, bus, and human targets");
        const Worker::PopulationBalance ungroupedBalance(balanceCandidates, 2, 1, 2, {}, false);
        require(ungroupedBalance.countFor(balanceCandidates[2]) == 2 &&
                    ungroupedBalance.targetFor(balanceCandidates[2]) == 2 &&
                    !ungroupedBalance.needsSpawn(balanceCandidates[2]) &&
                    ungroupedBalance.countFor(balanceCandidates[4]) == 2,
                "ungrouped road demand counts all road vehicles against the shared target");

        openbus::app::MapTrafficWorker worker;
        openbus::map::MapTrafficActor vehicle = makeActor();
        vehicle.route.segments[0].spline.placement.radius = 25.0;
        vehicle.rearAxleOffsetMeters = -3.0;
        const auto expectedActor = vehicle;
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
        const auto expectedVehiclePose = openbus::map::sampleMapTrafficActorPose(
            expectedActor, results->actors[0].motion.distanceMeters);
        require(expectedVehiclePose.has_value() &&
                    std::abs(results->actors[0].pose.position[0] -
                             expectedVehiclePose->position[0]) < 1.0e-6 &&
                    std::abs(results->actors[0].pose.position[1] -
                             expectedVehiclePose->position[1]) < 1.0e-6 &&
                    std::abs(results->actors[0].travelHeadingRadians -
                             expectedVehiclePose->headingRadians) < 1.0e-6,
                "the AI worker publishes the rear-axle body pose and heading");
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
            {{6, 0, std::nullopt, false, false, false, true, false, 80.0},
             {7, 0, std::nullopt, false, false, false, true, false, 5.0}},
            1, 0, 0, {}, false);
        populationDecisions.reset();
        const auto priorityDeadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (!populationDecisions && std::chrono::steady_clock::now() < priorityDeadline) {
            populationDecisions = worker.takePopulationDecisions();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        require(populationDecisions.has_value() && populationDecisions->size() == 2 &&
                    !(*populationDecisions)[0].spawn && (*populationDecisions)[1].spawn &&
                    (*populationDecisions)[1].stateIndex == 7,
                "nearer eligible vehicle spawn candidates are selected first");

        worker.submitPopulationPlan(
            {{0, 0, 1, false, false, true, false, true},
             {1, 0, 1, false, false, true, false, true},
             {2, 0, 2, false, false, false, true, false, 20.0},
             {3, 0, 2, false, false, false, true, false, 10.0},
             {4, 0, 1, false, false, false, true, false, 1.0}},
            2, 0, 0, {{1, 1}, {2, 1}}, true);
        populationDecisions.reset();
        const auto groupPriorityDeadline =
            std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (!populationDecisions && std::chrono::steady_clock::now() < groupPriorityDeadline) {
            populationDecisions = worker.takePopulationDecisions();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        require(populationDecisions.has_value() && populationDecisions->size() == 5 &&
                    (*populationDecisions)[0].despawn && !(*populationDecisions)[1].despawn &&
                    !(*populationDecisions)[2].spawn && (*populationDecisions)[3].spawn &&
                    !(*populationDecisions)[4].spawn,
                "group deficits remain eligible when another group fills the global target");

        worker.submitPopulationPlan(
            {{0, 0, std::nullopt, false, false, false, false, false, 1.0},
             {1, 0, std::nullopt, false, false, false, true, false, 5.0},
             {2, 0, std::nullopt, false, false, false, true, false, 5.0}},
            1, 0, 0, {}, false);
        populationDecisions.reset();
        const auto tieDeadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (!populationDecisions && std::chrono::steady_clock::now() < tieDeadline) {
            populationDecisions = worker.takePopulationDecisions();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        require(populationDecisions.has_value() && populationDecisions->size() == 3 &&
                    !(*populationDecisions)[0].spawn && (*populationDecisions)[1].spawn &&
                    !(*populationDecisions)[2].spawn,
                "ineligible candidates are skipped and equal spawn priorities keep input order");

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

        Worker searchWorker;
        auto searchActor = makeActor();
        searchActor.lengthMeters = 4.0;
        searchActor.widthMeters = 2.0;
        searchActor.rearAxleOffsetMeters = -1.0;
        const auto firstPose = openbus::map::sampleMapTrafficActorPose(searchActor, 20.0);
        const auto secondPose = openbus::map::sampleMapTrafficActorPose(searchActor, 40.0);
        const auto thirdPose = openbus::map::sampleMapTrafficActorPose(searchActor, 60.0);
        require(firstPose && secondPose && thirdPose, "spawn probe poses can be sampled");
        const auto actorTemplate =
            std::make_shared<const openbus::map::MapTrafficActor>(searchActor);
        auto spawnCandidates =
            std::make_shared<std::vector<openbus::map::MapTrafficSpawnCandidate>>();
        for (const auto& [distance, pose] :
             std::vector<std::pair<double, openbus::map::MapTrafficPose>>{
                 {20.0, *firstPose}, {40.0, *secondPose}, {60.0, *thirdPose}}) {
            spawnCandidates->push_back({distance, openbus::map::mapWorldTile(pose.position[0]),
                                        openbus::map::mapWorldTile(pose.position[1])});
        }
        Worker::PopulationCandidate searchCandidate{8, 42, std::nullopt, false, false, false};
        searchCandidate.spawnSearch =
            Worker::PopulationSpawnSearch{actorTemplate, spawnCandidates, {}, 0.0};
        Worker::PopulationSearchContext searchContext;
        for (const auto& candidate : *spawnCandidates) {
            searchContext.activeTiles.emplace_back(candidate.tileX, candidate.tileY);
        }
        searchContext.spawnInView = true;
        searchContext.hasPlayerPose = true;
        const double directionX = secondPose->position[0] - firstPose->position[0];
        const double directionY = secondPose->position[1] - firstPose->position[1];
        const double directionLength = std::hypot(directionX, directionY);
        require(directionLength > 0.0, "spawn probe route has distinct poses");
        searchContext.playerPose = *firstPose;
        searchContext.playerPose.position[0] -= directionX / directionLength * 10.0;
        searchContext.playerPose.position[1] -= directionY / directionLength * 10.0;
        searchContext.playerPose.headingRadians = std::atan2(directionY, directionX);
        searchContext.playerMoving = true;
        searchContext.playerLengthMeters = 4.0;
        searchContext.playerWidthMeters = 2.0;
        searchContext.diagnostics = true;
        const auto search = [&](const Worker::PopulationCandidate& candidate,
                                const Worker::PopulationSearchContext& context,
                                std::size_t roadTarget = 1, std::size_t busTarget = 0) {
            searchWorker.submitPopulationPlan({candidate}, roadTarget, busTarget, 0, {}, false,
                                              context);
            std::optional<std::vector<Worker::PopulationDecision>> result;
            const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
            while (!result && std::chrono::steady_clock::now() < deadline) {
                result = searchWorker.takePopulationDecisions();
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            require(result && result->size() == 1, "worker publishes a spawn search result");
            return result->front();
        };
        auto decision = search(searchCandidate, searchContext);
        require(decision.spawn && decision.generation == 42 && decision.spawnLocation &&
                    decision.spawnLocation->candidateIndex == 2 &&
                    decision.roadSearchDiagnostics.frontZoneRejectedCandidates == 2,
                "moving-player exclusion and priority are applied on the worker");
        searchContext.playerMoving = false;
        decision = search(searchCandidate, searchContext);
        require(decision.spawnLocation && decision.spawnLocation->candidateIndex == 0,
                "stationary player permits the nearest road candidate");
        auto crowdedContext = searchContext;
        crowdedContext.playerPose = *firstPose;
        for (const double lateral : {8.0, 9.0, 10.0}) {
            auto obstaclePose = *firstPose;
            obstaclePose.position[0] -= directionY / directionLength * lateral;
            obstaclePose.position[1] += directionX / directionLength * lateral;
            crowdedContext.occupied.push_back({obstaclePose, 4.0, 2.0});
        }
        const auto uncrowdedDecision = search(searchCandidate, crowdedContext);
        require(uncrowdedDecision.spawnLocation &&
                    uncrowdedDecision.spawnLocation->candidateIndex == 2,
                "spawn selection prefers a less crowded nearby road location");
        auto elevatedContext = crowdedContext;
        for (auto& obstacle : elevatedContext.occupied) {
            obstacle.pose.position[2] += 10.0;
        }
        const auto elevatedDecision = search(searchCandidate, elevatedContext);
        require(elevatedDecision.spawnLocation &&
                    elevatedDecision.spawnLocation->candidateIndex == 0,
                "vehicles on a different road elevation do not bias spawn distribution");
        auto penalizedCandidates =
            std::make_shared<std::vector<openbus::map::MapTrafficSpawnCandidate>>(*spawnCandidates);
        (*penalizedCandidates)[0].locationPenaltyMeters = 50.0;
        auto penalizedCandidate = searchCandidate;
        penalizedCandidate.spawnSearch->candidates = penalizedCandidates;
        const auto saferDecision = search(penalizedCandidate, searchContext);
        require(saferDecision.spawnLocation && saferDecision.spawnLocation->candidateIndex == 1,
                "a less awkward spawn location can outweigh modest player proximity");
        penalizedCandidate.spawnSearch->rejectedCandidates = {1, 2};
        const auto fallbackDecision = search(penalizedCandidate, searchContext);
        require(fallbackDecision.spawnLocation && fallbackDecision.spawnLocation->candidateIndex == 0,
                "a penalized location remains available when no alternative is eligible");
        auto entryCandidates =
            std::make_shared<std::vector<openbus::map::MapTrafficSpawnCandidate>>(*spawnCandidates);
        (*entryCandidates)[1].entryPreferenceMeters = 25.0;
        auto entryCandidate = searchCandidate;
        entryCandidate.spawnSearch->candidates = entryCandidates;
        const auto entryDecision = search(entryCandidate, searchContext);
        require(entryDecision.spawnLocation && entryDecision.spawnLocation->candidateIndex == 1,
                "an entry-lane preference influences the worker's spawn selection");
        auto secondCandidate = searchCandidate;
        secondCandidate.stateIndex = 9;
        searchWorker.submitPopulationPlan({searchCandidate, secondCandidate}, 2, 0, 0, {}, false,
                                          searchContext);
        std::optional<std::vector<Worker::PopulationDecision>> batchDecisions;
        const auto batchDeadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (!batchDecisions && std::chrono::steady_clock::now() < batchDeadline) {
            batchDecisions = searchWorker.takePopulationDecisions();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        require(batchDecisions && batchDecisions->size() == 2 &&
                    (*batchDecisions)[0].spawn && (*batchDecisions)[1].spawn &&
                    (*batchDecisions)[0].spawnLocation && (*batchDecisions)[1].spawnLocation &&
                    (*batchDecisions)[0].spawnLocation->candidateIndex !=
                        (*batchDecisions)[1].spawnLocation->candidateIndex,
                "a population batch reserves earlier proposals and spreads later spawns");
        const auto proposedRoadSpawn = decision;
        const Worker::PopulationBalance admissionBalance({searchCandidate}, 1, 0, 0, {}, false);
        const auto revalidate = [&](const Worker::PopulationCandidate& current,
                                    const Worker::PopulationBalance& balance,
                                    const Worker::PopulationDecision& proposal,
                                    const Worker::PopulationSearchContext& context,
                                    const std::vector<openbus::map::MapTrafficObstacle>& occupied,
                                    bool departureDue = true) {
            return Worker::revalidatePopulationSpawn(current, balance, proposal, searchActor,
                                                     *spawnCandidates, {}, context, occupied,
                                                     departureDue);
        };
        require(revalidate(searchCandidate, admissionBalance, proposedRoadSpawn, searchContext, {})
                    .has_value(),
                "current safe spawn proposals pass main-thread revalidation");
        auto changedCandidate = searchCandidate;
        ++changedCandidate.generation;
        require(!revalidate(changedCandidate, admissionBalance, proposedRoadSpawn, searchContext, {}),
                "revalidation rejects a stale actor generation");
        const Worker::PopulationBalance noRoadDemand({searchCandidate}, 0, 0, 0, {}, false);
        require(!revalidate(searchCandidate, noRoadDemand, proposedRoadSpawn, searchContext, {}),
                "revalidation rejects proposals when the current target is full");
        auto changedContext = searchContext;
        changedContext.activeTiles.clear();
        require(!revalidate(searchCandidate, admissionBalance, proposedRoadSpawn, changedContext, {}),
                "revalidation rejects proposals when the active tile changes");
        changedContext = searchContext;
        changedContext.playerMoving = true;
        require(!revalidate(searchCandidate, admissionBalance, proposedRoadSpawn, changedContext, {}),
                "revalidation rejects a proposal newly ahead of the moving player");
        changedContext = searchContext;
        changedContext.spawnInView = false;
        changedContext.cameraValid = false;
        require(!revalidate(searchCandidate, admissionBalance, proposedRoadSpawn, changedContext, {}),
                "revalidation rejects a proposal after the camera becomes invalid");
        require(!revalidate(searchCandidate, admissionBalance, proposedRoadSpawn, searchContext,
                            {{*firstPose, 4.0, 2.0}}),
                "revalidation rejects a position occupied after the worker snapshot");
        require(!Worker::revalidatePopulationSpawn(
                    searchCandidate, admissionBalance, proposedRoadSpawn, searchActor,
                    *spawnCandidates, {0}, searchContext, {}, true),
                "revalidation rejects a candidate rejected after the worker snapshot");
        Worker::PopulationCandidate anotherCandidate = searchCandidate;
        anotherCandidate.stateIndex = 10;
        Worker::PopulationDecision anotherProposal = proposedRoadSpawn;
        anotherProposal.stateIndex = 10;
        require(!revalidate(anotherCandidate, admissionBalance, anotherProposal, searchContext,
                            {{*firstPose, 4.0, 2.0}}),
                "later proposals cannot overlap a spawn already admitted in the same pass");
        Worker::PopulationBalance filledBalance({searchCandidate, anotherCandidate}, 1, 0, 0, {},
                                                 false);
        ++filledBalance.countFor(searchCandidate);
        require(!revalidate(anotherCandidate, filledBalance, anotherProposal, searchContext, {}),
                "later proposals cannot exceed a target filled by an earlier admission");
        searchContext.occupied.push_back({*firstPose, 4.0, 2.0});
        decision = search(searchCandidate, searchContext);
        require(decision.spawnLocation && decision.spawnLocation->candidateIndex == 1 &&
                    decision.roadSearchDiagnostics.obstructedCandidates == 1,
                "worker skips occupied candidates and selects the nearest clear one");
        searchContext.occupied.clear();
        searchCandidate.spawnSearch->rejectedCandidates = {0, 1};
        decision = search(searchCandidate, searchContext);
        require(decision.spawnLocation && decision.spawnLocation->candidateIndex == 2 &&
                    decision.roadSearchDiagnostics.rejectedCandidates == 2,
                "worker respects rejected spawn candidates");
        searchCandidate.spawnSearch->rejectedCandidates.clear();
        searchCandidate.spawnSearch->lateralOffsetMeters = 1.5;
        const auto offsetPose =
            openbus::map::sampleMapTrafficActorPose(searchActor.route, 20.0, 1.5,
                                                     searchActor.rearAxleOffsetMeters);
        decision = search(searchCandidate, searchContext);
        require(offsetPose && decision.spawnLocation &&
                    std::abs(decision.spawnLocation->pose.position[0] - offsetPose->position[0]) <
                        1.0e-6 &&
                    std::abs(decision.spawnLocation->pose.position[1] - offsetPose->position[1]) <
                        1.0e-6,
                "worker samples the snapshot lateral offset and rear axle geometry");
        searchCandidate.spawnSearch->lateralOffsetMeters = 0.0;
        searchContext.activeTiles.clear();
        decision = search(searchCandidate, searchContext);
        require(!decision.spawn && decision.roadSearchDiagnostics.inactiveCandidateTiles == 3,
                "worker rejects candidates outside the active tile snapshot");
        searchContext.activeTiles.emplace_back((*spawnCandidates)[0].tileX,
                                               (*spawnCandidates)[0].tileY);
        searchCandidate.spawnSearch->rejectedCandidates = {1, 2};
        searchContext.spawnInView = false;
        searchContext.cameraValid = false;
        decision = search(searchCandidate, searchContext);
        require(!decision.spawn, "worker requires a valid camera when spawning out of view");
        searchContext.cameraValid = true;
        searchContext.mainCamera.position = {1000.0, 1000.0, 0.0};
        searchContext.mainCamera.viewProjection = {1.0, 0.0, 0.0, 0.0,
                                                    0.0, 1.0, 0.0, 0.0,
                                                    0.0, 0.0, 1.0, 0.0,
                                                    0.0, 0.0, 0.0, 1.0};
        decision = search(searchCandidate, searchContext);
        require(decision.spawn, "worker accepts a candidate hidden from the main camera");
        require(revalidate(searchCandidate, admissionBalance, proposedRoadSpawn, searchContext, {})
                    .has_value(),
                "revalidation accepts a proposal hidden from the current main camera");
        auto reflection = searchContext.mainCamera;
        reflection.position = firstPose->position;
        searchContext.reflectionCameras.push_back(reflection);
        decision = search(searchCandidate, searchContext);
        require(!decision.spawn, "worker rejects candidates visible to reflection cameras");
        require(!revalidate(searchCandidate, admissionBalance, proposedRoadSpawn, searchContext, {}),
                "revalidation rejects a proposal visible to a newly moved reflection camera");
        searchContext.reflectionCameras.clear();
        decision = search(searchCandidate, searchContext, 0);
        require(!decision.spawn && !decision.searchedSpawn,
                "worker skips route search when the population is already at target");

        Worker::PopulationCandidate busCandidate{9, 7, std::nullopt, false, true, false};
        busCandidate.spawnSearch =
            Worker::PopulationSpawnSearch{actorTemplate, nullptr, {}, 0.0};
        searchContext.activeTiles.clear();
        const auto busPose = openbus::map::sampleMapTrafficActorPose(searchActor, 0.0);
        require(busPose.has_value(), "scheduled bus entry pose can be sampled");
        searchContext.activeTiles.emplace_back(openbus::map::mapWorldTile(busPose->position[0]),
                                               openbus::map::mapWorldTile(busPose->position[1]));
        searchContext.spawnInView = true;
        decision = search(busCandidate, searchContext, 0, 1);
        require(decision.spawn && decision.spawnLocation &&
                    decision.spawnLocation->distanceMeters == 0.0,
                "worker proposes the scheduled bus entry");
        const Worker::PopulationBalance busAdmissionBalance({busCandidate}, 0, 1, 0, {}, false);
        require(revalidate(busCandidate, busAdmissionBalance, decision, searchContext, {}, true) &&
                    !revalidate(busCandidate, busAdmissionBalance, decision, searchContext, {},
                                false),
                "scheduled bus admission rechecks that its departure is still due");
        searchContext.occupied.push_back({*busPose, 4.0, 2.0});
        decision = search(busCandidate, searchContext, 0, 1);
        require(!decision.spawn && decision.searchedSpawn && decision.busSearchDiagnostics &&
                    !decision.busSearchDiagnostics->clear,
                "worker rejects an obstructed scheduled bus entry");

        Worker busyWorker;
        std::atomic<std::size_t> submittedPlans = 0;
        std::jthread planner([&](std::stop_token stop) {
            const std::vector<Worker::PopulationCandidate> candidates(
                1000, {0, 0, std::nullopt, false, false, false});
            while (!stop.stop_requested()) {
                busyWorker.submitPopulationPlan(candidates, 0, 0, 0, {}, false);
                ++submittedPlans;
                std::this_thread::yield();
            }
        });
        while (submittedPlans.load() == 0) {
            std::this_thread::yield();
        }
        busyWorker.submit({makeVehicleInput(0, 0, makeActor())}, {}, {}, {}, {}, {}, 0.05, 0.05);
        std::optional<Worker::UpdateResults> busyResults;
        const auto busyDeadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (!busyResults && std::chrono::steady_clock::now() < busyDeadline) {
            busyResults = busyWorker.takeLatestResults();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        planner.request_stop();
        planner.join();
        require(busyResults && !busyResults->actors.empty() &&
                    busyResults->actors[0].motion.distanceMeters > 10.0,
                "movement progresses while population requests continuously arrive");

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
