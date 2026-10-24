#include "physics/BusSimulation.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <numbers>
#include <stdexcept>

namespace {

BusConfiguration configuration() {
    BusConfiguration value{};
    value.mass = 12620.0;
    value.length = 12.6;
    value.width = 2.546;
    value.bodyHalfLength = 6.3;
    value.bodyHalfWidth = 1.273;
    value.bodyHalfHeight = 1.55;
    value.wheelRadius = 0.47825;
    value.wheelHalfWidth = 0.145;
    value.collisionLength = 12.6;
    value.collisionWidth = 2.546;
    value.collisionHeight = 3.1;
    value.centerOfGravityHeight = 1.3;
    value.axles = {{3.2921, 2.4, 2.4, 1.85, 0.9565, 240000.0, 90000.0,
                    20000.0, true, false},
                   {-3.0008, 2.45, 2.45, 1.25, 0.9565, 280000.0, 116000.0,
                    20000.0, false, true}};
    return value;
}

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void advance(const BusSimulation& simulation, OnFootState& state, int frames,
             double yaw = 0.0, double forward = 0.0, double right = 0.0) {
    for (int frame = 0; frame < frames; ++frame) {
        simulation.updateOnFoot(state, 1.0 / 60.0, yaw, forward, right, false);
    }
}

} // namespace

int main() {
    try {
        BusSimulation simulation(configuration(), VehiclePlacement{{0.0, 0.0, 0.0}, 0.0});
        simulation.setFrozen(true);
        const auto busPosition = simulation.chassisPose().position;
        OnFootState state{{0.0, 0.0, 0.0}};
        advance(simulation, state, 60);
        require(state.grounded && std::abs(state.feetPosition[2]) < 0.002,
                "Walker must stand on the ground, ignoring the hidden bus body");
        advance(simulation, state, 60, 0.0, 1.0, 0.0);
        require(std::abs(state.feetPosition[0] - 1.5) < 0.01,
                "W must walk forward at 1.5 m/s");
        advance(simulation, state, 60, 0.0, -1.0, 0.0);
        require(std::abs(state.feetPosition[0]) < 0.01, "S must walk backward");
        advance(simulation, state, 60, std::numbers::pi / 2.0, 1.0, 0.0);
        require(std::abs(state.feetPosition[1] - 1.5) < 0.01,
                "Walking forward must turn with the camera heading");
        advance(simulation, state, 60, std::numbers::pi / 2.0, -1.0, 0.0);
        advance(simulation, state, 60, std::numbers::pi / 2.0, 0.0, 1.0);
        require(std::abs(state.feetPosition[0] - 1.5) < 0.01,
                "D must strafe right relative to the camera heading");
        advance(simulation, state, 60, std::numbers::pi / 2.0, 0.0, -1.0);
        require(std::abs(state.feetPosition[0]) < 0.01, "A must strafe left");
        const auto beforeDiagonal = state.feetPosition;
        advance(simulation, state, 60, 0.0, 1.0, 1.0);
        require(std::abs(std::hypot(state.feetPosition[0] - beforeDiagonal[0],
                                    state.feetPosition[1] - beforeDiagonal[1]) - 1.5) < 0.01,
                "Diagonal movement must not exceed straight walking speed");

        simulation.updateOnFoot(state, 1.0 / 60.0, 0.0, 0.0, 0.0, true);
        require(!state.grounded && state.verticalVelocity > 0.0, "Space must start a jump");
        const double jumpVelocity = state.verticalVelocity;
        simulation.updateOnFoot(state, 1.0 / 60.0, 0.0, 0.0, 0.0, true);
        require(state.verticalVelocity < jumpVelocity, "Airborne jumps must not reset velocity");
        advance(simulation, state, 20);
        require(state.feetPosition[2] > 0.9, "Jump must lift the walker off the ground");
        advance(simulation, state, 120);
        require(state.grounded && std::abs(state.feetPosition[2]) < 0.002,
                "Gravity must land the walker");
        require(simulation.chassisPose().position == busPosition,
                "Walking must not move the frozen bus");

        StaticCollisionMesh platform;
        platform.vertices = {-10, -10, 2, 10, -10, 2, 10, 10, 2, -10, 10, 2};
        platform.indices = {0, 1, 2, 0, 2, 3};
        StaticCollisionMesh wall;
        wall.vertices = {2, -10, 0, 2, 10, 0, 2, 10, 5, 2, -10, 5};
        wall.indices = {0, 2, 1, 0, 3, 2};
        simulation.addMapTileCollision(0, 0, std::nullopt, {platform, wall});
        state = {{0.0, 0.0, 2.0}};
        advance(simulation, state, 120, 0.0, 1.0, 0.0);
        require(state.grounded && std::abs(state.feetPosition[2] - 2.0) < 0.002,
                "Walker must use an elevated road rather than the terrain below");
        require(state.feetPosition[0] <= 1.701 && state.feetPosition[0] >= 1.65,
                "Capsule must stop at static walls");
        simulation.removeMapTileCollision(0, 0);
        advance(simulation, state, 120);
        require(state.grounded && std::abs(state.feetPosition[2]) < 0.002,
                "Walker must fall when streamed support is removed");

        StaticCollisionMesh ceiling;
        ceiling.vertices = {-10, -10, 2.2, 10, -10, 2.2, 10, 10, 2.2, -10, 10, 2.2};
        ceiling.indices = {0, 2, 1, 0, 3, 2};
        simulation.addMapTileCollision(0, 0, std::nullopt, {ceiling});
        state = {{0.0, 0.0, 0.0}};
        advance(simulation, state, 1);
        simulation.updateOnFoot(state, 1.0 / 60.0, 0.0, 0.0, 0.0, true);
        advance(simulation, state, 12);
        require(state.feetPosition[2] <= 0.401 && state.verticalVelocity <= 0.0,
                "Jumping must stop at a low ceiling");
        advance(simulation, state, 60);
        require(state.grounded, "Walker must land after hitting a ceiling");
        simulation.removeMapTileCollision(0, 0);

        const auto beforePause = state.feetPosition;
        simulation.updateOnFoot(state, 0.0, 0.0, 1.0, 0.0, true);
        require(state.feetPosition == beforePause && state.verticalVelocity == 0.0,
                "A zero-duration update must not move or jump");
        simulation.updateOnFoot(state, 10.0, 0.0, 1.0, 0.0, false);
        require(std::abs(state.feetPosition[0] - beforePause[0] - 0.15) < 0.001,
                "A long frame must be capped to avoid teleporting through collision");

        TerrainCollisionGrid terrain;
        terrain.tileX = 13333;
        terrain.tileY = 0;
        terrain.tileSizeMeters = 300.0;
        terrain.intervals = 1;
        terrain.heights = {0, 30, 0, 30};
        simulation.addMapTileTerrainCollision(terrain.tileX, 0, terrain);
        const double origin = terrain.tileX * terrain.tileSizeMeters;
        state = {{origin + 10.0, 10.0, 1.2}};
        advance(simulation, state, 60);
        advance(simulation, state, 120, 0.0, 1.0, 0.0);
        require(state.feetPosition[0] > origin + 12.5 &&
                    std::abs(state.feetPosition[2] -
                             (state.feetPosition[0] - origin) * 0.1) < 0.02,
                "Walking must climb terrain at large map coordinates");
        advance(simulation, state, 120, 0.0, -1.0, 0.0);
        require(std::abs(state.feetPosition[2] -
                         (state.feetPosition[0] - origin) * 0.1) < 0.02,
                "Walking must descend terrain without falling through it");

        bool invalidRejected = false;
        try {
            simulation.updateOnFoot(state, std::numeric_limits<double>::quiet_NaN(),
                                    0.0, 0.0, 0.0, false);
        } catch (const std::invalid_argument&) {
            invalidRejected = true;
        }
        require(invalidRejected, "Invalid movement must be reported");
    } catch (const std::exception& error) {
        std::cerr << "On-foot movement probe failed: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
