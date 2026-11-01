#include "render/HumanRig.h"
#include "render/MatrixMath.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <numbers>
#include <stdexcept>

using namespace openbus::rendering;

namespace {

void require(bool condition, const char* message) {
    if (!condition)
        throw std::runtime_error(message);
}

bool near(double first, double second) {
    return std::abs(first - second) < 1.0e-6;
}

void runProbe() {
    ParsedObj mesh;
    mesh.positions.resize(4);
    mesh.bones = {{"left", {{0, 0.25F}, {0, 0.5F}, {1, 1.0F}, {9, 1.0F}}},
                  {"right", {{0, 0.5F}, {2, 0.5F}, {3, -1.0F},
                             {3, std::numeric_limits<float>::quiet_NaN()}}},
                  {"unbound", {{0, 1.0F}}}};
    const auto resolved = resolveHumanSkinWeights(mesh, {{"LEFT", -2}, {"right", -3}});
    require(resolved.hasWeights && resolved.invalidRecords == 3 && resolved.unboundBones == 1 &&
                resolved.unweightedVertices == 1,
            "weight diagnostics did not report invalid and unweighted records");
    require(near(resolved.vertices[0][0], 0.5) && near(resolved.vertices[0][1], 0.5) &&
                near(resolved.vertices[1][0], 1.0) && near(resolved.vertices[2][1], 1.0) &&
                near(resolved.vertices[3][0], 0.0),
            "duplicate, normalized, or unweighted vertex influences are incorrect");
    const auto conflict = resolveHumanSkinWeights(mesh, {{"left", -2}, {"LEFT", -3}});
    require(conflict.invalidRecords > 0 && !conflict.hasWeights,
            "conflicting bone bindings were accepted");

    openbus::config::HumanConfig config;
    config.numericRecords.push_back({"links",
                                     {},
                                     {0.1, -0.02, 0.9, 0.12, -0.03, 0.5, -0.04, 1.0,
                                      0.16, -0.02, 1.35, 0.4, -0.05, 1.3, -0.01, 1.5,
                                      0.65, -0.05, 1.25, 0.75, -0.04, 1.2},
                                     true});
    const auto joints = makeHumanJoints(config);
    require(joints && near(joints->hip[1], -0.02) && near(joints->elbow[1], -0.05) &&
                near(joints->hand[0], 0.65) && near(joints->neck[2], 1.5),
            "full authored joint coordinates were not preserved");
    HumanGaitParameters gait;
    gait.upperArmBetaDegrees = 0.0;
    const auto bind = makeHumanBonePalette(*joints, gait);
    for (const auto& matrix : bind) {
        const auto actual = transformPoint(matrix, {0.2, 0.3, 0.4, 1.0});
        require(near(actual[0], 0.2) && near(actual[1], 0.3) && near(actual[2], 0.4),
                "zero-angle bind pose was not identity");
    }
    gait.upperArmBetaDegrees = 90.0;
    const auto standing = makeHumanBonePalette(*joints, gait);
    const auto rightArm =
        transformPoint(standing[5], {joints->shoulder[0] + 0.2, joints->shoulder[1],
                                     joints->shoulder[2], 1.0});
    const auto leftArm =
        transformPoint(standing[4], {-joints->shoulder[0] - 0.2, joints->shoulder[1],
                                     joints->shoulder[2], 1.0});
    require(near(rightArm[0], joints->shoulder[0]) &&
                near(leftArm[0], -joints->shoulder[0]) &&
                near(rightArm[2], joints->shoulder[2] - 0.2) &&
                near(leftArm[2], joints->shoulder[2] - 0.2),
            "stationary arm lowering was not symmetric about authored shoulders");
    gait.phaseRadians = std::numbers::pi / 2.0;
    gait.weight = 1.0;
    const auto walking = makeHumanBonePalette(*joints, gait);
    const auto leftLeg = transformPoint(walking[0], {-0.1, -0.02, 0.6, 1.0});
    const auto rightLeg = transformPoint(walking[1], {0.1, -0.02, 0.6, 1.0});
    require((leftLeg[1] + 0.02) * (rightLeg[1] + 0.02) < 0.0,
            "walking legs did not swing in opposite directions");
}

} // namespace

int main() {
    try {
        runProbe();
    } catch (const std::exception& error) {
        std::cerr << "Human skinning probe: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
