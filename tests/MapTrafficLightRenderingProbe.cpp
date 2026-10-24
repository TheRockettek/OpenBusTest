#include "ProbeTempDirectory.h"
#include "core/Variables.h"
#include "render/AssetRequestManager.h"
#include "render/CoreRenderer.h"
#include "render/MatrixMath.h"
#include "render/OpenGLFunctions.h"
#include "render/map/MapRenderer.h"

#include <GLFW/glfw3.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

using namespace openbus::rendering;

namespace {

void writeText(const std::filesystem::path& path, const std::string& text) {
    std::ofstream output(path, std::ios::binary);
    output << text;
    if (!output) {
        throw std::runtime_error("Could not write traffic-light fixture: " + path.string());
    }
}

void writeLamp(const std::filesystem::path& path, const std::string& color) {
    writeText(path, "xof 0303txt 0032\n"
                    "Mesh { 3; -2;0;-2;, 2;0;-2;, 0;0;2;; 2; 3;0,1,2;, 3;2,1,0;;\n"
                    "MeshMaterialList { 1; 2; 0,0;; Material { " +
                        color + ";1;; 0; 0;0;0;; 1;1;1;; } } }\n");
}

void check(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void runProbe(const std::filesystem::path& root, bool conditional) {
    std::filesystem::create_directories(root / "model");
    writeLamp(root / "model" / "red.x", "1;0;0");
    writeLamp(root / "model" / "yellow.x", "1;1;0");
    writeLamp(root / "model" / "green.x", "0;1;0");
    writeText(root / "head.sco",
              "[trafficlight]\n[script]\n1\nlamp.osc\n[varnamelist]\n1\nlamp_vars.txt\n"
              "[mesh]\nred.x\n[visible]\nlamp_red\n1\n"
              "[mesh]\nyellow.x\n[visible]\nlamp_yellow\n1\n"
              "[mesh]\ngreen.x\n[visible]\nlamp_green\n1\n");
    writeText(root / "lamp_vars.txt", "lamp_red\nlamp_yellow\nlamp_green\n");
    writeText(root / "lamp.osc",
              "{frame}\n"
              "(L.L.TrafficLightPhase) 0 = (S.L.lamp_red)\n"
              "(L.L.TrafficLightPhase) 9 = (S.L.lamp_yellow)\n"
              "(L.L.TrafficLightPhase) 6 = (S.L.lamp_green)\n{end}\n");
    writeText(root / "controller.sco",
              "[onlyeditor]\n[traffic_lights_group]\n6\n"
              "[traffic_light]\nOther approach\n[phase]\n0\n6\n"
              "[traffic_light]\nSelected approach\n[phase]\n0\n2\n"
              "[phase]\n6\n2\n[phase]\n9\n2\n"
              "[traffic_light_jump]\n0\n10\n6\n2\n"
              "[path]\n0\n0\n0\n0\n0\n20\n0\n0\n0\n3\n0\n0\n"
              "[use_traffic_light]\n1\n"
              "[mesh]\ngreen.x\n" +
                  std::string(conditional ? "[traffic_light_stop]\n1\n4\n0\n" : ""));
    writeText(root / "tile.map",
              "[version]\n14\n[terrain]\n"
              "[object]\n0\ncontroller.sco\n100\n0\n0\n0\n0\n0\n0\n0\n"
              "[object]\n0\nhead.sco\n101\n0\n0\n1\n0\n0\n0\n1\n1\n[varparent]\n100\n");
    // A one-pixel uncompressed TGA supplies the mandatory map ground texture.
    std::array<std::uint8_t, 21> ground = {};
    ground[2] = 2;
    ground[12] = ground[14] = 1;
    ground[16] = 24;
    {
        std::ofstream output(root / "ground.tga", std::ios::binary);
        output.write(reinterpret_cast<const char*>(ground.data()), ground.size());
        check(static_cast<bool>(output), "Could not write probe ground texture");
    }
    {
        std::array<std::uint8_t, 20> terrain = {};
        terrain[0] = 1;
        std::ofstream output(root / "tile.terrain", std::ios::binary);
        output.write(reinterpret_cast<const char*>(terrain.data()), terrain.size());
        check(static_cast<bool>(output), "Could not write probe terrain");
    }
    openbus::map::MapDefinition map;
    map.rootPath = root;
    map.groundTextures.push_back({"ground.tga", "", {}});
    openbus::map::MapTileReference tile;
    tile.textPath = root / "tile.map";
    tile.terrainPath = root / "tile.terrain";
    tile.hasTerrainFile = true;
    map.tiles.push_back(tile);
    SimulationState simulation;
    AssetRequestManager assets;
    MapRenderer renderer(map, 0, 0, 0, 0, root, simulation, assets);
    const auto& paths = renderer.sceneryTrafficPathSegments();
    check(paths.size() == 1 && paths.front().trafficSignal.has_value() &&
              paths.front().pathIndex == 0 && paths.front().widthMeters == 3.0 &&
              paths.front().trafficSignal->lightIndex == 1 &&
              paths.front().trafficSignal->program != nullptr &&
              paths.front().trafficSignal->program->controlPoints.size() ==
                  (conditional ? 1U : 0U),
          "F2 signal debug path must retain controller phase and stopline data");
    check(renderer.renderStats().activeSceneryObjects == 1,
          "Editor-only junction geometry must remain hidden");
    setPerspective(32, 32, 90);
    setModelViewMatrix(translationMatrix({0, 0, -2}));
    glViewport(0, 0, 32, 32);
    glDisable(GL_DITHER);
    double previousClock = 0.0;
    const auto drawPhase = [&](double clock, int phase, std::array<int, 3> expected) {
        const double elapsed = clock + 0.1 - previousClock;
        simulation.sharedVariables().updateFrame(static_cast<float>(elapsed),
                                                  static_cast<float>(clock), 0, 0);
        renderer.updateTrafficSignalControllers({}, elapsed, previousClock);
        previousClock = clock + 0.1;
        renderer.updateScripts();
        const auto stats = renderer.trafficSignalDebugStats();
        check(stats.activeSignalModels == 2 && stats.modelsWithoutPhase == 0 &&
                  stats.modelsByPhase[phase] >= 1 &&
                  stats.controllerDrivenModels == 2,
              "Visible head did not inherit its labelled editor-only controller phase");
        glDepthMask(GL_TRUE);
        glClearColor(0, 0, 0, 0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        renderer.draw();
        std::array<std::uint8_t, 4> pixel = {};
        glReadPixels(16, 16, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, pixel.data());
        for (std::size_t channel = 0; channel < 3; ++channel) {
            if (std::abs(static_cast<int>(pixel[channel]) - expected[channel]) > 2) {
                std::cerr << "phase=" << phase << " pixel=" << static_cast<int>(pixel[0])
                          << ',' << static_cast<int>(pixel[1]) << ','
                          << static_cast<int>(pixel[2]) << '\n';
                throw std::runtime_error("Traffic-light lamp pixels did not change colour");
            }
        }
    };
    drawPhase(0, 0, {255, 0, 0});
    drawPhase(2, 6, {0, 255, 0});
    drawPhase(4, 9, {255, 255, 0});
    drawPhase(6, 0, {255, 0, 0});
    const auto& signalPath = paths.front().points;
    check(signalPath.size() >= 2, "bus-priority signal path has direction geometry");
    const double directionX = signalPath[1][0] - signalPath[0][0];
    const double directionY = signalPath[1][1] - signalPath[0][1];
    const double directionLength = std::hypot(directionX, directionY);
    check(directionLength > 0.0, "bus-priority signal path has non-zero direction");
    const std::array<double, 2> playerBusPosition = {
        signalPath[0][0] - directionX / directionLength * 5.0,
        signalPath[0][1] - directionY / directionLength * 5.0};
    renderer.updateTrafficSignalControllers({}, 0.1, 6.1, playerBusPosition);
    renderer.updateScripts();
    const auto priorityStats = renderer.trafficSignalDebugStats();
    check(priorityStats.modelsByPhase[6] >= 1 &&
              priorityStats.controllerDrivenModels == 2,
          "player bus approaching the stopline activates its configured green priority phase");
    check(glGetError() == GL_NO_ERROR, "Traffic-light rendering produced an OpenGL error");
}

} // namespace

int main(int argc, char** argv) {
    if (argc > 2 || (argc == 2 && std::string(argv[1]) != "--conditional")) {
        std::cerr << "Expected no arguments or --conditional\n";
        return 1;
    }
    if (!glfwInit()) {
        return 77;
    }
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    GLFWwindow* window = glfwCreateWindow(32, 32, "Traffic-light probe", nullptr, nullptr);
    if (window == nullptr) {
        glfwTerminate();
        return 77;
    }
    glfwMakeContextCurrent(window);
    bool passed = loadOpenGLFunctions() && initializeCoreRenderer();
    const auto root = openbus::probe::makeProbeDirectory("openbus-traffic-light-rendering");
    if (passed) {
        try {
            runProbe(root, argc == 2);
        } catch (const std::exception& error) {
            std::cerr << error.what() << '\n';
            passed = false;
        }
    }
    shutdownCoreRenderer();
    glfwDestroyWindow(window);
    glfwTerminate();
    openbus::probe::removeProbeDirectory(root);
    return passed ? 0 : 1;
}
