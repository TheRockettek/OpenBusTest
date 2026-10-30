#include "ProbeTempDirectory.h"
#include "core/Logger.h"
#include "core/Variables.h"
#include "map/MapTrafficRuntime.h"
#include "map/MapTileWindow.h"
#include "render/AssetRequestManager.h"
#include "render/ComputeCulling.h"
#include "render/CoreRenderer.h"
#include "render/MatrixMath.h"
#include "render/OpenGLFunctions.h"
#include "render/map/MapRenderer.h"

#include <GLFW/glfw3.h>

#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

using namespace openbus::rendering;

namespace {

BeginQueryProc beginQuery = nullptr;
std::size_t occlusionQueryCount = 0;

void APIENTRY recordOcclusionQuery(GLenum target, GLuint query) {
    ++occlusionQueryCount;
    beginQuery(target, query);
}

void writeText(const std::filesystem::path& path, const std::string& text) {
    std::ofstream output(path, std::ios::binary);
    output << text;
    if (!output) {
        throw std::runtime_error("Could not write traffic-light fixture: " + path.string());
    }
}

void check(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void configureProbeLog(const std::filesystem::path& path) {
#ifdef _WIN32
    check(_putenv_s("OPENBUS_LOG_FILE", path.string().c_str()) == 0,
          "Could not configure probe log path");
#else
    check(setenv("OPENBUS_LOG_FILE", path.string().c_str(), 1) == 0,
          "Could not configure probe log path");
#endif
}

bool logContains(const std::filesystem::path& path, const std::string& text) {
    std::ifstream input(path);
    std::string line;
    while (std::getline(input, line)) {
        if (line.find(text) != std::string::npos) {
            return true;
        }
    }
    return false;
}

bool logContains(const std::filesystem::path& path, const std::string& first,
                 const std::string& second) {
    std::ifstream input(path);
    std::string line;
    while (std::getline(input, line)) {
        if (line.find(first) != std::string::npos && line.find(second) != std::string::npos) {
            return true;
        }
    }
    return false;
}

void writeTinyTerrain(const std::filesystem::path& path) {
    std::array<std::uint8_t, 20> terrain = {};
    terrain[0] = 1;
    std::ofstream output(path, std::ios::binary);
    output.write(reinterpret_cast<const char*>(terrain.data()),
                 static_cast<std::streamsize>(terrain.size()));
    check(static_cast<bool>(output), "Could not write probe terrain");
}

void writeBrokenTerrain(const std::filesystem::path& path) {
    const std::array<std::uint8_t, 2> terrain = {};
    std::ofstream output(path, std::ios::binary);
    output.write(reinterpret_cast<const char*>(terrain.data()),
                 static_cast<std::streamsize>(terrain.size()));
    check(static_cast<bool>(output), "Could not write broken probe terrain");
}

void writeTinyWater(const std::filesystem::path& path) {
    std::ofstream output(path, std::ios::binary);
    const std::array<std::uint32_t, 5> values = {
        1U, std::bit_cast<std::uint32_t>(1.0F), std::bit_cast<std::uint32_t>(1.5F),
        std::bit_cast<std::uint32_t>(2.0F), std::bit_cast<std::uint32_t>(2.5F)};
    for (const std::uint32_t value : values) {
        for (unsigned int byte = 0; byte < 4; ++byte) {
            output.put(static_cast<char>((value >> (byte * 8U)) & 0xffU));
        }
    }
    check(static_cast<bool>(output), "Could not write probe water sidecar");
}

void writeTinyTexture(const std::filesystem::path& path) {
    std::array<std::uint8_t, 21> image = {};
    image[2] = 2;
    image[12] = image[14] = 1;
    image[16] = 24;
    image[18] = 255;
    image[19] = 255;
    image[20] = 255;
    std::ofstream output(path, std::ios::binary);
    output.write(reinterpret_cast<const char*>(image.data()),
                 static_cast<std::streamsize>(image.size()));
    check(static_cast<bool>(output), "Could not write probe scenery texture");
}

std::string splineTileText(int splineId) {
    return "[version]\n14\n[terrain]\n[spline]\n0\nSplines/Test.sli\n" + std::to_string(splineId) +
           "\n0\n0\n10\n0\n20\n0\n10\n0\n0\n0\n0\n0\n0\n0\n0\n";
}

void writeLamp(const std::filesystem::path& path, const std::string& color) {
    writeText(path, "xof 0303txt 0032\n"
                    "Mesh { 3; -2;0;-2;, 2;0;-2;, 0;0;2;; 2; 3;0,1,2;, 3;2,1,0;;\n"
                    "MeshMaterialList { 1; 2; 0,0;; Material { " +
                        color + ";1;; 0; 0;0;0;; 1;1;1;; } } }\n");
}

void runProbe(const std::filesystem::path& root, bool conditional) {
    std::filesystem::create_directories(root / "model");
    std::filesystem::create_directories(root / "Splines");
    std::filesystem::create_directories(root / "Texture");
    std::filesystem::create_directories(root / "Script");
    writeText(root / "Splines" / "Test.sli",
              "[texture]\nroad.tga\n[profile]\n0\n[profilepnt]\n-2\n0.25\n0\n0.2\n"
              "[profilepnt]\n2\n0.25\n1\n0.2\n");
    writeTinyTexture(root / "Texture" / "road.tga");
    writeTinyTexture(root / "traf_LED.tga");
    writeLamp(root / "model" / "red.x", "1;0;0");
    writeLamp(root / "model" / "yellow.x", "1;1;0");
    writeLamp(root / "model" / "green.x", "0;1;0");
    writeText(root / "shared.sco", "[mesh]\nred.x\n");
    writeText(root / "retry.sco", "[tree]\nretry.tga\n10\n10\n1\n1\n");
    const std::array<std::uint8_t, 2> brokenTexture = {};
    {
        std::ofstream output(root / "retry.tga", std::ios::binary);
        output.write(reinterpret_cast<const char*>(brokenTexture.data()),
                     static_cast<std::streamsize>(brokenTexture.size()));
        check(static_cast<bool>(output), "Could not write broken probe scenery texture");
    }
    writeText(root / "head.sco",
              "[script]\n1\nscript\\traf_car.osc\n"
              "[varnamelist]\n1\nscript\\traf_car_varlist.txt\n"
              "[mesh]\nred.x\n[matl]\ntraf_LED.tga\n0\n[matl_alpha]\n2\n"
              "[visible]\nmain_red\n1\n"
              "[mesh]\nyellow.x\n[matl]\ntraf_LED.tga\n0\n[matl_alpha]\n2\n"
              "[visible]\nmain_yellow\n1\n"
              "[mesh]\ngreen.x\n[matl]\ntraf_LED.tga\n0\n[matl_alpha]\n2\n"
              "[visible]\nmain_green\n1\n");
    writeText(root / "support.sco",
              "[new_attachment]\nhead_mount\nattach_trans\n0\n0\n0\n");
    writeText(root / "Script" / "traf_car_varlist.txt",
              "main_Red\nmain_Yellow\nmain_Green\nsupp_Green\nTime_last\n");
    writeText(root / "Script" / "traf_car.osc",
              "{init}\n0 (S.L.Time_last)\n{end}\n"
              "{frame}\n"
              "(L.L.TrafficLightPhase) s0\n"
              "l0 0 >= l0 5 <= && (S.L.main_Red)\n"
              "l0 6 = l0 7 = || (S.L.main_Green)\n"
              "l0 1 = l0 4 = || l0 7 = || (S.L.supp_Green)\n"
              "l0 3 >= l0 5 <= && l0 9 >= l0 11 <= && || "
              "(S.L.main_Yellow)\n"
              "{end}\n");
    writeText(root / "controller.sco",
              "[onlyeditor]\n[traffic_lights_group]\n6\n"
              "[traffic_light]\nSelected approach\n[phase]\n0\n2\n"
              "[phase]\n6\n2\n[phase]\n9\n2\n"
              "[traffic_light]\nOther approach\n[phase]\n0\n6\n"
              "[traffic_light_jump]\n0\n10\n6\n2\n"
              "[path]\n60\n0\n0\n0\n0\n10\n0\n0\n0\n3\n0\n0\n"
              "[use_traffic_light]\n0\n"
              "[path]\n80\n0\n0\n0\n0\n10\n0\n0\n0\n3\n0\n0\n"
              "[use_traffic_light]\n1\n"
              "[mesh]\ngreen.x\n" +
                  std::string(conditional ? "[traffic_light_stop]\n1\n4\n0\n"
                                            "[traffic_light_stop]\n1\n8\n0\n"
                                          : ""));
    writeText(root / "tile.map",
              splineTileText(1) +
                      "[object]\n0\ncontroller.sco\n15340\n0\n0\n5\n0\n0\n0\n0\n"
                  "[rule]\n0\npriority\n192\n0\n"
                      "[object]\n0\nsupport.sco\n15400\n0\n0\n5\n0\n0\n0\n0\n0\n"
                      "[object]\n0\nsupport.sco\n15401\n80\n0\n5\n0\n0\n0\n0\n0\n"
                      "[attachObj]\n0\nhead.sco\n47862\n15400\n0\n0\n0\n0\n0\n0\n"
                      "[varparent]\n15340\n"
                      "[attachObj]\n0\nhead.sco\n47735\n15401\n0\n0\n0\n0\n0\n0\n"
                      "[varparent]\n15340\n"
                      "[attachObj]\n0\nshared.sco\n301\n204\n0\n0\n0\n0\n0\n0\n");
    {
        std::ofstream output(root / "tile.map", std::ios::binary | std::ios::app);
        output << "[water]\n";
        check(static_cast<bool>(output), "Could not append water marker to probe tile");
    }
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
    writeTinyTexture(root / "ground-layer.tga");
    writeTinyTexture(root / "tile.map.2.tga");
    writeTinyTerrain(root / "tile.terrain");
    writeTinyWater(root / "tile.map.water");
    for (const int tileX : {4, 5}) {
        const std::string tileName = "tile_" + std::to_string(tileX);
        writeText(root / (tileName + ".map"),
                  splineTileText(tileX + 2) +
                      (tileX == 4 ? "[splineAttachement]\n0\nshared.sco\n204\n0\n0\n0\n0\n0\n0\n"
                                    "0\n0\n0\n0\n0\n"
                                  : "[object]\n0\nshared.sco\n" + std::to_string(200 + tileX) +
                                        "\n0\n0\n0\n0\n0\n0\n0\n[object]\n0\nretry.sco\n250\n"
                                        "0\n0\n0\n0\n0\n0\n0\n") +
                      "[water]\n");
        writeTinyTexture(root / (tileName + ".map.2.tga"));
        if (tileX == 4) {
            writeBrokenTerrain(root / (tileName + ".terrain"));
        } else {
            writeTinyTerrain(root / (tileName + ".terrain"));
        }
        writeTinyWater(root / (tileName + ".map.water"));
    }
    openbus::map::MapDefinition map;
    map.rootPath = root;
    map.groundTextures.push_back({"ground.tga", "", {}});
    map.groundTextures.push_back({"ground-layer.tga", "", {}});
    openbus::map::MapTileReference tile;
    tile.textPath = root / "tile.map";
    tile.terrainPath = root / "tile.terrain";
    tile.hasTerrainFile = true;
    tile.waterPath = root / "tile.map.water";
    tile.hasWaterFile = true;
    tile.groundTextureSidecars.push_back({"2", root / "tile.map.2.tga"});
    map.tiles.push_back(tile);
    for (const int tileX : {4, 5}) {
        openbus::map::MapTileReference distantTile;
        const std::string tileName = "tile_" + std::to_string(tileX);
        distantTile.x = tileX;
        distantTile.textPath = root / (tileName + ".map");
        distantTile.terrainPath = root / (tileName + ".terrain");
        distantTile.hasTerrainFile = true;
        distantTile.waterPath = root / (tileName + ".map.water");
        distantTile.hasWaterFile = true;
        distantTile.groundTextureSidecars.push_back({"2", root / (tileName + ".map.2.tga")});
        map.tiles.push_back(std::move(distantTile));
    }
    SimulationState simulation;
    AssetRequestManager assets;
    MapRenderer renderer(map, 0, 0, 0, 0, root, simulation, assets);
    check(
        renderer.renderStats().totalRoadBatches == 1 &&
            renderer.renderStats().totalWaterBatches == 1 &&
            renderer.renderStats().totalTextures == 4,
        "Initial road, water, and asynchronously decoded ground-layer textures must be published");
    const auto& paths = renderer.sceneryTrafficPathSegments();
    check(    paths.size() == 2 && paths.front().trafficSignal.has_value() &&
              paths.front().pathIndex == 0 && paths.front().widthMeters == 3.0 &&
              paths.front().trafficSignal->lightIndex == 0 &&
              paths.front().trafficSignal->program != nullptr &&
              paths.front().trafficSignal->program->controlPoints.size() == (conditional ? 2U : 0U) &&
              paths.front().pathRules.groups.size() == 1 &&
              paths.front().pathRules.groups.front().priority == 192.0 &&
              paths.back().trafficSignal.has_value() &&
              paths.back().trafficSignal->lightIndex == 1,
          "SCO F2 path must retain its map traffic rule and controller/stopline data");
    const auto aiRoute = renderer.buildDrivingRoute(15340);
    check(aiRoute.has_value() && !aiRoute->segments.empty() &&
              aiRoute->segments.front().trafficSignal.has_value() &&
              aiRoute->segments.front().pathRules.groups.size() == 1 &&
              aiRoute->segments.front().pathRules.groups.front().priority == 192.0,
          "AI route from an SCO path must retain the same map rule and signal schedule");
    openbus::map::MapTrafficActor signalApproach;
    signalApproach.route = *aiRoute;
    check(openbus::map::mapTrafficActorWaitingAtSignal(signalApproach, 0.0),
          "AI actor on an SCO route must stop at its red signal");
    check(renderer.renderStats().activeSceneryObjects == 3,
          "Cross-tile spline parents must resolve for active attached scenery");
    setPerspective(32, 32, 90);
    setModelViewMatrix(translationMatrix({0, 0, -12}));
    glViewport(0, 0, 32, 32);
    glDisable(GL_DITHER);
    double previousClock = 0.0;
    const auto drawPhase = [&](double clock, int phase, std::array<int, 3> expected) {
        const double elapsed = clock + 0.1 - previousClock;
        simulation.sharedVariables().updateFrame(static_cast<float>(elapsed),
                                                 static_cast<float>(clock), 0, 0);
        openbus::map::advanceMapTrafficSignalControllers(renderer.trafficSignalControllers(), {},
                                                          elapsed, previousClock);
        previousClock = clock + 0.1;
        renderer.updateTrafficSignalScripts(clock);
        renderer.updateScripts();
        const auto stats = renderer.trafficSignalDebugStats();
        check(stats.activeSignalModels == 3 && stats.modelsWithoutPhase == 0 &&
                  stats.modelsByPhase[phase] >= 1 && stats.controllerDrivenModels == 3,
              "Visible head did not inherit its labelled editor-only controller phase");
        const auto signalHeads = renderer.trafficSignalHeadDebugInfo(clock);
        const auto firstHead = std::find_if(
            signalHeads.begin(), signalHeads.end(),
            [](const auto& head) { return head.objectId == 47862; });
        const auto secondHead = std::find_if(
            signalHeads.begin(), signalHeads.end(),
            [](const auto& head) { return head.objectId == 47735; });
        check(signalHeads.size() == 2 && firstHead != signalHeads.end() &&
                  firstHead->linkedJunctionObjectId == 15340 && firstHead->lightIndex == 0 &&
                  firstHead->phase == phase && firstHead->tileX == 0 && firstHead->tileY == 0 &&
                  secondHead != signalHeads.end() &&
                  secondHead->linkedJunctionObjectId == 15340 &&
                  secondHead->lightIndex == 1 && secondHead->phase == 0,
              "Heads sharing a junction resolve separate nearby signal-path light indices");
        glDepthMask(GL_TRUE);
        glClearColor(0, 0, 0, 0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        const std::size_t queriesBefore = occlusionQueryCount;
        renderer.draw();
        check(beginQuery == nullptr || occlusionQueryCount > queriesBefore,
              "Traffic-light phase did not exercise an occlusion query");
        std::array<std::uint8_t, 4> pixel = {};
        glReadPixels(16, 16, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, pixel.data());
        for (std::size_t channel = 0; channel < 3; ++channel) {
            if (std::abs(static_cast<int>(pixel[channel]) - expected[channel]) > 2) {
                std::cerr << "phase=" << phase << " pixel=" << static_cast<int>(pixel[0]) << ','
                          << static_cast<int>(pixel[1]) << ',' << static_cast<int>(pixel[2])
                          << '\n';
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
    openbus::map::advanceMapTrafficSignalControllers(
        renderer.trafficSignalControllers(), {}, 0.1, 6.1,
        renderer.trafficSignalBusPriorityRequests(playerBusPosition));
    renderer.updateTrafficSignalScripts(6.2);
    renderer.updateScripts();
    const auto priorityStats = renderer.trafficSignalDebugStats();
    check(priorityStats.modelsByPhase[6] >= 1 && priorityStats.controllerDrivenModels == 3,
          "player bus approaching the stopline activates its configured green priority phase");
    check(glGetError() == GL_NO_ERROR, "Traffic-light rendering produced an OpenGL error");

    SimulationState asyncSimulation;
    AssetRequestManager asyncAssets;
    MapRenderer asyncRenderer(map, 0, 0, 0, 0, root, asyncSimulation, asyncAssets, true);
    for (int attempt = 0; attempt < 1000 && !asyncRenderer.initialWindowReady(); ++attempt) {
        asyncRenderer.updateStreaming(0.0, 0.0);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    check(asyncRenderer.initialWindowReady(),
          "Asynchronously prepared map must eventually activate its initial tile window");
    asyncRenderer.updateTrafficSignalScripts(0.0);
    asyncRenderer.updateScripts();
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    asyncRenderer.draw();
    std::array<std::uint8_t, 4> asyncPixel = {};
    glReadPixels(16, 16, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, asyncPixel.data());
    check(asyncPixel[0] > 250 && asyncPixel[1] < 5 && asyncPixel[2] < 5,
          "Traffic-light lamp scripts must run after asynchronous initial-window loading");

    writeText(root / "signal_heads.map",
              splineTileText(10) +
                  "[object]\n0\nsupport.sco\n15400\n0\n0\n5\n0\n0\n0\n0\n0\n"
                  "[object]\n0\nsupport.sco\n15401\n80\n0\n5\n0\n0\n0\n0\n0\n"
                  "[attachObj]\n0\nhead.sco\n47862\n15400\n0\n0\n0\n0\n0\n1\n5\n"
                  "[varparent]\n15340\n"
                  "[attachObj]\n0\nhead.sco\n47735\n15401\n0\n0\n0\n0\n0\n1\n1\n"
                  "[varparent]\n15340\n");
    std::string crossTileController = "[onlyeditor]\n[traffic_lights_group]\n6\n";
    for (int lightIndex = 0; lightIndex < 7; ++lightIndex) {
        crossTileController += "[traffic_light]\nLight " + std::to_string(lightIndex) +
                               "\n[phase]\n" + (lightIndex == 1 ? "6" : "0") + "\n6\n";
    }
    for (int lightIndex = 0; lightIndex < 7; ++lightIndex) {
        crossTileController += "[path]\n" + std::to_string(lightIndex * 20) +
                               "\n0\n0\n0\n0\n10\n0\n0\n0\n3\n0\n0\n"
                               "[use_traffic_light]\n" +
                               std::to_string(lightIndex) + "\n";
    }
    crossTileController += "[mesh]\ngreen.x\n";
    writeText(root / "cross_tile_controller.sco", crossTileController);
    writeText(root / "signal_controller.map",
              splineTileText(11) +
                  "[object]\n0\ncross_tile_controller.sco\n15340\n0\n0\n5\n0\n0\n0\n0\n");
    openbus::map::MapDefinition crossTileMap;
    crossTileMap.rootPath = root;
    crossTileMap.coordinates = map.coordinates;
    crossTileMap.groundTextures = map.groundTextures;
    for (int tileX = 0; tileX <= 1; ++tileX) {
        openbus::map::MapTileReference signalTile;
        signalTile.x = tileX;
        signalTile.textPath = root / (tileX == 0 ? "signal_heads.map" : "signal_controller.map");
        signalTile.terrainPath = root / "tile.terrain";
        signalTile.hasTerrainFile = true;
        crossTileMap.tiles.push_back(std::move(signalTile));
    }
    SimulationState crossTileSimulation;
    AssetRequestManager crossTileAssets;
    MapRenderer crossTileRenderer(crossTileMap, 0, 0, 0, 0, root, crossTileSimulation,
                                  crossTileAssets, true);
    for (int attempt = 0; attempt < 1000 && !crossTileRenderer.initialWindowReady(); ++attempt) {
        crossTileRenderer.updateStreaming(0.0, 0.0);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    check(crossTileRenderer.initialWindowReady(), "Cross-tile signal fixture must finish loading");
    const auto crossTileHeads = crossTileRenderer.trafficSignalHeadDebugInfo(0.0);
    const auto crossTileFirst = std::find_if(
        crossTileHeads.begin(), crossTileHeads.end(),
        [](const auto& head) { return head.objectId == 47862; });
    const auto crossTileSecond = std::find_if(
        crossTileHeads.begin(), crossTileHeads.end(),
        [](const auto& head) { return head.objectId == 47735; });
    check(crossTileFirst != crossTileHeads.end() && crossTileFirst->lightIndex == 5 &&
              crossTileFirst->phase == 0 && crossTileSecond != crossTileHeads.end() &&
              crossTileSecond->lightIndex == 1 && crossTileSecond->phase == 6,
          "Asynchronously loaded cross-tile heads must resolve their explicit SCO light indices");

    const std::vector<openbus::map::MapTileCoordinate> originalWindow = {{0, 0}};
    const std::vector<openbus::map::MapTileCoordinate> nextWindow = {{4, 0}, {5, 0}};
    check(renderer.activeTileCoordinates() == originalWindow,
          "Initial map window must contain only the spawn tile");
    renderer.updateStreaming(4.0 * map.coordinates.tileSize, 0.0);
    check(renderer.activeTileCoordinates() == originalWindow,
          "Current active window must remain until the requested tiles are ready");
    for (int attempt = 0; attempt < 50; ++attempt) {
        renderer.updateStreaming(4.0 * map.coordinates.tileSize, 0.0);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    writeTinyTerrain(root / "tile_4.terrain");
    const std::filesystem::path probeLog = root / "probe.log";
    const auto retryDeadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    bool assetRetryObserved = false;
    while (std::chrono::steady_clock::now() < retryDeadline) {
        renderer.updateStreaming(4.0 * map.coordinates.tileSize, 0.0);
        if (logContains(probeLog, "scenery/model asset preparation failed for", "retrying in")) {
            assetRetryObserved = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    check(renderer.activeTileCoordinates() == originalWindow,
          "Tile window must remain on the prior view while scenery asset retries are pending");
    check(assetRetryObserved, "Scenery preparation failure must be reported for retry");
    writeTinyTexture(root / "retry.tga");
    bool nextWindowActivated = false;
    const auto activationDeadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (std::chrono::steady_clock::now() < activationDeadline) {
        renderer.updateStreaming(4.0 * map.coordinates.tileSize, 0.0);
        if (renderer.activeTileCoordinates() == nextWindow) {
            nextWindowActivated = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    check(
        nextWindowActivated,
        "Background tile parsing and bounded publication must eventually activate the new window");
    const MapRenderStats streamedStats = renderer.renderStats();
    check(streamedStats.activeRoadBatches == 2 && streamedStats.totalRoadBatches == 3,
          "Background road geometry preparation must publish both streamed tiles");
    check(streamedStats.totalWaterBatches == 3,
          "Background water sidecar preparation must publish both streamed tiles");
    check(streamedStats.activeSceneryObjects == 3,
          "Tiles sharing a scenery asset must both publish their placements");
    check(streamedStats.totalTextures == 7,
          "Streamed road and terrain tiles must reuse shared textures and publish tile masks");

    openbus::map::MapDefinition startupMap;
    startupMap.rootPath = root;
    startupMap.coordinates = map.coordinates;
    startupMap.groundTextures = map.groundTextures;
    {
        const std::array<std::uint8_t, 2> brokenTexture = {};
        std::ofstream output(root / "startup_shared.tga", std::ios::binary);
        output.write(reinterpret_cast<const char*>(brokenTexture.data()),
                     static_cast<std::streamsize>(brokenTexture.size()));
        check(static_cast<bool>(output), "Could not write broken startup texture");
    }
    for (int assetIndex = 0; assetIndex < 9; ++assetIndex) {
        const std::string assetName = "startup_batch_" + std::to_string(assetIndex);
        writeText(root / (assetName + ".sco"), "[tree]\nstartup_shared.tga\n10\n10\n1\n1\n");
    }
    for (int tileY = -1; tileY <= 1; ++tileY) {
        for (int tileX = -1; tileX <= 1; ++tileX) {
            const std::filesystem::path startupTilePath =
                root /
                ("startup_tile_" + std::to_string(tileX) + "_" + std::to_string(tileY) + ".map");
            std::string startupObjects;
            for (int assetIndex = 0; assetIndex < 9; ++assetIndex) {
                const std::string assetName = "startup_batch_" + std::to_string(assetIndex);
                const int objectId = ((tileX + 1) * 3 + tileY + 1) * 100 + assetIndex;
                startupObjects += "[object]\n0\n" + assetName + ".sco\n" +
                                  std::to_string(objectId) + "\n0\n0\n0\n0\n0\n0\n0\n";
            }
            writeText(startupTilePath, "[version]\n14\n[terrain]\n" + startupObjects);
            openbus::map::MapTileReference tileReference;
            tileReference.x = tileX;
            tileReference.y = tileY;
            tileReference.textPath = startupTilePath;
            tileReference.terrainPath = root / "tile.terrain";
            tileReference.hasTerrainFile = true;
            startupMap.tiles.push_back(std::move(tileReference));
        }
    }
    MapRenderer startupRenderer(startupMap, 0, 0, 0, 0, root, simulation, assets, true);
    check(!startupRenderer.initialWindowReady(),
          "Menu startup map preparation must return before the initial tile window is ready");
    const auto startupRetryDeadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    bool startupAssetRetryObserved = false;
    bool startupLaneLookupChecked = false;
    while (std::chrono::steady_clock::now() < startupRetryDeadline) {
        startupRenderer.updateStreaming(0, 0);
        if (!startupLaneLookupChecked &&
            logContains(probeLog, "Map index phase complete:", "9 tiles in")) {
            check(startupRenderer.trafficSpawnLanes().empty(),
                  "Spawn-lane lookup must remain available while startup tiles are pending");
            check(!startupRenderer.selectTrafficSpawnLane({}, 1).has_value(),
                  "Spawn selection must defer while startup lane data is unavailable");
            startupLaneLookupChecked = true;
        }
        if (logContains(probeLog, "startup_batch_", "attempt 1 of 3")) {
            startupAssetRetryObserved = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    check(startupAssetRetryObserved,
          "A batch of failed scenery assets must consume one tile retry attempt");
    check(startupLaneLookupChecked,
          "Spawn-lane lookup must be exercised while startup tiles are pending");
    writeTinyTexture(root / "startup_shared.tga");
    const auto startupDeadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (!startupRenderer.initialWindowReady() &&
           std::chrono::steady_clock::now() < startupDeadline) {
        startupRenderer.updateStreaming(0, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    check(startupRenderer.initialWindowReady(),
          "Initial map window must become ready while the startup menu remains active");
    check(logContains(probeLog, "preparing road traffic lane network asynchronously"),
          "Traffic lane graph finalization must be dispatched to a worker thread");
    check(logContains(probeLog, "preparing human traffic lane network asynchronously"),
          "Human lane graph finalization must be dispatched to a worker thread");
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
    const char* backendSetting = std::getenv("OPENBUS_CULLING_BACKEND");
    const bool indirect = backendSetting != nullptr &&
                          std::string_view(backendSetting) == "compute-indirect";
    const char* querySetting = std::getenv("OPENBUS_OCCLUSION_CULLING");
    const bool queriesRequested =
        querySetting != nullptr && std::string_view(querySetting) == "queries";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, indirect ? 4 : 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    GLFWwindow* window = glfwCreateWindow(32, 32, "Traffic-light probe", nullptr, nullptr);
    if (window == nullptr) {
        glfwTerminate();
        return 77;
    }
    glfwMakeContextCurrent(window);
    const auto root = openbus::probe::makeProbeDirectory("openbus-traffic-light-rendering");
    configureProbeLog(root / "probe.log");
    bool passed = loadOpenGLFunctions() && initializeCoreRenderer();
    bool unavailable = false;
    if (passed && indirect && !indirectCullingAvailable())
        unavailable = true;
    if (passed && queriesRequested) {
        unavailable |= pglGenQueries == nullptr || pglDeleteQueries == nullptr ||
                       pglBeginQuery == nullptr || pglEndQuery == nullptr ||
                       pglBeginConditionalRender == nullptr || pglEndConditionalRender == nullptr;
        if (!unavailable) {
            beginQuery = pglBeginQuery;
            pglBeginQuery = recordOcclusionQuery;
        }
    }
    if (passed && !unavailable) {
        try {
            runProbe(root, argc == 2);
            check(!queriesRequested || occlusionQueryCount > 0,
                  "Traffic-light fixture did not exercise occlusion queries");
        } catch (const std::exception& error) {
            std::cerr << error.what() << '\n';
            passed = false;
        }
        if (beginQuery != nullptr) pglBeginQuery = beginQuery;
    }
    shutdownCoreRenderer();
    glfwDestroyWindow(window);
    glfwTerminate();
    openbus::probe::removeProbeDirectory(root);
    return unavailable ? 77 : passed ? 0 : 1;
}
