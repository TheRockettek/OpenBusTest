#include "BusConfiguration.h"
#include "BusSimulation.h"
#include "CrashHandler.h"
#include "Environment.h"
#include "Logger.h"
#include "MapCollisionStreamer.h"
#include "MapConfigLoader.h"
#include "ModelConfigLoader.h"
#include "PerfTrace.h"
#include "RenderLoop.h"

#include <GLFW/glfw3.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

std::string environmentValue(const char* name) {
    const char* value = openbus::getEnvironment(name);
    return value == nullptr ? std::string{} : std::string(value);
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
            (traceEnabled.empty() ? std::string("disabled") : traceEnabled) + ", output=" +
            (tracePath.empty() ? std::string("openbus_trace.json") : tracePath) + ".");
#else
        applicationLog.Log("Performance tracing instrumentation is not compiled in.");
#endif

        // Resolve configuration paths for the bus and model.
        std::filesystem::path busConfigPath;
        std::filesystem::path modelConfigPath;
        std::filesystem::path aiBusConfigPath;
        std::filesystem::path aiModelConfigPath;
        std::filesystem::path mapDirectory;
        int aiVehicleCount = 0;
        {
            openbus::rendering::TraceScope trace("config", "main.resolveConfigurationPaths");
            busConfigPath = busConfigurationPathFor();
            modelConfigPath = modelConfigurationPathForBus(busConfigPath);
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
            const std::string configuredMapPath = environmentValue("OPENBUS_MAP_PATH");
            mapDirectory = omsiRootPath() / "maps" /
                           (configuredMapPath.empty() ? "Grande Porto 2022" : configuredMapPath);
            if (mapDirectory.is_relative()) {
                mapDirectory = omsiRootPath() / mapDirectory;
            }
        }

        openbus::map::MapDefinition mapDefinition;
        {
            openbus::rendering::TraceScope trace("map", "main.loadMapDefinition");
            mapDefinition = openbus::map::loadMapDefinition(mapDirectory);
        }
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
        const std::string configuredSpawn = environmentValue("OPENBUS_MAP_SPAWN");
        const std::string configuredEntryPoint = environmentValue("OPENBUS_MAP_ENTRY");
        if (configuredSpawn.empty()) {
            applicationLog.Log("Map spawn points (set OPENBUS_MAP_SPAWN to a zero-based index):");
            if (mapDefinition.entryPoints.empty()) {
                applicationLog.Log("  (no spawn points in global.cfg)");
            }
            for (std::size_t index = 0; index < mapDefinition.entryPoints.size(); ++index) {
                const openbus::map::MapEntryPoint& entryPoint = mapDefinition.entryPoints[index];
                const openbus::map::MapTileReference& tile =
                    mapDefinition.tiles[static_cast<std::size_t>(entryPoint.tileIndex)];
                applicationLog.Log(
                    "  [" + std::to_string(index) + "] " +
                    (entryPoint.name.empty() ? std::string("(unnamed)") : entryPoint.name) +
                    " - tile (" + std::to_string(tile.x) + ", " + std::to_string(tile.y) +
                    "), position " + formatPlacement(entryPoint.placement) + " m, yaw " +
                    std::to_string(entryPoint.placement.yawDegrees) + " deg");
            }
        }
        const std::size_t spawnEntryPoint =
            !configuredSpawn.empty()
                ? openbus::map::selectMapSpawnPoint(mapDefinition, configuredSpawn)
                : (!configuredEntryPoint.empty()
                       ? openbus::map::selectMapEntryPoint(mapDefinition, configuredEntryPoint)
                       : openbus::map::selectMapSpawnPoint(mapDefinition, {}));
        const std::size_t groundTextureIndex = openbus::map::selectMapGroundTextureIndex(
            mapDefinition, environmentValue("OPENBUS_MAP_GROUNDTEX"));
        const VehiclePlacement busPlacement = mapDefinition.entryPoints[spawnEntryPoint].placement;
        applicationLog.Log("Map " + mapDirectory.string() + " spawn: [" +
                           std::to_string(spawnEntryPoint) + "] " +
                           mapDefinition.entryPoints[spawnEntryPoint].name + " at " +
                           formatPlacement(busPlacement) + " m, yaw " +
                           std::to_string(busPlacement.yawDegrees) + " deg");

        BusConfiguration configuration;
        {
            openbus::rendering::TraceScope trace("config", "main.loadBusConfiguration");
            configuration = loadBusConfiguration(busConfigPath);
        }

        ModelConfig modelConfiguration;
        {
            openbus::rendering::TraceScope trace("config", "main.loadBusModelConfiguration");
            modelConfiguration = loadBusModelConfiguration(busConfigPath);
        }

        {
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

        BusSimulation simulation(configuration, busPlacement, 60.0, 8, busPlacement.position[2], {},
                                 {}, false);
        openbus::map::MapCollisionStreamer collisionStreamer(mapDefinition, omsiRootPath(),
                                                             simulation);
        {
            openbus::rendering::TraceScope trace("map", "main.loadSpawnCollisionNeighborhood");
            collisionStreamer.update(busPlacement.position[0], busPlacement.position[1]);
        }
        applicationLog.Log("Map collision streaming enabled: resident tiles=" +
                           std::to_string(collisionStreamer.loadedTileCount()) +
                           " around spawn; map manifest tiles=" +
                           std::to_string(mapDefinition.tiles.size()) + ".");

        const bool benchmarkMode =
            openbus::rendering::parseEnabledFlag(openbus::getEnvironment("OPENBUS_BENCHMARK"));
        const int windowWidth =
            benchmarkMode ? positiveEnvironmentInt("OPENBUS_BENCHMARK_WIDTH", 1280, 16384) : 1920;
        const int windowHeight =
            benchmarkMode ? positiveEnvironmentInt("OPENBUS_BENCHMARK_HEIGHT", 720, 16384) : 1080;
        RenderLoop renderer(windowWidth, windowHeight, "OpenBus");
        renderer.SetMap(mapDefinition, spawnEntryPoint, groundTextureIndex, omsiRootPath());
        Vehicle* playerVehicle =
            renderer.AddVehicle(busConfigPath, modelConfigPath, busPlacement,
                                {AssetLoadingMode::Deferred, AssetLoadingMode::Eager});
        renderer.SetPlayerVehicle(playerVehicle);
        if (!aiBusConfigPath.empty()) {
            std::cout << "AI_VEHICLES=" << aiVehicleCount << '\n';
            for (int index = 0; index < aiVehicleCount; ++index) {
                renderer.AddVehicle(aiBusConfigPath, aiModelConfigPath,
                                    aiVehiclePlacement(index, aiVehicleCount),
                                    {AssetLoadingMode::Deferred, AssetLoadingMode::Eager});
            }
        }

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
                renderer.updatePostPhysicsVariables(simulation);
                collisionStreamer.update(simulation.positionX(), simulation.positionY());
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
            bool captureOnStartup = openbus::getEnvironment("OPENBUS_CAPTURE_VIEWS") != nullptr;
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
                renderer.updatePostPhysicsVariables(simulation);
                collisionStreamer.update(simulation.positionX(), simulation.positionY());
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
