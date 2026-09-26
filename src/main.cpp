#include "BusConfiguration.h"
#include "BusSimulation.h"
#include "CrashHandler.h"
#include "Logger.h"
#include "ModelConfigLoader.h"
#include "PerfTrace.h"
#include "Renderer.h"

#include <GLFW/glfw3.h>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <vector>

int main() {
    installCrashHandler();
    Logger applicationLog("Application");
    try {
        applicationLog.Log("Starting OpenBus");
        const BusVehicle vehicle = busVehicleFromEnvironment();

        // Resolve configuration paths for the bus and model.
        std::filesystem::path busConfigPath;
        std::filesystem::path modelConfigPath;
        {
            openbus::rendering::TraceScope trace("config", "main.resolveConfigurationPaths");
            busConfigPath = busConfigurationPathFor(vehicle);
            modelConfigPath = modelConfigurationPathForBus(busConfigPath);
        }

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

        BusSimulation simulation(configuration);

        Renderer renderer(1280, 720, "OpenBus");
        BusModel* playerBusModel = renderer.AddBusModel(
            vehicle, {0, 0, 20}, {AssetLoadingMode::Deferred, AssetLoadingMode::Eager});
        renderer.SetPlayerBusModel(playerBusModel);

        double previousTime = glfwGetTime();
        bool captureOnStartup = std::getenv("OPENBUS_CAPTURE_VIEWS") != nullptr;
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

            const std::vector<KeyEvent> frameKeyEvents = renderer.consumeKeyEvents();
            pendingKeyEvents.insert(pendingKeyEvents.end(), frameKeyEvents.begin(),
                                    frameKeyEvents.end());
            simulation.update(elapsed, renderer.throttle(), renderer.steering(), renderer.brake());
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
        applicationLog.Log("OpenBus shut down cleanly");
    } catch (const std::exception& error) {
        applicationLog.Log(std::string("OpenBus failed: ") + error.what());
        std::cerr << "OpenBus failed: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
