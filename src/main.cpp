#include "BusSimulation.h"
#include "BusConfiguration.h"
#include "CrashHandler.h"
#include "Logger.h"
#include "ModelConfigLoader.h"
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
        const BusVehicle vehicle = busVehicleFromEnvironment();
        applicationLog.Log("Starting OpenBus");
        const std::filesystem::path busConfigPath = busConfigurationPathFor(vehicle);
        const std::filesystem::path modelConfigPath = modelConfigurationPathForBus(busConfigPath);
        const BusConfiguration configuration = loadBusConfiguration(busConfigPath);
        const ModelConfig modelConfiguration = loadBusModelConfiguration(busConfigPath);
        std::ofstream configurationOutput("OpenBus_configuration.json", std::ios::trunc);
        if (!configurationOutput) {
            applicationLog.Log("Failed to open OpenBus_configuration.json");
            throw std::runtime_error("Failed to open OpenBus_configuration.json");
        }
        writeBusConfigurationJson(configurationOutput, configuration);
        configurationOutput.flush();
        std::ofstream modelConfigurationOutput("OpenBus_model_configuration.json", std::ios::trunc);
        if (!modelConfigurationOutput) {
            applicationLog.Log("Failed to open OpenBus_model_configuration.json");
            throw std::runtime_error("Failed to open OpenBus_model_configuration.json");
        }
        writeModelConfigurationJson(modelConfigurationOutput, modelConfigPath, modelConfiguration);
        modelConfigurationOutput.flush();
        BusSimulation simulation(configuration);
        Renderer renderer(1280, 720, "OpenBus", vehicle,
                          {AssetLoadingMode::Deferred, AssetLoadingMode::Eager});
        std::ofstream diagnostics("OpenBus_physics.json", std::ios::trunc);
        if (!diagnostics) {
            applicationLog.Log("Failed to open OpenBus_physics.json");
            throw std::runtime_error("Failed to open OpenBus_physics.json for diagnostics");
        }

        applicationLog.Log("Diagnostics files opened");
        diagnostics << "{\n"
                       "  \"format_version\": 2,\n"
                       "  \"sample_rate_hz\": 5,\n"
                       "  \"samples\": [\n";
        diagnostics.flush();

        double previousTime = glfwGetTime();
        bool captureOnStartup = std::getenv("OPENBUS_CAPTURE_VIEWS") != nullptr;
        bool pendingCaptureRequest = false;
        std::vector<KeyEvent> pendingKeyEvents;
        while (!renderer.shouldClose()) {
            const double currentTime = glfwGetTime();
            const double elapsed = currentTime - previousTime;
            previousTime = currentTime;

            renderer.beginFrame();
            const std::vector<KeyEvent> frameKeyEvents = renderer.consumeKeyEvents();
            pendingKeyEvents.insert(pendingKeyEvents.end(), frameKeyEvents.begin(),
                                    frameKeyEvents.end());
            simulation.update(elapsed, renderer.throttle(), renderer.steering(), renderer.brake());
            renderer.draw(simulation);
            if (captureOnStartup) {
                pendingCaptureRequest = true;
            }
            if (renderer.consumeCaptureRequest()) {
                pendingCaptureRequest = true;
            }
            if (pendingCaptureRequest && renderer.isCaptureReady()) {
                pendingCaptureRequest = false;
                captureOnStartup = false;
                renderer.captureViews(simulation, "screenshots");
                exit(0);
            }
            renderer.endFrame();
        }
        diagnostics << "\n  ]\n}\n";
        diagnostics.flush();
        applicationLog.Log("OpenBus shut down cleanly");
    } catch (const std::exception& error) {
        applicationLog.Log(std::string("OpenBus failed: ") + error.what());
        std::cerr << "OpenBus failed: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
