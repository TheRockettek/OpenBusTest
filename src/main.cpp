#include "BusSimulation.h"
#include "BusConfiguration.h"
#include "CrashHandler.h"
#include "Logger.h"
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
        BusSimulation simulation(busConfigurationFor(vehicle));
        Renderer renderer(1280, 720, "OpenBus", vehicle,
                  {AssetLoadingMode::Eager, AssetLoadingMode::Eager});
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
        double diagnosticsAccumulator = 0.0;
        bool firstDiagnosticSample = true;
        bool captureOnStartup = std::getenv("OPENBUS_CAPTURE_VIEWS") != nullptr;
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
            if ((captureOnStartup && currentTime >= 10.0) || renderer.consumeCaptureRequest()) {
                captureOnStartup = false;
                renderer.captureViews(simulation, "screenshots");
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
