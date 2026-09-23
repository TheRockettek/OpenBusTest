#pragma once

#include "BusSimulation.h"

#include <array>
#include <filesystem>
#include <memory>
#include <vector>

struct GLFWwindow;
struct BusModel;

class Renderer {
  public:
    Renderer(int width, int height, const char* title, BusVehicle vehicle = BusVehicle::ManDl05);
    ~Renderer();

    Renderer(const Renderer&) = delete;
    Renderer& operator=(const Renderer&) = delete;

    bool shouldClose() const;
    void beginFrame();
    void draw(const BusSimulation& simulation);
    void endFrame();
    void captureViews(const BusSimulation& simulation, const std::filesystem::path& directory);
    bool consumeCaptureRequest();
    double throttle() const;
    double steering() const;
    double brake() const;
    std::vector<KeyEvent> consumeKeyEvents();

  private:
    static void scrollCallback(GLFWwindow* window, double xOffset, double yOffset);

    GLFWwindow* window_;
    double cameraYaw_ = -2.3;
    double cameraPitch_ = 0.45;
    double viewLookYaw_ = 0.0;
    double viewLookPitch_ = 0.0;
    double lastStatsTitleTime_ = 0.0;
    double cameraDistance_ = 24.0;
    int cameraView_ = 1;
    double previousCursorX_ = 0.0;
    double previousCursorY_ = 0.0;
    bool draggingCamera_ = false;
    std::array<bool, 4> previousKeyStates_ = {};
    std::array<bool, 10> previousViewKeyStates_ = {};
    bool previousCaptureKeyState_ = false;
    bool captureRequested_ = false;
    bool captureMode_ = false;
    std::vector<KeyEvent> keyEvents_;
    std::unique_ptr<BusModel> busModel_;
};
