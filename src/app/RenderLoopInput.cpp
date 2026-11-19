// Per-frame input polling, camera control and driver input accessors.

#include "app/RenderLoopInternal.h"
#include "app/ImGuiLayer.h"
#include "app/SteamPresence.h"
#include "render/RendererFramePresenter.h"

#include <imgui.h>

#include <cstdint>

void RenderLoop::scrollCallback(GLFWwindow* window, double, double yOffset) {
    auto* renderer = static_cast<RenderLoop*>(glfwGetWindowUserPointer(window));
    if (renderer == nullptr) {
        return;
    }
    if (renderer->gameMouseInputCaptured_) {
        return;
    }
    if (renderer->freeCameraEnabled_) {
        renderer->freeCameraSpeed_ =
            std::clamp(renderer->freeCameraSpeed_ + yOffset * 2.0, 1.0, 100.0);
        return;
    }
    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS) {
        renderer->fieldOfViewOffset_ =
            std::clamp(renderer->fieldOfViewOffset_ - yOffset * 2.0, -40.0, 60.0);
        return;
    }
    renderer->cameraDistance_ = std::clamp(renderer->cameraDistance_ - yOffset * 2.0, 6.0, 80.0);
}

bool RenderLoop::isExteriorView() const {
    if (freeCameraEnabled_ && !captureMode_ && !renderingReflection_) {
        return true;
    }
    if (cameraView_ == 0) {
        return true;
    }
    const VehicleCamera* camera = currentVehicleCamera();
    return camera != nullptr && camera->kind == VehicleCameraKind::Passenger &&
           camera->orbitDistance > 1.0;
}

void RenderLoop::setFreeCameraEnabled(bool enabled) {
    if (freeCameraEnabled_ == enabled) {
        return;
    }
    if (enabled && lastMainCameraPoseValid_) {
        freeCameraPose_ = lastMainCameraPose_;
    }
    freeCameraEnabled_ = enabled;
    draggingCamera_ = false;
    draggingFov_ = false;
    pendingMouseClick_ = false;
    mouseInteractionPending_ = false;
    clickableHoverCacheValid_ = false;
    clickableHoverCacheHit_ = false;
    if (enabled) {
        const auto releaseMouse = [this](const std::string& event) {
            if (playerVehicle_ != nullptr && playerVehicle_->scripts) {
                playerVehicle_->scripts->invokeMouseRelease(event);
            }
        };
        mouseInteraction_.cancel(releaseMouse);
        glfwSetCursor(window_, nullptr);
    }
    gameLog.Log(std::string("Free camera ") + (enabled ? "enabled" : "disabled"));
}

const VehicleCamera* RenderLoop::currentVehicleCamera() const {
    if (cameraView_ <= 0 || static_cast<std::size_t>(cameraView_) > vehicleCameras_.size()) {
        return nullptr;
    }
    return &vehicleCameras_[static_cast<std::size_t>(cameraView_ - 1)];
}

void RenderLoop::selectVehicleCamera(int direction) {
    if (vehicleCameras_.empty() || direction == 0) {
        return;
    }
    const int cameraCount = static_cast<int>(vehicleCameras_.size());
    int next = cameraView_ == 0 ? (direction > 0 ? 0 : cameraCount - 1) : cameraView_ - 1;
    if (cameraView_ != 0) {
        next = (next + direction + cameraCount) % cameraCount;
    }
    cameraView_ = next + 1;
    viewLookYaw_ = 0.0;
    viewLookPitch_ = 0.0;
    gameLog.Log("Changed vehicle camera to " + std::to_string(next));
}

bool RenderLoop::selectDriverCameraView(int driverViewIndex) {
    const auto cameraIndex = vehicleCameraArrayIndexForDriverView(vehicleCameras_, driverViewIndex);
    if (!cameraIndex) {
        return false;
    }
    cameraView_ = static_cast<int>(*cameraIndex + 1);
    viewLookYaw_ = 0.0;
    viewLookPitch_ = 0.0;
    gameLog.Log("Selected configured driver view " + std::to_string(driverViewIndex));
    return true;
}

double RenderLoop::currentFieldOfView() const {
    const VehicleCamera* camera = currentVehicleCamera();
    const double baseFieldOfView = camera != nullptr && camera->fieldOfView > 0.0
                                       ? camera->fieldOfView
                                       : DEFAULT_FIELD_OF_VIEW;
    return std::clamp(baseFieldOfView + fieldOfViewOffset_, MIN_FIELD_OF_VIEW, MAX_FIELD_OF_VIEW);
}

void RenderLoop::beginFrame(double fixedTimeStep) {
    TraceScope trace("frame", "RenderLoop::beginFrame");
    if (activeMetarStations_.refreshIfDue() && activeMetarStations_.failed())
        gameLog.Error(activeMetarStations_.status());
    const bool directoryWasPending = activeMetarStations_.pending();
    if (auto codes = activeMetarStations_.poll()) {
        auto updatedStations = openbus::matchMetarStations(weatherStations_, *codes);
        openbus::remapMetarStationSelection(weatherStationPicker_,
                                            recentWeatherStationsAvailable_ ? recentWeatherStations_
                                                                            : weatherStations_,
                                            updatedStations);
        recentWeatherStations_ = std::move(updatedStations);
        recentWeatherStationsAvailable_ = true;
        gameLog.Log(activeMetarStations_.status());
    } else if (directoryWasPending && !activeMetarStations_.pending() &&
               activeMetarStations_.failed()) {
        gameLog.Error(activeMetarStations_.status());
    }
    if (weatherSelection_.refreshIfDue()) {
        weatherUiMessage_ = weatherSelection_.status();
        weatherUiError_ = weatherSelection_.failed();
        if (weatherUiError_)
            gameLog.Error(weatherUiMessage_);
        else
            gameLog.Log(weatherUiMessage_);
    }
    const bool weatherWasPending = weatherSelection_.pending();
    const auto metarWeather = weatherSelection_.poll();
    if (metarWeather) {
        SetWeather(*metarWeather);
        weatherUiMessage_ = weatherSelection_.status();
        weatherUiError_ = false;
        gameLog.Log(weatherUiMessage_);
    } else if (weatherWasPending && !weatherSelection_.pending() && weatherSelection_.failed()) {
        weatherUiMessage_ = weatherSelection_.status();
        weatherUiError_ = true;
        gameLog.Error(weatherUiMessage_);
    }
    steamPresence_.runCallbacks();
    {
        TraceScope phase("frame", "RenderLoop::beginFrame.pollEvents");
        glfwPollEvents();
    }
    if (imguiEnabled_) {
        const bool menuOwnsMouseCursor = playerVehicle_ == nullptr || paused_ || pauseMenuOpen_ ||
                                         mainMenuGraphicsOpen_ || displayPreviewPending_ ||
                                         debugMenuOpen_ || openPlayerTimetablePopup_;
        beginUiFrame(menuOwnsMouseCursor);
    }
    const bool uiWantsKeyboard = imguiEnabled_ && ImGui::GetIO().WantCaptureKeyboard;
    const bool uiWantsText = imguiEnabled_ && ImGui::GetIO().WantTextInput;
    const bool keyboardAvailable =
        !imguiEnabled_ || !uiWantsKeyboard || pauseMenuOpen_ || debugMenuOpen_;
    const bool gameKeyboardAvailable = !imguiEnabled_ || (!uiWantsKeyboard && !uiWantsText);
    const bool gameKeyboardCaptured = !gameKeyboardAvailable;
    gameMouseInputCaptured_ = imguiEnabled_ && ImGui::GetIO().WantCaptureMouse;
    if (gameKeyboardCaptured && !gameKeyboardInputCaptured_ && playerVehicle_ != nullptr &&
        playerVehicle_->scripts) {
        const auto& bindings = vehicleKeyBindings();
        for (std::size_t index = 0;
             index < bindings.size() && index < dispatchedVehicleKeyStates_.size(); ++index) {
            if (!dispatchedVehicleKeyStates_[index]) {
                continue;
            }
            const VehicleKeyBinding& binding = bindings[index];
            if (binding.action != "mouse_control_toggle" &&
                binding.action != "debug_dump_variables" && binding.key != GLFW_KEY_F2 &&
                binding.key != GLFW_KEY_F3 && binding.key != GLFW_KEY_F11 &&
                binding.key != GLFW_KEY_P) {
                playerVehicle_->scripts->invokeKeyBinding(binding.action, false, false);
            }
            dispatchedVehicleKeyStates_[index] = false;
        }
    }
    gameKeyboardInputCaptured_ = gameKeyboardCaptured;
    if (gameMouseInputCaptured_) {
        const auto releaseMouse = [this](const std::string& event) {
            if (playerVehicle_ != nullptr && playerVehicle_->scripts) {
                playerVehicle_->scripts->invokeMouseRelease(event);
            }
        };
        mouseInteraction_.cancel(releaseMouse);
        pendingMouseClick_ = false;
        mouseInteractionPending_ = false;
        mouseThrottle_ = 0.0;
        mouseSteering_ = 0.0;
        mouseBrake_ = 0.0;
    }
    const bool pauseMenuTogglePressed = glfwGetKey(window_, GLFW_KEY_ESCAPE) == GLFW_PRESS;
    if (fixedTimeStep < 0.0 && keyboardAvailable && pauseMenuTogglePressed &&
        !previousPauseMenuToggleState_) {
        if (debugMenuOpen_) {
            debugMenuOpen_ = false;
        } else if (pauseMenuOpen_) {
            pauseMenuOpen_ = false;
            setPausedState(false);
        } else {
            pauseMenuOpen_ = true;
            setPausedState(true);
        }
    }
    previousPauseMenuToggleState_ = pauseMenuTogglePressed;

    const bool pauseTogglePressed = glfwGetKey(window_, GLFW_KEY_P) == GLFW_PRESS;
    if (fixedTimeStep < 0.0 && keyboardAvailable && gameKeyboardAvailable && pauseTogglePressed &&
        !previousPauseToggleState_) {
        const bool resume = paused_;
        setPausedState(!paused_);
        if (resume) {
            pauseMenuOpen_ = false;
        }
    }
    previousPauseToggleState_ = pauseTogglePressed;

    const double currentTime = glfwGetTime();
    frameTiming_.sampleFrameStart(currentTime);
    const double wallTimegap =
        hasPreviousVariableTime_ ? std::max(0.0, currentTime - previousVariableTime_) : 0.0;
    const double requestedTimegap =
        fixedTimeStep >= 0.0 && std::isfinite(fixedTimeStep) ? fixedTimeStep : wallTimegap;
    const double timegap = paused_ ? 0.0 : requestedTimegap;
    const double cameraTimegap =
        fixedTimeStep >= 0.0 && std::isfinite(fixedTimeStep) ? fixedTimeStep : wallTimegap;
    previousVariableTime_ = currentTime;
    hasPreviousVariableTime_ = true;
    frameTimeStep_ = timegap;
    mouseInteractionPending_ = !paused_ && !gameMouseInputCaptured_;
    int width = 1;
    int height = 1;
    double cursorX = 0.0;
    double cursorY = 0.0;
    {
        TraceScope phase("frame", "RenderLoop::beginFrame.windowAndVariables");
        glfwGetFramebufferSize(window_, &width, &height);
        framePresenter_->resize(static_cast<std::uint32_t>(std::max(width, 0)),
                                static_cast<std::uint32_t>(std::max(height, 0)));
        framebufferWidth_ = std::max(width, 1);
        framebufferHeight_ = std::max(height, 1);
        glfwGetCursorPos(window_, &cursorX, &cursorY);
        simulationState_.sharedVariables().setSoundAvailable(soundEngine_.isAvailable());
        simulationState_.sharedVariables().updateFrame(timegap, currentTime, cursorX, cursorY);
        const auto environment =
            openbus::scripting::Vehicle::captureEnvironment(simulationState_.sharedVariables());
        for (const std::unique_ptr<Vehicle>& vehicle : vehicles_) {
            if (!vehicle->mapTrafficRouteFinished) {
                vehicle->variables.updateEnvironment(environment);
                vehicle->updateFrameVariables(vehicle.get() != playerVehicle_, timegap);
            }
        }
        soundViewpoint_ = isExteriorView() ? RenderViewContext::PlayerExterior
                                           : RenderViewContext::PlayerInterior;
    }
    {
        TraceScope phase("frame", "RenderLoop::beginFrame.keyboardInput");
        const auto& bindings = vehicleKeyBindings();
        if (previousVehicleKeyStates_.size() != bindings.size() ||
            dispatchedVehicleKeyStates_.size() != bindings.size()) {
            previousVehicleKeyStates_.assign(bindings.size(), false);
            dispatchedVehicleKeyStates_.assign(bindings.size(), false);
        }
        for (std::size_t index = 0; index < bindings.size(); ++index) {
            const VehicleKeyBinding& binding = bindings[index];
            const bool pressed = vehicleBindingPressed(window_, binding);
            const bool changed = pressed != previousVehicleKeyStates_[index];
            const bool mouseControlToggle = binding.action == "mouse_control_toggle";
            const bool dumpVariables = binding.action == "debug_dump_variables";
            const bool freeCameraToggle = binding.key == GLFW_KEY_F3;
            const bool trafficSplineDebugToggle = binding.key == GLFW_KEY_F2;
            if (changed && gameKeyboardAvailable) {
                if (mouseControlToggle && pressed && playerVehicle_ != nullptr) {
                    mouseControlEnabled_ = !mouseControlEnabled_;
                    if (mouseControlEnabled_) {
                        pendingMouseClick_ = false;
                        clickableHoverCacheValid_ = false;
                        clickableHoverCacheHit_ = false;
                    }
                    gameLog.Log(std::string("Mouse bus control ") +
                                (mouseControlEnabled_ ? "enabled" : "disabled"));
                } else if (dumpVariables && pressed) {
                    logDiagnosticVariables();
                }
                if (!mouseControlToggle) {
                    gameLog.Debug(std::string("Key binding ") + binding.action +
                                  (pressed ? " pressed" : " released"));
                }
            }
            if (changed && gameKeyboardAvailable && !mouseControlToggle && !dumpVariables &&
                !freeCameraToggle && binding.key != GLFW_KEY_P && !trafficSplineDebugToggle &&
                !paused_ && playerVehicle_ != nullptr && playerVehicle_->scripts &&
                openbus::input::shouldDispatchKeyBinding(binding.flags, pressed,
                                                         previousVehicleKeyStates_[index])) {
                playerVehicle_->scripts->invokeKeyBinding(binding.action, pressed, changed);
                dispatchedVehicleKeyStates_[index] = pressed;
            }
            previousVehicleKeyStates_[index] = pressed;
        }
        const bool freeCameraTogglePressed = glfwGetKey(window_, GLFW_KEY_F3) == GLFW_PRESS;
        if (gameKeyboardAvailable && freeCameraTogglePressed && !previousFreeCameraToggleState_ &&
            playerVehicle_ != nullptr) {
            setFreeCameraEnabled(!freeCameraEnabled_);
        }
        previousFreeCameraToggleState_ = freeCameraTogglePressed;
        const bool profileTogglePressed = glfwGetKey(window_, GLFW_KEY_F10) == GLFW_PRESS;
        if (gameKeyboardAvailable && profileTogglePressed && !previousProfileToggleState_) {
#if OPENBUS_ENABLE_PERF_TRACE
            const bool wasRecording = openbus::rendering::IsRecording();
            const bool recording = openbus::rendering::SetRecording(!wasRecording);
            gameLog.Log(recording || wasRecording ? LogLevel::Info : LogLevel::Warning,
                        recording ? "Performance recording started (F10 to pause)"
                        : wasRecording
                            ? "Performance recording paused and saved (F10 to resume)"
                            : "Performance recording could not start; check trace output errors");
#else
            gameLog.Warning(
                "Performance recording is unavailable; launch with profile.bat or profile.sh");
#endif
        }
        previousProfileToggleState_ = profileTogglePressed;
    }
    {
        TraceScope phase("frame", "RenderLoop::beginFrame.viewInput");
        const auto& bindings = vehicleKeyBindings();
        for (std::size_t index = 0; index < previousViewKeyStates_.size(); ++index) {
            const int key = GLFW_KEY_0 + static_cast<int>(index);
            const bool keyDown = glfwGetKey(window_, key) == GLFW_PRESS;
            bool vehicleBindingPressedForKey = false;
            for (std::size_t bindingIndex = 0;
                 bindingIndex < bindings.size() && bindingIndex < previousVehicleKeyStates_.size();
                 ++bindingIndex) {
                if (bindings[bindingIndex].key == key && previousVehicleKeyStates_[bindingIndex]) {
                    vehicleBindingPressedForKey = true;
                    break;
                }
            }
            if (gameKeyboardAvailable && !freeCameraEnabled_ && keyDown &&
                !previousViewKeyStates_[index] && !vehicleBindingPressedForKey) {
                if (index == 0 || index <= vehicleCameras_.size()) {
                    cameraView_ = static_cast<int>(index);
                    viewLookYaw_ = 0.0;
                    viewLookPitch_ = 0.0;
                    gameLog.Log("Changed camera view to " + std::to_string(cameraView_));
                }
            }
            previousViewKeyStates_[index] = keyDown;
        }
    }
    {
        TraceScope phase("frame", "RenderLoop::beginFrame.cameraNavigation");
        const bool mouseLookTogglePressed = glfwGetKey(window_, GLFW_KEY_F4) == GLFW_PRESS;
        if (gameKeyboardAvailable && mouseLookTogglePressed && !previousMouseLookToggleState_) {
            mouseLookEnabled_ = !mouseLookEnabled_;
            draggingCamera_ = false;
            previousCursorX_ = cursorX;
            previousCursorY_ = cursorY;
            glfwSetInputMode(window_, GLFW_CURSOR,
                             mouseLookEnabled_ ? GLFW_CURSOR_DISABLED : GLFW_CURSOR_NORMAL);
            if (mouseLookEnabled_) {
                if (playerVehicle_ == nullptr && imguiEnabled_) {
                    ImGui::SetWindowFocus(nullptr);
                }
                if (mouseControlEnabled_) {
                    mouseControlEnabled_ = false;
                    gameLog.Log("Mouse bus control disabled (mouse camera look enabled)");
                }
                pendingMouseClick_ = false;
                clickableHoverCacheValid_ = false;
                clickableHoverCacheHit_ = false;
            }
            previousCameraNavigationStates_.fill(false);
            gameLog.Log(
                std::string("Mouse camera look ") +
                (mouseLookEnabled_ ? "enabled; move the mouse to look around" : "disabled"));
        }
        previousMouseLookToggleState_ = mouseLookTogglePressed;

        const std::array<int, 2> cameraNavigationKeys = {GLFW_KEY_LEFT, GLFW_KEY_RIGHT};
        if (!mouseLookEnabled_) {
            for (std::size_t index = 0; index < cameraNavigationKeys.size(); ++index) {
                const bool pressed = glfwGetKey(window_, cameraNavigationKeys[index]) == GLFW_PRESS;
                if (gameKeyboardAvailable && !freeCameraEnabled_ && pressed &&
                    !previousCameraNavigationStates_[index]) {
                    selectVehicleCamera(index == 0 ? -1 : 1);
                }
                previousCameraNavigationStates_[index] = pressed;
            }
        }
    }
    {
        TraceScope phase("frame", "RenderLoop::beginFrame.freeCameraMovement");
        if (gameKeyboardAvailable && freeCameraEnabled_ && playerVehicle_ != nullptr) {
            const double forward =
                static_cast<double>(glfwGetKey(window_, GLFW_KEY_UP) == GLFW_PRESS) -
                static_cast<double>(glfwGetKey(window_, GLFW_KEY_DOWN) == GLFW_PRESS);
            const double right =
                static_cast<double>(glfwGetKey(window_, GLFW_KEY_RIGHT) == GLFW_PRESS) -
                static_cast<double>(glfwGetKey(window_, GLFW_KEY_LEFT) == GLFW_PRESS);
            const double vertical =
                static_cast<double>(glfwGetKey(window_, GLFW_KEY_PAGE_UP) == GLFW_PRESS) -
                static_cast<double>(glfwGetKey(window_, GLFW_KEY_PAGE_DOWN) == GLFW_PRESS);
            freeCameraPose_ = openbus::rendering::moveFreeCamera(
                freeCameraPose_, forward, right, vertical, cameraTimegap, freeCameraSpeed_);
        }
    }
    {
        TraceScope phase("frame", "RenderLoop::beginFrame.captureInput");
        const bool captureKeyPressed = glfwGetKey(window_, GLFW_KEY_F12) == GLFW_PRESS;
        if (gameKeyboardAvailable && captureKeyPressed && !previousCaptureKeyState_) {
            captureRequested_ = true;
        }
        previousCaptureKeyState_ = captureKeyPressed;
    }
    {
        const bool clickableDebugKeyPressed = glfwGetKey(window_, GLFW_KEY_I) == GLFW_PRESS;
        if (gameKeyboardAvailable && clickableDebugKeyPressed && !previousClickableDebugKeyState_) {
            if (playerVehicle_ != nullptr) {
                const RenderViewContext context = isExteriorView()
                                                      ? RenderViewContext::PlayerExterior
                                                      : RenderViewContext::PlayerInterior;
                playerVehicle_->logClickableDebugInfo(context);
            }
        }
        previousClickableDebugKeyState_ = clickableDebugKeyPressed;
    }
    {
        const bool collisionDebugKeyPressed = glfwGetKey(window_, GLFW_KEY_C) == GLFW_PRESS;
        if (gameKeyboardAvailable && collisionDebugKeyPressed && !previousCollisionDebugKeyState_) {
            collisionDebugOverlay_ = !collisionDebugOverlay_;
            gameLog.Log(std::string("Collision wireframe overlay ") +
                        (collisionDebugOverlay_
                             ? "enabled (cyan = .surf roads/pole colliders, red = pole contact, "
                               "gray = broken pole)"
                             : "disabled"));
        }
        previousCollisionDebugKeyState_ = collisionDebugKeyPressed;
    }
    {
        const bool tileBoundaryDebugKeyPressed = glfwGetKey(window_, GLFW_KEY_F11) == GLFW_PRESS;
        if (gameKeyboardAvailable && tileBoundaryDebugKeyPressed &&
            !previousTileBoundaryDebugKeyState_) {
            tileBoundaryDebugOverlay_ = !tileBoundaryDebugOverlay_;
            gameLog.Log(
                std::string("Active map tile boundary overlay ") +
                (tileBoundaryDebugOverlay_ ? "enabled (green lines follow terrain)" : "disabled"));
        }
        previousTileBoundaryDebugKeyState_ = tileBoundaryDebugKeyPressed;
    }
    {
        const bool trafficSplineDebugKeyPressed = glfwGetKey(window_, GLFW_KEY_F2) == GLFW_PRESS;
        if (gameKeyboardAvailable && trafficSplineDebugKeyPressed &&
            !previousTrafficSplineDebugKeyState_) {
            trafficSplineDebugOverlay_ = !trafficSplineDebugOverlay_;
            gameLog.Log(
                std::string("AI and human traffic spline overlay ") +
                (trafficSplineDebugOverlay_
                     ? "enabled (road color=default priority: green=0/high right-of-way, "
                       "yellow=128, red=255/low right-of-way; labels show spline/path priority and "
                       "CAR/TRK/COM/AMB group overrides; "
                       "magenta=human, white=other vehicle; "
                       "path arrows=travel direction, white arrows=AI facing; AI "
                       "green=clear, yellow=following, purple=junction yield, amber="
                       "crossing reservation, red=signal/obstacle, blue=in junction, "
                       "bright red=deadlocked; x-ray footprints show occupied areas, "
                       "cyan shows 3s route, teal shows junction claim, and lines link "
                       "blockers; red/green bars show signal stops; signalized junction paths "
                       "show exact stoplines and J/P/L/PH phase, S schedule, CP control labels; "
                       "HUD C=clear F=following "
                       "Y=yield R=reservation S=signal B=blocked J=junction L=claim; "
                       "HOT=slowest blocked vehicle with spline, reason, current/target speed)"
                     : "disabled"));
        }
        previousTrafficSplineDebugKeyState_ = trafficSplineDebugKeyPressed;
    }
    {
        const bool raisePlayerKeyPressed = glfwGetKey(window_, GLFW_KEY_U) == GLFW_PRESS;
        if (gameKeyboardAvailable && raisePlayerKeyPressed && !previousRaisePlayerKeyState_) {
            raisePlayerRequestPending_ = true;
        }
        previousRaisePlayerKeyState_ = raisePlayerKeyPressed;
    }
    {
        TraceScope phase("frame", "RenderLoop::beginFrame.mouseControl");
        if (mouseControlEnabled_ && !gameMouseInputCaptured_) {
            int windowWidth = 1;
            int windowHeight = 1;
            glfwGetWindowSize(window_, &windowWidth, &windowHeight);
            const openbus::input::MouseControlInputs inputs = openbus::input::mouseControlInputs(
                cursorX, cursorY, static_cast<double>(windowWidth),
                static_cast<double>(windowHeight));
            mouseThrottle_ = inputs.throttle;
            mouseSteering_ = inputs.steering;
            mouseBrake_ = inputs.brake;
        }
    }
    {
        TraceScope phase("frame", "RenderLoop::beginFrame.mouseInput");
        if (mouseLookEnabled_ && !gameMouseInputCaptured_) {
            const double cursorDeltaX = cursorX - previousCursorX_;
            const double cursorDeltaY = cursorY - previousCursorY_;
            if (freeCameraEnabled_) {
                freeCameraPose_.yawRadians -= cursorDeltaX * 0.005;
                freeCameraPose_.pitchRadians =
                    std::clamp(freeCameraPose_.pitchRadians - cursorDeltaY * 0.005, -1.35, 1.35);
            } else if (isExteriorView()) {
                cameraYaw_ -= cursorDeltaX * 0.005;
                cameraPitch_ = std::clamp(cameraPitch_ - cursorDeltaY * 0.005, -1.35, 1.35);
            } else {
                viewLookYaw_ -= cursorDeltaX * 0.005;
                viewLookPitch_ = std::clamp(viewLookPitch_ - cursorDeltaY * 0.005, -1.35, 1.35);
            }
        }
        const bool rightMouse = !gameMouseInputCaptured_ &&
                                glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
        if (rightMouse && !draggingFov_) {
            previousFovCursorY_ = cursorY;
        } else if (rightMouse) {
            const double cursorDeltaY = cursorY - previousFovCursorY_;
            if (freeCameraEnabled_) {
                fieldOfViewOffset_ =
                    std::clamp(fieldOfViewOffset_ + cursorDeltaY * 0.15, -40.0, 60.0);
            } else if (cameraView_ == 0) {
                cameraDistance_ = std::clamp(cameraDistance_ + cursorDeltaY * 0.1, 0.1, 80.0);
            } else {
                fieldOfViewOffset_ =
                    std::clamp(fieldOfViewOffset_ + cursorDeltaY * 0.15, -40.0, 60.0);
            }
        }
        draggingFov_ = rightMouse;
        previousFovCursorY_ = cursorY;
        const bool middleMouse =
            !gameMouseInputCaptured_ &&
            glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
        if (!mouseLookEnabled_ && middleMouse && !draggingCamera_) {
            previousCursorX_ = cursorX;
            previousCursorY_ = cursorY;
        } else if (!mouseLookEnabled_ && middleMouse) {
            const double cursorDeltaX = cursorX - previousCursorX_;
            const double cursorDeltaY = cursorY - previousCursorY_;
            if (freeCameraEnabled_) {
                freeCameraPose_.yawRadians -= cursorDeltaX * 0.005;
                freeCameraPose_.pitchRadians =
                    std::clamp(freeCameraPose_.pitchRadians - cursorDeltaY * 0.005, -1.35, 1.35);
            } else if (isExteriorView()) {
                cameraYaw_ -= cursorDeltaX * 0.005;
                cameraPitch_ -= cursorDeltaY * 0.005;
                cameraPitch_ = std::clamp(cameraPitch_, -1.35, 1.35);
            } else {
                viewLookYaw_ -= cursorDeltaX * 0.005;
                viewLookPitch_ -= cursorDeltaY * 0.005;
                viewLookPitch_ = std::clamp(viewLookPitch_, -1.35, 1.35);
            }
        }
        draggingCamera_ = middleMouse;
        previousCursorX_ = cursorX;
        previousCursorY_ = cursorY;
        const bool leftMouse = !gameMouseInputCaptured_ &&
                               glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
        leftMousePressed_ = leftMouse;
        if (!mouseControlEnabled_ && leftMouse && !previousLeftMouseState_ && !rightMouse &&
            !middleMouse) {
            pendingMouseClick_ = true;
            pendingMouseClickX_ = cursorX;
            pendingMouseClickY_ = cursorY;
        }
        previousLeftMouseState_ = leftMouse;
    }
}

void RenderLoop::setPausedState(bool paused) {
    if (paused_ == paused) {
        return;
    }
    if (paused) {
        const auto releaseMouse = [this](const std::string& event) {
            if (playerVehicle_ != nullptr && playerVehicle_->scripts) {
                playerVehicle_->scripts->invokeMouseRelease(event);
            }
        };
        mouseInteraction_.cancel(releaseMouse);
        if (playerVehicle_ != nullptr && playerVehicle_->scripts) {
            const auto& bindings = vehicleKeyBindings();
            for (std::size_t index = 0;
                 index < bindings.size() && index < dispatchedVehicleKeyStates_.size(); ++index) {
                if (!dispatchedVehicleKeyStates_[index]) {
                    continue;
                }
                const VehicleKeyBinding& binding = bindings[index];
                if (binding.action != "mouse_control_toggle" &&
                    binding.action != "debug_dump_variables" && binding.key != GLFW_KEY_F2 &&
                    binding.key != GLFW_KEY_F3 && binding.key != GLFW_KEY_F11 &&
                    binding.key != GLFW_KEY_P) {
                    playerVehicle_->scripts->invokeKeyBinding(binding.action, false, false);
                }
                dispatchedVehicleKeyStates_[index] = false;
            }
        }
        pendingMouseClick_ = false;
        clickableHoverCacheValid_ = false;
        clickableHoverCacheHit_ = false;
        glfwSetInputMode(window_, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
        glfwSetCursor(window_, nullptr);
    } else if (playerVehicle_ != nullptr && mouseLookEnabled_) {
        glfwSetInputMode(window_, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
        glfwGetCursorPos(window_, &previousCursorX_, &previousCursorY_);
    }
    paused_ = paused;
    simulationState_.sharedVariables().setPaused(paused_);
    gameLog.Log(paused_ ? "Simulation paused" : "Simulation resumed");
}

double RenderLoop::throttle() const {
    if (playerVehicle_ == nullptr) {
        return 0.0;
    }
    if (mouseControlEnabled_) {
        return gameMouseInputCaptured_ ? 0.0 : mouseThrottle_;
    }
    if (gameKeyboardInputCaptured_) {
        return 0.0;
    }
    const VehicleKeyBinding* binding = findVehicleKeyBinding("throttle");
    return binding != nullptr && vehicleBindingPressed(window_, *binding) ? 1.0 : 0.0;
}

double RenderLoop::steering() const {
    if (playerVehicle_ == nullptr) {
        return 0.0;
    }
    if (mouseControlEnabled_) {
        return gameMouseInputCaptured_ ? 0.0 : mouseSteering_;
    }
    if (gameKeyboardInputCaptured_) {
        return 0.0;
    }
    const VehicleKeyBinding* leftBinding = findVehicleKeyBinding("steering_left");
    const VehicleKeyBinding* rightBinding = findVehicleKeyBinding("steering_right");
    const bool left = leftBinding != nullptr && vehicleBindingPressed(window_, *leftBinding);
    const bool right = rightBinding != nullptr && vehicleBindingPressed(window_, *rightBinding);
    return static_cast<double>(right) - static_cast<double>(left);
}

double RenderLoop::brake() const {
    if (playerVehicle_ == nullptr) {
        return 0.0;
    }
    if (mouseControlEnabled_) {
        return gameMouseInputCaptured_ ? 0.0 : mouseBrake_;
    }
    if (gameKeyboardInputCaptured_) {
        return 0.0;
    }
    const VehicleKeyBinding* binding = findVehicleKeyBinding("brake");
    return binding != nullptr && vehicleBindingPressed(window_, *binding) ? 1.0 : 0.0;
}

double RenderLoop::physicsThrottle() const {
    return playerVehicle_ == nullptr ? throttle() : playerVehicle_->variables.get("throttle");
}

double RenderLoop::physicsWheelTorque() const {
    return playerVehicle_ == nullptr ? 0.0 : playerVehicle_->variables.get("m_wheel");
}

double RenderLoop::physicsSteering() const {
    return playerVehicle_ == nullptr ? smoothedSteering_
                                     : playerVehicle_->variables.get("steering");
}

std::vector<double> RenderLoop::physicsWheelBrakeForces(std::size_t axleCount) const {
    if (playerVehicle_ == nullptr) {
        return std::vector<double>(axleCount * 2, 0.0);
    }
    return physicsWheelBrakeForces(*playerVehicle_, axleCount);
}

std::vector<double> RenderLoop::physicsWheelBrakeForces(const Vehicle& vehicle,
                                                        std::size_t axleCount) const {
    std::vector<double> forces(axleCount * 2, 0.0);
    for (std::size_t axle = 0; axle < axleCount; ++axle) {
        const std::size_t variableAxle = axle + vehicle.odeWheelIndexOffset / 2;
        const std::string prefix = "axle_brakeforce_" + std::to_string(variableAxle) + "_";
        forces[axle * 2] = vehicle.variables.get(prefix + "l");
        forces[axle * 2 + 1] = vehicle.variables.get(prefix + "r");
    }
    return forces;
}

std::vector<double> RenderLoop::physicsAxleSpringFactors(std::size_t axleCount) const {
    if (playerVehicle_ == nullptr) {
        return std::vector<double>(axleCount * 2, 1.0);
    }
    return physicsAxleSpringFactors(*playerVehicle_, axleCount);
}

std::vector<double> RenderLoop::physicsAxleSpringFactors(const Vehicle& vehicle,
                                                         std::size_t axleCount) const {
    std::vector<double> factors(axleCount * 2, 1.0);
    for (std::size_t axle = 0; axle < axleCount; ++axle) {
        const std::size_t variableAxle = axle + vehicle.odeWheelIndexOffset / 2;
        const std::string prefix = "axle_springfactor_" + std::to_string(variableAxle) + "_";
        const std::string leftName = prefix + "l";
        const std::string rightName = prefix + "r";
        if (vehicle.variables.has(leftName)) {
            factors[axle * 2] = vehicle.variables.get(leftName);
        }
        if (vehicle.variables.has(rightName)) {
            factors[axle * 2 + 1] = vehicle.variables.get(rightName);
        }
    }
    return factors;
}
