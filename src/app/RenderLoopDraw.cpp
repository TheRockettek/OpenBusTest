// Frame rendering: reflections, scene draw, HUD overlay and capture.

#include "app/RenderLoopInternal.h"

void RenderLoop::renderReflectionViews(const BusSimulation& simulation) {
    if (renderingReflection_ || reflectionRenderer_->empty()) {
        return;
    }
    const int previousCameraView = cameraView_;
    const double previousFovOffset = fieldOfViewOffset_;
    const double previousLookYaw = viewLookYaw_;
    const double previousLookPitch = viewLookPitch_;
    bool reflectionTrianglesReset = false;
    renderingReflection_ = true;
    openbus::rendering::setFrameTriangleCountReflectionPass(true);
    reflectionRenderer_->render(
        simulation, vehicleCameras_, viewport_,
        [this](std::size_t reflectionIndex) {
            openbus::rendering::ReflectionRequirement result;
            for (const std::unique_ptr<Vehicle>& vehicle : vehicles_) {
                if (vehicle->needsReflectionTexture(reflectionIndex)) {
                    result.needed = true;
                    result.size =
                        std::max(result.size, vehicle->requiredReflectionSize(reflectionIndex));
                }
            }
            return result;
        },
        [this, &reflectionTrianglesReset](const BusSimulation& reflectionSimulation,
                                          std::size_t cameraIndex, int width, int height) {
            if (!reflectionTrianglesReset) {
                openbus::rendering::resetFrameReflectionTriangleCount();
                reflectionTrianglesReset = true;
            }
            framebufferWidth_ = std::max(width, 1);
            framebufferHeight_ = std::max(height, 1);
            cameraView_ = static_cast<int>(cameraIndex + 1);
            fieldOfViewOffset_ = 0.0;
            viewLookYaw_ = 0.0;
            viewLookPitch_ = 0.0;
            setPerspective(static_cast<double>(width), static_cast<double>(height),
                           currentFieldOfView());
            draw(reflectionSimulation);
        },
        [this, previousCameraView, previousFovOffset, previousLookYaw,
         previousLookPitch](int width, int height) {
            framebufferWidth_ = std::max(width, 1);
            framebufferHeight_ = std::max(height, 1);
            cameraView_ = previousCameraView;
            fieldOfViewOffset_ = previousFovOffset;
            viewLookYaw_ = previousLookYaw;
            viewLookPitch_ = previousLookPitch;
            setPerspective(static_cast<double>(width), static_cast<double>(height),
                           currentFieldOfView());
        });
    openbus::rendering::setFrameTriangleCountReflectionPass(false);
    renderingReflection_ = false;
}

void RenderLoop::draw(const BusSimulation& simulation) {
    TraceScope trace("frame", "RenderLoop::draw");
    if (!renderingReflection_) {
        openbus::rendering::resetFrameTriangleCount();
    }
    const BodyPose chassis = simulation.chassisPose();
    const ChassisCollisionBox collision = simulation.chassisCollisionBox();
    constexpr double radiansToDegrees = 180.0 / 3.14159265358979323846;
    const Matrix4 playerModelBase = multiplyMatrix4(
        poseMatrix(chassis),
        translationMatrix(
            {0.0, 0.0, playerVehicle_ != nullptr ? playerVehicle_->modelOffsetZ : 0.0}));
    double playerGroundHeight = [&]() {
        TraceScope phase("render", "RenderLoop::draw.groundHeight");
        return mapRenderer_ != nullptr
                   ? mapRenderer_->terrainHeightAt(chassis.position[0], chassis.position[1])
                         .value_or(playerVehicle_ != nullptr ? playerVehicle_->placement.position[2]
                                                             : 0.0)
                   : (playerVehicle_ != nullptr ? playerVehicle_->placement.position[2] : 0.0);
    }();
    double wheelGroundHeightSum = 0.0;
    std::size_t validWheelGroundHeights = 0;
    for (std::size_t wheelIndex = 0; wheelIndex < simulation.wheelCount(); ++wheelIndex) {
        const double wheelBottom =
            simulation.wheelPose(wheelIndex).position[2] - simulation.wheelRadius(wheelIndex);
        if (std::isfinite(wheelBottom)) {
            wheelGroundHeightSum += wheelBottom;
            ++validWheelGroundHeights;
        }
    }
    if (validWheelGroundHeights > 0) {
        // Wheel bottoms follow the actual contact surface, including elevated
        // roads and ramps that can differ from the underlying terrain grid.
        playerGroundHeight = wheelGroundHeightSum / static_cast<double>(validWheelGroundHeights);
    }
    const Matrix4 shadowGroundBase =
        multiplyMatrix4(translationMatrix({chassis.position[0], chassis.position[1],
                                           playerGroundHeight + GROUND_SHADOW_OFFSET_Z}),
                        rotationMatrix(simulation.yaw() * radiansToDegrees, 0.0, 0.0, 1.0));
    Matrix4 inversePlayerModelBase = identityMatrix();
    const bool canGroundClampShadow = invertAffineMatrix(playerModelBase, inversePlayerModelBase);
    const Matrix4 groundShadowTransform =
        canGroundClampShadow ? multiplyMatrix4(inversePlayerModelBase, shadowGroundBase)
                             : identityMatrix();
    {
        TraceScope phase("render", "RenderLoop::draw.camera");
        if (freeCameraEnabled_ && !renderingReflection_ && !captureMode_) {
            const std::array<double, 3> forward = openbus::rendering::freeCameraForward(
                freeCameraPose_.yawRadians, freeCameraPose_.pitchRadians);
            lookAt(freeCameraPose_.position[0], freeCameraPose_.position[1],
                   freeCameraPose_.position[2], freeCameraPose_.position[0] + forward[0],
                   freeCameraPose_.position[1] + forward[1],
                   freeCameraPose_.position[2] + forward[2]);
        } else if (cameraView_ == 0) {
            const std::array<double, 3> target =
                transformLocalPoint(chassis, simulation.outsideCameraCenter());
            const double targetX = target[0];
            const double targetY = target[1];
            const double targetZ = target[2];
            const double horizontalDistance = cameraDistance_ * std::cos(cameraPitch_);
            const double orbitYaw = simulation.yaw() + cameraYaw_;
            const double eyeX = targetX + horizontalDistance * std::cos(orbitYaw);
            const double eyeY = targetY + horizontalDistance * std::sin(orbitYaw);
            const double eyeZ = targetZ + cameraDistance_ * std::sin(cameraPitch_);
            lookAt(eyeX, eyeY, eyeZ, targetX, targetY, targetZ);
        } else if (const VehicleCamera* camera = currentVehicleCamera(); camera != nullptr) {
            constexpr double DEGREES_TO_RADIANS = 3.141592653589793 / 180.0;
            const bool reflectionCamera = camera->kind == VehicleCameraKind::Reflexion ||
                                          camera->kind == VehicleCameraKind::Reflexion2;
            const double panSign = reflectionCamera ? 1.0 : -1.0;
            double pan = panSign * camera->pan * DEGREES_TO_RADIANS + viewLookYaw_;
            double tilt = camera->tilt * DEGREES_TO_RADIANS + viewLookPitch_;
            const double modelOffsetZ =
                playerVehicle_ != nullptr ? playerVehicle_->modelOffsetZ : 0.0;
            const std::array<double, 3> centerLocal = {camera->position[1], -camera->position[0],
                                                       camera->position[2] + modelOffsetZ};
            if (reflectionCamera && camera->followsViewer && renderingReflection_ &&
                reflectionViewerEyeLocalValid_ && reflectionReferenceCamera_ >= 0 &&
                static_cast<std::size_t>(reflectionReferenceCamera_) < vehicleCameras_.size()) {
                const VehicleCamera& reference =
                    vehicleCameras_[static_cast<std::size_t>(reflectionReferenceCamera_)];
                const double referencePan = -reference.pan * DEGREES_TO_RADIANS;
                const double referenceTilt = reference.tilt * DEGREES_TO_RADIANS;
                const std::array<double, 3> referenceEye = {
                    reference.position[1] -
                        std::cos(referenceTilt) * std::cos(referencePan) * reference.orbitDistance,
                    -reference.position[0] -
                        std::cos(referenceTilt) * std::sin(referencePan) * reference.orbitDistance,
                    reference.position[2] + modelOffsetZ -
                        std::sin(referenceTilt) * reference.orbitDistance};
                const std::array<double, 3> reflected =
                    openbus::rendering::mirrorReflectionDirection(
                        centerLocal,
                        {std::cos(tilt) * std::cos(pan), std::cos(tilt) * std::sin(pan),
                         std::sin(tilt)},
                        referenceEye, reflectionViewerEyeLocal_);
                pan = std::atan2(reflected[1], reflected[0]);
                tilt = std::asin(std::clamp(reflected[2], -1.0, 1.0));
            }
            const std::array<double, 3> direction = {
                std::cos(tilt) * std::cos(pan), std::cos(tilt) * std::sin(pan), std::sin(tilt)};
            const std::array<double, 3> upLocal = {-std::sin(tilt) * std::cos(pan),
                                                   -std::sin(tilt) * std::sin(pan), std::cos(tilt)};
            const std::array<double, 3> eyeLocal = {
                centerLocal[0] - direction[0] * camera->orbitDistance,
                centerLocal[1] - direction[1] * camera->orbitDistance,
                centerLocal[2] - direction[2] * camera->orbitDistance};
            const std::array<double, 3> targetLocal = {eyeLocal[0] + direction[0] * 3.0,
                                                       eyeLocal[1] + direction[1] * 3.0,
                                                       eyeLocal[2] + direction[2] * 3.0};
            const std::array<double, 3> eye = transformLocalPoint(chassis, eyeLocal);
            const std::array<double, 3> target = transformLocalPoint(chassis, targetLocal);
            const std::array<double, 3> up = {
                chassis.rotation[0] * upLocal[0] + chassis.rotation[1] * upLocal[1] +
                    chassis.rotation[2] * upLocal[2],
                chassis.rotation[3] * upLocal[0] + chassis.rotation[4] * upLocal[1] +
                    chassis.rotation[5] * upLocal[2],
                chassis.rotation[6] * upLocal[0] + chassis.rotation[7] * upLocal[1] +
                    chassis.rotation[8] * upLocal[2]};
            lookAt(eye[0], eye[1], eye[2], target[0], target[1], target[2], up[0], up[1], up[2]);
        } else {
            std::array<double, 3> eyeLocal = {5.65, 0.70, 0.70};
            std::array<double, 3> targetLocal = {8.65, 0.70, 0.72};
            switch (cameraView_) {
            case 2:
                eyeLocal = {4.25, -0.55, 0.78};
                targetLocal = {7.25, -0.55, 0.80};
                break;
            case 3:
                eyeLocal = {-2.75, 0.0, 0.72};
                targetLocal = {0.25, 0.0, 0.74};
                break;
            case 4:
                eyeLocal = {5.35, 1.05, 0.86};
                targetLocal = {5.35, 4.0, 0.82};
                break;
            case 5:
                eyeLocal = {5.35, -1.05, 0.86};
                targetLocal = {5.35, -4.0, 0.82};
                break;
            case 6:
                eyeLocal = {6.85, 0.0, 0.80};
                targetLocal = {3.85, 0.0, 0.80};
                break;
            case 7:
                eyeLocal = {-6.15, 0.0, 0.80};
                targetLocal = {-3.15, 0.0, 0.80};
                break;
            case 8:
                eyeLocal = {0.0, 1.05, 1.30};
                targetLocal = {0.0, 4.0, 1.30};
                break;
            case 9:
                eyeLocal = {0.0, -1.05, 1.30};
                targetLocal = {0.0, -4.0, 1.30};
                break;
            default:
                break;
            }
            const std::array<double, 3> eye = transformLocalPoint(chassis, eyeLocal);
            const double directionX = targetLocal[0] - eyeLocal[0];
            const double directionY = targetLocal[1] - eyeLocal[1];
            const double directionZ = targetLocal[2] - eyeLocal[2];
            const double horizontalLength = std::hypot(directionX, directionY);
            const double distance = std::sqrt(directionX * directionX + directionY * directionY +
                                              directionZ * directionZ);
            const double baseYaw = std::atan2(directionY, directionX);
            const double basePitch = std::atan2(directionZ, horizontalLength);
            const double lookYaw = baseYaw + viewLookYaw_;
            const double lookPitch = basePitch + viewLookPitch_;
            const std::array<double, 3> target = transformLocalPoint(
                chassis, {eyeLocal[0] + distance * std::cos(lookPitch) * std::cos(lookYaw),
                          eyeLocal[1] + distance * std::cos(lookPitch) * std::sin(lookYaw),
                          eyeLocal[2] + distance * std::sin(lookPitch)});
            lookAt(eye[0], eye[1], eye[2], target[0], target[1], target[2]);
        }
        if (!renderingReflection_) {
            const auto& view = modelViewMatrix();
            const std::array<double, 3> eyeWorld = {
                -(view[0] * view[12] + view[1] * view[13] + view[2] * view[14]),
                -(view[4] * view[12] + view[5] * view[13] + view[6] * view[14]),
                -(view[8] * view[12] + view[9] * view[13] + view[10] * view[14])};
            const std::array<double, 3> forwardWorld = {-view[2], -view[6], -view[10]};
            const std::array<double, 3> upWorld = {view[1], view[5], view[9]};
            if (!captureMode_) {
                lastMainCameraPose_.position = eyeWorld;
                lastMainCameraPose_.yawRadians = std::atan2(forwardWorld[1], forwardWorld[0]);
                lastMainCameraPose_.pitchRadians =
                    std::atan2(forwardWorld[2], std::hypot(forwardWorld[0], forwardWorld[1]));
                lastMainCameraPoseValid_ = true;
            }
            const auto toLocalPoint = [&chassis](const std::array<double, 3>& world) {
                const double x = world[0] - chassis.position[0];
                const double y = world[1] - chassis.position[1];
                const double z = world[2] - chassis.position[2];
                return std::array<double, 3>{
                    chassis.rotation[0] * x + chassis.rotation[3] * y + chassis.rotation[6] * z,
                    chassis.rotation[1] * x + chassis.rotation[4] * y + chassis.rotation[7] * z,
                    chassis.rotation[2] * x + chassis.rotation[5] * y + chassis.rotation[8] * z};
            };
            const auto toLocalDirection = [&chassis](const std::array<double, 3>& world) {
                return std::array<double, 3>{
                    chassis.rotation[0] * world[0] + chassis.rotation[3] * world[1] +
                        chassis.rotation[6] * world[2],
                    chassis.rotation[1] * world[0] + chassis.rotation[4] * world[1] +
                        chassis.rotation[7] * world[2],
                    chassis.rotation[2] * world[0] + chassis.rotation[5] * world[1] +
                        chassis.rotation[8] * world[2]};
            };
            soundEngine_.setListenerPose(toLocalPoint(eyeWorld), toLocalDirection(forwardWorld),
                                         toLocalDirection(upWorld));
            reflectionViewerEyeLocal_ = toLocalPoint(eyeWorld);
            reflectionViewerEyeLocalValid_ = true;
        }
    }
    {
        TraceScope phase("render", "RenderLoop::draw.visibility");
        if (!renderingReflection_) {
            const RenderViewContext context = isExteriorView() ? RenderViewContext::PlayerExterior
                                                               : RenderViewContext::PlayerInterior;
            if (playerVehicle_ != nullptr) {
                playerVehicle_->setOdeSimulation(simulation);
            }
            for (const std::unique_ptr<Vehicle>& vehicle : vehicles_) {
                pushMatrix();
                if (vehicle.get() == playerVehicle_) {
                    applyPose(chassis);
                    vehicle->prepareFrameVisibility(context, framebufferWidth_, framebufferHeight_);
                } else {
                    applyVehiclePlacement(
                        makeModelRootPlacement(vehicle->placement, vehicle->modelOffsetZ));
                    vehicle->prepareFrameVisibility(RenderViewContext::NonPlayer, framebufferWidth_,
                                                    framebufferHeight_);
                }
                popMatrix();
            }
        }
    }
    if (!renderingReflection_ && !freeCameraEnabled_ && !mouseLookEnabled_ &&
        playerVehicle_ != nullptr && mouseInteractionPending_) {
        TraceScope phase("input", "RenderLoop::draw.interaction");
        // Reflections, captures and repeated draws are not additional input frames.
        mouseInteractionPending_ = false;
        double cursorX = 0.0;
        double cursorY = 0.0;
        glfwGetCursorPos(window_, &cursorX, &cursorY);
        int windowWidth = 1;
        int windowHeight = 1;
        glfwGetWindowSize(window_, &windowWidth, &windowHeight);
        const double framebufferScaleX =
            static_cast<double>(framebufferWidth_) / static_cast<double>(std::max(windowWidth, 1));
        const double framebufferScaleY = static_cast<double>(framebufferHeight_) /
                                         static_cast<double>(std::max(windowHeight, 1));
        const double framebufferCursorX = cursorX * framebufferScaleX;
        const double framebufferCursorY = cursorY * framebufferScaleY;
        pushMatrix();
        applyPose(chassis);
        const RenderViewContext interactionContext = isExteriorView()
                                                         ? RenderViewContext::PlayerExterior
                                                         : RenderViewContext::PlayerInterior;
        if (!mouseControlEnabled_ && benchmarkClickDiscoveryPending_) {
            TraceScope trace("benchmark", "Benchmark.discoverClickable");
            benchmarkClickDiscoveryPending_ = false;
            benchmarkClickDiscoveryComplete_ = true;
            if (playerVehicle_->hasVisibleClickable(interactionContext)) {
                constexpr int columns = 40;
                constexpr int rows = 24;
                for (int row = 1; row < rows && !benchmarkClickTargetFound_; ++row) {
                    const double y = static_cast<double>(framebufferHeight_) * row / rows;
                    for (int column = 1; column < columns; ++column) {
                        const double x = static_cast<double>(framebufferWidth_) * column / columns;
                        if (playerVehicle_
                                ->mouseEventAt(x, y, framebufferWidth_, framebufferHeight_,
                                               interactionContext)
                                .empty()) {
                            continue;
                        }
                        benchmarkClickFramebufferX_ = x;
                        benchmarkClickFramebufferY_ = y;
                        benchmarkClickTargetFound_ = true;
                        break;
                    }
                }
            }
        }
        if (!mouseControlEnabled_ && benchmarkClickQueued_ && benchmarkClickTargetFound_) {
            pendingMouseClick_ = true;
            pendingMouseClickX_ = benchmarkClickFramebufferX_ / framebufferScaleX;
            pendingMouseClickY_ = benchmarkClickFramebufferY_ / framebufferScaleY;
        }
        benchmarkClickQueued_ = false;
        if (!mouseControlEnabled_) {
            const int interactionContextValue = static_cast<int>(interactionContext);
            const std::uint64_t clickableRevision = playerVehicle_->clickableRevision();
            // Pose changes every simulation step; throttle stationary-pointer hover checks.
            const double currentTime = glfwGetTime();
            const bool hoverCacheMatches =
                clickableHoverCacheValid_ && clickableHoverCacheX_ == framebufferCursorX &&
                clickableHoverCacheY_ == framebufferCursorY &&
                clickableHoverCacheWidth_ == framebufferWidth_ &&
                clickableHoverCacheHeight_ == framebufferHeight_ &&
                clickableHoverCacheContext_ == interactionContextValue &&
                clickableHoverCacheCameraView_ == cameraView_ &&
                clickableHoverCacheCameraYaw_ == cameraYaw_ &&
                clickableHoverCacheCameraPitch_ == cameraPitch_ &&
                clickableHoverCacheLookYaw_ == viewLookYaw_ &&
                clickableHoverCacheLookPitch_ == viewLookPitch_ &&
                clickableHoverCacheRevision_ == clickableRevision &&
                currentTime - clickableHoverCacheTimestamp_ < clickableHoverRefreshInterval_;
            if (!hoverCacheMatches) {
                clickableHoverCacheValid_ = true;
                clickableHoverCacheX_ = framebufferCursorX;
                clickableHoverCacheY_ = framebufferCursorY;
                clickableHoverCacheWidth_ = framebufferWidth_;
                clickableHoverCacheHeight_ = framebufferHeight_;
                clickableHoverCacheContext_ = interactionContextValue;
                clickableHoverCacheCameraView_ = cameraView_;
                clickableHoverCacheCameraYaw_ = cameraYaw_;
                clickableHoverCacheCameraPitch_ = cameraPitch_;
                clickableHoverCacheLookYaw_ = viewLookYaw_;
                clickableHoverCacheLookPitch_ = viewLookPitch_;
                clickableHoverCacheRevision_ = clickableRevision;
                clickableHoverCacheTimestamp_ = currentTime;
                clickableHoverCacheHit_ =
                    playerVehicle_->hasVisibleClickable(interactionContext) &&
                    playerVehicle_->hasClickableAt(framebufferCursorX, framebufferCursorY,
                                                   framebufferWidth_, framebufferHeight_,
                                                   interactionContext);
            }
        } else {
            pendingMouseClick_ = false;
            clickableHoverCacheValid_ = false;
            clickableHoverCacheHit_ = false;
        }
        const bool hoveringClickable = !mouseControlEnabled_ && clickableHoverCacheHit_;
        GLFWcursor* cursor = mouseControlEnabled_ ? mouseSteeringCursor_
                             : hoveringClickable  ? clickableCursor_
                                                  : nullptr;
        glfwSetCursor(window_, cursor);
        const auto releaseMouse = [this](const std::string& event) {
            if (playerVehicle_->scripts) {
                playerVehicle_->scripts->invokeMouseRelease(event);
            }
        };
        if (mouseControlEnabled_) {
            pendingMouseClick_ = false;
            mouseInteraction_.cancel(releaseMouse);
        } else if (pendingMouseClick_) {
            std::string event = playerVehicle_->mouseEventAt(
                pendingMouseClickX_ * framebufferScaleX, pendingMouseClickY_ * framebufferScaleY,
                framebufferWidth_, framebufferHeight_, interactionContext);
            if (clickableDebugOverlay_) {
                gameLog.Log("Mouse press picked event=" +
                            (event.empty() ? std::string("none") : event));
            }
            mouseDragDiagnosticLogged_ = false;
            mouseInteraction_.press(
                std::move(event), pendingMouseClickX_, pendingMouseClickY_,
                [this](const std::string& pressedEvent) {
                    playerVehicle_->handleMouseClick(pressedEvent);
                },
                releaseMouse);
            pendingMouseClick_ = false;
        } else {
            mouseInteraction_.update(
                true, leftMousePressed_, cursorX, cursorY,
                [this](const std::string& event, double deltaX, double deltaY, double x, double y) {
                    if (playerVehicle_->scripts) {
                        if (clickableDebugOverlay_ && !mouseDragDiagnosticLogged_ &&
                            (deltaX != 0.0 || deltaY != 0.0)) {
                            std::string normalizedEvent = event;
                            std::transform(normalizedEvent.begin(), normalizedEvent.end(),
                                           normalizedEvent.begin(), [](unsigned char character) {
                                               return std::isalnum(character)
                                                          ? static_cast<char>(
                                                                std::tolower(character))
                                                          : '_';
                                           });
                            const std::string functionName = "trigger_" + normalizedEvent + "_drag";
                            const bool dragHandlerFound =
                                playerVehicle_->scripts->hasScriptEntryPoint(functionName);
                            mouseDragDiagnosticLogged_ = true;
                            playerVehicle_->scripts->invokeMouseDrag(event, deltaX, deltaY, x, y);
                            std::ostringstream description;
                            description
                                << "Mouse drag event=" << event << " dx=" << deltaX
                                << " dy=" << deltaY
                                << " drag_handler=" << (dragHandlerFound ? "found" : "missing");
                            if (normalizedEvent == "cp_rollo") {
                                description << " cp_rollo_pos="
                                            << playerVehicle_->variables.get("cp_rollo_pos");
                            } else if (normalizedEvent == "cp_rollof") {
                                description << " cp_rollof_pos="
                                            << playerVehicle_->variables.get("cp_rollof_pos");
                            }
                            gameLog.Log(description.str());
                            return;
                        }
                        playerVehicle_->scripts->invokeMouseDrag(event, deltaX, deltaY, x, y);
                    }
                },
                releaseMouse);
        }
        popMatrix();
    }
    {
        TraceScope phase("render", "RenderLoop::draw.reflections");
        renderReflectionViews(simulation);
    }
    {
        TraceScope phase("render", "RenderLoop::draw.ground");
        if (mapRenderer_ != nullptr) {
            glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
            glEnable(GL_DEPTH_TEST);
            glDepthFunc(GL_LESS);
            glDepthMask(GL_TRUE);
            glDisable(GL_BLEND);
            glDisable(GL_CULL_FACE);
            glDisable(GL_POLYGON_OFFSET_FILL);
            mapRenderer_->draw();
        } else if (!captureMode_) {
            glEnable(GL_DEPTH_TEST);
            glDepthFunc(GL_LESS);
            glDepthMask(GL_TRUE);
            glDisable(GL_BLEND);
            glDisable(GL_CULL_FACE);
            glDisable(GL_POLYGON_OFFSET_FILL);
            drawGround(simulation.roadBumps());
        }
    }
    const RenderViewContext context = (renderingReflection_ || isExteriorView())
                                          ? RenderViewContext::PlayerExterior
                                          : RenderViewContext::PlayerInterior;
    {
        TraceScope phase("render", "RenderLoop::draw.model");
        bool playerDrawn = false;
        const auto drawVehicles = [&](VehicleRenderPass renderPass, bool playerVehicles) {
            for (const std::unique_ptr<Vehicle>& vehicle : vehicles_) {
                if (!vehicle->loaded || vehicle->displayLists.empty() ||
                    (vehicle.get() == playerVehicle_) != playerVehicles) {
                    continue;
                }
                pushMatrix();
                const RenderViewContext vehicleContext =
                    vehicle.get() == playerVehicle_ ? context : RenderViewContext::NonPlayer;
                VehiclePlacement placedVehicle = vehicle->placement;
                if (vehicle.get() == playerVehicle_) {
                    applyPose(chassis);
                    if (renderPass == VehicleRenderPass::Opaque) {
                        playerDrawn = true;
                    }
                } else {
                    const std::optional<double> terrainHeight =
                        mapRenderer_ != nullptr
                            ? mapRenderer_->terrainHeightAt(placedVehicle.position[0],
                                                            placedVehicle.position[1])
                            : std::nullopt;
                    if (terrainHeight.has_value()) {
                        placedVehicle.position[2] = *terrainHeight;
                    }
                    applyVehiclePlacement(
                        makeModelRootPlacement(placedVehicle, vehicle->modelOffsetZ));
                }
                Matrix4 aiGroundShadowTransform = {};
                const Matrix4* shadowTransform = nullptr;
                if (vehicle.get() == playerVehicle_) {
                    if (canGroundClampShadow) {
                        shadowTransform = &groundShadowTransform;
                    }
                } else {
                    const VehiclePlacement rootPlacement =
                        makeModelRootPlacement(placedVehicle, vehicle->modelOffsetZ);
                    const double fallbackGroundHeight = placedVehicle.position[2];
                    const double shadowGroundHeight =
                        mapRenderer_ != nullptr ? mapRenderer_
                                                      ->terrainHeightAt(placedVehicle.position[0],
                                                                        placedVehicle.position[1])
                                                      .value_or(fallbackGroundHeight)
                                                : fallbackGroundHeight;
                    aiGroundShadowTransform =
                        makeGroundShadowTransform(rootPlacement, vehicle->modelOffsetZ,
                                                  shadowGroundHeight + GROUND_SHADOW_OFFSET_Z);
                    shadowTransform = &aiGroundShadowTransform;
                }
                vehicle->draw(vehicleContext, renderPass, shadowTransform);
                popMatrix();
            }
        };
        const auto drawTypeFourScenery = [&]() {
            if (mapRenderer_ == nullptr || renderingReflection_) {
                return;
            }
            glEnable(GL_DEPTH_TEST);
            glDepthFunc(GL_LESS);
            glDepthMask(GL_TRUE);
            glDisable(GL_BLEND);
            mapRenderer_->draw(true);
        };
        drawVehicles(VehicleRenderPass::Opaque, false);
        drawVehicles(VehicleRenderPass::Transparent, false);
        if (context == RenderViewContext::PlayerInterior) {
            drawTypeFourScenery();
        }
        drawVehicles(VehicleRenderPass::Opaque, true);
        drawVehicles(VehicleRenderPass::Transparent, true);
        if (context == RenderViewContext::PlayerExterior) {
            drawTypeFourScenery();
        }
        if (!playerDrawn && collision.enabled && !collision.mesh) {
            pushMatrix();
            applyPose(chassis);
            translate(collision.offsetX, collision.offsetY, collision.offsetZ);
            drawBox(collision.length, collision.width, collision.height, 0.85, 0.70, 0.08);
            popMatrix();
        }
        if (!renderingReflection_ && !mouseControlEnabled_ && clickableDebugOverlay_ &&
            playerVehicle_ != nullptr) {
            pushMatrix();
            applyPose(chassis);
            playerVehicle_->drawClickableDebug(context);
            popMatrix();
        }
    }
    if (!renderingReflection_ && collisionDebugOverlay_) {
        TraceScope phase("render", "RenderLoop::draw.collisionDebug");
        const std::uint64_t collisionRevision = simulation.collisionDebugRevision();
        if (!collisionWireframeBuilt_ || collisionWireframeRevision_ != collisionRevision) {
            TraceScope phase("render", "RenderLoop::draw.collisionDebug.build");
            collisionWireframeVertices_.clear();
            const std::vector<StaticCollisionMesh> meshes = simulation.collisionDebugMeshes();
            std::size_t triangleCount = 0;
            for (const StaticCollisionMesh& mesh : meshes) {
                if (mesh.vertices.size() % 3 != 0 || mesh.indices.size() % 3 != 0) {
                    continue;
                }
                triangleCount += mesh.indices.size() / 3;
                for (std::size_t index = 0; index + 2 < mesh.indices.size(); index += 3) {
                    const std::array<int, 3> triangle = {
                        mesh.indices[index], mesh.indices[index + 1], mesh.indices[index + 2]};
                    for (int edge = 0; edge < 3; ++edge) {
                        for (const int vertexIndex :
                             {triangle[static_cast<std::size_t>(edge)],
                              triangle[static_cast<std::size_t>((edge + 1) % 3)]}) {
                            if (vertexIndex < 0 || static_cast<std::size_t>(vertexIndex) * 3 + 2 >=
                                                       mesh.vertices.size()) {
                                continue;
                            }
                            const std::size_t vertexOffset =
                                static_cast<std::size_t>(vertexIndex) * 3;
                            collisionWireframeVertices_.push_back(
                                {static_cast<float>(mesh.vertices[vertexOffset]),
                                 static_cast<float>(mesh.vertices[vertexOffset + 1]),
                                 static_cast<float>(mesh.vertices[vertexOffset + 2] + 0.025), 1.0F,
                                 0.58F, 0.08F});
                        }
                    }
                }
            }
            collisionWireframeBuilt_ = true;
            collisionWireframeRevision_ = collisionRevision;
            gameLog.Log("Collision wireframe cached: " + std::to_string(triangleCount) +
                        " triangles across " + std::to_string(meshes.size()) +
                        " active terrain/road/scenery meshes");
        }

        GLboolean depthWasEnabled = GL_FALSE;
        GLboolean depthWriteWasEnabled = GL_TRUE;
        {
            TraceScope phase("render", "RenderLoop::draw.collisionDebug.queryState");
            depthWasEnabled = glIsEnabled(GL_DEPTH_TEST);
            glGetBooleanv(GL_DEPTH_WRITEMASK, &depthWriteWasEnabled);
        }
        glEnable(GL_DEPTH_TEST);
        glDepthMask(GL_FALSE);
        openbus::rendering::drawStaticPrimitives(collisionWireframeBuffer_,
                                                 collisionWireframeVertices_, GL_LINES, 1.5F,
                                                 collisionRevision);
        if (collision.enabled && !collision.mesh) {
            pushMatrix();
            applyPose(chassis);
            translate(collision.offsetX, collision.offsetY, collision.offsetZ);
            openbus::rendering::drawBox(collision.length, collision.width, collision.height, 1.0,
                                        0.15, 0.12);
            popMatrix();
        }
        glDepthMask(depthWriteWasEnabled);
        if (!depthWasEnabled) {
            glDisable(GL_DEPTH_TEST);
        }
    }
    if (!renderingReflection_ && statusHudTexture_ != 0) {
        TraceScope phase("render", "RenderLoop::draw.statusHud");
        {
            TraceScope phase("render", "RenderLoop::draw.statusHud.update");
            const openbus::rendering::MapRenderStats mapStats =
                mapRenderer_ != nullptr ? mapRenderer_->renderStats()
                                        : openbus::rendering::MapRenderStats{};
            std::ostringstream statusText;
            statusText << "X " << std::fixed << std::setprecision(1) << simulation.positionX()
                       << " Y " << simulation.positionY() << " Z " << simulation.positionZ() << '\n'
                       << "CHUNKS " << mapStats.activeTiles << '/' << mapStats.totalTiles
                       << " TERRAIN " << mapStats.activeTerrainTiles << '/'
                       << mapStats.totalTerrainTiles << " OBJECTS " << mapStats.activeSceneryObjects
                       << '/' << mapStats.totalSceneryObjects << '\n'
                       << "ROADS " << mapStats.activeRoadBatches << '/' << mapStats.totalRoadBatches
                       << " COLLISION TILES " << simulation.loadedMapCollisionTileCount()
                       << " VEHICLES " << vehicles_.size();
            const std::string currentStatus = statusText.str();
            if (currentStatus != statusHudText_) {
                const std::vector<std::uint8_t> pixels = [&]() {
                    TraceScope phase("render", "RenderLoop::draw.statusHud.rasterize");
                    return makeStatusHudPixels(currentStatus);
                }();
                TraceScope phase("render", "RenderLoop::draw.statusHud.upload");
                pglActiveTexture(GL_TEXTURE0);
                glBindTexture(GL_TEXTURE_2D, statusHudTexture_);
                glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, STATUS_HUD_TEXTURE_WIDTH,
                                STATUS_HUD_TEXTURE_HEIGHT, GL_RGBA, GL_UNSIGNED_BYTE,
                                pixels.data());
                openbus::rendering::invalidateTextureBindings();
                statusHudText_ = currentStatus;
            }
        }

        {
            TraceScope phase("render", "RenderLoop::draw.statusHud.submit");
            glDisable(GL_DEPTH_TEST);
            glDisable(GL_CULL_FACE);
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            const float scale = std::min(
                {1.5F,
                 static_cast<float>(std::max(framebufferWidth_ - 24, 1)) / STATUS_HUD_TEXTURE_WIDTH,
                 static_cast<float>(std::max(framebufferHeight_ - 24, 1)) /
                     STATUS_HUD_TEXTURE_HEIGHT});
            const float hudWidth = STATUS_HUD_TEXTURE_WIDTH * scale;
            const float hudHeight = STATUS_HUD_TEXTURE_HEIGHT * scale;
            const float leftPixels = 12.0F;
            const float bottomPixels = framebufferHeight_ - 12.0F - hudHeight;
            openbus::rendering::drawTextureQuad(
                statusHudTexture_, -1.0F + 2.0F * leftPixels / framebufferWidth_,
                -1.0F + 2.0F * bottomPixels / framebufferHeight_,
                2.0F * hudWidth / framebufferWidth_, 2.0F * hudHeight / framebufferHeight_);
            // The final overlay restores the scene pass's depth/blend state.
            // Face culling stays disabled; scene shaders handle sidedness.
            glEnable(GL_DEPTH_TEST);
            glDisable(GL_BLEND);
        }
    }
    {
        TraceScope phase("render", "RenderLoop::draw.windowTitle");
        const double titleUpdateTime = glfwGetTime();
        if (!renderingReflection_ && !captureMode_) {
            if (!fpsSampleInitialized_) {
                fpsSampleStartTime_ = titleUpdateTime;
                fpsSampleFrameCount_ = 0;
                fpsSampleInitialized_ = true;
            }
            ++fpsSampleFrameCount_;
            const double sampleDuration = titleUpdateTime - fpsSampleStartTime_;
            if (sampleDuration >= 1.0) {
                displayedFps_ = static_cast<double>(fpsSampleFrameCount_) / sampleDuration;
                fpsSampleFrameCount_ = 0;
                fpsSampleStartTime_ = titleUpdateTime;
            }
        }
        if (!renderingReflection_ && playerVehicle_ &&
            titleUpdateTime - lastStatsTitleTime_ > 0.25) {
            const double speedMetresPerSecond = simulation.speed();
            const long speedMph = std::lround(speedMetresPerSecond * 2.2369362921);
            const long speedKmh = std::lround(speedMetresPerSecond * 3.6);
            const long throttlePercent = std::lround(
                std::clamp(playerVehicle_->variables.get("throttle"), 0.0F, 1.0F) * 100.0F);
            const long brakePercent = std::lround(
                std::clamp(playerVehicle_->variables.get("brake"), 0.0F, 1.0F) * 100.0F);
            std::ostringstream title;
            title << "OpenBus - " << speedMph << " mph / " << speedKmh << " km/h - Throttle "
                  << throttlePercent << "% - Brake " << brakePercent << "% - "
                  << openbus::rendering::frameTriangleCount() << " main + "
                  << openbus::rendering::frameReflectionTriangleCount()
                  << " reflection triangles/refresh - " << std::fixed << std::setprecision(1)
                  << displayedFps_ << " FPS";
            {
                TraceScope phase("render", "RenderLoop::draw.windowTitle.set");
                glfwSetWindowTitle(window_, title.str().c_str());
            }
            lastStatsTitleTime_ = titleUpdateTime;
        }
    }
}

void RenderLoop::endFrame() {
    TraceScope trace("frame", "RenderLoop::endFrame.swapBuffers");
    glfwSwapBuffers(window_);
}

void RenderLoop::captureViews(const BusSimulation& simulation,
                              const std::filesystem::path& directory) {
    TraceScope trace("capture", "RenderLoop::captureViews");
    std::filesystem::create_directories(directory);
    int width = 1;
    int height = 1;
    glfwGetFramebufferSize(window_, &width, &height);
    framebufferWidth_ = std::max(width, 1);
    framebufferHeight_ = std::max(height, 1);

    const int previousCameraView = cameraView_;
    const double previousCameraYaw = cameraYaw_;
    const double previousCameraPitch = cameraPitch_;
    const double previousCameraDistance = cameraDistance_;
    const double previousViewLookYaw = viewLookYaw_;
    const double previousViewLookPitch = viewLookPitch_;
    captureMode_ = true;
    cameraView_ = 0;
    cameraDistance_ = 24.0;
    viewLookYaw_ = 0.0;
    viewLookPitch_ = 0.0;

    struct CaptureView {
        const char* name;
        int cameraView;
        double yaw;
        double pitch;
        double distance;
    };
    const std::array<CaptureView, 5> views = {{{"dashboard", 1, 0.0, 0.0, 0.0},
                                               {"three-quarter", 0, 0.55, 0.45, 11.0},
                                               {"front", 0, 0.0, 0.45, 10.5},
                                               {"left", 0, 1.5707963267948966, 0.45, 10.5},
                                               {"right", 0, -1.5707963267948966, 0.45, 10.5}}};
    for (const CaptureView& view : views) {
        TraceScope viewTrace("capture", "RenderLoop::captureViews.view");
        cameraView_ = view.cameraView;
        cameraYaw_ = view.yaw;
        cameraPitch_ = view.pitch;
        cameraDistance_ = view.distance;
        {
            TraceScope phase("capture", "RenderLoop::captureViews.render");
            viewport_ = {0, 0, width, height};
            glViewport(viewport_[0], viewport_[1], viewport_[2], viewport_[3]);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            setPerspective(static_cast<double>(width), static_cast<double>(height), 60.0);
            draw(simulation);
            glFinish();
        }
        const std::filesystem::path path = directory / (std::string(view.name) + ".png");
        {
            TraceScope phase("capture", "RenderLoop::captureViews.save");
            if (!openbus::rendering::saveFramebufferPng(path, width, height)) {
                gameLog.Log("Failed to save screenshot: " + path.string());
            } else {
                gameLog.Log("Saved screenshot: " + path.string());
            }
        }
    }

    cameraView_ = previousCameraView;
    cameraYaw_ = previousCameraYaw;
    cameraPitch_ = previousCameraPitch;
    cameraDistance_ = previousCameraDistance;
    viewLookYaw_ = previousViewLookYaw;
    viewLookPitch_ = previousViewLookPitch;
    captureMode_ = false;
}
