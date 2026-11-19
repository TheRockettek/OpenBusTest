#include "app/RenderLoopInternal.h"

#include "app/DiligentFrameRenderer.h"
#include "render/RendererFramePresenter.h"

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

std::optional<openbus::map::MapTrafficPose>
RenderLoop::passengerQueuePose(int stopId, std::size_t queueSlot) const {
    if (mapRenderer_ == nullptr) {
        return std::nullopt;
    }
    const auto authored = authoredPassengerPositions_.find(stopId);
    if (authored != authoredPassengerPositions_.end() && !authored->second.empty()) {
        return authored->second[queueSlot % authored->second.size()];
    }
    const auto pedestrian = passengerPedestrianQueuePositions_.find(stopId);
    if (pedestrian != passengerPedestrianQueuePositions_.end() && !pedestrian->second.empty()) {
        return pedestrian->second[queueSlot % pedestrian->second.size()];
    }
    return std::nullopt;
}

std::optional<openbus::map::MapTrafficPose>
RenderLoop::passengerQueuePose(int stopId, const openbus::app::WaitingPassenger& passenger) const {
    const auto held = passengerHeldQueuePositions_.find({stopId, passenger.modelIndex});
    if (held != passengerHeldQueuePositions_.end()) {
        return held->second;
    }
    return passengerQueuePose(stopId, passenger.queueSlot);
}

void RenderLoop::draw(const BusSimulation& simulation, const World& world) {
    static_cast<void>(world);
    if (framePresenter_ == nullptr || dynamic_cast<openbus::rendering::DiligentFrameRenderer*>(
                                          framePresenter_.get()) == nullptr) {
        throw std::logic_error("The production scene requires the Diligent frame presenter");
    }
    prepareDiligentSceneAssets(simulation);
    drawDiligentScene(simulation);
}

void RenderLoop::endFrame() {
    if (framePresenter_ == nullptr || dynamic_cast<openbus::rendering::DiligentFrameRenderer*>(
                                          framePresenter_.get()) == nullptr) {
        throw std::logic_error("Frame pacing requires the Diligent frame presenter");
    }
    if (const auto deadline = frameTiming_.deadlineAfterFrame(std::chrono::steady_clock::now())) {
        std::this_thread::sleep_until(*deadline);
    }
}

void RenderLoop::captureViews(const BusSimulation& simulation, const World& world,
                              const std::filesystem::path& directory) {
    if (framePresenter_ == nullptr) {
        throw std::logic_error("Screenshot capture requires an initialized renderer");
    }
    auto* diligentRenderer =
        dynamic_cast<openbus::rendering::DiligentFrameRenderer*>(framePresenter_.get());
    if (diligentRenderer == nullptr) {
        throw std::logic_error("Screenshot capture requires the Diligent frame presenter");
    }

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
    const bool previousCaptureMode = captureMode_;
    const auto restoreCaptureState = [&] {
        cameraView_ = previousCameraView;
        cameraYaw_ = previousCameraYaw;
        cameraPitch_ = previousCameraPitch;
        cameraDistance_ = previousCameraDistance;
        viewLookYaw_ = previousViewLookYaw;
        viewLookPitch_ = previousViewLookPitch;
        captureMode_ = previousCaptureMode;
    };

    struct CaptureView {
        const char* name;
        int cameraView;
        double yaw;
        double pitch;
        double distance;
    };
    const int driverCameraView =
        reflectionReferenceCamera_ >= 0 ? reflectionReferenceCamera_ + 1 : 1;
    std::vector<CaptureView> views = {{"dashboard", driverCameraView, 0.0, 0.0, 0.0},
                                      {"three-quarter", 0, 0.55, 0.45, 11.0},
                                      {"front", 0, 0.0, 0.45, 10.5},
                                      {"left", 0, 1.5707963267948966, 0.45, 10.5},
                                      {"right", 0, -1.5707963267948966, 0.45, 10.5}};
    if (const auto panelZoomCamera = vehicleCameraArrayIndexForDriverView(vehicleCameras_, 7)) {
        views.push_back({"driver-view-7", static_cast<int>(*panelZoomCamera) + 1, 0.0, 0.0, 0.0});
    }

    captureMode_ = true;
    try {
        for (const CaptureView& view : views) {
            cameraView_ = view.cameraView;
            cameraYaw_ = view.yaw;
            cameraPitch_ = view.pitch;
            cameraDistance_ = view.distance;
            const std::filesystem::path path = directory / (std::string(view.name) + ".png");
            invalidateTemporalHistory();
            diligentRenderer->captureNextFrame(path);
            draw(simulation, world);
            if (diligentRenderer->hasPendingCapture()) {
                throw std::runtime_error(
                    "Diligent screenshot frame was suspended before composition");
            }
            gameLog.Log("Saved screenshot: " + path.string());
        }
    } catch (...) {
        diligentRenderer->cancelPendingCapture();
        restoreCaptureState();
        throw;
    }
    restoreCaptureState();
}
