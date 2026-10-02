#include "RendererReflection.h"

#include "BusSimulation.h"
#include "CoreRenderer.h"
#include "Logger.h"
#include "OpenGLFunctions.h"
#include "PerfTrace.h"
#include "Variables.h"

#ifndef GLFW_INCLUDE_NONE
#define GLFW_INCLUDE_NONE
#endif
#include <GLFW/glfw3.h>
#include <GL/gl.h>

#include <algorithm>
#include <cmath>
#include <cctype>
#include <cstdlib>
#include <cstdint>
#include <filesystem>
#include <string_view>
#include <unordered_map>

#ifndef GL_FRAMEBUFFER
#define GL_FRAMEBUFFER 0x8D40
#define GL_COLOR_ATTACHMENT0 0x8CE0
#define GL_DEPTH_ATTACHMENT 0x8D00
#define GL_FRAMEBUFFER_COMPLETE 0x8CD5
#define GL_DEPTH_COMPONENT24 0x81A6
#endif

#ifndef GL_RGBA8
#define GL_RGBA8 0x8058
#endif

extern Logger gameLog;

namespace openbus::rendering {
namespace {

struct ReflectionTarget {
    unsigned int framebuffer = 0;
    unsigned int texture = 0;
    unsigned int depthTexture = 0;
    int width = 0;
    int height = 0;
};

std::unordered_map<int, unsigned int> activeReflectionTextures;
bool activeReflectionPass = false;

int reflectionSizeFromEnvironment() {
    const char* value = std::getenv("OPENBUS_REFLECTION_SIZE");
    if (value == nullptr) {
        return 1024;
    }
    const int size = std::atoi(value);
    return size == 256 || size == 512 || size == 1024 ? size : 256;
}

int reflectionIntervalFromEnvironment() {
    const char* value = std::getenv("OPENBUS_REFLECTION_INTERVAL");
    return value == nullptr ? 1 : std::max(1, std::atoi(value));
}

} // namespace

struct ReflectionRenderer::Impl {
    int maximumSize;
    int frameInterval;
    std::uint64_t frameCounter = 0;
    std::vector<ReflectionTarget> targets;

    Impl(int maximumSizeValue, int frameIntervalValue)
        : maximumSize(std::max(maximumSizeValue, kMinReflectionTargetSize)),
          frameInterval(std::max(frameIntervalValue, 1)) {}

    void resizeTarget(ReflectionTarget& target, int size) {
        size = std::clamp(size, kMinReflectionTargetSize, maximumSize);
        if (target.width == size && target.height == size) {
            return;
        }
        target.width = size;
        target.height = size;

        glBindTexture(GL_TEXTURE_2D, target.texture);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, size, size, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
        glBindTexture(GL_TEXTURE_2D, target.depthTexture);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, size, size, 0, GL_DEPTH_COMPONENT,
                     GL_FLOAT, nullptr);
        glBindTexture(GL_TEXTURE_2D, 0);
    }
};

ReflectionRenderer::ReflectionRenderer()
    : impl_(std::make_unique<Impl>(reflectionSizeFromEnvironment(),
                                   reflectionIntervalFromEnvironment())) {
    gameLog.Log("Reflection targets: " + std::to_string(impl_->maximumSize) + "x" +
                std::to_string(impl_->maximumSize) + ", every " +
                std::to_string(impl_->frameInterval) + " frame(s)");
}

ReflectionRenderer::~ReflectionRenderer() {
    destroy();
}

void ReflectionRenderer::initialize(const std::vector<VehicleCamera>& cameras) {
    TraceScope trace("render", "ReflectionRenderer::initialize");
    std::size_t reflectionCount = 0;
    for (const VehicleCamera& camera : cameras) {
        if (camera.kind == VehicleCameraKind::Reflexion ||
            camera.kind == VehicleCameraKind::Reflexion2) {
            ++reflectionCount;
        }
    }
    if (reflectionCount == 0 || !impl_->targets.empty()) {
        return;
    }

    impl_->targets.resize(reflectionCount);
    for (std::size_t index = 0; index < impl_->targets.size(); ++index) {
        ReflectionTarget& target = impl_->targets[index];
        target.width = impl_->maximumSize;
        target.height = impl_->maximumSize;
        glGenTextures(1, &target.texture);
        glBindTexture(GL_TEXTURE_2D, target.texture);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, impl_->maximumSize, impl_->maximumSize, 0, GL_RGBA,
                     GL_UNSIGNED_BYTE, nullptr);

        glGenTextures(1, &target.depthTexture);
        glBindTexture(GL_TEXTURE_2D, target.depthTexture);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, impl_->maximumSize, impl_->maximumSize,
                     0, GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);

        pglGenFramebuffers(1, &target.framebuffer);
        pglBindFramebuffer(GL_FRAMEBUFFER, target.framebuffer);
        pglFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, target.texture,
                                0);
        pglFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D,
                                target.depthTexture, 0);
        if (pglCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
            gameLog.Log("Reflection framebuffer is incomplete at index " + std::to_string(index));
        }
        activeReflectionTextures[static_cast<int>(index)] = target.texture;
    }
    pglBindFramebuffer(GL_FRAMEBUFFER, 0);
    glBindTexture(GL_TEXTURE_2D, 0);
}

void ReflectionRenderer::destroy() {
    if (impl_ == nullptr) {
        return;
    }
    TraceScope trace("render", "ReflectionRenderer::destroy");
    activeReflectionTextures.clear();
    for (const ReflectionTarget& target : impl_->targets) {
        if (target.texture != 0) {
            glDeleteTextures(1, &target.texture);
        }
        if (target.depthTexture != 0) {
            glDeleteTextures(1, &target.depthTexture);
        }
        if (target.framebuffer != 0) {
            pglDeleteFramebuffers(1, &target.framebuffer);
        }
    }
    impl_->targets.clear();
}

bool ReflectionRenderer::empty() const {
    return impl_ == nullptr || impl_->targets.empty();
}

void ReflectionRenderer::render(const BusSimulation& simulation,
                                const std::vector<VehicleCamera>& cameras,
                                const VisibilityCallback& visibility, const DrawCallback& draw,
                                const RestoreCallback& restore) {
    TraceScope trace("render", "ReflectionRenderer::render");
    if (empty()) {
        return;
    }
    ++impl_->frameCounter;
    if (impl_->frameCounter % static_cast<std::uint64_t>(impl_->frameInterval) != 0) {
        return;
    }

    GLint viewport[4] = {};
    glGetIntegerv(GL_VIEWPORT, viewport);
    const Matrix4 previousModelView = modelViewMatrix();
    activeReflectionPass = true;
    std::size_t reflectionIndex = 0;
    for (std::size_t cameraIndex = 0; cameraIndex < cameras.size(); ++cameraIndex) {
        TraceScope targetTrace("render", "ReflectionRenderer::render.target");
        const VehicleCamera& camera = cameras[cameraIndex];
        if (camera.kind != VehicleCameraKind::Reflexion &&
            camera.kind != VehicleCameraKind::Reflexion2) {
            continue;
        }
        if (reflectionIndex >= impl_->targets.size()) {
            break;
        }
        ReflectionTarget& target = impl_->targets[reflectionIndex];
        const ReflectionRequirement requirement = visibility(reflectionIndex);
        if (!requirement.needed) {
            ++reflectionIndex;
            continue;
        }

        pglBindFramebuffer(GL_FRAMEBUFFER, target.framebuffer);
        const double reflectionFieldOfView =
            camera.fieldOfView > 0.0 ? camera.fieldOfView : kDefaultFieldOfView;
        const double referenceFovScale = std::tan(kDefaultFieldOfView * 3.141592653589793 / 360.0);
        const double reflectionFovScale =
            std::tan(std::clamp(reflectionFieldOfView, kMinFieldOfView, kMaxFieldOfView) *
                     3.141592653589793 / 360.0);
        const int desiredSize = std::max(
            requirement.size, static_cast<int>(std::ceil(requirement.size * referenceFovScale /
                                                         std::max(reflectionFovScale, 0.001))));
        int targetSize = kMinReflectionTargetSize;
        while (targetSize < desiredSize && targetSize < impl_->maximumSize) {
            targetSize *= 2;
        }
        targetSize = std::min(targetSize, impl_->maximumSize);
        if (targetSize < target.width && desiredSize > target.width / 2) {
            targetSize = target.width;
        }
        if (target.width != targetSize || target.height != targetSize) {
            gameLog.Log("Reflection target " + std::to_string(reflectionIndex) + " resized from " +
                        std::to_string(target.width) + "x" + std::to_string(target.height) +
                        " to " + std::to_string(targetSize) + "x" + std::to_string(targetSize) +
                        " (max " + std::to_string(impl_->maximumSize) + "x" +
                        std::to_string(impl_->maximumSize) + ")");
        }
        impl_->resizeTarget(target, targetSize);
        glViewport(0, 0, target.width, target.height);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        draw(simulation, cameraIndex, target.width, target.height);
        ++reflectionIndex;
    }
    activeReflectionPass = false;
    pglBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(viewport[0], viewport[1], viewport[2], viewport[3]);
    restore(viewport[2], viewport[3]);
    setModelViewMatrix(previousModelView);
}

void ReflectionRenderer::renderDebugOverlay(GLFWwindow* window, bool enabled) const {
    TraceScope trace("render", "ReflectionRenderer::renderDebugOverlay");
    if (!enabled || activeReflectionPass || empty()) {
        return;
    }
    int width = 1;
    int height = 1;
    glfwGetFramebufferSize(window, &width, &height);
    const std::size_t columnCount = std::min<std::size_t>(4, impl_->targets.size());
    const std::size_t rowCount = (impl_->targets.size() + columnCount - 1) / columnCount;
    const float gapPixels = 6.0f;
    constexpr float maxTilePixels = 220.0f;
    const float tilePixels = std::min(
        {maxTilePixels, (static_cast<float>(width) - gapPixels * (columnCount + 1)) / columnCount,
         (static_cast<float>(height) - gapPixels * (rowCount + 1)) / rowCount});
    if (tilePixels <= 0.0f) {
        return;
    }
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    for (std::size_t index = 0; index < impl_->targets.size(); ++index) {
        const std::size_t column = index % columnCount;
        const std::size_t row = index / columnCount;
        const float leftPixels = gapPixels + column * (tilePixels + gapPixels);
        const float topPixels = gapPixels + row * (tilePixels + gapPixels);
        const float left = -1.0f + 2.0f * leftPixels / static_cast<float>(width);
        const float bottom = 1.0f - 2.0f * (topPixels + tilePixels) / static_cast<float>(height);
        drawTextureQuad(impl_->targets[index].texture, left, bottom,
                        2.0f * tilePixels / static_cast<float>(width),
                        2.0f * tilePixels / static_cast<float>(height));
    }
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
}

bool reflectionPassActive() {
    return activeReflectionPass;
}

unsigned int reflectionTextureForIndex(int index) {
    const auto found = activeReflectionTextures.find(index);
    return found == activeReflectionTextures.end() ? 0 : found->second;
}

int reflectionTextureIndex(const std::string& textureName) {
    std::string stem = std::filesystem::path(textureName).stem().string();
    std::transform(stem.begin(), stem.end(), stem.begin(), [](unsigned char character) {
        return static_cast<char>(std::tolower(character));
    });
    constexpr std::string_view prefix = "reflexion";
    if (stem.rfind(prefix, 0) != 0 || stem.size() == prefix.size()) {
        return -1;
    }
    try {
        const int index = std::stoi(stem.substr(prefix.size()));
        return index >= 0 ? index : -1;
    } catch (const std::exception&) {
        return -1;
    }
}

bool reflectionTransparentEnabled() {
    const char* value = std::getenv("OPENBUS_REFLECTION_TRANSPARENT");
    return value == nullptr || parseEnabledFlag(value);
}

} // namespace openbus::rendering
