#include "RendererReflection.h"

#include "BusSimulation.h"
#include "CoreRenderer.h"
#include "Environment.h"
#include "Logger.h"
#include "OpenGLFunctions.h"
#include "PerfTrace.h"
#include "Variables.h"

#include <GL/gl.h>

#include <algorithm>
#include <cmath>
#include <cctype>
#include <cstdlib>
#include <cstdint>
#include <chrono>
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
    const char* value = openbus::getEnvironment("OPENBUS_REFLECTION_SIZE");
    if (value == nullptr) {
        return 1024;
    }
    const int size = std::atoi(value);
    return size == 256 || size == 512 || size == 1024 ? size : 256;
}

int reflectionIntervalFromEnvironment() {
    const char* value = openbus::getEnvironment("OPENBUS_REFLECTION_INTERVAL");
    return value == nullptr ? 1 : std::max(1, std::atoi(value));
}

double reflectionMaxFrameRateFromEnvironment() {
    const char* value = openbus::getEnvironment("OPENBUS_REFLECTION_MAX_FPS");
    if (value == nullptr) {
        return 0.0;
    }
    try {
        const double frameRate = std::stod(value);
        return std::isfinite(frameRate) && frameRate > 0.0 ? std::clamp(frameRate, 1.0, 1000.0)
                                                           : 0.0;
    } catch (const std::exception&) {
        return 0.0;
    }
}

} // namespace

struct ReflectionRenderer::Impl {
    int maximumSize;
    int frameInterval;
    double maximumFrameRate;
    std::uint64_t frameCounter = 0;
    std::chrono::steady_clock::time_point lastRenderTime = {};
    bool hasRendered = false;
    std::vector<ReflectionTarget> targets;

    Impl(int maximumSizeValue, int frameIntervalValue, double maximumFrameRateValue)
        : maximumSize(std::max(maximumSizeValue, kMinReflectionTargetSize)),
          frameInterval(std::max(frameIntervalValue, 1)), maximumFrameRate(maximumFrameRateValue) {}

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
                                   reflectionIntervalFromEnvironment(),
                                   reflectionMaxFrameRateFromEnvironment())) {
    gameLog.Log("Reflection targets: " + std::to_string(impl_->maximumSize) + "x" +
                std::to_string(impl_->maximumSize) + ", every " +
                std::to_string(impl_->frameInterval) + " frame(s)");
    if (impl_->maximumFrameRate > 0.0) {
        gameLog.Log("Reflection updates capped at " + std::to_string(impl_->maximumFrameRate) +
                    " FPS");
    }
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
                                const std::array<int, 4>& viewport,
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
    const auto renderStart = std::chrono::steady_clock::now();
    if (impl_->maximumFrameRate > 0.0 && impl_->hasRendered &&
        std::chrono::duration<double>(renderStart - impl_->lastRenderTime).count() <
            1.0 / impl_->maximumFrameRate) {
        return;
    }
    impl_->lastRenderTime = renderStart;
    impl_->hasRendered = true;

    Matrix4 previousModelView;
    {
        TraceScope phase("render", "ReflectionRenderer::render.setup");
        previousModelView = modelViewMatrix();
        activeReflectionPass = true;
    }
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
        ReflectionRequirement requirement;
        {
            TraceScope phase("render", "ReflectionRenderer::render.target.visibility");
            requirement = visibility(reflectionIndex);
        }
        if (!requirement.needed) {
            ++reflectionIndex;
            continue;
        }

        const int requestedTargetSize =
            std::clamp(requirement.size, kMinReflectionTargetSize, impl_->maximumSize);
        if (target.width != requestedTargetSize || target.height != requestedTargetSize) {
            TraceScope phase("render", "ReflectionRenderer::render.target.resize");
            impl_->resizeTarget(target, requestedTargetSize);
        }
        {
            TraceScope phase("render", "ReflectionRenderer::render.target.clear");
            pglBindFramebuffer(GL_FRAMEBUFFER, target.framebuffer);
            glViewport(0, 0, target.width, target.height);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        }
        {
            TraceScope phase("render", "ReflectionRenderer::render.target.draw");
            draw(simulation, cameraIndex, target.width, target.height);
        }
        ++reflectionIndex;
    }
    {
        TraceScope phase("render", "ReflectionRenderer::render.restore");
        activeReflectionPass = false;
        pglBindFramebuffer(GL_FRAMEBUFFER, 0);
        glViewport(viewport[0], viewport[1], viewport[2], viewport[3]);
        restore(viewport[2], viewport[3]);
        setModelViewMatrix(previousModelView);
    }
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
    const char* value = openbus::getEnvironment("OPENBUS_REFLECTION_TRANSPARENT");
    return value == nullptr || parseEnabledFlag(value);
}

} // namespace openbus::rendering
