#include "render/CoreRenderer.h"
#include "render/GlCompat.h"
#include "render/MatrixMath.h"
#include "render/OpenGLFunctions.h"

#include <GLFW/glfw3.h>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string_view>

namespace {

using namespace openbus::rendering;

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void readMotionPixel(GLuint framebuffer, const MotionDataResources& resources, int x, int y,
                     std::array<float, 2>& motion, std::uint8_t& validity, float& depth) {
    GLint previousFramebuffer = 0;
    glGetIntegerv(GL_FRAMEBUFFER_BINDING, &previousFramebuffer);
    pglBindFramebuffer(GL_FRAMEBUFFER, framebuffer);
    pglFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
                            resources.motionVectors, 0);
    const GLenum drawBuffer = GL_COLOR_ATTACHMENT0;
    pglDrawBuffers(1, &drawBuffer);
    require(pglCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE,
            "motion readback framebuffer is incomplete");
    glReadPixels(x, y, 1, 1, GL_RG, GL_FLOAT, motion.data());

    pglFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
                            resources.validity, 0);
    require(pglCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE,
            "validity readback framebuffer is incomplete");
    glReadPixels(x, y, 1, 1, GL_RED, GL_UNSIGNED_BYTE, &validity);

    pglFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, resources.depth,
                            0);
    require(pglCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE,
            "depth readback framebuffer is incomplete");
    glReadPixels(x, y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &depth);
    pglBindFramebuffer(GL_FRAMEBUFFER, static_cast<GLuint>(previousFramebuffer));
}

struct BenchmarkResult {
    double cpuMillisecondsPerFrame = 0.0;
    double gpuMillisecondsPerFrame = 0.0;
};

void benchmarkFrames(bool withMotion, GLuint vertexBuffer, MotionObjectKey object, int width,
                     int height, int firstFrame, int frameCount, double renderScale = 1.0) {
    for (int frameIndex = firstFrame; frameIndex < firstFrame + frameCount; ++frameIndex) {
        setModelViewMatrix(identityMatrix());
        if (withMotion) {
            const double timestamp = 10.0 + static_cast<double>(frameIndex) / 60.0;
            require(beginMotionDataFrame(width, height, timestamp, renderScale),
                    "motion-data benchmark target could not be created");
        } else {
            pglBindFramebuffer(GL_FRAMEBUFFER, 0);
            glViewport(0, 0, width, height);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        }
        drawModelBatch(vertexBuffer, 3, ModelMaterial{}, {1.0, 1.0, 1.0}, 1.0, 0, false,
                       std::nullopt, 1.0, nullptr, object);
        if (withMotion) {
            finishMotionDataFrame();
        }
    }

}

BenchmarkResult measureFrames(bool withMotion, GLuint vertexBuffer, MotionObjectKey object,
                             int width, int height, int firstFrame, int frameCount,
                             double renderScale = 1.0) {
    GLuint query = 0;
    pglGenQueries(1, &query);
    require(query != 0, "GPU timing query allocation failed");
    glFinish();
    pglBeginQuery(GL_TIME_ELAPSED, query);
    const auto start = std::chrono::steady_clock::now();
    benchmarkFrames(withMotion, vertexBuffer, object, width, height, firstFrame, frameCount,
                    renderScale);
    pglEndQuery(GL_TIME_ELAPSED);
    glFinish();
    const auto end = std::chrono::steady_clock::now();
    std::uint64_t gpuNanoseconds = 0;
    pglGetQueryObjectui64v(query, GL_QUERY_RESULT, &gpuNanoseconds);
    pglDeleteQueries(1, &query);
    return {std::chrono::duration<double, std::milli>(end - start).count() /
                static_cast<double>(frameCount),
            static_cast<double>(gpuNanoseconds) / 1'000'000.0 /
                static_cast<double>(frameCount)};
}

void runBenchmark(GLuint vertexBuffer, MotionObjectKey object, int width, int height) {
    constexpr int warmupFrames = 10;
    constexpr int measuredFrames = 120;
    benchmarkFrames(false, vertexBuffer, object, width, height, 0, warmupFrames);
    benchmarkFrames(true, vertexBuffer, object, width, height, warmupFrames, warmupFrames);
    const BenchmarkResult baseline = measureFrames(false, vertexBuffer, object, width, height,
                                                   warmupFrames * 2, measuredFrames);
    const BenchmarkResult motion = measureFrames(true, vertexBuffer, object, width, height,
                                                 warmupFrames * 2 + measuredFrames,
                                                 measuredFrames);
    setFxaaEnabled(true);
    const BenchmarkResult fxaa = measureFrames(true, vertexBuffer, object, width, height,
                                               warmupFrames * 2 + measuredFrames * 2,
                                               measuredFrames);
    setFxaaEnabled(false);
    const int halfScaleFirstFrame = warmupFrames * 2 + measuredFrames * 3;
    benchmarkFrames(true, vertexBuffer, object, width, height, halfScaleFirstFrame, warmupFrames,
                    0.5);
    const BenchmarkResult halfScale = measureFrames(
        true, vertexBuffer, object, width, height, halfScaleFirstFrame + warmupFrames,
        measuredFrames, 0.5);
    const double memoryMiB = static_cast<double>(width) * static_cast<double>(height) * 13.0 /
                             (1024.0 * 1024.0);
    std::cout << "Motion data GPU benchmark\n"
              << "resolution=" << width << 'x' << height << " frames=" << measuredFrames << '\n'
              << "baseline_cpu_ms_per_frame=" << baseline.cpuMillisecondsPerFrame << '\n'
              << "motion_cpu_ms_per_frame=" << motion.cpuMillisecondsPerFrame << '\n'
              << "baseline_gpu_ms_per_frame=" << baseline.gpuMillisecondsPerFrame << '\n'
              << "motion_gpu_ms_per_frame=" << motion.gpuMillisecondsPerFrame << '\n'
              << "fxaa_motion_cpu_ms_per_frame=" << fxaa.cpuMillisecondsPerFrame << '\n'
              << "fxaa_motion_gpu_ms_per_frame=" << fxaa.gpuMillisecondsPerFrame << '\n'
              << "fxaa_gpu_delta_vs_blit_ms_per_frame="
              << fxaa.gpuMillisecondsPerFrame - motion.gpuMillisecondsPerFrame << '\n'
              << "half_scale_motion_cpu_ms_per_frame=" << halfScale.cpuMillisecondsPerFrame
              << '\n'
              << "half_scale_motion_gpu_ms_per_frame=" << halfScale.gpuMillisecondsPerFrame
              << '\n'
              << "half_scale_gpu_delta_vs_native_ms_per_frame="
              << halfScale.gpuMillisecondsPerFrame - motion.gpuMillisecondsPerFrame << '\n'
              << "half_scale_targets_MiB=" << memoryMiB * 0.25 << '\n'
              << "estimated_targets_MiB=" << memoryMiB << '\n'
              << "GPU result includes existing geometry, MRT clears, and color blit; CPU "
                 "measurement includes submission and glFinish synchronization. FXAA replaces "
                 "the color blit with a fullscreen filter draw. Half scale is a separate "
                 "640x360 motion-MRT run; it is not a full-scene performance prediction.\n";
}

} // namespace

int main(int argc, char** argv) {
    GLFWwindow* window = nullptr;
    GLuint vertexBuffer = 0;
    GLuint readbackFramebuffer = 0;
    bool rendererInitialized = false;
    if (glfwInit() != GLFW_TRUE) {
        return 77;
    }
    try {
        const bool benchmarkMode =
            argc == 2 && std::string_view(argv[1]) == "--benchmark";
        const int width = benchmarkMode ? 1280 : 32;
        const int height = benchmarkMode ? 720 : 32;
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
        glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
        window = glfwCreateWindow(width, height, "Motion data GPU probe", nullptr, nullptr);
        if (window == nullptr) {
            glfwTerminate();
            return 77;
        }
        glfwMakeContextCurrent(window);
        require(loadOpenGLFunctions(), "OpenGL function loading failed");
        rendererInitialized = initializeCoreRenderer();
        require(rendererInitialized, "core renderer initialization failed");

        constexpr std::array<float, 39> vertices = {
            -0.8F, -0.8F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, -1.0F, 1.0F, 1.0F, 1.0F, 1.0F,
             0.8F, -0.8F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, -1.0F, 1.0F, 1.0F, 1.0F, 1.0F,
             0.0F,  0.8F, 0.0F, 0.5F, 1.0F, 0.0F, 0.0F, 0.0F, -1.0F, 1.0F, 1.0F, 1.0F, 1.0F,
        };
        pglGenBuffers(1, &vertexBuffer);
        pglBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
        pglBufferData(GL_ARRAY_BUFFER, static_cast<std::ptrdiff_t>(sizeof(vertices)),
                      vertices.data(), GL_STATIC_DRAW);
        glEnable(GL_DEPTH_TEST);
        glDepthMask(GL_TRUE);
        glClearColor(0.0F, 0.0F, 0.0F, 1.0F);
        setProjectionMatrix(identityMatrix());
        const MotionObjectKey object{allocateMotionObjectId(), 1};

        if (benchmarkMode) {
            require(pglGenQueries != nullptr && pglDeleteQueries != nullptr &&
                        pglBeginQuery != nullptr && pglEndQuery != nullptr &&
                        pglGetQueryObjectui64v != nullptr,
                    "OpenGL timer-query functions are unavailable");
            runBenchmark(vertexBuffer, object, width, height);
            pglDeleteBuffers(1, &vertexBuffer);
            vertexBuffer = 0;
            shutdownCoreRenderer();
            rendererInitialized = false;
            glfwDestroyWindow(window);
            window = nullptr;
            glfwTerminate();
            return 0;
        }

        pglGenFramebuffers(1, &readbackFramebuffer);
        setFxaaEnabled(true);
        for (int frameIndex = 0; frameIndex < 4; ++frameIndex) {
            const Matrix4 view =
                frameIndex == 3 ? translationMatrix({-0.25, 0.0, 0.0}) : identityMatrix();
            setModelViewMatrix(view);
            const double timestamp = 1.0 + static_cast<double>(frameIndex) / 60.0;
            require(beginMotionDataFrame(width, height, timestamp),
                    "motion-data render target could not be created");
            if (frameIndex == 2) {
                setModelViewMatrix(translationMatrix({0.25, 0.0, 0.0}));
            } else if (frameIndex == 3) {
                setModelViewMatrix(
                    multiplyMatrix4(view, translationMatrix({0.25, 0.0, 0.0})));
            }
            drawModelBatch(vertexBuffer, 3, ModelMaterial{}, {1.0, 1.0, 1.0}, 1.0, 0, false,
                           std::nullopt, 1.0, nullptr, object);
            finishMotionDataFrame();

            const MotionDataResources resources = motionDataResources();
            require(resources.available && resources.frame.frameId ==
                                               static_cast<std::uint64_t>(frameIndex + 1),
                    "rendered motion-data metadata is unavailable or out of sequence");
            if (frameIndex == 0) {
                require(!resources.frame.historyValid,
                        "the first motion-data frame must not claim valid camera history");
            } else {
                require(resources.frame.historyValid,
                        "continuous rendered frames should have valid camera history");
            }
            if (frameIndex == 0) {
                std::array<float, 2> motion = {};
                std::uint8_t validity = 0;
                float depth = 1.0F;
                readMotionPixel(readbackFramebuffer, resources, 16, 16, motion, validity, depth);
                require(validity == 0,
                        "geometry on the first rendered frame must have invalid motion");
            } else if (frameIndex == 1) {
                std::array<float, 2> motion = {};
                std::uint8_t validity = 0;
                float depth = 1.0F;
                readMotionPixel(readbackFramebuffer, resources, 16, 16, motion, validity, depth);
                require(validity >= 250, "stationary geometry must have valid motion data");
                require(std::abs(motion[0]) < 0.15F && std::abs(motion[1]) < 0.15F,
                        "stationary geometry must produce zero pixel displacement");
                require(depth > 0.45F && depth < 0.55F,
                        "depth texture must contain the current projected geometry depth");
            } else if (frameIndex == 2) {
                std::array<float, 2> motion = {};
                std::uint8_t validity = 0;
                float depth = 1.0F;
                readMotionPixel(readbackFramebuffer, resources, 20, 16, motion, validity, depth);
                require(validity >= 250, "moving opaque geometry must have valid motion data");
                require(std::abs(motion[0] + 4.0F) < 0.15F,
                        "motion X must be current-to-previous displacement in pixels");
                require(std::abs(motion[1]) < 0.15F,
                        "stationary vertical position must produce zero Y motion");
                require(depth > 0.45F && depth < 0.55F,
                        "depth texture must contain the current projected geometry depth");
            } else {
                std::array<float, 2> motion = {};
                std::uint8_t validity = 0;
                float depth = 1.0F;
                readMotionPixel(readbackFramebuffer, resources, 16, 16, motion, validity, depth);
                require(validity >= 250, "camera motion over static geometry must be valid");
                require(std::abs(motion[0] - 4.0F) < 0.15F,
                        "camera movement must produce current-to-previous pixel displacement");
                require(std::abs(motion[1]) < 0.15F,
                        "horizontal camera movement must produce zero Y motion");
                require(depth > 0.45F && depth < 0.55F,
                        "depth texture must contain the current projected geometry depth");
            }
        }
        setFxaaEnabled(false);

        glViewport(0, 0, width, height);
        setModelViewMatrix(identityMatrix());
        require(beginMotionDataFrame(width, height, 2.0, 0.5),
                "scaled motion-data render target could not be created");
        drawModelBatch(vertexBuffer, 3, ModelMaterial{}, {1.0, 1.0, 1.0}, 1.0, 0, false,
                       std::nullopt, 1.0, nullptr, object);
        finishMotionDataFrame();
        const MotionDataResources scaledResources = motionDataResources();
        require(scaledResources.available && scaledResources.width == width / 2 &&
                    scaledResources.height == height / 2 &&
                    scaledResources.frame.displayWidth == width &&
                    scaledResources.frame.displayHeight == height &&
                    !scaledResources.frame.historyValid,
                "scaled motion buffers must expose render/display sizes and reset history");
        std::array<std::uint8_t, 4> scaledOutputPixel = {};
        glReadPixels(width / 2, height / 2, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE,
                     scaledOutputPixel.data());
        require(scaledOutputPixel[0] > 100 && scaledOutputPixel[1] > 100 &&
                    scaledOutputPixel[2] > 100,
                "scaled world color must be upscaled into the display framebuffer");

        setFxaaEnabled(true);
        setModelViewMatrix(identityMatrix());
        require(beginMotionDataFrame(width, height, 2.016, 0.5),
                "scaled FXAA motion-data target could not be created");
        drawModelBatch(vertexBuffer, 3, ModelMaterial{}, {1.0, 1.0, 1.0}, 1.0, 0, false,
                       std::nullopt, 1.0, nullptr, object);
        finishMotionDataFrame();
        require(motionDataResources().frame.historyValid,
                "history should remain valid for stable scaled frames");
        setFxaaEnabled(false);

        pglDeleteFramebuffers(1, &readbackFramebuffer);
        readbackFramebuffer = 0;
        pglDeleteBuffers(1, &vertexBuffer);
        vertexBuffer = 0;
        shutdownCoreRenderer();
        rendererInitialized = false;
        glfwDestroyWindow(window);
        window = nullptr;
        glfwTerminate();
        std::cout << "Motion data GPU output checks passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Motion data GPU probe failed: " << error.what() << '\n';
        if (readbackFramebuffer != 0) {
            pglDeleteFramebuffers(1, &readbackFramebuffer);
        }
        if (vertexBuffer != 0) {
            pglDeleteBuffers(1, &vertexBuffer);
        }
        if (rendererInitialized) {
            shutdownCoreRenderer();
        }
        if (window != nullptr) {
            glfwDestroyWindow(window);
        }
        glfwTerminate();
        return 1;
    }
}
