#include "ComputeCulling.h"
#include "Logger.h"
#include "OpenGLFunctions.h"

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <cmath>
#include <iostream>
#include <limits>
#include <random>
#include <string_view>

Logger gameLog("ComputeCullingProbe");
using namespace openbus::rendering;

static_assert(!computeCullingRequested(""));
static_assert(!computeCullingRequested("auto"));
static_assert(!computeCullingRequested("cpu"));
static_assert(!computeCullingRequested("1"));
static_assert(computeCullingRequested("compute-readback"));

int main(int argc, char** argv) {
    const std::array<double, 16> identity = {1, 0, 0, 0, 0, 1, 0, 0,
                                            0, 0, 1, 0, 0, 0, 0, 1};
    const auto planes = makeCullingPlanes(identity);
    std::vector<float> boundaries;
    const std::array<double, 3> thresholds = {1.0, 0.5, 0.1};
    appendCullingLodBoundaries(boundaries, thresholds);
    const auto require = [](bool condition, const char* message) {
        if (!condition) std::cerr << message << '\n';
        return condition;
    };
    auto item = makeCullingItem(identity, {0, 0, 0}, 0.5);
    if (!require(cullingItemVisible(item, planes, {}), "Inside sphere rejected")) return 1;
    for (int axis = 0; axis < 3; ++axis) {
        for (const double sign : {-1.0, 1.0}) {
            std::array<double, 3> center = {};
            center[axis] = 2.0 * sign;
            if (!require(!cullingItemVisible(makeCullingItem(identity, center, 0.5), planes, {}),
                         "Outside sphere retained")) return 1;
            center[axis] = 1.5 * sign;
            if (!require(cullingItemVisible(makeCullingItem(identity, center, 0.5), planes, {}),
                         "Tangent sphere rejected")) return 1;
        }
    }
    auto translated = identity;
    translated[12] = -1000000000.0;
    item = makeCullingItem(translated, {1000000000.25, 0, 0}, 0.1);
    if (!require(item.eyeSphere[0] == 0.25F, "Large-origin precision lost")) return 1;
    item.metadata[0] = CullMapDistance;
    item.distanceBounds = {10000, -1, 10001, 1};
    if (!require(cullingItemVisible(item, planes, {}), "Distance boundary rejected")) return 1;
    item.distanceBounds[0] = 10001;
    if (!require(!cullingItemVisible(item, planes, {}), "Distant bounds retained")) return 1;
    item = makeCullingItem(identity, {30, 0, 0}, 0);
    item.metadata = {CullLod, 1, 0, 3};
    if (!require(cullingItemVisible(item, planes, boundaries), "Correct LOD rejected")) return 1;
    item.metadata[1] = 0;
    if (!require(!cullingItemVisible(item, planes, boundaries), "Wrong LOD retained")) return 1;
    item.metadata = {CullReject, 0, 0, 0};
    if (!require(!cullingItemVisible(item, planes, {}), "Explicit rejection ignored")) return 1;
    std::vector<std::uint32_t> result;
    if (!require(!computeCull({}, planes, {}, result), "Uninitialized compute should fall back")) return 1;
    // Default initialization must be safe even without a GL context or loader.
    initializeComputeCulling();
    if (!require(!computeCullingAvailable(), "Default backend must not enable GPU readback")) return 1;
    if (argc < 2 || std::string_view(argv[1]) != "--gpu") return 0;

    if (!glfwInit()) return 77;
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    GLFWwindow* window = glfwCreateWindow(64, 64, "Compute culling probe", nullptr, nullptr);
    if (!window) {
        glfwTerminate();
        return 77;
    }
    glfwMakeContextCurrent(window);
    if (!loadOpenGLFunctions()) return 1;
    initializeComputeCulling(true);
    if (!require(computeCullingAvailable(), "Compute initialization failed on a 4.3 context")) return 1;
    int exitCode = 0;
    const auto dispatch = pglDispatchCompute;
    pglDispatchCompute = nullptr;
    initializeComputeCulling(true);
    if (!require(!computeCullingAvailable(), "Missing compute entry point did not select fallback")) exitCode = 1;
    pglDispatchCompute = dispatch;
    initializeComputeCulling(true);
    if (!require(computeCullingAvailable(), "Compute reinitialization failed")) exitCode = 1;
    std::mt19937 random(813);
    std::uniform_real_distribution<float> position(-20000, 20000);
    std::vector<CullingItem> items;
    for (std::size_t count : {0U, 1U, 63U, 64U, 65U, 4097U}) {
        items.clear();
        for (std::size_t i = 0; i < count; ++i) {
            auto candidate = makeCullingItem(identity, {position(random), position(random), position(random)},
                                             std::abs(position(random)));
            candidate.metadata = {static_cast<std::uint32_t>(i % 16),
                                  static_cast<std::uint32_t>(i % 3), 0, 3};
            const float x = position(random), y = position(random);
            candidate.distanceBounds = {x - 500, y - 500, x + 500, y + 500};
            items.push_back(candidate);
        }
        if (!require(computeCull(items, planes, boundaries, result), "Compute dispatch failed") ||
            !require(result.size() == items.size(), "Wrong result count")) {
            exitCode = 1;
            break;
        }
        for (std::size_t i = 0; i < count; ++i) {
            if (!require((result[i] != 0) == cullingItemVisible(items[i], planes, boundaries),
                         "CPU/GPU visibility mismatch")) exitCode = 1;
        }
        GLint activeProgram = -1;
        glGetIntegerv(0x8B8D, &activeProgram);
        if (!require(activeProgram == 0, "Program binding was not restored")) exitCode = 1;
    }
    // Perspective planes and successive camera changes must not reuse stale masks.
    const std::array<double, 16> perspective = {1, 0, 0, 0, 0, 1, 0, 0,
                                               0, 0, -1.002002002, -1,
                                               0, 0, -0.2002002002, 0};
    const auto perspectivePlanes = makeCullingPlanes(perspective);
    const auto probeView = [&](const std::array<double, 16>& view, bool expected) {
        auto candidate = makeCullingItem(view, {0, 0, -10}, 0.5);
        return computeCull(std::span(&candidate, 1), perspectivePlanes, {}, result) &&
               result.size() == 1 && (result[0] != 0) == expected;
    };
    if (!require(probeView(identity, true), "Perspective visible sphere rejected")) exitCode = 1;
    translated = identity;
    translated[12] = 100;
    if (!require(probeView(translated, false) && probeView(identity, true),
                 "Camera change returned stale visibility")) exitCode = 1;
    // Restore nonzero SSBO bindings as well as the generic target binding.
    GLuint sentinel = 0;
    pglGenBuffers(1, &sentinel);
    pglBindBuffer(0x90D2, sentinel);
    pglBufferData(0x90D2, 64, nullptr, 0x88E0);
    pglBindBufferBase(0x90D2, 0, sentinel);
    if (!require(probeView(identity, true), "Bound-state compute dispatch failed")) exitCode = 1;
    GLint binding = 0;
    pglGetIntegeri(0x90D3, 0, &binding);
    if (!require(static_cast<GLuint>(binding) == sentinel, "Indexed binding was not restored")) exitCode = 1;
    glGetIntegerv(0x90D3, &binding);
    if (!require(static_cast<GLuint>(binding) == sentinel, "Generic binding was not restored")) exitCode = 1;
    pglBindBufferBase(0x90D2, 0, 0);
    pglBindBuffer(0x90D2, 0);
    pglDeleteBuffers(1, &sentinel);
    item.metadata = {CullLod, 0, std::numeric_limits<std::uint32_t>::max(), 3};
    if (!require(!computeCull(std::span(&item, 1), planes, boundaries, result),
                 "Invalid LOD range accepted")) exitCode = 1;
    initializeComputeCulling(false);
    if (!require(!computeCullingAvailable(), "CPU selection retained compute state")) exitCode = 1;
    if (!require(!computeCull(items, planes, boundaries, result) && result.empty(),
                 "CPU selection still submitted compute work")) exitCode = 1;
    const auto arraysIndirect = pglDrawArraysIndirect;
    pglDrawArraysIndirect = nullptr;
    initializeComputeCulling(false, true);
    if (!require(!indirectCullingAvailable(), "Missing indirect entry point did not fall back")) exitCode = 1;
    pglDrawArraysIndirect = arraysIndirect;
    const auto readback = pglGetBufferSubData;
    pglGetBufferSubData = nullptr;
    initializeComputeCulling(false, true);
    if (!require(indirectCullingAvailable() && !computeCullingAvailable(),
                 "Indirect backend must not require a readback entry point")) return 1;
    {
        GLuint vao = 0, indices = 0;
        pglGenVertexArrays(1, &vao);
        pglBindVertexArray(vao);
        pglGenBuffers(1, &indices);
        pglBindBuffer(0x8893, indices);
        const std::array<GLuint, 3> elements = {0, 1, 2};
        pglBufferData(0x8893, sizeof(elements), elements.data(), 0x88E4);
        const char* vertex = "#version 330 core\nvoid main(){vec2 p[3]=vec2[3](vec2(-1,-1),vec2(1,-1),vec2(0,1));gl_Position=vec4(p[gl_VertexID],0,1);}";
        const char* fragment = "#version 330 core\nout vec4 color;void main(){color=vec4(1);}";
        GLuint vs = pglCreateShader(GL_VERTEX_SHADER), fs = pglCreateShader(GL_FRAGMENT_SHADER);
        pglShaderSource(vs, 1, &vertex, nullptr); pglCompileShader(vs);
        pglShaderSource(fs, 1, &fragment, nullptr); pglCompileShader(fs);
        GLuint render = pglCreateProgram();
        pglAttachShader(render, vs); pglAttachShader(render, fs); pglLinkProgram(render);
        GLint linked = 0;
        pglGetProgramiv(render, GL_LINK_STATUS, &linked);
        if (!require(linked != 0, "Indirect raster probe shader failed")) exitCode = 1;
        pglDeleteShader(vs); pglDeleteShader(fs);
        pglUseProgram(render);
        glViewport(0, 0, 64, 64);
        glDisable(GL_DEPTH_TEST); glDisable(GL_CULL_FACE); glDisable(GL_BLEND);
        glClearColor(0, 0, 0, 1);
        IndirectCullingBatch batch;
        for (bool indexed : {false, true}) {
            for (int state : {0, 1, 2, 3, 0}) {
                auto candidate = makeCullingItem(identity, {state == 1 ? 10.0 : 0.0, 0, 0}, 0);
                candidate.draw[0] = state == 2 ? 0 : 3;
                if (state == 3) candidate.metadata[0] |= CullReject;
                if (!require(batch.dispatch(std::span(&candidate, 1), planes, {}), "Indirect dispatch failed without readback")) exitCode = 1;
                glClear(GL_COLOR_BUFFER_BIT);
                {
                    IndirectCullingScope scope;
                    batch.select(0);
                    if (!require(drawSelectedIndirect(indexed), "Indirect submission missing")) exitCode = 1;
                    {
                        IndirectCullingScope nested;
                        if (!require(!drawSelectedIndirect(indexed), "Nested scope retained selection")) exitCode = 1;
                    }
                    batch.select(999);
                    if (!require(!drawSelectedIndirect(indexed), "Invalid slot retained selection")) exitCode = 1;
                }
                if (!require(!drawSelectedIndirect(indexed), "Scope leaked selection")) exitCode = 1;
                std::array<GLubyte, 4> pixel = {};
                glReadPixels(32, 32, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, pixel.data());
                if (!require((pixel[0] > 200) == (state == 0), "Indirect raster visibility/count mismatch")) exitCode = 1;
            }
        }
        // TEST-ONLY downloads verify every command word and partial workgroup.
        pglGetBufferSubData = readback;
        for (std::size_t count : {0U, 1U, 63U, 64U, 65U, 4097U}) {
            items.resize(count);
            for (std::size_t i = 0; i < count; ++i) {
                auto candidate = makeCullingItem(identity, {position(random), position(random), position(random)}, std::abs(position(random)));
                candidate.metadata = {static_cast<std::uint32_t>(i % 16), static_cast<std::uint32_t>(i % 3), 0, 3};
                const float x = position(random), y = position(random);
                candidate.distanceBounds = {x - 500, y - 500, x + 500, y + 500};
                candidate.draw[0] = static_cast<std::uint32_t>(i * 3);
                items[i] = candidate;
            }
            if (!require(batch.dispatch(items, planes, boundaries), "Indirect parity dispatch failed")) exitCode = 1;
            if (!count) continue;
            pglMemoryBarrier(0x0200);
            pglBindBuffer(0x8F3F, batch.commandBuffer());
            std::vector<std::uint32_t> commands(count * 5);
            readback(0x8F3F, 0, static_cast<std::ptrdiff_t>(commands.size() * sizeof(std::uint32_t)), commands.data());
            for (std::size_t i = 0; i < count; ++i) {
                if (!require(commands[i * 5] == items[i].draw[0] &&
                    commands[i * 5 + 1] == static_cast<std::uint32_t>(cullingItemVisible(items[i], planes, boundaries)) &&
                    commands[i * 5 + 2] == 0 && commands[i * 5 + 3] == 0 && commands[i * 5 + 4] == 0,
                    "Indirect command/reference mismatch")) { exitCode = 1; break; }
            }
        }
        // Separate queued views retain their own commands even after another
        // dispatch reuses the service's input buffers.
        {
            IndirectCullingScope scope;
            IndirectCullingBatch visibleView, hiddenView;
            auto candidate = makeCullingItem(identity, {0, 0, 0}, 0);
            candidate.draw[0] = 3;
            if (!require(visibleView.dispatch(std::span(&candidate, 1), planes, {}), "Visible queued view failed")) exitCode = 1;
            candidate.eyeSphere[0] = 10;
            if (!require(hiddenView.dispatch(std::span(&candidate, 1), planes, {}), "Hidden queued view failed")) exitCode = 1;
            glClear(GL_COLOR_BUFFER_BIT);
            visibleView.select(0);
            drawSelectedIndirect(false);
            std::array<GLubyte, 4> pixel = {};
            glReadPixels(32, 32, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, pixel.data());
            if (!require(pixel[0] > 200, "Later view overwrote earlier commands")) exitCode = 1;
            glClear(GL_COLOR_BUFFER_BIT);
            hiddenView.select(0);
            drawSelectedIndirect(false);
            glReadPixels(32, 32, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, pixel.data());
            if (!require(pixel[0] == 0, "Queued hidden view rendered")) exitCode = 1;
        }
        item = makeCullingItem(identity, {0, 0, 0}, 0);
        item.draw[0] = std::numeric_limits<std::uint32_t>::max();
        if (!require(!batch.dispatch(std::span(&item, 1), planes, {}), "Oversized indirect geometry count accepted")) exitCode = 1;
        item.metadata = {CullLod, 0, std::numeric_limits<std::uint32_t>::max(), 3};
        if (!require(!batch.dispatch(std::span(&item, 1), planes, boundaries), "Indirect invalid LOD accepted")) exitCode = 1;
        pglBindBuffer(0x8F3F, indices);
        {
            IndirectCullingScope scope;
            batch.select(0);
        }
        glGetIntegerv(0x8F43, &binding);
        if (!require(binding == static_cast<GLint>(indices), "Indirect binding restoration failed")) exitCode = 1;
        if (!require(glGetError() == GL_NO_ERROR, "Indirect GL error")) exitCode = 1;
        pglBindBuffer(0x8F3F, 0);
        pglUseProgram(0); pglDeleteProgram(render);
        pglBindVertexArray(0); pglDeleteVertexArrays(1, &vao); pglDeleteBuffers(1, &indices);
    }
    pglGetBufferSubData = readback;
    shutdownComputeCulling();
    glfwDestroyWindow(window);
    glfwTerminate();
    return exitCode;
}