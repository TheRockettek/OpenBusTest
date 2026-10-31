#include "render/ComputeCulling.h"
#include "core/Logger.h"
#include "render/CoreRenderer.h"
#include "render/OpenGLFunctions.h"

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <random>
#include <string_view>

using namespace openbus::rendering;

namespace {
BindBufferProc originalBindBuffer = nullptr;
BufferDataProc originalBufferData = nullptr;
GetIntegeriProc originalGetIntegeri = nullptr;
unsigned int indirectBindCount = 0;
unsigned int storageUploads = 0;
unsigned int indexedStorageQueries = 0;

void recordBufferBind(GLenum target, GLuint buffer) {
    if (target == 0x8F3F /* GL_DRAW_INDIRECT_BUFFER */) ++indirectBindCount;
    originalBindBuffer(target, buffer);
}
void recordBufferData(GLenum target, std::ptrdiff_t size, const void* data, GLenum usage) {
    if (target == 0x90D2 /* GL_SHADER_STORAGE_BUFFER */) ++storageUploads;
    originalBufferData(target, size, data, usage);
}
void APIENTRY recordGetIntegeri(GLenum target, GLuint index, GLint* value) {
    if (target == 0x90D3 /* GL_SHADER_STORAGE_BUFFER_BINDING */) ++indexedStorageQueries;
    originalGetIntegeri(target, index, value);
}
} // namespace

static_assert(!indirectCullingRequested(""));
static_assert(!indirectCullingRequested("auto"));
static_assert(!indirectCullingRequested("cpu"));
static_assert(!indirectCullingRequested("1"));
static_assert(!indirectCullingRequested("compute-readback"));
static_assert(indirectCullingRequested("compute-indirect"));

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
    auto screenProjection = identity;
    screenProjection[5] = 1.0;
    const auto identityView = identity;
    if (!require(sphereBelowMinimumViewportFraction(identityView, screenProjection, {0, 0, -10},
                                                    0.1, 0.02),
                 "Small projected sphere retained")) return 1;
    if (!require(!sphereBelowMinimumViewportFraction(identityView, screenProjection, {0, 0, -10},
                                                     0.1, 0.01),
                 "Sphere above the minimum projected size culled")) return 1;
    if (!require(!sphereBelowMinimumViewportFraction(identityView, screenProjection, {0, 0, -0.5},
                                                     1.0, 0.10),
                 "Near-plane sphere rejected by projected-size test")) return 1;
    if (!require(!sphereBelowMinimumViewportFraction(identityView, screenProjection, {0, 0, -10},
                                                     0.0, 0.02),
                 "Zero-radius sphere rejected by projected-size test")) return 1;
    const auto sizeSettings = makeCullingProjectedSize(screenProjection, 0.02);
    if (!require(projectedSizeFadeStart(0.02) == 0.01,
                 "Projected-size fade did not start at half the configured threshold")) return 1;
    const auto fadeBelowStart = projectedSizeFadeAlpha(
        identityView, screenProjection, {0, 0, -(0.1 + 0.1 / 0.005)}, 0.1, 0.02);
    const auto fadeMidpoint = projectedSizeFadeAlpha(
        identityView, screenProjection, {0, 0, -(0.1 + 0.1 / 0.015)}, 0.1, 0.02);
    const auto fadeAtThreshold = projectedSizeFadeAlpha(
        identityView, screenProjection, {0, 0, -(0.1 + 0.1 / 0.02)}, 0.1, 0.02);
    if (!require(fadeBelowStart == 0.0, "Projected-size fade started below its fade band") ||
        !require(std::abs(fadeMidpoint - 0.5) < 1.0e-9,
                 "Projected-size fade was not smooth at its midpoint") ||
        !require(fadeAtThreshold == 1.0,
                 "Projected-size fade was not fully opaque at its configured threshold") ||
        !require(projectedSizeFadeAlpha(identityView, screenProjection, {0, 0, -10}, 0.1, 0.0) ==
                     1.0,
                 "Disabled projected-size cutoff faded an object") ||
        !require(projectedSizeFadeAlpha(identityView, screenProjection, {0, 0, -0.5}, 1.0, 0.02) ==
                     1.0,
                 "Near-camera projected-size bounds were not retained fully opaque")) return 1;
    const auto fadeBandSettings =
        makeCullingProjectedSize(screenProjection, projectedSizeFadeStart(0.02));
    auto fadeBandItem =
        makeCullingItem(identityView, {0, 0, -(0.1 + 0.1 / 0.015)}, 0.1);
    fadeBandItem.metadata[0] = CullProjectedSize;
    auto belowFadeBandItem =
        makeCullingItem(identityView, {0, 0, -(0.1 + 0.1 / 0.005)}, 0.1);
    belowFadeBandItem.metadata[0] = CullProjectedSize;
    const CullingPlanes unboundedPlanes = {};
    if (!require(cullingProjectedSizeMayPass(fadeBandItem, fadeBandSettings) &&
                     cullingItemVisible(fadeBandItem, unboundedPlanes, {}, fadeBandSettings),
                 "Projected-size culling removed an object inside the fade band") ||
        !require(!cullingProjectedSizeMayPass(belowFadeBandItem, fadeBandSettings) &&
                     !cullingItemVisible(belowFadeBandItem, unboundedPlanes, {},
                                         fadeBandSettings),
                 "Projected-size culling retained an object below the fade band")) return 1;
    auto sizeItem = makeCullingItem(identity, {0, 0, -10}, 0.1);
    sizeItem.metadata[0] = CullProjectedSize;
    if (!require(!cullingProjectedSizeMayPass(sizeItem, sizeSettings),
                 "Projected-size broad phase retained clearly small object") ||
        !require(cullingProjectedSizeMayPass(sizeItem, {}),
                 "Projected-size broad phase rejected disabled cutoff")) return 1;
    if (!require(!cullingItemVisible(sizeItem, planes, {}, sizeSettings),
                 "Projected-size CPU reference retained small object") ||
        !require(cullingItemVisible(sizeItem, planes, {}, {}),
                 "Disabled projected-size cutoff rejected object") ||
        !require(cullingItemVisible(sizeItem, planes, {},
                                    makeCullingProjectedSize(screenProjection, 0.01)),
                 "Main-view projected-size threshold rejected large object")) return 1;
    sizeItem.eyeSphere[2] = -0.05F;
    if (!require(cullingProjectedSizeMayPass(sizeItem, sizeSettings),
                 "Projected-size broad phase rejected sphere across camera") ||
        !require(cullingItemVisible(sizeItem, planes, {}, sizeSettings),
                 "Projected-size CPU reference rejected sphere across camera")) return 1;
    sizeItem.eyeSphere[2] = -10.0F;
    sizeItem.eyeSphere[3] = 0.0F;
    if (!require(cullingProjectedSizeMayPass(sizeItem, sizeSettings),
                 "Projected-size broad phase rejected degenerate sphere") ||
        !require(cullingItemVisible(sizeItem, planes, {}, sizeSettings),
                 "Projected-size CPU reference rejected degenerate sphere")) return 1;
    sizeItem.eyeSphere[2] = -102.0F;
    sizeItem.eyeSphere[3] = 2.0F;
    if (!require(cullingProjectedSizeMayPass(sizeItem, sizeSettings),
                 "Projected-size broad phase rejected threshold boundary") ||
        !require(cullingItemVisible(sizeItem, planes, {}, sizeSettings),
                 "Projected-size CPU reference rejected threshold boundary")) return 1;
    sizeItem.eyeSphere[2] = -10.0F;
    sizeItem.eyeSphere[3] = 0.1F;
    sizeItem.metadata[0] = 0;
    if (!require(cullingProjectedSizeMayPass(sizeItem, sizeSettings),
                 "Projected-size broad phase ignored disabled item flag")) return 1;
    sizeItem.metadata[0] = CullProjectedSize;
    for (float invalid : {std::numeric_limits<float>::quiet_NaN(),
                          std::numeric_limits<float>::infinity(),
                          std::numeric_limits<float>::denorm_min()}) {
        sizeItem.eyeSphere[3] = invalid;
        if (!require(cullingProjectedSizeMayPass(sizeItem, sizeSettings),
                     "Projected-size broad phase rejected uncertain radius")) return 1;
    }
    sizeItem.eyeSphere[3] = std::numeric_limits<float>::max();
    sizeItem.eyeSphere[2] = -std::numeric_limits<float>::max();
    if (!require(cullingProjectedSizeMayPass(sizeItem, sizeSettings),
                 "Projected-size broad phase rejected extreme sphere")) return 1;
    sizeItem.eyeSphere[2] = -1.0e20F;
    sizeItem.eyeSphere[3] = 1.0e19F;
    if (!require(cullingProjectedSizeMayPass(sizeItem, {1.0e20F, 0.5F}),
                 "Projected-size broad phase rejected overflowing GPU arithmetic")) return 1;
    sizeItem.eyeSphere[2] = -1.0e20F;
    sizeItem.eyeSphere[3] = 1.0e-20F;
    if (!require(cullingProjectedSizeMayPass(sizeItem, {1.0F, 0.5F}),
                 "Projected-size broad phase rejected underflowing GPU arithmetic")) return 1;
    auto item = makeCullingItem(identity, {0, 0, 0}, 0.5);
    if (!require(cullingItemVisible(item, planes, {}), "Inside sphere rejected")) return 1;
    if (!require(cullingFrustumMayIntersect(item, planes), "Inside sphere rejected by broad phase"))
        return 1;
    for (int axis = 0; axis < 3; ++axis) {
        for (const double sign : {-1.0, 1.0}) {
            std::array<double, 3> center = {};
            center[axis] = 2.0 * sign;
            if (!require(!cullingItemVisible(makeCullingItem(identity, center, 0.5), planes, {}),
                         "Outside sphere retained")) return 1;
            center[axis] = 1.5 * sign;
            if (!require(cullingItemVisible(makeCullingItem(identity, center, 0.5), planes, {}),
                         "Tangent sphere rejected")) return 1;
            if (!require(cullingFrustumMayIntersect(makeCullingItem(identity, center, 0.5), planes),
                         "Tangent sphere rejected by broad phase")) return 1;
        }
    }
    item.eyeSphere[0] = std::numeric_limits<float>::infinity();
    if (!require(cullingFrustumMayIntersect(item, planes),
                 "Uncertain sphere rejected by broad phase")) return 1;
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
    if (!require(!cullingLodMayMatch(item, boundaries), "Wrong LOD passed the broad phase"))
        return 1;
    item = makeCullingItem(identity, {25.0, 0, 0}, 0);
    item.metadata = {CullLod, 0, 0, 3};
    if (!require(cullingLodMayMatch(item, boundaries), "Boundary LOD rejected by broad phase"))
        return 1;
    item.metadata[1] = 1;
    if (!require(cullingLodMayMatch(item, boundaries),
                 "Adjacent boundary LOD rejected by broad phase")) return 1;
    item.eyeSphere[0] = std::numeric_limits<float>::infinity();
    if (!require(cullingLodMayMatch(item, boundaries), "Uncertain LOD rejected by broad phase"))
        return 1;
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
    {
        auto candidate = makeCullingItem(identity, {30, 0, 0}, 0);
        candidate.metadata = {CullLod, 0, 0, 3};
        auto changedBoundaries = boundaries;
        changedBoundaries[0] = 35.0F;
        originalBufferData = pglBufferData;
        pglBufferData = recordBufferData;
        const auto checkUpload = [&](std::span<const float> levels, unsigned int expectedUploads,
                                     std::uint32_t expectedResult) {
            storageUploads = 0;
            if (!require(computeCull(std::span(&candidate, 1), planes, levels, result),
                         "Cached LOD dispatch failed") ||
                !require(result.size() == 1 && result[0] == expectedResult,
                         "Cached LOD dispatch used stale boundaries") ||
                !require(storageUploads == expectedUploads,
                         "LOD buffer was uploaded or reused unexpectedly"))
                exitCode = 1;
        };
        checkUpload(boundaries, 3, 0);
        checkUpload(boundaries, 2, 0);
        checkUpload(changedBoundaries, 3, 1);
        checkUpload(changedBoundaries, 2, 1);
        candidate.metadata[3] = 0;
        checkUpload({}, 3, 1);
        checkUpload({}, 2, 1);
        candidate.metadata[3] = 3;
        checkUpload(boundaries, 3, 0);
        pglBufferData = originalBufferData;
    }
    {
        std::vector<CullingItem> frustumItems;
        std::mt19937 frustumRandom(438);
        std::uniform_real_distribution<float> position(-10.0F, 10.0F);
        std::uniform_real_distribution<float> radius(0.0F, 2.0F);
        for (std::size_t index = 0; index < 4096; ++index) {
            std::array<double, 3> center = {position(frustumRandom), position(frustumRandom),
                                            position(frustumRandom)};
            const float size = radius(frustumRandom);
            if (index < 72) {
                center = {};
                const std::size_t axis = (index / 12) % 3;
                const double sign = (index / 6) % 2 == 0 ? -1.0 : 1.0;
                center[axis] = sign * (1.0 + size +
                                       (static_cast<int>(index % 6) - 3) * 0.0001);
            }
            frustumItems.push_back(makeCullingItem(identity, center, size));
        }
        if (!require(computeCull(frustumItems, planes, {}, result),
                     "Frustum broad-phase GPU dispatch failed")) {
            exitCode = 1;
        } else {
            std::size_t rejected = 0;
            for (std::size_t index = 0; index < frustumItems.size(); ++index) {
                if (!cullingFrustumMayIntersect(frustumItems[index], planes)) {
                    ++rejected;
                    if (!require(result[index] == 0,
                                 "Frustum broad phase rejected a GPU-visible candidate")) {
                        exitCode = 1;
                        break;
                    }
                }
            }
            if (!require(rejected > frustumItems.size() / 2,
                         "Frustum broad phase did not remove enough distant candidates")) exitCode = 1;
        }
    }
    {
        std::array<double, 16> perspective = {};
        perspective[0] = 1.2;
        perspective[5] = 1.3;
        perspective[10] = -1.0002;
        perspective[11] = -1.0;
        perspective[14] = -0.2;
        const auto perspectivePlanes = makeCullingPlanes(perspective);
        std::vector<CullingItem> candidates;
        std::mt19937 random(431);
        std::uniform_real_distribution<float> position(-20000.0F, 20000.0F);
        std::uniform_real_distribution<float> depth(1.0F, 10000.0F);
        for (std::size_t index = 0; index < 4096; ++index) {
            std::array<double, 3> center = {position(random), position(random), -depth(random)};
            const double radius = static_cast<double>(index % 10);
            if (index < 512) {
                center[1] = 0;
                const auto& plane = perspectivePlanes[index % 2];
                const double boundary = -radius + (static_cast<int>(index % 7) - 3) * 0.0001;
                center[0] = (boundary - plane[2] * center[2] - plane[3]) / plane[0];
            }
            candidates.push_back(makeCullingItem(identity, center, radius));
        }
        if (!require(computeCull(candidates, perspectivePlanes, {}, result),
                     "Perspective broad-phase GPU dispatch failed")) {
            exitCode = 1;
        } else {
            for (std::size_t index = 0; index < candidates.size(); ++index) {
                if (!cullingFrustumMayIntersect(candidates[index], perspectivePlanes) &&
                    !require(result[index] == 0,
                             "Perspective broad phase rejected a GPU-visible candidate")) {
                    exitCode = 1;
                    break;
                }
            }
        }
    }
    {
        std::vector<CullingItem> lodItems;
        std::mt19937 lodRandom(612);
        std::uniform_real_distribution<float> distance(0.0F, 350.0F);
        for (std::size_t index = 0; index < 4096; ++index) {
            const float value = index < 18
                                    ? boundaries[(index / 6) % boundaries.size()] +
                                          static_cast<float>(static_cast<int>(index % 6) - 3) *
                                              0.0001F
                                    : distance(lodRandom);
            for (std::uint32_t level = 0; level < 3; ++level) {
                auto candidate = makeCullingItem(
                    identity, {value * 0.36, value * 0.48, value * 0.8}, 0);
                candidate.metadata = {CullLod, level, 0, 3};
                lodItems.push_back(candidate);
            }
        }
        if (!require(computeCull(lodItems, planes, boundaries, result),
                     "LOD broad-phase GPU dispatch failed")) {
            exitCode = 1;
        } else {
            std::size_t rejected = 0;
            for (std::size_t index = 0; index < lodItems.size(); ++index) {
                if (!cullingLodMayMatch(lodItems[index], boundaries)) {
                    ++rejected;
                    if (!require(result[index] == 0,
                                 "LOD broad phase rejected a GPU-selected candidate")) {
                        exitCode = 1;
                        break;
                    }
                }
            }
            if (!require(rejected > lodItems.size() / 2,
                         "LOD broad phase did not remove enough irrelevant levels")) exitCode = 1;
        }
    }
    {
        const std::vector<float> unusualBoundaries = {
            50, 25, 100, 25, 25, 250, std::numeric_limits<float>::quiet_NaN(), 25, 250};
        std::vector<CullingItem> candidates;
        for (std::uint32_t offset : {0U, 3U, 6U}) {
            for (double distance : {0.0, 24.999, 25.0, 25.001, 49.999, 50.0, 50.001, 300.0}) {
                for (std::uint32_t level = 0; level < 4; ++level) {
                    auto candidate = makeCullingItem(identity, {0, 0, -distance}, 0);
                    candidate.metadata = {CullLod, level, offset, 3};
                    candidates.push_back(candidate);
                }
            }
        }
        if (!require(computeCull(candidates, planes, unusualBoundaries, result),
                     "Unusual LOD GPU dispatch failed")) {
            exitCode = 1;
        } else {
            for (std::size_t index = 0; index < candidates.size(); ++index) {
                if (!cullingLodMayMatch(candidates[index], unusualBoundaries) &&
                    !require(result[index] == 0,
                             "LOD broad phase rejected an unusual GPU-selected level")) {
                    exitCode = 1;
                    break;
                }
            }
        }
    }
    std::vector<CullingItem> sizeItems;
    for (double radius : {0.0, 0.05, 0.1, 0.5, 1.0, 10.0}) {
        auto candidate = makeCullingItem(identity, {0, 0, -10}, radius);
        candidate.metadata[0] = CullProjectedSize;
        candidate.draw[0] = 3;
        sizeItems.push_back(candidate);
    }
    const auto checkSizeReadback = [&](double threshold) {
        const auto settings = makeCullingProjectedSize(screenProjection, threshold);
        if (!computeCull(sizeItems, planes, {}, result, settings) ||
            result.size() != sizeItems.size()) return false;
        for (std::size_t i = 0; i < sizeItems.size(); ++i) {
            if ((result[i] != 0) != cullingItemVisible(sizeItems[i], planes, {}, settings))
                return false;
        }
        return true;
    };
    if (!require(checkSizeReadback(0.0) && checkSizeReadback(0.02) &&
                 checkSizeReadback(0.10) && checkSizeReadback(0.0),
                 "Projected-size GPU readback mismatch or stale view settings")) exitCode = 1;
    if (!require(computeCull(sizeItems, planes, {}, result,
                             makeCullingProjectedSize(screenProjection, 0.02)) &&
                     result == std::vector<std::uint32_t>({1, 0, 0, 1, 1, 1}),
                 "Main-view GPU projected-size decisions incorrect") ||
        !require(computeCull(sizeItems, planes, {}, result,
                             makeCullingProjectedSize(screenProjection, 0.10)) &&
                     result == std::vector<std::uint32_t>({1, 0, 0, 0, 1, 1}),
                 "Mirror-view GPU projected-size decisions incorrect")) exitCode = 1;
    std::mt19937 sizeRandom(9721);
    std::uniform_real_distribution<float> radiusDistribution(0.001F, 1000.0F);
    std::uniform_real_distribution<float> offsetDistribution(-0.0002F, 0.0002F);
    for (const float threshold : {0.0F, 0.02F, 0.5F, 1.0F}) {
        std::vector<CullingItem> candidates;
        for (int i = 0; i < 2048; ++i) {
            const float radius = radiusDistribution(sizeRandom);
            const float effectiveThreshold = threshold > 0.0F ? threshold : 0.5F;
            const float depth = radius + radius / effectiveThreshold *
                                             (1.0F + offsetDistribution(sizeRandom));
            auto candidate = makeCullingItem(identity, {0, 0, -depth}, radius);
            candidate.metadata[0] = CullProjectedSize;
            candidate.draw[0] = 3;
            candidates.push_back(candidate);
        }
        const auto settings = makeCullingProjectedSize(screenProjection, threshold);
        if (!require(computeCull(candidates, planes, {}, result, settings),
                     "Projected-size boundary GPU dispatch failed")) {
            exitCode = 1;
            break;
        }
        for (std::size_t i = 0; i < candidates.size(); ++i) {
            if (!cullingProjectedSizeMayPass(candidates[i], settings) &&
                !require(result[i] == 0,
                         "Projected-size broad phase rejected a GPU-visible candidate")) {
                exitCode = 1;
                break;
            }
        }
    }
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
        const char* vertex = "#version 330 core\nuniform float depth;void main(){vec2 p[3]=vec2[3](vec2(-1,-1),vec2(1,-1),vec2(0,1));gl_Position=vec4(p[gl_VertexID%3],depth,1);}";
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
            for (int state : {0, 1, 2, 3, 4, 5, 6, 0}) {
                auto candidate = makeCullingItem(identity, {state == 1 ? 10.0 : 0.0, 0, 0}, 0);
                candidate.draw[0] = state == 2 ? 0 : 3;
                if (state == 3) candidate.metadata[0] |= CullReject;
                CullingProjectedSize settings;
                if (state >= 4) {
                    candidate = makeCullingItem(identity, {0, 0, -10}, 0.5);
                    candidate.metadata[0] = CullProjectedSize;
                    candidate.draw[0] = 3;
                    settings = makeCullingProjectedSize(
                        screenProjection, state == 4 ? 0.0 : state == 5 ? 0.02 : 0.10);
                }
                if (!require(batch.dispatch(std::span(&candidate, 1), planes, {}, settings), "Indirect dispatch failed without readback")) exitCode = 1;
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
                if (!require((pixel[0] > 200) == (state == 0 || state == 4 || state == 5),
                             "Indirect raster visibility/count mismatch")) exitCode = 1;
            }
        }
        // TEST-ONLY downloads verify every command word and partial workgroup.
        pglGetBufferSubData = readback;
        for (double threshold : {0.0, 0.02, 0.10}) {
            const auto settings = makeCullingProjectedSize(screenProjection, threshold);
            if (!require(batch.dispatch(sizeItems, planes, {}, settings),
                         "Projected-size indirect dispatch failed")) {
                exitCode = 1;
                continue;
            }
            pglMemoryBarrier(0x0200);
            pglBindBuffer(0x8F3F, batch.commandBuffer());
            std::vector<std::uint32_t> commands(sizeItems.size() * 5);
            readback(0x8F3F, 0,
                     static_cast<std::ptrdiff_t>(commands.size() * sizeof(std::uint32_t)),
                     commands.data());
            for (std::size_t i = 0; i < sizeItems.size(); ++i) {
                if (!require(commands[i * 5 + 1] ==
                                 static_cast<std::uint32_t>(
                                     cullingItemVisible(sizeItems[i], planes, {}, settings)),
                             "Projected-size indirect command mismatch")) exitCode = 1;
            }
        }
        for (bool proxyCommands : {false, true}) {
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
                if (!require(batch.dispatch(items, planes, boundaries, {}, proxyCommands), "Indirect parity dispatch failed")) exitCode = 1;
                if (!count) continue;
                pglMemoryBarrier(0x0200);
                pglBindBuffer(0x8F3F, batch.commandBuffer());
                const std::size_t stride = proxyCommands ? 10 : 5;
                std::vector<std::uint32_t> commands(count * stride);
                readback(0x8F3F, 0, static_cast<std::ptrdiff_t>(commands.size() * sizeof(std::uint32_t)), commands.data());
                for (std::size_t i = 0; i < count; ++i) {
                    const auto offset = i * stride;
                    if (!require(commands[offset] == items[i].draw[0] &&
                        commands[offset + 1] == static_cast<std::uint32_t>(cullingItemVisible(items[i], planes, boundaries)) &&
                        commands[offset + 2] == 0 && commands[offset + 3] == 0 && commands[offset + 4] == 0 &&
                        (!proxyCommands || (commands[offset + 5] == 36 &&
                        commands[offset + 6] == commands[offset + 1] &&
                        commands[offset + 7] == 0 && commands[offset + 8] == 0 && commands[offset + 9] == 0)),
                        "Indirect command/reference mismatch")) { exitCode = 1; break; }
                }
            }
        }
        // A selected compute-rejected candidate must not produce proxy samples.
        // Conditional rendering also preserves partially visible candidates.
        {
            std::array<CullingItem, 2> candidates = {
                makeCullingItem(identity, {10, 0, 0}, 0),
                makeCullingItem(identity, {0, 0, 0}, 0)};
            for (auto& candidate : candidates) candidate.draw[0] = 3;
            if (!require(batch.dispatch(candidates, planes, {}, {}, true), "Proxy dispatch failed")) exitCode = 1;
            GLuint query = 0;
            pglGenQueries(1, &query);
            const GLint depthUniform = pglGetUniformLocation(render, "depth");
            glEnable(GL_DEPTH_TEST);
            glDepthFunc(GL_LESS);
            for (int scenario : {0, 1, 2, 3}) {
                glEnable(GL_DEPTH_TEST);
                glDepthMask(GL_TRUE);
                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
                if (scenario == 1 || scenario == 2) {
                    if (scenario == 2) {
                        glEnable(GL_SCISSOR_TEST);
                        glScissor(0, 0, 32, 64);
                    }
                    pglUniform1f(depthUniform, -0.5F);
                    glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
                    glDrawArrays(GL_TRIANGLES, 0, 3);
                    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
                    glDisable(GL_SCISSOR_TEST);
                }
                pglUniform1f(depthUniform, 0.0F);
                glDepthMask(GL_FALSE);
                glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
                glDepthFunc(GL_LEQUAL);
                IndirectCullingScope scope;
                batch.select(scenario == 3 ? 0 : 1);
                pglBeginQuery(0x8C2F /* GL_ANY_SAMPLES_PASSED */, query);
                if (!require(drawSelectedIndirectProxy(), "Indirect proxy submission missing")) exitCode = 1;
                pglEndQuery(0x8C2F);
                pglBeginConditionalRender(query, 0x8E13 /* GL_QUERY_WAIT */);
                glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
                glDepthMask(GL_TRUE);
                glDepthFunc(GL_LESS);
                glDisable(GL_DEPTH_TEST);
                glDrawArrays(GL_TRIANGLES, 0, 3);
                pglEndConditionalRender();
                std::array<GLubyte, 4> pixel = {};
                glReadPixels(32, 32, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, pixel.data());
                if (!require((pixel[0] > 200) == (scenario == 0 || scenario == 2),
                             "Occlusion or indirect proxy visibility incorrect")) exitCode = 1;
            }
            pglDeleteQueries(1, &query);
            glDisable(GL_DEPTH_TEST);
        }
        // Separate queued views retain their own commands even after another
        // dispatch reuses the service's input buffers.
        {
            IndirectCullingScope scope;
            IndirectCullingBatch visibleView, hiddenView;
            auto candidate = makeCullingItem(identity, {0, 0, 0}, 0);
            candidate.draw[0] = 3;
            if (!require(visibleView.dispatch(std::span(&candidate, 1), planes, {}, {}, true), "Visible queued view failed")) exitCode = 1;
            candidate.eyeSphere[0] = 10;
            if (!require(hiddenView.dispatch(std::span(&candidate, 1), planes, {}), "Hidden queued view failed")) exitCode = 1;
            glClear(GL_COLOR_BUFFER_BIT);
            visibleView.select(0);
            {
                IndirectCullingScope nested;
                hiddenView.select(0);
                if (!require(!drawSelectedIndirectProxy(), "Non-proxy view exposed a proxy command")) exitCode = 1;
            }
            if (!require(drawSelectedIndirectProxy(), "Nested view lost proxy command layout")) exitCode = 1;
            glClear(GL_COLOR_BUFFER_BIT);
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
        {
            IndirectCullingBatch reusedView;
            IndirectCullingScope scope;
            auto candidate = makeCullingItem(identity, {0, 0, 0}, 0);
            candidate.draw[0] = 3;
            for (bool proxyCommands : {false, true}) {
                for (bool drawProxy : {false, true}) {
                    if (drawProxy && !proxyCommands) continue;
                    for (bool firstVisible : {false, true}) {
                        glClear(GL_COLOR_BUFFER_BIT);
                        glEnable(GL_SCISSOR_TEST);
                        glScissor(0, 0, 32, 64);
                        candidate.eyeSphere[0] = firstVisible ? 0 : 10;
                        if (!require(reusedView.dispatch(std::span(&candidate, 1), planes, {},
                                                         {}, proxyCommands),
                                     "First reusable view dispatch failed")) exitCode = 1;
                        reusedView.select(0);
                        if (!require(drawProxy ? drawSelectedIndirectProxy()
                                               : drawSelectedIndirect(false),
                                     "First reusable view submission failed")) exitCode = 1;
                        glScissor(32, 0, 32, 64);
                        candidate.eyeSphere[0] = firstVisible ? 10 : 0;
                        if (!require(reusedView.dispatch(std::span(&candidate, 1), planes, {},
                                                         {}, proxyCommands),
                                     "Second reusable view dispatch failed")) exitCode = 1;
                        reusedView.select(0);
                        if (!require(drawProxy ? drawSelectedIndirectProxy()
                                               : drawSelectedIndirect(false),
                                     "Second reusable view submission failed")) exitCode = 1;
                        glDisable(GL_SCISSOR_TEST);
                        std::array<GLubyte, 4> left = {}, right = {};
                        glReadPixels(24, 32, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, left.data());
                        glReadPixels(40, 32, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, right.data());
                        if (!require((left[0] > 200) == firstVisible &&
                                         (right[0] > 200) != firstVisible,
                                     "Reused buffer overwrote a queued view's commands")) exitCode = 1;
                    }
                }
            }
        }
        {
            IndirectCullingBatch reusedView;
            IndirectCullingScope scope;
            auto candidate = makeCullingItem(identity, {30, 0, 0}, 0);
            candidate.metadata = {CullLod, 0, 0, 2};
            candidate.draw[0] = 3;
            const std::array<float, 2> visibleLevels = {35, 100};
            const std::array<float, 2> hiddenLevels = {25, 100};
            for (bool visibleFirst : {false, true}) {
                glDisable(GL_SCISSOR_TEST);
                glClear(GL_COLOR_BUFFER_BIT);
                glEnable(GL_SCISSOR_TEST);
                glScissor(0, 0, 32, 64);
                if (!require(reusedView.dispatch(std::span(&candidate, 1), planes,
                                                 visibleFirst ? visibleLevels : hiddenLevels),
                             "First queued LOD dispatch failed")) exitCode = 1;
                reusedView.select(0);
                drawSelectedIndirect(false);
                glScissor(32, 0, 32, 64);
                if (!require(reusedView.dispatch(std::span(&candidate, 1), planes,
                                                 visibleFirst ? hiddenLevels : visibleLevels),
                             "Second queued LOD dispatch failed")) exitCode = 1;
                reusedView.select(0);
                drawSelectedIndirect(false);
                glDisable(GL_SCISSOR_TEST);
                std::array<GLubyte, 4> left = {}, right = {};
                glReadPixels(24, 32, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, left.data());
                glReadPixels(40, 32, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, right.data());
                if (!require((left[0] > 200) == visibleFirst &&
                                 (right[0] > 200) != visibleFirst,
                             "Changed LOD boundaries overwrote queued dispatches")) exitCode = 1;
            }
        }
        {
            IndirectCullingBatch outerBatch, innerBatch;
            auto candidate = makeCullingItem(identity, {0, 0, 0}, 0);
            candidate.draw[0] = 3;
            if (!require(outerBatch.dispatch(std::span(&candidate, 1), planes, {}, {}, true) &&
                             innerBatch.dispatch(std::span(&candidate, 1), planes, {}),
                         "Binding-cache setup failed")) exitCode = 1;
            pglBindBuffer(0x8F3F, indices);
            originalBindBuffer = pglBindBuffer;
            indirectBindCount = 0;
            pglBindBuffer = recordBufferBind;
            {
                IndirectCullingScope outer;
                outerBatch.select(0);
                drawSelectedIndirect(false);
                drawSelectedIndirectProxy();
                drawSelectedIndirect(false);
                if (!require(indirectBindCount == 1, "Repeated commands rebound the same buffer"))
                    exitCode = 1;
                {
                    IndirectCullingScope inner;
                    innerBatch.select(0);
                    drawSelectedIndirect(false);
                }
                drawSelectedIndirect(false);
                if (!require(indirectBindCount == 3, "Nested indirect binding was not restored"))
                    exitCode = 1;
            }
            pglBindBuffer = originalBindBuffer;
            glGetIntegerv(0x8F43, &binding);
            if (!require(indirectBindCount == 4 && binding == static_cast<GLint>(indices),
                         "Outer indirect binding was not restored")) exitCode = 1;
        }
        {
            GLuint storageSentinel = 0;
            pglGenBuffers(1, &storageSentinel);
            pglBindBuffer(0x90D2, storageSentinel);
            pglBufferData(0x90D2, 64, nullptr, 0x88E0);
            pglBindBufferBase(0x90D2, 0, storageSentinel);
            originalGetIntegeri = pglGetIntegeri;
            indexedStorageQueries = 0;
            pglGetIntegeri = recordGetIntegeri;
            auto candidate = makeCullingItem(identity, {0, 0, 0}, 0);
            candidate.draw[0] = 3;
            IndirectCullingBatch reusable;
            {
                IndirectCullingScope outer;
                if (!require(reusable.dispatch(std::span(&candidate, 1), planes, {}),
                             "Scoped storage dispatch failed")) exitCode = 1;
                originalGetIntegeri(0x90D3, 0, &binding);
                if (!require(indexedStorageQueries == 3 &&
                                 static_cast<GLuint>(binding) == storageSentinel,
                             "First storage snapshot or restoration failed")) exitCode = 1;
                glGetIntegerv(0x90D3, &binding);
                if (!require(static_cast<GLuint>(binding) == storageSentinel,
                             "Generic storage binding was not restored")) exitCode = 1;
                {
                    IndirectCullingScope nested;
                    if (!require(reusable.dispatch(std::span(&candidate, 1), planes, {}),
                                 "Nested storage dispatch failed")) exitCode = 1;
                }
                originalGetIntegeri(0x90D3, 0, &binding);
                if (!require(indexedStorageQueries == 3 &&
                                 static_cast<GLuint>(binding) == storageSentinel,
                             "Nested dispatch did not reuse restored storage state")) exitCode = 1;
                deleteRendererBuffers(1, &storageSentinel);
                if (!require(reusable.dispatch(std::span(&candidate, 1), planes, {}),
                             "Dispatch after deleting a storage binding failed")) exitCode = 1;
                originalGetIntegeri(0x90D3, 0, &binding);
                if (!require(indexedStorageQueries == 6 && binding == 0,
                             "Deleted storage binding was not recaptured")) exitCode = 1;
                glGetIntegerv(0x90D3, &binding);
                if (!require(binding == 0, "Deleted generic binding was not recaptured"))
                    exitCode = 1;
            }
            pglGetIntegeri = originalGetIntegeri;
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
    if (!require(initializeCoreRenderer(), "Core renderer initialization failed")) return 1;
    initializeComputeCulling(false, true);
    {
        setProjectionMatrix(identity);
        setModelViewMatrix(identity);
        glViewport(0, 0, 64, 64);
        glDisable(GL_DEPTH_TEST);
        glDisable(GL_CULL_FACE);
        glDisable(GL_BLEND);
        std::vector<PrimitiveVertex> proxy;
        for (int i = 0; i < 12; ++i) {
            proxy.push_back({-1, -1, 0, 1, 1, 1});
            proxy.push_back({1, -1, 0, 1, 1, 1});
            proxy.push_back({0, 1, 0, 1, 1, 1});
        }
        std::array<CullingItem, 2> candidates = {
            makeCullingItem(identity, {10, 0, 0}, 0),
            makeCullingItem(identity, {0, 0, 0}, 0)};
        IndirectCullingBatch batch;
        if (!require(batch.dispatch(candidates, planes, {}, {}, true), "Core proxy dispatch failed")) exitCode = 1;
        StaticPrimitiveBuffer cache;
        for (std::size_t slot : {1U, 0U, 1U}) {
            glClear(GL_COLOR_BUFFER_BIT);
            IndirectCullingScope scope;
            batch.select(slot);
            if (!require(drawStaticPrimitives(cache, proxy, GL_TRIANGLES, 1.0F, 0, true),
                         "Core proxy submission failed")) exitCode = 1;
            std::array<GLubyte, 4> pixel = {};
            glReadPixels(32, 32, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, pixel.data());
            if (!require((pixel[0] > 200) == (slot == 1),
                         "Core proxy upload or indirect visibility incorrect")) exitCode = 1;
        }
        using IsBufferProc = GLboolean(APIENTRY*)(GLuint);
        const auto isBuffer = reinterpret_cast<IsBufferProc>(glfwGetProcAddress("glIsBuffer"));
        if (!require(isBuffer != nullptr, "Buffer lifetime entry point unavailable")) exitCode = 1;
        if (isBuffer) {
            const GLuint previousBuffer = cache.buffer;
            if (!require(isBuffer(previousBuffer) == GL_TRUE,
                         "Core proxy buffer was not allocated")) exitCode = 1;
            releaseStaticPrimitiveBuffer(cache);
            if (!require(cache.buffer == 0 && isBuffer(previousBuffer) == GL_FALSE,
                         "Core proxy buffer was not released")) exitCode = 1;
            releaseStaticPrimitiveBuffer(cache);
            if (!require(drawStaticPrimitives(cache, proxy, GL_TRIANGLES),
                         "Released core proxy buffer did not recreate")) exitCode = 1;
            releaseStaticPrimitiveBuffer(cache);
        }
        if (!require(glGetError() == GL_NO_ERROR, "Core proxy GL error")) exitCode = 1;
    }
    shutdownCoreRenderer();
    const auto setBackend = [](const char* value) {
#ifdef _WIN32
        return _putenv_s("OPENBUS_CULLING_BACKEND", value) == 0;
#else
        return setenv("OPENBUS_CULLING_BACKEND", value, 1) == 0;
#endif
    };
    for (const char* backend : {"compute-readback", "compute-indirect"}) {
        if (!require(setBackend(backend) && initializeCoreRenderer(),
                     "Backend selection renderer initialization failed")) {
            exitCode = 1;
            continue;
        }
        const bool expectIndirect = std::string_view(backend) == "compute-indirect";
        if (!require(!computeCullingAvailable() && indirectCullingAvailable() == expectIndirect,
                     "Application selected an unexpected culling backend")) exitCode = 1;
        shutdownCoreRenderer();
    }
    glfwDestroyWindow(window);
    glfwTerminate();
    return exitCode;
}