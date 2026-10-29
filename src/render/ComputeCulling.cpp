#include "render/ComputeCulling.h"

#include "core/Logger.h"
#include "render/OpenGLFunctions.h"
#include "core/PerfTrace.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

namespace openbus::rendering {
namespace {
constexpr GLenum storageBuffer = 0x90D2;
constexpr GLenum storageBinding = 0x90D3;
constexpr GLenum streamDraw = 0x88E0;
constexpr float visibilityRadiusSquared = 10000.0F * 10000.0F;
constexpr float frustumMargin = 0.001F;
GLuint program = 0;
std::array<GLuint, 3> buffers = {};
GLint planesLocation = -1;
GLint countLocation = -1;
GLint indirectLocation = -1;
GLint proxyCommandsLocation = -1;
GLint projectionScaleLocation = -1;
GLint minimumFractionLocation = -1;
std::size_t maxItems = 0;
bool indirectMode = false;
std::vector<float> uploadedLodBoundaries;
bool lodBufferInitialized = false;
GLuint selectedBuffer = 0;
std::size_t selectedSlot = 0;
bool selectedOcclusionProxies = false;
constexpr GLenum drawIndirectBuffer = 0x8F3F;
GLuint trackedIndirectBinding = 0;
bool indirectBindingTracked = false;
bool storageBindingTracked = false;
bool storageBindingsCaptured = false;
GLint savedStorageBinding = 0;
std::array<GLint, 3> savedIndexedStorageBindings = {};

void bindIndirectBuffer(GLuint buffer) {
    if (!indirectBindingTracked || trackedIndirectBinding != buffer) {
        pglBindBuffer(drawIndirectBuffer, buffer);
        if (indirectBindingTracked)
            trackedIndirectBinding = buffer;
    }
}

const char* source = R"GLSL(
#version 430 core
layout(local_size_x = 64) in;
struct Item { vec4 eyeSphere; vec4 distanceBounds; uvec4 metadata; uvec4 draw; };
layout(std430, binding = 0) readonly buffer Inputs { Item items[]; };
layout(std430, binding = 1) writeonly buffer Outputs { uint visible[]; };
layout(std430, binding = 2) readonly buffer LodData { float boundaries[]; };
uniform vec4 uPlanes[6];
uniform int uCount;
uniform int uIndirect;
uniform int uProxyCommands;
uniform float uProjectionScale;
uniform float uMinimumFraction;
void main() {
    uint index = gl_GlobalInvocationID.x;
    if (index >= uint(uCount)) return;
    Item item = items[index];
    bool keep = (item.metadata.x & 8u) == 0u;
    if (keep && (item.metadata.x & 1u) != 0u) {
        for (int p = 0; p < 6; ++p) {
            if (dot(uPlanes[p].xyz, item.eyeSphere.xyz) + uPlanes[p].w <
                -item.eyeSphere.w - 0.001) {
                keep = false;
                break;
            }
        }
    }
    if (keep && (item.metadata.x & 2u) != 0u) {
        vec2 delta = max(max(item.distanceBounds.xy, -item.distanceBounds.zw), vec2(0));
        if (dot(delta, delta) > 100000000.0) keep = false;
    }
    if (keep && (item.metadata.x & 4u) != 0u && item.metadata.w != 0u) {
        float distanceToEye = length(item.eyeSphere.xyz);
        uint selected = item.metadata.w - 1u;
        for (uint level = 0u; level < item.metadata.w; ++level) {
            if (distanceToEye <= boundaries[item.metadata.z + level]) {
                selected = level;
                break;
            }
        }
        if (selected != item.metadata.y) keep = false;
    }
    if (keep && (item.metadata.x & 16u) != 0u) {
        float depth = -item.eyeSphere.z;
        float radius = item.eyeSphere.w;
        if (!isnan(depth) && !isinf(depth) && !isnan(radius) && !isinf(radius) &&
            radius > 0.0 && depth > radius && !isnan(uProjectionScale) &&
            !isinf(uProjectionScale) && uProjectionScale > 0.0 &&
            !isnan(uMinimumFraction) && !isinf(uMinimumFraction) && uMinimumFraction > 0.0) {
            float fraction = uProjectionScale * radius / (depth - radius);
            if (!isnan(fraction) && !isinf(fraction) &&
                fraction < uMinimumFraction * (1.0 - 0.00001)) keep = false;
        }
    }
    if (uIndirect == 0) {
        visible[index] = keep ? 1u : 0u;
    } else {
        // Five scalar words: indexed command layout, also valid as the first
        // four words of an arrays command (all offsets/base instances are zero).
        uint offset = index * (uProxyCommands != 0 ? 10u : 5u);
        visible[offset] = item.draw.x;
        visible[offset + 1u] = keep ? 1u : 0u;
        visible[offset + 2u] = 0u;
        visible[offset + 3u] = 0u;
        visible[offset + 4u] = 0u;
        if (uProxyCommands != 0) {
            visible[offset + 5u] = 36u;
            visible[offset + 6u] = keep ? 1u : 0u;
            visible[offset + 7u] = 0u;
            visible[offset + 8u] = 0u;
            visible[offset + 9u] = 0u;
        }
    }
}
)GLSL";
} // namespace

CullingProjectedSize makeCullingProjectedSize(const std::array<double, 16>& projection,
                                              double minimumFraction) {
    const double scale = std::abs(projection[5]);
    if (!std::isfinite(scale) || scale <= 0.0 || !std::isfinite(minimumFraction) ||
        minimumFraction <= 0.0 || scale > std::numeric_limits<float>::max() ||
        minimumFraction > std::numeric_limits<float>::max()) {
        return {};
    }
    return {static_cast<float>(scale), static_cast<float>(minimumFraction)};
}

CullingPlanes makeCullingPlanes(const std::array<double, 16>& projection) {
    CullingPlanes result = {};
    for (std::size_t p = 0; p < 6; ++p) {
        const std::size_t axis = p / 2;
        const double sign = p % 2 == 0 ? 1.0 : -1.0;
        std::array<double, 4> plane = {};
        for (std::size_t c = 0; c < 4; ++c) {
            plane[c] = projection[3 + c * 4] + sign * projection[axis + c * 4];
        }
        const double length = std::hypot(plane[0], plane[1], plane[2]);
        for (std::size_t c = 0; c < 4; ++c) {
            result[p][c] = static_cast<float>(length > 0.0 ? plane[c] / length : plane[c]);
        }
    }
    return result;
}

CullingItem makeCullingItem(const std::array<double, 16>& modelView,
                            const std::array<double, 3>& center, double radius) {
    CullingItem item;
    for (std::size_t row = 0; row < 3; ++row) {
        item.eyeSphere[row] =
            static_cast<float>(modelView[row] * center[0] + modelView[row + 4] * center[1] +
                               modelView[row + 8] * center[2] + modelView[row + 12]);
    }
    item.eyeSphere[3] = static_cast<float>(radius);
    item.metadata[0] = CullFrustum;
    return item;
}

bool sphereBelowMinimumViewportFraction(const std::array<double, 16>& modelView,
                                        const std::array<double, 16>& projection,
                                        const std::array<double, 3>& center, double radius,
                                        double minimumFraction) {
    if (!std::isfinite(minimumFraction) || minimumFraction <= 0.0 || !std::isfinite(radius) ||
        radius <= 0.0) {
        return false;
    }

    const double eyeDepth = -(modelView[2] * center[0] + modelView[6] * center[1] +
                              modelView[10] * center[2] + modelView[14]);
    const double verticalProjectionScale = std::abs(projection[5]);
    if (!std::isfinite(eyeDepth) || !std::isfinite(verticalProjectionScale) ||
        verticalProjectionScale <= 0.0 || eyeDepth <= radius) {
        return false;
    }

    const double projectedFraction = verticalProjectionScale * radius / (eyeDepth - radius);
    return std::isfinite(projectedFraction) && projectedFraction < minimumFraction;
}

std::uint32_t appendCullingLodBoundaries(std::vector<float>& boundaries,
                                         std::span<const double> thresholds) {
    const auto offset = static_cast<std::uint32_t>(boundaries.size());
    for (const double threshold : thresholds) {
        boundaries.push_back(static_cast<float>(25.0 / std::max(threshold, 0.001)));
    }
    return offset;
}

bool cullingLodMayMatch(const CullingItem& item, std::span<const float> boundaries) {
    const std::size_t offset = item.metadata[2];
    const std::size_t count = item.metadata[3];
    if ((item.metadata[0] & CullLod) == 0 || count == 0 || offset > boundaries.size() ||
        count > boundaries.size() - offset)
        return true;
    const std::size_t desired = item.metadata[1];
    if (desired >= count)
        return false;
    const double distance =
        std::hypot(static_cast<double>(item.eyeSphere[0]), static_cast<double>(item.eyeSphere[1]),
                   static_cast<double>(item.eyeSphere[2]));
    // Float length evaluation can overflow on the GPU even when double hypot
    // remains finite. Uncertain values must be left to the compute shader.
    if (!std::isfinite(distance) || distance > 1.0e18)
        return true;
    const double margin = 0.001 + distance * 0.00001;
    const double minimumDistance = std::max(0.0, distance - margin);
    const double maximumDistance = distance + margin;
    for (std::size_t level = 0; level < desired; ++level) {
        const float boundary = boundaries[offset + level];
        if (!std::isfinite(boundary))
            return true;
        if (maximumDistance <= boundary)
            return false;
    }
    if (desired + 1 < count) {
        const float boundary = boundaries[offset + desired];
        if (!std::isfinite(boundary))
            return true;
        if (minimumDistance > boundary)
            return false;
    }
    return true;
}

bool cullingFrustumMayIntersect(const CullingItem& item, const CullingPlanes& planes) {
    if ((item.metadata[0] & CullFrustum) == 0)
        return true;
    const auto& sphere = item.eyeSphere;
    if (!std::isfinite(sphere[0]) || !std::isfinite(sphere[1]) || !std::isfinite(sphere[2]) ||
        !std::isfinite(sphere[3]) || sphere[3] < 0.0F)
        return true;
    for (const auto& plane : planes) {
        double distance = plane[3];
        double magnitude = std::abs(static_cast<double>(plane[3])) + sphere[3];
        for (std::size_t axis = 0; axis < 3; ++axis) {
            const double term = static_cast<double>(plane[axis]) * sphere[axis];
            distance += term;
            magnitude += std::abs(term);
        }
        if (!std::isfinite(distance) || !std::isfinite(magnitude) || magnitude > 1.0e30)
            continue;
        const double margin = 0.002 + magnitude * 0.00001;
        if (distance < -static_cast<double>(sphere[3]) - margin)
            return false;
    }
    return true;
}

bool cullingItemVisible(const CullingItem& item, const CullingPlanes& planes,
                        std::span<const float> lodBoundaries, CullingProjectedSize projectedSize) {
    if ((item.metadata[0] & CullLod) != 0 &&
        (item.metadata[2] > lodBoundaries.size() ||
         item.metadata[3] > lodBoundaries.size() - item.metadata[2]))
        return false;
    if ((item.metadata[0] & CullReject) != 0)
        return false;
    const auto& sphere = item.eyeSphere;
    if ((item.metadata[0] & CullProjectedSize) != 0) {
        const float depth = -sphere[2];
        const float radius = sphere[3];
        if (std::isfinite(depth) && std::isfinite(radius) && radius > 0.0F && depth > radius &&
            std::isfinite(projectedSize.projectionScale) && projectedSize.projectionScale > 0.0F &&
            std::isfinite(projectedSize.minimumFraction) && projectedSize.minimumFraction > 0.0F) {
            const float fraction = projectedSize.projectionScale * radius / (depth - radius);
            if (std::isfinite(fraction) &&
                fraction < projectedSize.minimumFraction * (1.0F - 0.00001F))
                return false;
        }
    }
    if ((item.metadata[0] & CullFrustum) != 0) {
        for (const auto& plane : planes) {
            const float distance =
                plane[0] * sphere[0] + plane[1] * sphere[1] + plane[2] * sphere[2] + plane[3];
            if (distance < -sphere[3] - frustumMargin)
                return false;
        }
    }
    if ((item.metadata[0] & CullMapDistance) != 0) {
        const auto& b = item.distanceBounds;
        const float dx = std::max({b[0], -b[2], 0.0F});
        const float dy = std::max({b[1], -b[3], 0.0F});
        if (dx * dx + dy * dy > visibilityRadiusSquared)
            return false;
    }
    if ((item.metadata[0] & CullLod) != 0 && item.metadata[3] != 0) {
        const float distance =
            std::sqrt(sphere[0] * sphere[0] + sphere[1] * sphere[1] + sphere[2] * sphere[2]);
        std::uint32_t selected = item.metadata[3] - 1;
        for (std::uint32_t level = 0; level < item.metadata[3]; ++level) {
            if (distance <= lodBoundaries[item.metadata[2] + level]) {
                selected = level;
                break;
            }
        }
        if (selected != item.metadata[1])
            return false;
    }
    return true;
}

void shutdownComputeCulling() {
    if (program != 0)
        pglDeleteProgram(program);
    if (buffers[0] != 0 || buffers[1] != 0 || buffers[2] != 0) {
        for (GLuint buffer : buffers)
            invalidateCullingStorageBinding(buffer);
        pglDeleteBuffers(static_cast<GLsizei>(buffers.size()), buffers.data());
    }
    program = 0;
    buffers = {};
    maxItems = 0;
    indirectMode = false;
    uploadedLodBoundaries.clear();
    lodBufferInitialized = false;
    selectedBuffer = 0;
}

void initializeComputeCulling(bool enableReadback, bool enableIndirect) {
    shutdownComputeCulling();
    if (!enableReadback && !enableIndirect) {
        gameLog.Log("Visibility culling: CPU; optional GPU backend "
                    "OPENBUS_CULLING_BACKEND=compute-indirect");
        return;
    }
    GLint major = 0, minor = 0;
    glGetIntegerv(0x821B /* GL_MAJOR_VERSION */, &major);
    glGetIntegerv(0x821C /* GL_MINOR_VERSION */, &minor);
    if ((major < 4 || (major == 4 && minor < 3)) || !pglDispatchCompute || !pglMemoryBarrier ||
        !pglBindBufferBase || !pglGetIntegeri || (enableReadback && !pglGetBufferSubData) ||
        (enableIndirect && (!pglDrawArraysIndirect || !pglDrawElementsIndirect))) {
        gameLog.Warning(
            "Compute culling unavailable; using CPU visibility tests (OpenGL 4.3 required)");
        return;
    }
    const GLuint shader = pglCreateShader(0x91B9 /* GL_COMPUTE_SHADER */);
    if (shader == 0) {
        gameLog.Warning("Could not create compute culling shader; using CPU visibility tests");
        return;
    }
    pglShaderSource(shader, 1, &source, nullptr);
    pglCompileShader(shader);
    GLint success = GL_FALSE;
    pglGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (success == GL_FALSE) {
        std::array<char, 4096> log = {};
        pglGetShaderInfoLog(shader, static_cast<GLsizei>(log.size()), nullptr, log.data());
        gameLog.Warning(std::string("Compute culling compilation failed; CPU fallback: ") +
                        log.data());
        pglDeleteShader(shader);
        return;
    }
    program = pglCreateProgram();
    if (program != 0) {
        pglAttachShader(program, shader);
        pglLinkProgram(program);
        pglGetProgramiv(program, GL_LINK_STATUS, &success);
    } else {
        success = GL_FALSE;
    }
    pglDeleteShader(shader);
    if (success == GL_FALSE) {
        gameLog.Warning("Compute culling link failed; using CPU visibility tests");
        shutdownComputeCulling();
        return;
    }
    pglGenBuffers(static_cast<GLsizei>(buffers.size()), buffers.data());
    if (std::any_of(buffers.begin(), buffers.end(), [](GLuint buffer) { return buffer == 0; })) {
        gameLog.Warning("Could not allocate compute culling buffers; using CPU visibility tests");
        shutdownComputeCulling();
        return;
    }
    // GL 4.3 guarantees >=65535 groups in X and >=128 MiB per storage block.
    // Use the guaranteed minima rather than accepting an unchecked dispatch.
    maxItems = std::min<std::size_t>(65535U * 64U, (128U * 1024U * 1024U) / sizeof(CullingItem));
    planesLocation = pglGetUniformLocation(program, "uPlanes");
    countLocation = pglGetUniformLocation(program, "uCount");
    indirectLocation = pglGetUniformLocation(program, "uIndirect");
    proxyCommandsLocation = pglGetUniformLocation(program, "uProxyCommands");
    projectionScaleLocation = pglGetUniformLocation(program, "uProjectionScale");
    minimumFractionLocation = pglGetUniformLocation(program, "uMinimumFraction");
    if (planesLocation < 0 || countLocation < 0 || indirectLocation < 0 ||
        proxyCommandsLocation < 0 || projectionScaleLocation < 0 || minimumFractionLocation < 0) {
        gameLog.Warning("Compute culling uniforms unavailable; using CPU visibility tests");
        shutdownComputeCulling();
        return;
    }
    indirectMode = enableIndirect;
    gameLog.Log(indirectMode
                    ? "Compute indirect culling enabled: GPU commands, no visibility readback"
                    : "Experimental compute culling enabled: synchronous current-view readback may "
                      "reduce frame rate");
}

bool computeCullingAvailable() {
    return program != 0 && !indirectMode;
}
bool indirectCullingAvailable() {
    return program != 0 && indirectMode;
}

static bool dispatchCulling(std::span<const CullingItem> items, const CullingPlanes& planes,
                            std::span<const float> lodBoundaries, GLuint output,
                            std::vector<std::uint32_t>* visibility,
                            CullingProjectedSize projectedSize, bool occlusionProxies) {
    if (program == 0)
        return false;
    if (items.size() > maxItems || lodBoundaries.size() > (128U * 1024U * 1024U) / sizeof(float)) {
        static bool loggedLimit = false;
        if (!loggedLimit) {
            gameLog.Warning(
                "Compute culling input exceeds supported limits; using CPU for this view");
            loggedLimit = true;
        }
        return false;
    }
    if (items.empty())
        return true;
    TraceScope trace("render", visibility ? "ComputeCulling::dispatchAndReadback"
                                          : "ComputeCulling::dispatchIndirect");
    for (const auto& item : items) {
        if ((item.metadata[0] & CullLod) != 0 &&
            (item.metadata[2] > lodBoundaries.size() ||
             item.metadata[3] > lodBoundaries.size() - item.metadata[2])) {
            static bool loggedInvalidLod = false;
            if (!loggedInvalidLod) {
                gameLog.Warning("Invalid compute culling LOD range; using CPU for this view");
                loggedInvalidLod = true;
            }
            return false;
        }
        if (!visibility &&
            item.draw[0] > static_cast<std::uint32_t>(std::numeric_limits<GLsizei>::max())) {
            static bool loggedInvalidCount = false;
            if (!loggedInvalidCount) {
                gameLog.Warning("Indirect draw count exceeds GLsizei; using CPU for this view");
                loggedInvalidCount = true;
            }
            return false;
        }
    }
    // Errors from earlier texture uploads/draws must not be attributed to this
    // dispatch and permanently disable an otherwise functioning compute path.
    bool earlierError = false;
    while (glGetError() != GL_NO_ERROR)
        earlierError = true;
    static bool loggedEarlierError = false;
    if (earlierError && !loggedEarlierError) {
        gameLog.Warning(
            "OpenGL error predates compute culling; cleared before dispatch validation");
        loggedEarlierError = true;
    }
    GLint previousProgram = 0, previousBuffer = 0;
    std::array<GLint, 3> previousBindings = {};
    glGetIntegerv(0x8B8D /* GL_CURRENT_PROGRAM */, &previousProgram);
    if (storageBindingTracked && storageBindingsCaptured) {
        previousBuffer = savedStorageBinding;
        previousBindings = savedIndexedStorageBindings;
    } else {
        glGetIntegerv(storageBinding, &previousBuffer);
        for (GLuint i = 0; i < 3; ++i)
            pglGetIntegeri(storageBinding, i, &previousBindings[i]);
        if (storageBindingTracked) {
            savedStorageBinding = previousBuffer;
            savedIndexedStorageBindings = previousBindings;
            storageBindingsCaptured = true;
        }
    }
    pglBindBuffer(storageBuffer, buffers[0]);
    pglBufferData(storageBuffer, static_cast<std::ptrdiff_t>(items.size_bytes()), items.data(),
                  streamDraw);
    pglBindBuffer(storageBuffer, output);
    pglBufferData(storageBuffer,
                  static_cast<std::ptrdiff_t>(items.size() * sizeof(std::uint32_t) *
                                              (visibility         ? 1
                                               : occlusionProxies ? 10
                                                                  : 5)),
                  nullptr, streamDraw);
    const bool uploadLod = !lodBufferInitialized ||
                           !std::equal(lodBoundaries.begin(), lodBoundaries.end(),
                                       uploadedLodBoundaries.begin(), uploadedLodBoundaries.end());
    if (uploadLod) {
        pglBindBuffer(storageBuffer, buffers[2]);
        const float dummy = 0.0F;
        pglBufferData(
            storageBuffer,
            static_cast<std::ptrdiff_t>(std::max(lodBoundaries.size_bytes(), sizeof(float))),
            lodBoundaries.empty() ? &dummy : lodBoundaries.data(), streamDraw);
    }
    pglBindBufferBase(storageBuffer, 0, buffers[0]);
    pglBindBufferBase(storageBuffer, 1, output);
    pglBindBufferBase(storageBuffer, 2, buffers[2]);
    pglUseProgram(program);
    pglUniform4fv(planesLocation, 6, planes[0].data());
    pglUniform1i(countLocation, static_cast<GLint>(items.size()));
    pglUniform1i(indirectLocation, visibility ? 0 : 1);
    pglUniform1i(proxyCommandsLocation, occlusionProxies ? 1 : 0);
    pglUniform1f(projectionScaleLocation, projectedSize.projectionScale);
    pglUniform1f(minimumFractionLocation, projectedSize.minimumFraction);
    pglDispatchCompute(static_cast<GLuint>((items.size() + 63) / 64), 1, 1);
    pglMemoryBarrier(visibility ? 0x0200 /* BUFFER_UPDATE */ : 0x0040 /* COMMAND */);
    // CPU submission needs this view's result for sorting/material resolution.
    // This read blocks; an indirect renderer would remove that synchronization.
    if (visibility) {
        visibility->resize(items.size());
        pglBindBuffer(storageBuffer, output);
        TraceScope readback("render", "ComputeCulling::readback.wait");
        pglGetBufferSubData(storageBuffer, 0,
                            static_cast<std::ptrdiff_t>(visibility->size() * sizeof(std::uint32_t)),
                            visibility->data());
    }
    const bool success = glGetError() == GL_NO_ERROR;
    if (success && uploadLod) {
        uploadedLodBoundaries.assign(lodBoundaries.begin(), lodBoundaries.end());
        lodBufferInitialized = true;
    }
    pglUseProgram(static_cast<GLuint>(previousProgram));
    for (GLuint i = 0; i < 3; ++i)
        pglBindBufferBase(storageBuffer, i, static_cast<GLuint>(previousBindings[i]));
    pglBindBuffer(storageBuffer, static_cast<GLuint>(previousBuffer));
    if (!success) {
        if (visibility)
            visibility->clear();
        gameLog.Warning("Compute culling dispatch/readback failed; disabling compute culling");
        shutdownComputeCulling();
    }
    return success;
}

bool computeCull(std::span<const CullingItem> items, const CullingPlanes& planes,
                 std::span<const float> boundaries, std::vector<std::uint32_t>& visibility,
                 CullingProjectedSize projectedSize) {
    visibility.clear();
    return computeCullingAvailable() && dispatchCulling(items, planes, boundaries, buffers[1],
                                                        &visibility, projectedSize, false);
}

IndirectCullingBatch::~IndirectCullingBatch() {
    if (selectedBuffer == buffer_)
        selectedBuffer = 0;
    if (buffer_) {
        invalidateCullingStorageBinding(buffer_);
        pglDeleteBuffers(1, &buffer_);
        if (indirectBindingTracked && trackedIndirectBinding == buffer_)
            trackedIndirectBinding = 0;
    }
}

bool IndirectCullingBatch::dispatch(std::span<const CullingItem> items, const CullingPlanes& planes,
                                    std::span<const float> boundaries,
                                    CullingProjectedSize projectedSize, bool occlusionProxies) {
    if (selectedBuffer == buffer_)
        selectedBuffer = 0;
    count_ = 0;
    if (!indirectCullingAvailable())
        return false;
    if (!buffer_)
        pglGenBuffers(1, &buffer_);
    if (!buffer_) {
        static bool loggedAllocationFailure = false;
        if (!loggedAllocationFailure) {
            gameLog.Warning("Could not allocate indirect command buffer; using CPU for this view");
            loggedAllocationFailure = true;
        }
        return false;
    }
    if (!dispatchCulling(items, planes, boundaries, buffer_, nullptr, projectedSize,
                         occlusionProxies))
        return false;
    count_ = items.size();
    occlusionProxies_ = occlusionProxies;
    return true;
}

void IndirectCullingBatch::select(std::size_t slot) const {
    selectedBuffer = slot < count_ ? buffer_ : 0;
    selectedSlot = slot;
    selectedOcclusionProxies = occlusionProxies_;
}

IndirectCullingScope::IndirectCullingScope()
    : buffer_(selectedBuffer), slot_(selectedSlot), occlusionProxies_(selectedOcclusionProxies),
      previousBindingTracked_(indirectBindingTracked),
      previousStorageTracked_(storageBindingTracked), active_(indirectCullingAvailable()) {
    if (active_) {
        if (previousBindingTracked_)
            binding_ = static_cast<GLint>(trackedIndirectBinding);
        else
            glGetIntegerv(0x8F43 /* DRAW_INDIRECT_BUFFER_BINDING */, &binding_);
        trackedIndirectBinding = static_cast<GLuint>(binding_);
        indirectBindingTracked = true;
        if (!previousStorageTracked_) {
            storageBindingTracked = true;
            storageBindingsCaptured = false;
        }
    }
    selectedBuffer = 0;
}
IndirectCullingScope::~IndirectCullingScope() {
    selectedBuffer = buffer_;
    selectedSlot = slot_;
    selectedOcclusionProxies = occlusionProxies_;
    if (active_) {
        bindIndirectBuffer(static_cast<GLuint>(binding_));
        indirectBindingTracked = previousBindingTracked_;
        if (!previousStorageTracked_) {
            storageBindingTracked = false;
            storageBindingsCaptured = false;
        }
    }
}

void invalidateCullingStorageBinding(std::uint32_t buffer) {
    if (buffer == 0 || !storageBindingsCaptured)
        return;
    if (savedStorageBinding == static_cast<GLint>(buffer) ||
        std::find(savedIndexedStorageBindings.begin(), savedIndexedStorageBindings.end(),
                  static_cast<GLint>(buffer)) != savedIndexedStorageBindings.end())
        storageBindingsCaptured = false;
}

bool drawSelectedIndirect(bool indexed) {
    if (!selectedBuffer || !indirectCullingAvailable())
        return false;
    bindIndirectBuffer(selectedBuffer);
    const auto offset = reinterpret_cast<const void*>(
        selectedSlot * (selectedOcclusionProxies ? 10 : 5) * sizeof(std::uint32_t));
    if (indexed)
        pglDrawElementsIndirect(GL_TRIANGLES, GL_UNSIGNED_INT, offset);
    else
        pglDrawArraysIndirect(GL_TRIANGLES, offset);
    return true;
}

bool drawSelectedIndirectProxy() {
    if (!selectedBuffer || !selectedOcclusionProxies || !indirectCullingAvailable())
        return false;
    bindIndirectBuffer(selectedBuffer);
    const auto offset =
        reinterpret_cast<const void*>((selectedSlot * 10 + 5) * sizeof(std::uint32_t));
    pglDrawArraysIndirect(GL_TRIANGLES, offset);
    return true;
}
} // namespace openbus::rendering