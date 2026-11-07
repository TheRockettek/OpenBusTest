#include "render/CoreRenderer.h"
#include "render/ComputeCulling.h"
#include "render/MatrixMath.h"
#include "core/Environment.h"

#include "core/Logger.h"
#include "render/OpenGLFunctions.h"
#include "core/PerfTrace.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

namespace openbus::rendering {
namespace {

constexpr GLsizei MODEL_VERTEX_STRIDE = static_cast<GLsizei>(13 * sizeof(float));
constexpr std::size_t MAX_MATERIAL_TEXTURES = 8;
constexpr std::size_t MAX_TEXTURE_UNITS = 16;
std::size_t submittedFrameTriangleCount = 0;
std::size_t submittedReflectionTriangleCount = 0;
bool reflectionTrianglePass = false;

constexpr std::size_t kGpuTimerQueriesPerPass = 32;
constexpr std::size_t kGpuTimerPassCount = 2;
constexpr std::size_t kGpuTimerQueryCount = kGpuTimerQueriesPerPass * kGpuTimerPassCount;
constexpr auto kGpuTimerPollInterval = std::chrono::milliseconds(100);
constexpr auto kGpuTimerSummaryInterval = std::chrono::seconds(2);

struct GpuTimerPassState {
    std::array<bool, kGpuTimerQueriesPerPass> pending = {};
    std::size_t nextQuery = 0;
    std::uint64_t completed = 0;
    std::uint64_t totalNanoseconds = 0;
    std::uint64_t maximumNanoseconds = 0;
    std::uint64_t dropped = 0;
};

struct GpuTimerState {
    bool enabled = false;
    bool frameSampled = false;
    bool scopeActive = false;
    std::uint64_t frameCount = 0;
    int sampleEveryNFrames = 1;
    std::array<GLuint, kGpuTimerQueryCount> queries = {};
    std::array<GpuTimerPassState, kGpuTimerPassCount> passes = {};
    std::size_t activePass = 0;
    std::size_t activeQuery = 0;
    std::chrono::steady_clock::time_point nextPoll = {};
    std::chrono::steady_clock::time_point lastSummary = {};
};

GpuTimerState gpuTimerState;

std::size_t gpuTimerPassIndex(GpuTimingPass pass) {
    return static_cast<std::size_t>(pass);
}

void initializeGpuTiming() {
    const char* setting = openbus::getEnvironment("OPENBUS_GPU_TIMING");
    const std::string_view value = setting != nullptr ? setting : "";
    if (value != "1" && value != "true") {
        return;
    }

    if (pglGenQueries == nullptr || pglDeleteQueries == nullptr || pglBeginQuery == nullptr ||
        pglEndQuery == nullptr || pglGetQueryObjectuiv == nullptr ||
        pglGetQueryObjectui64v == nullptr) {
        gameLog.Warning("GPU timing requested, but OpenGL timer-query functions are unavailable");
        return;
    }

    const char* sampleSetting = openbus::getEnvironment("OPENBUS_GPU_TIMING_SAMPLE_EVERY_N_FRAMES");
    const int sampleEveryNFrames =
        sampleSetting != nullptr ? std::clamp(std::atoi(sampleSetting), 1, 10000) : 1;

    GpuTimerState nextState;
    nextState.sampleEveryNFrames = sampleEveryNFrames;
    pglGenQueries(static_cast<GLsizei>(nextState.queries.size()), nextState.queries.data());
    if (std::any_of(nextState.queries.begin(), nextState.queries.end(),
                    [](GLuint query) { return query == 0; })) {
        if (std::any_of(nextState.queries.begin(), nextState.queries.end(),
                        [](GLuint query) { return query != 0; })) {
            pglDeleteQueries(static_cast<GLsizei>(nextState.queries.size()),
                             nextState.queries.data());
        }
        gameLog.Warning("GPU timing requested, but OpenGL timer-query allocation failed");
        return;
    }

    nextState.enabled = true;
    nextState.lastSummary = std::chrono::steady_clock::now();
    gpuTimerState = nextState;
    gameLog.Log("GPU timer queries enabled; sampling every " + std::to_string(sampleEveryNFrames) +
                " main frame(s)");
}

void shutdownGpuTiming() {
    if (pglDeleteQueries != nullptr &&
        std::any_of(gpuTimerState.queries.begin(), gpuTimerState.queries.end(),
                    [](GLuint query) { return query != 0; })) {
        pglDeleteQueries(static_cast<GLsizei>(gpuTimerState.queries.size()),
                         gpuTimerState.queries.data());
    }
    gpuTimerState = {};
}

bool beginGpuTimerQuery(GpuTimingPass pass) {
    GpuTimerState& state = gpuTimerState;
    const std::size_t passIndex = gpuTimerPassIndex(pass);
    if (!state.enabled || !state.frameSampled || state.scopeActive ||
        passIndex >= state.passes.size()) {
        return false;
    }

    GpuTimerPassState& passState = state.passes[passIndex];
    for (std::size_t offset = 0; offset < passState.pending.size(); ++offset) {
        const std::size_t slot = (passState.nextQuery + offset) % passState.pending.size();
        if (passState.pending[slot]) {
            continue;
        }
        const std::size_t queryIndex = passIndex * kGpuTimerQueriesPerPass + slot;
        pglBeginQuery(GL_TIME_ELAPSED, state.queries[queryIndex]);
        passState.pending[slot] = true;
        passState.nextQuery = (slot + 1) % passState.pending.size();
        state.scopeActive = true;
        state.activePass = passIndex;
        state.activeQuery = slot;
        return true;
    }

    ++passState.dropped;
    return false;
}

void endGpuTimerQuery() {
    GpuTimerState& state = gpuTimerState;
    if (!state.enabled || !state.scopeActive) {
        return;
    }
    pglEndQuery(GL_TIME_ELAPSED);
    state.scopeActive = false;
}

void appendGpuTimerSummary(std::ostringstream& summary, const char* name,
                           GpuTimerPassState& passState) {
    summary << ' ' << name << "_samples=" << passState.completed;
    if (passState.completed > 0) {
        const double averageMilliseconds = static_cast<double>(passState.totalNanoseconds) /
                                           static_cast<double>(passState.completed) / 1'000'000.0;
        const double maximumMilliseconds =
            static_cast<double>(passState.maximumNanoseconds) / 1'000'000.0;
        summary << ' ' << name << "_avg_ms=" << averageMilliseconds << ' ' << name
                << "_max_ms=" << maximumMilliseconds;
    }
    summary << ' ' << name << "_dropped=" << passState.dropped;
    passState.completed = 0;
    passState.totalNanoseconds = 0;
    passState.maximumNanoseconds = 0;
    passState.dropped = 0;
}

void pollGpuTimingQueries() {
    GpuTimerState& state = gpuTimerState;
    if (!state.enabled) {
        return;
    }

    const auto now = std::chrono::steady_clock::now();
    if (state.nextPoll != std::chrono::steady_clock::time_point{} && now < state.nextPoll) {
        return;
    }
    state.nextPoll = now + kGpuTimerPollInterval;

    for (std::size_t passIndex = 0; passIndex < state.passes.size(); ++passIndex) {
        GpuTimerPassState& passState = state.passes[passIndex];
        for (std::size_t slot = 0; slot < passState.pending.size(); ++slot) {
            if (!passState.pending[slot]) {
                continue;
            }
            const std::size_t queryIndex = passIndex * kGpuTimerQueriesPerPass + slot;
            GLuint available = GL_FALSE;
            pglGetQueryObjectuiv(state.queries[queryIndex], GL_QUERY_RESULT_AVAILABLE, &available);
            if (available == GL_FALSE) {
                continue;
            }
            std::uint64_t nanoseconds = 0;
            pglGetQueryObjectui64v(state.queries[queryIndex], GL_QUERY_RESULT, &nanoseconds);
            passState.pending[slot] = false;
            ++passState.completed;
            passState.totalNanoseconds += nanoseconds;
            passState.maximumNanoseconds = std::max(passState.maximumNanoseconds, nanoseconds);
        }
    }

    const double summaryElapsedSeconds =
        std::chrono::duration<double>(now - state.lastSummary).count();
    if (summaryElapsedSeconds < std::chrono::duration<double>(kGpuTimerSummaryInterval).count()) {
        return;
    }
    std::ostringstream summary;
    summary << std::fixed << std::setprecision(3)
            << "GPU_TIMING interval_s=" << summaryElapsedSeconds;
    appendGpuTimerSummary(summary, "main_view", state.passes[0]);
    appendGpuTimerSummary(summary, "reflection", state.passes[1]);
    gameLog.Log(summary.str());
    state.lastSummary = now;
}

std::size_t trianglesForPrimitive(GLenum primitive, std::size_t vertexCount) {
    switch (primitive) {
    case GL_TRIANGLES:
        return vertexCount / 3;
    case GL_TRIANGLE_STRIP:
    case GL_TRIANGLE_FAN:
    case GL_POLYGON:
        return vertexCount >= 3 ? vertexCount - 2 : 0;
    case GL_QUADS:
        return (vertexCount / 4) * 2;
    case GL_QUAD_STRIP:
        return vertexCount >= 4 ? (vertexCount / 2 - 1) * 2 : 0;
    default:
        return 0;
    }
}

void recordTriangleSubmission(std::size_t triangles) {
    if (reflectionTrianglePass) {
        submittedReflectionTriangleCount += triangles;
    } else {
        submittedFrameTriangleCount += triangles;
    }
}

struct Uniforms {
    GLint projection = -1;
    GLint modelView = -1;
    GLint texture = -1;
    GLint textureArray = -1;
    GLint textureScale = -1;
    GLint detailTexture = -1;
    GLint detailTextureScale = -1;
    GLint useTexture = -1;
    GLint textureAlphaOnly = -1;
    GLint useDetailTexture = -1;
    GLint useTextureArray = -1;
    GLint color = -1;
    GLint alphaMode = -1;
    GLint visibilityFade = -1;
    GLint cullBackFaces = -1;
    GLint lightmap = -1;
    GLint nightmap = -1;
    GLint transmap = -1;
    GLint transmapScale = -1;
    GLint flipTransmapY = -1;
    GLint specularColor = -1;
    GLint emissionColor = -1;
    GLint specularPower = -1;
    GLint snowCoverage = -1;
    GLint bumpmap = -1;
    GLint freeTexture = -1;
    GLint textTexture = -1;
    GLint useLightmap = -1;
    GLint useNightmap = -1;
    GLint useTransmap = -1;
    GLint useBumpmap = -1;
    GLint useFreeTexture = -1;
    GLint useTextTexture = -1;
    GLint weatherLitDynamicTexture = -1;
    GLint whiteAmbient = -1;
    GLint useInteriorLight = -1;
    GLint interiorLightCount = -1;
    std::array<GLint, MAX_INTERIOR_LIGHTS> interiorLightViewPositions = {};
    std::array<GLint, MAX_INTERIOR_LIGHTS> interiorLightColors = {};
    std::array<GLint, MAX_INTERIOR_LIGHTS> interiorLightStrengths = {};
    GLint lightmapStrength = -1;
    GLint nightmapStrength = -1;
    GLint bumpmapStrength = -1;
    GLint interiorLightStrength = -1;
    GLint interiorLightColor = -1;
    GLint lightDirection = -1;
    GLint ambientLight = -1;
    GLint diffuseLight = -1;
    GLint spotlightCount = -1;
    GLint mapLightCount = -1;
    GLint mapLightStrength = -1;
    GLint fogDensity = -1;
    GLint fogColor = -1;
    GLint weatherBrightness = -1;
    GLint texcoordOffset = -1;
    GLint flipTextureY = -1;
    GLint humanGaitEnabled = -1;
    GLint humanSkinEnabled = -1;
    GLint humanBones = -1;
    GLint humanGaitPhase = -1;
    GLint humanGaitWeight = -1;
    GLint humanGaitHeight = -1;
    GLint humanUpperArmBeta = -1;
    GLint humanArmSwing = -1;
    GLint humanHipSway = -1;
    GLint humanWaistAnimation = -1;
    GLint humanShoulderXFraction = -1;
    GLint humanShoulderZFraction = -1;
    GLint humanHipXFraction = -1;
    GLint humanHipZFraction = -1;
    GLint humanKneeXFraction = -1;
    GLint humanKneeYFraction = -1;
    GLint humanKneeZFraction = -1;
    GLint humanElbowXFraction = -1;
    GLint humanElbowZFraction = -1;
    GLint humanHandXFraction = -1;
    GLint humanHandZFraction = -1;
    GLint humanFingerXFraction = -1;
    GLint humanFingerZFraction = -1;
    GLint humanNeckYFraction = -1;
    GLint humanNeckZFraction = -1;
    GLint humanWaistYFraction = -1;
    GLint humanWaistZFraction = -1;
    std::array<GLint, MAX_MATERIAL_TEXTURES> materialFlipTextureY = {};
    GLint useMaterialColors = -1;
    GLint materialColors = -1;
    GLint useMaterialTextures = -1;
    std::array<GLint, MAX_MATERIAL_TEXTURES> materialTextures = {};
    GLint environment = -1;
    GLint environmentAlpha = -1;
    GLint useEnvironment = -1;
};

struct alignas(16) GpuVehicleSpotlightData {
    std::array<std::array<float, 4>, MAX_VEHICLE_SPOTLIGHTS> positionRange = {};
    std::array<std::array<float, 4>, MAX_VEHICLE_SPOTLIGHTS> directionInner = {};
    std::array<std::array<float, 4>, MAX_VEHICLE_SPOTLIGHTS> colorOuter = {};
};

struct alignas(16) GpuMapLightData {
    std::array<std::array<float, 4>, MAX_MAP_LIGHTS> positionRadius = {};
    std::array<std::array<float, 4>, MAX_MAP_LIGHTS> color = {};
};

static_assert(sizeof(GpuVehicleSpotlightData) == MAX_VEHICLE_SPOTLIGHTS * 48);
static_assert(sizeof(GpuMapLightData) == MAX_MAP_LIGHTS * 32);

struct ModelUniformState {
    bool valid = false;
    bool textured = false;
    bool textureAlphaOnly = false;
    bool textureArray = false;
    bool useDetailTexture = false;
    bool useLightmap = false;
    bool useNightmap = false;
    bool useTransmap = false;
    bool flipTransmapY = false;
    bool useBumpmap = false;
    bool useEnvironmentMap = false;
    bool useFreeTexture = false;
    bool useTextTexture = false;
    bool weatherLitDynamicTexture = false;
    bool whiteAmbient = false;
    bool useInteriorLight = false;
    bool flipTextureY = false;
    bool useMaterialColors = false;
    const void* materialColorData = nullptr;
    GLsizei materialColorCount = 0;
    bool useMaterialTextures = false;
    const void* materialTextureData = nullptr;
    GLsizei materialTextureCount = 0;
    int alphaMode = 0;
    float lightmapStrength = 0.0f;
    float nightmapStrength = 0.0f;
    float bumpmapStrength = 0.0f;
    float environmentStrength = 0.0f;
    std::array<float, 2> textureScale = {1.0F, 1.0F};
    std::array<float, 2> detailTextureScale = {1.0F, 1.0F};
    std::array<float, 2> transmapScale = {1.0F, 1.0F};
    std::array<float, 3> specularColor = {};
    std::array<float, 3> emissionColor = {};
    float specularPower = 8.0F;
    float snowCoverage = 0.0F;
    int interiorLightCount = 0;
    std::array<std::array<float, 3>, MAX_INTERIOR_LIGHTS> interiorLightViewPositions = {};
    std::array<std::array<float, 3>, MAX_INTERIOR_LIGHTS> interiorLightColors = {};
    std::array<float, MAX_INTERIOR_LIGHTS> interiorLightStrengths = {};
    float texcoordOffsetX = 0.0f;
    float texcoordOffsetY = 0.0f;
    std::array<float, 4> color = {};
};

GLuint modelProgram = 0;
GLuint materialProgram = 0;
GLuint environmentProgram = 0;
GLuint primitiveProgram = 0;
GLuint textureQuadProgram = 0;
GLuint cloudProgram = 0;
GLuint precipitationProgram = 0;
GLuint precipitationVertexArray = 0;
GLuint modelVertexArray = 0;
GLuint primitiveVertexArray = 0;
GLuint primitiveBuffer = 0;
GLuint vehicleSpotlightBuffer = 0;
GLuint mapLightBuffer = 0;
constexpr GLuint vehicleSpotlightBinding = 0;
constexpr GLuint mapLightBinding = 1;
std::vector<GLuint> staticPrimitiveBuffers;
std::uint64_t rendererGeneration = 0;
GLuint textureQuadVertexArray = 0;
GLuint textureQuadBuffer = 0;
Uniforms modelUniforms;
Uniforms materialUniforms;
Uniforms environmentUniforms;
Uniforms primitiveUniforms;
GLint textureQuadRect = -1;
GLint textureQuadTexture = -1;
GLint textureQuadFlipTextureY = -1;
GLint cloudView = -1;
GLint cloudProjectionScale = -1;
GLint cloudOrigin = -1;
GLint cloudHeight = -1;
GLint cloudStyle = -1;
GLint cloudBrightness = -1;
GLint cloudBackground = -1;
GLint precipitationView = -1;
GLint precipitationProjection = -1;
GLint precipitationEye = -1;
GLint precipitationOffset = -1;
GLint precipitationWind = -1;
GLint precipitationType = -1;
GLint precipitationBrightness = -1;
GLuint currentProgram = 0;
struct FogUniforms {
    GLuint program = 0;
    GLint density = -1;
    GLint layerHeight = -1;
    GLint cameraHeight = -1;
    GLint worldUp = -1;
};
std::array<FogUniforms, 5> fogUniforms;
GLuint currentVertexArray = 0;
GLuint currentArrayBuffer = 0;
thread_local GLenum currentTextureUnit = GL_TEXTURE0;
thread_local std::array<GLuint, MAX_TEXTURE_UNITS> boundTexture2D = {};
thread_local std::array<GLuint, MAX_TEXTURE_UNITS> boundTextureArray = {};
thread_local bool wrapParametersValid = false;
thread_local GLuint wrapParametersTexture = 0;
thread_local GLenum wrapParametersTarget = 0;
thread_local GLint wrapParametersS = 0;
thread_local GLint wrapParametersT = 0;
GLuint matrixProgram = 0;
Matrix4 cachedProjection = {};
Matrix4 cachedModelView = {};
ModelUniformState modelUniformState;
struct MaterialUniformState {
    bool valid = false;
    const void* colorData = nullptr;
    GLsizei colorCount = 0;
};
MaterialUniformState materialUniformState;

const char* modelVertexShader = R"GLSL(
#version 330 core
layout(location = 0) in vec3 aPosition;
layout(location = 1) in vec3 aTexCoord;
layout(location = 2) in vec3 aNormal;
layout(location = 3) in vec4 aHumanWeights0;
layout(location = 4) in vec4 aHumanWeights1;
layout(location = 5) in vec4 aHumanWeights2;
layout(location = 6) in vec4 aHumanWeights3;
uniform mat4 uProjection;
uniform mat4 uModelView;
uniform vec2 uTexcoordOffset;
uniform bool uFlipModelTextureY;
uniform bool uHumanGaitEnabled;
uniform bool uHumanSkinEnabled;
uniform mat4 uHumanBones[13];
uniform float uHumanGaitPhase;
uniform float uHumanGaitWeight;
uniform float uHumanGaitHeight;
uniform float uHumanUpperArmBeta;
uniform float uHumanArmSwing;
uniform float uHumanHipSway;
uniform float uHumanWaistAnimation;
uniform float uHumanShoulderXFraction;
uniform float uHumanShoulderZFraction;
uniform float uHumanHipXFraction;
uniform float uHumanHipZFraction;
uniform float uHumanKneeXFraction;
uniform float uHumanKneeYFraction;
uniform float uHumanKneeZFraction;
uniform float uHumanElbowXFraction;
uniform float uHumanElbowZFraction;
uniform float uHumanHandXFraction;
uniform float uHumanHandZFraction;
uniform float uHumanFingerXFraction;
uniform float uHumanFingerZFraction;
uniform float uHumanNeckYFraction;
uniform float uHumanNeckZFraction;
uniform float uHumanWaistYFraction;
uniform float uHumanWaistZFraction;
out vec3 vTexCoord;
out vec3 vNormal;
out vec3 vViewPosition;
void main() {
    vec3 position = aPosition;
    vec3 normal = aNormal;
    if (uHumanSkinEnabled) {
        float weights[13] = float[13](
            aHumanWeights0.x, aHumanWeights0.y, aHumanWeights0.z, aHumanWeights0.w,
            aHumanWeights1.x, aHumanWeights1.y, aHumanWeights1.z, aHumanWeights1.w,
            aHumanWeights2.x, aHumanWeights2.y, aHumanWeights2.z, aHumanWeights2.w,
            aHumanWeights3.x);
        vec3 skinnedPosition = vec3(0.0);
        vec3 skinnedNormal = vec3(0.0);
        float totalWeight = 0.0;
        for (int i = 0; i < 13; ++i) {
            if (weights[i] > 0.0) {
                skinnedPosition += (uHumanBones[i] * vec4(aPosition, 1.0)).xyz * weights[i];
                skinnedNormal += (mat3(uHumanBones[i]) * aNormal) * weights[i];
                totalWeight += weights[i];
            }
        }
        if (totalWeight > 0.0) {
            position = skinnedPosition / totalWeight;
            normal = skinnedNormal / totalWeight;
        }
    }
    if (uHumanGaitEnabled && uHumanGaitHeight > 0.0) {
        const float pi = 3.14159265;
        float heightFraction = clamp(position.z / uHumanGaitHeight, 0.0, 1.0);
        float side = position.x < 0.0 ? -1.0 : 1.0;
        float lateralWeight = smoothstep(0.025, 0.09, abs(position.x));
        float legWeight = (1.0 - smoothstep(0.36, 0.58, heightFraction)) * lateralWeight;
        float armWeight = smoothstep(0.62, 0.70, heightFraction) *
                          (1.0 - smoothstep(0.88, 0.96, heightFraction)) *
                          smoothstep(0.045, 0.11, abs(position.x));
        float sidePhase = uHumanGaitPhase + (side < 0.0 ? pi : 0.0);
        float legSwing = sin(sidePhase) * uHumanGaitWeight;
        position.y += legSwing * 0.12 * legWeight;
        position.z += max(0.0, cos(sidePhase)) * 0.045 * legWeight * uHumanGaitWeight;
        float kneeX = side * uHumanGaitHeight * uHumanKneeXFraction;
        float kneeY = uHumanGaitHeight * uHumanKneeYFraction;
        float kneeZ = uHumanGaitHeight * uHumanKneeZFraction;
        float lowerLegWeight =
            (1.0 - smoothstep(kneeZ - uHumanGaitHeight * 0.035,
                              kneeZ + uHumanGaitHeight * 0.035, position.z)) *
            smoothstep(abs(kneeX) - uHumanGaitHeight * 0.045,
                       abs(kneeX) + uHumanGaitHeight * 0.045, abs(position.x));
        float kneeAngle = -max(0.0, cos(sidePhase)) * 0.28 * uHumanGaitWeight;
        vec2 lowerLegOffset = vec2(position.y - kneeY, position.z - kneeZ);
        vec2 flexedLowerLeg =
            vec2(cos(kneeAngle) * lowerLegOffset.x - sin(kneeAngle) * lowerLegOffset.y,
                 sin(kneeAngle) * lowerLegOffset.x + cos(kneeAngle) * lowerLegOffset.y);
        position.y = mix(position.y, kneeY + flexedLowerLeg.x, lowerLegWeight);
        position.z = mix(position.z, kneeZ + flexedLowerLeg.y, lowerLegWeight);
        float hipHeight = uHumanGaitHeight * uHumanHipZFraction;
        float hipWeight = smoothstep(hipHeight - uHumanGaitHeight * 0.22,
                                     hipHeight - uHumanGaitHeight * 0.08, position.z) *
                          (1.0 - smoothstep(hipHeight + uHumanGaitHeight * 0.08,
                                            hipHeight + uHumanGaitHeight * 0.24, position.z));
        float hipX = side * uHumanGaitHeight * uHumanHipXFraction;
        float hipAngle = sin(uHumanGaitPhase) * 0.12 * uHumanGaitWeight * uHumanHipSway;
        vec2 hipOffset = vec2(position.x - hipX, position.z - hipHeight);
        vec2 swungHipOffset =
            vec2(cos(hipAngle) * hipOffset.x + sin(hipAngle) * hipOffset.y,
                 -sin(hipAngle) * hipOffset.x + cos(hipAngle) * hipOffset.y);
        position.x = mix(position.x, hipX + swungHipOffset.x, hipWeight);
        position.z = mix(position.z, hipHeight + swungHipOffset.y, hipWeight);
        float armAngle =
            (radians(uHumanUpperArmBeta) +
             sin(sidePhase) * 0.12 * uHumanGaitWeight * uHumanArmSwing) *
            armWeight;
        float shoulderX = side * uHumanGaitHeight * uHumanShoulderXFraction;
        float shoulderZ = uHumanGaitHeight * uHumanShoulderZFraction;
        vec2 armOffset = vec2(position.x - shoulderX, position.z - shoulderZ);
        float armCos = cos(side * armAngle);
        float armSin = sin(side * armAngle);
        vec2 loweredArm = vec2(armCos * armOffset.x + armSin * armOffset.y,
                               -armSin * armOffset.x + armCos * armOffset.y);
        position.x = mix(position.x, shoulderX + loweredArm.x, armWeight);
        position.z = mix(position.z, shoulderZ + loweredArm.y, armWeight);
        float elbowX = side * uHumanGaitHeight * uHumanElbowXFraction;
        float elbowZ = uHumanGaitHeight * uHumanElbowZFraction;
        vec2 elbowOffset = vec2(elbowX - shoulderX, elbowZ - shoulderZ);
        vec2 loweredElbow =
            vec2(armCos * elbowOffset.x + armSin * elbowOffset.y,
                 -armSin * elbowOffset.x + armCos * elbowOffset.y);
        float swungElbowX = shoulderX + loweredElbow.x;
        float swungElbowZ = shoulderZ + loweredElbow.y;
        float forearmWeight =
            armWeight * smoothstep(side * swungElbowX - uHumanGaitHeight * 0.035,
                                   side * swungElbowX + uHumanGaitHeight * 0.035,
                                   side * position.x);
        float elbowAngle = side * sin(uHumanGaitPhase) * 0.24 * uHumanGaitWeight *
                           uHumanArmSwing;
        vec2 forearmOffset = vec2(position.x - swungElbowX, position.z - swungElbowZ);
        vec2 flexedForearm =
            vec2(cos(elbowAngle) * forearmOffset.x + sin(elbowAngle) * forearmOffset.y,
                 -sin(elbowAngle) * forearmOffset.x + cos(elbowAngle) * forearmOffset.y);
        position.x = mix(position.x, swungElbowX + flexedForearm.x, forearmWeight);
        position.z = mix(position.z, swungElbowZ + flexedForearm.y, forearmWeight);
        float handX = side * uHumanGaitHeight * uHumanHandXFraction;
        float handZ = uHumanGaitHeight * uHumanHandZFraction;
        vec2 handUpperOffset = vec2(handX - shoulderX, handZ - shoulderZ);
        vec2 handLowerOffset =
            vec2(armCos * handUpperOffset.x + armSin * handUpperOffset.y,
                 -armSin * handUpperOffset.x + armCos * handUpperOffset.y);
        float swungHandUpperX = shoulderX + handLowerOffset.x;
        float swungHandUpperZ = shoulderZ + handLowerOffset.y;
        vec2 handElbowOffset =
            vec2(swungHandUpperX - swungElbowX, swungHandUpperZ - swungElbowZ);
        vec2 swungHandOffset =
            vec2(cos(elbowAngle) * handElbowOffset.x + sin(elbowAngle) * handElbowOffset.y,
                 -sin(elbowAngle) * handElbowOffset.x + cos(elbowAngle) * handElbowOffset.y);
        float swungHandX = swungElbowX + swungHandOffset.x;
        float swungHandZ = swungElbowZ + swungHandOffset.y;
        float handWeight =
            armWeight * smoothstep(side * swungHandX - uHumanGaitHeight * 0.025,
                                   side * swungHandX + uHumanGaitHeight * 0.025,
                                   side * position.x);
        float wristAngle = cos(sidePhase) * 0.30 * uHumanGaitWeight * uHumanArmSwing;
        vec2 handOffset = vec2(position.x - swungHandX, position.z - swungHandZ);
        vec2 swungHand =
            vec2(cos(wristAngle) * handOffset.x + sin(wristAngle) * handOffset.y,
                 -sin(wristAngle) * handOffset.x + cos(wristAngle) * handOffset.y);
        position.x = mix(position.x, swungHandX + swungHand.x, handWeight);
        position.z = mix(position.z, swungHandZ + swungHand.y, handWeight);
        float fingerX = side * uHumanGaitHeight * uHumanFingerXFraction;
        float fingerZ = uHumanGaitHeight * uHumanFingerZFraction;
        vec2 fingerUpperOffset = vec2(fingerX - shoulderX, fingerZ - shoulderZ);
        vec2 fingerShoulderOffset =
            vec2(armCos * fingerUpperOffset.x + armSin * fingerUpperOffset.y,
                 -armSin * fingerUpperOffset.x + armCos * fingerUpperOffset.y);
        float swungFingerUpperX = shoulderX + fingerShoulderOffset.x;
        float swungFingerUpperZ = shoulderZ + fingerShoulderOffset.y;
        vec2 fingerElbowOffset =
            vec2(swungFingerUpperX - swungElbowX, swungFingerUpperZ - swungElbowZ);
        vec2 swungFingerForearmOffset =
            vec2(cos(elbowAngle) * fingerElbowOffset.x + sin(elbowAngle) * fingerElbowOffset.y,
                 -sin(elbowAngle) * fingerElbowOffset.x + cos(elbowAngle) * fingerElbowOffset.y);
        float swungFingerX = swungElbowX + swungFingerForearmOffset.x;
        float swungFingerZ = swungElbowZ + swungFingerForearmOffset.y;
        vec2 fingerHandOffset = vec2(swungFingerX - swungHandX, swungFingerZ - swungHandZ);
        vec2 swungFingerHandOffset =
            vec2(cos(wristAngle) * fingerHandOffset.x + sin(wristAngle) * fingerHandOffset.y,
                 -sin(wristAngle) * fingerHandOffset.x + cos(wristAngle) * fingerHandOffset.y);
        float swungFingerTipX = swungHandX + swungFingerHandOffset.x;
        float swungFingerTipZ = swungHandZ + swungFingerHandOffset.y;
        float fingerWeight =
            handWeight * smoothstep(side * swungFingerTipX - uHumanGaitHeight * 0.012,
                                    side * swungFingerTipX + uHumanGaitHeight * 0.012,
                                    side * position.x);
        float fingerAngle = sin(uHumanGaitPhase * 2.0) * 0.10 * uHumanGaitWeight *
                            uHumanArmSwing;
        vec2 fingerOffset = vec2(position.x - swungFingerTipX, position.z - swungFingerTipZ);
        vec2 flexedFinger =
            vec2(cos(fingerAngle) * fingerOffset.x + sin(fingerAngle) * fingerOffset.y,
                 -sin(fingerAngle) * fingerOffset.x + cos(fingerAngle) * fingerOffset.y);
        position.x = mix(position.x, swungFingerTipX + flexedFinger.x, fingerWeight);
        position.z = mix(position.z, swungFingerTipZ + flexedFinger.y, fingerWeight);
        float neckY = uHumanGaitHeight * uHumanNeckYFraction;
        float neckZ = uHumanGaitHeight * uHumanNeckZFraction;
        float headWeight = smoothstep(neckZ + uHumanGaitHeight * 0.015,
                                      neckZ + uHumanGaitHeight * 0.065, position.z);
        float headAngle = -sin(uHumanGaitPhase) * 0.045 * uHumanGaitWeight;
        vec2 headOffset = vec2(position.y - neckY, position.z - neckZ);
        vec2 tiltedHead =
            vec2(cos(headAngle) * headOffset.x - sin(headAngle) * headOffset.y,
                 sin(headAngle) * headOffset.x + cos(headAngle) * headOffset.y);
        position.y = mix(position.y, neckY + tiltedHead.x, headWeight);
        position.z = mix(position.z, neckZ + tiltedHead.y, headWeight);
        float waistY = uHumanGaitHeight * uHumanWaistYFraction;
        float waistZ = uHumanGaitHeight * uHumanWaistZFraction;
        float torsoWeight = smoothstep(waistZ - uHumanGaitHeight * 0.035,
                                       waistZ + uHumanGaitHeight * 0.08, position.z);
        float torsoAngle =
            sin(uHumanGaitPhase) * 0.012 * uHumanGaitWeight * uHumanWaistAnimation;
        vec2 torsoOffset = vec2(position.y - waistY, position.z - waistZ);
        vec2 leanedTorso =
            vec2(cos(torsoAngle) * torsoOffset.x - sin(torsoAngle) * torsoOffset.y,
                 sin(torsoAngle) * torsoOffset.x + cos(torsoAngle) * torsoOffset.y);
        position.y = mix(position.y, waistY + leanedTorso.x, torsoWeight);
        position.z = mix(position.z, waistZ + leanedTorso.y, torsoWeight);
        position.y -= sin(sidePhase) * 0.035 * armWeight * uHumanGaitWeight;
        position.z += 0.018 * (0.5 - 0.5 * cos(2.0 * uHumanGaitPhase)) *
                      smoothstep(0.08, 0.28, heightFraction) * uHumanGaitWeight *
                      uHumanWaistAnimation;
    }
    vec4 viewPosition = uModelView * vec4(position, 1.0);
    float textureY = uFlipModelTextureY ? 1.0 - aTexCoord.y : aTexCoord.y;
    vTexCoord = vec3(aTexCoord.x + uTexcoordOffset.x, textureY + uTexcoordOffset.y,
                     aTexCoord.z);
    // Normals must use the inverse-transpose so animated parts with non-uniform
    // scale still receive lighting from the correct surface direction.
    vNormal = mat3(transpose(inverse(uModelView))) * normal;
    vViewPosition = viewPosition.xyz;
    gl_Position = uProjection * viewPosition;
}
)GLSL";

const char* modelFragmentShader = R"GLSL(
#version 330 core
in vec3 vTexCoord;
in vec3 vNormal;
in vec3 vViewPosition;
uniform mat4 uModelView;
uniform vec3 uLightDirection;
uniform float uAmbientLight;
uniform float uDiffuseLight;
uniform vec3 uFogColor;
uniform float uWeatherBrightness;
uniform sampler2D uTexture;
uniform sampler2DArray uTextureArray;
uniform vec2 uTextureScale;
uniform sampler2D uDetailTexture;
uniform vec2 uDetailTextureScale;
uniform sampler2D uLightmap;
uniform sampler2D uNightmap;
uniform sampler2D uTransmap;
uniform vec2 uTransmapScale;
uniform bool uFlipTransmapY;
uniform vec3 uSpecularColor;
uniform vec3 uEmissionColor;
uniform float uSpecularPower;
uniform float uSnowCoverage;
uniform sampler2D uBumpmap;
uniform sampler2D uEnvironment;
uniform sampler2D uFreeTexture;
uniform sampler2D uTextTexture;
uniform bool uUseTexture;
uniform bool uTextureAlphaOnly;
uniform bool uUseDetailTexture;
uniform bool uUseTextureArray;
uniform bool uUseLightmap;
uniform bool uUseNightmap;
uniform bool uUseTransmap;
uniform bool uUseBumpmap;
uniform bool uUseEnvironment;
uniform bool uUseFreeTexture;
uniform bool uUseTextTexture;
uniform bool uWeatherLitDynamicTexture;
uniform bool uWhiteAmbient;
uniform bool uUseInteriorLight;
uniform int uInteriorLightCount;
uniform vec3 uInteriorLightViewPositions[4];
uniform vec3 uInteriorLightColors[4];
uniform float uInteriorLightStrengths[4];
uniform float uLightmapStrength;
uniform float uNightmapStrength;
uniform float uBumpmapStrength;
uniform float uEnvironmentAlpha;
uniform float uInteriorLightStrength;
uniform vec3 uInteriorLightColor;
uniform vec4 uColor;
uniform int uAlphaMode;
uniform float uVisibilityFade;
uniform bool uCullBackFaces;
out vec4 fragmentColor;
void main() {
    // Imported content meshes are one-sided; generated geometry and mirrored
    // instances remain two-sided.
    if (uCullBackFaces && !gl_FrontFacing) {
        discard;
    }
    vec4 color = vec4(1.0, 1.0, 1.0, uColor.a);
    vec2 textureCoordinate = vTexCoord.xy * uTextureScale;
    if (uUseTextTexture) {
        color *= texture(uTextTexture, vTexCoord.xy);
    } else if (uUseFreeTexture) {
        color *= texture(uFreeTexture, textureCoordinate);
    } else if (uUseTextureArray) {
        color *= texture(uTextureArray, vec3(textureCoordinate, vTexCoord.z));
    } else if (uUseTexture) {
        vec4 sampledTexture = texture(uTexture, textureCoordinate);
        if (uTextureAlphaOnly) {
            sampledTexture.rgb = vec3(1.0);
        }
        color *= sampledTexture;
    }
    vec3 ambientReflectance = color.rgb;
    color.rgb *= uColor.rgb;
    // Empty text-atlas pixels must remain transparent and must not occlude other
    // scenery. Surviving glyph pixels can then safely write depth during the
    // blended map-text pass, preventing later scenery from showing through letters.
    if (uUseTextTexture && color.a <= 0.01) {
        discard;
    }
    // OMSI's second [groundtex] image is a high-frequency detail map. Multiplying
    // it over the albedo preserves colored base textures and reveals detail on
    // deliberately white base images such as Cotterell's wh_white.bmp.
    if (uUseDetailTexture) {
        vec3 detail = texture(uDetailTexture, vTexCoord.xy * uDetailTextureScale).rgb;
        color.rgb *= detail;
        ambientReflectance *= detail;
    }
    color.rgb = mix(color.rgb, vec3(0.90, 0.92, 0.94), clamp(uSnowCoverage, 0.0, 1.0));
    ambientReflectance = mix(ambientReflectance, vec3(0.90, 0.92, 0.94),
                             clamp(uSnowCoverage, 0.0, 1.0));
    vec3 diffuseReflectance = color.rgb;
    vec3 unlitColor = color.rgb;
    if (!uUseTextTexture && !uUseFreeTexture) {
        vec3 surfaceNormal = normalize(vNormal);
        vec3 lightDirection = normalize(uLightDirection);
        float diffuse = max(dot(surfaceNormal, lightDirection), 0.0);
        if (uWhiteAmbient) {
            color.rgb = clamp(ambientReflectance * uAmbientLight +
                              color.rgb * uDiffuseLight * diffuse, 0.0, 1.0);
        } else {
            color.rgb *= clamp(uAmbientLight + uDiffuseLight * diffuse, 0.0, 1.0);
        }
    } else if (uWeatherLitDynamicTexture) {
        color.rgb *= uWeatherBrightness;
    }
    if (!uUseTextTexture && uUseLightmap) {
        vec4 lightmap = texture(uLightmap, vTexCoord.xy);
        // A lightmap masks self-illumination of the authored surface. White
        // restores its unlit colour, not a white rectangle; black leaves the
        // normal lighting intact. This also preserves display contrast.
        color.rgb = mix(color.rgb, unlitColor,
                clamp(lightmap.rgb * uLightmapStrength, 0.0, 1.0));
    }
    vec3 surfaceNormal = normalize(vNormal);
    vec3 lightDirection = normalize(uLightDirection);
    vec3 viewDirection = normalize(-vViewPosition);
    vec3 halfDirection = normalize(lightDirection + viewDirection);
    float specular = max(dot(surfaceNormal, lightDirection), 0.0) > 0.0
        ? pow(max(dot(surfaceNormal, halfDirection), 0.0), max(uSpecularPower, 1.0))
        : 0.0;
    // OMSI emissive materials illuminate their texture, not an additive solid
    // color. White emission must preserve red glyphs and black display pixels.
    color.rgb = mix(color.rgb, unlitColor, clamp(uEmissionColor, 0.0, 1.0));
    color.rgb += uSpecularColor * specular * uWeatherBrightness;
    if (!uUseTextTexture && uUseNightmap) {
        vec4 nightmap = texture(uNightmap, vTexCoord.xy);
        // OMSI nightmaps are authored as self-lit overlays: black texels mean
        // "no emission", not an opaque black replacement for the base surface.
        // Derive coverage from the map itself so this works for any material
        // without relying on texture or variable names.
        float nightmapCoverage = max(max(nightmap.r, nightmap.g), nightmap.b);
        color.rgb = mix(color.rgb, nightmap.rgb * uColor.rgb,
                        uNightmapStrength * nightmapCoverage);
    }
    if (!uUseTextTexture && uUseTransmap) {
        vec2 transmapCoordinate = vTexCoord.xy * uTransmapScale;
        if (uFlipTransmapY) {
            transmapCoordinate.y = 1.0 - transmapCoordinate.y;
        }
        color.a = uColor.a * texture(uTransmap, transmapCoordinate).a;
    }
    if (!uUseTextTexture && uUseBumpmap) {
        vec3 surfaceNormal = normalize(vNormal);
        vec3 positionDerivativeX = dFdx(vViewPosition);
        vec3 positionDerivativeY = dFdy(vViewPosition);
        vec2 coordinateDerivativeX = dFdx(vTexCoord.xy);
        vec2 coordinateDerivativeY = dFdy(vTexCoord.xy);
        float coordinateDeterminant = coordinateDerivativeX.x * coordinateDerivativeY.y -
                                      coordinateDerivativeX.y * coordinateDerivativeY.x;
        if (abs(coordinateDeterminant) > 0.00001) {
            vec3 tangent = (positionDerivativeX * coordinateDerivativeY.y -
                            positionDerivativeY * coordinateDerivativeX.y) /
                           coordinateDeterminant;
            vec3 bitangent = (-positionDerivativeX * coordinateDerivativeY.x +
                              positionDerivativeY * coordinateDerivativeX.x) /
                             coordinateDeterminant;
            if (length(tangent) > 0.00001 && length(bitangent) > 0.00001) {
                tangent = normalize(tangent);
                bitangent = normalize(bitangent);
                vec3 mapNormal = texture(uBumpmap, vTexCoord.xy).xyz * 2.0 - 1.0;
                float strength = clamp(uBumpmapStrength, 0.0, 1.0);
                mapNormal.xy *= strength;
                surfaceNormal = normalize(tangent * mapNormal.x + bitangent * mapNormal.y +
                                           surfaceNormal * mapNormal.z);
                vec3 lightDirection = normalize(uLightDirection);
                float bumpLighting = 0.65 +
                                     0.35 * max(dot(surfaceNormal, lightDirection), 0.0);
                color.rgb *= mix(1.0, bumpLighting, strength);
            }
        }
    }
    if (!uUseTextTexture && uUseInteriorLight) {
        vec3 surfaceNormal = normalize(vNormal);
        for (int index = 0; index < 4; ++index) {
            if (index >= uInteriorLightCount) {
                break;
            }
            vec3 lightVector = uInteriorLightViewPositions[index] - vViewPosition;
            float distanceToLight = length(lightVector);
            if (distanceToLight <= 0.0001) {
                continue;
            }
            vec3 lightDirection = lightVector / distanceToLight;
            float diffuse = max(dot(surfaceNormal, lightDirection), 0.0);
            float attenuation = 1.0 / (1.0 + 0.12 * distanceToLight * distanceToLight);
            // OMSI's authored intensity values are relative (commonly 1.0–3.5),
            // not photometric units. Scale them into a stable LDR diffuse range.
            float illumination = attenuation * diffuse *
                                 max(uInteriorLightStrengths[index], 0.0) * 0.25;
            color.rgb += diffuseReflectance * uInteriorLightColors[index] * illumination;
        }
        color.rgb = clamp(color.rgb, 0.0, 1.0);
    }
    if (!uUseTextTexture || uWeatherLitDynamicTexture) {
        color.rgb += diffuseReflectance * vehicleSpotlightLighting(vViewPosition, normalize(vNormal));
        color.rgb += diffuseReflectance * mapLightLighting(vViewPosition, normalize(vNormal));
    }
    if (!uUseTextTexture && uUseEnvironment) {
        vec3 reflection = reflect(-viewDirection, normalize(vNormal));
        float denominator = 2.0 * sqrt(max(reflection.z + 1.0, 0.0001));
        vec2 coordinates = reflection.xy / denominator + vec2(0.5);
        color.rgb = mix(color.rgb, texture(uEnvironment, coordinates).rgb * uWeatherBrightness,
                        clamp(uEnvironmentAlpha, 0.0, 1.0));
    }
    if (uFogDensity > 0.0) {
        float transmission = weatherFogTransmission(vViewPosition);
        color.rgb = mix(uFogColor, color.rgb, transmission);
    }
    color.a *= clamp(uVisibilityFade, 0.0, 1.0);
    if (uAlphaMode == 1 && color.a < 0.5) {
        discard;
    }
    if (uAlphaMode == 3 && color.a < 0.99) {
        discard;
    }
    fragmentColor = color;
}
)GLSL";

const char* materialFragmentShader = R"GLSL(
#version 330 core
in vec3 vTexCoord;
in vec3 vNormal;
in vec3 vViewPosition;
uniform vec4 uMaterialColors[128];
uniform sampler2D uMaterialTextures[8];
uniform bool uFlipTextureY[8];
uniform vec3 uLightDirection;
uniform float uAmbientLight;
uniform float uDiffuseLight;
uniform vec3 uFogColor;
uniform float uVisibilityFade;
out vec4 fragmentColor;
void main() {
    int materialIndex = clamp(int(vTexCoord.z + 0.5), 0, 127);
    int textureIndex = clamp(materialIndex, 0, 7);
    vec2 textureCoord = uFlipTextureY[textureIndex]
                            ? vec2(vTexCoord.x, 1.0 - vTexCoord.y)
                            : vTexCoord.xy;
    vec4 color = uMaterialColors[materialIndex];
    if (textureIndex == 0) {
        color *= texture(uMaterialTextures[0], textureCoord);
    } else if (textureIndex == 1) {
        color *= texture(uMaterialTextures[1], textureCoord);
    } else if (textureIndex == 2) {
        color *= texture(uMaterialTextures[2], textureCoord);
    } else if (textureIndex == 3) {
        color *= texture(uMaterialTextures[3], textureCoord);
    } else if (textureIndex == 4) {
        color *= texture(uMaterialTextures[4], textureCoord);
    } else if (textureIndex == 5) {
        color *= texture(uMaterialTextures[5], textureCoord);
    } else if (textureIndex == 6) {
        color *= texture(uMaterialTextures[6], textureCoord);
    } else {
        color *= texture(uMaterialTextures[7], textureCoord);
    }
    vec3 surfaceNormal = normalize(vNormal);
    float diffuse = max(dot(surfaceNormal, normalize(uLightDirection)), 0.0);
    vec3 reflectance = color.rgb;
    color.rgb *= clamp(uAmbientLight + uDiffuseLight * diffuse, 0.0, 1.0);
    color.rgb += reflectance * vehicleSpotlightLighting(vViewPosition, surfaceNormal);
    color.rgb += reflectance * mapLightLighting(vViewPosition, surfaceNormal);
    if (uFogDensity > 0.0) {
        float transmission = weatherFogTransmission(vViewPosition);
        color.rgb = mix(uFogColor, color.rgb, transmission);
    }
    color.a *= clamp(uVisibilityFade, 0.0, 1.0);
    fragmentColor = color;
}
)GLSL";

const char* environmentFragmentShader = R"GLSL(
#version 330 core
in vec3 vNormal;
in vec3 vViewPosition;
uniform sampler2D uEnvironment;
uniform float uEnvironmentAlpha;
uniform vec3 uFogColor;
uniform float uWeatherBrightness;
out vec4 fragmentColor;
void main() {
    vec3 normal = normalize(vNormal);
    vec3 viewDirection = normalize(-vViewPosition);
    vec3 reflection = reflect(-viewDirection, normal);
    float denominator = 2.0 * sqrt(max(reflection.z + 1.0, 0.0001));
    vec2 coordinates = reflection.xy / denominator + vec2(0.5);
    float transmission = weatherFogTransmission(vViewPosition);
    vec3 color = mix(uFogColor, texture(uEnvironment, coordinates).rgb * uWeatherBrightness,
                     transmission);
    fragmentColor = vec4(color, uEnvironmentAlpha);
}
)GLSL";

const char* primitiveVertexShader = R"GLSL(
#version 330 core
layout(location = 0) in vec3 aPosition;
layout(location = 1) in vec3 aColor;
uniform mat4 uProjection;
uniform mat4 uModelView;
out vec3 vColor;
void main() {
    vColor = aColor;
    gl_Position = uProjection * uModelView * vec4(aPosition, 1.0);
}
)GLSL";

const char* primitiveFragmentShader = R"GLSL(
#version 330 core
in vec3 vColor;
out vec4 fragmentColor;
void main() {
    fragmentColor = vec4(vColor, 1.0);
}
)GLSL";

const char* textureQuadVertexShader = R"GLSL(
#version 330 core
layout(location = 0) in vec2 aPosition;
layout(location = 1) in vec2 aTexCoord;
uniform vec4 uRect;
uniform bool uFlipTextureY;
out vec2 vTexCoord;
void main() {
    vec2 position = uRect.xy + aPosition * uRect.zw;
    gl_Position = vec4(position, 0.0, 1.0);
    vTexCoord = vec2(aTexCoord.x, uFlipTextureY ? 1.0 - aTexCoord.y : aTexCoord.y);
}
)GLSL";

const char* textureQuadFragmentShader = R"GLSL(
#version 330 core
in vec2 vTexCoord;
uniform sampler2D uTexture;
out vec4 fragmentColor;
void main() {
    fragmentColor = texture(uTexture, vTexCoord);
}
)GLSL";

const char* cloudVertexShader = R"GLSL(
#version 330 core
out vec2 vPosition;
void main() {
    vec2 position = vec2((gl_VertexID << 1) & 2, gl_VertexID & 2);
    vPosition = position * 2.0 - 1.0;
    gl_Position = vec4(vPosition, 0.0, 1.0);
}
)GLSL";

const char* precipitationVertexShader = R"GLSL(
#version 330 core
uniform mat4 uView;
uniform mat4 uProjection;
uniform vec3 uEye;
uniform vec3 uOffset;
uniform vec2 uWind;
uniform int uType;
flat out int vType;
out vec3 vViewPosition;

uint hashBits(uint x) {
    x ^= x >> 16;
    x *= 0x7feb352du;
    x ^= x >> 15;
    x *= 0x846ca68bu;
    return x ^ (x >> 16);
}
float randomValue(uint x) { return float(hashBits(x) & 0x00ffffffu) / 16777216.0; }
void main() {
    uint id = uint(uType == 1 ? gl_VertexID / 2 : gl_VertexID) + 1u;
    vec3 seed = vec3(randomValue(id * 3u), randomValue(id * 3u + 1u),
                     randomValue(id * 3u + 2u)) * 24.0;
    vec3 position = mod(seed + uOffset - uEye + 12.0, 24.0) - 12.0;
    if (uType == 1 && (gl_VertexID & 1) != 0)
        position += vec3(uWind * 0.025, -0.32);
    vType = uType;
    vViewPosition = mat3(uView) * position;
    gl_Position = uProjection * vec4(vViewPosition, 1.0);
}
)GLSL";

const char* precipitationFragmentShader = R"GLSL(
#version 330 core
flat in int vType;
in vec3 vViewPosition;
uniform float uBrightness;
out vec4 fragmentColor;
void main() {
    if (vType == 2 && length(gl_PointCoord - vec2(0.5)) > 0.5) discard;
    vec3 color = vType == 1 ? vec3(0.68, 0.76, 0.84) : vec3(0.95, 0.97, 1.0);
    float fog = weatherFogTransmission(vViewPosition);
    fragmentColor = vec4(color * mix(0.3, 1.0, uBrightness),
                         (vType == 1 ? 0.34 : 0.8) * fog);
}
)GLSL";

const char* cloudFragmentShader = R"GLSL(
#version 330 core
in vec2 vPosition;
uniform mat4 uView;
uniform vec2 uProjectionScale;
uniform vec2 uOrigin;
uniform float uHeight;
uniform int uStyle;
uniform float uBrightness;
uniform vec3 uBackground;
out vec4 fragmentColor;

float hash2(vec2 p) {
    p = mod(p, 1024.0);
    return fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453);
}
float noise2(vec2 p) {
    vec2 cell = floor(p);
    vec2 f = fract(p);
    f = f * f * (3.0 - 2.0 * f);
    return mix(mix(hash2(cell), hash2(cell + vec2(1.0, 0.0)), f.x),
               mix(hash2(cell + vec2(0.0, 1.0)), hash2(cell + vec2(1.0, 1.0)), f.x), f.y);
}
void main() {
    vec3 ray = normalize(transpose(mat3(uView)) *
                         vec3(vPosition / uProjectionScale, -1.0));
    float coverage = 0.0;
    float shading = 0.0;
    if (ray.z > 0.01 && uHeight > 0.0) {
        float distanceToLayer = uHeight / ray.z;
        vec2 p = (uOrigin + ray.xy * distanceToLayer) / 1800.0;
        float noise = 0.5333 * noise2(p) + 0.2667 * noise2(p * 2.0)
                    + 0.1333 * noise2(p * 4.0) + 0.0667 * noise2(p * 8.0);
        float threshold = uStyle == 0 ? 0.62 : uStyle == 1 ? 0.53 :
                          uStyle == 2 ? 0.43 : 0.24;
        coverage = smoothstep(threshold, threshold + 0.12, noise);
        coverage *= smoothstep(0.01, 0.12, ray.z);
        coverage *= weatherFogTransmission(mat3(uView) * (ray * distanceToLayer));
        shading = noise;
    }
    vec3 cloudColor = mix(vec3(0.66, 0.69, 0.73), vec3(1.0), shading) * uBrightness;
    fragmentColor = vec4(mix(uBackground, cloudColor, coverage), 1.0);
}
)GLSL";

const char* fogShaderFunctions = R"GLSL(
uniform float uFogDensity;
uniform float uFogLayerHeight;
uniform float uFogCameraHeight;
uniform vec3 uFogWorldUp;
float weatherFogTransmission(vec3 viewPosition) {
    float fraction = 1.0;
    if (uFogLayerHeight >= 0.0) {
        float deltaHeight = dot(uFogWorldUp, viewPosition);
        float start = uFogCameraHeight - uFogLayerHeight;
        float end = start + deltaHeight;
        if (start > 0.0 && end > 0.0) {
            fraction = 0.0;
        } else if ((start > 0.0) != (end > 0.0)) {
            float crossing = clamp(-start / deltaHeight, 0.0, 1.0);
            fraction = start <= 0.0 ? crossing : 1.0 - crossing;
        }
    }
    return exp(-uFogDensity * length(viewPosition) * fraction);
}
)GLSL";

const char* vehicleSpotlightShaderFunctions = R"GLSL(
layout(std140) uniform VehicleSpotlightData {
    vec4 uSpotlightPositionRange[128];
    vec4 uSpotlightDirectionInner[128];
    vec4 uSpotlightColorOuter[128];
};
uniform int uSpotlightCount;
vec3 vehicleSpotlightLighting(vec3 position, vec3 normal) {
    vec3 result = vec3(0.0);
    for (int i = 0; i < uSpotlightCount; ++i) {
        vec3 displacement = position - uSpotlightPositionRange[i].xyz;
        float distanceToLight = length(displacement);
        float range = uSpotlightPositionRange[i].w;
        if (distanceToLight <= 0.0001 || distanceToLight / range >= 4.0) continue;
        vec3 ray = displacement / distanceToLight;
        float angle = dot(ray, uSpotlightDirectionInner[i].xyz);
        float inner = uSpotlightDirectionInner[i].w;
        float outer = uSpotlightColorOuter[i].w;
        float cone = inner > outer ? clamp((angle - outer) / (inner - outer), 0.0, 1.0)
                                   : step(inner, angle);
        float relativeDistance = distanceToLight / range;
        float attenuation = 1.0 /
            (0.2 + relativeDistance + 40.0 * relativeDistance * relativeDistance);
        float diffuse = max(dot(normal, -ray), 0.0);
        result += uSpotlightColorOuter[i].rgb * cone * cone * attenuation * (0.1 + 0.3 * diffuse);
    }
    return result;
}
)GLSL";

const char* mapLightShaderFunctions = R"GLSL(
layout(std140) uniform MapLightData {
    vec4 uMapLightPositionRadius[128];
    vec4 uMapLightColor[128];
};
uniform int uMapLightCount;
uniform float uMapLightStrength;
vec3 mapLightLighting(vec3 position, vec3 normal) {
    vec3 result = vec3(0.0);
    for (int i = 0; i < uMapLightCount; ++i) {
        vec3 displacement = position - uMapLightPositionRadius[i].xyz;
        float distanceToLight = length(displacement);
        if (distanceToLight <= 0.0001) continue;
        float radius = uMapLightPositionRadius[i].w;
        float falloff = distanceToLight <= radius ? 1.0 : (radius * radius) /
                                                           (distanceToLight * distanceToLight);
        float diffuse = max(dot(normal, -displacement / distanceToLight), 0.0);
        result += uMapLightColor[i].rgb * falloff * (0.12 + 0.88 * diffuse);
    }
    return result * uMapLightStrength;
}
)GLSL";

GLuint compileShader(GLenum type, const char* source) {
    const GLuint shader = pglCreateShader(type);
    pglShaderSource(shader, 1, &source, nullptr);
    pglCompileShader(shader);
    GLint compiled = GL_FALSE;
    pglGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);
    if (compiled == GL_TRUE) {
        return shader;
    }
    GLint logLength = 0;
    pglGetShaderiv(shader, GL_INFO_LOG_LENGTH, &logLength);
    std::string log(static_cast<std::size_t>(std::max(logLength, 1)), '\0');
    GLsizei written = 0;
    pglGetShaderInfoLog(shader, static_cast<GLsizei>(log.size()), &written, log.data());
    gameLog.Error("OpenGL shader compilation failed: " + log);
    pglDeleteShader(shader);
    return 0;
}

GLuint createProgram(const char* vertexSource, const char* fragmentSource,
                     bool usesWeatherFog = false, bool usesVehicleSpotlights = false,
                     bool usesMapLights = false) {
    const GLuint vertexShader = compileShader(GL_VERTEX_SHADER, vertexSource);
    std::string fragment(fragmentSource);
    if (usesWeatherFog) {
        const auto versionEnd = fragment.find('\n', fragment.find("#version"));
        if (versionEnd == std::string::npos)
            throw std::runtime_error("Weather shader is missing a version directive");
        fragment.insert(versionEnd + 1, fogShaderFunctions);
    }
    if (usesVehicleSpotlights) {
        const auto versionEnd = fragment.find('\n', fragment.find("#version"));
        if (versionEnd == std::string::npos)
            throw std::runtime_error("Spotlight shader is missing a version directive");
        fragment.insert(versionEnd + 1, vehicleSpotlightShaderFunctions);
    }
    if (usesMapLights) {
        const auto versionEnd = fragment.find('\n', fragment.find("#version"));
        if (versionEnd == std::string::npos)
            throw std::runtime_error("Map-light shader is missing a version directive");
        fragment.insert(versionEnd + 1, mapLightShaderFunctions);
    }
    const GLuint fragmentShader = compileShader(GL_FRAGMENT_SHADER, fragment.c_str());
    if (vertexShader == 0 || fragmentShader == 0) {
        if (vertexShader != 0) {
            pglDeleteShader(vertexShader);
        }
        if (fragmentShader != 0) {
            pglDeleteShader(fragmentShader);
        }
        return 0;
    }
    const GLuint program = pglCreateProgram();
    pglAttachShader(program, vertexShader);
    pglAttachShader(program, fragmentShader);
    pglLinkProgram(program);
    pglDeleteShader(vertexShader);
    pglDeleteShader(fragmentShader);
    GLint linked = GL_FALSE;
    pglGetProgramiv(program, GL_LINK_STATUS, &linked);
    if (linked == GL_TRUE) {
        return program;
    }
    GLint logLength = 0;
    pglGetProgramiv(program, GL_INFO_LOG_LENGTH, &logLength);
    std::string log(static_cast<std::size_t>(std::max(logLength, 1)), '\0');
    GLsizei written = 0;
    pglGetProgramInfoLog(program, static_cast<GLsizei>(log.size()), &written, log.data());
    gameLog.Error("OpenGL shader link failed: " + log);
    pglDeleteProgram(program);
    return 0;
}

void findSpotlightUniforms(GLuint program, Uniforms& uniforms) {
    uniforms.spotlightCount = pglGetUniformLocation(program, "uSpotlightCount");
}

void findMapLightUniforms(GLuint program, Uniforms& uniforms) {
    uniforms.mapLightCount = pglGetUniformLocation(program, "uMapLightCount");
    uniforms.mapLightStrength = pglGetUniformLocation(program, "uMapLightStrength");
}

void bindLightUniformBlocks(GLuint program) {
    const GLuint spotlightBlock = pglGetUniformBlockIndex(program, "VehicleSpotlightData");
    if (spotlightBlock != (std::numeric_limits<GLuint>::max)()) {
        pglUniformBlockBinding(program, spotlightBlock, vehicleSpotlightBinding);
    }
    const GLuint mapLightBlock = pglGetUniformBlockIndex(program, "MapLightData");
    if (mapLightBlock != (std::numeric_limits<GLuint>::max)()) {
        pglUniformBlockBinding(program, mapLightBlock, mapLightBinding);
    }
}

Uniforms modelUniformsFor(GLuint program, bool environment) {
    Uniforms uniforms;
    uniforms.projection = pglGetUniformLocation(program, "uProjection");
    uniforms.modelView = pglGetUniformLocation(program, "uModelView");
    uniforms.fogDensity = pglGetUniformLocation(program, "uFogDensity");
    uniforms.fogColor = pglGetUniformLocation(program, "uFogColor");
    uniforms.weatherBrightness = pglGetUniformLocation(program, "uWeatherBrightness");
    if (environment) {
        uniforms.environment = pglGetUniformLocation(program, "uEnvironment");
        uniforms.environmentAlpha = pglGetUniformLocation(program, "uEnvironmentAlpha");
    } else {
        findSpotlightUniforms(program, uniforms);
        findMapLightUniforms(program, uniforms);
        uniforms.texture = pglGetUniformLocation(program, "uTexture");
        uniforms.textureArray = pglGetUniformLocation(program, "uTextureArray");
        uniforms.textureScale = pglGetUniformLocation(program, "uTextureScale");
        uniforms.detailTexture = pglGetUniformLocation(program, "uDetailTexture");
        uniforms.detailTextureScale = pglGetUniformLocation(program, "uDetailTextureScale");
        uniforms.useTexture = pglGetUniformLocation(program, "uUseTexture");
        uniforms.textureAlphaOnly = pglGetUniformLocation(program, "uTextureAlphaOnly");
        uniforms.useDetailTexture = pglGetUniformLocation(program, "uUseDetailTexture");
        uniforms.useTextureArray = pglGetUniformLocation(program, "uUseTextureArray");
        uniforms.color = pglGetUniformLocation(program, "uColor");
        uniforms.alphaMode = pglGetUniformLocation(program, "uAlphaMode");
        uniforms.visibilityFade = pglGetUniformLocation(program, "uVisibilityFade");
        uniforms.cullBackFaces = pglGetUniformLocation(program, "uCullBackFaces");
        uniforms.lightmap = pglGetUniformLocation(program, "uLightmap");
        uniforms.nightmap = pglGetUniformLocation(program, "uNightmap");
        uniforms.transmap = pglGetUniformLocation(program, "uTransmap");
        uniforms.transmapScale = pglGetUniformLocation(program, "uTransmapScale");
        uniforms.flipTransmapY = pglGetUniformLocation(program, "uFlipTransmapY");
        uniforms.specularColor = pglGetUniformLocation(program, "uSpecularColor");
        uniforms.emissionColor = pglGetUniformLocation(program, "uEmissionColor");
        uniforms.specularPower = pglGetUniformLocation(program, "uSpecularPower");
        uniforms.snowCoverage = pglGetUniformLocation(program, "uSnowCoverage");
        uniforms.bumpmap = pglGetUniformLocation(program, "uBumpmap");
        uniforms.environment = pglGetUniformLocation(program, "uEnvironment");
        uniforms.environmentAlpha = pglGetUniformLocation(program, "uEnvironmentAlpha");
        uniforms.useEnvironment = pglGetUniformLocation(program, "uUseEnvironment");
        uniforms.freeTexture = pglGetUniformLocation(program, "uFreeTexture");
        uniforms.textTexture = pglGetUniformLocation(program, "uTextTexture");
        uniforms.useLightmap = pglGetUniformLocation(program, "uUseLightmap");
        uniforms.useNightmap = pglGetUniformLocation(program, "uUseNightmap");
        uniforms.useTransmap = pglGetUniformLocation(program, "uUseTransmap");
        uniforms.useBumpmap = pglGetUniformLocation(program, "uUseBumpmap");
        uniforms.useFreeTexture = pglGetUniformLocation(program, "uUseFreeTexture");
        uniforms.useTextTexture = pglGetUniformLocation(program, "uUseTextTexture");
        uniforms.weatherLitDynamicTexture =
            pglGetUniformLocation(program, "uWeatherLitDynamicTexture");
        uniforms.whiteAmbient = pglGetUniformLocation(program, "uWhiteAmbient");
        uniforms.useInteriorLight = pglGetUniformLocation(program, "uUseInteriorLight");
        uniforms.interiorLightCount = pglGetUniformLocation(program, "uInteriorLightCount");
        for (std::size_t index = 0; index < MAX_INTERIOR_LIGHTS; ++index) {
            uniforms.interiorLightViewPositions[index] = pglGetUniformLocation(
                program, ("uInteriorLightViewPositions[" + std::to_string(index) + "]").c_str());
            uniforms.interiorLightColors[index] = pglGetUniformLocation(
                program, ("uInteriorLightColors[" + std::to_string(index) + "]").c_str());
            uniforms.interiorLightStrengths[index] = pglGetUniformLocation(
                program, ("uInteriorLightStrengths[" + std::to_string(index) + "]").c_str());
        }
        uniforms.lightmapStrength = pglGetUniformLocation(program, "uLightmapStrength");
        uniforms.nightmapStrength = pglGetUniformLocation(program, "uNightmapStrength");
        uniforms.bumpmapStrength = pglGetUniformLocation(program, "uBumpmapStrength");
        uniforms.interiorLightStrength = pglGetUniformLocation(program, "uInteriorLightStrength");
        uniforms.interiorLightColor = pglGetUniformLocation(program, "uInteriorLightColor");
        uniforms.lightDirection = pglGetUniformLocation(program, "uLightDirection");
        uniforms.ambientLight = pglGetUniformLocation(program, "uAmbientLight");
        uniforms.diffuseLight = pglGetUniformLocation(program, "uDiffuseLight");
        uniforms.texcoordOffset = pglGetUniformLocation(program, "uTexcoordOffset");
        uniforms.flipTextureY = pglGetUniformLocation(program, "uFlipModelTextureY");
        uniforms.humanGaitEnabled = pglGetUniformLocation(program, "uHumanGaitEnabled");
        uniforms.humanSkinEnabled = pglGetUniformLocation(program, "uHumanSkinEnabled");
        uniforms.humanBones = pglGetUniformLocation(program, "uHumanBones[0]");
        uniforms.humanGaitPhase = pglGetUniformLocation(program, "uHumanGaitPhase");
        uniforms.humanGaitWeight = pglGetUniformLocation(program, "uHumanGaitWeight");
        uniforms.humanGaitHeight = pglGetUniformLocation(program, "uHumanGaitHeight");
        uniforms.humanUpperArmBeta = pglGetUniformLocation(program, "uHumanUpperArmBeta");
        uniforms.humanArmSwing = pglGetUniformLocation(program, "uHumanArmSwing");
        uniforms.humanHipSway = pglGetUniformLocation(program, "uHumanHipSway");
        uniforms.humanWaistAnimation = pglGetUniformLocation(program, "uHumanWaistAnimation");
        uniforms.humanShoulderXFraction = pglGetUniformLocation(program, "uHumanShoulderXFraction");
        uniforms.humanShoulderZFraction = pglGetUniformLocation(program, "uHumanShoulderZFraction");
        uniforms.humanHipXFraction = pglGetUniformLocation(program, "uHumanHipXFraction");
        uniforms.humanHipZFraction = pglGetUniformLocation(program, "uHumanHipZFraction");
        uniforms.humanKneeXFraction = pglGetUniformLocation(program, "uHumanKneeXFraction");
        uniforms.humanKneeYFraction = pglGetUniformLocation(program, "uHumanKneeYFraction");
        uniforms.humanKneeZFraction = pglGetUniformLocation(program, "uHumanKneeZFraction");
        uniforms.humanElbowXFraction = pglGetUniformLocation(program, "uHumanElbowXFraction");
        uniforms.humanElbowZFraction = pglGetUniformLocation(program, "uHumanElbowZFraction");
        uniforms.humanHandXFraction = pglGetUniformLocation(program, "uHumanHandXFraction");
        uniforms.humanHandZFraction = pglGetUniformLocation(program, "uHumanHandZFraction");
        uniforms.humanFingerXFraction = pglGetUniformLocation(program, "uHumanFingerXFraction");
        uniforms.humanFingerZFraction = pglGetUniformLocation(program, "uHumanFingerZFraction");
        uniforms.humanNeckYFraction = pglGetUniformLocation(program, "uHumanNeckYFraction");
        uniforms.humanNeckZFraction = pglGetUniformLocation(program, "uHumanNeckZFraction");
        uniforms.humanWaistYFraction = pglGetUniformLocation(program, "uHumanWaistYFraction");
        uniforms.humanWaistZFraction = pglGetUniformLocation(program, "uHumanWaistZFraction");
        uniforms.useMaterialColors = pglGetUniformLocation(program, "uUseMaterialColors");
        uniforms.materialColors = pglGetUniformLocation(program, "uMaterialColors");
        uniforms.useMaterialTextures = pglGetUniformLocation(program, "uUseMaterialTextures");
        for (std::size_t index = 0; index < MAX_MATERIAL_TEXTURES; ++index) {
            uniforms.materialTextures[index] = pglGetUniformLocation(
                program, ("uMaterialTextures[" + std::to_string(index) + "]").c_str());
        }
    }
    return uniforms;
}

Uniforms materialUniformsFor(GLuint program) {
    Uniforms uniforms;
    findSpotlightUniforms(program, uniforms);
    findMapLightUniforms(program, uniforms);
    uniforms.projection = pglGetUniformLocation(program, "uProjection");
    uniforms.modelView = pglGetUniformLocation(program, "uModelView");
    uniforms.fogDensity = pglGetUniformLocation(program, "uFogDensity");
    uniforms.fogColor = pglGetUniformLocation(program, "uFogColor");
    uniforms.visibilityFade = pglGetUniformLocation(program, "uVisibilityFade");
    uniforms.materialColors = pglGetUniformLocation(program, "uMaterialColors");
    uniforms.lightDirection = pglGetUniformLocation(program, "uLightDirection");
    uniforms.ambientLight = pglGetUniformLocation(program, "uAmbientLight");
    uniforms.diffuseLight = pglGetUniformLocation(program, "uDiffuseLight");
    for (std::size_t index = 0; index < MAX_MATERIAL_TEXTURES; ++index) {
        uniforms.materialFlipTextureY[index] = pglGetUniformLocation(
            program, ("uFlipTextureY[" + std::to_string(index) + "]").c_str());
    }
    for (std::size_t index = 0; index < MAX_MATERIAL_TEXTURES; ++index) {
        uniforms.materialTextures[index] = pglGetUniformLocation(
            program, ("uMaterialTextures[" + std::to_string(index) + "]").c_str());
    }
    return uniforms;
}

void uploadMatrices(const Uniforms& uniforms) {
    const Matrix4& projection = projectionMatrix();
    const Matrix4& modelView = modelViewMatrix();
    const bool projectionChanged =
        matrixProgram != currentProgram || cachedProjection != projection;
    const bool modelViewChanged = matrixProgram != currentProgram || cachedModelView != modelView;
    if (!projectionChanged && !modelViewChanged) {
        return;
    }
    std::array<GLfloat, 16> projectionFloat = {};
    std::array<GLfloat, 16> modelViewFloat = {};
    for (std::size_t index = 0; index < projection.size(); ++index) {
        projectionFloat[index] = static_cast<GLfloat>(projection[index]);
        modelViewFloat[index] = static_cast<GLfloat>(modelView[index]);
    }
    if (projectionChanged) {
        pglUniformMatrix4fv(uniforms.projection, 1, GL_FALSE, projectionFloat.data());
        cachedProjection = projection;
    }
    if (modelViewChanged) {
        pglUniformMatrix4fv(uniforms.modelView, 1, GL_FALSE, modelViewFloat.data());
        cachedModelView = modelView;
    }
    matrixProgram = currentProgram;
}

bool modelViewReversesWinding() {
    const Matrix4& m = modelViewMatrix();
    const double determinant = m[0] * (m[5] * m[10] - m[9] * m[6]) -
                               m[4] * (m[1] * m[10] - m[9] * m[2]) +
                               m[8] * (m[1] * m[6] - m[5] * m[2]);
    return determinant < 0.0;
}

void bindModelVertexBuffer(GLuint buffer, GLuint vertexArray) {
    bool vertexArrayChanged = false;
    if (currentVertexArray != vertexArray) {
        pglBindVertexArray(vertexArray);
        currentVertexArray = vertexArray;
        vertexArrayChanged = true;
    }
    if (!vertexArrayChanged && currentArrayBuffer == buffer) {
        return;
    }
    pglBindBuffer(GL_ARRAY_BUFFER, buffer);
    currentArrayBuffer = buffer;
    pglVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, MODEL_VERTEX_STRIDE, nullptr);
    pglVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, MODEL_VERTEX_STRIDE,
                           reinterpret_cast<const void*>(3 * sizeof(float)));
    pglVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, MODEL_VERTEX_STRIDE,
                           reinterpret_cast<const void*>(6 * sizeof(float)));
}

void useProgram(GLuint program) {
    if (currentProgram == program) {
        return;
    }
    pglUseProgram(program);
    currentProgram = program;
}

void bindTexture(GLenum target, GLuint texture) {
    const std::size_t unit = static_cast<std::size_t>(currentTextureUnit - GL_TEXTURE0);
    GLuint& cachedTexture =
        target == GL_TEXTURE_2D_ARRAY ? boundTextureArray[unit] : boundTexture2D[unit];
    if (cachedTexture == texture) {
        return;
    }
    glBindTexture(target, texture);
    cachedTexture = texture;
}

void selectTextureUnit(GLenum textureUnit) {
    if (currentTextureUnit == textureUnit) {
        return;
    }
    pglActiveTexture(textureUnit);
    currentTextureUnit = textureUnit;
}

void setTextureWrap(GLenum target, GLuint texture, GLint wrapS, GLint wrapT) {
    if (wrapParametersValid && wrapParametersTexture == texture && wrapParametersTarget == target &&
        wrapParametersS == wrapS && wrapParametersT == wrapT) {
        return;
    }
    glTexParameteri(target, GL_TEXTURE_WRAP_S, wrapS);
    glTexParameteri(target, GL_TEXTURE_WRAP_T, wrapT);
    wrapParametersValid = true;
    wrapParametersTexture = texture;
    wrapParametersTarget = target;
    wrapParametersS = wrapS;
    wrapParametersT = wrapT;
}

void uploadModelUniforms(const ModelMaterial& material, const std::array<double, 3>& color,
                         double alpha, int alphaMode, double visibilityFade) {
    const std::array<float, 4> colorValue = {
        static_cast<float>(color[0]), static_cast<float>(color[1]), static_cast<float>(color[2]),
        static_cast<float>(alpha)};
    const bool flagsChanged =
        !modelUniformState.valid || modelUniformState.textured != material.textured ||
        modelUniformState.textureAlphaOnly != material.textureAlphaOnly ||
        modelUniformState.textureArray != (material.textured && material.textureArray) ||
        modelUniformState.useDetailTexture != material.useDetailTexture ||
        modelUniformState.useLightmap != material.useLightmap ||
        modelUniformState.useNightmap != material.useNightmap ||
        modelUniformState.useTransmap != material.useTransmap ||
        modelUniformState.flipTransmapY != material.flipTransmapY ||
        modelUniformState.useBumpmap != material.useBumpmap ||
        modelUniformState.useEnvironmentMap != material.useEnvironmentMap ||
        modelUniformState.useFreeTexture != material.useFreeTexture ||
        modelUniformState.useTextTexture != material.useTextTexture ||
        modelUniformState.weatherLitDynamicTexture != material.weatherLitDynamicTexture ||
        modelUniformState.whiteAmbient != material.whiteAmbient ||
        modelUniformState.useInteriorLight != material.useInteriorLight ||
        modelUniformState.interiorLightCount != material.interiorLightCount;
    if (flagsChanged) {
        pglUniform1i(modelUniforms.useTexture, material.textured ? 1 : 0);
        pglUniform1i(modelUniforms.textureAlphaOnly, material.textureAlphaOnly ? 1 : 0);
        pglUniform1i(modelUniforms.useTextureArray,
                     material.textured && material.textureArray ? 1 : 0);
        pglUniform1i(modelUniforms.useDetailTexture, material.useDetailTexture ? 1 : 0);
        pglUniform1i(modelUniforms.useLightmap, material.useLightmap ? 1 : 0);
        pglUniform1i(modelUniforms.useNightmap, material.useNightmap ? 1 : 0);
        pglUniform1i(modelUniforms.useTransmap, material.useTransmap ? 1 : 0);
        pglUniform1i(modelUniforms.flipTransmapY, material.flipTransmapY ? 1 : 0);
        pglUniform1i(modelUniforms.useBumpmap, material.useBumpmap ? 1 : 0);
        pglUniform1i(modelUniforms.useEnvironment, material.useEnvironmentMap ? 1 : 0);
        pglUniform1i(modelUniforms.useFreeTexture, material.useFreeTexture ? 1 : 0);
        pglUniform1i(modelUniforms.useTextTexture, material.useTextTexture ? 1 : 0);
        pglUniform1i(modelUniforms.weatherLitDynamicTexture,
                     material.weatherLitDynamicTexture ? 1 : 0);
        pglUniform1i(modelUniforms.whiteAmbient, material.whiteAmbient ? 1 : 0);
        pglUniform1i(modelUniforms.useInteriorLight, material.useInteriorLight ? 1 : 0);
        pglUniform1i(modelUniforms.interiorLightCount, material.interiorLightCount);
    }
    if (!modelUniformState.valid || modelUniformState.alphaMode != alphaMode) {
        pglUniform1i(modelUniforms.alphaMode, alphaMode);
    }
    pglUniform1f(modelUniforms.visibilityFade,
                 static_cast<GLfloat>(std::clamp(visibilityFade, 0.0, 1.0)));
    if (!modelUniformState.valid ||
        modelUniformState.lightmapStrength != material.lightmapStrength) {
        pglUniform1f(modelUniforms.lightmapStrength, material.lightmapStrength);
    }
    if (!modelUniformState.valid ||
        modelUniformState.nightmapStrength != material.nightmapStrength) {
        pglUniform1f(modelUniforms.nightmapStrength, material.nightmapStrength);
    }
    if (!modelUniformState.valid || modelUniformState.bumpmapStrength != material.bumpmapStrength) {
        pglUniform1f(modelUniforms.bumpmapStrength, material.bumpmapStrength);
    }
    if (!modelUniformState.valid ||
        modelUniformState.environmentStrength != material.environmentStrength) {
        pglUniform1f(modelUniforms.environmentAlpha, material.environmentStrength);
    }
    if (!modelUniformState.valid || modelUniformState.textureScale != material.textureScale) {
        pglUniform2f(modelUniforms.textureScale, material.textureScale[0],
                     material.textureScale[1]);
    }
    if (!modelUniformState.valid ||
        modelUniformState.detailTextureScale != material.detailTextureScale) {
        pglUniform2f(modelUniforms.detailTextureScale, material.detailTextureScale[0],
                     material.detailTextureScale[1]);
    }
    if (!modelUniformState.valid || modelUniformState.transmapScale != material.transmapScale) {
        pglUniform2f(modelUniforms.transmapScale, material.transmapScale[0],
                     material.transmapScale[1]);
    }
    if (!modelUniformState.valid || modelUniformState.specularColor != material.specularColor) {
        pglUniform3f(modelUniforms.specularColor, material.specularColor[0],
                     material.specularColor[1], material.specularColor[2]);
    }
    if (!modelUniformState.valid || modelUniformState.emissionColor != material.emissionColor) {
        pglUniform3f(modelUniforms.emissionColor, material.emissionColor[0],
                     material.emissionColor[1], material.emissionColor[2]);
    }
    if (!modelUniformState.valid || modelUniformState.specularPower != material.specularPower) {
        pglUniform1f(modelUniforms.specularPower, material.specularPower);
    }
    if (!modelUniformState.valid || modelUniformState.snowCoverage != material.snowCoverage) {
        pglUniform1f(modelUniforms.snowCoverage, material.snowCoverage);
    }
    if (!modelUniformState.valid ||
        modelUniformState.interiorLightViewPositions != material.interiorLightViewPositions) {
        for (std::size_t index = 0; index < MAX_INTERIOR_LIGHTS; ++index) {
            pglUniform3f(modelUniforms.interiorLightViewPositions[index],
                         material.interiorLightViewPositions[index][0],
                         material.interiorLightViewPositions[index][1],
                         material.interiorLightViewPositions[index][2]);
        }
    }
    if (!modelUniformState.valid ||
        modelUniformState.interiorLightColors != material.interiorLightColors) {
        for (std::size_t index = 0; index < MAX_INTERIOR_LIGHTS; ++index) {
            pglUniform3f(
                modelUniforms.interiorLightColors[index], material.interiorLightColors[index][0],
                material.interiorLightColors[index][1], material.interiorLightColors[index][2]);
        }
    }
    if (!modelUniformState.valid ||
        modelUniformState.interiorLightStrengths != material.interiorLightStrengths) {
        for (std::size_t index = 0; index < MAX_INTERIOR_LIGHTS; ++index) {
            pglUniform1f(modelUniforms.interiorLightStrengths[index],
                         material.interiorLightStrengths[index]);
        }
    }
    if (!modelUniformState.valid || modelUniformState.texcoordOffsetX != material.texcoordOffsetX ||
        modelUniformState.texcoordOffsetY != material.texcoordOffsetY) {
        pglUniform2f(modelUniforms.texcoordOffset, material.texcoordOffsetX,
                     material.texcoordOffsetY);
    }
    if (!modelUniformState.valid || modelUniformState.flipTextureY != material.flipTextureY) {
        pglUniform1i(modelUniforms.flipTextureY, material.flipTextureY ? 1 : 0);
    }
    if (!modelUniformState.valid || modelUniformState.color != colorValue) {
        pglUniform4f(modelUniforms.color, colorValue[0], colorValue[1], colorValue[2],
                     colorValue[3]);
    }
    modelUniformState.valid = true;
    modelUniformState.textured = material.textured;
    modelUniformState.textureAlphaOnly = material.textureAlphaOnly;
    modelUniformState.textureArray = material.textured && material.textureArray;
    modelUniformState.useDetailTexture = material.useDetailTexture;
    modelUniformState.useLightmap = material.useLightmap;
    modelUniformState.useNightmap = material.useNightmap;
    modelUniformState.useTransmap = material.useTransmap;
    modelUniformState.flipTransmapY = material.flipTransmapY;
    modelUniformState.useBumpmap = material.useBumpmap;
    modelUniformState.useEnvironmentMap = material.useEnvironmentMap;
    modelUniformState.useFreeTexture = material.useFreeTexture;
    modelUniformState.useTextTexture = material.useTextTexture;
    modelUniformState.weatherLitDynamicTexture = material.weatherLitDynamicTexture;
    modelUniformState.whiteAmbient = material.whiteAmbient;
    modelUniformState.useInteriorLight = material.useInteriorLight;
    modelUniformState.flipTextureY = material.flipTextureY;
    modelUniformState.alphaMode = alphaMode;
    modelUniformState.lightmapStrength = material.lightmapStrength;
    modelUniformState.nightmapStrength = material.nightmapStrength;
    modelUniformState.bumpmapStrength = material.bumpmapStrength;
    modelUniformState.environmentStrength = material.environmentStrength;
    modelUniformState.textureScale = material.textureScale;
    modelUniformState.detailTextureScale = material.detailTextureScale;
    modelUniformState.transmapScale = material.transmapScale;
    modelUniformState.specularColor = material.specularColor;
    modelUniformState.emissionColor = material.emissionColor;
    modelUniformState.specularPower = material.specularPower;
    modelUniformState.snowCoverage = material.snowCoverage;
    modelUniformState.interiorLightCount = material.interiorLightCount;
    modelUniformState.interiorLightViewPositions = material.interiorLightViewPositions;
    modelUniformState.interiorLightColors = material.interiorLightColors;
    modelUniformState.interiorLightStrengths = material.interiorLightStrengths;
    modelUniformState.texcoordOffsetX = material.texcoordOffsetX;
    modelUniformState.texcoordOffsetY = material.texcoordOffsetY;
    modelUniformState.color = colorValue;
}

} // namespace

void resetFrameTriangleCount() {
    submittedFrameTriangleCount = 0;
    reflectionTrianglePass = false;
}

GpuTimerScope::GpuTimerScope(GpuTimingPass pass) : active_(beginGpuTimerQuery(pass)) {}

GpuTimerScope::~GpuTimerScope() {
    if (active_) {
        endGpuTimerQuery();
    }
}

void beginGpuTimingFrame() {
    pollGpuTimingQueries();
    if (!gpuTimerState.enabled) {
        return;
    }
    ++gpuTimerState.frameCount;
    gpuTimerState.frameSampled =
        gpuTimerState.frameCount % static_cast<std::uint64_t>(gpuTimerState.sampleEveryNFrames) ==
        0;
}

void resetFrameReflectionTriangleCount() {
    submittedReflectionTriangleCount = 0;
}

std::size_t frameTriangleCount() {
    return submittedFrameTriangleCount;
}

std::size_t frameReflectionTriangleCount() {
    return submittedReflectionTriangleCount;
}

void setFrameTriangleCountReflectionPass(bool reflectionPass) {
    reflectionTrianglePass = reflectionPass;
}

bool initializeCoreRenderer() {
    modelProgram = createProgram(modelVertexShader, modelFragmentShader, true, true, true);
    materialProgram = createProgram(modelVertexShader, materialFragmentShader, true, true, true);
    environmentProgram = createProgram(modelVertexShader, environmentFragmentShader, true);
    primitiveProgram = createProgram(primitiveVertexShader, primitiveFragmentShader);
    textureQuadProgram = createProgram(textureQuadVertexShader, textureQuadFragmentShader);
    cloudProgram = createProgram(cloudVertexShader, cloudFragmentShader, true);
    precipitationProgram =
        createProgram(precipitationVertexShader, precipitationFragmentShader, true);
    if (modelProgram == 0 || materialProgram == 0 || environmentProgram == 0 ||
        primitiveProgram == 0 || textureQuadProgram == 0 || cloudProgram == 0 ||
        precipitationProgram == 0) {
        shutdownCoreRenderer();
        return false;
    }
    modelUniforms = modelUniformsFor(modelProgram, false);
    materialUniforms = materialUniformsFor(materialProgram);
    environmentUniforms = modelUniformsFor(environmentProgram, true);
    primitiveUniforms = modelUniformsFor(primitiveProgram, false);
    bindLightUniformBlocks(modelProgram);
    bindLightUniformBlocks(materialProgram);
    textureQuadRect = pglGetUniformLocation(textureQuadProgram, "uRect");
    textureQuadTexture = pglGetUniformLocation(textureQuadProgram, "uTexture");
    textureQuadFlipTextureY = pglGetUniformLocation(textureQuadProgram, "uFlipTextureY");
    cloudView = pglGetUniformLocation(cloudProgram, "uView");
    cloudProjectionScale = pglGetUniformLocation(cloudProgram, "uProjectionScale");
    cloudOrigin = pglGetUniformLocation(cloudProgram, "uOrigin");
    cloudHeight = pglGetUniformLocation(cloudProgram, "uHeight");
    cloudStyle = pglGetUniformLocation(cloudProgram, "uStyle");
    cloudBrightness = pglGetUniformLocation(cloudProgram, "uBrightness");
    cloudBackground = pglGetUniformLocation(cloudProgram, "uBackground");
    precipitationView = pglGetUniformLocation(precipitationProgram, "uView");
    precipitationProjection = pglGetUniformLocation(precipitationProgram, "uProjection");
    precipitationEye = pglGetUniformLocation(precipitationProgram, "uEye");
    precipitationOffset = pglGetUniformLocation(precipitationProgram, "uOffset");
    precipitationWind = pglGetUniformLocation(precipitationProgram, "uWind");
    precipitationType = pglGetUniformLocation(precipitationProgram, "uType");
    precipitationBrightness = pglGetUniformLocation(precipitationProgram, "uBrightness");
    const std::array<GLuint, 5> weatherPrograms = {
        modelProgram, materialProgram, environmentProgram, cloudProgram, precipitationProgram};
    for (std::size_t i = 0; i < weatherPrograms.size(); ++i) {
        const GLuint program = weatherPrograms[i];
        fogUniforms[i] = {program, pglGetUniformLocation(program, "uFogDensity"),
                          pglGetUniformLocation(program, "uFogLayerHeight"),
                          pglGetUniformLocation(program, "uFogCameraHeight"),
                          pglGetUniformLocation(program, "uFogWorldUp")};
    }
    pglGenVertexArrays(1, &modelVertexArray);
    pglGenVertexArrays(1, &primitiveVertexArray);
    pglGenVertexArrays(1, &precipitationVertexArray);
    pglGenBuffers(1, &primitiveBuffer);
    pglGenBuffers(1, &vehicleSpotlightBuffer);
    pglGenBuffers(1, &mapLightBuffer);
    pglGenVertexArrays(1, &textureQuadVertexArray);
    pglGenBuffers(1, &textureQuadBuffer);
    if (modelVertexArray != 0 && primitiveVertexArray != 0 && precipitationVertexArray != 0 &&
        primitiveBuffer != 0 && vehicleSpotlightBuffer != 0 && mapLightBuffer != 0 &&
        textureQuadVertexArray != 0 && textureQuadBuffer != 0) {
        pglBindBuffer(GL_UNIFORM_BUFFER, vehicleSpotlightBuffer);
        pglBufferData(GL_UNIFORM_BUFFER, sizeof(GpuVehicleSpotlightData), nullptr, GL_DYNAMIC_DRAW);
        pglBindBufferBase(GL_UNIFORM_BUFFER, vehicleSpotlightBinding, vehicleSpotlightBuffer);
        pglBindBuffer(GL_UNIFORM_BUFFER, mapLightBuffer);
        pglBufferData(GL_UNIFORM_BUFFER, sizeof(GpuMapLightData), nullptr, GL_DYNAMIC_DRAW);
        pglBindBufferBase(GL_UNIFORM_BUFFER, mapLightBinding, mapLightBuffer);
        pglBindBuffer(GL_UNIFORM_BUFFER, 0);
        pglBindVertexArray(modelVertexArray);
        pglEnableVertexAttribArray(0);
        pglEnableVertexAttribArray(1);
        pglEnableVertexAttribArray(2);
        pglBindVertexArray(primitiveVertexArray);
        pglEnableVertexAttribArray(0);
        pglEnableVertexAttribArray(1);
        pglBindVertexArray(textureQuadVertexArray);
        pglEnableVertexAttribArray(0);
        pglEnableVertexAttribArray(1);
        pglBindVertexArray(0);
        useProgram(modelProgram);
        pglUniform1f(modelUniforms.ambientLight, 0.58f);
        pglUniform1f(modelUniforms.diffuseLight, 0.42f);
        pglUniform1i(modelUniforms.texture, 0);
        // sampler2D and sampler2DArray cannot alias the same unit in GLSL 3.3;
        // OpenGL rejects every draw even when only one shader branch is active.
        pglUniform1i(modelUniforms.textureArray, 7);
        pglUniform1i(modelUniforms.detailTexture, 8);
        pglUniform1i(modelUniforms.lightmap, 1);
        pglUniform1i(modelUniforms.nightmap, 2);
        pglUniform1i(modelUniforms.transmap, 3);
        pglUniform1i(modelUniforms.bumpmap, 6);
        pglUniform1i(modelUniforms.environment, 9);
        pglUniform1i(modelUniforms.freeTexture, 4);
        pglUniform1i(modelUniforms.textTexture, 5);
        useProgram(materialProgram);
        pglUniform1f(materialUniforms.ambientLight, 0.58f);
        pglUniform1f(materialUniforms.diffuseLight, 0.42f);
        for (std::size_t index = 0; index < MAX_MATERIAL_TEXTURES; ++index) {
            pglUniform1i(materialUniforms.materialTextures[index], static_cast<GLint>(index));
        }
        useProgram(environmentProgram);
        pglUniform1i(environmentUniforms.environment, 0);
        useProgram(primitiveProgram);
        useProgram(textureQuadProgram);
        pglUniform1i(textureQuadTexture, 0);
        useProgram(0);
        setLightingViewMatrix(identityMatrix());
        setWeatherLighting(std::nullopt);
        setWeatherFog(std::nullopt);
        currentVertexArray = 0;
        currentArrayBuffer = 0;
        currentProgram = 0;
        matrixProgram = 0;
        modelUniformState = {};
        materialUniformState = {};
        currentTextureUnit = GL_TEXTURE0;
        boundTexture2D.fill(0);
        boundTextureArray.fill(0);
        initializeGpuTiming();
        ++rendererGeneration;
        if (rendererGeneration == 0) {
            ++rendererGeneration;
        }
        const char* cullingBackend = openbus::getEnvironment("OPENBUS_CULLING_BACKEND");
        const std::string_view backend = cullingBackend != nullptr ? cullingBackend : "";
        if (backend == "compute-readback") {
            gameLog.Warning("compute-readback culling is retired; using CPU visibility tests. "
                            "Set OPENBUS_CULLING_BACKEND=compute-indirect for GPU culling");
        }
        initializeComputeCulling(false, indirectCullingRequested(backend));
        return true;
    }
    shutdownCoreRenderer();
    return false;
}

void shutdownCoreRenderer() {
    shutdownGpuTiming();
    shutdownComputeCulling();
    fogUniforms = {};
    currentProgram = 0;
    currentVertexArray = 0;
    currentArrayBuffer = 0;
    matrixProgram = 0;
    modelUniformState = {};
    materialUniformState = {};
    currentTextureUnit = GL_TEXTURE0;
    boundTexture2D.fill(0);
    boundTextureArray.fill(0);
    if (!staticPrimitiveBuffers.empty()) {
        pglDeleteBuffers(static_cast<GLsizei>(staticPrimitiveBuffers.size()),
                         staticPrimitiveBuffers.data());
    }
    staticPrimitiveBuffers.clear();
    if (primitiveBuffer != 0) {
        pglDeleteBuffers(1, &primitiveBuffer);
        primitiveBuffer = 0;
    }
    if (vehicleSpotlightBuffer != 0) {
        pglDeleteBuffers(1, &vehicleSpotlightBuffer);
        vehicleSpotlightBuffer = 0;
    }
    if (mapLightBuffer != 0) {
        pglDeleteBuffers(1, &mapLightBuffer);
        mapLightBuffer = 0;
    }
    if (textureQuadBuffer != 0) {
        pglDeleteBuffers(1, &textureQuadBuffer);
        textureQuadBuffer = 0;
    }
    if (primitiveVertexArray != 0) {
        pglDeleteVertexArrays(1, &primitiveVertexArray);
        primitiveVertexArray = 0;
    }
    if (precipitationVertexArray != 0) {
        pglDeleteVertexArrays(1, &precipitationVertexArray);
        precipitationVertexArray = 0;
    }
    if (textureQuadVertexArray != 0) {
        pglDeleteVertexArrays(1, &textureQuadVertexArray);
        textureQuadVertexArray = 0;
    }
    if (modelVertexArray != 0) {
        pglDeleteVertexArrays(1, &modelVertexArray);
        modelVertexArray = 0;
    }
    if (primitiveProgram != 0) {
        pglDeleteProgram(primitiveProgram);
        primitiveProgram = 0;
    }
    if (environmentProgram != 0) {
        pglDeleteProgram(environmentProgram);
        environmentProgram = 0;
    }
    if (modelProgram != 0) {
        pglDeleteProgram(modelProgram);
        modelProgram = 0;
    }
    if (materialProgram != 0) {
        pglDeleteProgram(materialProgram);
        materialProgram = 0;
    }
    if (textureQuadProgram != 0) {
        pglDeleteProgram(textureQuadProgram);
        textureQuadProgram = 0;
    }
    if (cloudProgram != 0) {
        pglDeleteProgram(cloudProgram);
        cloudProgram = 0;
    }
    if (precipitationProgram != 0) {
        pglDeleteProgram(precipitationProgram);
        precipitationProgram = 0;
    }
}

void invalidateTextureBindings() {
    boundTexture2D.fill(std::numeric_limits<GLuint>::max());
    boundTextureArray.fill(std::numeric_limits<GLuint>::max());
    wrapParametersValid = false;
    pglActiveTexture(GL_TEXTURE0);
    currentTextureUnit = GL_TEXTURE0;
}

void deleteRendererBuffers(GLsizei count, const GLuint* buffers) {
    for (GLsizei index = 0; index < count; ++index) {
        if (buffers[index] == currentArrayBuffer)
            currentArrayBuffer = 0;
        invalidateCullingStorageBinding(buffers[index]);
    }
    pglDeleteBuffers(count, buffers);
}

void setLightingViewMatrix(const Matrix4& view, std::optional<std::array<double, 3>> sunDirection) {
    const auto world = sunDirection.value_or(std::array<double, 3>{-0.35, 0.5, 0.8});
    if (!std::isfinite(world[0]) || !std::isfinite(world[1]) || !std::isfinite(world[2]) ||
        std::hypot(world[0], world[1], world[2]) < 1e-12) {
        throw std::invalid_argument("Sun direction must be finite and non-zero");
    }
    const auto direction = transformPoint(view, {world[0], world[1], world[2], 0.0});
    const GLuint previousProgram = currentProgram;
    useProgram(modelProgram);
    pglUniform3f(modelUniforms.lightDirection, static_cast<float>(direction[0]),
                 static_cast<float>(direction[1]), static_cast<float>(direction[2]));
    useProgram(materialProgram);
    pglUniform3f(materialUniforms.lightDirection, static_cast<float>(direction[0]),
                 static_cast<float>(direction[1]), static_cast<float>(direction[2]));
    useProgram(previousProgram);
}

void setVehicleSpotlights(const std::vector<VehicleSpotlight>& lights) {
    if (lights.size() > MAX_VEHICLE_SPOTLIGHTS || vehicleSpotlightBuffer == 0) {
        throw std::invalid_argument("Invalid vehicle spotlight collection");
    }
    GpuVehicleSpotlightData gpuLights;
    for (const VehicleSpotlight& light : lights) {
        const auto finite = [](double value) {
            return std::isfinite(value) && std::abs(value) <= std::numeric_limits<float>::max();
        };
        if (!std::all_of(light.position.begin(), light.position.end(), finite) ||
            !std::all_of(light.direction.begin(), light.direction.end(), finite) ||
            !std::all_of(light.color.begin(), light.color.end(), finite) || !finite(light.range) ||
            light.range <= 0.0 ||
            std::hypot(light.direction[0], light.direction[1], light.direction[2]) < 1e-12 ||
            !finite(light.innerAngleDegrees) || !finite(light.outerAngleDegrees) ||
            light.innerAngleDegrees < 0.0 || light.innerAngleDegrees > light.outerAngleDegrees ||
            light.outerAngleDegrees > 180.0 ||
            std::any_of(light.color.begin(), light.color.end(),
                        [](double value) { return value < 0.0 || value > 1.0; })) {
            throw std::invalid_argument("Invalid vehicle spotlight");
        }
    }
    for (std::size_t index = 0; index < lights.size(); ++index) {
        const VehicleSpotlight& light = lights[index];
        const double length =
            std::hypot(light.direction[0], light.direction[1], light.direction[2]);
        gpuLights.positionRange[index] = {
            static_cast<float>(light.position[0]), static_cast<float>(light.position[1]),
            static_cast<float>(light.position[2]), static_cast<float>(light.range)};
        constexpr double halfDegreesToRadians = 3.14159265358979323846 / 360.0;
        gpuLights.directionInner[index] = {
            static_cast<float>(light.direction[0] / length),
            static_cast<float>(light.direction[1] / length),
            static_cast<float>(light.direction[2] / length),
            static_cast<float>(std::cos(light.innerAngleDegrees * halfDegreesToRadians))};
        gpuLights.colorOuter[index] = {
            static_cast<float>(light.color[0]), static_cast<float>(light.color[1]),
            static_cast<float>(light.color[2]),
            static_cast<float>(std::cos(light.outerAngleDegrees * halfDegreesToRadians))};
    }
    pglBindBuffer(GL_UNIFORM_BUFFER, vehicleSpotlightBuffer);
    pglBufferData(GL_UNIFORM_BUFFER, sizeof(gpuLights), &gpuLights, GL_DYNAMIC_DRAW);
    pglBindBufferBase(GL_UNIFORM_BUFFER, vehicleSpotlightBinding, vehicleSpotlightBuffer);
    pglBindBuffer(GL_UNIFORM_BUFFER, 0);
    const GLuint previousProgram = currentProgram;
    for (const auto& entry :
         {std::pair{modelProgram, &modelUniforms}, std::pair{materialProgram, &materialUniforms}}) {
        useProgram(entry.first);
        const Uniforms& uniforms = *entry.second;
        pglUniform1i(uniforms.spotlightCount, static_cast<GLint>(lights.size()));
    }
    useProgram(previousProgram);
}

void setWeatherFog(std::optional<double> visibilityMeters,
                   std::optional<double> layerHeightMeters) {
    if (visibilityMeters && (!std::isfinite(*visibilityMeters) || *visibilityMeters < 0.0)) {
        throw std::invalid_argument("Weather visibility must be finite and non-negative");
    }
    if (layerHeightMeters && (!std::isfinite(*layerHeightMeters) || *layerHeightMeters < 0.0 ||
                              *layerHeightMeters > std::numeric_limits<float>::max())) {
        throw std::invalid_argument("Weather fog height must be finite and non-negative");
    }
    const float density =
        visibilityMeters ? static_cast<float>(3.912023005428146 / std::max(*visibilityMeters, 1.0))
                         : 0.0F;
    const GLuint previousProgram = currentProgram;
    for (const auto& entry : fogUniforms) {
        if (entry.program == 0) {
            continue;
        }
        useProgram(entry.program);
        pglUniform1f(entry.density, density);
        pglUniform1f(entry.layerHeight,
                     layerHeightMeters ? static_cast<float>(*layerHeightMeters) : -1.0F);
    }
    useProgram(previousProgram);
}

void setWeatherFogView(const Matrix4& view, double groundHeightMeters) {
    const double eyeZ = -(view[8] * view[12] + view[9] * view[13] + view[10] * view[14]);
    const double height = eyeZ - groundHeightMeters;
    const double maxFloat = std::numeric_limits<float>::max();
    if (!std::isfinite(height) || std::abs(height) > maxFloat || !std::isfinite(view[8]) ||
        !std::isfinite(view[9]) || !std::isfinite(view[10]) || std::abs(view[8]) > maxFloat ||
        std::abs(view[9]) > maxFloat || std::abs(view[10]) > maxFloat) {
        throw std::invalid_argument("Weather fog camera or ground height is invalid");
    }
    const GLuint previousProgram = currentProgram;
    for (const auto& entry : fogUniforms) {
        if (entry.program == 0)
            continue;
        useProgram(entry.program);
        pglUniform1f(entry.cameraHeight, static_cast<float>(height));
        pglUniform3f(entry.worldUp, static_cast<float>(view[8]), static_cast<float>(view[9]),
                     static_cast<float>(view[10]));
    }
    useProgram(previousProgram);
}

void setMapLights(const std::vector<MapPointLight>& lights, double strength) {
    const auto finite = [](double value) {
        return std::isfinite(value) && std::abs(value) <= std::numeric_limits<float>::max();
    };
    if (lights.size() > MAX_MAP_LIGHTS || !finite(strength) || strength < 0.0 || strength > 1.0 ||
        mapLightBuffer == 0) {
        throw std::invalid_argument("Invalid map-light collection");
    }
    for (const MapPointLight& light : lights) {
        if (!std::all_of(light.position.begin(), light.position.end(), finite) ||
            !std::all_of(light.color.begin(), light.color.end(),
                         [](double value) {
                             return std::isfinite(value) && value >= 0.0 && value <= 1.0;
                         }) ||
            !finite(light.radius) || light.radius <= 0.0) {
            throw std::invalid_argument("Invalid map light");
        }
    }
    GpuMapLightData gpuLights;
    for (std::size_t index = 0; index < lights.size(); ++index) {
        const MapPointLight& light = lights[index];
        gpuLights.positionRadius[index] = {
            static_cast<float>(light.position[0]), static_cast<float>(light.position[1]),
            static_cast<float>(light.position[2]), static_cast<float>(light.radius)};
        gpuLights.color[index] = {static_cast<float>(light.color[0]),
                                  static_cast<float>(light.color[1]),
                                  static_cast<float>(light.color[2]), 0.0F};
    }
    pglBindBuffer(GL_UNIFORM_BUFFER, mapLightBuffer);
    pglBufferData(GL_UNIFORM_BUFFER, sizeof(gpuLights), &gpuLights, GL_DYNAMIC_DRAW);
    pglBindBufferBase(GL_UNIFORM_BUFFER, mapLightBinding, mapLightBuffer);
    pglBindBuffer(GL_UNIFORM_BUFFER, 0);
    const GLuint previousProgram = currentProgram;
    for (const auto& entry :
         {std::pair{modelProgram, &modelUniforms}, std::pair{materialProgram, &materialUniforms}}) {
        useProgram(entry.first);
        const Uniforms& uniforms = *entry.second;
        pglUniform1i(uniforms.mapLightCount, static_cast<GLint>(lights.size()));
        pglUniform1f(uniforms.mapLightStrength, static_cast<float>(strength));
    }
    useProgram(previousProgram);
}

std::array<float, 3> weatherBackgroundColor(std::optional<double> brightness) {
    if (brightness && (!std::isfinite(*brightness) || *brightness < 0.0 || *brightness > 1.0)) {
        throw std::invalid_argument("Weather brightness must be finite and between 0 and 1");
    }
    const float value = static_cast<float>(brightness.value_or(1.0));
    return {0.45F * value, 0.65F * value, 0.88F * value};
}

void setWeatherLighting(std::optional<double> brightness, bool reflectionPass) {
    const auto color = weatherBackgroundColor(brightness);
    const float value = static_cast<float>(brightness.value_or(1.0));
    const float ambientLight =
        std::max(0.58F * value, reflectionPass ? 0.25F : 0.0F);
    const GLuint previousProgram = currentProgram;
    for (const auto& entry :
         {std::pair{modelProgram, &modelUniforms}, std::pair{materialProgram, &materialUniforms},
          std::pair{environmentProgram, &environmentUniforms}}) {
        if (entry.first == 0) {
            continue;
        }
        useProgram(entry.first);
        pglUniform1f(entry.second->ambientLight, ambientLight);
        pglUniform1f(entry.second->diffuseLight, 0.42F * value);
        pglUniform1f(entry.second->weatherBrightness, value);
        pglUniform3f(entry.second->fogColor, color[0], color[1], color[2]);
    }
    useProgram(previousProgram);
}

void drawWeatherClouds(std::optional<openbus::WeatherCloudType> type,
                       std::optional<double> baseAboveGroundMeters,
                       std::optional<double> windDirectionDegrees,
                       std::optional<double> windSpeedMetersPerSecond,
                       std::optional<double> visibilityMeters, std::optional<double> brightness,
                       double groundHeightMeters, double simulationSeconds) {
    if (!type) {
        return;
    }
    openbus::WeatherState settings;
    settings.cloudType = type;
    settings.cloudBaseAboveGroundMeters = baseAboveGroundMeters;
    settings.windDirectionDegrees = windDirectionDegrees;
    settings.windSpeedMetersPerSecond = windSpeedMetersPerSecond;
    settings.visibilityMeters = visibilityMeters;
    settings.brightness = brightness;
    openbus::validateWeatherState(settings);
    if (!std::isfinite(groundHeightMeters) || !std::isfinite(simulationSeconds)) {
        throw std::invalid_argument("Cloud ground height and simulation time must be finite");
    }
    const Matrix4& view = modelViewMatrix();
    const Matrix4& projection = projectionMatrix();
    const double eyeX = -(view[0] * view[12] + view[1] * view[13] + view[2] * view[14]);
    const double eyeY = -(view[4] * view[12] + view[5] * view[13] + view[6] * view[14]);
    const double eyeZ = -(view[8] * view[12] + view[9] * view[13] + view[10] * view[14]);
    const double direction = windDirectionDegrees.value_or(0.0) * 3.14159265358979323846 / 180.0;
    const double distance = windDirectionDegrees && windSpeedMetersPerSecond
                                ? *windSpeedMetersPerSecond * simulationSeconds
                                : 0.0;
    const double originX = eyeX + std::sin(direction) * distance;
    const double originY = eyeY + std::cos(direction) * distance;
    const double height = groundHeightMeters + baseAboveGroundMeters.value_or(1500.0) - eyeZ;
    if (!std::isfinite(originX) || !std::isfinite(originY) || !std::isfinite(height) ||
        !std::isfinite(projection[0]) || !std::isfinite(projection[5]) || projection[0] == 0.0 ||
        projection[5] == 0.0) {
        throw std::invalid_argument("Cloud camera or wind state is invalid");
    }
    std::array<GLfloat, 16> viewFloat = {};
    for (std::size_t i = 0; i < view.size(); ++i) {
        viewFloat[i] = static_cast<GLfloat>(view[i]);
    }
    const auto background = weatherBackgroundColor(brightness);
    const GLboolean depthTest = glIsEnabled(GL_DEPTH_TEST);
    const GLboolean blend = glIsEnabled(GL_BLEND);
    const GLboolean cull = glIsEnabled(GL_CULL_FACE);
    GLboolean depthWrite = GL_FALSE;
    glGetBooleanv(GL_DEPTH_WRITEMASK, &depthWrite);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
    glDisable(GL_CULL_FACE);
    glDepthMask(GL_FALSE);
    useProgram(cloudProgram);
    pglBindVertexArray(primitiveVertexArray);
    currentVertexArray = primitiveVertexArray;
    pglUniformMatrix4fv(cloudView, 1, GL_FALSE, viewFloat.data());
    pglUniform2f(cloudProjectionScale, static_cast<float>(projection[0]),
                 static_cast<float>(projection[5]));
    constexpr double cloudPeriodMeters = 1800.0 * 1024.0;
    pglUniform2f(cloudOrigin, static_cast<float>(std::fmod(originX, cloudPeriodMeters)),
                 static_cast<float>(std::fmod(originY, cloudPeriodMeters)));
    pglUniform1f(cloudHeight,
                 static_cast<float>(std::clamp(height, -openbus::MAX_CLOUD_BASE_ABOVE_GROUND_METERS,
                                               openbus::MAX_CLOUD_BASE_ABOVE_GROUND_METERS)));
    pglUniform1i(cloudStyle, static_cast<int>(*type));
    pglUniform1f(fogUniforms[3].density,
                 visibilityMeters
                     ? static_cast<float>(3.912023005428146 / std::max(*visibilityMeters, 1.0))
                     : 0.0F);
    pglUniform1f(cloudBrightness, static_cast<float>(brightness.value_or(1.0)));
    pglUniform3f(cloudBackground, background[0], background[1], background[2]);
    glDrawArrays(GL_TRIANGLES, 0, 3);
    glDepthMask(depthWrite);
    if (depthTest)
        glEnable(GL_DEPTH_TEST);
    if (blend)
        glEnable(GL_BLEND);
    if (cull)
        glEnable(GL_CULL_FACE);
}

void drawWeatherPrecipitation(int type, double rate, double simulationSeconds,
                              std::optional<double> windDirectionDegrees,
                              std::optional<double> windSpeedMetersPerSecond,
                              std::optional<double> brightness,
                              std::optional<double> visibilityMeters) {
    openbus::WeatherState settings;
    settings.precipitationType = type;
    settings.precipitationRate = rate;
    settings.windDirectionDegrees = windDirectionDegrees;
    settings.windSpeedMetersPerSecond = windSpeedMetersPerSecond;
    settings.brightness = brightness;
    settings.visibilityMeters = visibilityMeters;
    openbus::validateWeatherState(settings);
    if (!std::isfinite(simulationSeconds) || simulationSeconds < 0.0)
        throw std::invalid_argument(
            "Precipitation simulation time must be finite and non-negative");
    if (type == 0 || rate == 0.0)
        return;

    const Matrix4& view = modelViewMatrix();
    const Matrix4& projection = projectionMatrix();
    const std::array<double, 3> eye = {
        -(view[0] * view[12] + view[1] * view[13] + view[2] * view[14]),
        -(view[4] * view[12] + view[5] * view[13] + view[6] * view[14]),
        -(view[8] * view[12] + view[9] * view[13] + view[10] * view[14])};
    std::array<GLfloat, 16> rotation = {};
    std::array<GLfloat, 16> projectionFloat = {};
    for (std::size_t i = 0; i < view.size(); ++i) {
        if (!std::isfinite(view[i]) || !std::isfinite(projection[i]) ||
            std::abs(view[i]) > std::numeric_limits<float>::max() ||
            std::abs(projection[i]) > std::numeric_limits<float>::max())
            throw std::invalid_argument("Precipitation camera matrices are invalid");
        rotation[i] = static_cast<float>(view[i]);
        projectionFloat[i] = static_cast<float>(projection[i]);
    }
    for (double coordinate : eye) {
        if (!std::isfinite(coordinate))
            throw std::invalid_argument("Precipitation camera position is invalid");
    }
    rotation[12] = rotation[13] = rotation[14] = 0.0F;
    const double direction = windDirectionDegrees.value_or(0.0) * 3.14159265358979323846 / 180.0;
    const double speed = windDirectionDegrees && windSpeedMetersPerSecond
                             ? std::min(*windSpeedMetersPerSecond, 50.0)
                             : 0.0;
    const double windX = -std::sin(direction) * speed;
    const double windY = -std::cos(direction) * speed;
    const auto offset = [simulationSeconds](double velocity) {
        return velocity == 0.0 ? 0.0
                               : std::fmod(simulationSeconds, 24.0 / std::abs(velocity)) * velocity;
    };
    const double fallSpeed = type == 1 ? -9.0 : -1.2;
    const GLuint previousProgram = currentProgram;
    const GLuint previousVertexArray = currentVertexArray;
    const GLboolean depthTest = glIsEnabled(GL_DEPTH_TEST);
    const GLboolean blend = glIsEnabled(GL_BLEND);
    const GLboolean programPointSize = glIsEnabled(GL_PROGRAM_POINT_SIZE);
    GLboolean depthWrite = GL_FALSE;
    GLint depthFunction = 0;
    GLint srcRgb = 0, dstRgb = 0, srcAlpha = 0, dstAlpha = 0, equationRgb = 0, equationAlpha = 0;
    GLfloat lineWidth = 0, pointSize = 0;
    glGetBooleanv(GL_DEPTH_WRITEMASK, &depthWrite);
    glGetIntegerv(GL_DEPTH_FUNC, &depthFunction);
    glGetIntegerv(GL_BLEND_SRC_RGB, &srcRgb);
    glGetIntegerv(GL_BLEND_DST_RGB, &dstRgb);
    glGetIntegerv(GL_BLEND_SRC_ALPHA, &srcAlpha);
    glGetIntegerv(GL_BLEND_DST_ALPHA, &dstAlpha);
    glGetIntegerv(GL_BLEND_EQUATION_RGB, &equationRgb);
    glGetIntegerv(GL_BLEND_EQUATION_ALPHA, &equationAlpha);
    glGetFloatv(GL_LINE_WIDTH, &lineWidth);
    glGetFloatv(GL_POINT_SIZE, &pointSize);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glDepthMask(GL_FALSE);
    glEnable(GL_BLEND);
    pglBlendEquationSeparate(GL_FUNC_ADD, GL_FUNC_ADD);
    pglBlendFuncSeparate(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA, GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
    glDisable(GL_PROGRAM_POINT_SIZE);
    glLineWidth(1.0F);
    glPointSize(2.0F);
    useProgram(precipitationProgram);
    pglBindVertexArray(precipitationVertexArray);
    currentVertexArray = precipitationVertexArray;
    pglUniformMatrix4fv(precipitationView, 1, GL_FALSE, rotation.data());
    pglUniformMatrix4fv(precipitationProjection, 1, GL_FALSE, projectionFloat.data());
    pglUniform3f(precipitationEye, static_cast<float>(std::fmod(eye[0], 24.0)),
                 static_cast<float>(std::fmod(eye[1], 24.0)),
                 static_cast<float>(std::fmod(eye[2], 24.0)));
    pglUniform3f(precipitationOffset, static_cast<float>(offset(windX)),
                 static_cast<float>(offset(windY)), static_cast<float>(offset(fallSpeed)));
    pglUniform2f(precipitationWind, static_cast<float>(windX), static_cast<float>(windY));
    pglUniform1i(precipitationType, type);
    pglUniform1f(precipitationBrightness, static_cast<float>(brightness.value_or(1.0)));
    pglUniform1f(fogUniforms[4].density,
                 visibilityMeters
                     ? static_cast<float>(3.912023005428146 / std::max(*visibilityMeters, 1.0))
                     : 0.0F);
    const GLsizei particles = static_cast<GLsizei>(std::lround(rate * (type == 1 ? 12000 : 6000)));
    glDrawArrays(type == 1 ? GL_LINES : GL_POINTS, 0, particles * (type == 1 ? 2 : 1));
    glLineWidth(lineWidth);
    glPointSize(pointSize);
    if (programPointSize)
        glEnable(GL_PROGRAM_POINT_SIZE);
    pglBlendFuncSeparate(srcRgb, dstRgb, srcAlpha, dstAlpha);
    pglBlendEquationSeparate(equationRgb, equationAlpha);
    if (!blend)
        glDisable(GL_BLEND);
    glDepthFunc(depthFunction);
    glDepthMask(depthWrite);
    if (!depthTest)
        glDisable(GL_DEPTH_TEST);
    pglBindVertexArray(previousVertexArray);
    currentVertexArray = previousVertexArray;
    useProgram(previousProgram);
}

void configureModelAlphaState(int alphaMode, bool noZwrite) {
    if (alphaMode == 2 || noZwrite) {
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glDepthMask(alphaMode == 2 && !noZwrite ? GL_TRUE : GL_FALSE);
    } else if (alphaMode == 1) {
        glDisable(GL_BLEND);
        // Surviving cutout fragments are opaque and must occlude later draws.
        glDepthMask(GL_TRUE);
        glEnable(GL_POLYGON_OFFSET_FILL);
        glPolygonOffset(-1.0F, -1.0F);
    } else {
        glDisable(GL_BLEND);
        glDepthMask(GL_TRUE);
    }
}

void drawModelBatch(GLuint buffer, std::size_t vertexCount, const ModelMaterial& material,
                    const std::array<double, 3>& color, double alpha, int alphaMode,
                    bool cullBackFaces, std::optional<HumanGaitParameters> humanGait,
                    double visibilityFade, const HumanSkinDraw* humanSkin) {
    TraceScope trace("render", "CoreRenderer::drawModelBatch");
    if (modelProgram == 0 || buffer == 0 || vertexCount == 0) {
        return;
    }
    useProgram(modelProgram);
    uploadMatrices(modelUniforms);
    uploadModelUniforms(material, color, alpha, alphaMode, visibilityFade);
    pglUniform1i(modelUniforms.cullBackFaces, cullBackFaces && !modelViewReversesWinding() ? 1 : 0);
    const bool skinHuman =
        humanSkin != nullptr && humanSkin->weightBuffer != 0 && humanSkin->palette != nullptr;
    pglUniform1i(modelUniforms.humanSkinEnabled, skinHuman ? 1 : 0);
    if (skinHuman) {
        std::array<GLfloat, HUMAN_BONE_COUNT * 16> matrices = {};
        for (std::size_t bone = 0; bone < HUMAN_BONE_COUNT; ++bone) {
            for (std::size_t component = 0; component < 16; ++component) {
                matrices[bone * 16 + component] =
                    static_cast<GLfloat>((*humanSkin->palette)[bone][component]);
            }
        }
        pglUniformMatrix4fv(modelUniforms.humanBones, static_cast<GLsizei>(HUMAN_BONE_COUNT),
                            GL_FALSE, matrices.data());
    }
    const bool animateHuman = !skinHuman && humanGait.has_value() && humanGait->heightMeters > 0.0;
    pglUniform1i(modelUniforms.humanGaitEnabled, animateHuman ? 1 : 0);
    if (animateHuman) {
        pglUniform1f(modelUniforms.humanGaitPhase, static_cast<GLfloat>(humanGait->phaseRadians));
        pglUniform1f(modelUniforms.humanGaitWeight,
                     static_cast<GLfloat>(std::clamp(humanGait->weight, 0.0, 1.0)));
        pglUniform1f(modelUniforms.humanGaitHeight, static_cast<GLfloat>(humanGait->heightMeters));
        pglUniform1f(modelUniforms.humanUpperArmBeta,
                     static_cast<GLfloat>(humanGait->upperArmBetaDegrees));
        pglUniform1f(modelUniforms.humanArmSwing, static_cast<GLfloat>(humanGait->armSwing));
        pglUniform1f(modelUniforms.humanHipSway, static_cast<GLfloat>(humanGait->hipSway));
        pglUniform1f(modelUniforms.humanWaistAnimation,
                     static_cast<GLfloat>(humanGait->waistAnimation));
        pglUniform1f(modelUniforms.humanShoulderXFraction,
                     static_cast<GLfloat>(humanGait->shoulderXHeightFraction));
        pglUniform1f(modelUniforms.humanShoulderZFraction,
                     static_cast<GLfloat>(humanGait->shoulderZHeightFraction));
        pglUniform1f(modelUniforms.humanHipXFraction,
                     static_cast<GLfloat>(humanGait->hipXHeightFraction));
        pglUniform1f(modelUniforms.humanHipZFraction,
                     static_cast<GLfloat>(humanGait->hipZHeightFraction));
        pglUniform1f(modelUniforms.humanKneeXFraction,
                     static_cast<GLfloat>(humanGait->kneeXHeightFraction));
        pglUniform1f(modelUniforms.humanKneeYFraction,
                     static_cast<GLfloat>(humanGait->kneeYHeightFraction));
        pglUniform1f(modelUniforms.humanKneeZFraction,
                     static_cast<GLfloat>(humanGait->kneeZHeightFraction));
        pglUniform1f(modelUniforms.humanElbowXFraction,
                     static_cast<GLfloat>(humanGait->elbowXHeightFraction));
        pglUniform1f(modelUniforms.humanElbowZFraction,
                     static_cast<GLfloat>(humanGait->elbowZHeightFraction));
        pglUniform1f(modelUniforms.humanHandXFraction,
                     static_cast<GLfloat>(humanGait->handXHeightFraction));
        pglUniform1f(modelUniforms.humanHandZFraction,
                     static_cast<GLfloat>(humanGait->handZHeightFraction));
        pglUniform1f(modelUniforms.humanFingerXFraction,
                     static_cast<GLfloat>(humanGait->fingerXHeightFraction));
        pglUniform1f(modelUniforms.humanFingerZFraction,
                     static_cast<GLfloat>(humanGait->fingerZHeightFraction));
        pglUniform1f(modelUniforms.humanNeckYFraction,
                     static_cast<GLfloat>(humanGait->neckYHeightFraction));
        pglUniform1f(modelUniforms.humanNeckZFraction,
                     static_cast<GLfloat>(humanGait->neckZHeightFraction));
        pglUniform1f(modelUniforms.humanWaistYFraction,
                     static_cast<GLfloat>(humanGait->waistYHeightFraction));
        pglUniform1f(modelUniforms.humanWaistZFraction,
                     static_cast<GLfloat>(humanGait->waistZHeightFraction));
    }
    selectTextureUnit(GL_TEXTURE0);
    if (material.textureArray) {
        selectTextureUnit(GL_TEXTURE0 + 7);
        bindTexture(GL_TEXTURE_2D_ARRAY, material.texture);
        if (material.texture != 0) {
            setTextureWrap(GL_TEXTURE_2D_ARRAY, material.texture, material.textureWrapS,
                           material.textureWrapT);
        }
        selectTextureUnit(GL_TEXTURE0);
    } else {
        bindTexture(GL_TEXTURE_2D, material.texture);
        if (material.texture != 0) {
            setTextureWrap(GL_TEXTURE_2D, material.texture, material.textureWrapS,
                           material.textureWrapT);
        }
    }
    selectTextureUnit(GL_TEXTURE1);
    bindTexture(GL_TEXTURE_2D, material.lightmap);
    selectTextureUnit(GL_TEXTURE2);
    bindTexture(GL_TEXTURE_2D, material.nightmap);
    selectTextureUnit(GL_TEXTURE3);
    bindTexture(GL_TEXTURE_2D, material.transmap);
    selectTextureUnit(GL_TEXTURE0 + 6);
    bindTexture(GL_TEXTURE_2D, material.bumpmap);
    selectTextureUnit(GL_TEXTURE4);
    bindTexture(GL_TEXTURE_2D, material.freeTexture);
    selectTextureUnit(GL_TEXTURE0 + 5);
    bindTexture(GL_TEXTURE_2D, material.textTexture);
    selectTextureUnit(GL_TEXTURE8);
    bindTexture(GL_TEXTURE_2D, material.detailTexture);
    selectTextureUnit(GL_TEXTURE0 + 9);
    bindTexture(GL_TEXTURE_2D, material.environmentMap);
    selectTextureUnit(GL_TEXTURE0);
    bindModelVertexBuffer(buffer, modelVertexArray);
    if (skinHuman) {
        pglBindBuffer(GL_ARRAY_BUFFER, humanSkin->weightBuffer);
        currentArrayBuffer = humanSkin->weightBuffer;
        for (GLuint attribute = 0; attribute < 4; ++attribute) {
            pglEnableVertexAttribArray(3 + attribute);
            pglVertexAttribPointer(3 + attribute, 4, GL_FLOAT, GL_FALSE,
                                   static_cast<GLsizei>(HUMAN_WEIGHT_COUNT * sizeof(float)),
                                   reinterpret_cast<const void*>(
                                       static_cast<std::uintptr_t>(attribute * 4 * sizeof(float))));
        }
    }
    recordTriangleSubmission(vertexCount / 3);
    if (!drawSelectedIndirect(false))
        glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(vertexCount));
    if (skinHuman) {
        for (GLuint attribute = 0; attribute < 4; ++attribute) {
            pglDisableVertexAttribArray(3 + attribute);
        }
    }
}

void drawIndexedModelBatch(GLuint buffer, GLuint indexBuffer, std::size_t indexCount,
                           const ModelMaterial& material, const std::array<double, 3>& color,
                           double alpha, int alphaMode, bool cullBackFaces, double visibilityFade) {
    TraceScope trace("render", "CoreRenderer::drawIndexedModelBatch");
    if (modelProgram == 0 || buffer == 0 || indexBuffer == 0 || indexCount == 0) {
        return;
    }
    useProgram(modelProgram);
    uploadMatrices(modelUniforms);
    uploadModelUniforms(material, color, alpha, alphaMode, visibilityFade);
    pglUniform1i(modelUniforms.cullBackFaces, cullBackFaces && !modelViewReversesWinding() ? 1 : 0);
    pglUniform1i(modelUniforms.humanGaitEnabled, GL_FALSE);
    pglUniform1i(modelUniforms.humanSkinEnabled, GL_FALSE);
    selectTextureUnit(GL_TEXTURE0);
    if (material.textureArray) {
        selectTextureUnit(GL_TEXTURE0 + 7);
        bindTexture(GL_TEXTURE_2D_ARRAY, material.texture);
        if (material.texture != 0) {
            setTextureWrap(GL_TEXTURE_2D_ARRAY, material.texture, material.textureWrapS,
                           material.textureWrapT);
        }
        selectTextureUnit(GL_TEXTURE0);
    } else {
        bindTexture(GL_TEXTURE_2D, material.texture);
        if (material.texture != 0) {
            setTextureWrap(GL_TEXTURE_2D, material.texture, material.textureWrapS,
                           material.textureWrapT);
        }
    }
    selectTextureUnit(GL_TEXTURE1);
    bindTexture(GL_TEXTURE_2D, material.lightmap);
    selectTextureUnit(GL_TEXTURE2);
    bindTexture(GL_TEXTURE_2D, material.nightmap);
    selectTextureUnit(GL_TEXTURE3);
    bindTexture(GL_TEXTURE_2D, material.transmap);
    selectTextureUnit(GL_TEXTURE0 + 6);
    bindTexture(GL_TEXTURE_2D, material.bumpmap);
    selectTextureUnit(GL_TEXTURE4);
    bindTexture(GL_TEXTURE_2D, material.freeTexture);
    selectTextureUnit(GL_TEXTURE0 + 5);
    bindTexture(GL_TEXTURE_2D, material.textTexture);
    selectTextureUnit(GL_TEXTURE8);
    bindTexture(GL_TEXTURE_2D, material.detailTexture);
    selectTextureUnit(GL_TEXTURE0 + 9);
    bindTexture(GL_TEXTURE_2D, material.environmentMap);
    selectTextureUnit(GL_TEXTURE0);
    bindModelVertexBuffer(buffer, modelVertexArray);
    pglBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexBuffer);
    recordTriangleSubmission(indexCount / 3);
    if (!drawSelectedIndirect(true))
        glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(indexCount), GL_UNSIGNED_INT, nullptr);
}

void drawMaterialBatch(GLuint buffer, std::size_t vertexCount,
                       const std::vector<std::array<float, 4>>& colors,
                       const std::vector<GLuint>& textures, const std::vector<bool>& flipTextureY,
                       GLuint indexBuffer, double visibilityFade) {
    TraceScope trace("render", "CoreRenderer::drawMaterialBatch");
    if (materialProgram == 0 || buffer == 0 || vertexCount == 0 || colors.empty() ||
        textures.empty()) {
        return;
    }
    useProgram(materialProgram);
    uploadMatrices(materialUniforms);
    pglUniform1f(materialUniforms.visibilityFade,
                 static_cast<GLfloat>(std::clamp(visibilityFade, 0.0, 1.0)));
    for (std::size_t index = 0; index < MAX_MATERIAL_TEXTURES; ++index) {
        pglUniform1i(materialUniforms.materialFlipTextureY[index],
                     flipTextureY.size() > index && flipTextureY[index] ? 1 : 0);
    }
    const GLsizei colorCount = static_cast<GLsizei>(std::min<std::size_t>(colors.size(), 128));
    if (!materialUniformState.valid || materialUniformState.colorData != colors.data() ||
        materialUniformState.colorCount != colorCount) {
        pglUniform4fv(materialUniforms.materialColors, colorCount,
                      reinterpret_cast<const GLfloat*>(colors.data()));
        materialUniformState.valid = true;
        materialUniformState.colorData = colors.data();
        materialUniformState.colorCount = colorCount;
    }
    const std::size_t textureCount = std::min<std::size_t>(textures.size(), MAX_MATERIAL_TEXTURES);
    for (std::size_t index = 0; index < textureCount; ++index) {
        selectTextureUnit(GL_TEXTURE0 + static_cast<GLenum>(index));
        bindTexture(GL_TEXTURE_2D, textures[index]);
    }
    selectTextureUnit(GL_TEXTURE0);
    bindModelVertexBuffer(buffer, modelVertexArray);
    if (indexBuffer != 0) {
        pglBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexBuffer);
    }
    recordTriangleSubmission(vertexCount / 3);
    if (!drawSelectedIndirect(indexBuffer != 0)) {
        if (indexBuffer != 0) {
            glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(vertexCount), GL_UNSIGNED_INT,
                           nullptr);
        } else {
            glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(vertexCount));
        }
    }
}

void drawEnvironmentBatch(GLuint buffer, std::size_t vertexCount, GLuint texture, double alpha) {
    TraceScope trace("render", "CoreRenderer::drawEnvironmentBatch");
    if (environmentProgram == 0 || buffer == 0 || texture == 0 || vertexCount == 0) {
        return;
    }
    useProgram(environmentProgram);
    uploadMatrices(environmentUniforms);
    pglUniform1f(environmentUniforms.environmentAlpha, static_cast<GLfloat>(alpha));
    selectTextureUnit(GL_TEXTURE0);
    bindTexture(GL_TEXTURE_2D, texture);
    bindModelVertexBuffer(buffer, modelVertexArray);
    recordTriangleSubmission(vertexCount / 3);
    if (!drawSelectedIndirect(false))
        glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(vertexCount));
}

void drawPrimitives(const std::vector<PrimitiveVertex>& vertices, GLenum primitive,
                    float lineWidth) {
    TraceScope trace("render", "CoreRenderer::drawPrimitives");
    if (primitiveProgram == 0 || vertices.empty()) {
        return;
    }
    useProgram(primitiveProgram);
    uploadMatrices(primitiveUniforms);
    if (currentVertexArray != primitiveVertexArray) {
        pglBindVertexArray(primitiveVertexArray);
        currentVertexArray = primitiveVertexArray;
    }
    // Asset uploads also bind GL_ARRAY_BUFFER outside this renderer's cache.
    pglBindBuffer(GL_ARRAY_BUFFER, primitiveBuffer);
    currentArrayBuffer = primitiveBuffer;
    {
        TraceScope phase("render", "CoreRenderer::drawPrimitives.upload");
        pglBufferData(GL_ARRAY_BUFFER,
                      static_cast<std::ptrdiff_t>(vertices.size() * sizeof(PrimitiveVertex)),
                      vertices.data(), GL_STREAM_DRAW);
        pglVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(PrimitiveVertex), nullptr);
        pglVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(PrimitiveVertex),
                               reinterpret_cast<const void*>(3 * sizeof(float)));
    }
    glLineWidth(lineWidth);
    {
        TraceScope phase("render", "CoreRenderer::drawPrimitives.submit");
        recordTriangleSubmission(trianglesForPrimitive(primitive, vertices.size()));
        glDrawArrays(primitive, 0, static_cast<GLsizei>(vertices.size()));
    }
}

bool drawStaticPrimitives(StaticPrimitiveBuffer& cache,
                          const std::vector<PrimitiveVertex>& vertices, GLenum primitive,
                          float lineWidth, std::uint64_t contentRevision,
                          bool useIndirectOcclusionProxy) {
    TraceScope trace("render", "CoreRenderer::drawStaticPrimitives");
    if (primitiveProgram == 0 || vertices.empty()) {
        return false;
    }
    if (cache.rendererGeneration != rendererGeneration) {
        cache = {};
        cache.rendererGeneration = rendererGeneration;
    }
    useProgram(primitiveProgram);
    uploadMatrices(primitiveUniforms);
    const bool needsUpload = cache.requiresUpload(vertices.size(), contentRevision);
    if (cache.buffer == 0) {
        pglGenBuffers(1, &cache.buffer);
        if (cache.buffer == 0) {
            return false;
        }
        staticPrimitiveBuffers.push_back(cache.buffer);
    }
    if (currentVertexArray != primitiveVertexArray) {
        pglBindVertexArray(primitiveVertexArray);
        currentVertexArray = primitiveVertexArray;
    }
    pglBindBuffer(GL_ARRAY_BUFFER, cache.buffer);
    currentArrayBuffer = cache.buffer;
    if (needsUpload) {
        {
            TraceScope phase("render", "CoreRenderer::drawStaticPrimitives.upload");
            pglBufferData(GL_ARRAY_BUFFER,
                          static_cast<std::ptrdiff_t>(vertices.size() * sizeof(PrimitiveVertex)),
                          vertices.data(), GL_STATIC_DRAW);
        }
        if (glGetError() != GL_NO_ERROR) {
            gameLog.Warning("OpenGL error during or before static primitive upload; "
                            "skipping this primitive and retrying the upload later");
            return false;
        }
        cache.vertexCount = vertices.size();
        cache.contentRevision = contentRevision;
    }
    pglVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(PrimitiveVertex), nullptr);
    pglVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(PrimitiveVertex),
                           reinterpret_cast<const void*>(3 * sizeof(float)));
    glLineWidth(lineWidth);
    {
        TraceScope phase("render", "CoreRenderer::drawStaticPrimitives.submit");
        recordTriangleSubmission(trianglesForPrimitive(primitive, cache.vertexCount));
        if (!useIndirectOcclusionProxy || !drawSelectedIndirectProxy()) {
            glDrawArrays(primitive, 0, static_cast<GLsizei>(cache.vertexCount));
        }
    }
    return true;
}

void releaseStaticPrimitiveBuffer(StaticPrimitiveBuffer& cache) {
    if (cache.buffer != 0 && cache.rendererGeneration == rendererGeneration) {
        const auto found =
            std::find(staticPrimitiveBuffers.begin(), staticPrimitiveBuffers.end(), cache.buffer);
        if (found != staticPrimitiveBuffers.end()) {
            deleteRendererBuffers(1, &cache.buffer);
            staticPrimitiveBuffers.erase(found);
        }
    }
    cache = {};
}

void drawTextureQuad(GLuint texture, float x, float y, float width, float height,
                     bool flipTextureY) {
    if (textureQuadProgram == 0 || textureQuadVertexArray == 0 || textureQuadBuffer == 0 ||
        texture == 0) {
        return;
    }
    constexpr std::array<float, 16> vertices = {0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f,
                                                0.0f, 1.0f, 0.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
    useProgram(textureQuadProgram);
    pglBindVertexArray(textureQuadVertexArray);
    currentVertexArray = textureQuadVertexArray;
    pglBindBuffer(GL_ARRAY_BUFFER, textureQuadBuffer);
    currentArrayBuffer = textureQuadBuffer;
    pglBufferData(GL_ARRAY_BUFFER, static_cast<std::ptrdiff_t>(vertices.size() * sizeof(float)),
                  vertices.data(), GL_STREAM_DRAW);
    pglVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), nullptr);
    pglVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float),
                           reinterpret_cast<const void*>(2 * sizeof(float)));
    pglUniform4f(textureQuadRect, x, y, width, height);
    pglUniform1i(textureQuadFlipTextureY, flipTextureY ? 1 : 0);
    selectTextureUnit(GL_TEXTURE0);
    bindTexture(GL_TEXTURE_2D, texture);
    recordTriangleSubmission(trianglesForPrimitive(GL_TRIANGLE_STRIP, 4));
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
}

} // namespace openbus::rendering
