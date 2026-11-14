#include "render/CoreRenderer.h"
#include "render/StaticMapLightFallback.h"
#include "render/VehicleSpotlightTileMask.h"
#include "render/ComputeCulling.h"
#include "render/MatrixMath.h"
#include "core/Environment.h"

#include "core/Logger.h"
#include "render/OpenGLFunctions.h"
#include "core/PerfTrace.h"
#include "SmaaShader.h"
#include <AreaTex.h>
#include <SearchTex.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <bit>
#include <chrono>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

#ifndef GL_TEXTURE_BUFFER
#define GL_TEXTURE_BUFFER 0x8C2A
#endif
#ifndef GL_MAX_TEXTURE_BUFFER_SIZE
#define GL_MAX_TEXTURE_BUFFER_SIZE 0x8C2B
#endif
#ifndef GL_RGBA32F
#define GL_RGBA32F 0x8814
#endif
#ifndef GL_RGBA32UI
#define GL_RGBA32UI 0x8D70
#endif
#ifndef GL_MAJOR_VERSION
#define GL_MAJOR_VERSION 0x821B
#endif
#ifndef GL_MINOR_VERSION
#define GL_MINOR_VERSION 0x821C
#endif
#ifndef GL_DEBUG_OUTPUT_SYNCHRONOUS
#define GL_DEBUG_OUTPUT_SYNCHRONOUS 0x8242
#endif

namespace openbus::rendering {
namespace {

constexpr GLsizei MODEL_VERTEX_STRIDE = static_cast<GLsizei>(13 * sizeof(float));
constexpr std::size_t MAX_MATERIAL_TEXTURES = 8;
constexpr std::size_t MAX_TEXTURE_UNITS = 16;
std::size_t submittedFrameTriangleCount = 0;
std::size_t submittedReflectionTriangleCount = 0;
bool reflectionTrianglePass = false;
std::atomic<std::uint64_t> glDebugErrorGeneration = 0;
std::size_t configuredMapLightSelectionLimit = 0;

constexpr std::size_t kGpuTimerQueriesPerPass = 32;
constexpr std::size_t kGpuTimerPassCount = 2;
constexpr std::size_t kGpuTimerQueryCount = kGpuTimerQueriesPerPass * kGpuTimerPassCount;
constexpr auto kGpuTimerPollInterval = std::chrono::milliseconds(100);
constexpr auto kGpuTimerSummaryInterval = std::chrono::seconds(2);

std::size_t readLightLimit(const char* variableName, std::size_t maximum,
                           std::size_t defaultLimit) {
    const char* setting = openbus::getEnvironment(variableName);
    if (setting == nullptr) {
        return defaultLimit;
    }

    const std::string_view value(setting);
    unsigned int parsed = 0;
    const auto [end, error] = std::from_chars(value.data(), value.data() + value.size(), parsed);
    if (error != std::errc{} || end != value.data() + value.size() || parsed > maximum) {
        gameLog.Warning(std::string(variableName) + " must be an integer from 0 to " +
                        std::to_string(maximum) + "; using default limit " +
                        std::to_string(defaultLimit));
        return defaultLimit;
    }
    gameLog.Log(std::string(variableName) + " set to " + std::to_string(parsed));
    return parsed;
}

std::size_t mapLightLimit() {
    constexpr std::size_t maximum = std::numeric_limits<GLint>::max() / 2;
    struct EnvironmentOverride {
        bool present;
        std::size_t limit;
    };
    static const EnvironmentOverride environmentOverride = [] {
        const bool present = openbus::getEnvironment("OPENBUS_MAP_LIGHT_LIMIT") != nullptr;
        return EnvironmentOverride{present,
                                   readLightLimit("OPENBUS_MAP_LIGHT_LIMIT", maximum, maximum)};
    }();
    if (environmentOverride.present) {
        return environmentOverride.limit;
    }
    return configuredMapLightSelectionLimit == 0
               ? maximum
               : std::min(configuredMapLightSelectionLimit, maximum);
}

std::size_t vehicleSpotlightLimit() {
    constexpr std::size_t maximum = std::numeric_limits<GLint>::max() / 3;
    static const std::size_t limit =
        readLightLimit("OPENBUS_VEHICLE_SPOTLIGHT_LIMIT", maximum, maximum);
    return limit;
}

bool mapLightVertexEvaluationEnabled() {
    static const bool enabled = [] {
        const char* setting = openbus::getEnvironment("OPENBUS_MAP_LIGHT_VERTEX_EVALUATION");
        const std::string_view value = setting != nullptr ? setting : "";
        if (value.empty() || value == "0" || value == "false") {
            return false;
        }
        if (value == "1" || value == "true") {
            gameLog.Log("Vertex-evaluated scenery map lighting enabled");
            return true;
        }
        gameLog.Warning("OPENBUS_MAP_LIGHT_VERTEX_EVALUATION must be 0, 1, false or true; "
                        "using 0");
        return false;
    }();
    return enabled;
}

bool vehicleSpotlightTileCullingEnabled() {
    static const bool enabled = [] {
        const char* setting = openbus::getEnvironment("OPENBUS_VEHICLE_SPOTLIGHT_TILE_CULLING");
        const std::string_view value = setting != nullptr ? setting : "";
        if (value == "1" || value == "true") {
            gameLog.Log("Vehicle spotlight tile culling enabled");
            return true;
        }
        if (value.empty() || value == "0" || value == "false") {
            gameLog.Log("Vehicle spotlight tile culling disabled");
            return false;
        }
        gameLog.Warning("OPENBUS_VEHICLE_SPOTLIGHT_TILE_CULLING must be 0, 1, false or true; "
                        "using 0");
        return false;
    }();
    return enabled;
}

float reflectionAmbientFloor() {
    static const float floor = [] {
        constexpr float defaultFloor = 0.25F;
        const char* setting = openbus::getEnvironment("OPENBUS_REFLECTION_AMBIENT_FLOOR");
        if (setting == nullptr) {
            return defaultFloor;
        }

        const std::string_view value(setting);
        float parsed = 0.0F;
        const auto [end, error] =
            std::from_chars(value.data(), value.data() + value.size(), parsed);
        if (error != std::errc{} || end != value.data() + value.size() || !std::isfinite(parsed) ||
            parsed < 0.0F || parsed > 1.0F) {
            gameLog.Warning("OPENBUS_REFLECTION_AMBIENT_FLOOR must be a number from 0 to 1; "
                            "using default 0.25");
            return defaultFloor;
        }
        gameLog.Log("OPENBUS_REFLECTION_AMBIENT_FLOOR set to " + std::to_string(parsed));
        return parsed;
    }();
    return floor;
}

bool validMapLight(const MapPointLight& light) {
    const auto finite = [](double value) {
        return std::isfinite(value) && std::abs(value) <= std::numeric_limits<float>::max();
    };
    return std::all_of(light.position.begin(), light.position.end(), finite) &&
           std::all_of(
               light.color.begin(), light.color.end(),
               [](double value) { return std::isfinite(value) && value >= 0.0 && value <= 1.0; }) &&
           finite(light.radius) && light.radius >= std::numeric_limits<float>::min();
}

void APIENTRY reportOpenGlDebugMessage(GLenum, GLenum type, GLuint id, GLenum severity,
                                       GLsizei length, const char* message, const void*) {
    if (type != GL_DEBUG_TYPE_ERROR || message == nullptr) {
        return;
    }
    glDebugErrorGeneration.fetch_add(1, std::memory_order_relaxed);
    const std::size_t messageLength =
        length > 0 ? static_cast<std::size_t>(length) : std::char_traits<char>::length(message);
    gameLog.Error("OpenGL API error id=" + std::to_string(id) + " severity=" +
                  std::to_string(severity) + ": " + std::string(message, messageLength));
}

std::uint64_t glDebugErrorCheckpoint() {
    return glDebugErrorGeneration.load(std::memory_order_relaxed);
}

void validateOpenGlUpload(const char* description, std::uint64_t errorCheckpoint) {
    if (glDebugErrorGeneration.load(std::memory_order_relaxed) != errorCheckpoint) {
        throw std::runtime_error(std::string("OpenGL ") + description +
                                 " upload failed; see "
                                 "the OpenGL debug error log");
    }
}

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
LightingBufferUploadStats lightUploadStats;

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
    std::ostringstream lightingSummary;
    lightingSummary << "LIGHTING_UPLOAD interval_s=" << summaryElapsedSeconds
                    << " vehicle_uploads=" << lightUploadStats.vehicleUploads
                    << " vehicle_bytes=" << lightUploadStats.vehicleBytes
                    << " vehicle_sets=" << lightUploadStats.vehicleSets
                    << " vehicle_lights_total=" << lightUploadStats.vehicleLightsTotal
                    << " vehicle_lights_max=" << lightUploadStats.vehicleLightsMax
                    << " spotlight_tile_uploads=" << lightUploadStats.vehicleSpotlightTileUploads
                    << " spotlight_tile_bytes=" << lightUploadStats.vehicleSpotlightTileBytes
                    << " spotlight_tile_sets=" << lightUploadStats.vehicleSpotlightTileSets
                    << " spotlight_tile_count=" << lightUploadStats.vehicleSpotlightTileCountTotal
                    << " spotlight_tile_assignments="
                    << lightUploadStats.vehicleSpotlightTileAssignmentsTotal
                    << " spotlight_tile_max=" << lightUploadStats.vehicleSpotlightTileAssignmentsMax
                    << " spotlight_tile_fallback_sets="
                    << lightUploadStats.vehicleSpotlightTileFallbackSets
                    << " map_uploads=" << lightUploadStats.mapUploads
                    << " map_bytes=" << lightUploadStats.mapBytes
                    << " static_map_uploads=" << lightUploadStats.staticMapUploads
                    << " static_map_bytes=" << lightUploadStats.staticMapBytes
                    << " map_view_preparations=" << lightUploadStats.mapViewPreparations
                    << " map_view_reuses=" << lightUploadStats.mapViewReuses
                    << " map_sets=" << lightUploadStats.mapSets
                    << " map_lights_total=" << lightUploadStats.mapLightsTotal
                    << " map_lights_max=" << lightUploadStats.mapLightsMax;
    gameLog.Log(lightingSummary.str());
    lightUploadStats = {};
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
    GLint previousProjection = -1;
    GLint previousModelView = -1;
    GLint motionRenderSize = -1;
    GLint motionHistoryValid = -1;
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
    GLint spotlightTexture = -1;
    GLint spotlightTileMaskTexture = -1;
    GLint spotlightTileMaskEnabled = -1;
    GLint spotlightTileColumns = -1;
    GLint spotlightTileRows = -1;
    GLint spotlightTileWordsPerTile = -1;
    GLint spotlightTileSize = -1;
    GLint spotlightViewportOrigin = -1;
    GLint mapLightCount = -1;
    GLint mapLightStrength = -1;
    GLint mapLightTexture = -1;
    GLint staticMapLightTexture = -1;
    GLint staticMapViewTexture = -1;
    GLint staticMapMode = -1;
    GLint vertexLightEvaluation = -1;
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

static_assert(sizeof(std::array<float, 4>) == 4 * sizeof(float));
static_assert(sizeof(float) == sizeof(std::uint32_t));
static_assert(sizeof(std::array<std::uint32_t, 4>) == 4 * sizeof(std::uint32_t));

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
GLuint motionResolveProgram = 0;
GLuint smaaEdgeProgram = 0;
GLuint smaaWeightsProgram = 0;
GLuint smaaNeighborhoodProgram = 0;
GLuint cloudProgram = 0;
GLuint precipitationProgram = 0;
GLuint precipitationVertexArray = 0;
GLuint modelVertexArray = 0;
GLuint primitiveVertexArray = 0;
GLuint primitiveBuffer = 0;
GLuint vehicleSpotlightBuffer = 0;
GLuint vehicleSpotlightTexture = 0;
GLuint vehicleSpotlightTileMaskBuffer = 0;
GLuint vehicleSpotlightTileMaskTexture = 0;
std::size_t vehicleSpotlightCapacity = 0;
std::size_t vehicleSpotlightTileMaskCapacity = 0;
GLuint mapLightBuffer = 0;
GLuint mapLightTexture = 0;
GLuint staticMapViewBuffer = 0;
GLuint staticMapViewTexture = 0;
std::size_t mapLightCapacity = 0;
std::size_t staticMapLightCapacity = 0;
std::vector<std::array<float, 4>> uploadedVehicleLights;
std::vector<std::uint32_t> uploadedVehicleSpotlightTileMask;
std::vector<std::array<float, 4>> uploadedMapLights;
std::vector<std::array<std::uint32_t, 4>> uploadedStaticMapView;
constexpr GLenum mapLightTextureUnit = GL_TEXTURE0 + 10;
constexpr GLenum vehicleSpotlightTextureUnit = GL_TEXTURE0 + 11;
constexpr GLenum staticMapLightTextureUnit = GL_TEXTURE0 + 12;
constexpr GLenum staticMapViewTextureUnit = GL_TEXTURE0 + 13;
constexpr GLenum vehicleSpotlightTileMaskTextureUnit = GL_TEXTURE0 + 14;
enum class AntiAliasingMode { Off, Fxaa, Msaa, Smaa };
std::vector<GLuint> staticPrimitiveBuffers;
std::vector<std::pair<GLuint, GLuint>> staticMapLightBuffers;
std::uint64_t rendererGeneration = 0;
GLuint textureQuadVertexArray = 0;
GLuint textureQuadBuffer = 0;
struct MotionTarget {
    GLuint framebuffer = 0;
    GLuint color = 0;
    GLuint motionVectors = 0;
    GLuint validity = 0;
    GLuint depth = 0;
    GLuint multisampleFramebuffer = 0;
    GLuint multisampleColor = 0;
    GLuint multisampleMotionVectors = 0;
    GLuint multisampleValidity = 0;
    GLuint multisampleDepth = 0;
    GLuint smaaEdgeFramebuffer = 0;
    GLuint smaaEdges = 0;
    GLuint smaaWeightsFramebuffer = 0;
    GLuint smaaWeights = 0;
    int width = 0;
    int height = 0;
    int samples = 0;
    AntiAliasingMode mode = AntiAliasingMode::Off;
    GLint previousFramebuffer = 0;
    GLint previousViewport[4] = {};
    GLboolean previousDepthMask = GL_TRUE;
    GLboolean previousColorMask[4] = {GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE};
    GLboolean previousMultisampleEnabled = GL_TRUE;
};
MotionTarget motionTarget;
GLuint smaaAreaTexture = 0;
GLuint smaaSearchTexture = 0;
bool smaaLookupReady = false;
MotionHistory motionHistory;
MotionDataResources publishedMotionData;
std::uint64_t motionFrameId = 0;
bool motionFrameActive = false;
int failedMotionTargetWidth = 0;
int failedMotionTargetHeight = 0;
Uniforms modelUniforms;
Uniforms materialUniforms;
Uniforms environmentUniforms;
Uniforms primitiveUniforms;
struct FullscreenUniforms {
    GLint rect = -1;
    GLint flipTextureY = -1;
};
FullscreenUniforms motionResolveFullscreen;
FullscreenUniforms smaaEdgeFullscreen;
FullscreenUniforms smaaWeightsFullscreen;
FullscreenUniforms smaaNeighborhoodFullscreen;
GLint textureQuadRect = -1;
GLint textureQuadTexture = -1;
GLint textureQuadFlipTextureY = -1;
GLint textureQuadMotionValidity = -1;
GLint textureQuadMotionDebugMode = -1;
GLint textureQuadFxaaEnabled = -1;
GLint textureQuadInverseResolution = -1;
GLint motionResolveColorSamples = -1;
GLint motionResolveMotionSamples = -1;
GLint motionResolveValiditySamples = -1;
GLint motionResolveDepthSamples = -1;
GLint motionResolveSampleCount = -1;
GLint smaaEdgeColorTexture = -1;
GLint smaaEdgeMetrics = -1;
GLint smaaWeightsEdgeTexture = -1;
GLint smaaWeightsAreaTexture = -1;
GLint smaaWeightsSearchTexture = -1;
GLint smaaWeightsMetrics = -1;
GLint smaaNeighborhoodColorTexture = -1;
GLint smaaNeighborhoodWeightsTexture = -1;
GLint smaaNeighborhoodMetrics = -1;
AntiAliasingMode antiAliasingMode = AntiAliasingMode::Off;
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

void destroyMotionTarget() {
    const std::array<GLuint, 4> framebuffers = {
        motionTarget.framebuffer, motionTarget.multisampleFramebuffer,
        motionTarget.smaaEdgeFramebuffer, motionTarget.smaaWeightsFramebuffer};
    pglDeleteFramebuffers(static_cast<GLsizei>(framebuffers.size()), framebuffers.data());
    const std::array<GLuint, 10> textures = {
        motionTarget.color,
        motionTarget.motionVectors,
        motionTarget.validity,
        motionTarget.depth,
        motionTarget.multisampleColor,
        motionTarget.multisampleMotionVectors,
        motionTarget.multisampleValidity,
        motionTarget.multisampleDepth,
        motionTarget.smaaEdges,
        motionTarget.smaaWeights};
    glDeleteTextures(static_cast<GLsizei>(textures.size()), textures.data());
    motionTarget = {};
    publishedMotionData = {};
}

bool ensureMotionTarget(int width, int height) {
    const auto targetModeFor = [](AntiAliasingMode mode) {
        return mode == AntiAliasingMode::Msaa || mode == AntiAliasingMode::Smaa
                   ? mode
                   : AntiAliasingMode::Off;
    };
    if (motionTarget.framebuffer != 0 && motionTarget.width == width &&
        motionTarget.height == height && motionTarget.mode == targetModeFor(antiAliasingMode)) {
        return true;
    }
    if (failedMotionTargetWidth == width && failedMotionTargetHeight == height) {
        return false;
    }
    GLint previousFramebuffer = 0;
    glGetIntegerv(GL_FRAMEBUFFER_BINDING, &previousFramebuffer);
    destroyMotionTarget();

    pglGenFramebuffers(1, &motionTarget.framebuffer);
    pglBindFramebuffer(GL_FRAMEBUFFER, motionTarget.framebuffer);
    const auto createTexture = [&](GLuint& texture, GLint internalFormat, GLenum format,
                                   GLenum type, GLint filter) {
        glGenTextures(1, &texture);
        glBindTexture(GL_TEXTURE_2D, texture);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, filter);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, filter);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexImage2D(GL_TEXTURE_2D, 0, internalFormat, width, height, 0, format, type, nullptr);
    };
    createTexture(motionTarget.color, GL_RGBA8, GL_RGBA, GL_UNSIGNED_BYTE, GL_LINEAR);
    pglFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, motionTarget.color,
                            0);
    createTexture(motionTarget.motionVectors, GL_RG16F, GL_RG, GL_FLOAT, GL_NEAREST);
    pglFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_TEXTURE_2D,
                            motionTarget.motionVectors, 0);
    createTexture(motionTarget.validity, GL_R8, GL_RED, GL_UNSIGNED_BYTE, GL_NEAREST);
    pglFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT2, GL_TEXTURE_2D,
                            motionTarget.validity, 0);
    createTexture(motionTarget.depth, GL_DEPTH_COMPONENT24, GL_DEPTH_COMPONENT, GL_UNSIGNED_INT,
                  GL_NEAREST);
    pglFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, motionTarget.depth,
                            0);
    const GLenum drawBuffers[] = {GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1, GL_COLOR_ATTACHMENT2};
    pglDrawBuffers(3, drawBuffers);
    const GLenum status = pglCheckFramebufferStatus(GL_FRAMEBUFFER);
    if (status != GL_FRAMEBUFFER_COMPLETE) {
        std::ostringstream error;
        error << "Motion-data framebuffer is incomplete (status 0x" << std::hex << status << ')';
        gameLog.Error(error.str());
        pglBindFramebuffer(GL_FRAMEBUFFER, static_cast<GLuint>(previousFramebuffer));
        glBindTexture(GL_TEXTURE_2D, 0);
        destroyMotionTarget();
        failedMotionTargetWidth = width;
        failedMotionTargetHeight = height;
        return false;
    }

    motionTarget.width = width;
    motionTarget.height = height;

    if (antiAliasingMode == AntiAliasingMode::Msaa && motionResolveProgram == 0) {
        antiAliasingMode = AntiAliasingMode::Fxaa;
        gameLog.Warning("MSAA resolve shader is unavailable; using FXAA");
    }
    if (antiAliasingMode == AntiAliasingMode::Smaa &&
        (smaaEdgeProgram == 0 || smaaWeightsProgram == 0 || smaaNeighborhoodProgram == 0 ||
         !smaaLookupReady)) {
        antiAliasingMode = AntiAliasingMode::Fxaa;
        gameLog.Warning("SMAA shaders or lookup textures are unavailable; using FXAA");
    }

    if (antiAliasingMode == AntiAliasingMode::Msaa) {
        GLint maximumSamples = 0;
        glGetIntegerv(GL_MAX_SAMPLES, &maximumSamples);
        const int requestedSamples = 4;
        const auto destroyMultisampleTarget = [&] {
            if (motionTarget.multisampleFramebuffer != 0) {
                pglDeleteFramebuffers(1, &motionTarget.multisampleFramebuffer);
            }
            const std::array<GLuint, 4> textures = {
                motionTarget.multisampleColor, motionTarget.multisampleMotionVectors,
                motionTarget.multisampleValidity, motionTarget.multisampleDepth};
            glDeleteTextures(static_cast<GLsizei>(textures.size()), textures.data());
            motionTarget.multisampleFramebuffer = 0;
            motionTarget.multisampleColor = 0;
            motionTarget.multisampleMotionVectors = 0;
            motionTarget.multisampleValidity = 0;
            motionTarget.multisampleDepth = 0;
            motionTarget.samples = 0;
        };
        const auto tryCreateMultisampleTarget = [&](int samples) {
            if (pglTexImage2DMultisample == nullptr || samples > maximumSamples) {
                return false;
            }
            pglGenFramebuffers(1, &motionTarget.multisampleFramebuffer);
            pglBindFramebuffer(GL_FRAMEBUFFER, motionTarget.multisampleFramebuffer);
            const auto createMultisampleTexture = [&](GLuint& texture, GLint internalFormat) {
                glGenTextures(1, &texture);
                glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, texture);
                pglTexImage2DMultisample(GL_TEXTURE_2D_MULTISAMPLE, samples, internalFormat,
                                         width, height, GL_TRUE);
            };
            createMultisampleTexture(motionTarget.multisampleColor, GL_RGBA8);
            pglFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                                    GL_TEXTURE_2D_MULTISAMPLE, motionTarget.multisampleColor, 0);
            createMultisampleTexture(motionTarget.multisampleMotionVectors, GL_RG16F);
            pglFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1,
                                    GL_TEXTURE_2D_MULTISAMPLE,
                                    motionTarget.multisampleMotionVectors, 0);
            createMultisampleTexture(motionTarget.multisampleValidity, GL_R8);
            pglFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT2,
                                    GL_TEXTURE_2D_MULTISAMPLE,
                                    motionTarget.multisampleValidity, 0);
            glGenTextures(1, &motionTarget.multisampleDepth);
            glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, motionTarget.multisampleDepth);
            pglTexImage2DMultisample(GL_TEXTURE_2D_MULTISAMPLE, samples, GL_DEPTH_COMPONENT24,
                                     width, height, GL_TRUE);
            pglFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
                                    GL_TEXTURE_2D_MULTISAMPLE, motionTarget.multisampleDepth, 0);
            const GLenum multisampleDrawBuffers[] = {
                GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1, GL_COLOR_ATTACHMENT2};
            pglDrawBuffers(3, multisampleDrawBuffers);
            const bool complete = pglCheckFramebufferStatus(GL_FRAMEBUFFER) ==
                                  GL_FRAMEBUFFER_COMPLETE;
            glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, 0);
            if (!complete) {
                destroyMultisampleTarget();
                return false;
            }
            motionTarget.samples = samples;
            return true;
        };
        pglActiveTexture(GL_TEXTURE0);
        currentTextureUnit = GL_TEXTURE0;
        if (!tryCreateMultisampleTarget(requestedSamples) &&
            !tryCreateMultisampleTarget(2)) {
            destroyMultisampleTarget();
            antiAliasingMode = AntiAliasingMode::Fxaa;
            gameLog.Warning("4x MSAA is unavailable for the motion-data targets; using FXAA");
        } else if (motionTarget.samples != requestedSamples) {
            gameLog.Warning("4x MSAA is unavailable; using 2x MSAA");
        }
    } else if (antiAliasingMode == AntiAliasingMode::Smaa) {
        const auto destroySmaaTargets = [&] {
            const std::array<GLuint, 2> framebuffers = {motionTarget.smaaEdgeFramebuffer,
                                                        motionTarget.smaaWeightsFramebuffer};
            pglDeleteFramebuffers(static_cast<GLsizei>(framebuffers.size()), framebuffers.data());
            const std::array<GLuint, 2> textures = {motionTarget.smaaEdges,
                                                    motionTarget.smaaWeights};
            glDeleteTextures(static_cast<GLsizei>(textures.size()), textures.data());
            motionTarget.smaaEdgeFramebuffer = 0;
            motionTarget.smaaEdges = 0;
            motionTarget.smaaWeightsFramebuffer = 0;
            motionTarget.smaaWeights = 0;
        };
        const auto createSmaaTexture = [&](GLuint& texture, GLint internalFormat, GLenum format) {
            glGenTextures(1, &texture);
            glBindTexture(GL_TEXTURE_2D, texture);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
            glTexImage2D(GL_TEXTURE_2D, 0, internalFormat, width, height, 0, format,
                         GL_UNSIGNED_BYTE, nullptr);
        };
        pglActiveTexture(GL_TEXTURE0);
        currentTextureUnit = GL_TEXTURE0;
        pglGenFramebuffers(1, &motionTarget.smaaEdgeFramebuffer);
        pglBindFramebuffer(GL_FRAMEBUFFER, motionTarget.smaaEdgeFramebuffer);
        createSmaaTexture(motionTarget.smaaEdges, GL_RG8, GL_RG);
        pglFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
                                motionTarget.smaaEdges, 0);
        const GLenum smaaDrawBuffer = GL_COLOR_ATTACHMENT0;
        pglDrawBuffers(1, &smaaDrawBuffer);
        bool smaaComplete =
            pglCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE;
        if (smaaComplete) {
            pglGenFramebuffers(1, &motionTarget.smaaWeightsFramebuffer);
            pglBindFramebuffer(GL_FRAMEBUFFER, motionTarget.smaaWeightsFramebuffer);
            createSmaaTexture(motionTarget.smaaWeights, GL_RGBA8, GL_RGBA);
            pglFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
                                    motionTarget.smaaWeights, 0);
            pglDrawBuffers(1, &smaaDrawBuffer);
            smaaComplete =
                pglCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE;
        }
        if (!smaaComplete) {
            destroySmaaTargets();
            antiAliasingMode = AntiAliasingMode::Fxaa;
            gameLog.Warning("SMAA render targets could not be created; using FXAA");
        }
    }

    motionTarget.mode = targetModeFor(antiAliasingMode);
    pglBindFramebuffer(GL_FRAMEBUFFER, static_cast<GLuint>(previousFramebuffer));
    glBindTexture(GL_TEXTURE_2D, 0);
    glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, 0);
    publishedMotionData.motionVectors = motionTarget.motionVectors;
    publishedMotionData.validity = motionTarget.validity;
    publishedMotionData.depth = motionTarget.depth;
    publishedMotionData.width = width;
    publishedMotionData.height = height;
    publishedMotionData.available = true;
    failedMotionTargetWidth = 0;
    failedMotionTargetHeight = 0;
    motionHistory.invalidate();
    invalidateTextureBindings();
    return true;
}

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
uniform mat4 uPreviousProjection;
uniform mat4 uPreviousModelView;
uniform bool uMotionHistoryValid;
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
out vec3 vVertexMapLighting;
out vec4 vPreviousClipPosition;
flat out float vMotionHistoryValid;
uniform bool uVertexMapLightEvaluation;
uniform bool uUseTextTexture;
uniform bool uWeatherLitDynamicTexture;
uniform bool uMaterialBatch;
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
    vVertexMapLighting = vec3(0.0);
    vPreviousClipPosition =
        uPreviousProjection * uPreviousModelView * vec4(position, 1.0);
    vMotionHistoryValid =
        uMotionHistoryValid && !uHumanSkinEnabled && !uHumanGaitEnabled ? 1.0 : 0.0;
    if (uVertexMapLightEvaluation &&
        (uMaterialBatch || !uUseTextTexture || uWeatherLitDynamicTexture)) {
        vVertexMapLighting = mapLightLighting(viewPosition.xyz, normalize(vNormal));
    }
    gl_Position = uProjection * viewPosition;
}
)GLSL";

const char* modelFragmentShader = R"GLSL(
#version 330 core
in vec3 vTexCoord;
in vec3 vNormal;
in vec3 vViewPosition;
in vec3 vVertexMapLighting;
in vec4 vPreviousClipPosition;
flat in float vMotionHistoryValid;
uniform bool uVertexMapLightEvaluation;
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
uniform vec2 uMotionRenderSize;
layout(location = 0) out vec4 fragmentColor;
layout(location = 1) out vec2 motionVector;
layout(location = 2) out float motionValidity;
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
    if (!uUseTextTexture && uUseTransmap) {
        vec2 transmapCoordinate = vTexCoord.xy * uTransmapScale;
        if (uFlipTransmapY) {
            transmapCoordinate.y = 1.0 - transmapCoordinate.y;
        }
        color.a = uColor.a * texture(uTransmap, transmapCoordinate).a;
    }
    float visibilityFade = clamp(uVisibilityFade, 0.0, 1.0);
    if (!uUseBumpmap) {
        float fadedAlpha = color.a * visibilityFade;
        if ((uAlphaMode == 1 && fadedAlpha < 0.5) ||
            (uAlphaMode == 3 && fadedAlpha < 0.99)) {
            discard;
        }
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
        vec3 dynamicLighting =
            vehicleSpotlightLighting(vViewPosition, normalize(vNormal)) +
            (uVertexMapLightEvaluation ? vVertexMapLighting
                                       : mapLightLighting(vViewPosition, normalize(vNormal)));
        color.rgb += diffuseReflectance * dynamicLighting;
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
    color.a *= visibilityFade;
    if (uUseBumpmap &&
        ((uAlphaMode == 1 && color.a < 0.5) || (uAlphaMode == 3 && color.a < 0.99))) {
        discard;
    }
    fragmentColor = color;
    bool previousPositionValid =
        vMotionHistoryValid > 0.5 && vPreviousClipPosition.w > 0.000001;
    vec2 previousNdc =
        previousPositionValid ? vPreviousClipPosition.xy / vPreviousClipPosition.w : vec2(0.0);
    vec2 previousPixel = (previousNdc * 0.5 + 0.5) * uMotionRenderSize;
    motionVector = previousPositionValid ? previousPixel - gl_FragCoord.xy : vec2(0.0);
    motionValidity = previousPositionValid ? 1.0 : 0.0;
}
)GLSL";

const char* materialFragmentShader = R"GLSL(
#version 330 core
in vec3 vTexCoord;
in vec3 vNormal;
in vec3 vViewPosition;
in vec3 vVertexMapLighting;
in vec4 vPreviousClipPosition;
flat in float vMotionHistoryValid;
uniform bool uVertexMapLightEvaluation;
uniform vec4 uMaterialColors[128];
uniform sampler2D uMaterialTextures[8];
uniform bool uFlipTextureY[8];
uniform sampler2D uTextTexture;
uniform bool uUseTextTexture;
uniform vec3 uLightDirection;
uniform float uAmbientLight;
uniform float uDiffuseLight;
uniform vec3 uFogColor;
uniform float uVisibilityFade;
uniform vec2 uMotionRenderSize;
layout(location = 0) out vec4 fragmentColor;
layout(location = 1) out vec2 motionVector;
layout(location = 2) out float motionValidity;
void main() {
    int materialIndex = clamp(int(vTexCoord.z + 0.5), 0, 127);
    int textureIndex = clamp(materialIndex, 0, 7);
    vec2 textureCoord = uFlipTextureY[textureIndex]
                            ? vec2(vTexCoord.x, 1.0 - vTexCoord.y)
                            : vTexCoord.xy;
    vec4 color = uMaterialColors[materialIndex];
    if (uUseTextTexture) {
        color *= texture(uTextTexture, vTexCoord.xy);
        if (color.a <= 0.01) {
            discard;
        }
    } else if (textureIndex == 0) {
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
    vec3 dynamicLighting =
        vehicleSpotlightLighting(vViewPosition, surfaceNormal) +
        (uVertexMapLightEvaluation ? vVertexMapLighting
                                   : mapLightLighting(vViewPosition, surfaceNormal));
    color.rgb += reflectance * dynamicLighting;
    if (uFogDensity > 0.0) {
        float transmission = weatherFogTransmission(vViewPosition);
        color.rgb = mix(uFogColor, color.rgb, transmission);
    }
    color.a *= clamp(uVisibilityFade, 0.0, 1.0);
    fragmentColor = color;
    bool previousPositionValid =
        vMotionHistoryValid > 0.5 && vPreviousClipPosition.w > 0.000001;
    vec2 previousNdc =
        previousPositionValid ? vPreviousClipPosition.xy / vPreviousClipPosition.w : vec2(0.0);
    vec2 previousPixel = (previousNdc * 0.5 + 0.5) * uMotionRenderSize;
    motionVector = previousPositionValid ? previousPixel - gl_FragCoord.xy : vec2(0.0);
    motionValidity = previousPositionValid ? 1.0 : 0.0;
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
uniform sampler2D uMotionValidity;
uniform int uMotionDebugMode;
uniform int uFxaaEnabled;
uniform vec2 uInverseResolution;
out vec4 fragmentColor;
float luma(vec3 color) {
    return dot(color, vec3(0.299, 0.587, 0.114));
}
void main() {
    vec2 motion = texture(uTexture, vTexCoord).rg;
    if (uMotionDebugMode == 0) {
        vec4 center = texture(uTexture, vTexCoord);
        if (uFxaaEnabled == 0) {
            fragmentColor = center;
            return;
        }
        vec3 northwest = texture(uTexture, vTexCoord + vec2(-1.0, -1.0) *
                                                         uInverseResolution).rgb;
        vec3 northeast = texture(uTexture, vTexCoord + vec2(1.0, -1.0) *
                                                         uInverseResolution).rgb;
        vec3 southwest = texture(uTexture, vTexCoord + vec2(-1.0, 1.0) *
                                                         uInverseResolution).rgb;
        vec3 southeast = texture(uTexture, vTexCoord + vec2(1.0, 1.0) *
                                                         uInverseResolution).rgb;
        float centerLuma = luma(center.rgb);
        float northwestLuma = luma(northwest);
        float northeastLuma = luma(northeast);
        float southwestLuma = luma(southwest);
        float southeastLuma = luma(southeast);
        float minimumLuma = min(centerLuma, min(min(northwestLuma, northeastLuma),
                                                min(southwestLuma, southeastLuma)));
        float maximumLuma = max(centerLuma, max(max(northwestLuma, northeastLuma),
                                                max(southwestLuma, southeastLuma)));
        if (maximumLuma - minimumLuma <
            max(0.0312, maximumLuma * 0.125)) {
            fragmentColor = center;
            return;
        }
        vec2 direction = vec2(
            -((northwestLuma + northeastLuma) - (southwestLuma + southeastLuma)),
            (northwestLuma + southwestLuma) - (northeastLuma + southeastLuma));
        float directionReduce =
            max((northwestLuma + northeastLuma + southwestLuma + southeastLuma) *
                    0.03125,
                1.0 / 128.0);
        float inverseDirectionMinimum =
            1.0 / (min(abs(direction.x), abs(direction.y)) + directionReduce);
        direction = clamp(direction * inverseDirectionMinimum, vec2(-8.0), vec2(8.0)) *
                    uInverseResolution;
        vec3 colorA = 0.5 *
                      (texture(uTexture, vTexCoord + direction * (1.0 / 3.0 - 0.5)).rgb +
                       texture(uTexture, vTexCoord + direction * (2.0 / 3.0 - 0.5)).rgb);
        vec3 colorB = colorA * 0.5 +
                      0.25 * (texture(uTexture, vTexCoord + direction * -0.5).rgb +
                              texture(uTexture, vTexCoord + direction * 0.5).rgb);
        float lumaB = luma(colorB);
        vec3 result = lumaB < minimumLuma || lumaB > maximumLuma ? colorA : colorB;
        fragmentColor = vec4(result, center.a);
    } else {
        float valid = texture(uMotionValidity, vTexCoord).r;
        if (valid < 0.5) {
            fragmentColor = vec4(1.0, 0.0, 1.0, 1.0);
        } else if (uMotionDebugMode == 1) {
            float magnitude = clamp(length(motion) / 64.0, 0.0, 1.0);
            vec2 direction = length(motion) > 0.0001 ? normalize(motion) : vec2(0.0);
            fragmentColor = vec4(direction * 0.5 + 0.5, magnitude, 1.0);
        } else {
            float value = clamp(length(motion) / 32.0, 0.0, 1.0);
            vec3 heat = clamp(vec3(1.5) - abs(4.0 * value - vec3(3.0, 2.0, 1.0)),
                              0.0, 1.0);
            fragmentColor = vec4(heat, 1.0);
        }
    }
}
)GLSL";

const char* motionResolveFragmentShader = R"GLSL(
#version 330 core
in vec2 vTexCoord;
uniform sampler2DMS uColorSamples;
uniform sampler2DMS uMotionSamples;
uniform sampler2DMS uValiditySamples;
uniform sampler2DMS uDepthSamples;
uniform int uSampleCount;
layout(location = 0) out vec4 fragmentColor;
layout(location = 1) out vec2 motionVector;
layout(location = 2) out float motionValidity;
void main() {
    ivec2 pixel = ivec2(gl_FragCoord.xy);
    vec4 colorSum = vec4(0.0);
    float nearestDepth = 1.0;
    int nearestSample = 0;
    for (int sampleIndex = 0; sampleIndex < 16; ++sampleIndex) {
        if (sampleIndex >= uSampleCount) {
            break;
        }
        colorSum += texelFetch(uColorSamples, pixel, sampleIndex);
        float sampleDepth = texelFetch(uDepthSamples, pixel, sampleIndex).r;
        if (sampleDepth < nearestDepth) {
            nearestDepth = sampleDepth;
            nearestSample = sampleIndex;
        }
    }
    fragmentColor = colorSum / float(uSampleCount);
    motionVector = texelFetch(uMotionSamples, pixel, nearestSample).rg;
    motionValidity = texelFetch(uValiditySamples, pixel, nearestSample).r;
    gl_FragDepth = nearestDepth;
}
)GLSL";

std::string makeSmaaFragmentShader(std::string_view mainFunction) {
    std::string source =
        "#version 330 core\n"
        "#define SMAA_GLSL_3\n"
        "#define SMAA_INCLUDE_VS 1\n"
        "#define SMAA_INCLUDE_PS 1\n"
        "#define SMAA_PRESET_HIGH\n"
        "#define SMAA_RT_METRICS uMetrics\n"
        "uniform vec4 uMetrics;\n";
    source += openbusSmaaShader;
    source += "\nin vec2 vTexCoord;\n"
              "layout(location = 0) out vec4 fragmentColor;\n"
              "layout(location = 1) out vec2 motionVector;\n"
              "layout(location = 2) out float motionValidity;\n";
    source.append(mainFunction);
    return source;
}

const char* smaaEdgeMain = R"GLSL(
uniform sampler2D uColorTexture;
void main() {
    vec4 offsets[3];
    SMAAEdgeDetectionVS(vTexCoord, offsets);
    fragmentColor = vec4(SMAAColorEdgeDetectionPS(vTexCoord, offsets, uColorTexture), 0.0, 1.0);
    motionVector = vec2(0.0);
    motionValidity = 0.0;
}
)GLSL";

const char* smaaWeightsMain = R"GLSL(
uniform sampler2D uEdgesTexture;
uniform sampler2D uAreaTexture;
uniform sampler2D uSearchTexture;
void main() {
    vec2 pixelCoordinates;
    vec4 offsets[3];
    SMAABlendingWeightCalculationVS(vTexCoord, pixelCoordinates, offsets);
    fragmentColor = SMAABlendingWeightCalculationPS(
        vTexCoord, pixelCoordinates, offsets, uEdgesTexture, uAreaTexture, uSearchTexture,
        vec4(0.0));
    motionVector = vec2(0.0);
    motionValidity = 0.0;
}
)GLSL";

const char* smaaNeighborhoodMain = R"GLSL(
uniform sampler2D uColorTexture;
uniform sampler2D uWeightsTexture;
void main() {
    vec4 offsets;
    SMAANeighborhoodBlendingVS(vTexCoord, offsets);
    fragmentColor = SMAANeighborhoodBlendingPS(
        vTexCoord, offsets, uColorTexture, uWeightsTexture);
    motionVector = vec2(0.0);
    motionValidity = 0.0;
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

const char* vehicleSpotlightContributionShaderFunctions = R"GLSL(
uniform samplerBuffer uSpotlightData;
uniform int uSpotlightCount;
vec3 vehicleSpotlightContribution(int index, vec3 position, vec3 normal) {
    vec4 positionRange = texelFetch(uSpotlightData, index * 3);
    vec3 displacement = position - positionRange.xyz;
    float distanceSquared = dot(displacement, displacement);
    if (distanceSquared <= 0.00000001) return vec3(0.0);
    float range = positionRange.w;
    vec3 relativeDisplacement = displacement / range;
    float relativeDistanceSquared = dot(relativeDisplacement, relativeDisplacement);
    if (relativeDistanceSquared >= 16.0) return vec3(0.0);
    float relativeDistance = sqrt(relativeDistanceSquared);
    vec3 ray;
    if (relativeDistanceSquared == 0.0 || isinf(distanceSquared)) {
        float scale = max(max(abs(displacement.x), abs(displacement.y)),
                          abs(displacement.z));
        if (isinf(scale)) return vec3(0.0);
        vec3 scaledDisplacement = displacement / scale;
        ray = scaledDisplacement * inversesqrt(dot(scaledDisplacement, scaledDisplacement));
    } else {
        ray = displacement * inversesqrt(distanceSquared);
    }
    vec4 directionInner = texelFetch(uSpotlightData, index * 3 + 1);
    vec4 colorOuter = texelFetch(uSpotlightData, index * 3 + 2);
    float angle = dot(ray, directionInner.xyz);
    float inner = directionInner.w;
    float outer = colorOuter.w;
    float cone = inner > outer ? clamp((angle - outer) / (inner - outer), 0.0, 1.0)
                               : step(inner, angle);
    if (cone == 0.0) return vec3(0.0);
    float attenuation = 1.0 /
        (0.2 + relativeDistance + 40.0 * relativeDistance * relativeDistance);
    float diffuse = max(dot(normal, -ray), 0.0);
    return colorOuter.rgb * cone * cone * attenuation * (0.1 + 0.3 * diffuse);
}
)GLSL";

const char* vehicleSpotlightVertexShaderFunctions = R"GLSL(
vec3 vehicleSpotlightLighting(vec3 position, vec3 normal) {
    vec3 result = vec3(0.0);
    for (int i = 0; i < uSpotlightCount; ++i) {
        result += vehicleSpotlightContribution(i, position, normal);
    }
    return result;
}
)GLSL";

const char* vehicleSpotlightFragmentShaderFunctions = R"GLSL(
uniform usamplerBuffer uSpotlightTileMask;
uniform bool uSpotlightTileMaskEnabled;
uniform int uSpotlightTileColumns;
uniform int uSpotlightTileRows;
uniform int uSpotlightTileWordsPerTile;
uniform int uSpotlightTileSize;
uniform vec2 uSpotlightViewportOrigin;
int spotlightFirstSetBit(uint value) {
    int bit = 0;
    if ((value & 65535u) == 0u) { value >>= 16u; bit += 16; }
    if ((value & 255u) == 0u) { value >>= 8u; bit += 8; }
    if ((value & 15u) == 0u) { value >>= 4u; bit += 4; }
    if ((value & 3u) == 0u) { value >>= 2u; bit += 2; }
    if ((value & 1u) == 0u) bit += 1;
    return bit;
}
vec3 vehicleSpotlightLighting(vec3 position, vec3 normal) {
    vec3 result = vec3(0.0);
    if (!uSpotlightTileMaskEnabled) {
        for (int i = 0; i < uSpotlightCount; ++i) {
            result += vehicleSpotlightContribution(i, position, normal);
        }
        return result;
    }
    ivec2 pixel = ivec2(gl_FragCoord.xy - uSpotlightViewportOrigin);
    ivec2 tile = clamp(pixel / uSpotlightTileSize, ivec2(0),
                       ivec2(uSpotlightTileColumns - 1, uSpotlightTileRows - 1));
    int tileOffset = (tile.y * uSpotlightTileColumns + tile.x) * uSpotlightTileWordsPerTile;
    for (int word = 0; word < uSpotlightTileWordsPerTile; ++word) {
        uint candidates = texelFetch(uSpotlightTileMask, tileOffset + word).r;
        while (candidates != 0u) {
            int index = word * 32 + spotlightFirstSetBit(candidates);
            if (index >= uSpotlightCount) break;
            result += vehicleSpotlightContribution(index, position, normal);
            candidates &= candidates - 1u;
        }
    }
    return result;
}
)GLSL";

const char* mapLightShaderFunctions = R"GLSL(
uniform samplerBuffer uMapLightData;
uniform samplerBuffer uStaticMapLightData;
uniform usamplerBuffer uStaticMapViewData;
uniform bool uStaticMapMode;
uniform int uMapLightCount;
uniform float uMapLightStrength;
vec3 mapLightLighting(vec3 position, vec3 normal) {
    vec3 result = vec3(0.0);
    for (int i = 0; i < uMapLightCount; ++i) {
        vec4 positionRadius;
        vec3 lightColor;
        if (uStaticMapMode) {
            uvec4 viewEntry = texelFetch(uStaticMapViewData, i);
            vec4 attributes = texelFetch(uStaticMapLightData, int(viewEntry.w));
            positionRadius = vec4(uintBitsToFloat(viewEntry.xyz), attributes.w);
            lightColor = attributes.rgb;
        } else {
            positionRadius = texelFetch(uMapLightData, i * 2);
            lightColor = texelFetch(uMapLightData, i * 2 + 1).rgb;
        }
        vec3 displacement = position - positionRadius.xyz;
        float distanceSquared = dot(displacement, displacement);
        if (distanceSquared <= 0.00000001) continue;
        float radius = positionRadius.w;
        float radiusSquared = radius * radius;
        float falloff;
        vec3 ray;
        if (isinf(distanceSquared) || isinf(radiusSquared)) {
            float scale = max(max(abs(displacement.x), abs(displacement.y)),
                              abs(displacement.z));
            if (isinf(scale)) continue;
            vec3 scaledDisplacement = displacement / scale;
            float scaledDistanceSquared = dot(scaledDisplacement, scaledDisplacement);
            float distanceOverRadius = scale / radius;
            float relativeDistanceSquared =
                distanceOverRadius * distanceOverRadius * scaledDistanceSquared;
            falloff = relativeDistanceSquared <= 1.0 ? 1.0 : 1.0 / relativeDistanceSquared;
            ray = scaledDisplacement * inversesqrt(scaledDistanceSquared);
        } else {
            float relativeDistanceSquared = distanceSquared / radiusSquared;
            falloff = relativeDistanceSquared <= 1.0 ? 1.0 : 1.0 / relativeDistanceSquared;
            ray = displacement * inversesqrt(distanceSquared);
        }
        float diffuse = max(dot(normal, -ray), 0.0);
        result += lightColor * falloff * (0.12 + 0.88 * diffuse);
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
    std::string vertex(vertexSource);
    if (vertexSource == modelVertexShader) {
        const auto versionEnd = vertex.find('\n', vertex.find("#version"));
        if (versionEnd == std::string::npos) {
            throw std::runtime_error("Model vertex shader is missing a version directive");
        }
        vertex.insert(versionEnd + 1, mapLightShaderFunctions);
        if (usesVehicleSpotlights) {
            std::string spotlightFunctions = vehicleSpotlightContributionShaderFunctions;
            spotlightFunctions += vehicleSpotlightVertexShaderFunctions;
            vertex.insert(versionEnd + 1, spotlightFunctions);
        }
    }
    const GLuint vertexShader = compileShader(GL_VERTEX_SHADER, vertex.c_str());
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
        std::string spotlightFunctions = vehicleSpotlightContributionShaderFunctions;
        spotlightFunctions += vehicleSpotlightFragmentShaderFunctions;
        fragment.insert(versionEnd + 1, spotlightFunctions);
    }
    if (usesMapLights) {
        const auto versionEnd = fragment.find('\n', fragment.find("#version"));
        if (versionEnd == std::string::npos)
            throw std::runtime_error("Map-light shader is missing a version directive");
        fragment.insert(versionEnd + 1, mapLightShaderFunctions);
    }
    if (fragment.find("layout(location = 1) out vec2 motionVector") == std::string::npos) {
        const auto versionEnd = fragment.find('\n', fragment.find("#version"));
        if (versionEnd == std::string::npos) {
            throw std::runtime_error("Fragment shader is missing a version directive");
        }
        const std::string colorOutput = "out vec4 fragmentColor;";
        const auto colorOutputPosition = fragment.find(colorOutput);
        if (colorOutputPosition != std::string::npos) {
            fragment.replace(colorOutputPosition, colorOutput.size(),
                             "layout(location = 0) out vec4 fragmentColor;");
        }
        fragment.insert(versionEnd + 1, "layout(location = 1) out vec2 motionVector;\n"
                                        "layout(location = 2) out float motionValidity;\n");
        const std::string mainDeclaration = "void main() {";
        const auto mainPosition = fragment.find(mainDeclaration);
        if (mainPosition == std::string::npos) {
            throw std::runtime_error("Fragment shader is missing its main function");
        }
        fragment.insert(mainPosition + mainDeclaration.size(), "\n    motionVector = vec2(0.0);\n"
                                                               "    motionValidity = 0.0;");
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
    uniforms.spotlightTexture = pglGetUniformLocation(program, "uSpotlightData");
    uniforms.spotlightTileMaskTexture = pglGetUniformLocation(program, "uSpotlightTileMask");
    uniforms.spotlightTileMaskEnabled = pglGetUniformLocation(program, "uSpotlightTileMaskEnabled");
    uniforms.spotlightTileColumns = pglGetUniformLocation(program, "uSpotlightTileColumns");
    uniforms.spotlightTileRows = pglGetUniformLocation(program, "uSpotlightTileRows");
    uniforms.spotlightTileWordsPerTile =
        pglGetUniformLocation(program, "uSpotlightTileWordsPerTile");
    uniforms.spotlightTileSize = pglGetUniformLocation(program, "uSpotlightTileSize");
    uniforms.spotlightViewportOrigin = pglGetUniformLocation(program, "uSpotlightViewportOrigin");
    uniforms.vertexLightEvaluation = pglGetUniformLocation(program, "uVertexMapLightEvaluation");
}

void findMapLightUniforms(GLuint program, Uniforms& uniforms) {
    uniforms.mapLightCount = pglGetUniformLocation(program, "uMapLightCount");
    uniforms.mapLightStrength = pglGetUniformLocation(program, "uMapLightStrength");
    uniforms.mapLightTexture = pglGetUniformLocation(program, "uMapLightData");
    uniforms.staticMapLightTexture = pglGetUniformLocation(program, "uStaticMapLightData");
    uniforms.staticMapViewTexture = pglGetUniformLocation(program, "uStaticMapViewData");
    uniforms.staticMapMode = pglGetUniformLocation(program, "uStaticMapMode");
    uniforms.vertexLightEvaluation = pglGetUniformLocation(program, "uVertexMapLightEvaluation");
}

Uniforms modelUniformsFor(GLuint program, bool environment) {
    Uniforms uniforms;
    uniforms.projection = pglGetUniformLocation(program, "uProjection");
    uniforms.modelView = pglGetUniformLocation(program, "uModelView");
    uniforms.previousProjection = pglGetUniformLocation(program, "uPreviousProjection");
    uniforms.previousModelView = pglGetUniformLocation(program, "uPreviousModelView");
    uniforms.motionRenderSize = pglGetUniformLocation(program, "uMotionRenderSize");
    uniforms.motionHistoryValid = pglGetUniformLocation(program, "uMotionHistoryValid");
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
    uniforms.previousProjection = pglGetUniformLocation(program, "uPreviousProjection");
    uniforms.previousModelView = pglGetUniformLocation(program, "uPreviousModelView");
    uniforms.motionRenderSize = pglGetUniformLocation(program, "uMotionRenderSize");
    uniforms.motionHistoryValid = pglGetUniformLocation(program, "uMotionHistoryValid");
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
    uniforms.textTexture = pglGetUniformLocation(program, "uTextTexture");
    uniforms.useTextTexture = pglGetUniformLocation(program, "uUseTextTexture");
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

void uploadMotionMatrices(const Uniforms& uniforms, MotionObjectKey key,
                          bool geometryHistoryValid) {
    Matrix4 previousModelView = modelViewMatrix();
    Matrix4 previousProjection = projectionMatrix();
    bool historyValid = false;
    int width = 1;
    int height = 1;
    if (motionFrameActive) {
        const MotionDataFrame& frame = motionHistory.frame();
        const MotionTransform transform =
            motionHistory.previousModelView(modelViewMatrix(), key, geometryHistoryValid);
        previousModelView = transform.previousModelView;
        previousProjection = frame.previousProjection;
        historyValid = transform.valid;
        width = frame.width;
        height = frame.height;
    }

    std::array<GLfloat, 16> projectionFloat = {};
    std::array<GLfloat, 16> modelViewFloat = {};
    for (std::size_t index = 0; index < previousProjection.size(); ++index) {
        projectionFloat[index] = static_cast<GLfloat>(previousProjection[index]);
        modelViewFloat[index] = static_cast<GLfloat>(previousModelView[index]);
    }
    pglUniformMatrix4fv(uniforms.previousProjection, 1, GL_FALSE, projectionFloat.data());
    pglUniformMatrix4fv(uniforms.previousModelView, 1, GL_FALSE, modelViewFloat.data());
    pglUniform1i(uniforms.motionHistoryValid, historyValid ? 1 : 0);
    pglUniform2f(uniforms.motionRenderSize, static_cast<GLfloat>(width),
                 static_cast<GLfloat>(height));
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
    if (currentProgram != program) {
        pglUseProgram(program);
        currentProgram = program;
    }
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

std::size_t mapLightSelectionLimit() {
    return mapLightLimit();
}

void configureMapLightSelectionLimit(std::size_t limit) {
    configuredMapLightSelectionLimit = limit;
    if (openbus::getEnvironment("OPENBUS_MAP_LIGHT_LIMIT") == nullptr) {
        gameLog.Log("Map scenery-light selection limit: " +
                    (limit == 0 ? std::string("unlimited") : std::to_string(limit)));
    }
}

bool initializeCoreRenderer() {
    const GLubyte* vendor = glGetString(GL_VENDOR);
    const GLubyte* renderer = glGetString(GL_RENDERER);
    const GLubyte* version = glGetString(GL_VERSION);
    if (vendor != nullptr && renderer != nullptr && version != nullptr) {
        gameLog.Log("OpenGL vendor=" + std::string(reinterpret_cast<const char*>(vendor)) +
                    " renderer=" + std::string(reinterpret_cast<const char*>(renderer)) +
                    " version=" + std::string(reinterpret_cast<const char*>(version)));
    } else {
        gameLog.Warning("Unable to query OpenGL vendor, renderer, or version");
    }
    GLint majorVersion = 0;
    GLint minorVersion = 0;
    glGetIntegerv(GL_MAJOR_VERSION, &majorVersion);
    glGetIntegerv(GL_MINOR_VERSION, &minorVersion);
    const bool debugOutputSupported = majorVersion > 4 ||
                                      (majorVersion == 4 && minorVersion >= 3) ||
                                      hasOpenGLExtension("GL_KHR_debug");
    if (debugOutputSupported && pglDebugMessageCallback != nullptr) {
        glDebugErrorGeneration.store(0, std::memory_order_relaxed);
        glEnable(GL_DEBUG_OUTPUT);
        glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
        pglDebugMessageCallback(reportOpenGlDebugMessage, nullptr);
        gameLog.Log("Synchronous OpenGL debug error reporting enabled");
    } else {
        gameLog.Warning("OpenGL debug error reporting is unavailable; upload errors cannot be "
                        "detected");
    }
    modelProgram = createProgram(modelVertexShader, modelFragmentShader, true, true, true);
    materialProgram = createProgram(modelVertexShader, materialFragmentShader, true, true, true);
    environmentProgram = createProgram(modelVertexShader, environmentFragmentShader, true);
    primitiveProgram = createProgram(primitiveVertexShader, primitiveFragmentShader);
    textureQuadProgram = createProgram(textureQuadVertexShader, textureQuadFragmentShader);
    motionResolveProgram = createProgram(textureQuadVertexShader, motionResolveFragmentShader);
    const std::string smaaEdgeSource = makeSmaaFragmentShader(smaaEdgeMain);
    const std::string smaaWeightsSource = makeSmaaFragmentShader(smaaWeightsMain);
    const std::string smaaNeighborhoodSource = makeSmaaFragmentShader(smaaNeighborhoodMain);
    smaaEdgeProgram = createProgram(textureQuadVertexShader, smaaEdgeSource.c_str());
    smaaWeightsProgram = createProgram(textureQuadVertexShader, smaaWeightsSource.c_str());
    smaaNeighborhoodProgram =
        createProgram(textureQuadVertexShader, smaaNeighborhoodSource.c_str());
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
    textureQuadRect = pglGetUniformLocation(textureQuadProgram, "uRect");
    textureQuadTexture = pglGetUniformLocation(textureQuadProgram, "uTexture");
    textureQuadFlipTextureY = pglGetUniformLocation(textureQuadProgram, "uFlipTextureY");
    textureQuadMotionValidity = pglGetUniformLocation(textureQuadProgram, "uMotionValidity");
    textureQuadMotionDebugMode = pglGetUniformLocation(textureQuadProgram, "uMotionDebugMode");
    textureQuadFxaaEnabled = pglGetUniformLocation(textureQuadProgram, "uFxaaEnabled");
    textureQuadInverseResolution = pglGetUniformLocation(textureQuadProgram, "uInverseResolution");
    if (motionResolveProgram != 0) {
        motionResolveFullscreen = {pglGetUniformLocation(motionResolveProgram, "uRect"),
                                   pglGetUniformLocation(motionResolveProgram, "uFlipTextureY")};
        motionResolveColorSamples =
            pglGetUniformLocation(motionResolveProgram, "uColorSamples");
        motionResolveMotionSamples =
            pglGetUniformLocation(motionResolveProgram, "uMotionSamples");
        motionResolveValiditySamples =
            pglGetUniformLocation(motionResolveProgram, "uValiditySamples");
        motionResolveDepthSamples =
            pglGetUniformLocation(motionResolveProgram, "uDepthSamples");
        motionResolveSampleCount =
            pglGetUniformLocation(motionResolveProgram, "uSampleCount");
    }
    if (smaaEdgeProgram != 0) {
        smaaEdgeFullscreen = {pglGetUniformLocation(smaaEdgeProgram, "uRect"),
                              pglGetUniformLocation(smaaEdgeProgram, "uFlipTextureY")};
        smaaEdgeColorTexture = pglGetUniformLocation(smaaEdgeProgram, "uColorTexture");
        smaaEdgeMetrics = pglGetUniformLocation(smaaEdgeProgram, "uMetrics");
    }
    if (smaaWeightsProgram != 0) {
        smaaWeightsFullscreen = {pglGetUniformLocation(smaaWeightsProgram, "uRect"),
                                 pglGetUniformLocation(smaaWeightsProgram, "uFlipTextureY")};
        smaaWeightsEdgeTexture = pglGetUniformLocation(smaaWeightsProgram, "uEdgesTexture");
        smaaWeightsAreaTexture = pglGetUniformLocation(smaaWeightsProgram, "uAreaTexture");
        smaaWeightsSearchTexture = pglGetUniformLocation(smaaWeightsProgram, "uSearchTexture");
        smaaWeightsMetrics = pglGetUniformLocation(smaaWeightsProgram, "uMetrics");
    }
    if (smaaNeighborhoodProgram != 0) {
        smaaNeighborhoodFullscreen = {
            pglGetUniformLocation(smaaNeighborhoodProgram, "uRect"),
            pglGetUniformLocation(smaaNeighborhoodProgram, "uFlipTextureY")};
        smaaNeighborhoodColorTexture =
            pglGetUniformLocation(smaaNeighborhoodProgram, "uColorTexture");
        smaaNeighborhoodWeightsTexture =
            pglGetUniformLocation(smaaNeighborhoodProgram, "uWeightsTexture");
        smaaNeighborhoodMetrics =
            pglGetUniformLocation(smaaNeighborhoodProgram, "uMetrics");
    }
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
    glGenTextures(1, &vehicleSpotlightTexture);
    pglGenBuffers(1, &vehicleSpotlightTileMaskBuffer);
    glGenTextures(1, &vehicleSpotlightTileMaskTexture);
    pglGenBuffers(1, &mapLightBuffer);
    glGenTextures(1, &mapLightTexture);
    pglGenBuffers(1, &staticMapViewBuffer);
    glGenTextures(1, &staticMapViewTexture);
    GLint maxLightBufferTexels = 0;
    glGetIntegerv(GL_MAX_TEXTURE_BUFFER_SIZE, &maxLightBufferTexels);
    const std::size_t lightBufferTexels =
        maxLightBufferTexels > 0 ? static_cast<std::size_t>(maxLightBufferTexels) : 0;
    gameLog.Log("OpenGL light texture-buffer capacity: " + std::to_string(lightBufferTexels) +
                " texels");
    mapLightCapacity = detail::mapLightTextureCapacity(lightBufferTexels, 2);
    staticMapLightCapacity = detail::mapLightTextureCapacity(lightBufferTexels, 1);
    vehicleSpotlightCapacity =
        maxLightBufferTexels > 0 ? static_cast<std::size_t>(maxLightBufferTexels) / 3 : 0;
    vehicleSpotlightTileMaskCapacity =
        maxLightBufferTexels > 0 ? static_cast<std::size_t>(maxLightBufferTexels) : 0;
    if (mapLightCapacity == 0 || vehicleSpotlightCapacity == 0) {
        gameLog.Error("OpenGL does not report usable light texture-buffer capacity");
    }
    pglGenVertexArrays(1, &textureQuadVertexArray);
    pglGenBuffers(1, &textureQuadBuffer);
    if (modelVertexArray != 0 && primitiveVertexArray != 0 && precipitationVertexArray != 0 &&
        primitiveBuffer != 0 && vehicleSpotlightBuffer != 0 && vehicleSpotlightTexture != 0 &&
        vehicleSpotlightTileMaskBuffer != 0 && vehicleSpotlightTileMaskTexture != 0 &&
        vehicleSpotlightCapacity > 0 && vehicleSpotlightTileMaskCapacity > 0 &&
        mapLightBuffer != 0 && mapLightTexture != 0 && staticMapViewBuffer != 0 &&
        staticMapViewTexture != 0 && mapLightCapacity > 0 && textureQuadVertexArray != 0 &&
        textureQuadBuffer != 0) {
        pglBindBuffer(GL_TEXTURE_BUFFER, vehicleSpotlightBuffer);
        pglBufferData(GL_TEXTURE_BUFFER, 3 * sizeof(std::array<float, 4>), nullptr,
                      GL_DYNAMIC_DRAW);
        selectTextureUnit(vehicleSpotlightTextureUnit);
        glBindTexture(GL_TEXTURE_BUFFER, vehicleSpotlightTexture);
        pglTexBuffer(GL_TEXTURE_BUFFER, GL_RGBA32F, vehicleSpotlightBuffer);
        pglBindBuffer(GL_TEXTURE_BUFFER, vehicleSpotlightTileMaskBuffer);
        pglBufferData(GL_TEXTURE_BUFFER, sizeof(std::uint32_t), nullptr, GL_DYNAMIC_DRAW);
        selectTextureUnit(vehicleSpotlightTileMaskTextureUnit);
        glBindTexture(GL_TEXTURE_BUFFER, vehicleSpotlightTileMaskTexture);
        pglTexBuffer(GL_TEXTURE_BUFFER, GL_R32UI, vehicleSpotlightTileMaskBuffer);
        pglBindBuffer(GL_TEXTURE_BUFFER, mapLightBuffer);
        pglBufferData(GL_TEXTURE_BUFFER, 2 * sizeof(std::array<float, 4>), nullptr,
                      GL_DYNAMIC_DRAW);
        selectTextureUnit(mapLightTextureUnit);
        glBindTexture(GL_TEXTURE_BUFFER, mapLightTexture);
        pglTexBuffer(GL_TEXTURE_BUFFER, GL_RGBA32F, mapLightBuffer);
        pglBindBuffer(GL_TEXTURE_BUFFER, staticMapViewBuffer);
        pglBufferData(GL_TEXTURE_BUFFER, sizeof(std::array<std::uint32_t, 4>), nullptr,
                      GL_DYNAMIC_DRAW);
        selectTextureUnit(staticMapViewTextureUnit);
        glBindTexture(GL_TEXTURE_BUFFER, staticMapViewTexture);
        pglTexBuffer(GL_TEXTURE_BUFFER, GL_RGBA32UI, staticMapViewBuffer);
        pglBindBuffer(GL_TEXTURE_BUFFER, 0);
        selectTextureUnit(GL_TEXTURE0);
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
        pglActiveTexture(GL_TEXTURE0);
        currentTextureUnit = GL_TEXTURE0;
        glGenTextures(1, &smaaAreaTexture);
        glBindTexture(GL_TEXTURE_2D, smaaAreaTexture);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RG8, AREATEX_WIDTH, AREATEX_HEIGHT, 0, GL_RG,
                     GL_UNSIGNED_BYTE, areaTexBytes);
        glGenTextures(1, &smaaSearchTexture);
        glBindTexture(GL_TEXTURE_2D, smaaSearchTexture);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_R8, SEARCHTEX_WIDTH, SEARCHTEX_HEIGHT, 0, GL_RED,
                     GL_UNSIGNED_BYTE, searchTexBytes);
        glBindTexture(GL_TEXTURE_2D, 0);
        smaaLookupReady = smaaAreaTexture != 0 && smaaSearchTexture != 0;
        if (!smaaLookupReady) {
            gameLog.Warning("SMAA lookup textures could not be initialized");
        }
        useProgram(modelProgram);
        pglUniform1i(modelUniforms.mapLightTexture, 10);
        pglUniform1i(modelUniforms.spotlightTexture, 11);
        pglUniform1i(modelUniforms.spotlightTileMaskTexture, 14);
        pglUniform1i(modelUniforms.staticMapLightTexture, 12);
        pglUniform1i(modelUniforms.staticMapViewTexture, 13);
        pglUniform1f(modelUniforms.ambientLight, 0.58f);
        pglUniform1f(modelUniforms.diffuseLight, 0.42f);
        pglUniform1i(modelUniforms.vertexLightEvaluation,
                     mapLightVertexEvaluationEnabled() ? 1 : 0);
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
        pglUniform1i(materialUniforms.mapLightTexture, 10);
        pglUniform1i(materialUniforms.spotlightTexture, 11);
        pglUniform1i(materialUniforms.spotlightTileMaskTexture, 14);
        pglUniform1i(materialUniforms.staticMapLightTexture, 12);
        pglUniform1i(materialUniforms.staticMapViewTexture, 13);
        pglUniform1f(materialUniforms.ambientLight, 0.58f);
        pglUniform1f(materialUniforms.diffuseLight, 0.42f);
        pglUniform1i(materialUniforms.vertexLightEvaluation,
                     mapLightVertexEvaluationEnabled() ? 1 : 0);
        pglUniform1i(pglGetUniformLocation(materialProgram, "uMaterialBatch"), 1);
        for (std::size_t index = 0; index < MAX_MATERIAL_TEXTURES; ++index) {
            pglUniform1i(materialUniforms.materialTextures[index], static_cast<GLint>(index));
        }
        pglUniform1i(materialUniforms.textTexture, 8);
        useProgram(environmentProgram);
        pglUniform1i(environmentUniforms.environment, 0);
        useProgram(primitiveProgram);
        useProgram(textureQuadProgram);
        pglUniform1i(textureQuadTexture, 0);
        pglUniform1i(textureQuadMotionValidity, 1);
        pglUniform1i(textureQuadMotionDebugMode, 0);
        pglUniform1i(textureQuadFxaaEnabled, 0);
        pglUniform2f(textureQuadInverseResolution, 1.0F, 1.0F);
        if (motionResolveProgram != 0) {
            useProgram(motionResolveProgram);
            pglUniform1i(motionResolveColorSamples, 0);
            pglUniform1i(motionResolveMotionSamples, 1);
            pglUniform1i(motionResolveValiditySamples, 2);
            pglUniform1i(motionResolveDepthSamples, 3);
        }
        if (smaaEdgeProgram != 0) {
            useProgram(smaaEdgeProgram);
            pglUniform1i(smaaEdgeColorTexture, 0);
        }
        if (smaaWeightsProgram != 0) {
            useProgram(smaaWeightsProgram);
            pglUniform1i(smaaWeightsEdgeTexture, 0);
            pglUniform1i(smaaWeightsAreaTexture, 1);
            pglUniform1i(smaaWeightsSearchTexture, 2);
        }
        if (smaaNeighborhoodProgram != 0) {
            useProgram(smaaNeighborhoodProgram);
            pglUniform1i(smaaNeighborhoodColorTexture, 0);
            pglUniform1i(smaaNeighborhoodWeightsTexture, 1);
        }
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
    destroyMotionTarget();
    const std::array<GLuint, 2> smaaLookupTextures = {smaaAreaTexture, smaaSearchTexture};
    glDeleteTextures(static_cast<GLsizei>(smaaLookupTextures.size()), smaaLookupTextures.data());
    smaaAreaTexture = 0;
    smaaSearchTexture = 0;
    smaaLookupReady = false;
    antiAliasingMode = AntiAliasingMode::Off;
    motionHistory.invalidate();
    motionFrameId = 0;
    motionFrameActive = false;
    failedMotionTargetWidth = 0;
    failedMotionTargetHeight = 0;
    uploadedVehicleLights.clear();
    uploadedVehicleSpotlightTileMask.clear();
    uploadedMapLights.clear();
    uploadedStaticMapView.clear();
    lightUploadStats = {};
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
    for (const auto& [buffer, texture] : staticMapLightBuffers) {
        pglDeleteBuffers(1, &buffer);
        glDeleteTextures(1, &texture);
    }
    staticMapLightBuffers.clear();
    if (primitiveBuffer != 0) {
        pglDeleteBuffers(1, &primitiveBuffer);
        primitiveBuffer = 0;
    }
    if (vehicleSpotlightBuffer != 0) {
        pglDeleteBuffers(1, &vehicleSpotlightBuffer);
        vehicleSpotlightBuffer = 0;
    }
    if (vehicleSpotlightTexture != 0) {
        glDeleteTextures(1, &vehicleSpotlightTexture);
        vehicleSpotlightTexture = 0;
    }
    if (vehicleSpotlightTileMaskBuffer != 0) {
        pglDeleteBuffers(1, &vehicleSpotlightTileMaskBuffer);
        vehicleSpotlightTileMaskBuffer = 0;
    }
    if (vehicleSpotlightTileMaskTexture != 0) {
        glDeleteTextures(1, &vehicleSpotlightTileMaskTexture);
        vehicleSpotlightTileMaskTexture = 0;
    }
    vehicleSpotlightCapacity = 0;
    vehicleSpotlightTileMaskCapacity = 0;
    if (mapLightBuffer != 0) {
        pglDeleteBuffers(1, &mapLightBuffer);
        mapLightBuffer = 0;
    }
    if (mapLightTexture != 0) {
        glDeleteTextures(1, &mapLightTexture);
        mapLightTexture = 0;
    }
    if (staticMapViewBuffer != 0) {
        pglDeleteBuffers(1, &staticMapViewBuffer);
        staticMapViewBuffer = 0;
    }
    if (staticMapViewTexture != 0) {
        glDeleteTextures(1, &staticMapViewTexture);
        staticMapViewTexture = 0;
    }
    mapLightCapacity = 0;
    staticMapLightCapacity = 0;
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
    if (motionResolveProgram != 0) {
        pglDeleteProgram(motionResolveProgram);
        motionResolveProgram = 0;
    }
    if (smaaEdgeProgram != 0) {
        pglDeleteProgram(smaaEdgeProgram);
        smaaEdgeProgram = 0;
    }
    if (smaaWeightsProgram != 0) {
        pglDeleteProgram(smaaWeightsProgram);
        smaaWeightsProgram = 0;
    }
    if (smaaNeighborhoodProgram != 0) {
        pglDeleteProgram(smaaNeighborhoodProgram);
        smaaNeighborhoodProgram = 0;
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

bool beginMotionDataFrame(int width, int height, double timestampSeconds, double renderScale) {
    if (motionFrameActive) {
        throw std::logic_error("A motion-data frame is already active");
    }
    if (!std::isfinite(renderScale) || renderScale < 0.5 || renderScale > 1.0) {
        throw std::invalid_argument("Render scale must be finite and between 0.5 and 1.0");
    }
    if (width <= 0 || height <= 0) {
        motionHistory.invalidate();
        publishedMotionData.available = false;
        publishedMotionData.frame.historyValid = false;
        return false;
    }
    const int renderWidth = std::max(1, static_cast<int>(std::lround(width * renderScale)));
    const int renderHeight = std::max(1, static_cast<int>(std::lround(height * renderScale)));
    if (!ensureMotionTarget(renderWidth, renderHeight)) {
        motionHistory.invalidate();
        return false;
    }
    if (motionFrameId == std::numeric_limits<std::uint64_t>::max()) {
        motionHistory.invalidate();
        motionFrameId = 0;
    }
    const std::uint64_t frameId = ++motionFrameId;
    motionHistory.beginFrame(frameId, timestampSeconds, renderWidth, renderHeight,
                             modelViewMatrix(), projectionMatrix(), width, height);

    glGetIntegerv(GL_FRAMEBUFFER_BINDING, &motionTarget.previousFramebuffer);
    glGetIntegerv(GL_VIEWPORT, motionTarget.previousViewport);
    glGetBooleanv(GL_DEPTH_WRITEMASK, &motionTarget.previousDepthMask);
    glGetBooleanv(GL_COLOR_WRITEMASK, motionTarget.previousColorMask);
    motionTarget.previousMultisampleEnabled = glIsEnabled(GL_MULTISAMPLE);
    if (motionTarget.samples > 1) {
        glEnable(GL_MULTISAMPLE);
    }
    const GLuint renderFramebuffer = motionTarget.samples > 1
                                         ? motionTarget.multisampleFramebuffer
                                         : motionTarget.framebuffer;
    pglBindFramebuffer(GL_FRAMEBUFFER, renderFramebuffer);
    const GLenum drawBuffers[] = {GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1, GL_COLOR_ATTACHMENT2};
    pglDrawBuffers(3, drawBuffers);
    glViewport(0, 0, renderWidth, renderHeight);
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    glDepthMask(GL_TRUE);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    const GLfloat zero[] = {0.0F, 0.0F, 0.0F, 0.0F};
    const GLfloat depth = 1.0F;
    pglClearBufferfv(GL_COLOR, 1, zero);
    pglClearBufferfv(GL_COLOR, 2, zero);
    pglClearBufferfv(GL_DEPTH, 0, &depth);
    motionFrameActive = true;
    publishedMotionData.frame = motionHistory.frame();
    return true;
}

void finishMotionDataFrame() {
    if (!motionFrameActive) {
        return;
    }
    const MotionDataFrame completedFrame = motionHistory.frame();
    motionHistory.completeFrame();
    const GLuint previousFramebuffer = static_cast<GLuint>(motionTarget.previousFramebuffer);
    const GLboolean depthTestEnabled = glIsEnabled(GL_DEPTH_TEST);
    const GLboolean blendEnabled = glIsEnabled(GL_BLEND);
    const GLboolean cullEnabled = glIsEnabled(GL_CULL_FACE);
    const GLboolean scissorEnabled = glIsEnabled(GL_SCISSOR_TEST);
    GLint previousDepthFunction = GL_LESS;
    glGetIntegerv(GL_DEPTH_FUNC, &previousDepthFunction);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
    glDisable(GL_CULL_FACE);
    glDisable(GL_SCISSOR_TEST);
    glDepthMask(GL_FALSE);
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);

    const auto drawFullscreen = [&](GLuint program, const FullscreenUniforms& quadUniforms,
                                    const GLuint* textures,
                                    const GLenum* textureTargets, const GLint* samplerLocations,
                                    int textureCount, GLint metricsLocation,
                                    GLint sampleCountLocation, int sampleCount) {
        constexpr std::array<float, 16> vertices = {
            0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 1.0F, 0.0F,
            0.0F, 1.0F, 0.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F};
        useProgram(program);
        pglBindVertexArray(textureQuadVertexArray);
        currentVertexArray = textureQuadVertexArray;
        pglBindBuffer(GL_ARRAY_BUFFER, textureQuadBuffer);
        currentArrayBuffer = textureQuadBuffer;
        pglBufferData(GL_ARRAY_BUFFER, static_cast<std::ptrdiff_t>(vertices.size() * sizeof(float)),
                      vertices.data(), GL_STREAM_DRAW);
        pglVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), nullptr);
        pglVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float),
                               reinterpret_cast<const void*>(2 * sizeof(float)));
        if (quadUniforms.rect >= 0) {
            pglUniform4f(quadUniforms.rect, -1.0F, -1.0F, 2.0F, 2.0F);
        }
        if (quadUniforms.flipTextureY >= 0) {
            pglUniform1i(quadUniforms.flipTextureY, 0);
        }
        for (int index = 0; index < textureCount; ++index) {
            const GLenum unit = GL_TEXTURE0 + static_cast<GLenum>(index);
            selectTextureUnit(unit);
            if (textureTargets[index] == GL_TEXTURE_2D) {
                bindTexture(textureTargets[index], textures[index]);
            } else {
                glBindTexture(textureTargets[index], textures[index]);
            }
            if (samplerLocations[index] >= 0) {
                pglUniform1i(samplerLocations[index], index);
            }
        }
        if (metricsLocation >= 0) {
            pglUniform4f(metricsLocation, 1.0F / static_cast<float>(motionTarget.width),
                         1.0F / static_cast<float>(motionTarget.height),
                         static_cast<float>(motionTarget.width),
                         static_cast<float>(motionTarget.height));
        }
        if (sampleCountLocation >= 0) {
            pglUniform1i(sampleCountLocation, sampleCount);
        }
        selectTextureUnit(GL_TEXTURE0);
        recordTriangleSubmission(trianglesForPrimitive(GL_TRIANGLE_STRIP, 4));
        glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    };

    if (motionTarget.samples > 1) {
        pglBindFramebuffer(GL_FRAMEBUFFER, motionTarget.framebuffer);
        const GLenum drawBuffers[] = {
            GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1, GL_COLOR_ATTACHMENT2};
        pglDrawBuffers(3, drawBuffers);
        glViewport(0, 0, motionTarget.width, motionTarget.height);
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_ALWAYS);
        glDepthMask(GL_TRUE);
        const std::array<GLuint, 4> textures = {
            motionTarget.multisampleColor, motionTarget.multisampleMotionVectors,
            motionTarget.multisampleValidity, motionTarget.multisampleDepth};
        const std::array<GLenum, 4> textureTargets = {
            GL_TEXTURE_2D_MULTISAMPLE, GL_TEXTURE_2D_MULTISAMPLE,
            GL_TEXTURE_2D_MULTISAMPLE, GL_TEXTURE_2D_MULTISAMPLE};
        const std::array<GLint, 4> samplers = {
            motionResolveColorSamples, motionResolveMotionSamples,
            motionResolveValiditySamples, motionResolveDepthSamples};
        drawFullscreen(motionResolveProgram, motionResolveFullscreen, textures.data(),
                       textureTargets.data(),
                       samplers.data(), static_cast<int>(textures.size()), -1,
                       motionResolveSampleCount, motionTarget.samples);
        glDisable(GL_DEPTH_TEST);
        glDepthMask(GL_FALSE);
    }

    pglBindFramebuffer(GL_FRAMEBUFFER, previousFramebuffer);
    glViewport(motionTarget.previousViewport[0], motionTarget.previousViewport[1],
               motionTarget.previousViewport[2], motionTarget.previousViewport[3]);
    if (antiAliasingMode == AntiAliasingMode::Fxaa && textureQuadProgram != 0) {
        drawTextureQuad(motionTarget.color, -1.0F, -1.0F, 2.0F, 2.0F, false, 0, 0, true,
                        1.0F / static_cast<float>(motionTarget.width),
                        1.0F / static_cast<float>(motionTarget.height));
    } else if (antiAliasingMode == AntiAliasingMode::Smaa &&
               motionTarget.smaaEdgeFramebuffer != 0 && motionTarget.smaaWeightsFramebuffer != 0) {
        const GLfloat zero[] = {0.0F, 0.0F, 0.0F, 0.0F};
        const GLenum colorTarget[] = {GL_TEXTURE_2D};
        const GLenum edgeTargets[] = {GL_TEXTURE_2D, GL_TEXTURE_2D, GL_TEXTURE_2D};
        const GLenum neighborhoodTargets[] = {GL_TEXTURE_2D, GL_TEXTURE_2D};

        pglBindFramebuffer(GL_FRAMEBUFFER, motionTarget.smaaEdgeFramebuffer);
        glViewport(0, 0, motionTarget.width, motionTarget.height);
        pglClearBufferfv(GL_COLOR, 0, zero);
        const GLuint edgeInput[] = {motionTarget.color};
        const GLint edgeSamplers[] = {smaaEdgeColorTexture};
        drawFullscreen(smaaEdgeProgram, smaaEdgeFullscreen, edgeInput, colorTarget, edgeSamplers, 1,
                       smaaEdgeMetrics, -1, 0);

        pglBindFramebuffer(GL_FRAMEBUFFER, motionTarget.smaaWeightsFramebuffer);
        const std::array<GLuint, 3> weightInputs = {
            motionTarget.smaaEdges, smaaAreaTexture, smaaSearchTexture};
        const std::array<GLint, 3> weightSamplers = {
            smaaWeightsEdgeTexture, smaaWeightsAreaTexture, smaaWeightsSearchTexture};
        drawFullscreen(smaaWeightsProgram, smaaWeightsFullscreen, weightInputs.data(), edgeTargets,
                       weightSamplers.data(), static_cast<int>(weightInputs.size()),
                       smaaWeightsMetrics, -1, 0);

        pglBindFramebuffer(GL_FRAMEBUFFER, previousFramebuffer);
        glViewport(motionTarget.previousViewport[0], motionTarget.previousViewport[1],
                   motionTarget.previousViewport[2], motionTarget.previousViewport[3]);
        const std::array<GLuint, 2> neighborhoodInputs = {
            motionTarget.color, motionTarget.smaaWeights};
        const std::array<GLint, 2> neighborhoodSamplers = {
            smaaNeighborhoodColorTexture, smaaNeighborhoodWeightsTexture};
        drawFullscreen(smaaNeighborhoodProgram, smaaNeighborhoodFullscreen,
                       neighborhoodInputs.data(),
                       neighborhoodTargets, neighborhoodSamplers.data(),
                       static_cast<int>(neighborhoodInputs.size()), smaaNeighborhoodMetrics, -1, 0);
    } else {
        pglBindFramebuffer(GL_READ_FRAMEBUFFER, motionTarget.framebuffer);
        glReadBuffer(GL_COLOR_ATTACHMENT0);
        pglBindFramebuffer(GL_DRAW_FRAMEBUFFER, previousFramebuffer);
        const GLint destinationWidth = std::max(motionTarget.previousViewport[2], 1);
        const GLint destinationHeight = std::max(motionTarget.previousViewport[3], 1);
        const GLenum filter =
            motionTarget.width == destinationWidth && motionTarget.height == destinationHeight
                ? GL_NEAREST
                : GL_LINEAR;
        pglBlitFramebuffer(
            0, 0, motionTarget.width, motionTarget.height, motionTarget.previousViewport[0],
            motionTarget.previousViewport[1], motionTarget.previousViewport[0] + destinationWidth,
            motionTarget.previousViewport[1] + destinationHeight, GL_COLOR_BUFFER_BIT, filter);
        pglBindFramebuffer(GL_FRAMEBUFFER, previousFramebuffer);
    }
    glViewport(motionTarget.previousViewport[0], motionTarget.previousViewport[1],
               motionTarget.previousViewport[2], motionTarget.previousViewport[3]);
    glColorMask(motionTarget.previousColorMask[0], motionTarget.previousColorMask[1],
                motionTarget.previousColorMask[2], motionTarget.previousColorMask[3]);
    glDepthMask(motionTarget.previousDepthMask);
    glDepthFunc(static_cast<GLenum>(previousDepthFunction));
    if (depthTestEnabled == GL_TRUE) {
        glEnable(GL_DEPTH_TEST);
    }
    if (blendEnabled == GL_TRUE) {
        glEnable(GL_BLEND);
    }
    if (cullEnabled == GL_TRUE) {
        glEnable(GL_CULL_FACE);
    }
    if (scissorEnabled == GL_TRUE) {
        glEnable(GL_SCISSOR_TEST);
    }
    if (motionTarget.previousMultisampleEnabled == GL_TRUE) {
        glEnable(GL_MULTISAMPLE);
    } else {
        glDisable(GL_MULTISAMPLE);
    }
    motionFrameActive = false;
    publishedMotionData.frame = completedFrame;

    static const int debugMode = [] {
        const char* setting = openbus::getEnvironment("OPENBUS_MOTION_DEBUG");
        if (setting == nullptr || *setting == '\0' || std::string_view(setting) == "0") {
            return 0;
        }
        if (std::string_view(setting) == "vectors") {
            return 1;
        }
        if (std::string_view(setting) == "magnitude") {
            return 2;
        }
        gameLog.Warning("Invalid OPENBUS_MOTION_DEBUG; expected vectors or magnitude");
        return 0;
    }();
    if (debugMode != 0) {
        const GLboolean depthTestEnabled = glIsEnabled(GL_DEPTH_TEST);
        const GLboolean blendEnabled = glIsEnabled(GL_BLEND);
        glDisable(GL_DEPTH_TEST);
        glDisable(GL_BLEND);
        glDepthMask(GL_FALSE);
        glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
        drawTextureQuad(motionTarget.motionVectors, -1.0F, -1.0F, 2.0F, 2.0F, false, debugMode,
                        motionTarget.validity);
        glDepthMask(motionTarget.previousDepthMask);
        glColorMask(motionTarget.previousColorMask[0], motionTarget.previousColorMask[1],
                    motionTarget.previousColorMask[2], motionTarget.previousColorMask[3]);
        if (depthTestEnabled == GL_TRUE) {
            glEnable(GL_DEPTH_TEST);
        }
        if (blendEnabled == GL_TRUE) {
            glEnable(GL_BLEND);
        }
    }
}

void setAntiAliasingMode(std::string_view mode) {
    if (mode == "off") {
        antiAliasingMode = AntiAliasingMode::Off;
    } else if (mode == "fxaa") {
        antiAliasingMode = AntiAliasingMode::Fxaa;
    } else if (mode == "msaa") {
        antiAliasingMode = AntiAliasingMode::Msaa;
    } else if (mode == "smaa") {
        antiAliasingMode = AntiAliasingMode::Smaa;
    } else {
        throw std::invalid_argument("Anti-aliasing mode must be off, fxaa, msaa or smaa");
    }
}

std::string_view effectiveAntiAliasingMode() noexcept {
    switch (antiAliasingMode) {
    case AntiAliasingMode::Fxaa:
        return "fxaa";
    case AntiAliasingMode::Msaa:
        return "msaa";
    case AntiAliasingMode::Smaa:
        return "smaa";
    case AntiAliasingMode::Off:
    default:
        return "off";
    }
}

int motionDataMultisampleCount() noexcept {
    return motionTarget.samples;
}

void invalidateMotionDataHistory() {
    motionHistory.invalidate();
    publishedMotionData.frame.historyValid = false;
}

MotionDataResources motionDataResources() {
    return publishedMotionData;
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

LightingBufferUploadStats lightingBufferUploadStats() {
    return lightUploadStats;
}

void resetLightingBufferUploadStats() {
    lightUploadStats = {};
}

void recordMapLightViewPreparation(bool reused) {
    if (reused) {
        ++lightUploadStats.mapViewReuses;
    } else {
        ++lightUploadStats.mapViewPreparations;
    }
}

namespace {
void uploadDynamicLightBuffer(GLuint buffer, std::size_t bytes, const void* data,
                              const char* description) {
    TraceScope trace("lighting", "uploadDynamicLightBuffer");
    if (bytes > static_cast<std::size_t>(std::numeric_limits<std::ptrdiff_t>::max())) {
        throw std::length_error(std::string(description) + " upload is too large");
    }
    const std::uint64_t errorCheckpoint = glDebugErrorCheckpoint();
    pglBindBuffer(GL_TEXTURE_BUFFER, buffer);
    {
        TraceScope bufferTrace("lighting", "uploadDynamicLightBuffer.bufferData");
        pglBufferData(GL_TEXTURE_BUFFER, static_cast<std::ptrdiff_t>(bytes), data, GL_DYNAMIC_DRAW);
    }
    pglBindBuffer(GL_TEXTURE_BUFFER, 0);
    validateOpenGlUpload(description, errorCheckpoint);
}
} // namespace

void setVehicleSpotlights(const std::vector<VehicleSpotlight>& lights) {
    TraceScope trace("lighting", "setVehicleSpotlights");
    if (vehicleSpotlightBuffer == 0 || vehicleSpotlightTexture == 0 ||
        vehicleSpotlightTileMaskBuffer == 0 || vehicleSpotlightTileMaskTexture == 0) {
        throw std::invalid_argument("Invalid vehicle spotlight collection");
    }
    const std::size_t activeLightCount = std::min(lights.size(), vehicleSpotlightLimit());
    if (activeLightCount > vehicleSpotlightCapacity ||
        activeLightCount > static_cast<std::size_t>(std::numeric_limits<GLint>::max() / 3) ||
        activeLightCount > static_cast<std::size_t>(std::numeric_limits<std::ptrdiff_t>::max()) /
                               (3 * sizeof(std::array<float, 4>))) {
        throw std::length_error("Vehicle spotlight count " + std::to_string(activeLightCount) +
                                " exceeds the OpenGL texture-buffer capacity " +
                                std::to_string(vehicleSpotlightCapacity));
    }
    for (const VehicleSpotlight& light : lights) {
        const auto finite = [](double value) {
            return std::isfinite(value) && std::abs(value) <= std::numeric_limits<float>::max();
        };
        if (!std::all_of(light.position.begin(), light.position.end(), finite) ||
            !std::all_of(light.direction.begin(), light.direction.end(), finite) ||
            !std::all_of(light.color.begin(), light.color.end(), finite) || !finite(light.range) ||
            light.range < std::numeric_limits<float>::min() ||
            std::hypot(light.direction[0], light.direction[1], light.direction[2]) < 1e-12 ||
            !finite(light.innerAngleDegrees) || !finite(light.outerAngleDegrees) ||
            light.innerAngleDegrees < 0.0 || light.innerAngleDegrees > light.outerAngleDegrees ||
            light.outerAngleDegrees > 180.0 ||
            std::any_of(light.color.begin(), light.color.end(),
                        [](double value) { return value < 0.0 || value > 1.0; })) {
            throw std::invalid_argument("Invalid vehicle spotlight");
        }
    }
    detail::VehicleSpotlightTileMask tileMask;
    if (activeLightCount > 0 && vehicleSpotlightTileCullingEnabled()) {
        std::array<GLint, 4> glViewport = {};
        glGetIntegerv(GL_VIEWPORT, glViewport.data());
        const std::array<int, 4> viewport = {glViewport[0], glViewport[1], glViewport[2],
                                             glViewport[3]};
        tileMask = detail::buildVehicleSpotlightTileMask(
            std::span<const VehicleSpotlight>(lights.data(), activeLightCount), projectionMatrix(),
            viewport, vehicleSpotlightTileMaskCapacity, 0.75);
    }
    ++lightUploadStats.vehicleSets;
    lightUploadStats.vehicleLightsTotal += activeLightCount;
    lightUploadStats.vehicleLightsMax =
        std::max(lightUploadStats.vehicleLightsMax, activeLightCount);
    std::vector<std::array<float, 4>> gpuLights(activeLightCount * 3);
    for (std::size_t index = 0; index < activeLightCount; ++index) {
        const VehicleSpotlight& light = lights[index];
        const double length =
            std::hypot(light.direction[0], light.direction[1], light.direction[2]);
        gpuLights[index * 3] = {
            static_cast<float>(light.position[0]), static_cast<float>(light.position[1]),
            static_cast<float>(light.position[2]), static_cast<float>(light.range)};
        constexpr double halfDegreesToRadians = 3.14159265358979323846 / 360.0;
        gpuLights[index * 3 + 1] = {
            static_cast<float>(light.direction[0] / length),
            static_cast<float>(light.direction[1] / length),
            static_cast<float>(light.direction[2] / length),
            static_cast<float>(std::cos(light.innerAngleDegrees * halfDegreesToRadians))};
        gpuLights[index * 3 + 2] = {
            static_cast<float>(light.color[0]), static_cast<float>(light.color[1]),
            static_cast<float>(light.color[2]),
            static_cast<float>(std::cos(light.outerAngleDegrees * halfDegreesToRadians))};
    }
    const std::size_t bytes = gpuLights.size() * sizeof(gpuLights[0]);
    if (activeLightCount > 0 &&
        (uploadedVehicleLights.size() != gpuLights.size() ||
         std::memcmp(uploadedVehicleLights.data(), gpuLights.data(), bytes) != 0)) {
        uploadedVehicleLights.clear();
        uploadDynamicLightBuffer(vehicleSpotlightBuffer, bytes, gpuLights.data(),
                                 "vehicle spotlight");
        uploadedVehicleLights = std::move(gpuLights);
        ++lightUploadStats.vehicleUploads;
        lightUploadStats.vehicleBytes += bytes;
    }
    if (tileMask.enabled) {
        ++lightUploadStats.vehicleSpotlightTileSets;
        lightUploadStats.vehicleSpotlightTileCountTotal +=
            static_cast<std::uint64_t>(tileMask.columns) * tileMask.rows;
        lightUploadStats.vehicleSpotlightTileAssignmentsTotal += tileMask.assignedReferences;
        lightUploadStats.vehicleSpotlightTileAssignmentsMax = std::max(
            lightUploadStats.vehicleSpotlightTileAssignmentsMax, tileMask.maximumLightsPerTile);
        if (uploadedVehicleSpotlightTileMask != tileMask.words) {
            const std::size_t maskBytes = tileMask.words.size() * sizeof(tileMask.words[0]);
            uploadedVehicleSpotlightTileMask.clear();
            uploadDynamicLightBuffer(vehicleSpotlightTileMaskBuffer, maskBytes,
                                     tileMask.words.data(), "vehicle spotlight tile mask");
            uploadedVehicleSpotlightTileMask = tileMask.words;
            ++lightUploadStats.vehicleSpotlightTileUploads;
            lightUploadStats.vehicleSpotlightTileBytes += maskBytes;
        }
    } else if (activeLightCount > 0) {
        ++lightUploadStats.vehicleSpotlightTileFallbackSets;
    }
    const GLuint previousProgram = currentProgram;
    for (const auto& entry :
         {std::pair{modelProgram, &modelUniforms}, std::pair{materialProgram, &materialUniforms}}) {
        useProgram(entry.first);
        const Uniforms& uniforms = *entry.second;
        pglUniform1i(uniforms.spotlightCount, static_cast<GLint>(activeLightCount));
        pglUniform1i(uniforms.spotlightTileMaskEnabled, tileMask.enabled ? 1 : 0);
        pglUniform1i(uniforms.spotlightTileColumns, std::max(tileMask.columns, 1));
        pglUniform1i(uniforms.spotlightTileRows, std::max(tileMask.rows, 1));
        pglUniform1i(uniforms.spotlightTileWordsPerTile, std::max(tileMask.wordsPerTile, 1));
        pglUniform1i(uniforms.spotlightTileSize, detail::VEHICLE_SPOTLIGHT_TILE_SIZE);
        pglUniform2f(uniforms.spotlightViewportOrigin, static_cast<float>(tileMask.viewportX),
                     static_cast<float>(tileMask.viewportY));
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
    TraceScope trace("lighting", "setMapLights");
    const auto finite = [](double value) {
        return std::isfinite(value) && std::abs(value) <= std::numeric_limits<float>::max();
    };
    if (!finite(strength) || strength < 0.0 || strength > 1.0 || mapLightBuffer == 0 ||
        mapLightTexture == 0) {
        throw std::invalid_argument("Invalid map-light collection");
    }
    for (const MapPointLight& light : lights) {
        if (!validMapLight(light)) {
            throw std::invalid_argument("Invalid map light");
        }
    }
    const std::size_t activeLightCount =
        strength > 0.0 ? std::min(lights.size(), mapLightLimit()) : 0;
    if (activeLightCount > mapLightCapacity ||
        activeLightCount > static_cast<std::size_t>(std::numeric_limits<GLint>::max() / 2) ||
        activeLightCount > static_cast<std::size_t>(std::numeric_limits<std::ptrdiff_t>::max()) /
                               (2 * sizeof(std::array<float, 4>))) {
        throw std::length_error("Map light count " + std::to_string(activeLightCount) +
                                " exceeds the OpenGL texture-buffer capacity " +
                                std::to_string(mapLightCapacity));
    }
    ++lightUploadStats.mapSets;
    lightUploadStats.mapLightsTotal += activeLightCount;
    lightUploadStats.mapLightsMax = std::max(lightUploadStats.mapLightsMax, activeLightCount);
    if (activeLightCount > 0) {
        std::vector<std::array<float, 4>> gpuLights(activeLightCount * 2);
        for (std::size_t index = 0; index < activeLightCount; ++index) {
            const MapPointLight& light = lights[index];
            gpuLights[index * 2] = {
                static_cast<float>(light.position[0]), static_cast<float>(light.position[1]),
                static_cast<float>(light.position[2]), static_cast<float>(light.radius)};
            gpuLights[index * 2 + 1] = {static_cast<float>(light.color[0]),
                                        static_cast<float>(light.color[1]),
                                        static_cast<float>(light.color[2]), 0.0F};
        }
        const std::size_t bytes = gpuLights.size() * sizeof(gpuLights[0]);
        if (uploadedMapLights.size() != gpuLights.size() ||
            std::memcmp(uploadedMapLights.data(), gpuLights.data(), bytes) != 0) {
            uploadedMapLights.clear();
            uploadDynamicLightBuffer(mapLightBuffer, bytes, gpuLights.data(), "map light");
            uploadedMapLights = std::move(gpuLights);
            ++lightUploadStats.mapUploads;
            lightUploadStats.mapBytes += bytes;
        }
    }
    const GLuint previousProgram = currentProgram;
    for (const auto& entry :
         {std::pair{modelProgram, &modelUniforms}, std::pair{materialProgram, &materialUniforms}}) {
        useProgram(entry.first);
        const Uniforms& uniforms = *entry.second;
        pglUniform1i(uniforms.mapLightCount, static_cast<GLint>(activeLightCount));
        pglUniform1f(uniforms.mapLightStrength, static_cast<float>(strength));
        pglUniform1i(uniforms.staticMapMode, 0);
    }
    useProgram(previousProgram);
}

void setStaticMapLights(StaticMapLightBuffer& cache, const std::vector<MapPointLight>& sources,
                        const std::vector<std::size_t>& sourceIndices,
                        const std::vector<StaticMapLightView>& viewLights, double strength,
                        std::uint64_t revision) {
    TraceScope trace("lighting", "setStaticMapLights");
    if (!std::isfinite(strength) || strength < 0.0 || strength > 1.0 || staticMapViewBuffer == 0 ||
        staticMapViewTexture == 0) {
        throw std::invalid_argument("Invalid static map-light collection");
    }
    const std::size_t count = strength > 0.0 ? std::min(viewLights.size(), mapLightLimit()) : 0;
    if (count == 0) {
        setMapLights({}, strength);
        return;
    }
    const std::size_t maxIntLights = static_cast<std::size_t>(std::numeric_limits<GLint>::max());
    const std::size_t maxBytesLights =
        static_cast<std::size_t>(std::numeric_limits<std::ptrdiff_t>::max()) /
        sizeof(std::array<float, 4>);
    if (count > staticMapLightCapacity || count > maxIntLights ||
        count > static_cast<std::size_t>(std::numeric_limits<std::ptrdiff_t>::max()) /
                    sizeof(std::array<std::uint32_t, 4>)) {
        throw std::length_error("Static map-light collection exceeds texture-buffer capacity " +
                                std::to_string(staticMapLightCapacity));
    }
    const std::size_t sourceCapacity =
        std::min({staticMapLightCapacity, maxIntLights, maxBytesLights});
    if (auto compact = detail::compactStaticMapLightSelection(sources, sourceIndices, viewLights,
                                                              count, sourceCapacity)) {
        static bool fallbackLogged = false;
        if (!fallbackLogged) {
            gameLog.Warning("Static map-light source set exceeds buffer capacity; using a compact "
                            "static buffer for the selected lights");
            fallbackLogged = true;
        }
        setStaticMapLights(cache, sources, compact->sourceIndices, compact->viewLights, strength,
                           revision);
        return;
    }
    const auto finite = [](double value) {
        return std::isfinite(value) && std::abs(value) <= std::numeric_limits<float>::max();
    };
    for (const std::size_t sourceIndex : sourceIndices) {
        if (sourceIndex >= sources.size()) {
            throw std::invalid_argument("Invalid static map-light source");
        }
    }
    if (cache.rendererGeneration != rendererGeneration) {
        cache = {};
        cache.rendererGeneration = rendererGeneration;
    }
    if (sources.size() <= sourceCapacity &&
        (!cache.sourceValidationCached || cache.validatedSourceCount != sources.size() ||
         cache.validatedRevision != revision)) {
        cache.allSourcesValid = std::all_of(sources.begin(), sources.end(), validMapLight);
        cache.validatedSourceCount = sources.size();
        cache.validatedRevision = revision;
        cache.sourceValidationCached = true;
    }
    const bool residentSources = sources.size() <= sourceCapacity && cache.allSourcesValid;
    std::vector<std::array<std::uint32_t, 4>> gpuView(count);
    for (std::size_t index = 0; index < count; ++index) {
        const auto& light = viewLights[index];
        if (light.sourceIndex >= sourceIndices.size() ||
            !std::all_of(light.position.begin(), light.position.end(), finite)) {
            throw std::invalid_argument("Invalid static map-light view");
        }
        gpuView[index] = {std::bit_cast<std::uint32_t>(static_cast<float>(light.position[0])),
                          std::bit_cast<std::uint32_t>(static_cast<float>(light.position[1])),
                          std::bit_cast<std::uint32_t>(static_cast<float>(light.position[2])),
                          static_cast<std::uint32_t>(residentSources
                                                         ? sourceIndices[light.sourceIndex]
                                                         : light.sourceIndex)};
    }
    const std::size_t storedSourceCount = residentSources ? sources.size() : sourceIndices.size();
    if (cache.buffer == 0 || !cache.sourceUploaded || cache.contentRevision != revision ||
        cache.residentSources != residentSources || cache.sourceCount != storedSourceCount ||
        (!residentSources && cache.sourceIndices != sourceIndices)) {
        TraceScope sourceTrace("lighting", "setStaticMapLights.uploadSources");
        std::vector<std::array<float, 4>> gpuSources(storedSourceCount);
        for (std::size_t index = 0; index < storedSourceCount; ++index) {
            const auto& light = sources[residentSources ? index : sourceIndices[index]];
            if (!validMapLight(light)) {
                throw std::invalid_argument("Invalid static map-light source");
            }
            gpuSources[index] = {
                static_cast<float>(light.color[0]), static_cast<float>(light.color[1]),
                static_cast<float>(light.color[2]), static_cast<float>(light.radius)};
        }
        if (cache.buffer == 0) {
            staticMapLightBuffers.reserve(staticMapLightBuffers.size() + 1);
            pglGenBuffers(1, &cache.buffer);
            glGenTextures(1, &cache.texture);
            if (cache.buffer == 0 || cache.texture == 0) {
                if (cache.buffer != 0) {
                    pglDeleteBuffers(1, &cache.buffer);
                }
                if (cache.texture != 0) {
                    glDeleteTextures(1, &cache.texture);
                }
                cache = {};
                throw std::runtime_error("Unable to allocate static map-light buffer");
            }
            staticMapLightBuffers.emplace_back(cache.buffer, cache.texture);
        }
        const std::size_t bytes = gpuSources.size() * sizeof(gpuSources[0]);
        cache.sourceUploaded = false;
        const std::uint64_t errorCheckpoint = glDebugErrorCheckpoint();
        pglBindBuffer(GL_TEXTURE_BUFFER, cache.buffer);
        pglBufferData(GL_TEXTURE_BUFFER, static_cast<std::ptrdiff_t>(bytes), gpuSources.data(),
                      GL_STATIC_DRAW);
        pglBindBuffer(GL_TEXTURE_BUFFER, 0);
        validateOpenGlUpload("static map-light source", errorCheckpoint);
        const GLenum previousUnit = currentTextureUnit;
        selectTextureUnit(staticMapLightTextureUnit);
        glBindTexture(GL_TEXTURE_BUFFER, cache.texture);
        pglTexBuffer(GL_TEXTURE_BUFFER, GL_RGBA32F, cache.buffer);
        selectTextureUnit(previousUnit);
        validateOpenGlUpload("static map-light texture binding", errorCheckpoint);
        cache.sourceIndices = residentSources ? std::vector<std::size_t>{} : sourceIndices;
        cache.sourceCount = storedSourceCount;
        cache.residentSources = residentSources;
        cache.contentRevision = revision;
        cache.sourceUploaded = true;
        ++lightUploadStats.staticMapUploads;
        lightUploadStats.staticMapBytes += bytes;
    }
    const std::size_t viewBytes = gpuView.size() * sizeof(gpuView[0]);
    if (uploadedStaticMapView.size() != gpuView.size() ||
        std::memcmp(uploadedStaticMapView.data(), gpuView.data(), viewBytes) != 0) {
        TraceScope viewTrace("lighting", "setStaticMapLights.uploadView");
        uploadedStaticMapView.clear();
        const std::uint64_t errorCheckpoint = glDebugErrorCheckpoint();
        pglBindBuffer(GL_TEXTURE_BUFFER, staticMapViewBuffer);
        {
            TraceScope bufferTrace("lighting", "setStaticMapLights.uploadView.bufferData");
            pglBufferData(GL_TEXTURE_BUFFER, static_cast<std::ptrdiff_t>(viewBytes), gpuView.data(),
                          GL_DYNAMIC_DRAW);
        }
        pglBindBuffer(GL_TEXTURE_BUFFER, 0);
        validateOpenGlUpload("static map-light view", errorCheckpoint);
        uploadedStaticMapView = std::move(gpuView);
        ++lightUploadStats.mapUploads;
        lightUploadStats.mapBytes += viewBytes;
    }
    const GLenum previousUnit = currentTextureUnit;
    selectTextureUnit(staticMapLightTextureUnit);
    glBindTexture(GL_TEXTURE_BUFFER, cache.texture);
    selectTextureUnit(previousUnit);
    ++lightUploadStats.mapSets;
    lightUploadStats.mapLightsTotal += count;
    lightUploadStats.mapLightsMax = std::max(lightUploadStats.mapLightsMax, count);
    const GLuint previousProgram = currentProgram;
    for (const auto& entry :
         {std::pair{modelProgram, &modelUniforms}, std::pair{materialProgram, &materialUniforms}}) {
        useProgram(entry.first);
        const Uniforms& uniforms = *entry.second;
        pglUniform1i(uniforms.mapLightCount, static_cast<GLint>(count));
        pglUniform1f(uniforms.mapLightStrength, static_cast<float>(strength));
        pglUniform1i(uniforms.staticMapMode, 1);
    }
    useProgram(previousProgram);
}

void releaseStaticMapLights(StaticMapLightBuffer& cache) {
    if (cache.buffer != 0 && cache.rendererGeneration == rendererGeneration) {
        const auto found = std::find(staticMapLightBuffers.begin(), staticMapLightBuffers.end(),
                                     std::pair{cache.buffer, cache.texture});
        if (found != staticMapLightBuffers.end()) {
            pglDeleteBuffers(1, &cache.buffer);
            glDeleteTextures(1, &cache.texture);
            staticMapLightBuffers.erase(found);
        }
    }
    cache = {};
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
        std::max(0.58F * value, reflectionPass ? reflectionAmbientFloor() : 0.0F);
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
                    double visibilityFade, const HumanSkinDraw* humanSkin,
                    MotionObjectKey motionObject) {
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
    uploadMotionMatrices(modelUniforms, motionObject,
                         !skinHuman && !animateHuman && alphaMode != 2 && alphaMode != 3);
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
                           double alpha, int alphaMode, bool cullBackFaces, double visibilityFade,
                           MotionObjectKey motionObject) {
    TraceScope trace("render", "CoreRenderer::drawIndexedModelBatch");
    if (modelProgram == 0 || buffer == 0 || indexBuffer == 0 || indexCount == 0) {
        return;
    }
    useProgram(modelProgram);
    uploadMatrices(modelUniforms);
    uploadModelUniforms(material, color, alpha, alphaMode, visibilityFade);
    uploadMotionMatrices(modelUniforms, motionObject, alphaMode != 2 && alphaMode != 3);
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
                       GLuint indexBuffer, double visibilityFade, GLuint textTexture,
                       bool useTextTexture, MotionObjectKey motionObject) {
    TraceScope trace("render", "CoreRenderer::drawMaterialBatch");
    if (materialProgram == 0 || buffer == 0 || vertexCount == 0 || colors.empty() ||
        textures.empty()) {
        return;
    }
    useProgram(materialProgram);
    uploadMatrices(materialUniforms);
    uploadMotionMatrices(materialUniforms, motionObject, glIsEnabled(GL_BLEND) == GL_FALSE);
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
    const bool textTextureEnabled = useTextTexture && textTexture != 0;
    pglUniform1i(materialUniforms.useTextTexture, textTextureEnabled ? 1 : 0);
    selectTextureUnit(GL_TEXTURE0 + 8);
    bindTexture(GL_TEXTURE_2D, textTextureEnabled ? textTexture : 0);
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
        const std::uint64_t errorCheckpoint = glDebugErrorCheckpoint();
        {
            TraceScope phase("render", "CoreRenderer::drawStaticPrimitives.upload");
            pglBufferData(GL_ARRAY_BUFFER,
                          static_cast<std::ptrdiff_t>(vertices.size() * sizeof(PrimitiveVertex)),
                          vertices.data(), GL_STATIC_DRAW);
        }
        try {
            validateOpenGlUpload("static primitive", errorCheckpoint);
        } catch (const std::runtime_error&) {
            gameLog.Warning("OpenGL error during static primitive upload; skipping this primitive "
                            "and retrying the upload later");
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

void drawTextureQuad(GLuint texture, float x, float y, float width, float height, bool flipTextureY,
                     int motionDebugMode, GLuint motionValidityTexture, bool fxaa,
                     float inverseWidth, float inverseHeight) {
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
    pglUniform1i(textureQuadMotionDebugMode, motionDebugMode);
    pglUniform1i(textureQuadFxaaEnabled, fxaa ? 1 : 0);
    pglUniform2f(textureQuadInverseResolution, inverseWidth, inverseHeight);
    selectTextureUnit(GL_TEXTURE0 + 1);
    bindTexture(GL_TEXTURE_2D, motionValidityTexture);
    selectTextureUnit(GL_TEXTURE0);
    bindTexture(GL_TEXTURE_2D, texture);
    recordTriangleSubmission(trianglesForPrimitive(GL_TRIANGLE_STRIP, 4));
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
}

} // namespace openbus::rendering
