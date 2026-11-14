#pragma once

#include "render/CameraMath.h"
#include "render/GlCompat.h"
#include "render/MotionData.h"
#include "core/WeatherState.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace openbus::rendering {

constexpr std::size_t MAX_INTERIOR_LIGHTS = 4;

enum class GpuTimingPass : std::uint8_t { MainView, Reflection };

class GpuTimerScope {
  public:
    explicit GpuTimerScope(GpuTimingPass pass);
    ~GpuTimerScope();

    GpuTimerScope(const GpuTimerScope&) = delete;
    GpuTimerScope& operator=(const GpuTimerScope&) = delete;

  private:
    bool active_ = false;
};

void beginGpuTimingFrame();

struct VehicleSpotlight {
    std::array<double, 3> position = {};
    std::array<double, 3> direction = {};
    std::array<double, 3> color = {};
    double range = 0.0;
    double innerAngleDegrees = 0.0;
    double outerAngleDegrees = 0.0;
};

struct MapPointLight {
    std::array<double, 3> position = {};
    std::array<double, 3> color = {};
    double radius = 0.0;
};

struct StaticMapLightBuffer {
    GLuint buffer = 0;
    GLuint texture = 0;
    std::uint64_t rendererGeneration = 0;
    std::uint64_t contentRevision = 0;
    std::size_t sourceCount = 0;
    bool sourceUploaded = false;
    bool residentSources = false;
    std::size_t validatedSourceCount = 0;
    std::uint64_t validatedRevision = 0;
    bool sourceValidationCached = false;
    bool allSourcesValid = false;
    std::vector<std::size_t> sourceIndices;
};

struct StaticMapLightView {
    std::array<double, 3> position = {};
    // Index into the sourceIndices argument, not directly into the source array.
    std::size_t sourceIndex = 0;
};

enum class HumanBoneSlot : std::uint8_t {
    LeftUpperLeg = 0,
    RightUpperLeg = 1,
    LeftLowerLeg = 2,
    RightLowerLeg = 3,
    LeftUpperArm = 4,
    RightUpperArm = 5,
    LeftLowerArm = 6,
    RightLowerArm = 7,
    Hip = 8,
    Main = 9,
    Head = 10,
    LeftHand = 11,
    RightHand = 12,
    Count = 13,
};
inline constexpr std::size_t HUMAN_BONE_COUNT = static_cast<std::size_t>(HumanBoneSlot::Count);
inline constexpr std::size_t HUMAN_WEIGHT_COUNT = 16;

constexpr std::size_t humanBoneSlotIndex(HumanBoneSlot slot) noexcept {
    return static_cast<std::size_t>(slot);
}

struct PrimitiveVertex {
    float x;
    float y;
    float z;
    float r;
    float g;
    float b;
};

// Renderer-owned GL storage for cached primitive vertices.
struct StaticPrimitiveBuffer {
    GLuint buffer = 0;
    std::size_t vertexCount = 0;
    std::uint64_t contentRevision = 0;
    std::uint64_t rendererGeneration = 0;

    bool requiresUpload(std::size_t requestedVertexCount,
                        std::uint64_t requestedContentRevision) const noexcept {
        return buffer == 0 || vertexCount != requestedVertexCount ||
               contentRevision != requestedContentRevision;
    }
};

inline constexpr double ENVIRONMENT_MAP_OPACITY = 0.1;

struct ModelMaterial {
    GLuint texture = 0;
    GLuint detailTexture = 0;
    bool textureArray = false;
    bool textured = false;
    bool textureAlphaOnly = false;
    bool useDetailTexture = false;
    std::array<float, 2> textureScale = {1.0F, 1.0F};
    std::array<float, 2> detailTextureScale = {1.0F, 1.0F};
    GLenum textureWrapS = GL_REPEAT;
    GLenum textureWrapT = GL_REPEAT;
    GLuint lightmap = 0;
    GLuint nightmap = 0;
    GLuint transmap = 0;
    std::array<float, 2> transmapScale = {1.0F, 1.0F};
    std::array<float, 3> specularColor = {};
    std::array<float, 3> emissionColor = {};
    float specularPower = 8.0F;
    float snowCoverage = 0.0F;
    bool flipTransmapY = false;
    GLuint bumpmap = 0;
    GLuint environmentMap = 0;
    GLuint freeTexture = 0;
    GLuint textTexture = 0;
    bool useLightmap = false;
    bool useNightmap = false;
    bool useTransmap = false;
    bool useBumpmap = false;
    bool useEnvironmentMap = false;
    bool useFreeTexture = false;
    bool useTextTexture = false;
    bool weatherLitDynamicTexture = false;
    bool whiteAmbient = false;
    bool useInteriorLight = false;
    bool flipTextureY = false;
    float lightmapStrength = 0.0f;
    float nightmapStrength = 0.0f;
    float bumpmapStrength = 0.0f;
    float environmentStrength = 0.0f;
    int interiorLightCount = 0;
    // Positions are in view space, transformed from the vehicle root (not the
    // receiving mesh's per-part animation transform).
    std::array<std::array<float, 3>, MAX_INTERIOR_LIGHTS> interiorLightViewPositions = {};
    std::array<std::array<float, 3>, MAX_INTERIOR_LIGHTS> interiorLightColors = {};
    std::array<float, MAX_INTERIOR_LIGHTS> interiorLightStrengths = {};
    float texcoordOffsetX = 0.0f;
    float texcoordOffsetY = 0.0f;
};

struct HumanGaitParameters {
    double phaseRadians = 0.0;
    double weight = 0.0;
    double heightMeters = 0.0;
    double upperArmBetaDegrees = 66.0;
    double armSwing = 1.0;
    double hipSway = 1.0;
    double waistAnimation = 0.0;
    double shoulderXHeightFraction = 0.105;
    double shoulderZHeightFraction = 0.76;
    double hipXHeightFraction = 0.06;
    double hipZHeightFraction = 0.5;
    double kneeXHeightFraction = 0.08;
    double kneeYHeightFraction = 0.0;
    double kneeZHeightFraction = 0.25;
    double elbowXHeightFraction = 0.25;
    double elbowZHeightFraction = 0.76;
    double handXHeightFraction = 0.40;
    double handZHeightFraction = 0.76;
    double fingerXHeightFraction = 0.5;
    double fingerZHeightFraction = 0.76;
    double neckYHeightFraction = 0.0;
    double neckZHeightFraction = 0.85;
    double waistYHeightFraction = 0.0;
    double waistZHeightFraction = 0.6;
};

struct HumanSkinDraw {
    GLuint weightBuffer = 0;
    const std::array<Matrix4, HUMAN_BONE_COUNT>* palette = nullptr;
};

struct MotionDataResources {
    GLuint motionVectors = 0;
    GLuint validity = 0;
    GLuint depth = 0;
    int width = 0;
    int height = 0;
    MotionDataFrame frame;
    bool available = false;
};

bool initializeCoreRenderer();
void shutdownCoreRenderer();
bool beginMotionDataFrame(int width, int height, double timestampSeconds, double renderScale = 1.0);
void finishMotionDataFrame();
void invalidateMotionDataHistory();
void setFxaaEnabled(bool enabled);
[[nodiscard]] MotionDataResources motionDataResources();
void invalidateTextureBindings();
// Use for renderer-visible buffers so cached VAO and compute bindings cannot
// retain a deleted name that OpenGL may later reuse.
void deleteRendererBuffers(GLsizei count, const GLuint* buffers);
// Call with the camera view before applying any object transforms.
void setLightingViewMatrix(const Matrix4& view,
                           std::optional<std::array<double, 3>> sunDirection = std::nullopt);
struct LightingBufferUploadStats {
    std::uint64_t vehicleUploads = 0;
    std::uint64_t vehicleBytes = 0;
    std::uint64_t vehicleSets = 0;
    std::uint64_t vehicleLightsTotal = 0;
    std::size_t vehicleLightsMax = 0;
    std::uint64_t vehicleSpotlightTileUploads = 0;
    std::uint64_t vehicleSpotlightTileBytes = 0;
    std::uint64_t vehicleSpotlightTileSets = 0;
    std::uint64_t vehicleSpotlightTileCountTotal = 0;
    std::uint64_t vehicleSpotlightTileAssignmentsTotal = 0;
    std::size_t vehicleSpotlightTileAssignmentsMax = 0;
    std::uint64_t vehicleSpotlightTileFallbackSets = 0;
    std::uint64_t mapUploads = 0;
    std::uint64_t mapBytes = 0;
    std::uint64_t staticMapUploads = 0;
    std::uint64_t staticMapBytes = 0;
    std::uint64_t mapViewPreparations = 0;
    std::uint64_t mapViewReuses = 0;
    std::uint64_t mapSets = 0;
    std::uint64_t mapLightsTotal = 0;
    std::size_t mapLightsMax = 0;
};

LightingBufferUploadStats lightingBufferUploadStats();
void resetLightingBufferUploadStats();
void recordMapLightViewPreparation(bool reused);
std::size_t mapLightSelectionLimit();
// Positions and directions are in the current view space.
void setVehicleSpotlights(const std::vector<VehicleSpotlight>& lights);
// Map-light positions are in the current view space; strength normally follows darkness.
void setMapLights(const std::vector<MapPointLight>& lights, double strength);
// Source indices refer to immutable registered lights; increment revision if they change.
// View indices refer to the compact sourceIndices array.
// Increment revision when existing sources change or their order changes.
void setStaticMapLights(StaticMapLightBuffer& cache, const std::vector<MapPointLight>& sources,
                        const std::vector<std::size_t>& sourceIndices,
                        const std::vector<StaticMapLightView>& viewLights, double strength,
                        std::uint64_t revision = 0);
void releaseStaticMapLights(StaticMapLightBuffer& cache);
// An absent brightness preserves the renderer's default daylight appearance.
std::array<float, 3> weatherBackgroundColor(std::optional<double> brightness);
void setWeatherLighting(std::optional<double> brightness, bool reflectionPass = false);
// Visibility is the distance at which approximately 2% of scene contrast remains.
// An absent value disables fog.
void setWeatherFog(std::optional<double> visibilityMeters,
                   std::optional<double> layerHeightMeters = std::nullopt);
// Call after setting each camera view; ground height is the local terrain reference.
void setWeatherFogView(const Matrix4& view, double groundHeightMeters);
// Draws an optional procedural cloud plane into the current view before scene geometry.
void drawWeatherClouds(std::optional<openbus::WeatherCloudType> type,
                       std::optional<double> baseAboveGroundMeters,
                       std::optional<double> windDirectionDegrees,
                       std::optional<double> windSpeedMetersPerSecond,
                       std::optional<double> visibilityMeters, std::optional<double> brightness,
                       double groundHeightMeters, double simulationSeconds);
// Draw a bounded camera-near, depth-tested world-space precipitation field.
void drawWeatherPrecipitation(int type, double rate, double simulationSeconds,
                              std::optional<double> windDirectionDegrees,
                              std::optional<double> windSpeedMetersPerSecond,
                              std::optional<double> brightness,
                              std::optional<double> visibilityMeters);
void configureModelAlphaState(int alphaMode, bool noZwrite);
// Frame-local total of triangle primitives submitted through the renderer.
void resetFrameTriangleCount();
void resetFrameReflectionTriangleCount();
std::size_t frameTriangleCount();
std::size_t frameReflectionTriangleCount();
void setFrameTriangleCountReflectionPass(bool reflectionPass);

void drawModelBatch(GLuint buffer, std::size_t vertexCount, const ModelMaterial& material,
                    const std::array<double, 3>& color, double alpha, int alphaMode,
                    bool cullBackFaces = false,
                    std::optional<HumanGaitParameters> humanGait = std::nullopt,
                    double visibilityFade = 1.0, const HumanSkinDraw* humanSkin = nullptr,
                    MotionObjectKey motionObject = {});
void drawIndexedModelBatch(GLuint buffer, GLuint indexBuffer, std::size_t indexCount,
                           const ModelMaterial& material, const std::array<double, 3>& color,
                           double alpha, int alphaMode, bool cullBackFaces = false,
                           double visibilityFade = 1.0, MotionObjectKey motionObject = {});
void drawMaterialBatch(GLuint buffer, std::size_t vertexCount,
                       const std::vector<std::array<float, 4>>& colors,
                       const std::vector<GLuint>& textures, const std::vector<bool>& flipTextureY,
                       GLuint indexBuffer = 0, double visibilityFade = 1.0, GLuint textTexture = 0,
                       bool useTextTexture = false, MotionObjectKey motionObject = {});
void drawEnvironmentBatch(GLuint buffer, std::size_t vertexCount, GLuint texture, double alpha);
void drawPrimitives(const std::vector<PrimitiveVertex>& vertices, GLenum primitive,
                    float lineWidth = 1.0f);
bool drawStaticPrimitives(StaticPrimitiveBuffer& cache,
                          const std::vector<PrimitiveVertex>& vertices, GLenum primitive,
                          float lineWidth = 1.0f, std::uint64_t contentRevision = 0,
                          bool useIndirectOcclusionProxy = false);
void releaseStaticPrimitiveBuffer(StaticPrimitiveBuffer& cache);
void drawTextureQuad(GLuint texture, float x, float y, float width, float height,
                     bool flipTextureY = false, int motionDebugMode = 0,
                     GLuint motionValidityTexture = 0, bool fxaaEnabled = false,
                     float inverseWidth = 0.0F, float inverseHeight = 0.0F);

} // namespace openbus::rendering
