#pragma once

#include "render/CameraMath.h"
#include "config/TextureAddressMode.h"
#include "render/ModelVertex.h"
#include "render/HumanSkinTypes.h"
#include "render/MotionData.h"
#include "render/diligent/DiligentMultisampling.h"

#include <Graphics/GraphicsEngine/interface/GraphicsTypes.h>

#include <array>
#include <cstdint>
#include <memory>
#include <span>

namespace Diligent {
struct IDeviceContext;
struct IRenderDevice;
struct ITextureView;
} // namespace Diligent

namespace openbus::rendering {

// Immutable triangle lists in OpenBus model-vertex layout. Indices, when supplied,
// are zero-based uint32 vertex offsets and must form complete triangles. Optional
// human skin weights are indexed by vertex, including for indexed meshes.
class DiligentTexturedMesh {
  public:
    DiligentTexturedMesh(Diligent::IRenderDevice& device, std::span<const ModelVertex> vertices);
    DiligentTexturedMesh(Diligent::IRenderDevice& device, std::span<const ModelVertex> vertices,
                         std::span<const std::uint32_t> indices);
    DiligentTexturedMesh(Diligent::IRenderDevice& device, std::span<const ModelVertex> vertices,
                         std::span<const HumanVertexWeights> weights);
    DiligentTexturedMesh(Diligent::IRenderDevice& device, std::span<const ModelVertex> vertices,
                         std::span<const std::uint32_t> indices,
                         std::span<const HumanVertexWeights> weights);
    ~DiligentTexturedMesh();
    DiligentTexturedMesh(const DiligentTexturedMesh&) = delete;
    DiligentTexturedMesh& operator=(const DiligentTexturedMesh&) = delete;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    friend class DiligentTexturedMeshRenderer;
};

class DiligentTexture2D;
class DiligentTexture2DArray;
class DiligentTexturedMesh;
class DiligentMotionRenderTargets;
class DiligentSpotlightShadowMaps;

inline constexpr std::size_t DILIGENT_MAX_MATERIAL_ARRAY_LAYERS = 8;
inline constexpr std::size_t DILIGENT_MAX_INTERIOR_LIGHTS = 4;

struct DiligentShadowCaster {
    const DiligentTexturedMesh* mesh = nullptr;
    Matrix4 modelViewProjection = {};
    // Optional 2D or 2D-array base texture for cutout shadow testing.
    Diligent::ITextureView* alphaTexture = nullptr;
    std::array<float, 4> textureTransform = {1.0F, 1.0F, 0.0F, 0.0F};
    // Zero disables alpha testing; positive values discard lower sampled alpha.
    float alphaCutoff = 0.0F;
    // Zero-based layer used only when alphaTexture is a 2D array view.
    Diligent::Uint32 alphaArrayLayer = 0;
    // Required for skinned meshes; the caller supplies the pose used for this shadow pass.
    const HumanBonePalette* currentPose = nullptr;
    // Sample a per-vertex array coordinate from ModelVertex::texcoord.z.
    bool useVertexArrayLayer = false;
};

enum class DiligentAlphaMode : std::uint8_t {
    Opaque,
    // Discard alpha values below 0.5 without blending.
    Cutout,
    // Standard source-alpha blending.
    Blend,
    // Discard alpha values below 0.99 without blending.
    NearOpaque,
    // Discard alpha below 0.99, then blend remaining values.
    NearOpaqueBlend,
};

enum class DiligentDepthMode : std::uint8_t {
    Disabled,
    Less,
    LessEqual,
};

struct DiligentMaterialState {
    DiligentAlphaMode alphaMode = DiligentAlphaMode::Opaque;
    // Mirrors production no-Z-write materials, which blend while preserving their alpha cutoff.
    bool blendEnabled = false;
    std::array<float, 4> baseColor = {1.0F, 1.0F, 1.0F, 1.0F};
    std::array<float, 2> textureScale = {1.0F, 1.0F};
    std::array<float, 2> textureOffset = {0.0F, 0.0F};
    bool textureAlphaOnly = false;
    std::array<float, 3> specularColor = {};
    // Per-channel mask that restores the unlit base color after directional shading.
    std::array<float, 3> emissionColor = {};
    float specularPower = 16.0F;
    // Scales tangent-space normal X/Y; zero keeps the geometric normal, one is full strength.
    float normalMapStrength = 1.0F;
    // Optional non-owning view. White texels restore the unlit base color; black preserve shading.
    Diligent::ITextureView* lightMap = nullptr;
    float lightMapStrength = 1.0F;
    // Optional non-owning view; non-black texels overlay self-illumination at this [0, 1] strength.
    Diligent::ITextureView* nightMap = nullptr;
    float nightMapStrength = 1.0F;
    // Optional non-owning view; when present, its alpha replaces base-texture alpha.
    Diligent::ITextureView* transMap = nullptr;
    std::array<float, 2> transMapScale = {1.0F, 1.0F};
    bool flipTransmapY = false;
    // Optional non-owning view; multiplies base color using this independent UV scale.
    Diligent::ITextureView* detailMap = nullptr;
    std::array<float, 2> detailMapScale = {1.0F, 1.0F};
    // Runtime free/script textures multiply the base texture using its scaled UVs.
    Diligent::ITextureView* freeMap = nullptr;
    // Text-atlas textures multiply the base texture using the untransformed mesh UVs.
    Diligent::ITextureView* textMap = nullptr;
    bool weatherLitDynamicTexture = false;
    float dynamicTextureBrightness = 1.0F;
    // Snow blends the surface toward the same pale color as the production map shader.
    float snowCoverage = 0.0F;
    // Optional view-space sphere map; the caller supplies the current weather brightness.
    Diligent::ITextureView* environmentMap = nullptr;
    float environmentStrength = 0.0F;
    float environmentBrightness = 1.0F;
    // Preserve uncolored ambient reflectance for authored scenery materials.
    bool whiteAmbient = false;
    // Per-material vehicle interior lights in view space, following the production model path.
    bool useInteriorLights = false;
    std::uint32_t interiorLightCount = 0;
    std::array<std::array<float, 3>, DILIGENT_MAX_INTERIOR_LIGHTS> interiorLightPositionsView = {};
    std::array<std::array<float, 3>, DILIGENT_MAX_INTERIOR_LIGHTS> interiorLightColors = {};
    std::array<float, DILIGENT_MAX_INTERIOR_LIGHTS> interiorLightStrengths = {};
    // Cull back faces unless the caller has disabled culling for a mirrored view.
    bool cullBackFaces = false;
    // Scenery model vertices carry source colors that the production model shader ignores.
    bool useVertexColor = true;
    bool flipTextureY = false;
    // Per-layer reflection views override selected layers of material texture arrays.
    std::array<Diligent::ITextureView*, DILIGENT_MAX_MATERIAL_ARRAY_LAYERS>
        arrayReflectionTextures = {};
    // Scripted and text textures may require nearest-neighbor sampling.
    bool nearestBaseTexture = false;
    bool nearestFreeTexture = false;
    bool nearestTextTexture = false;
    // Requested base-texture anisotropy: 1 disables it; supported values are 2, 4, 8, and 16.
    Diligent::Uint32 anisotropy = 1;
    TextureAddressMode addressU = TextureAddressMode::Repeat;
    TextureAddressMode addressV = TextureAddressMode::Repeat;
    float visibilityFade = 1.0F;
    DiligentDepthMode depthMode = DiligentDepthMode::Less;
    // Matches OpenBus's model draw default; disable for sorted transparent surfaces as needed.
    bool depthWrite = true;
    bool colorWriteEnabled = true;
};

[[nodiscard]] inline bool diligentMaterialBlends(const DiligentMaterialState& material) noexcept {
    return material.blendEnabled || material.alphaMode == DiligentAlphaMode::Blend ||
           material.alphaMode == DiligentAlphaMode::NearOpaqueBlend;
}

struct DiligentPointLight {
    // View-space position and range, with non-negative linear color and intensity.
    std::array<float, 3> positionView = {};
    std::array<float, 3> color = {1.0F, 1.0F, 1.0F};
    float radius = 1.0F;
    float intensity = 1.0F;
};

struct DiligentSpotLight {
    // View-space position/direction, non-negative linear color, and cone half-angles in degrees.
    std::array<float, 3> positionView = {};
    std::array<float, 3> directionView = {0.0F, 0.0F, -1.0F};
    std::array<float, 3> color = {1.0F, 1.0F, 1.0F};
    float range = 1.0F;
    float intensity = 1.0F;
    float innerAngleDegrees = 15.0F;
    float outerAngleDegrees = 30.0F;
    // Diligent clip-from-view matrix with depth in [0, 1], and the sampled depth-array layer.
    Matrix4 shadowClipFromView = {};
    Diligent::Uint32 shadowMapIndex = 0;
    float shadowBias = 0.001F;
    bool shadowValid = false;
};

struct DiligentWorldSpotLight {
    std::array<double, 3> position = {};
    std::array<double, 3> direction = {0.0, 0.0, -1.0};
    std::array<double, 3> color = {1.0, 1.0, 1.0};
    double range = 1.0;
    double intensity = 1.0;
    double innerAngleDegrees = 15.0;
    double outerAngleDegrees = 30.0;
};

struct DiligentWeatherFog {
    std::array<float, 3> color = {0.45F, 0.65F, 0.88F};
    float density = 0.0F;
    float layerHeight = -1.0F;
    float cameraHeight = 0.0F;
    std::array<float, 3> worldUpView = {0.0F, 0.0F, 1.0F};
};

// View-space inputs for lighting, normal maps, and environment maps.
// objectToView must match the draw's transform; normalTransform is its
// inverse-transpose linear transform so normals handle non-uniform scale.
struct DiligentLightingParameters {
    bool directionalEnabled = false;
    Matrix4 objectToView = {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                            0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    Matrix4 normalTransform = {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                               0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    std::array<float, 3> lightDirectionView = {0.0F, 0.0F, 1.0F};
    std::array<float, 3> diffuseColor = {1.0F, 1.0F, 1.0F};
    float ambientIntensity = 0.25F;
    float diffuseIntensity = 0.75F;
    float weatherBrightness = 1.0F;
    DiligentWeatherFog weatherFog;
};

// Previous-frame data for a motion-vector draw. The viewport is in target pixels.
// Motion is written in pixel units as previous minus current position, with +Y up.
struct DiligentMotionParameters {
    Matrix4 previousModelViewProjection = {};
    int viewportWidth = 0;
    int viewportHeight = 0;
    bool historyValid = false;

    [[nodiscard]] static DiligentMotionParameters fromHistory(const MotionDataFrame& frame,
                                                              const MotionTransform& transform);
};

struct DiligentSkinDraw {
    const HumanBonePalette& current;
    // Omit when the previous rendered pose is unavailable; motion is then invalid.
    // The caller owns pose history and supplies poses from the corresponding rendered frames.
    const HumanBonePalette* previous = nullptr;
};

// Base-texture rendering with optional directional and per-view local lighting,
// linear filtering by default (or nearest filtering for the base texture),
// and independently configurable base-texture U/V addressing.
// UVs are optionally flipped and offset for all maps. The base, detail, and
// transmap UVs then use their respective independent scales.
// The context and single-sample 2D shader-resource view must belong to the
// renderer's device. The caller binds compatible targets and owns
// frame submission and synchronization. Drawing changes context state.
// Optional normal/light/night/trans/detail/environment maps are non-owning, single-sample 2D
// shader views that must remain valid through the draw. Sample auxiliary map data in linear space.
class DiligentTexturedMeshRenderer {
  public:
    DiligentTexturedMeshRenderer(Diligent::IRenderDevice& device,
                                 Diligent::TEXTURE_FORMAT colorFormat,
                                 Diligent::TEXTURE_FORMAT depthFormat);
    // Color-only output; multisampled targets must match the configured sample count.
    DiligentTexturedMeshRenderer(Diligent::IRenderDevice& device,
                                 Diligent::TEXTURE_FORMAT colorFormat,
                                 Diligent::TEXTURE_FORMAT depthFormat,
                                 DiligentMultisampling multisampling);
    // Adds a positive axial view-space depth output after the color target.
    DiligentTexturedMeshRenderer(Diligent::IRenderDevice& device,
                                 Diligent::TEXTURE_FORMAT colorFormat,
                                 Diligent::TEXTURE_FORMAT depthFormat,
                                 Diligent::TEXTURE_FORMAT linearDepthFormat);
    DiligentTexturedMeshRenderer(Diligent::IRenderDevice& device,
                                 Diligent::TEXTURE_FORMAT colorFormat,
                                 Diligent::TEXTURE_FORMAT depthFormat,
                                 Diligent::TEXTURE_FORMAT motionVectorFormat,
                                 Diligent::TEXTURE_FORMAT motionValidityFormat);
    DiligentTexturedMeshRenderer(Diligent::IRenderDevice& device,
                                 Diligent::TEXTURE_FORMAT colorFormat,
                                 Diligent::TEXTURE_FORMAT depthFormat,
                                 Diligent::TEXTURE_FORMAT motionVectorFormat,
                                 Diligent::TEXTURE_FORMAT motionValidityFormat,
                                 DiligentMultisampling multisampling);
    // Motion targets are followed by the optional positive axial view-space depth output.
    DiligentTexturedMeshRenderer(Diligent::IRenderDevice& device,
                                 Diligent::TEXTURE_FORMAT colorFormat,
                                 Diligent::TEXTURE_FORMAT depthFormat,
                                 Diligent::TEXTURE_FORMAT motionVectorFormat,
                                 Diligent::TEXTURE_FORMAT motionValidityFormat,
                                 Diligent::TEXTURE_FORMAT linearDepthFormat);
    DiligentTexturedMeshRenderer(Diligent::IRenderDevice& device,
                                 Diligent::TEXTURE_FORMAT colorFormat,
                                 Diligent::TEXTURE_FORMAT depthFormat,
                                 Diligent::TEXTURE_FORMAT motionVectorFormat,
                                 Diligent::TEXTURE_FORMAT motionValidityFormat,
                                 Diligent::TEXTURE_FORMAT linearDepthFormat,
                                 DiligentMultisampling multisampling);
    ~DiligentTexturedMeshRenderer();
    DiligentTexturedMeshRenderer(const DiligentTexturedMeshRenderer&) = delete;
    DiligentTexturedMeshRenderer& operator=(const DiligentTexturedMeshRenderer&) = delete;

    // Positions are in view space; update once for each rendered view.
    void setPointLights(Diligent::IDeviceContext& context,
                        std::span<const DiligentPointLight> pointLights);
    // Directions are beam-forward in view space; inner/outer values are cone half-angles.
    void setSpotLights(Diligent::IDeviceContext& context,
                       std::span<const DiligentSpotLight> spotLights);
    void setSpotlightShadowMapView(Diligent::ITextureView* shadowMaps);
    // Draws caller-filtered opaque/cutout, undeformed casters into one light layer and restores
    // the scene targets and viewport before returning.
    std::size_t drawOpaqueShadowLayer(Diligent::IDeviceContext& context,
                                      DiligentMotionRenderTargets& sceneTargets,
                                      DiligentSpotlightShadowMaps& shadowMaps,
                                      Diligent::Uint32 lightIndex,
                                      std::span<const DiligentShadowCaster> casters);
    [[nodiscard]] Diligent::Uint32 maximumAnisotropy() const noexcept;
    void setDefaultAnisotropy(Diligent::Uint32 anisotropy);

    void draw(Diligent::IDeviceContext& context, const DiligentTexturedMesh& mesh,
              Diligent::ITextureView& texture, const Matrix4& modelViewProjection,
              DiligentMaterialState material = {}, const DiligentLightingParameters& lighting = {},
              Diligent::ITextureView* normalMap = nullptr);
    void drawMotion(Diligent::IDeviceContext& context, const DiligentTexturedMesh& mesh,
                    Diligent::ITextureView& texture, const Matrix4& modelViewProjection,
                    const DiligentMotionParameters& motion, DiligentMaterialState material = {},
                    const DiligentLightingParameters& lighting = {},
                    Diligent::ITextureView* normalMap = nullptr);
    void drawSkinned(Diligent::IDeviceContext& context, const DiligentTexturedMesh& mesh,
                     Diligent::ITextureView& texture, const Matrix4& modelViewProjection,
                     const DiligentSkinDraw& skin, DiligentMaterialState material = {},
                     const DiligentLightingParameters& lighting = {},
                     Diligent::ITextureView* normalMap = nullptr);
    void drawSkinnedMotion(Diligent::IDeviceContext& context, const DiligentTexturedMesh& mesh,
                           Diligent::ITextureView& texture, const Matrix4& modelViewProjection,
                           const DiligentMotionParameters& motion, const DiligentSkinDraw& skin,
                           DiligentMaterialState material = {},
                           const DiligentLightingParameters& lighting = {},
                           Diligent::ITextureView* normalMap = nullptr);
    void draw(Diligent::IDeviceContext& context, const DiligentTexturedMesh& mesh,
              const DiligentTexture2D& texture, const Matrix4& modelViewProjection,
              DiligentMaterialState material = {}, const DiligentLightingParameters& lighting = {},
              Diligent::ITextureView* normalMap = nullptr);
    void drawMotion(Diligent::IDeviceContext& context, const DiligentTexturedMesh& mesh,
                    const DiligentTexture2D& texture, const Matrix4& modelViewProjection,
                    const DiligentMotionParameters& motion, DiligentMaterialState material = {},
                    const DiligentLightingParameters& lighting = {},
                    Diligent::ITextureView* normalMap = nullptr);
    void draw(Diligent::IDeviceContext& context, const DiligentTexturedMesh& mesh,
              const DiligentTexture2DArray& texture, const Matrix4& modelViewProjection,
              DiligentMaterialState material = {}, const DiligentLightingParameters& lighting = {},
              Diligent::ITextureView* normalMap = nullptr);
    void drawMotion(Diligent::IDeviceContext& context, const DiligentTexturedMesh& mesh,
                    const DiligentTexture2DArray& texture, const Matrix4& modelViewProjection,
                    const DiligentMotionParameters& motion, DiligentMaterialState material = {},
                    const DiligentLightingParameters& lighting = {},
                    Diligent::ITextureView* normalMap = nullptr);

  private:
    void drawImpl(Diligent::IDeviceContext& context, const DiligentTexturedMesh& mesh,
                  Diligent::ITextureView& texture, bool textureArray,
                  const Matrix4& modelViewProjection, DiligentMaterialState material,
                  const DiligentLightingParameters& lighting,
                  const DiligentMotionParameters* motion, Diligent::ITextureView* normalMap,
                  const DiligentSkinDraw* skin = nullptr);
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace openbus::rendering
