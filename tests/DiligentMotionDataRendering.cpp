#include "DiligentMotionDataRendering.h"

#include "assets/TextureLoader.h"
#include "render/HumanMeshGeometry.h"
#include "render/diligent/DiligentMotionRenderTargets.h"
#include "render/diligent/DiligentTexturedMesh.h"
#include "render/diligent/DiligentTexture2D.h"

#include <Common/interface/RefCntAutoPtr.hpp>
#include <Graphics/GraphicsEngine/interface/DeviceContext.h>
#include <Graphics/GraphicsEngine/interface/RenderDevice.h>
#include <Graphics/GraphicsEngine/interface/Texture.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

namespace {

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

Diligent::RefCntAutoPtr<Diligent::ITexture> createTexture(Diligent::IRenderDevice& device,
                                                          const char* name, Diligent::Uint32 size,
                                                          Diligent::TEXTURE_FORMAT format,
                                                          Diligent::BIND_FLAGS bindFlags) {
    using namespace Diligent;
    TextureDesc desc;
    desc.Name = name;
    desc.Type = RESOURCE_DIM_TEX_2D;
    desc.Width = size;
    desc.Height = size;
    desc.MipLevels = 1;
    desc.Format = format;
    desc.BindFlags = bindFlags;
    RefCntAutoPtr<ITexture> texture;
    device.CreateTexture(desc, nullptr, &texture);
    require(static_cast<bool>(texture), "Failed to create Diligent motion-data texture");
    return texture;
}

Diligent::RefCntAutoPtr<Diligent::ITexture> createStagingTexture(Diligent::IRenderDevice& device,
                                                                 const char* name,
                                                                 Diligent::Uint32 size,
                                                                 Diligent::TEXTURE_FORMAT format) {
    using namespace Diligent;
    TextureDesc desc;
    desc.Name = name;
    desc.Type = RESOURCE_DIM_TEX_2D;
    desc.Width = size;
    desc.Height = size;
    desc.MipLevels = 1;
    desc.Format = format;
    desc.Usage = USAGE_STAGING;
    desc.BindFlags = BIND_NONE;
    desc.CPUAccessFlags = CPU_ACCESS_READ;
    RefCntAutoPtr<ITexture> texture;
    device.CreateTexture(desc, nullptr, &texture);
    require(static_cast<bool>(texture), "Failed to create Diligent motion-data staging texture");
    return texture;
}

} // namespace

void verifyDiligentMotionDataRendering(Diligent::IRenderDevice& device,
                                       Diligent::IDeviceContext& context) {
    using namespace Diligent;
    using namespace openbus::rendering;
    constexpr Uint32 size = 64;

    auto color = createTexture(device, "Motion test color", size, TEX_FORMAT_RGBA8_UNORM,
                               BIND_RENDER_TARGET);
    auto vectors = createTexture(device, "Motion test vectors", size, TEX_FORMAT_RG32_FLOAT,
                                 BIND_RENDER_TARGET);
    auto validity = createTexture(device, "Motion test validity", size, TEX_FORMAT_R8_UNORM,
                                  BIND_RENDER_TARGET);
    auto depth =
        createTexture(device, "Motion test depth", size, TEX_FORMAT_D32_FLOAT, BIND_DEPTH_STENCIL);
    auto vectorStaging =
        createStagingTexture(device, "Motion vector readback", size, TEX_FORMAT_RG32_FLOAT);
    auto validityStaging =
        createStagingTexture(device, "Motion validity readback", size, TEX_FORMAT_R8_UNORM);

    auto* colorView = color->GetDefaultView(TEXTURE_VIEW_RENDER_TARGET);
    auto* vectorView = vectors->GetDefaultView(TEXTURE_VIEW_RENDER_TARGET);
    auto* validityView = validity->GetDefaultView(TEXTURE_VIEW_RENDER_TARGET);
    auto* depthView = depth->GetDefaultView(TEXTURE_VIEW_DEPTH_STENCIL);
    require(colorView != nullptr && vectorView != nullptr && validityView != nullptr &&
                depthView != nullptr,
            "Diligent motion-data target view is missing");

    constexpr std::array<ModelVertex, 6> quad = {{
        {-0.8F, -0.8F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 1.0F},
        {0.8F, -0.8F, 0.0F, 1.0F, 1.0F, 0.0F, 0.0F, 0.0F, 1.0F},
        {0.8F, 0.8F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F},
        {-0.8F, -0.8F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 1.0F},
        {0.8F, 0.8F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F},
        {-0.8F, 0.8F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F},
    }};
    DiligentTexturedMesh mesh(device, quad);
    DiligentTexturedMeshRenderer renderer(device, TEX_FORMAT_RGBA8_UNORM, TEX_FORMAT_D32_FLOAT,
                                          TEX_FORMAT_RG32_FLOAT, TEX_FORMAT_R8_UNORM);
    const Image whiteImage{1, 1, {255, 255, 255, 255}};
    DiligentTexture2D whiteTexture(device, whiteImage);

    Matrix4 identity = {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                        0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    Matrix4 current = identity;
    current[12] = 0.25;
    MotionHistory history;
    history.beginFrame(1, 1.0, static_cast<int>(size), static_cast<int>(size), identity, identity);
    static_cast<void>(history.previousModelView(identity, MotionObjectKey{7, 0}));
    history.completeFrame();
    history.beginFrame(2, 1.016, static_cast<int>(size), static_cast<int>(size), identity,
                       identity);
    const MotionTransform previous = history.previousModelView(current, MotionObjectKey{7, 0});
    DiligentMotionParameters motion =
        DiligentMotionParameters::fromHistory(history.frame(), previous);
    history.completeFrame();

    const auto draw = [&](bool historyValid) {
        motion.historyValid = historyValid;
        ITextureView* targets[] = {colorView, vectorView, validityView};
        context.SetRenderTargets(3, targets, depthView, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        const Viewport viewport{0.0F, 0.0F, static_cast<float>(size), static_cast<float>(size),
                                0.0F, 1.0F};
        context.SetViewports(1, &viewport, size, size);
        constexpr std::array<float, 4> clear = {};
        for (ITextureView* target : targets) {
            context.ClearRenderTarget(target, clear.data(),
                                      RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        }
        context.ClearDepthStencil(depthView, CLEAR_DEPTH_FLAG, 1.0F, 0,
                                  RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        renderer.drawMotion(context, mesh, whiteTexture, current, motion);
        context.SetRenderTargets(0, nullptr, nullptr, RESOURCE_STATE_TRANSITION_MODE_NONE);
    };

    const auto copyToStaging = [&](ITexture* source, ITexture* staging) {
        CopyTextureAttribs copy;
        copy.pSrcTexture = source;
        copy.SrcTextureTransitionMode = RESOURCE_STATE_TRANSITION_MODE_TRANSITION;
        copy.pDstTexture = staging;
        copy.DstTextureTransitionMode = RESOURCE_STATE_TRANSITION_MODE_TRANSITION;
        context.CopyTexture(copy);
    };
    const auto readPixel = [&](float expectedX, float expectedY, bool expectValid) {
        copyToStaging(vectors, vectorStaging);
        copyToStaging(validity, validityStaging);
        context.WaitForIdle();
        MappedTextureSubresource vectorMap;
        MappedTextureSubresource validityMap;
        context.MapTextureSubresource(vectorStaging, 0, 0, MAP_READ, MAP_FLAG_DO_NOT_WAIT, nullptr,
                                      vectorMap);
        context.MapTextureSubresource(validityStaging, 0, 0, MAP_READ, MAP_FLAG_DO_NOT_WAIT,
                                      nullptr, validityMap);
        require(vectorMap.pData != nullptr && validityMap.pData != nullptr,
                "Failed to map Diligent motion-data readback");
        const auto* vectorRow =
            static_cast<const std::byte*>(vectorMap.pData) + 32 * vectorMap.Stride;
        std::array<float, 2> vectorPixel{};
        std::memcpy(vectorPixel.data(), vectorRow + 32 * 2 * sizeof(float), sizeof(vectorPixel));
        const auto* validityRow =
            static_cast<const std::uint8_t*>(validityMap.pData) + 32 * validityMap.Stride;
        const std::uint8_t validityValue = validityRow[32];
        const bool correctMotion =
            expectValid ? std::abs(vectorPixel[0] - expectedX) < 0.25F &&
                              std::abs(vectorPixel[1] - expectedY) < 0.25F && validityValue >= 220
                        : std::abs(vectorPixel[0]) < 0.01F && std::abs(vectorPixel[1]) < 0.01F &&
                              validityValue <= 10;
        context.UnmapTextureSubresource(validityStaging, 0, 0);
        context.UnmapTextureSubresource(vectorStaging, 0, 0);
        if (!correctMotion) {
            throw std::runtime_error(
                "Unexpected Diligent motion readback: x=" + std::to_string(vectorPixel[0]) +
                ", y=" + std::to_string(vectorPixel[1]) +
                ", validity=" + std::to_string(validityValue));
        }
        return correctMotion;
    };

    draw(true);
    require(readPixel(-8.0F, 0.0F, true),
            "Diligent motion vectors did not report previous-minus-current pixel displacement");

    current[13] = 0.125;
    draw(true);
    require(readPixel(-8.0F, -4.0F, true),
            "Diligent motion vectors did not preserve the documented positive-up Y convention");

    draw(false);
    require(readPixel(0.0F, 0.0F, false),
            "Diligent motion validity did not suppress vectors when history was invalid");

    motion.previousModelViewProjection = identity;
    motion.previousModelViewProjection[15] = -1.0;
    draw(true);
    require(readPixel(0.0F, 0.0F, false),
            "Diligent motion validity accepted a previous position behind the camera");

    motion.viewportWidth = 0;
    bool invalidViewportRejected = false;
    try {
        renderer.drawMotion(context, mesh, whiteTexture, current, motion);
    } catch (const std::invalid_argument&) {
        invalidViewportRejected = true;
    }
    require(invalidViewportRejected, "Diligent motion draw accepted a zero-width viewport");

    std::array<HumanVertexWeights, quad.size()> weights = {};
    for (auto& vertexWeights : weights) {
        vertexWeights[0] = 1.0F;
        vertexWeights[1] = 3.0F;
    }
    DiligentTexturedMesh skinnedMesh(device, quad, weights);
    const std::array<std::uint32_t, 6> indices = {0, 1, 2, 3, 4, 5};
    DiligentTexturedMesh indexedSkinnedMesh(device, quad, indices, weights);
    HumanBonePalette currentBones;
    HumanBonePalette previousBones;
    currentBones.fill(identity);
    previousBones.fill(identity);
    currentBones[0][12] = 0.5;
    motion.previousModelViewProjection = identity;
    motion.viewportWidth = size;
    motion.viewportHeight = size;
    motion.historyValid = true;
    const auto drawSkin = [&](const DiligentTexturedMesh& selectedMesh,
                              const HumanBonePalette* previousPalette) {
        ITextureView* targets[] = {colorView, vectorView, validityView};
        context.SetRenderTargets(3, targets, depthView, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        const Viewport viewport{0.0F, 0.0F, static_cast<float>(size), static_cast<float>(size),
                                0.0F, 1.0F};
        context.SetViewports(1, &viewport, size, size);
        constexpr std::array<float, 4> clear = {};
        for (ITextureView* target : targets) {
            context.ClearRenderTarget(target, clear.data(),
                                      RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        }
        context.ClearDepthStencil(depthView, CLEAR_DEPTH_FLAG, 1.0F, 0,
                                  RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        renderer.drawSkinnedMotion(context, selectedMesh, whiteTexture.shaderResourceView(),
                                   identity, motion,
                                   DiligentSkinDraw{currentBones, previousPalette});
        context.SetRenderTargets(0, nullptr, nullptr, RESOURCE_STATE_TRANSITION_MODE_NONE);
    };
    drawSkin(skinnedMesh, &previousBones);
    require(readPixel(-4.0F, 0.0F, true),
            "Diligent skinned motion did not use both weighted rendered poses");
    drawSkin(indexedSkinnedMesh, &previousBones);
    require(readPixel(-4.0F, 0.0F, true),
            "Diligent indexed skin weights did not follow vertex indices");
    ParsedObj parsedHuman;
    for (const ModelVertex& vertex : quad) {
        parsedHuman.positions.push_back({vertex.x, vertex.y, vertex.z});
    }
    parsedHuman.triangles = {{{{{1, 0, 0}, {2, 0, 0}, {3, 0, 0}}}, "body"},
                             {{{{4, 0, 0}, {5, 0, 0}, {6, 0, 0}}}, "body"}};
    parsedHuman.materials["body"].materialIndex = 4;
    ObjBone left{"left", {}};
    ObjBone right{"right", {}};
    for (std::uint32_t index = 0; index < quad.size(); ++index) {
        left.weights.push_back({index, 0.25F});
        right.weights.push_back({index, 0.75F});
    }
    parsedHuman.bones = {left, right};
    const auto parsedWeights = resolveHumanSkinWeights(parsedHuman, {{"left", -2}, {"right", -3}});
    const auto humanGeometry = prepareHumanMeshGeometry(parsedHuman, parsedWeights, HumanJoints{},
                                                        false, 1.0, 1.0, {0.0, 0.0}, 0.0);
    DiligentTexturedMesh parsedHumanMesh(device, humanGeometry.verticesByMaterial.at(4),
                                         humanGeometry.weightsByMaterial.at(4));
    drawSkin(parsedHumanMesh, &previousBones);
    require(readPixel(-4.0F, 0.0F, true),
            "Diligent did not render production-prepared human geometry and resolved weights");
    drawSkin(skinnedMesh, nullptr);
    require(readPixel(0.0F, 0.0F, false),
            "Diligent skinned motion accepted a missing previous palette");

    std::array<HumanVertexWeights, quad.size()> zeroWeights = {};
    DiligentTexturedMesh unweightedMesh(device, quad, zeroWeights);
    drawSkin(unweightedMesh, &previousBones);
    require(readPixel(0.0F, 0.0F, true),
            "Diligent unweighted skin vertices did not retain their original positions");

    bool missingWeightsRejected = false;
    try {
        renderer.drawSkinned(context, mesh, whiteTexture.shaderResourceView(), identity,
                             DiligentSkinDraw{currentBones});
    } catch (const std::invalid_argument&) {
        missingWeightsRejected = true;
    }
    require(missingWeightsRejected, "Diligent skinned draw accepted a mesh without weights");
    bool mismatchedWeightsRejected = false;
    try {
        DiligentTexturedMesh invalid(device, quad, std::span{weights}.first(1));
    } catch (const std::invalid_argument&) {
        mismatchedWeightsRejected = true;
    }
    require(mismatchedWeightsRejected, "Diligent skin mesh accepted a mismatched weight count");
    auto invalidWeights = weights;
    invalidWeights[0][0] = -1.0F;
    bool negativeWeightsRejected = false;
    try {
        DiligentTexturedMesh invalid(device, quad, invalidWeights);
    } catch (const std::invalid_argument&) {
        negativeWeightsRejected = true;
    }
    require(negativeWeightsRejected, "Diligent skin mesh accepted a negative weight");

    auto invalidPalette = currentBones;
    invalidPalette[0][0] = std::numeric_limits<double>::quiet_NaN();
    bool invalidPaletteRejected = false;
    try {
        renderer.drawSkinned(context, skinnedMesh, whiteTexture.shaderResourceView(), identity,
                             DiligentSkinDraw{invalidPalette});
    } catch (const std::invalid_argument&) {
        invalidPaletteRejected = true;
    }
    require(invalidPaletteRejected, "Diligent skinned draw accepted a non-finite palette");

    auto normalVertices = quad;
    for (auto& vertex : normalVertices) {
        vertex.nx = 1.0F;
        vertex.ny = 0.0F;
        vertex.nz = 0.0F;
    }
    auto normalWeights = zeroWeights;
    for (auto& vertexWeights : normalWeights) {
        vertexWeights[0] = 1.0F;
    }
    DiligentTexturedMesh normalMesh(device, normalVertices, normalWeights);
    auto rotatedBones = previousBones;
    rotatedBones[0][0] = 0.0;
    rotatedBones[0][1] = 1.0;
    rotatedBones[0][4] = -1.0;
    rotatedBones[0][5] = 0.0;
    DiligentLightingParameters lighting;
    lighting.directionalEnabled = true;
    lighting.lightDirectionView = {0.0F, 1.0F, 0.0F};
    lighting.ambientIntensity = 0.0F;
    lighting.diffuseIntensity = 1.0F;
    context.SetRenderTargets(1, &colorView, depthView, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    constexpr std::array<float, 4> clearColor = {};
    context.ClearRenderTarget(colorView, clearColor.data(),
                              RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearDepthStencil(depthView, CLEAR_DEPTH_FLAG, 1.0F, 0,
                              RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    renderer.drawSkinned(context, normalMesh, whiteTexture.shaderResourceView(), identity,
                         DiligentSkinDraw{rotatedBones}, {}, lighting);
    context.SetRenderTargets(0, nullptr, nullptr, RESOURCE_STATE_TRANSITION_MODE_NONE);
    auto colorStaging =
        createStagingTexture(device, "Skinned normal readback", size, TEX_FORMAT_RGBA8_UNORM);
    copyToStaging(color, colorStaging);
    context.WaitForIdle();
    MappedTextureSubresource colorMap;
    context.MapTextureSubresource(colorStaging, 0, 0, MAP_READ, MAP_FLAG_DO_NOT_WAIT, nullptr,
                                  colorMap);
    require(colorMap.pData != nullptr, "Failed to map Diligent skinned normal readback");
    const auto* centerColor =
        static_cast<const std::uint8_t*>(colorMap.pData) + 32 * colorMap.Stride + 32 * 4;
    const bool rotatedNormalCorrect =
        centerColor[0] > 220 && centerColor[1] > 220 && centerColor[2] > 220;
    context.UnmapTextureSubresource(colorStaging, 0, 0);
    require(rotatedNormalCorrect, "Diligent skinning did not transform the surface normal");

    bool invalidMotionSamplesRejected = false;
    try {
        DiligentMotionRenderTargets invalid(device, size, size, false, {3});
    } catch (const std::invalid_argument&) {
        invalidMotionSamplesRejected = true;
    }
    require(invalidMotionSamplesRejected, "Motion targets accepted an invalid sample count");
    const auto supported = device.GetTextureFormatInfoExt(TEX_FORMAT_RGBA8_UNORM).SampleCounts &
                           device.GetTextureFormatInfoExt(TEX_FORMAT_RG32_FLOAT).SampleCounts &
                           device.GetTextureFormatInfoExt(TEX_FORMAT_R8_UNORM).SampleCounts &
                           device.GetTextureFormatInfoExt(TEX_FORMAT_D32_FLOAT).SampleCounts;
    const auto linearSupported = device.GetTextureFormatInfoExt(TEX_FORMAT_R32_FLOAT).SampleCounts;
    const std::array<ModelVertex, 3> triangle = {{
        {-0.42F, -0.5F, 0, 0, 0, 0, 0, 0, 1},
        {0.42F, -0.5F, 0, 0, 0, 0, 0, 0, 1},
        {0, 0.53F, 0, 0, 0, 0, 0, 0, 1},
    }};
    DiligentTexturedMesh triangleMesh(device, triangle);
    auto depthStaging =
        createStagingTexture(device, "MSAA depth readback", size, TEX_FORMAT_D32_FLOAT);
    auto linearStaging =
        createStagingTexture(device, "MSAA linear depth readback", size, TEX_FORMAT_R32_FLOAT);
    int configurations = 0;
    for (const Uint32 samples : {2U, 4U}) {
        for (const bool withLinear : {false, true}) {
            if (!(supported & samples) || (withLinear && !(linearSupported & samples))) {
                continue;
            }
            ++configurations;
            DiligentMotionRenderTargets targets(device, size, size, withLinear, {samples});
            bool invalidResizeRejected = false;
            try {
                targets.resize(context, 0, size);
            } catch (const std::invalid_argument&) {
                invalidResizeRejected = true;
            }
            require(invalidResizeRejected && targets.width() == size && targets.height() == size,
                    "Invalid MSAA motion resize changed the attachments");
            std::unique_ptr<DiligentTexturedMeshRenderer> msaaRenderer;
            if (withLinear) {
                msaaRenderer = std::make_unique<DiligentTexturedMeshRenderer>(
                    device, TEX_FORMAT_RGBA8_UNORM, TEX_FORMAT_D32_FLOAT, TEX_FORMAT_RG32_FLOAT,
                    TEX_FORMAT_R8_UNORM, TEX_FORMAT_R32_FLOAT, DiligentMultisampling{samples});
            } else {
                msaaRenderer = std::make_unique<DiligentTexturedMeshRenderer>(
                    device, TEX_FORMAT_RGBA8_UNORM, TEX_FORMAT_D32_FLOAT, TEX_FORMAT_RG32_FLOAT,
                    TEX_FORMAT_R8_UNORM, DiligentMultisampling{samples});
            }
            require(targets.samples() == samples, "Motion target sample count changed");
            targets.clear(context, {0, 0, 0, 1});
            const Viewport viewport{0, 0, static_cast<float>(size), static_cast<float>(size), 0, 1};
            context.SetViewports(1, &viewport, size, size);
            Matrix4 farTransform = identity;
            farTransform[14] = 0.5;
            DiligentMotionParameters farMotion{farTransform, size, size, false};
            DiligentMaterialState farMaterial;
            farMaterial.baseColor = {0, 0, 1, 1};
            DiligentLightingParameters farLighting;
            farLighting.objectToView[14] = -10;
            msaaRenderer->drawMotion(context, mesh, whiteTexture, farTransform, farMotion,
                                     farMaterial, farLighting);
            Matrix4 nearTransform = identity;
            nearTransform[14] = -0.5;
            auto previousNear = nearTransform;
            previousNear[12] = 0.125;
            DiligentMotionParameters nearMotion{previousNear, size, size, true};
            DiligentMaterialState nearMaterial;
            nearMaterial.baseColor = {1, 0, 0, 1};
            DiligentLightingParameters nearLighting;
            nearLighting.objectToView[14] = -5;
            msaaRenderer->drawMotion(context, triangleMesh, whiteTexture, nearTransform, nearMotion,
                                     nearMaterial, nearLighting);
            targets.resolve(context);
            context.SetRenderTargets(0, nullptr, nullptr, RESOURCE_STATE_TRANSITION_MODE_NONE);
            copyToStaging(targets.colorTexture().GetTexture(), colorStaging);
            copyToStaging(targets.motionVectorTexture().GetTexture(), vectorStaging);
            copyToStaging(targets.motionValidityTexture().GetTexture(), validityStaging);
            copyToStaging(targets.depthTexture().GetTexture(), depthStaging);
            if (withLinear) {
                copyToStaging(targets.linearDepthTexture()->GetTexture(), linearStaging);
            }
            context.WaitForIdle();
            MappedTextureSubresource colorRead, motionRead, validityRead, depthRead, linearRead;
            context.MapTextureSubresource(colorStaging, 0, 0, MAP_READ, MAP_FLAG_DO_NOT_WAIT,
                                          nullptr, colorRead);
            context.MapTextureSubresource(vectorStaging, 0, 0, MAP_READ, MAP_FLAG_DO_NOT_WAIT,
                                          nullptr, motionRead);
            context.MapTextureSubresource(validityStaging, 0, 0, MAP_READ, MAP_FLAG_DO_NOT_WAIT,
                                          nullptr, validityRead);
            context.MapTextureSubresource(depthStaging, 0, 0, MAP_READ, MAP_FLAG_DO_NOT_WAIT,
                                          nullptr, depthRead);
            if (withLinear) {
                context.MapTextureSubresource(linearStaging, 0, 0, MAP_READ, MAP_FLAG_DO_NOT_WAIT,
                                              nullptr, linearRead);
            }
            require(colorRead.pData && motionRead.pData && validityRead.pData && depthRead.pData &&
                        (!withLinear || linearRead.pData),
                    "Failed to map MSAA motion resolve output");
            std::size_t edgePixels = 0, nearPixels = 0, farPixels = 0, emptyPixels = 0;
            bool metadataCorrect = true;
            for (Uint32 y = 0; y < size; ++y) {
                const auto* colors =
                    static_cast<const std::uint8_t*>(colorRead.pData) + y * colorRead.Stride;
                const auto* valid =
                    static_cast<const std::uint8_t*>(validityRead.pData) + y * validityRead.Stride;
                const auto* vectors =
                    static_cast<const std::byte*>(motionRead.pData) + y * motionRead.Stride;
                const auto* depths =
                    static_cast<const std::byte*>(depthRead.pData) + y * depthRead.Stride;
                const auto* linear = withLinear ? static_cast<const std::byte*>(linearRead.pData) +
                                                      y * linearRead.Stride
                                                : nullptr;
                for (Uint32 x = 0; x < size; ++x) {
                    const auto redValue = colors[x * 4];
                    const auto blueValue = colors[x * 4 + 2];
                    const bool foreground = redValue > 10;
                    const bool edge = redValue > 10 && redValue < 245 && blueValue > 10;
                    const bool nearPixel = redValue >= 245 && blueValue < 10;
                    const bool farPixel = redValue < 10 && blueValue >= 245;
                    const bool emptyPixel = redValue < 10 && blueValue < 10;
                    if (!edge && !nearPixel && !farPixel && !emptyPixel) {
                        continue;
                    }
                    edgePixels += edge;
                    nearPixels += nearPixel;
                    farPixels += farPixel;
                    emptyPixels += emptyPixel;
                    std::array<float, 2> vector;
                    float depthValue;
                    std::memcpy(vector.data(), vectors + x * 2 * sizeof(float), sizeof(vector));
                    std::memcpy(&depthValue, depths + x * sizeof(float), sizeof(depthValue));
                    float linearValue = foreground ? 5.0F : (farPixel ? 10.0F : 0.0F);
                    if (withLinear) {
                        std::memcpy(&linearValue, linear + x * sizeof(float), sizeof(linearValue));
                    }
                    const float expectedDepth = foreground ? 0.25F : (farPixel ? 0.75F : 1.0F);
                    const float expectedLinear = foreground ? 5.0F : (farPixel ? 10.0F : 0.0F);
                    metadataCorrect = metadataCorrect &&
                                      (foreground ? valid[x] >= 220 : valid[x] <= 10) &&
                                      std::abs(vector[0] - (foreground ? 4.0F : 0.0F)) < 0.1F &&
                                      std::abs(vector[1]) < 0.1F &&
                                      std::abs(depthValue - expectedDepth) < 0.001F &&
                                      std::abs(linearValue - expectedLinear) < 0.001F;
                }
            }
            if (withLinear) {
                context.UnmapTextureSubresource(linearStaging, 0, 0);
            }
            context.UnmapTextureSubresource(depthStaging, 0, 0);
            context.UnmapTextureSubresource(validityStaging, 0, 0);
            context.UnmapTextureSubresource(vectorStaging, 0, 0);
            context.UnmapTextureSubresource(colorStaging, 0, 0);
            require(edgePixels > 0 && nearPixels > 0 && farPixels > 0 && emptyPixels > 0 &&
                        metadataCorrect,
                    "MSAA motion resolve mixed metadata at a partially covered foreground edge");
            targets.resize(context, 13, 13);
            require(targets.width() == 13 && targets.height() == 13 &&
                        targets.colorTexture().GetTexture()->GetDesc().Width == 13,
                    "MSAA motion resize did not replace resolved attachments");
            targets.clear(context, {1, 0, 1, 1});
            targets.resolve(context);
            context.SetRenderTargets(0, nullptr, nullptr, RESOURCE_STATE_TRANSITION_MODE_NONE);
            auto resizedStaging = createStagingTexture(device, "Resized motion color readback", 13,
                                                       TEX_FORMAT_RGBA8_UNORM);
            copyToStaging(targets.colorTexture().GetTexture(), resizedStaging);
            context.WaitForIdle();
            MappedTextureSubresource resizedRead;
            context.MapTextureSubresource(resizedStaging, 0, 0, MAP_READ, MAP_FLAG_DO_NOT_WAIT,
                                          nullptr, resizedRead);
            require(resizedRead.pData != nullptr, "Failed to map resized MSAA resolve");
            const auto* resizedPixel = static_cast<const std::uint8_t*>(resizedRead.pData) +
                                       6 * resizedRead.Stride + 6 * 4;
            const bool resizedCorrect =
                resizedPixel[0] == 255 && resizedPixel[1] == 0 && resizedPixel[2] == 255;
            context.UnmapTextureSubresource(resizedStaging, 0, 0);
            require(resizedCorrect, "MSAA resolve sampled stale attachments after resize");
        }
    }
    std::cout << "Diligent MSAA motion test configurations: " << configurations
              << (configurations == 0 ? " (unsupported; rendering checks skipped)\n" : "\n");
}
