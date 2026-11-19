#include "DiligentColorPresent.h"

#include "assets/TextureLoader.h"
#include "render/diligent/DiligentColorPresent.h"
#include "render/diligent/DiligentMotionRenderTargets.h"
#include "render/diligent/DiligentMultisampling.h"
#include "render/diligent/DiligentPrimitiveRenderer.h"
#include "render/diligent/DiligentTexture2D.h"
#include "render/diligent/DiligentTexturedMesh.h"

#include <Common/interface/RefCntAutoPtr.hpp>
#include <Graphics/GraphicsEngine/interface/DeviceContext.h>
#include <Graphics/GraphicsEngine/interface/RenderDevice.h>
#include <Graphics/GraphicsEngine/interface/Texture.h>
#include <Graphics/GraphicsEngine/interface/TextureView.h>

#include <array>
#include <cstdint>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}
} // namespace

void verifyDiligentColorPresent(Diligent::IRenderDevice& device,
                                Diligent::IDeviceContext& context) {
    using namespace Diligent;
    using namespace openbus::rendering;
    const Image image{2, 2, {255, 0, 0, 255, 0, 255, 0, 255, 0, 0, 255, 255, 255, 255, 255, 255}};
    DiligentTexture2D source(device, image);
    TextureDesc desc;
    desc.Name = "Color present test target";
    desc.Type = RESOURCE_DIM_TEX_2D;
    desc.Width = 8;
    desc.Height = 8;
    desc.MipLevels = 1;
    desc.Format = TEX_FORMAT_RGBA8_UNORM;
    desc.BindFlags = BIND_RENDER_TARGET | BIND_SHADER_RESOURCE;
    RefCntAutoPtr<ITexture> target;
    device.CreateTexture(desc, nullptr, &target);
    require(static_cast<bool>(target), "Failed to create color present test target");
    auto* rtv = target->GetDefaultView(TEXTURE_VIEW_RENDER_TARGET);
    auto* srv = target->GetDefaultView(TEXTURE_VIEW_SHADER_RESOURCE);
    require(rtv != nullptr && srv != nullptr, "Color present test views are missing");

    desc.Name = "Color present readback";
    desc.BindFlags = BIND_NONE;
    desc.Usage = USAGE_STAGING;
    desc.CPUAccessFlags = CPU_ACCESS_READ;
    RefCntAutoPtr<ITexture> staging;
    device.CreateTexture(desc, nullptr, &staging);
    require(static_cast<bool>(staging), "Failed to create color present staging texture");

    DiligentColorPresent presenter(device, TEX_FORMAT_RGBA8_UNORM);
    const auto readback = [&](ITexture* sourceTexture, ITexture* stagingTexture) {
        context.SetRenderTargets(0, nullptr, nullptr, RESOURCE_STATE_TRANSITION_MODE_NONE);
        CopyTextureAttribs copy;
        copy.pSrcTexture = sourceTexture;
        copy.SrcTextureTransitionMode = RESOURCE_STATE_TRANSITION_MODE_TRANSITION;
        copy.pDstTexture = stagingTexture;
        copy.DstTextureTransitionMode = RESOURCE_STATE_TRANSITION_MODE_TRANSITION;
        context.CopyTexture(copy);
        context.WaitForIdle();
        MappedTextureSubresource mapped;
        context.MapTextureSubresource(stagingTexture, 0, 0, MAP_READ, MAP_FLAG_DO_NOT_WAIT, nullptr,
                                      mapped);
        require(mapped.pData != nullptr, "Failed to map color present readback");
        return mapped;
    };
    const auto channel = [](const MappedTextureSubresource& mapped, Uint32 x, Uint32 y,
                            std::size_t component) {
        const auto* pixel =
            static_cast<const std::uint8_t*>(mapped.pData) + y * mapped.Stride + x * 4;
        return pixel[component];
    };
    presenter.draw(context, source.shaderResourceView(), *rtv);
    auto mapped = readback(target, staging);
    const bool orientation = channel(mapped, 1, 1, 0) > 250 && channel(mapped, 1, 1, 1) < 5 &&
                             channel(mapped, 6, 1, 1) > 250 && channel(mapped, 6, 1, 0) < 5 &&
                             channel(mapped, 1, 6, 2) > 250 && channel(mapped, 1, 6, 0) < 5 &&
                             channel(mapped, 6, 6, 0) > 250 && channel(mapped, 6, 6, 1) > 250;
    const bool linearFiltering =
        channel(mapped, 3, 3, 0) >= 130 && channel(mapped, 3, 3, 0) <= 140 &&
        channel(mapped, 3, 3, 1) >= 90 && channel(mapped, 3, 3, 1) <= 100 &&
        channel(mapped, 3, 3, 2) >= 90 && channel(mapped, 3, 3, 2) <= 100 &&
        channel(mapped, 3, 3, 3) == 255;
    context.UnmapTextureSubresource(staging, 0, 0);
    require(orientation, "Color present changed source orientation or edge colors");
    require(linearFiltering, "Color present did not linearly scale scene color");

    DiligentTexture2D replacement(device, Image{1, 1, {17, 47, 97, 191}});
    presenter.draw(context, replacement.shaderResourceView(), *rtv);
    mapped = readback(target, staging);
    const bool rebound = channel(mapped, 4, 4, 0) == 17 && channel(mapped, 4, 4, 1) == 47 &&
                         channel(mapped, 4, 4, 2) == 97 && channel(mapped, 4, 4, 3) == 191;
    context.UnmapTextureSubresource(staging, 0, 0);
    require(rebound, "Color present did not bind the replacement texture");

    Image largeImage{4, 4, {}};
    for (int y = 0; y < 4; ++y) {
        for (int x = 0; x < 4; ++x) {
            largeImage.rgba.push_back(x < 2 ? 255 : 0);
            largeImage.rgba.push_back(y < 2 ? 255 : 0);
            largeImage.rgba.push_back(0);
            largeImage.rgba.push_back(255);
        }
    }
    DiligentTexture2D largeSource(device, largeImage);
    desc.Width = 2;
    desc.Height = 2;
    desc.Name = "Color present downscale target";
    desc.Usage = USAGE_DEFAULT;
    desc.BindFlags = BIND_RENDER_TARGET;
    desc.CPUAccessFlags = CPU_ACCESS_NONE;
    RefCntAutoPtr<ITexture> smallTarget;
    device.CreateTexture(desc, nullptr, &smallTarget);
    require(static_cast<bool>(smallTarget), "Failed to create downscale target");
    desc.Name = "Color present downscale readback";
    desc.Usage = USAGE_STAGING;
    desc.BindFlags = BIND_NONE;
    desc.CPUAccessFlags = CPU_ACCESS_READ;
    RefCntAutoPtr<ITexture> smallStaging;
    device.CreateTexture(desc, nullptr, &smallStaging);
    require(static_cast<bool>(smallStaging), "Failed to create downscale readback");
    presenter.draw(context, largeSource.shaderResourceView(),
                   *smallTarget->GetDefaultView(TEXTURE_VIEW_RENDER_TARGET));
    mapped = readback(smallTarget, smallStaging);
    const bool downscaled = channel(mapped, 0, 0, 0) == 255 && channel(mapped, 0, 0, 1) == 255 &&
                            channel(mapped, 1, 0, 0) == 0 && channel(mapped, 1, 0, 1) == 255 &&
                            channel(mapped, 0, 1, 0) == 255 && channel(mapped, 0, 1, 1) == 0 &&
                            channel(mapped, 1, 1, 0) == 0 && channel(mapped, 1, 1, 1) == 0;
    context.UnmapTextureSubresource(smallStaging, 0, 0);
    require(downscaled, "Color present did not correctly downscale or resize its viewport");

    DiligentMotionRenderTargets scene(device, 3, 5, false);
    scene.clear(context, {0.25F, 0.5F, 0.75F, 1.0F});
    presenter.draw(context, scene.colorTexture(), *rtv);
    mapped = readback(target, staging);
    const bool sceneColor = channel(mapped, 4, 4, 0) >= 63 && channel(mapped, 4, 4, 0) <= 64 &&
                            channel(mapped, 4, 4, 1) >= 127 && channel(mapped, 4, 4, 1) <= 128 &&
                            channel(mapped, 4, 4, 2) >= 191 && channel(mapped, 4, 4, 2) <= 192;
    context.UnmapTextureSubresource(staging, 0, 0);
    require(sceneColor, "Color present could not sample the scene motion target color");

    DiligentMotionRenderTargets debugTargets(device, 2, 2, true);
    const float motionValues[] = {8.0F, -16.0F, 0.0F, 0.0F};
    const float validValue[] = {1.0F, 0.0F, 0.0F, 0.0F};
    context.ClearRenderTarget(&debugTargets.motionVectorRenderTarget(), motionValues,
                              RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(&debugTargets.motionValidityRenderTarget(), validValue,
                              RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    presenter.drawDebug(context, debugTargets.motionVectorTexture(),
                        debugTargets.motionValidityTexture(), *rtv,
                        DiligentDebugView::MotionVectors);
    mapped = readback(target, staging);
    const bool motionVisualization =
        channel(mapped, 4, 4, 0) >= 158 && channel(mapped, 4, 4, 0) <= 160 &&
        channel(mapped, 4, 4, 1) >= 63 && channel(mapped, 4, 4, 1) <= 65 &&
        channel(mapped, 4, 4, 2) >= 142 && channel(mapped, 4, 4, 2) <= 144;
    context.UnmapTextureSubresource(staging, 0, 0);
    require(motionVisualization, "Motion-vector debug view did not encode signed pixel motion");

    presenter.drawDebug(context, debugTargets.motionVectorTexture(),
                        debugTargets.motionValidityTexture(), *rtv,
                        DiligentDebugView::MotionMagnitude);
    mapped = readback(target, staging);
    const bool magnitudeVisualization =
        channel(mapped, 4, 4, 0) >= 142 && channel(mapped, 4, 4, 0) <= 144 &&
        channel(mapped, 4, 4, 1) >= 142 && channel(mapped, 4, 4, 1) <= 144 &&
        channel(mapped, 4, 4, 2) >= 142 && channel(mapped, 4, 4, 2) <= 144;
    context.UnmapTextureSubresource(staging, 0, 0);
    require(magnitudeVisualization, "Motion-magnitude debug view did not encode pixel speed");

    presenter.drawDebug(context, debugTargets.motionVectorTexture(),
                        debugTargets.motionValidityTexture(), *rtv,
                        DiligentDebugView::MotionValidity);
    mapped = readback(target, staging);
    const bool validityVisualization = channel(mapped, 4, 4, 0) == 255 &&
                                       channel(mapped, 4, 4, 1) == 255 &&
                                       channel(mapped, 4, 4, 2) == 255;
    context.UnmapTextureSubresource(staging, 0, 0);
    require(validityVisualization, "Motion-validity debug view did not expose valid pixels");

    const float invalidValue[] = {0.0F, 0.0F, 0.0F, 0.0F};
    context.ClearRenderTarget(&debugTargets.motionValidityRenderTarget(), invalidValue,
                              RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    presenter.drawDebug(context, debugTargets.motionVectorTexture(),
                        debugTargets.motionValidityTexture(), *rtv,
                        DiligentDebugView::MotionVectors);
    mapped = readback(target, staging);
    const bool invalidVisualization = channel(mapped, 4, 4, 0) == 255 &&
                                      channel(mapped, 4, 4, 1) == 0 &&
                                      channel(mapped, 4, 4, 2) == 255;
    context.UnmapTextureSubresource(staging, 0, 0);
    require(invalidVisualization, "Invalid motion history was not highlighted in debug view");

    const float linearDepthValue[] = {250.0F, 0.0F, 0.0F, 0.0F};
    context.ClearRenderTarget(debugTargets.linearDepthRenderTarget(), linearDepthValue,
                              RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    presenter.drawDebug(context, debugTargets.motionVectorTexture(),
                        debugTargets.motionValidityTexture(), *rtv,
                        DiligentDebugView::LinearDepth,
                        debugTargets.linearDepthTexture());
    mapped = readback(target, staging);
    const bool linearDepthVisualization =
        channel(mapped, 4, 4, 0) >= 226 && channel(mapped, 4, 4, 0) <= 228 &&
        channel(mapped, 4, 4, 1) >= 226 && channel(mapped, 4, 4, 1) <= 228 &&
        channel(mapped, 4, 4, 2) >= 226 && channel(mapped, 4, 4, 2) <= 228;
    context.UnmapTextureSubresource(staging, 0, 0);
    require(linearDepthVisualization, "Linear-depth debug view did not visualize camera distance");

    scene.resize(context, 7, 2);
    scene.clear(context, {1.0F, 0.0F, 1.0F, 1.0F});
    presenter.draw(context, scene.colorTexture(), *rtv);
    mapped = readback(target, staging);
    const bool resizedSceneColor = channel(mapped, 4, 4, 0) == 255 &&
                                   channel(mapped, 4, 4, 1) == 0 && channel(mapped, 4, 4, 2) == 255;
    context.UnmapTextureSubresource(staging, 0, 0);
    require(resizedSceneColor, "Color present used a stale scene view after resize");

    bool invalidMsaaRejected = false;
    try {
        DiligentMultisampleTargets invalid(device, 8, 8, 3);
    } catch (const std::invalid_argument&) {
        invalidMsaaRejected = true;
    }
    require(invalidMsaaRejected, "MSAA accepted an invalid sample count");
    bool invalidRendererSamplesRejected = false;
    try {
        DiligentPrimitiveRenderer invalid(device, TEX_FORMAT_RGBA8_UNORM, TEX_FORMAT_D32_FLOAT,
                                          {3});
    } catch (const std::invalid_argument&) {
        invalidRendererSamplesRejected = true;
    }
    require(invalidRendererSamplesRejected, "Primitive renderer accepted invalid samples");
    invalidRendererSamplesRejected = false;
    try {
        DiligentTexturedMeshRenderer invalid(device, TEX_FORMAT_RGBA8_UNORM, TEX_FORMAT_D32_FLOAT,
                                             DiligentMultisampling{3});
    } catch (const std::invalid_argument&) {
        invalidRendererSamplesRejected = true;
    }
    require(invalidRendererSamplesRejected, "Textured renderer accepted invalid samples");
    const auto supported = device.GetTextureFormatInfoExt(TEX_FORMAT_RGBA8_UNORM).SampleCounts &
                           device.GetTextureFormatInfoExt(TEX_FORMAT_D32_FLOAT).SampleCounts;
    const Uint32 samples = (supported & 4) ? 4 : ((supported & 2) ? 2 : 0);
    std::cout << "Diligent MSAA test samples: " << samples
              << (samples == 0 ? " (unsupported; rendering checks skipped)\n" : "\n");
    if (samples != 0) {
        constexpr Uint32 msaaSize = 64;
        DiligentMultisampleTargets msaa(device, msaaSize, msaaSize, samples);
        DiligentPrimitiveRenderer primitives(device, TEX_FORMAT_RGBA8_UNORM, TEX_FORMAT_D32_FLOAT,
                                             {samples});
        DiligentTexturedMeshRenderer meshes(device, TEX_FORMAT_RGBA8_UNORM, TEX_FORMAT_D32_FLOAT,
                                            DiligentMultisampling{samples});
        const Matrix4 identity = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
        const std::array<PrimitiveVertex, 3> red = {{
            {-0.4F, -0.5F, -0.5F, 1, 0, 0},
            {0.4F, -0.5F, -0.5F, 1, 0, 0},
            {0.0F, 0.5F, -0.5F, 1, 0, 0},
        }};
        const std::array<ModelVertex, 3> blueVertices = {{
            {-0.8F, -0.8F, 0.5F, 0, 0, 0, 0, 0, 1},
            {0.8F, -0.8F, 0.5F, 0, 0, 0, 0, 0, 1},
            {0.0F, 0.8F, 0.5F, 0, 0, 0, 0, 0, 1},
        }};
        DiligentTexturedMesh blueMesh(device, blueVertices);
        DiligentTexture2D blue(device, Image{1, 1, {0, 0, 255, 255}});
        msaa.clear(context, {0, 0, 0, 1});
        primitives.draw(context, red, identity, identity);
        meshes.draw(context, blueMesh, blue.shaderResourceView(), identity);
        msaa.resolve(context);
        TextureDesc msaaReadbackDesc;
        msaaReadbackDesc.Name = "MSAA resolved readback";
        msaaReadbackDesc.Type = RESOURCE_DIM_TEX_2D;
        msaaReadbackDesc.Width = msaaSize;
        msaaReadbackDesc.Height = msaaSize;
        msaaReadbackDesc.MipLevels = 1;
        msaaReadbackDesc.Format = TEX_FORMAT_RGBA8_UNORM;
        msaaReadbackDesc.Usage = USAGE_STAGING;
        msaaReadbackDesc.CPUAccessFlags = CPU_ACCESS_READ;
        RefCntAutoPtr<ITexture> msaaReadback;
        device.CreateTexture(msaaReadbackDesc, nullptr, &msaaReadback);
        require(static_cast<bool>(msaaReadback), "Failed to create MSAA readback");
        mapped = readback(msaa.resolvedColor().GetTexture(), msaaReadback);
        std::size_t redPixels = 0;
        std::size_t bluePixels = 0;
        std::size_t partialPixels = 0;
        for (Uint32 y = 0; y < msaaSize; ++y) {
            for (Uint32 x = 0; x < msaaSize; ++x) {
                const auto r = channel(mapped, x, y, 0);
                const auto b = channel(mapped, x, y, 2);
                redPixels += r > 240 && b < 10;
                bluePixels += b > 240 && r < 10;
                partialPixels += (r > 10 && r < 240) || (b > 10 && b < 240);
            }
        }
        const bool frontIsRed = channel(mapped, 32, 32, 0) > 240 && channel(mapped, 32, 32, 2) < 10;
        context.UnmapTextureSubresource(msaaReadback, 0, 0);
        require(frontIsRed && redPixels > 200 && bluePixels > 300 && partialPixels > 0,
                "MSAA resolve lost depth, geometry, or partial sample coverage");
        presenter.draw(context, msaa.resolvedColor(), *rtv);
        mapped = readback(target, staging);
        const bool composed = channel(mapped, 4, 4, 0) > 200;
        context.UnmapTextureSubresource(staging, 0, 0);
        require(composed, "Color present could not sample resolved MSAA scene");

        bool invalidResizeRejected = false;
        try {
            msaa.resize(context, 0, 1);
        } catch (const std::invalid_argument&) {
            invalidResizeRejected = true;
        }
        require(invalidResizeRejected && msaa.width() == msaaSize && msaa.height() == msaaSize,
                "MSAA invalid resize changed the attachments");
        bool invalidClearRejected = false;
        try {
            msaa.clear(context, {0, 0, 0, 1}, std::numeric_limits<float>::quiet_NaN());
        } catch (const std::invalid_argument&) {
            invalidClearRejected = true;
        }
        require(invalidClearRejected, "MSAA accepted an invalid depth clear");
        msaa.resize(context, 3, 5);
        require(msaa.width() == 3 && msaa.height() == 5 && msaa.samples() == samples,
                "MSAA resize did not update target dimensions");
        msaa.clear(context, {1, 0, 1, 1});
        msaa.resolve(context);
        presenter.draw(context, msaa.resolvedColor(), *rtv);
        mapped = readback(target, staging);
        const bool resizedMsaa = channel(mapped, 4, 4, 0) == 255 && channel(mapped, 4, 4, 1) == 0 &&
                                 channel(mapped, 4, 4, 2) == 255;
        context.UnmapTextureSubresource(staging, 0, 0);
        require(resizedMsaa, "MSAA resize or resolve left stale color");
    }

    Image edgeImage{8, 8, {}};
    for (int y = 0; y < 8; ++y) {
        for (int x = 0; x < 8; ++x) {
            const std::uint8_t value = x >= y ? 255 : 0;
            edgeImage.rgba.insert(edgeImage.rgba.end(), {value, value, value, 191});
        }
    }
    DiligentTexture2D edgeSource(device, edgeImage);
    presenter.draw(context, edgeSource.shaderResourceView(), *rtv);
    mapped = readback(target, staging);
    const bool unfilteredEdge = channel(mapped, 3, 3, 0) == 255;
    context.UnmapTextureSubresource(staging, 0, 0);
    presenter.draw(context, edgeSource.shaderResourceView(), *rtv, DiligentColorFilter::Fxaa);
    mapped = readback(target, staging);
    const auto edge = channel(mapped, 3, 3, 0);
    const bool filteredEdge = edge >= 205 && edge <= 210 && channel(mapped, 3, 3, 1) == edge &&
                              channel(mapped, 3, 3, 2) == edge && channel(mapped, 3, 3, 3) == 191 &&
                              channel(mapped, 7, 0, 0) == 255 && channel(mapped, 0, 7, 0) == 0;
    context.UnmapTextureSubresource(staging, 0, 0);
    require(unfilteredEdge && filteredEdge,
            "FXAA did not smooth a high-contrast edge while preserving flat regions and alpha");
    presenter.draw(context, edgeSource.shaderResourceView(), *rtv, DiligentColorFilter::None);
    mapped = readback(target, staging);
    const bool filterDisabled = channel(mapped, 3, 3, 0) == 255;
    context.UnmapTextureSubresource(staging, 0, 0);
    require(filterDisabled, "Color present retained FXAA after switching it off");

    presenter.draw(context, edgeSource.shaderResourceView(), *rtv, DiligentColorFilter::Smaa);
    mapped = readback(target, staging);
    std::size_t smoothedPixels = 0;
    bool alphaPreserved = true;
    for (Uint32 y = 0; y < 8; ++y) {
        for (Uint32 x = 0; x < 8; ++x) {
            const auto value = channel(mapped, x, y, 0);
            smoothedPixels += value > 10 && value < 245;
            alphaPreserved = alphaPreserved && channel(mapped, x, y, 3) == 191;
        }
    }
    const bool flatPreserved = channel(mapped, 7, 0, 0) == 255 && channel(mapped, 0, 7, 0) == 0;
    context.UnmapTextureSubresource(staging, 0, 0);
    require(smoothedPixels > 0 && flatPreserved && alphaPreserved,
            "SMAA did not smooth an edge while preserving flat regions and alpha");
    Image oppositeEdgeImage{8, 8, {}};
    for (int y = 0; y < 8; ++y) {
        for (int x = 0; x < 8; ++x) {
            const std::uint8_t value = x + y >= 7 ? 255 : 0;
            oppositeEdgeImage.rgba.insert(oppositeEdgeImage.rgba.end(), {value, value, value, 191});
        }
    }
    DiligentTexture2D oppositeEdgeSource(device, oppositeEdgeImage);
    presenter.draw(context, oppositeEdgeSource.shaderResourceView(), *rtv,
                   DiligentColorFilter::Smaa);
    mapped = readback(target, staging);
    std::size_t oppositeSmoothed = 0;
    for (Uint32 y = 0; y < 8; ++y) {
        for (Uint32 x = 0; x < 8; ++x) {
            const auto value = channel(mapped, x, y, 0);
            oppositeSmoothed += value > 10 && value < 245;
        }
    }
    context.UnmapTextureSubresource(staging, 0, 0);
    require(oppositeSmoothed > 0, "SMAA did not smooth the opposite diagonal");
    Image flatImage{8, 8, {}};
    for (int i = 0; i < 64; ++i) {
        flatImage.rgba.insert(flatImage.rgba.end(), {17, 47, 97, 191});
    }
    DiligentTexture2D flatSource(device, flatImage);
    presenter.draw(context, flatSource.shaderResourceView(), *rtv, DiligentColorFilter::Smaa);
    mapped = readback(target, staging);
    bool clearedWeights = true;
    for (Uint32 y = 0; y < 8; ++y) {
        for (Uint32 x = 0; x < 8; ++x) {
            clearedWeights = clearedWeights && channel(mapped, x, y, 0) == 17 &&
                             channel(mapped, x, y, 1) == 47 && channel(mapped, x, y, 2) == 97 &&
                             channel(mapped, x, y, 3) == 191;
        }
    }
    context.UnmapTextureSubresource(staging, 0, 0);
    require(clearedWeights, "SMAA retained edge data from the previous frame");
    desc.Width = 16;
    desc.Height = 16;
    desc.Name = "SMAA upscaled target";
    desc.Usage = USAGE_DEFAULT;
    desc.BindFlags = BIND_RENDER_TARGET;
    desc.CPUAccessFlags = CPU_ACCESS_NONE;
    RefCntAutoPtr<ITexture> upscaled;
    device.CreateTexture(desc, nullptr, &upscaled);
    require(static_cast<bool>(upscaled), "Failed to create SMAA upscale target");
    desc.Name = "SMAA upscaled readback";
    desc.Usage = USAGE_STAGING;
    desc.BindFlags = BIND_NONE;
    desc.CPUAccessFlags = CPU_ACCESS_READ;
    RefCntAutoPtr<ITexture> upscaledStaging;
    device.CreateTexture(desc, nullptr, &upscaledStaging);
    require(static_cast<bool>(upscaledStaging), "Failed to create SMAA upscale readback");
    presenter.draw(context, edgeSource.shaderResourceView(),
                   *upscaled->GetDefaultView(TEXTURE_VIEW_RENDER_TARGET),
                   DiligentColorFilter::Smaa);
    mapped = readback(upscaled, upscaledStaging);
    const bool upscaledSmaa = channel(mapped, 14, 1, 0) == 255 && channel(mapped, 1, 14, 0) == 0 &&
                              channel(mapped, 14, 1, 3) == 191;
    context.UnmapTextureSubresource(upscaledStaging, 0, 0);
    require(upscaledSmaa, "SMAA upscale changed orientation or flat colors");
    presenter.draw(context, source.shaderResourceView(), *rtv, DiligentColorFilter::Smaa);
    mapped = readback(target, staging);
    const bool colorOrientation = channel(mapped, 1, 1, 0) > channel(mapped, 1, 1, 1) + 60 &&
                                  channel(mapped, 6, 1, 1) > channel(mapped, 6, 1, 0) + 60 &&
                                  channel(mapped, 1, 6, 2) > channel(mapped, 1, 6, 0) + 60 &&
                                  channel(mapped, 6, 6, 0) > 200 &&
                                  channel(mapped, 6, 6, 1) > 200 && channel(mapped, 6, 6, 2) > 200;
    context.UnmapTextureSubresource(staging, 0, 0);
    require(colorOrientation, "SMAA changed the orientation of a scaled color source");
    presenter.draw(context, replacement.shaderResourceView(), *rtv, DiligentColorFilter::Smaa);
    mapped = readback(target, staging);
    const bool resizedSmaa = channel(mapped, 4, 4, 0) == 17 && channel(mapped, 4, 4, 1) == 47 &&
                             channel(mapped, 4, 4, 2) == 97 && channel(mapped, 4, 4, 3) == 191;
    context.UnmapTextureSubresource(staging, 0, 0);
    require(resizedSmaa, "SMAA resized source did not preserve a flat color");
    presenter.draw(context, edgeSource.shaderResourceView(), *rtv, DiligentColorFilter::None);
    mapped = readback(target, staging);
    const bool smaaDisabled = channel(mapped, 3, 3, 0) == 255;
    context.UnmapTextureSubresource(staging, 0, 0);
    require(smaaDisabled, "Color present retained SMAA after switching it off");

    bool aliasRejected = false;
    try {
        presenter.draw(context, *srv, *rtv);
    } catch (const std::invalid_argument&) {
        aliasRejected = true;
    }
    require(aliasRejected, "Color present accepted the same source and destination texture");
    aliasRejected = false;
    try {
        presenter.draw(context, *srv, *rtv, DiligentColorFilter::Smaa);
    } catch (const std::invalid_argument&) {
        aliasRejected = true;
    }
    require(aliasRejected, "SMAA accepted the same source and destination texture");
    bool invalidViewRejected = false;
    try {
        presenter.draw(context, *rtv, *rtv);
    } catch (const std::invalid_argument&) {
        invalidViewRejected = true;
    }
    require(invalidViewRejected, "Color present accepted an invalid source view");
    bool invalidFilterRejected = false;
    try {
        presenter.draw(context, source.shaderResourceView(), *rtv,
                       static_cast<DiligentColorFilter>(-1));
    } catch (const std::invalid_argument&) {
        invalidFilterRejected = true;
    }
    require(invalidFilterRejected, "Color present accepted an invalid filter");
}
