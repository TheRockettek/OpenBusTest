#include "DiligentTexturedMeshRendering.h"

#include "ProbeTempDirectory.h"
#include "assets/TextureLoader.h"
#include "assets/VehicleMeshGeometry.h"
#include "app/DiligentMapBridge.h"
#include "app/DiligentScriptTexture.h"
#include "core/Variables.h"
#include "map/MapConfigLoader.h"
#include "map/MapRoadGeometry.h"
#include "map/MapRoadRenderGeometry.h"
#include "map/MapSceneryGeometry.h"
#include "map/MapSurfaceGeometry.h"
#include "render/diligent/DiligentObjModel.h"
#include "render/diligent/DiligentSceneryAsset.h"
#include "render/diligent/DiligentSceneryAssetCache.h"
#include "render/diligent/DiligentDynamicTexture2D.h"
#include "render/diligent/DiligentHumanAsset.h"
#include "render/diligent/DiligentMapTileAsset.h"
#include "render/diligent/DiligentMapTileSet.h"
#include "render/diligent/DiligentMotionRenderTargets.h"
#include "render/diligent/DiligentRuntimeTexture2D.h"
#include "render/diligent/DiligentSceneFrame.h"
#include "render/diligent/DiligentSceneryAsset.h"
#include "render/diligent/DiligentTexture2D.h"
#include "render/diligent/DiligentTexture2DArray.h"
#include "render/diligent/DiligentTexturedMesh.h"
#include "render/diligent/DiligentVehicleAsset.h"
#include "render/CoreRenderer.h"
#include "render/MatrixMath.h"
#include "render/map/PreparedMapScenery.h"
#include "config/ModelConfigTypes.h"
#include "scripting/ScriptRuntime.h"

#include <Common/interface/RefCntAutoPtr.hpp>
#include <Graphics/GraphicsEngine/interface/DeviceContext.h>
#include <Graphics/GraphicsEngine/interface/RenderDevice.h>
#include <Graphics/GraphicsEngine/interface/Texture.h>
#include <gli/gli.hpp>
#include <gli/save_dds.hpp>

#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#ifdef near
#undef near
#endif
#ifdef far
#undef far
#endif

namespace {

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

} // namespace

void verifyDiligentTexturedMeshRendering(Diligent::IRenderDevice& device,
                                         Diligent::IDeviceContext& context) {
    using namespace Diligent;
    using namespace openbus::rendering;
    constexpr Uint32 size = 64;

    DiligentMotionRenderTargets motionTargets(device, 16, 8);
    require(motionTargets.width() == 16 && motionTargets.height() == 8 &&
                motionTargets.hasLinearDepth() &&
                motionTargets.motionVectorTexture().GetTexture()->GetDesc().Format ==
                    TEX_FORMAT_RG32_FLOAT &&
                motionTargets.motionValidityTexture().GetTexture()->GetDesc().Format ==
                    TEX_FORMAT_R8_UNORM &&
                motionTargets.linearDepthTexture()->GetTexture()->GetDesc().Format ==
                    TEX_FORMAT_R32_FLOAT,
            "Diligent motion targets did not create the expected metadata resources");
    motionTargets.clear(context);
    motionTargets.resize(context, 32, 16);
    require(motionTargets.width() == 32 && motionTargets.height() == 16,
            "Diligent motion target resize did not update its dimensions");
    DiligentMotionRenderTargets colorMotionTargets(device, 8, 8, false);
    require(!colorMotionTargets.hasLinearDepth() &&
                colorMotionTargets.linearDepthTexture() == nullptr,
            "Diligent motion targets created linear depth when it was disabled");

    TextureDesc targetDesc;
    targetDesc.Name = "Textured mesh test target";
    targetDesc.Type = RESOURCE_DIM_TEX_2D;
    targetDesc.Width = size;
    targetDesc.Height = size;
    targetDesc.MipLevels = 1;
    targetDesc.Format = TEX_FORMAT_RGBA8_UNORM;
    targetDesc.BindFlags = BIND_RENDER_TARGET;
    RefCntAutoPtr<ITexture> target;
    device.CreateTexture(targetDesc, nullptr, &target);
    require(static_cast<bool>(target), "Failed to create textured mesh target");

    TextureDesc depthDesc = targetDesc;
    depthDesc.Format = TEX_FORMAT_D32_FLOAT;
    depthDesc.BindFlags = BIND_DEPTH_STENCIL;
    RefCntAutoPtr<ITexture> depth;
    device.CreateTexture(depthDesc, nullptr, &depth);
    require(static_cast<bool>(depth), "Failed to create textured mesh depth target");

    TextureDesc linearDepthDesc = targetDesc;
    linearDepthDesc.Name = "Textured mesh linear-depth target";
    linearDepthDesc.Format = TEX_FORMAT_R32_FLOAT;
    RefCntAutoPtr<ITexture> linearDepthTarget;
    device.CreateTexture(linearDepthDesc, nullptr, &linearDepthTarget);
    require(static_cast<bool>(linearDepthTarget),
            "Failed to create textured mesh linear-depth target");

    TextureDesc stagingDesc = targetDesc;
    stagingDesc.Usage = USAGE_STAGING;
    stagingDesc.BindFlags = BIND_NONE;
    stagingDesc.CPUAccessFlags = CPU_ACCESS_READ;
    RefCntAutoPtr<ITexture> staging;
    device.CreateTexture(stagingDesc, nullptr, &staging);
    require(static_cast<bool>(staging), "Failed to create textured mesh staging texture");

    TextureDesc linearDepthStagingDesc = linearDepthDesc;
    linearDepthStagingDesc.Name = "Textured mesh linear-depth staging texture";
    linearDepthStagingDesc.Usage = USAGE_STAGING;
    linearDepthStagingDesc.BindFlags = BIND_NONE;
    linearDepthStagingDesc.CPUAccessFlags = CPU_ACCESS_READ;
    RefCntAutoPtr<ITexture> linearDepthStaging;
    device.CreateTexture(linearDepthStagingDesc, nullptr, &linearDepthStaging);
    require(static_cast<bool>(linearDepthStaging),
            "Failed to create textured mesh linear-depth staging texture");

    const std::array<std::uint8_t, 16> pattern = {
        255, 0, 0, 255, 0, 255, 0, 255, 0, 0, 255, 255, 255, 255, 255, 255,
    };
    const auto textureDirectory =
        openbus::probe::makeProbeDirectory("openbus_diligent_texture", false);
    struct DirectoryCleanup {
        std::filesystem::path path;
        ~DirectoryCleanup() {
            openbus::probe::removeProbeDirectory(path);
        }
    } cleanup{textureDirectory};
    std::array<std::uint8_t, 34> tga = {};
    tga[2] = 2;
    tga[12] = 2;
    tga[14] = 2;
    tga[16] = 32;
    tga[17] = 0x20;
    for (std::size_t pixel = 0; pixel < 4; ++pixel) {
        tga[18 + pixel * 4] = pattern[pixel * 4 + 2];
        tga[19 + pixel * 4] = pattern[pixel * 4 + 1];
        tga[20 + pixel * 4] = pattern[pixel * 4];
        tga[21 + pixel * 4] = pattern[pixel * 4 + 3];
    }
    const auto imagePath = textureDirectory / "decoded-pattern.tga";
    {
        std::ofstream output(imagePath, std::ios::binary);
        output.write(reinterpret_cast<const char*>(tga.data()),
                     static_cast<std::streamsize>(tga.size()));
        require(static_cast<bool>(output), "Failed to write Diligent texture upload fixture");
    }
    Image patternImage;
    require(TextureLoader::readImage(imagePath, patternImage),
            "Failed to decode Diligent texture upload fixture");
    require(patternImage.width == 2 && patternImage.height == 2 &&
                std::equal(patternImage.rgba.begin(), patternImage.rgba.end(), pattern.begin(),
                           pattern.end()),
            "Texture loader did not preserve the RGBA fixture");
    auto patternTexture = DiligentTexture2D::load(device, imagePath);
    require(patternTexture->shaderResourceView().GetTexture()->GetDesc().MipLevels == 1,
            "Diligent decoded-image upload changed its default single-mip behavior");
    auto patternMipTexture = DiligentTexture2D::load(device, imagePath, true);
    require(patternMipTexture->shaderResourceView().GetTexture()->GetDesc().MipLevels == 2,
            "Diligent decoded-image loader did not generate the requested mip level");
    bool missingTextureRejected = false;
    try {
        static_cast<void>(DiligentTexture2D::load(device, textureDirectory / "missing.tga"));
    } catch (const std::runtime_error&) {
        missingTextureRejected = true;
    }
    require(missingTextureRejected, "Diligent texture loader accepted a missing asset");
    const std::array<std::uint8_t, 4> magenta = {255, 0, 255, 255};
    const std::array<std::uint8_t, 4> cyan = {0, 255, 255, 255};
    const std::array<std::uint8_t, 4> transparentGreen = {0, 255, 0, 0};
    const std::array<std::uint8_t, 4> translucentGreen = {0, 255, 0, 128};
    const std::array<std::uint8_t, 16> scalePattern = {
        255, 0, 0, 255, 255, 0, 0, 255, 0, 0, 255, 255, 0, 0, 255, 255,
    };
    const auto makeTexture = [&](const char* name, Uint32 width, Uint32 height,
                                 const std::uint8_t* pixels) {
        TextureDesc desc;
        desc.Name = name;
        desc.Type = RESOURCE_DIM_TEX_2D;
        desc.Width = width;
        desc.Height = height;
        desc.MipLevels = 1;
        desc.Format = TEX_FORMAT_RGBA8_UNORM;
        desc.BindFlags = BIND_SHADER_RESOURCE;
        desc.Usage = USAGE_IMMUTABLE;
        TextureSubResData subresource;
        subresource.pData = pixels;
        subresource.Stride = width * 4;
        TextureData data;
        data.pSubResources = &subresource;
        data.NumSubresources = 1;
        RefCntAutoPtr<ITexture> texture;
        device.CreateTexture(desc, &data, &texture);
        require(static_cast<bool>(texture), "Failed to create textured mesh test texture");
        return texture;
    };
    Image mipSource;
    mipSource.width = 4;
    mipSource.height = 4;
    mipSource.rgba.resize(4 * 4 * 4);
    const std::array<std::array<std::uint8_t, 4>, 4> mipColors = {{
        {255, 0, 0, 255}, {0, 255, 0, 255}, {0, 0, 255, 255}, {255, 255, 255, 255},
    }};
    for (int y = 0; y < mipSource.height; ++y) {
        for (int x = 0; x < mipSource.width; ++x) {
            const auto& color = mipColors[static_cast<std::size_t>((y / 2) * 2 + x / 2)];
            const std::size_t offset =
                (static_cast<std::size_t>(y) * mipSource.width + x) * 4;
            std::copy(color.begin(), color.end(), mipSource.rgba.begin() + offset);
        }
    }
    DiligentTexture2D generatedMipTexture(device, mipSource, true);
    const auto& generatedMipDesc = generatedMipTexture.shaderResourceView().GetTexture()->GetDesc();
    require(generatedMipDesc.Width == 4 && generatedMipDesc.Height == 4 &&
                generatedMipDesc.MipLevels == 3,
            "Diligent decoded-image upload did not create its full mip chain");
    TextureDesc finalMipStagingDesc;
    finalMipStagingDesc.Name = "Diligent generated-mip readback";
    finalMipStagingDesc.Type = RESOURCE_DIM_TEX_2D;
    finalMipStagingDesc.Width = 1;
    finalMipStagingDesc.Height = 1;
    finalMipStagingDesc.MipLevels = 1;
    finalMipStagingDesc.Format = TEX_FORMAT_RGBA8_UNORM;
    finalMipStagingDesc.Usage = USAGE_STAGING;
    finalMipStagingDesc.BindFlags = BIND_NONE;
    finalMipStagingDesc.CPUAccessFlags = CPU_ACCESS_READ;
    RefCntAutoPtr<ITexture> finalMipStaging;
    device.CreateTexture(finalMipStagingDesc, nullptr, &finalMipStaging);
    require(static_cast<bool>(finalMipStaging),
            "Failed to create Diligent generated-mip readback texture");
    CopyTextureAttribs finalMipCopy;
    finalMipCopy.pSrcTexture = generatedMipTexture.shaderResourceView().GetTexture();
    finalMipCopy.SrcMipLevel = 2;
    finalMipCopy.SrcTextureTransitionMode = RESOURCE_STATE_TRANSITION_MODE_TRANSITION;
    finalMipCopy.pDstTexture = finalMipStaging;
    finalMipCopy.DstTextureTransitionMode = RESOURCE_STATE_TRANSITION_MODE_TRANSITION;
    context.CopyTexture(finalMipCopy);
    context.WaitForIdle();
    MappedTextureSubresource finalMipMapped;
    context.MapTextureSubresource(finalMipStaging, 0, 0, MAP_READ, MAP_FLAG_DO_NOT_WAIT, nullptr,
                                  finalMipMapped);
    require(finalMipMapped.pData != nullptr, "Failed to map Diligent generated-mip readback");
    const auto* finalMipPixel = static_cast<const std::uint8_t*>(finalMipMapped.pData);
    const bool generatedMipAverageCorrect =
        finalMipPixel[0] == 128 && finalMipPixel[1] == 128 && finalMipPixel[2] == 128 &&
        finalMipPixel[3] == 255;
    context.UnmapTextureSubresource(finalMipStaging, 0, 0);
    require(generatedMipAverageCorrect,
            "Diligent decoded-image mip chain did not preserve the expected area average");
    Image oddMipSource;
    oddMipSource.width = 5;
    oddMipSource.height = 1;
    oddMipSource.rgba.resize(5 * 4);
    for (int x = 0; x < oddMipSource.width; ++x) {
        oddMipSource.rgba[static_cast<std::size_t>(x) * 4] = x >= 2 ? 255 : 0;
        oddMipSource.rgba[static_cast<std::size_t>(x) * 4 + 3] = 255;
    }
    DiligentTexture2D oddGeneratedMipTexture(device, oddMipSource, true);
    TextureDesc oddMipStagingDesc = finalMipStagingDesc;
    oddMipStagingDesc.Name = "Diligent odd-sized generated-mip readback";
    oddMipStagingDesc.Width = 2;
    RefCntAutoPtr<ITexture> oddMipStaging;
    device.CreateTexture(oddMipStagingDesc, nullptr, &oddMipStaging);
    require(static_cast<bool>(oddMipStaging),
            "Failed to create Diligent odd-sized mip readback texture");
    CopyTextureAttribs oddMipCopy;
    oddMipCopy.pSrcTexture = oddGeneratedMipTexture.shaderResourceView().GetTexture();
    oddMipCopy.SrcMipLevel = 1;
    oddMipCopy.SrcTextureTransitionMode = RESOURCE_STATE_TRANSITION_MODE_TRANSITION;
    oddMipCopy.pDstTexture = oddMipStaging;
    oddMipCopy.DstTextureTransitionMode = RESOURCE_STATE_TRANSITION_MODE_TRANSITION;
    context.CopyTexture(oddMipCopy);
    context.WaitForIdle();
    MappedTextureSubresource oddMipMapped;
    context.MapTextureSubresource(oddMipStaging, 0, 0, MAP_READ, MAP_FLAG_DO_NOT_WAIT, nullptr,
                                  oddMipMapped);
    require(oddMipMapped.pData != nullptr, "Failed to map Diligent odd-sized mip readback");
    const auto* oddMipPixels = static_cast<const std::uint8_t*>(oddMipMapped.pData);
    const bool oddMipAverageCorrect =
        oddMipPixels[0] == 51 && oddMipPixels[4] == 255;
    context.UnmapTextureSubresource(oddMipStaging, 0, 0);
    require(oddMipAverageCorrect,
            "Diligent odd-sized mip generation did not area-weight source texels correctly");
    constexpr std::array<std::uint8_t, 16> solidRedBc3Block = {
        255, 255, 0, 0, 0, 0, 0, 0, 0, 248, 0, 248, 0, 0, 0, 0,
    };
    CompressedDds compressedRedDds;
    compressedRedDds.width = 8;
    compressedRedDds.height = 8;
    compressedRedDds.levels.emplace_back(4 * solidRedBc3Block.size());
    for (std::size_t block = 0; block < 4; ++block) {
        std::copy(solidRedBc3Block.begin(), solidRedBc3Block.end(),
                  compressedRedDds.levels[0].begin() + block * solidRedBc3Block.size());
    }
    compressedRedDds.levels.emplace_back(solidRedBc3Block.begin(), solidRedBc3Block.end());
    DiligentTexture2D compressedRedTexture(device, compressedRedDds);
    const auto& compressedTextureDesc =
        compressedRedTexture.shaderResourceView().GetTexture()->GetDesc();
    require(compressedTextureDesc.Format == TEX_FORMAT_BC3_UNORM &&
                compressedTextureDesc.MipLevels == 2,
            "Diligent compressed DDS upload did not preserve BC3 format and mip levels");
    gli::texture2d bc1Data(gli::FORMAT_RGB_DXT1_UNORM_BLOCK8, gli::extent2d(8, 8), 2);
    std::memset(bc1Data.data(), 0, bc1Data.size());
    DiligentTexture2D bc1Texture(device, bc1Data);
    const auto& bc1Desc = bc1Texture.shaderResourceView().GetTexture()->GetDesc();
    require(bc1Desc.Format == TEX_FORMAT_BC1_UNORM && bc1Desc.Width == 8 &&
                bc1Desc.Height == 8 && bc1Desc.MipLevels == 2,
            "Diligent GLI upload did not preserve BC1 format and mip levels");
    gli::texture2d bc1AlphaData(gli::FORMAT_RGBA_DXT1_UNORM_BLOCK8, gli::extent2d(8, 8), 2);
    auto* bc1AlphaBytes = static_cast<std::uint8_t*>(bc1AlphaData.data());
    std::memset(bc1AlphaBytes, 0, bc1AlphaData.size());
    for (std::size_t blockOffset = 0; blockOffset < bc1AlphaData.size(); blockOffset += 8) {
        std::memset(bc1AlphaBytes + blockOffset + 4, 0xff, 4);
    }
    const auto bc1AlphaDdsPath = textureDirectory / "bc1-alpha.dds";
    require(gli::save_dds(bc1AlphaData, bc1AlphaDdsPath.string()),
            "Failed to write Diligent BC1 alpha DDS fixture");
    const auto bc1AlphaTexture = DiligentTexture2D::load(device, bc1AlphaDdsPath);
    const auto& bc1AlphaDesc = bc1AlphaTexture->shaderResourceView().GetTexture()->GetDesc();
    require(bc1AlphaDesc.Format == TEX_FORMAT_RGBA8_UNORM && bc1AlphaDesc.Width == 8 &&
                bc1AlphaDesc.Height == 8 && bc1AlphaDesc.MipLevels == 2,
            "Diligent BC1 alpha upload did not preserve transparency and mip levels");
    gli::texture2d bc1AlphaSrgbData(gli::FORMAT_RGBA_DXT1_SRGB_BLOCK8, gli::extent2d(4, 4), 1);
    std::memset(bc1AlphaSrgbData.data(), 0, bc1AlphaSrgbData.size());
    DiligentTexture2D bc1AlphaSrgbTexture(device, bc1AlphaSrgbData);
    require(bc1AlphaSrgbTexture.shaderResourceView().GetTexture()->GetDesc().Format ==
                TEX_FORMAT_RGBA8_UNORM_SRGB,
            "Diligent BC1 alpha sRGB upload did not preserve sRGB sampling");
    gli::texture2d bc2Data(gli::FORMAT_RGBA_DXT3_SRGB_BLOCK16, gli::extent2d(4, 4), 1);
    std::memset(bc2Data.data(), 0, bc2Data.size());
    const auto bc2DdsPath = textureDirectory / "bc2-srgb.dds";
    require(gli::save_dds(bc2Data, bc2DdsPath.string()),
            "Failed to write Diligent BC2 sRGB DDS fixture");
    const auto bc2Texture = DiligentTexture2D::load(device, bc2DdsPath);
    const auto& bc2Desc = bc2Texture->shaderResourceView().GetTexture()->GetDesc();
    require(bc2Desc.Format == TEX_FORMAT_BC2_UNORM_SRGB && bc2Desc.MipLevels == 1,
            "Diligent GLI upload did not preserve BC2 sRGB format");
    gli::texture2d bc5Data(gli::FORMAT_RG_ATI2N_UNORM_BLOCK16, gli::extent2d(8, 8), 2);
    std::memset(bc5Data.data(), 0, bc5Data.size());
    const auto bc5DdsPath = textureDirectory / "bc5.dds";
    require(gli::save_dds(bc5Data, bc5DdsPath.string()),
            "Failed to write Diligent BC5 DDS fixture");
    require(TextureLoader::isSafeCompressedDds(bc5DdsPath),
            "Texture loader did not recognize the BC5 DDS block layout");
    const auto bc5Texture = DiligentTexture2D::load(device, bc5DdsPath);
    const auto& bc5Desc = bc5Texture->shaderResourceView().GetTexture()->GetDesc();
    require(bc5Desc.Format == TEX_FORMAT_BC5_UNORM && bc5Desc.Width == 8 &&
                bc5Desc.Height == 8 && bc5Desc.MipLevels == 2,
            "Diligent GLI upload did not preserve BC5 format and mip levels");
    gli::texture2d bc4Data(gli::FORMAT_R_ATI1N_SNORM_BLOCK8, gli::extent2d(4, 4), 1);
    std::memset(bc4Data.data(), 0, bc4Data.size());
    DiligentTexture2D bc4Texture(device, bc4Data);
    require(bc4Texture.shaderResourceView().GetTexture()->GetDesc().Format == TEX_FORMAT_BC4_SNORM,
            "Diligent GLI upload did not preserve signed BC4 format");
    gli::texture2d bc5SnormData(gli::FORMAT_RG_ATI2N_SNORM_BLOCK16, gli::extent2d(4, 4), 1);
    std::memset(bc5SnormData.data(), 0, bc5SnormData.size());
    DiligentTexture2D bc5SnormTexture(device, bc5SnormData);
    require(bc5SnormTexture.shaderResourceView().GetTexture()->GetDesc().Format ==
                TEX_FORMAT_BC5_SNORM,
            "Diligent GLI upload did not preserve signed BC5 format");
    gli::texture2d bc6Data(gli::FORMAT_RGB_BP_UFLOAT_BLOCK16, gli::extent2d(4, 4), 1);
    std::memset(bc6Data.data(), 0, bc6Data.size());
    DiligentTexture2D bc6Texture(device, bc6Data);
    require(bc6Texture.shaderResourceView().GetTexture()->GetDesc().Format ==
                TEX_FORMAT_BC6H_UF16,
            "Diligent GLI upload did not preserve BC6H HDR format");
    gli::texture2d bc7Data(gli::FORMAT_RGBA_BP_SRGB_BLOCK16, gli::extent2d(4, 4), 1);
    std::memset(bc7Data.data(), 0, bc7Data.size());
    const auto bc7DdsPath = textureDirectory / "bc7-srgb.dds";
    require(gli::save_dds(bc7Data, bc7DdsPath.string()),
            "Failed to write Diligent BC7 sRGB DDS fixture");
    const auto bc7Texture = DiligentTexture2D::load(device, bc7DdsPath);
    require(bc7Texture->shaderResourceView().GetTexture()->GetDesc().Format ==
                TEX_FORMAT_BC7_UNORM_SRGB,
            "Diligent asset loading did not preserve BC7 sRGB format");
    auto* patternView = &patternTexture->shaderResourceView();
    auto magentaTexture = makeTexture("Magenta", 1, 1, magenta.data());
    auto cyanTexture = makeTexture("Cyan", 1, 1, cyan.data());
    auto transparentTexture = makeTexture("Transparent green", 1, 1, transparentGreen.data());
    auto translucentTexture = makeTexture("Translucent green", 1, 1, translucentGreen.data());
    auto scaleTexture = makeTexture("UV scale pattern", 4, 1, scalePattern.data());
    auto* magentaView = magentaTexture->GetDefaultView(TEXTURE_VIEW_SHADER_RESOURCE);
    auto* cyanView = cyanTexture->GetDefaultView(TEXTURE_VIEW_SHADER_RESOURCE);
    auto* scaleView = scaleTexture->GetDefaultView(TEXTURE_VIEW_SHADER_RESOURCE);
    auto* transparentView = transparentTexture->GetDefaultView(TEXTURE_VIEW_SHADER_RESOURCE);
    auto* translucentView = translucentTexture->GetDefaultView(TEXTURE_VIEW_SHADER_RESOURCE);
    auto* targetView = target->GetDefaultView(TEXTURE_VIEW_RENDER_TARGET);
    auto* depthView = depth->GetDefaultView(TEXTURE_VIEW_DEPTH_STENCIL);
    auto* linearDepthView = linearDepthTarget->GetDefaultView(TEXTURE_VIEW_RENDER_TARGET);
    require(patternView && magentaView && cyanView && scaleView && transparentView &&
                translucentView && targetView && depthView && linearDepthView,
            "Textured mesh test view is missing");

    const auto quad = [](float halfSize, float z) {
        return std::array<ModelVertex, 6>{{
            {-halfSize, -halfSize, z, 0, 1, 0, 0, 0, 1},
            {halfSize, -halfSize, z, 1, 1, 0, 0, 0, 1},
            {halfSize, halfSize, z, 1, 0, 0, 0, 1},
            {-halfSize, -halfSize, z, 0, 1, 0, 0, 0, 1},
            {halfSize, halfSize, z, 1, 0, 0, 0, 1},
            {-halfSize, halfSize, z, 0, 0, 0, 0, 1},
        }};
    };
    const auto fullVertices = quad(0.8f, -0.5f);
    const auto nearVertices = quad(0.2f, -0.75f);
    const auto farVertices = quad(0.8f, 0.5f);
    DiligentTexturedMesh full(device, fullVertices);
    DiligentTexturedMesh near(device, nearVertices);
    DiligentTexturedMesh far(device, farVertices);
    std::array<Image, 2> textureLayers;
    textureLayers[0] = {1, 1, {255, 0, 0, 255}};
    textureLayers[1] = {1, 1, {0, 0, 255, 255}};
    DiligentTexture2DArray textureArray(device, textureLayers);
    require(textureArray.layerCount() == textureLayers.size() &&
                textureArray.shaderResourceView().GetDesc().TextureDim == RESOURCE_DIM_TEX_2D_ARRAY,
            "Diligent texture array did not expose its array view and layer count");
    const std::array<Image, 1> singleLayerImage = {Image{1, 1, {255, 0, 0, 255}}};
    DiligentTexture2DArray singleLayerArray(device, singleLayerImage);
    require(singleLayerArray.shaderResourceView().GetDesc().TextureDim == RESOURCE_DIM_TEX_2D_ARRAY,
            "Single-layer texture array did not retain its array view dimension");
    auto* textureArrayView = &textureArray.shaderResourceView();
    DiligentTexturedMeshRenderer renderer(device, targetDesc.Format, depthDesc.Format);
    const Matrix4 identity = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    const float clear[] = {0, 0, 0, 1};
    context.SetRenderTargets(1, &targetView, depthView, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearDepthStencil(depthView, CLEAR_DEPTH_FLAG, 1, 0,
                              RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    renderer.draw(context, full, *patternTexture, identity);
    renderer.draw(context, near, *magentaView, identity);
    renderer.draw(context, far, *cyanView, identity);
    DiligentTexturedMesh beforeNear(device, quad(0.8f, -1.1f));
    DiligentTexturedMesh beyondFar(device, quad(0.8f, 1.1f));
    renderer.draw(context, beforeNear, *cyanView, identity);
    renderer.draw(context, beyondFar, *cyanView, identity);

    bool invalidCount = false;
    try {
        DiligentTexturedMesh invalid(device, std::span{fullVertices}.first(2));
    } catch (const std::invalid_argument&) {
        invalidCount = true;
    }
    require(invalidCount, "Textured mesh accepted an incomplete triangle");
    bool emptyMesh = false;
    try {
        DiligentTexturedMesh invalid(device, {});
    } catch (const std::invalid_argument&) {
        emptyMesh = true;
    }
    require(emptyMesh, "Textured mesh accepted an empty vertex list");
    bool invalidTextureArray = false;
    try {
        std::array<Image, 2> invalidLayers = {
            Image{1, 1, {255, 255, 255, 255}},
            Image{2, 1, {255, 255, 255, 255, 255, 255, 255, 255}},
        };
        DiligentTexture2DArray rejected(device, invalidLayers);
    } catch (const std::invalid_argument&) {
        invalidTextureArray = true;
    }
    require(invalidTextureArray, "Diligent texture array accepted mismatched layer dimensions");
    bool invalidVertex = false;
    try {
        auto invalidVertices = fullVertices;
        invalidVertices[0].u = std::numeric_limits<float>::quiet_NaN();
        DiligentTexturedMesh rejected(device, invalidVertices);
    } catch (const std::invalid_argument&) {
        invalidVertex = true;
    }
    require(invalidVertex, "Diligent mesh accepted non-finite vertex data");
    bool invalidImageSize = false;
    try {
        Image invalid;
        invalid.width = 2;
        invalid.height = 2;
        DiligentTexture2D rejected(device, invalid);
    } catch (const std::invalid_argument&) {
        invalidImageSize = true;
    }
    require(invalidImageSize, "Diligent texture accepted mismatched decoded image storage");
    bool invalidImageDimensions = false;
    try {
        Image invalid;
        invalid.width = -1;
        invalid.height = 1;
        DiligentTexture2D rejected(device, invalid);
    } catch (const std::invalid_argument&) {
        invalidImageDimensions = true;
    }
    require(invalidImageDimensions, "Diligent texture accepted invalid decoded image dimensions");
    bool oversizedImage = false;
    try {
        Image invalid;
        invalid.width = std::numeric_limits<int>::max();
        invalid.height = 1;
        DiligentTexture2D rejected(device, invalid);
    } catch (const std::invalid_argument&) {
        oversizedImage = true;
    }
    require(oversizedImage, "Diligent texture accepted an image beyond the decoder size limit");
    bool invalidView = false;
    try {
        renderer.draw(context, full, *targetView, identity);
    } catch (const std::invalid_argument&) {
        invalidView = true;
    }
    require(invalidView, "Textured mesh accepted a render-target view as a texture");
    Matrix4 invalidMatrix = identity;
    invalidMatrix[0] = std::numeric_limits<double>::quiet_NaN();
    bool invalidTransform = false;
    try {
        renderer.draw(context, full, *patternView, invalidMatrix);
    } catch (const std::invalid_argument&) {
        invalidTransform = true;
    }
    require(invalidTransform, "Textured mesh accepted a non-finite transform");
    bool invalidAlphaMode = false;
    try {
        DiligentMaterialState invalidMaterial;
        invalidMaterial.alphaMode = static_cast<DiligentAlphaMode>(255);
        renderer.draw(context, full, *patternView, identity, invalidMaterial);
    } catch (const std::invalid_argument&) {
        invalidAlphaMode = true;
    }
    require(invalidAlphaMode, "Textured mesh accepted an unsupported alpha mode");
    bool invalidMaterialColor = false;
    try {
        DiligentMaterialState invalidMaterial;
        invalidMaterial.baseColor[1] = std::numeric_limits<float>::infinity();
        renderer.draw(context, full, *patternView, identity, invalidMaterial);
    } catch (const std::invalid_argument&) {
        invalidMaterialColor = true;
    }
    require(invalidMaterialColor, "Textured mesh accepted a non-finite material color");
    bool invalidSpecularPower = false;
    try {
        DiligentMaterialState invalidMaterial;
        invalidMaterial.specularPower = 0.0F;
        renderer.draw(context, full, *patternView, identity, invalidMaterial);
    } catch (const std::invalid_argument&) {
        invalidSpecularPower = true;
    }
    require(invalidSpecularPower, "Textured mesh accepted a non-positive specular power");
    bool invalidEmissionColor = false;
    try {
        DiligentMaterialState invalidMaterial;
        invalidMaterial.emissionColor[0] = 1.1F;
        renderer.draw(context, full, *patternView, identity, invalidMaterial);
    } catch (const std::invalid_argument&) {
        invalidEmissionColor = true;
    }
    require(invalidEmissionColor, "Textured mesh accepted an out-of-range emission mask");
    bool invalidVisibilityFade = false;
    try {
        DiligentMaterialState invalidMaterial;
        invalidMaterial.visibilityFade = std::numeric_limits<float>::quiet_NaN();
        renderer.draw(context, full, *patternView, identity, invalidMaterial);
    } catch (const std::invalid_argument&) {
        invalidVisibilityFade = true;
    }
    require(invalidVisibilityFade, "Textured mesh accepted a non-finite visibility fade");
    bool invalidTextureScale = false;
    try {
        DiligentMaterialState invalidMaterial;
        invalidMaterial.textureScale[0] = std::numeric_limits<float>::infinity();
        renderer.draw(context, full, *patternView, identity, invalidMaterial);
    } catch (const std::invalid_argument&) {
        invalidTextureScale = true;
    }
    require(invalidTextureScale, "Textured mesh accepted a non-finite texture scale");
    bool invalidTextureOffset = false;
    try {
        DiligentMaterialState invalidMaterial;
        invalidMaterial.textureOffset[1] = std::numeric_limits<float>::quiet_NaN();
        renderer.draw(context, full, *patternView, identity, invalidMaterial);
    } catch (const std::invalid_argument&) {
        invalidTextureOffset = true;
    }
    require(invalidTextureOffset, "Textured mesh accepted a non-finite texture offset");
    bool invalidLightDirection = false;
    try {
        DiligentLightingParameters invalidLighting;
        invalidLighting.directionalEnabled = true;
        invalidLighting.lightDirectionView = {0.0F, 0.0F, 0.0F};
        renderer.draw(context, full, *patternView, identity, {}, invalidLighting);
    } catch (const std::invalid_argument&) {
        invalidLightDirection = true;
    }
    require(invalidLightDirection, "Textured mesh accepted a zero directional-light vector");
    bool invalidPointLight = false;
    try {
        DiligentPointLight invalidLighting;
        invalidLighting.radius = 0.0F;
        renderer.setPointLights(context, std::span{&invalidLighting, 1});
    } catch (const std::invalid_argument&) {
        invalidPointLight = true;
    }
    require(invalidPointLight, "Textured mesh accepted a point light with zero radius");
    bool invalidSpotLightDirection = false;
    try {
        DiligentSpotLight invalidLighting;
        invalidLighting.directionView = {};
        renderer.setSpotLights(context, std::span{&invalidLighting, 1});
    } catch (const std::invalid_argument&) {
        invalidSpotLightDirection = true;
    }
    require(invalidSpotLightDirection, "Textured mesh accepted a zero spotlight direction");

    CopyTextureAttribs copy;
    copy.pSrcTexture = target;
    copy.SrcTextureTransitionMode = RESOURCE_STATE_TRANSITION_MODE_TRANSITION;
    copy.pDstTexture = staging;
    copy.DstTextureTransitionMode = RESOURCE_STATE_TRANSITION_MODE_TRANSITION;
    const auto readback = [&]() {
        context.SetRenderTargets(0, nullptr, nullptr, RESOURCE_STATE_TRANSITION_MODE_NONE);
        context.CopyTexture(copy);
        context.WaitForIdle();
        MappedTextureSubresource mapped;
        context.MapTextureSubresource(staging, 0, 0, MAP_READ, MAP_FLAG_DO_NOT_WAIT, nullptr,
                                      mapped);
        require(mapped.pData != nullptr, "Failed to map textured mesh readback");
        return mapped;
    };
    const auto pixelIs = [](const MappedTextureSubresource& mapped, Uint32 x, Uint32 y,
                            std::array<std::uint8_t, 3> expected) {
        const auto* pixel =
            static_cast<const std::uint8_t*>(mapped.pData) + y * mapped.Stride + x * 4;
        for (std::size_t channel = 0; channel < 3; ++channel) {
            if (expected[channel] == 255 ? pixel[channel] < 220 : pixel[channel] > 35) {
                return false;
            }
        }
        return true;
    };

    const auto verifyDiligentHumanAsset = [&] {
        HumanMeshGeometry humanGeometry;
        humanGeometry.verticesByMaterial.emplace(
            4, std::vector<ModelVertex>(fullVertices.begin(), fullVertices.end()));
        HumanVertexWeights standingWeights = {};
        standingWeights[humanBoneSlotIndex(HumanBoneSlot::Main)] = 1.0F;
        HumanVertexWeights seatedWeights = {};
        seatedWeights[humanBoneSlotIndex(HumanBoneSlot::Head)] = 1.0F;
        humanGeometry.weightsByMaterial.emplace(
            4, std::vector<HumanVertexWeights>(fullVertices.size(), standingWeights));
        humanGeometry.seatedWeightsByMaterial.emplace(
            4, std::vector<HumanVertexWeights>(fullVertices.size(), seatedWeights));
        std::unordered_map<int, std::shared_ptr<const Image>> humanTextures;
        humanTextures.emplace(4, std::make_shared<Image>(Image{1, 1, {0, 255, 255, 255}}));
        DiligentHumanAsset humanAsset(device, humanGeometry, humanTextures);
        require(humanAsset.submeshes().size() == 1 && humanAsset.submeshes()[0].mesh &&
                    humanAsset.submeshes()[0].seatedMesh && humanAsset.submeshes()[0].texture,
                "Diligent human asset did not retain standing, seated, and texture resources");

        HumanBonePalette humanPose = {};
        for (auto& bone : humanPose) {
            bone = identity;
        }
        HumanBonePalette seatedPose = humanPose;
        seatedPose[humanBoneSlotIndex(HumanBoneSlot::Head)][12] = 2.0;
        const auto& humanSubmesh = humanAsset.submeshes()[0];
        context.SetRenderTargets(1, &targetView, depthView,
                                 RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        context.ClearDepthStencil(depthView, CLEAR_DEPTH_FLAG, 1, 0,
                                  RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        humanSubmesh.draw(renderer, context, *patternView, identity, humanPose, nullptr, false);
        auto humanMapped = readback();
        const bool standingPoseCorrect = pixelIs(humanMapped, 32, 32, {0, 255, 255});
        context.UnmapTextureSubresource(staging, 0, 0);
        require(
            standingPoseCorrect,
            "Diligent human asset did not upload and render its CPU texture and standing weights");

        context.SetRenderTargets(1, &targetView, depthView,
                                 RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        context.ClearDepthStencil(depthView, CLEAR_DEPTH_FLAG, 1, 0,
                                  RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        humanSubmesh.draw(renderer, context, *patternView, identity, seatedPose, nullptr, true);
        humanMapped = readback();
        const bool seatedPoseCorrect = pixelIs(humanMapped, 32, 32, {0, 0, 0});
        context.UnmapTextureSubresource(staging, 0, 0);
        require(seatedPoseCorrect,
                "Diligent human asset did not select its distinct seated skin weights");

        HumanMeshGeometry invalidHumanGeometry = humanGeometry;
        invalidHumanGeometry.weightsByMaterial[4].pop_back();
        bool invalidHumanWeightsRejected = false;
        try {
            DiligentHumanAsset rejectedHumanAsset(device, invalidHumanGeometry, humanTextures);
        } catch (const std::invalid_argument&) {
            invalidHumanWeightsRejected = true;
        }
        require(invalidHumanWeightsRejected,
                "Diligent human asset accepted skin weights with a mismatched vertex count");
    };

    auto mapped = readback();
    const bool patternCorrect =
        pixelIs(mapped, 19, 19, {255, 0, 0}) && pixelIs(mapped, 44, 19, {0, 255, 0}) &&
        pixelIs(mapped, 19, 44, {0, 0, 255}) && pixelIs(mapped, 44, 44, {255, 255, 255});
    const bool depthAndSwitchCorrect = pixelIs(mapped, 32, 32, {255, 0, 255});
    context.UnmapTextureSubresource(staging, 0, 0);
    require(patternCorrect, "Textured mesh UV orientation, interpolation or depth is incorrect");
    require(depthAndSwitchCorrect, "Textured mesh texture switching or depth testing is incorrect");

    const float alphaClear[] = {0.0F, 0.0F, 1.0F, 1.0F};
    context.ClearRenderTarget(targetView, alphaClear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearDepthStencil(depthView, CLEAR_DEPTH_FLAG, 1, 0,
                              RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    DiligentMaterialState bc1AlphaMaterial;
    bc1AlphaMaterial.alphaMode = DiligentAlphaMode::Blend;
    renderer.draw(context, full, *bc1AlphaTexture, identity, bc1AlphaMaterial);
    mapped = readback();
    const bool bc1AlphaCorrect = pixelIs(mapped, 32, 32, {0, 0, 255});
    context.UnmapTextureSubresource(staging, 0, 0);
    require(bc1AlphaCorrect, "Diligent BC1 alpha upload did not preserve transparent selectors");

    verifyDiligentHumanAsset();

    const auto verifyDiligentVehicleAsset = [&] {
        const auto vehicleTexture = std::make_shared<Image>(Image{1, 1, {255, 0, 255, 255}});
        const std::array<DiligentVehicleBatchSource, 2> sources = {{
            {9, fullVertices, vehicleTexture, {}},
            {2, nearVertices, vehicleTexture, {}},
        }};
        DiligentVehicleAsset vehicleAsset(device, sources);
        require(vehicleAsset.submeshes().size() == 2 &&
                    vehicleAsset.submeshes()[0].materialIndex == 9 &&
                    vehicleAsset.submeshes()[1].materialIndex == 2 &&
                    vehicleAsset.submeshes()[0].texture == vehicleAsset.submeshes()[1].texture,
                "Diligent vehicle asset did not preserve batch order or share texture uploads");
        auto mipVehicleTexture =
            std::make_shared<Image>(Image{4, 4, std::vector<std::uint8_t>(4 * 4 * 4, 255)});
        DiligentVehicleBatchSource mipVehicleBatch;
        mipVehicleBatch.materialIndex = 5;
        mipVehicleBatch.vertices = fullVertices;
        mipVehicleBatch.texture = mipVehicleTexture;
        DiligentVehicleAsset mipVehicle(
            device, std::span<const DiligentVehicleBatchSource>(&mipVehicleBatch, 1), true);
        require(mipVehicle.submeshes()[0].texture->shaderResourceView().GetTexture()->GetDesc()
                        .MipLevels == 3,
                "Diligent vehicle asset did not apply its decoded-texture mip option");

        context.SetRenderTargets(1, &targetView, depthView,
                                 RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        context.ClearDepthStencil(depthView, CLEAR_DEPTH_FLAG, 1, 0,
                                  RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        vehicleAsset.submeshes()[1].draw(renderer, context, *patternView, identity, nullptr,
                                         cyanView);
        auto vehicleMapped = readback();
        const bool vehicleTextureCorrect = pixelIs(vehicleMapped, 32, 32, {0, 255, 255});
        context.UnmapTextureSubresource(staging, 0, 0);
        require(vehicleTextureCorrect,
                "Diligent vehicle asset did not render its directly uploaded CPU texture");

        gli::texture2d compressedBaseData(
            gli::FORMAT_RGBA_DXT5_UNORM_BLOCK16, gli::extent2d(4, 4), 1);
        std::memcpy(compressedBaseData.data(0, 0, 0), solidRedBc3Block.data(),
                    solidRedBc3Block.size());
        DiligentVehicleBatchSource compressedBaseBatch;
        compressedBaseBatch.materialIndex = 10;
        compressedBaseBatch.vertices = fullVertices;
        compressedBaseBatch.texture =
            std::make_shared<gli::texture>(compressedBaseData);
        DiligentVehicleAsset compressedBaseVehicle(
            device, std::span<const DiligentVehicleBatchSource>(&compressedBaseBatch, 1));
        require(compressedBaseVehicle.submeshes()[0].texture != nullptr,
                "Diligent vehicle asset did not upload its compressed base texture");
        context.SetRenderTargets(1, &targetView, depthView,
                                 RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        context.ClearDepthStencil(depthView, CLEAR_DEPTH_FLAG, 1, 0,
                                  RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        compressedBaseVehicle.submeshes()[0].draw(renderer, context, *patternView, identity);
        vehicleMapped = readback();
        const bool compressedBaseTextureCorrect = pixelIs(vehicleMapped, 32, 32, {255, 0, 0});
        context.UnmapTextureSubresource(staging, 0, 0);
        require(compressedBaseTextureCorrect,
                "Diligent vehicle asset did not render its compressed base texture");

        DiligentVehicleBatchSource compressedDdsBatch;
        compressedDdsBatch.materialIndex = 12;
        compressedDdsBatch.vertices = fullVertices;
        compressedDdsBatch.texture = std::make_shared<CompressedDds>(compressedRedDds);
        DiligentVehicleAsset compressedDdsVehicle(
            device, std::span<const DiligentVehicleBatchSource>(&compressedDdsBatch, 1));
        context.SetRenderTargets(1, &targetView, depthView,
                                 RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        context.ClearDepthStencil(depthView, CLEAR_DEPTH_FLAG, 1, 0,
                                  RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        compressedDdsVehicle.submeshes()[0].draw(renderer, context, *patternView, identity);
        vehicleMapped = readback();
        const bool compressedDdsTextureCorrect = pixelIs(vehicleMapped, 32, 32, {255, 0, 0});
        context.UnmapTextureSubresource(staging, 0, 0);
        require(compressedDdsTextureCorrect,
                "Diligent vehicle asset did not render its decoded compressed DDS source");

        auto arrayVertices = fullVertices;
        for (ModelVertex& vertex : arrayVertices) {
            vertex.layer = 1.0F;
        }
        const std::array<std::shared_ptr<const Image>, 2> vehicleTextureLayers = {
            std::make_shared<Image>(Image{1, 1, {255, 0, 0, 255}}),
            std::make_shared<Image>(Image{1, 1, {0, 0, 255, 255}}),
        };
        DiligentVehicleBatchSource arrayBatch;
        arrayBatch.materialIndex = 3;
        arrayBatch.vertices = arrayVertices;
        arrayBatch.textureArrayLayers = vehicleTextureLayers;
        auto secondArrayBatch = arrayBatch;
        secondArrayBatch.materialIndex = 4;
        const std::array<DiligentVehicleBatchSource, 2> arraySources = {
            arrayBatch, secondArrayBatch,
        };
        DiligentVehicleAsset arrayVehicle(device, arraySources);
        require(arrayVehicle.submeshes().size() == 2 &&
                    arrayVehicle.submeshes()[0].textureArray != nullptr &&
                    arrayVehicle.submeshes()[0].textureArray ==
                        arrayVehicle.submeshes()[1].textureArray &&
                    arrayVehicle.submeshes()[0].texture == nullptr,
                "Diligent vehicle asset did not share its uploaded texture array");
        context.SetRenderTargets(1, &targetView, depthView,
                                 RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        context.ClearDepthStencil(depthView, CLEAR_DEPTH_FLAG, 1, 0,
                                  RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        arrayVehicle.submeshes()[0].draw(renderer, context, *patternView, identity);
        vehicleMapped = readback();
        const bool vehicleArrayLayerCorrect = pixelIs(vehicleMapped, 32, 32, {0, 0, 255});
        context.UnmapTextureSubresource(staging, 0, 0);
        require(vehicleArrayLayerCorrect,
                "Diligent vehicle asset did not select the vertex's texture-array layer");

        DiligentMaterialState firstLayerReflection;
        firstLayerReflection.arrayReflectionTextures[0] = cyanView;
        context.SetRenderTargets(1, &targetView, depthView,
                                 RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        context.ClearDepthStencil(depthView, CLEAR_DEPTH_FLAG, 1, 0,
                                  RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        renderer.draw(context, full, *textureArrayView, identity, firstLayerReflection);
        vehicleMapped = readback();
        const bool firstArrayReflectionCorrect = pixelIs(vehicleMapped, 32, 32, {0, 255, 255});
        context.UnmapTextureSubresource(staging, 0, 0);
        require(firstArrayReflectionCorrect,
                "Diligent array material did not replace its selected reflection layer");

        DiligentMaterialState wrongLayerReflection;
        wrongLayerReflection.arrayReflectionTextures[0] = cyanView;
        context.SetRenderTargets(1, &targetView, depthView,
                                 RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        context.ClearDepthStencil(depthView, CLEAR_DEPTH_FLAG, 1, 0,
                                  RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        arrayVehicle.submeshes()[0].draw(renderer, context, *patternView, identity,
                                         &wrongLayerReflection);
        vehicleMapped = readback();
        const bool unrelatedArrayLayerPreserved = pixelIs(vehicleMapped, 32, 32, {0, 0, 255});
        context.UnmapTextureSubresource(staging, 0, 0);
        require(unrelatedArrayLayerPreserved,
                "Diligent array reflection override changed an unrelated material layer");

        DiligentMaterialState secondLayerReflection;
        secondLayerReflection.arrayReflectionTextures[1] = cyanView;
        context.SetRenderTargets(1, &targetView, depthView,
                                 RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        context.ClearDepthStencil(depthView, CLEAR_DEPTH_FLAG, 1, 0,
                                  RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        arrayVehicle.submeshes()[0].draw(renderer, context, *patternView, identity,
                                         &secondLayerReflection);
        vehicleMapped = readback();
        const bool secondArrayReflectionCorrect = pixelIs(vehicleMapped, 32, 32, {0, 255, 255});
        context.UnmapTextureSubresource(staging, 0, 0);
        require(secondArrayReflectionCorrect,
                "Diligent array material did not replace a higher reflection layer");

        gli::texture2d_array compressedArrayData(
            gli::FORMAT_RGBA_DXT5_UNORM_BLOCK16, gli::extent2d(4, 4), 2, 2);
        const std::array<std::uint8_t, 16> compressedRedBlock = solidRedBc3Block;
        const std::array<std::uint8_t, 16> compressedBlackBlock = {};
        for (std::size_t mip = 0; mip < compressedArrayData.levels(); ++mip) {
            std::memcpy(compressedArrayData.data(0, 0, mip), compressedBlackBlock.data(),
                        compressedBlackBlock.size());
            std::memcpy(compressedArrayData.data(1, 0, mip), compressedRedBlock.data(),
                        compressedRedBlock.size());
        }
        auto compressedArrayImage = std::make_shared<gli::texture>(compressedArrayData);
        DiligentVehicleBatchSource compressedArrayBatch;
        compressedArrayBatch.materialIndex = 7;
        compressedArrayBatch.vertices = arrayVertices;
        compressedArrayBatch.compressedTextureArray = compressedArrayImage;
        DiligentVehicleAsset compressedArrayVehicle(
            device, std::span<const DiligentVehicleBatchSource>(&compressedArrayBatch, 1));
        require(compressedArrayVehicle.submeshes()[0].textureArray != nullptr &&
                    compressedArrayVehicle.submeshes()[0].textureArray->layerCount() == 2 &&
                    compressedArrayVehicle.submeshes()[0]
                            .textureArray->shaderResourceView()
                            .GetTexture()
                            ->GetDesc()
                            .MipLevels == 2,
                "Diligent vehicle asset did not preserve compressed array layers and mips");
        context.SetRenderTargets(1, &targetView, depthView,
                                 RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        context.ClearDepthStencil(depthView, CLEAR_DEPTH_FLAG, 1, 0,
                                  RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        compressedArrayVehicle.submeshes()[0].draw(renderer, context, *patternView, identity);
        vehicleMapped = readback();
        const bool compressedArrayLayerCorrect = pixelIs(vehicleMapped, 32, 32, {255, 0, 0});
        const bool compressedArraySelectedLayerZero = pixelIs(vehicleMapped, 32, 32, {0, 0, 0});
        const auto* compressedArrayPixel =
            static_cast<const std::uint8_t*>(vehicleMapped.pData) + 32 * vehicleMapped.Stride +
            32 * 4;
        const std::string compressedArrayPixelDescription =
            "Diligent compressed-array pixel was " + std::to_string(compressedArrayPixel[0]) +
            "," + std::to_string(compressedArrayPixel[1]) + "," +
            std::to_string(compressedArrayPixel[2]);
        context.UnmapTextureSubresource(staging, 0, 0);
        require(compressedArrayLayerCorrect,
                compressedArraySelectedLayerZero
                    ? "Diligent vehicle asset selected the wrong compressed array layer"
                    : compressedArrayPixelDescription.c_str());

        gli::texture2d_array bc1AlphaArrayData(
            gli::FORMAT_RGBA_DXT1_UNORM_BLOCK8, gli::extent2d(4, 4), 2, 2);
        const std::array<std::uint8_t, 8> bc1BlackBlock = {};
        const std::array<std::uint8_t, 8> bc1RedBlock = {0, 0xf8, 0, 0, 0, 0, 0, 0};
        for (std::size_t mip = 0; mip < bc1AlphaArrayData.levels(); ++mip) {
            std::memcpy(bc1AlphaArrayData.data(0, 0, mip), bc1BlackBlock.data(),
                        bc1BlackBlock.size());
            std::memcpy(bc1AlphaArrayData.data(1, 0, mip), bc1RedBlock.data(),
                        bc1RedBlock.size());
        }
        auto bc1AlphaArrayImage = std::make_shared<gli::texture>(bc1AlphaArrayData);
        DiligentVehicleBatchSource bc1AlphaArrayBatch;
        bc1AlphaArrayBatch.materialIndex = 8;
        bc1AlphaArrayBatch.vertices = arrayVertices;
        bc1AlphaArrayBatch.compressedTextureArray = bc1AlphaArrayImage;
        DiligentVehicleAsset bc1AlphaArrayVehicle(
            device, std::span<const DiligentVehicleBatchSource>(&bc1AlphaArrayBatch, 1));
        require(bc1AlphaArrayVehicle.submeshes()[0].textureArray != nullptr &&
                    bc1AlphaArrayVehicle.submeshes()[0]
                            .textureArray->shaderResourceView()
                            .GetTexture()
                            ->GetDesc()
                            .Format == TEX_FORMAT_RGBA8_UNORM,
                "Diligent BC1-alpha array upload did not preserve texture alpha");
        context.SetRenderTargets(1, &targetView, depthView,
                                 RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        context.ClearDepthStencil(depthView, CLEAR_DEPTH_FLAG, 1, 0,
                                  RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        bc1AlphaArrayVehicle.submeshes()[0].draw(renderer, context, *patternView, identity);
        vehicleMapped = readback();
        const bool bc1AlphaArrayLayerCorrect = pixelIs(vehicleMapped, 32, 32, {255, 0, 0});
        context.UnmapTextureSubresource(staging, 0, 0);
        require(bc1AlphaArrayLayerCorrect,
                "Diligent vehicle asset did not decode and select a BC1-alpha array layer");

        gli::texture2d_array compressedSrgbArrayData(
            gli::FORMAT_RGBA_DXT5_SRGB_BLOCK16, gli::extent2d(4, 4), 2, 1);
        DiligentTexture2DArray compressedSrgbArray(device, compressedSrgbArrayData);
        require(compressedSrgbArray.shaderResourceView().GetTexture()->GetDesc().Format ==
                    TEX_FORMAT_BC3_UNORM_SRGB,
                "Diligent compressed array upload did not preserve its sRGB format");

        auto vehicleBaseImage = std::make_shared<Image>(Image{1, 1, {255, 255, 255, 255}});
        auto vehicleDetailImage = std::make_shared<Image>(Image{1, 1, {0, 0, 0, 255}});
        DiligentVehicleBatchSource detailBatch;
        detailBatch.materialIndex = 5;
        detailBatch.vertices = fullVertices;
        detailBatch.texture = vehicleBaseImage;
        detailBatch.auxiliaryTextures.detailMap = vehicleDetailImage;
        auto secondDetailBatch = detailBatch;
        secondDetailBatch.materialIndex = 6;
        const std::array<DiligentVehicleBatchSource, 2> detailSources = {
            detailBatch, secondDetailBatch,
        };
        DiligentVehicleAsset detailVehicle(device, detailSources);
        require(detailVehicle.submeshes()[0].auxiliaryTextures.detailMap != nullptr &&
                    detailVehicle.submeshes()[0].auxiliaryTextures.detailMap ==
                        detailVehicle.submeshes()[1].auxiliaryTextures.detailMap &&
                    detailVehicle.submeshes()[0].material.detailMap != nullptr,
                "Diligent vehicle asset did not retain and share auxiliary material textures");
        context.SetRenderTargets(1, &targetView, depthView,
                                 RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        context.ClearDepthStencil(depthView, CLEAR_DEPTH_FLAG, 1, 0,
                                  RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        DiligentMaterialState vehicleRuntimeMaterial;
        detailVehicle.submeshes()[0].draw(renderer, context, *patternView, identity,
                                           &vehicleRuntimeMaterial);
        vehicleMapped = readback();
        const bool vehicleDetailMapCorrect = pixelIs(vehicleMapped, 32, 32, {0, 0, 0});
        context.UnmapTextureSubresource(staging, 0, 0);
        require(vehicleDetailMapCorrect,
                "Diligent vehicle asset did not bind its owned detail map for rendering");

        DiligentVehicleBatchSource compressedDetailBatch;
        compressedDetailBatch.materialIndex = 11;
        compressedDetailBatch.vertices = fullVertices;
        compressedDetailBatch.texture = vehicleBaseImage;
        compressedDetailBatch.auxiliaryTextures.detailMap =
            std::make_shared<gli::texture>(compressedBaseData);
        DiligentVehicleAsset compressedDetailVehicle(
            device, std::span<const DiligentVehicleBatchSource>(&compressedDetailBatch, 1));
        context.SetRenderTargets(1, &targetView, depthView,
                                 RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        context.ClearDepthStencil(depthView, CLEAR_DEPTH_FLAG, 1, 0,
                                  RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        compressedDetailVehicle.submeshes()[0].draw(renderer, context, *patternView, identity);
        vehicleMapped = readback();
        const bool compressedDetailMapCorrect = pixelIs(vehicleMapped, 32, 32, {255, 0, 0});
        context.UnmapTextureSubresource(staging, 0, 0);
        require(compressedDetailMapCorrect,
                "Diligent vehicle asset did not bind its compressed auxiliary detail map");

        auto nearVehicleTexture =
            std::make_shared<Image>(Image{1, 1, {255, 0, 0, 255}});
        auto farVehicleTexture =
            std::make_shared<Image>(Image{1, 1, {0, 0, 255, 255}});
        DiligentMaterialState blendedVehicleMaterial;
        blendedVehicleMaterial.alphaMode = DiligentAlphaMode::Blend;
        blendedVehicleMaterial.baseColor[3] = 0.5F;
        blendedVehicleMaterial.depthMode = DiligentDepthMode::Disabled;
        blendedVehicleMaterial.depthWrite = false;
        const std::array<DiligentVehicleBatchSource, 2> transparentSources = {{
            {9, fullVertices, nearVehicleTexture, blendedVehicleMaterial},
            {2, nearVertices, farVehicleTexture, blendedVehicleMaterial},
        }};
        DiligentVehicleAsset transparentVehicle(device, transparentSources);
        context.SetRenderTargets(1, &targetView, depthView,
                                 RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        context.ClearDepthStencil(depthView, CLEAR_DEPTH_FLAG, 1, 0,
                                  RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        DiligentLightingParameters vehicleLighting;
        vehicleLighting.ambientIntensity = 1.0F;
        vehicleLighting.diffuseIntensity = 0.0F;
        transparentVehicle.draw(renderer, context, *patternView, identity, identity,
                                vehicleLighting);
        vehicleMapped = readback();
        const auto* sortedVehiclePixel =
            static_cast<const std::uint8_t*>(vehicleMapped.pData) + 32 * vehicleMapped.Stride +
            32 * 4;
        const bool transparentVehicleSorted =
            sortedVehiclePixel[0] > sortedVehiclePixel[2] &&
            sortedVehiclePixel[0] > 30 && sortedVehiclePixel[2] > 30;
        context.UnmapTextureSubresource(staging, 0, 0);
        require(transparentVehicleSorted,
                "Diligent vehicle asset did not submit transparent batches back-to-front");

        const std::array<DiligentVehicleBatchSource, 2> overriddenSources = {{
            {9, fullVertices, nearVehicleTexture, {}},
            {2, nearVertices, farVehicleTexture, {}},
        }};
        DiligentVehicleAsset overriddenVehicle(device, overriddenSources);
        const std::array<DiligentVehicleMaterialOverride, 2> blendedOverrides = {{
            {9, &blendedVehicleMaterial, nullptr},
            {2, &blendedVehicleMaterial, nullptr},
        }};
        context.SetRenderTargets(1, &targetView, depthView,
                                 RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        context.ClearDepthStencil(depthView, CLEAR_DEPTH_FLAG, 1, 0,
                                  RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        overriddenVehicle.draw(renderer, context, *patternView, identity, identity,
                               vehicleLighting, nullptr, blendedOverrides);
        vehicleMapped = readback();
        const auto* overriddenVehiclePixel =
            static_cast<const std::uint8_t*>(vehicleMapped.pData) + 32 * vehicleMapped.Stride +
            32 * 4;
        const bool overriddenVehicleSorted =
            overriddenVehiclePixel[0] > overriddenVehiclePixel[2] &&
            overriddenVehiclePixel[0] > 30 && overriddenVehiclePixel[2] > 30;
        context.UnmapTextureSubresource(staging, 0, 0);
        require(overriddenVehicleSorted,
                "Diligent vehicle asset did not sort runtime-blended material overrides");
        DiligentRuntimeTexture2D runtimeVehicleTexture(device);
        runtimeVehicleTexture.synchronize(context, 1, 1, cyan, false, 1);
        const std::array<DiligentVehicleBatchSource, 1> runtimeVehicleSources = {{
            {9, fullVertices, nearVehicleTexture, {}},
        }};
        DiligentVehicleAsset runtimeTextureVehicle(device, runtimeVehicleSources);
        const std::array<DiligentVehicleMaterialOverride, 1> runtimeTextureOverride = {{
            {9, nullptr, nullptr, &runtimeVehicleTexture},
        }};
        context.SetRenderTargets(1, &targetView, depthView,
                                 RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        context.ClearDepthStencil(depthView, CLEAR_DEPTH_FLAG, 1, 0,
                                  RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        runtimeTextureVehicle.draw(renderer, context, *patternView, identity, identity,
                                   vehicleLighting, nullptr, runtimeTextureOverride);
        vehicleMapped = readback();
        const bool runtimeVehicleTextureCorrect =
            pixelIs(vehicleMapped, 32, 32, {0, 255, 255});
        context.UnmapTextureSubresource(staging, 0, 0);
        require(runtimeVehicleTextureCorrect,
                "Diligent vehicle override did not bind its runtime script texture");

        DiligentRuntimeTexture2D runtimeFreeTexture(device);
        runtimeFreeTexture.synchronize(context, 1, 1, cyan, false, 1);
        DiligentVehicleBatchSource runtimeFreeMapBatch;
        runtimeFreeMapBatch.materialIndex = 42;
        runtimeFreeMapBatch.vertices = fullVertices;
        runtimeFreeMapBatch.texture = vehicleBaseImage;
        runtimeFreeMapBatch.animationGroupIndex = 3;
        runtimeFreeMapBatch.sourceBatchIndex = 7;
        DiligentVehicleAsset runtimeFreeMapVehicle(
            device, std::span<const DiligentVehicleBatchSource>(&runtimeFreeMapBatch, 1));
        DiligentMaterialState genericVehicleOverrideMaterial;
        genericVehicleOverrideMaterial.baseColor = {1.0F, 0.0F, 0.0F, 1.0F};
        DiligentVehicleMaterialOverride genericVehicleOverride;
        genericVehicleOverride.materialIndex = 42;
        genericVehicleOverride.material = &genericVehicleOverrideMaterial;
        DiligentVehicleMaterialOverride runtimeFreeMapOverride;
        runtimeFreeMapOverride.materialIndex = 42;
        runtimeFreeMapOverride.animationGroupIndex = 3;
        runtimeFreeMapOverride.sourceBatchIndex = 7;
        runtimeFreeMapOverride.freeMapTexture = &runtimeFreeTexture;
        std::array<DiligentVehicleMaterialOverride, 2> runtimeFreeMapOverrides = {
            genericVehicleOverride, runtimeFreeMapOverride};
        context.SetRenderTargets(1, &targetView, depthView,
                                 RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        context.ClearDepthStencil(depthView, CLEAR_DEPTH_FLAG, 1, 0,
                                  RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        runtimeFreeMapVehicle.draw(renderer, context, *patternView, identity, identity,
                                   vehicleLighting, nullptr, runtimeFreeMapOverrides);
        vehicleMapped = readback();
        const bool runtimeFreeMapCorrect = pixelIs(vehicleMapped, 32, 32, {0, 255, 255});
        context.UnmapTextureSubresource(staging, 0, 0);
        require(runtimeFreeMapCorrect,
                "Diligent vehicle did not route its batch-addressed runtime texture to the free map");

        runtimeFreeMapOverrides[1].freeMapTexture = nullptr;
        runtimeFreeMapOverrides[1].textMapTexture = &runtimeFreeTexture;
        context.SetRenderTargets(1, &targetView, depthView,
                                 RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        context.ClearDepthStencil(depthView, CLEAR_DEPTH_FLAG, 1, 0,
                                  RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        runtimeFreeMapVehicle.draw(renderer, context, *patternView, identity, identity,
                                   vehicleLighting, nullptr, runtimeFreeMapOverrides);
        vehicleMapped = readback();
        const bool runtimeTextMapCorrect = pixelIs(vehicleMapped, 32, 32, {0, 255, 255});
        context.UnmapTextureSubresource(staging, 0, 0);
        require(runtimeTextMapCorrect,
                "Diligent vehicle did not route its batch-addressed runtime texture to the text map");

        DiligentVehicleBatchSource incompleteBatch;
        incompleteBatch.materialIndex = 5;
        incompleteBatch.vertices = std::span{fullVertices}.first(2);
        const std::array<DiligentVehicleBatchSource, 1> invalidSources = {incompleteBatch};
        bool incompleteBatchRejected = false;
        try {
            DiligentVehicleAsset rejectedVehicleAsset(device, invalidSources);
        } catch (const std::invalid_argument&) {
            incompleteBatchRejected = true;
        }
        require(incompleteBatchRejected,
                "Diligent vehicle asset accepted an incomplete triangle batch");
    };
    verifyDiligentVehicleAsset();

    DiligentTexturedMeshRenderer noDepth(device, targetDesc.Format, TEX_FORMAT_UNKNOWN);
    context.SetRenderTargets(1, &targetView, nullptr, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    DiligentMaterialState freeTextureMaterial;
    freeTextureMaterial.freeMap = cyanView;
    noDepth.draw(context, full, *magentaView, identity, freeTextureMaterial);
    mapped = readback();
    const bool freeTextureOverlayCorrect = pixelIs(mapped, 32, 32, {0, 0, 255});
    context.UnmapTextureSubresource(staging, 0, 0);
    require(freeTextureOverlayCorrect,
            "Diligent free texture did not multiply the base texture using scaled UVs");

    context.SetRenderTargets(1, &targetView, nullptr, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    DiligentMaterialState textTextureMaterial;
    textTextureMaterial.alphaMode = DiligentAlphaMode::Blend;
    textTextureMaterial.textMap = cyanView;
    noDepth.draw(context, full, *magentaView, identity, textTextureMaterial);
    mapped = readback();
    const bool textTextureOverlayCorrect = pixelIs(mapped, 32, 32, {0, 0, 255});
    context.UnmapTextureSubresource(staging, 0, 0);
    require(textTextureOverlayCorrect,
            "Diligent text texture did not multiply the base texture using raw mesh UVs");

    context.SetRenderTargets(1, &targetView, depthView, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearDepthStencil(depthView, CLEAR_DEPTH_FLAG, 1, 0,
                              RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    textTextureMaterial.textMap = transparentView;
    renderer.draw(context, full, *magentaView, identity, textTextureMaterial);
    renderer.draw(context, far, *cyanView, identity);
    mapped = readback();
    const bool transparentTextDiscarded = pixelIs(mapped, 32, 32, {0, 255, 255});
    context.UnmapTextureSubresource(staging, 0, 0);
    require(transparentTextDiscarded,
            "Diligent transparent text pixels incorrectly occluded later geometry");

    context.SetRenderTargets(1, &targetView, nullptr, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    freeTextureMaterial.weatherLitDynamicTexture = true;
    freeTextureMaterial.dynamicTextureBrightness = 0.5F;
    noDepth.draw(context, full, *magentaView, identity, freeTextureMaterial);
    mapped = readback();
    const auto* halfLitBlue =
        static_cast<const std::uint8_t*>(mapped.pData) + 32 * mapped.Stride + 32 * 4;
    const bool dynamicWeatherBrightnessCorrect =
        halfLitBlue[0] < 35 && halfLitBlue[1] < 35 && halfLitBlue[2] > 105 && halfLitBlue[2] < 150;
    context.UnmapTextureSubresource(staging, 0, 0);
    require(dynamicWeatherBrightnessCorrect,
            "Diligent dynamic texture did not apply caller-provided weather brightness");

    auto reversedVertices = fullVertices;
    std::swap(reversedVertices[1], reversedVertices[2]);
    std::swap(reversedVertices[4], reversedVertices[5]);
    DiligentTexturedMesh reversedMesh(device, reversedVertices);
    DiligentMaterialState culledMaterial;
    culledMaterial.cullBackFaces = true;
    context.SetRenderTargets(1, &targetView, nullptr, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    noDepth.draw(context, reversedMesh, *magentaView, identity, culledMaterial);
    mapped = readback();
    const bool frontFaceRendered = pixelIs(mapped, 32, 32, {255, 0, 255});
    context.UnmapTextureSubresource(staging, 0, 0);
    require(frontFaceRendered, "Diligent scenery culling rejected a clockwise front face");
    context.SetRenderTargets(1, &targetView, nullptr, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    noDepth.draw(context, full, *magentaView, identity, culledMaterial);
    mapped = readback();
    const bool backFaceDiscarded = pixelIs(mapped, 32, 32, {0, 0, 0});
    context.UnmapTextureSubresource(staging, 0, 0);
    require(backFaceDiscarded, "Diligent scenery culling did not reject the back face");
    culledMaterial.cullBackFaces = false;
    context.SetRenderTargets(1, &targetView, nullptr, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    noDepth.draw(context, full, *magentaView, identity, culledMaterial);
    mapped = readback();
    const bool twoSidedRendered = pixelIs(mapped, 32, 32, {255, 0, 255});
    context.UnmapTextureSubresource(staging, 0, 0);
    require(twoSidedRendered, "Diligent scenery could not disable culling for a mirrored view");

    const auto clearDepthScene = [&]() {
        context.SetRenderTargets(1, &targetView, depthView,
                                 RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        context.ClearDepthStencil(depthView, CLEAR_DEPTH_FLAG, 1.0F, 0,
                                  RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    };
    clearDepthScene();
    renderer.draw(context, near, *magentaView, identity);
    renderer.draw(context, near, *cyanView, identity);
    mapped = readback();
    const bool lessRejectedEqual = pixelIs(mapped, 32, 32, {255, 0, 255});
    context.UnmapTextureSubresource(staging, 0, 0);
    require(lessRejectedEqual, "Diligent less depth test accepted equal depth");
    DiligentMaterialState lessEqual;
    lessEqual.depthMode = DiligentDepthMode::LessEqual;
    context.SetRenderTargets(1, &targetView, depthView, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    renderer.draw(context, near, *cyanView, identity, lessEqual);
    mapped = readback();
    const bool lessEqualAcceptedEqual = pixelIs(mapped, 32, 32, {0, 255, 255});
    context.UnmapTextureSubresource(staging, 0, 0);
    require(lessEqualAcceptedEqual, "Diligent less-equal depth test rejected equal depth");
    clearDepthScene();
    renderer.draw(context, near, *magentaView, identity);
    DiligentMaterialState noDepthTest;
    noDepthTest.depthMode = DiligentDepthMode::Disabled;
    renderer.draw(context, far, *cyanView, identity, noDepthTest);
    renderer.draw(context, full, *magentaView, identity);
    mapped = readback();
    const bool disabledDidNotWriteDepth = pixelIs(mapped, 32, 32, {0, 255, 255});
    context.UnmapTextureSubresource(staging, 0, 0);
    require(disabledDidNotWriteDepth, "Diligent disabled depth test still wrote depth");
    bool invalidDepthModeRejected = false;
    try {
        DiligentMaterialState invalidDepth;
        invalidDepth.depthMode = static_cast<DiligentDepthMode>(255);
        noDepth.draw(context, full, *magentaView, identity, invalidDepth);
    } catch (const std::invalid_argument&) {
        invalidDepthModeRejected = true;
    }
    require(invalidDepthModeRejected, "Diligent accepted an invalid depth mode");

    std::filesystem::create_directories(textureDirectory / "model");
    {
        std::ofstream meshFile(textureDirectory / "model" / "prepared.x", std::ios::binary);
        meshFile << "xof 0303txt 0032\n"
                    "Mesh { 3; -0.8;-0.5;-0.8;, 0.8;-0.5;-0.8;, 0;-0.5;0.8;; "
                    "1; 3;0,1,2;;\n"
                    "MeshTextureCoords { 3; 0.25;0.25;, 0.25;0.25;, 0.25;0.25;; }\n"
                    "MeshMaterialList { 1; 1; 0;; Material { "
                    "1;1;1;1;; 8; 0;0;0;; 0;0;0;; "
                    "TextureFilename { \"decoded-pattern.tga\"; } } } }\n";
        require(static_cast<bool>(meshFile), "Failed to write prepared scenery mesh");
        std::ofstream configFile(textureDirectory / "prepared.sco", std::ios::binary);
        configFile << "[mesh]\nprepared.x\n[matl]\ndecoded-pattern.tga\n0\n"
                      "[matl_alpha]\n1\n[matl_texadress_clamp]\n";
        require(static_cast<bool>(configFile), "Failed to write prepared scenery configuration");
    }
    SharedMapTextureImages sharedSceneryTextures;
    PreparedSceneryAsset preparedScenery = prepareSceneryAsset(
        textureDirectory / "prepared.sco", textureDirectory, sharedSceneryTextures);
    require(preparedScenery.batches.size() == 1 && preparedScenery.decodedTextures.size() == 1,
            "CPU scenery preparation did not retain the mesh material and decoded texture");
    const PreparedSceneryBatch& preparedBatch = preparedScenery.batches.front();
    require(preparedBatch.batch.alphaMode == 1 &&
                preparedBatch.batch.material.addressU == TextureAddressMode::Clamp &&
                preparedBatch.batch.material.addressV == TextureAddressMode::Clamp,
            "CPU scenery preparation lost authored alpha or texture addressing");
    const auto preparedMesh = preparedScenery.meshes.find(preparedBatch.meshKey);
    require(preparedMesh != preparedScenery.meshes.end(),
            "CPU scenery preparation did not retain its source mesh");
    const auto preparedVertices =
        preparedMesh->second.verticesByMaterial.find(preparedBatch.materialIndex);
    require(preparedVertices != preparedMesh->second.verticesByMaterial.end() &&
                preparedVertices->second.size() == 3,
            "CPU scenery preparation did not retain its material geometry");
    for (auto& vertex : preparedVertices->second) {
        vertex.r = 0.0F;
        vertex.g = 1.0F;
        vertex.b = 0.0F;
        vertex.a = 0.1F;
    }
    const auto preparedTexturePath = preparedBatch.texturePath;
    preparedScenery.batches.push_back(preparedScenery.batches.front());
    auto& secondSceneryBatch = preparedScenery.batches.back();
    secondSceneryBatch.transmapTextureName = "decoded-pattern.tga";
    secondSceneryBatch.transmapTexturePath = preparedTexturePath;
    secondSceneryBatch.freeTextureName = "decoded-pattern.tga";
    secondSceneryBatch.freeTexturePath = preparedTexturePath;
    secondSceneryBatch.lightmap = {"decoded-pattern.tga", preparedTexturePath, {}};
    secondSceneryBatch.nightmap = secondSceneryBatch.lightmap;
    secondSceneryBatch.bumpmap = secondSceneryBatch.lightmap;
    secondSceneryBatch.environment = secondSceneryBatch.lightmap;
    secondSceneryBatch.itemLightmaps.push_back(secondSceneryBatch.lightmap);
    secondSceneryBatch.itemNightmaps.push_back({"missing.tga", {}, "test decoding failure"});
    secondSceneryBatch.batch.lightmapStrengthVariable = "lamp";
    secondSceneryBatch.batch.texcoordTransXVariable = "scroll";
    secondSceneryBatch.batch.alphaScaleVariable = "opacity";
    ModelMaterialState runtimeSelection;
    ModelMaterialState::TextureChange runtimeChange;
    runtimeChange.activationVariable = "variant";
    auto runtimeItem = std::make_shared<ModelMaterialState>();
    runtimeItem->lightmapStrengthVariable = "item_lamp";
    runtimeChange.items.push_back(runtimeItem);
    runtimeSelection.textureChanges.push_back(runtimeChange);
    secondSceneryBatch.batch.auxiliaryMaterialSelection = std::move(runtimeSelection);
    preparedScenery.modelConfiguration.lodThresholds = {1.0, 0.1};
    DiligentSceneryAsset directScenery(device, preparedScenery);
    require(directScenery.batches().size() == 2 && directScenery.selectLod(20.0) == 0 &&
                directScenery.selectLod(30.0) == 1,
            "Diligent scenery upload did not retain both material batches");
    auto distantSceneryModelView = identityMatrix();
    distantSceneryModelView[14] = -30.0;
    require(openbus::app::selectDiligentMapSceneryLod(directScenery,
                                                      distantSceneryModelView) == 1,
            "Diligent map scenery did not select the production far LOD");
    openbus::app::DiligentMapSceneryPlacement fadePlacement;
    fadePlacement.motionObjectId = 1;
    fadePlacement.worldPosition = {0.0, 0.0, 0.0};
    fadePlacement.boundingRadius = 1.0;
    auto fadeView = identityMatrix();
    fadeView[14] = -20.0;
    const auto fadeProjection = identityMatrix();
    require(openbus::app::diligentMapSceneryVisibilityFade(
                fadePlacement, fadeView, fadeProjection, 0.2) == 0.0F,
            "Diligent map scenery remained visible below the projected-size fade range");
    fadeView[14] = -5.0;
    require(openbus::app::diligentMapSceneryVisibilityFade(
                fadePlacement, fadeView, fadeProjection, 0.2) == 1.0F,
            "Diligent map scenery was faded above the projected-size fade range");
    require(std::abs(openbus::rendering::configuredReflectionAmbientFloor() - 0.37F) < 1.0e-6F,
            "Diligent reflection configuration did not expose the shared OpenGL ambient floor");
    const auto fadedShadowCasters = openbus::app::buildDiligentMapSceneryShadowCasters(
        fadePlacement, directScenery, identityMatrix(), identityMatrix(), {}, {}, 0.0F);
    require(fadedShadowCasters.empty(),
            "Diligent scenery shadow casters ignored a fully faded placement");
    auto nearOpaqueShadowSource = preparedScenery;
    for (auto& batch : nearOpaqueShadowSource.batches) {
        batch.batch.alphaMode = 3;
    }
    DiligentSceneryAsset nearOpaqueShadowAsset(device, nearOpaqueShadowSource);
    const auto nearOpaqueShadowCasters = openbus::app::buildDiligentMapSceneryShadowCasters(
        fadePlacement, nearOpaqueShadowAsset, identityMatrix(), identityMatrix());
    require(!nearOpaqueShadowCasters.empty() &&
                std::all_of(nearOpaqueShadowCasters.begin(), nearOpaqueShadowCasters.end(),
                            [](const auto& caster) {
                                return caster.alphaTexture != nullptr &&
                                       std::abs(caster.alphaCutoff - 0.99F) < 1.0e-6F;
                            }),
            "Diligent scenery shadows omitted near-opaque alpha-cutout casters");
    auto noDepthShadowSource = nearOpaqueShadowSource;
    for (auto& batch : noDepthShadowSource.batches) {
        batch.batch.noZwrite = true;
    }
    DiligentSceneryAsset noDepthShadowAsset(device, noDepthShadowSource);
    require(openbus::app::buildDiligentMapSceneryShadowCasters(
                fadePlacement, noDepthShadowAsset, identityMatrix(), identityMatrix())
                .empty(),
            "Diligent scenery shadow pass included no-Z-write batches");
    DiligentSceneryAsset mipmappedScenery(device, preparedScenery, true);
    DiligentSceneryAssetCache sceneryCache(device);
    const auto preparedSceneryPath = textureDirectory / "prepared.sco";
    require(sceneryCache.request(preparedSceneryPath, textureDirectory) &&
                sceneryCache.find(preparedSceneryPath, textureDirectory) == nullptr,
            "Diligent scenery cache did not queue CPU preparation asynchronously");
    const auto sceneryDeadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (sceneryCache.find(preparedSceneryPath, textureDirectory) == nullptr &&
           std::chrono::steady_clock::now() < sceneryDeadline) {
        sceneryCache.publishReady();
        if (sceneryCache.find(preparedSceneryPath, textureDirectory) == nullptr) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    const DiligentSceneryAsset* cachedScenery =
        sceneryCache.find(preparedSceneryPath, textureDirectory);
    require(cachedScenery != nullptr && cachedScenery->batches().size() == 1 &&
                sceneryCache.assetCount() == 1 &&
                !sceneryCache.request(preparedSceneryPath, textureDirectory),
            "Diligent scenery cache did not publish and reuse its GPU asset");
    require(sceneryCache.requestSeasonalTexture(imagePath),
            "Diligent scenery cache did not queue a seasonal texture");
    const auto seasonalDeadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (sceneryCache.seasonalTexture(imagePath) == nullptr &&
           std::chrono::steady_clock::now() < seasonalDeadline) {
        sceneryCache.publishReady();
        if (sceneryCache.seasonalTexture(imagePath) == nullptr) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    const DiligentTexture2D* seasonalTexture = sceneryCache.seasonalTexture(imagePath);
    require(seasonalTexture != nullptr &&
                seasonalTexture->shaderResourceView().GetTexture()->GetDesc().Width > 0 &&
                !sceneryCache.requestSeasonalTexture(imagePath),
            "Diligent scenery cache did not upload and reuse its seasonal texture");
    const auto missingSceneryPath = textureDirectory / "missing-prepared.sco";
    require(sceneryCache.request(missingSceneryPath, textureDirectory),
            "Diligent scenery cache did not queue a missing asset for failure reporting");
    bool sceneryPreparationFailureSurfaced = false;
    const auto sceneryFailureDeadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (!sceneryPreparationFailureSurfaced &&
           std::chrono::steady_clock::now() < sceneryFailureDeadline) {
        try {
            sceneryCache.publishReady();
        } catch (const std::exception&) {
            sceneryPreparationFailureSurfaced = true;
        }
        if (!sceneryPreparationFailureSurfaced) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    require(sceneryPreparationFailureSurfaced && sceneryCache.assetCount() == 1,
            "Diligent scenery cache did not surface worker failures without caching a fake asset");
    const std::array<std::pair<std::filesystem::path, std::filesystem::path>, 1>
        retainedSceneryAssets = {{{preparedSceneryPath, textureDirectory}}};
    const std::array<std::filesystem::path, 1> retainedSeasonalTextures = {imagePath};
    require(sceneryCache.retain(retainedSceneryAssets, retainedSeasonalTextures) == 0 &&
                sceneryCache.find(preparedSceneryPath, textureDirectory) == cachedScenery &&
                sceneryCache.seasonalTexture(imagePath) == seasonalTexture,
            "Diligent scenery retention removed an asset still in use");
    require(sceneryCache.retain({}, {}) == 2 && sceneryCache.assetCount() == 0 &&
                sceneryCache.find(preparedSceneryPath, textureDirectory) == nullptr &&
                sceneryCache.seasonalTexture(imagePath) == nullptr,
            "Diligent scenery retention failed to release stale GPU assets and seasonal textures");
    bool unretainedSceneryRejected = false;
    bool unretainedSeasonalTextureRejected = false;
    try {
        sceneryCache.request(preparedSceneryPath, textureDirectory);
    } catch (const std::logic_error&) {
        unretainedSceneryRejected = true;
    }
    try {
        sceneryCache.requestSeasonalTexture(imagePath);
    } catch (const std::logic_error&) {
        unretainedSeasonalTextureRejected = true;
    }
    require(unretainedSceneryRejected && unretainedSeasonalTextureRejected,
            "Diligent scenery cache accepted requests outside its retained set");
    require(directScenery.batches()[0].textures.base->shaderResourceView()
                    .GetTexture()->GetDesc().MipLevels == 1 &&
                mipmappedScenery.batches()[0].textures.base->shaderResourceView()
                        .GetTexture()->GetDesc().MipLevels == 2,
            "Diligent scenery asset did not preserve its default and enabled mip options");
    openbus::scripting::SceneryObject sceneryVariables;
    sceneryVariables.set("lamp", 0.5F);
    sceneryVariables.set("scroll", 0.25F);
    sceneryVariables.set("opacity", 0.75F);
    const auto resolvedScenery = directScenery.batches()[1].resolveRuntime(&sceneryVariables, 0.8);
    require(resolvedScenery.lightmapStrength == 0.5F && resolvedScenery.texcoordOffsetX == 0.25F &&
                resolvedScenery.alphaScale == 0.75 &&
                std::abs(resolvedScenery.nightmapStrength - 0.2F) < 0.0001F,
            "Diligent scenery runtime material did not resolve script and weather state");
    sceneryVariables.set("variant", 1.0F);
    sceneryVariables.set("item_lamp", 0.25F);
    const auto resolvedItem = directScenery.batches()[1].resolveRuntime(&sceneryVariables, 0.8);
    require(resolvedItem.selectedItem == runtimeItem.get() &&
                resolvedItem.lightmapStrength == 0.25F && resolvedItem.nightmapStrength == 1.0F,
            "Diligent scenery runtime material did not resolve selected item controls");
    const auto& auxiliary = directScenery.batches()[1].textures;
    require(directScenery.batches()[0].mesh == directScenery.batches()[1].mesh &&
                directScenery.batches()[0].textures.base == auxiliary.base &&
                auxiliary.base == auxiliary.transmap && auxiliary.base == auxiliary.lightmap &&
                auxiliary.base == auxiliary.nightmap && auxiliary.base == auxiliary.bumpmap &&
                auxiliary.base == auxiliary.environment &&
                auxiliary.base == auxiliary.freeTexture && auxiliary.itemLightmaps.size() == 1 &&
                auxiliary.base == auxiliary.itemLightmaps[0] &&
                auxiliary.itemNightmaps.size() == 1 && !auxiliary.itemNightmaps[0] &&
                directScenery.diagnostics().size() == 1 &&
                directScenery.diagnostics()[0].find("test decoding failure") != std::string::npos,
            "Diligent scenery upload did not share resources or expose missing textures");
    DiligentSceneryOpaqueDraw sceneryDraw;
    sceneryDraw.reverseWinding = true;
    context.SetRenderTargets(1, &targetView, nullptr, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    require(directScenery.batches()[0].drawOpaque(noDepth, context, *magentaView, identity,
                                                  sceneryDraw),
            "Diligent opaque scenery submission skipped a visible batch");
    mapped = readback();
    const bool directSceneryCorrect = pixelIs(mapped, 32, 32, {255, 0, 0});
    const auto* directSceneryPixel =
        static_cast<const std::uint8_t*>(mapped.pData) + 32 * mapped.Stride + 32 * 4;
    const std::array<std::uint8_t, 4> directSceneryColor = {
        directSceneryPixel[0], directSceneryPixel[1], directSceneryPixel[2], directSceneryPixel[3]};
    context.UnmapTextureSubresource(staging, 0, 0);
    if (!directSceneryCorrect) {
        throw std::runtime_error(
            "Diligent direct scenery color: " + std::to_string(directSceneryColor[0]) + "," +
            std::to_string(directSceneryColor[1]) + "," + std::to_string(directSceneryColor[2]) +
            "," + std::to_string(directSceneryColor[3]));
    }
    sceneryDraw.sceneryLodIndex = 1;
    auto farLodBatch = directScenery.batches()[0];
    farLodBatch.source.batch.sceneryLodIndex = 1;
    require(!directScenery.batches()[0].drawOpaque(noDepth, context, *magentaView, identity,
                                                   sceneryDraw) &&
                farLodBatch.drawOpaque(noDepth, context, *magentaView, identity, sceneryDraw),
            "Diligent scenery submitted batches from an unselected LOD");
    sceneryDraw.sceneryLodIndex.reset();
    sceneryDraw.seasonalTexture = cyanView;
    context.SetRenderTargets(1, &targetView, nullptr, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    require(directScenery.batches()[0].drawOpaque(noDepth, context, *magentaView, identity,
                                                  sceneryDraw),
            "Diligent scenery did not submit the seasonal override");
    mapped = readback();
    const bool seasonalSceneryCorrect = pixelIs(mapped, 32, 32, {0, 255, 255});
    context.UnmapTextureSubresource(staging, 0, 0);
    require(seasonalSceneryCorrect, "Diligent scenery did not bind the seasonal override");
    sceneryDraw.seasonalTexture = nullptr;
    sceneryDraw.visibilityFade = 0.0F;
    require(!directScenery.batches()[0].drawOpaque(noDepth, context, *magentaView, identity,
                                                   sceneryDraw),
            "Diligent scenery submitted a fully faded batch");
    sceneryDraw.visibilityFade = 0.5F;
    require(!directScenery.batches()[0].drawOpaque(noDepth, context, *magentaView, identity,
                                                   sceneryDraw) &&
                directScenery.batches()[0].drawTransparent(noDepth, context, *magentaView, identity,
                                                           sceneryDraw),
            "Diligent partially faded opaque scenery was not routed to the blended pass");
    sceneryDraw.visibilityFade = std::numeric_limits<float>::quiet_NaN();
    bool invalidSceneryFadeRejected = false;
    try {
        directScenery.batches()[0].drawOpaque(noDepth, context, *magentaView, identity,
                                              sceneryDraw);
    } catch (const std::invalid_argument&) {
        invalidSceneryFadeRejected = true;
    }
    require(invalidSceneryFadeRejected, "Diligent scenery accepted a non-finite visibility fade");
    sceneryDraw.visibilityFade = 1.0F;
    auto depthScenery = directScenery.batches()[0];
    depthScenery.source.batch.alphaMode = 0;
    const auto clearSceneryDepth = [&] {
        context.SetRenderTargets(1, &targetView, depthView,
                                 RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        context.ClearDepthStencil(depthView, CLEAR_DEPTH_FLAG, 1.0F, 0,
                                  RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    };
    clearSceneryDepth();
    renderer.draw(context, near, *cyanView, identity);
    depthScenery.source.batch.noZcheck = true;
    require(depthScenery.drawOpaque(renderer, context, *magentaView, identity, sceneryDraw),
            "Diligent noZcheck scenery was not submitted");
    mapped = readback();
    const bool disabledSceneryDepth = pixelIs(mapped, 32, 32, {255, 0, 0});
    context.UnmapTextureSubresource(staging, 0, 0);
    require(disabledSceneryDepth, "Diligent scenery noZcheck did not disable depth testing");
    clearSceneryDepth();
    renderer.draw(context, near, *cyanView, identity);
    depthScenery.source.batch.noZwrite = true;
    require(depthScenery.drawOpaque(renderer, context, *magentaView, identity, sceneryDraw),
            "Diligent noZwrite scenery was not submitted");
    mapped = readback();
    const bool overlayRetainedDepthTest = pixelIs(mapped, 32, 32, {0, 255, 255});
    context.UnmapTextureSubresource(staging, 0, 0);
    require(overlayRetainedDepthTest,
            "Diligent noZcheck/noZwrite overlay ignored foreground depth");
    clearSceneryDepth();
    require(depthScenery.drawOpaque(renderer, context, *magentaView, identity, sceneryDraw),
            "Diligent noZwrite scenery was not submitted on a clear target");
    renderer.draw(context, far, *cyanView, identity);
    mapped = readback();
    const bool sceneryDidNotWriteDepth = pixelIs(mapped, 32, 32, {0, 255, 255});
    context.UnmapTextureSubresource(staging, 0, 0);
    require(sceneryDidNotWriteDepth, "Diligent scenery noZwrite still wrote depth");
    clearSceneryDepth();
    depthScenery.source.batch.noZcheck = false;
    depthScenery.source.batch.noZwrite = false;
    require(depthScenery.drawOpaque(renderer, context, *magentaView, identity, sceneryDraw),
            "Diligent depth-writing scenery was not submitted");
    renderer.draw(context, far, *cyanView, identity);
    mapped = readback();
    const bool sceneryWroteDepth = pixelIs(mapped, 32, 32, {255, 0, 0});
    context.UnmapTextureSubresource(staging, 0, 0);
    require(sceneryWroteDepth, "Diligent opaque scenery did not write depth");
    auto runtimeScenery = directScenery.batches()[1];
    runtimeScenery.textures.freeTexture.reset();
    openbus::scripting::SceneryObject transparentSceneryVariables;
    transparentSceneryVariables.set("opacity", 1.0F);
    sceneryDraw.variables = &transparentSceneryVariables;
    runtimeScenery.source.batch.alphaMode = 2;
    runtimeScenery.source.batch.alpha = 0.5;
    const float sceneryBlueBackground[] = {0.0F, 0.0F, 1.0F, 1.0F};
    context.SetRenderTargets(1, &targetView, nullptr, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(targetView, sceneryBlueBackground,
                              RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    require(runtimeScenery.drawTransparent(noDepth, context, *magentaView, identity, sceneryDraw),
            "Diligent transparent scenery did not submit");
    mapped = readback();
    const auto* blendedSceneryPixel =
        static_cast<const std::uint8_t*>(mapped.pData) + 32 * mapped.Stride + 32 * 4;
    const std::array<std::uint8_t, 3> blendedSceneryColor = {
        blendedSceneryPixel[0], blendedSceneryPixel[1], blendedSceneryPixel[2]};
    const bool sceneryBlendCorrect = blendedSceneryColor[0] >= 110 &&
                                     blendedSceneryColor[0] <= 145 &&
                                     blendedSceneryColor[2] >= 110 && blendedSceneryColor[2] <= 145;
    context.UnmapTextureSubresource(staging, 0, 0);
    if (!sceneryBlendCorrect) {
        throw std::runtime_error("Diligent transparent scenery did not alpha blend: " +
                                 std::to_string(blendedSceneryColor[0]) + "," +
                                 std::to_string(blendedSceneryColor[1]) + "," +
                                 std::to_string(blendedSceneryColor[2]));
    }
    sceneryDraw.visibilityFade = 0.5F;
    context.SetRenderTargets(1, &targetView, nullptr, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(targetView, sceneryBlueBackground,
                              RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    require(runtimeScenery.drawTransparent(noDepth, context, *magentaView, identity, sceneryDraw),
            "Diligent partially faded transparent scenery did not submit");
    mapped = readback();
    const auto* fadedSceneryPixel =
        static_cast<const std::uint8_t*>(mapped.pData) + 32 * mapped.Stride + 32 * 4;
    const bool sceneryFadeCorrect = fadedSceneryPixel[0] >= 50 && fadedSceneryPixel[0] <= 80 &&
                                    fadedSceneryPixel[2] >= 175 && fadedSceneryPixel[2] <= 205;
    context.UnmapTextureSubresource(staging, 0, 0);
    require(sceneryFadeCorrect, "Diligent transparent scenery did not apply visibility fade");
    sceneryDraw.visibilityFade = 1.0F;
    runtimeScenery.source.batch.alphaMode = 3;
    runtimeScenery.source.batch.alpha = 0.995;
    context.SetRenderTargets(1, &targetView, nullptr, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(targetView, sceneryBlueBackground,
                              RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    require(runtimeScenery.drawTransparent(noDepth, context, *magentaView, identity, sceneryDraw),
            "Diligent near-opaque scenery did not submit");
    mapped = readback();
    const auto* nearOpaquePixel =
        static_cast<const std::uint8_t*>(mapped.pData) + 32 * mapped.Stride + 32 * 4;
    const bool nearOpaqueBlended = nearOpaquePixel[0] >= 240 && nearOpaquePixel[2] <= 20;
    context.UnmapTextureSubresource(staging, 0, 0);
    require(nearOpaqueBlended, "Diligent near-opaque scenery did not blend after cutout");
    runtimeScenery.source.batch.alpha = 0.98;
    context.SetRenderTargets(1, &targetView, nullptr, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(targetView, sceneryBlueBackground,
                              RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    require(runtimeScenery.drawTransparent(noDepth, context, *magentaView, identity, sceneryDraw),
            "Diligent cutout scenery did not submit");
    mapped = readback();
    const bool nearOpaqueDiscarded = pixelIs(mapped, 32, 32, {0, 0, 255});
    context.UnmapTextureSubresource(staging, 0, 0);
    require(nearOpaqueDiscarded,
            "Diligent near-opaque scenery did not discard sub-threshold alpha");
    bool opaqueSceneryInTransparentPassRejected = false;
    try {
        directScenery.batches()[0].drawTransparent(noDepth, context, *magentaView, identity,
                                                   sceneryDraw);
    } catch (const std::invalid_argument&) {
        opaqueSceneryInTransparentPassRejected = true;
    }
    require(opaqueSceneryInTransparentPassRejected,
            "Diligent transparent pass accepted an opaque scenery batch");

    DiligentSceneryBatch transmapDepthScenery = directScenery.batches()[0];
    transmapDepthScenery.source.batch.alphaMode = 2;
    transmapDepthScenery.textures.transmap = transmapDepthScenery.textures.base;
    clearSceneryDepth();
    require(transmapDepthScenery.drawTransmapDepthPrepass(renderer, context, *magentaView, identity,
                                                          sceneryDraw),
            "Diligent transmap depth prepass did not submit");
    renderer.draw(context, far, *cyanView, identity);
    mapped = readback();
    const bool depthOnlyPrepass = pixelIs(mapped, 32, 32, {0, 0, 0});
    context.UnmapTextureSubresource(staging, 0, 0);
    require(depthOnlyPrepass, "Diligent transmap prepass wrote color or omitted opaque depth");
    const Image transparentTransmapImage{1, 1, {255, 255, 255, 0}};
    transmapDepthScenery.textures.transmap =
        std::make_shared<DiligentTexture2D>(device, transparentTransmapImage);
    clearSceneryDepth();
    require(transmapDepthScenery.drawTransmapDepthPrepass(renderer, context, *magentaView, identity,
                                                          sceneryDraw),
            "Diligent transparent texels did not submit to the transmap prepass");
    renderer.draw(context, far, *cyanView, identity);
    mapped = readback();
    const bool transparentTexelDidNotWriteDepth = pixelIs(mapped, 32, 32, {0, 255, 255});
    context.UnmapTextureSubresource(staging, 0, 0);
    require(transparentTexelDidNotWriteDepth,
            "Diligent transparent transmap texel incorrectly occluded later geometry");

    runtimeScenery.source.batch.alphaMode = 1;
    runtimeScenery.source.batch.alpha = 1.0;
    runtimeScenery.source.batch.freeTextureVariable = "free_texture";
    bool dynamicSceneryRejected = false;
    try {
        runtimeScenery.drawOpaque(noDepth, context, *magentaView, identity, sceneryDraw);
    } catch (const std::logic_error&) {
        dynamicSceneryRejected = true;
    }
    require(dynamicSceneryRejected,
            "Diligent scenery did not report a missing runtime texture view");
    const Image magentaImage{1, 1, {255, 0, 255, 255}};
    runtimeScenery.textures.base = std::make_shared<DiligentTexture2D>(device, magentaImage);
    DiligentDynamicTexture2D sceneryDynamicTexture(device, context, 1, 1, cyan);
    sceneryDraw.dynamicTexture = &sceneryDynamicTexture.shaderResourceView();
    context.SetRenderTargets(1, &targetView, nullptr, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    require(runtimeScenery.drawOpaque(noDepth, context, *magentaView, identity, sceneryDraw),
            "Diligent scenery did not submit with its runtime free texture");
    mapped = readback();
    const bool dynamicSceneryTextureRendered = pixelIs(mapped, 32, 32, {0, 0, 255});
    context.UnmapTextureSubresource(staging, 0, 0);
    require(dynamicSceneryTextureRendered,
            "Diligent scenery did not multiply its base texture by its runtime free texture");

    const std::array<std::uint8_t, 4> whiteDynamicPixel = {255, 255, 255, 255};
    auto* sceneryDynamicView = &sceneryDynamicTexture.shaderResourceView();
    sceneryDynamicTexture.update(context, 1, 1, whiteDynamicPixel);
    require(&sceneryDynamicTexture.shaderResourceView() == sceneryDynamicView,
            "Same-size Diligent scenery texture refresh replaced its shader view");
    context.SetRenderTargets(1, &targetView, nullptr, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    require(runtimeScenery.drawOpaque(noDepth, context, *magentaView, identity, sceneryDraw),
            "Diligent scenery did not submit after refreshing its runtime texture");
    mapped = readback();
    const bool refreshedDynamicTextureRendered = pixelIs(mapped, 32, 32, {255, 0, 255});
    context.UnmapTextureSubresource(staging, 0, 0);
    require(refreshedDynamicTextureRendered,
            "Diligent scenery did not sample refreshed runtime texture contents");

    sceneryDraw.dynamicTexture = nullptr;
    runtimeScenery.source.batch.freeTextureVariable.clear();
    sceneryDraw.variables = nullptr;
    runtimeScenery.textures.freeTexture.reset();

    runtimeScenery.source.batch.textTextureIndex = 0;
    bool missingTextTextureRejected = false;
    try {
        runtimeScenery.drawTransparent(noDepth, context, *magentaView, identity, sceneryDraw);
    } catch (const std::logic_error&) {
        missingTextTextureRejected = true;
    }
    require(missingTextTextureRejected,
            "Diligent scenery did not report a missing runtime text texture view");
    sceneryVariables.set("opacity", 1.0F);
    sceneryDraw.variables = &sceneryVariables;
    sceneryDraw.textTexture = cyanView;
    context.SetRenderTargets(1, &targetView, nullptr, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    require(runtimeScenery.drawTransparent(noDepth, context, *magentaView, identity, sceneryDraw),
            "Diligent scenery did not submit its runtime text-atlas texture");
    mapped = readback();
    const bool textSceneryTextureRendered = pixelIs(mapped, 32, 32, {0, 0, 255});
    const auto* textSceneryPixel =
        static_cast<const std::uint8_t*>(mapped.pData) + 32 * mapped.Stride + 32 * 4;
    const std::array<std::uint8_t, 3> textSceneryColor = {
        textSceneryPixel[0], textSceneryPixel[1], textSceneryPixel[2]};
    context.UnmapTextureSubresource(staging, 0, 0);
    if (!textSceneryTextureRendered) {
        throw std::runtime_error(
            "Diligent scenery text-atlas pixel was " + std::to_string(textSceneryColor[0]) + "," +
            std::to_string(textSceneryColor[1]) + "," + std::to_string(textSceneryColor[2]));
    }
    sceneryDraw.textTexture = nullptr;
    sceneryDraw.variables = nullptr;
    runtimeScenery.source.batch.textTextureIndex = -1;

    runtimeScenery.source.batch.alphaMode = 2;
    bool transparentSceneryRejected = false;
    try {
        runtimeScenery.drawOpaque(noDepth, context, *magentaView, identity, sceneryDraw);
    } catch (const std::invalid_argument&) {
        transparentSceneryRejected = true;
    }
    require(transparentSceneryRejected, "Diligent opaque scenery accepted a transparent batch");
    runtimeScenery.source.batch.alphaMode = 1;
    runtimeScenery.source.batch.visibilityVariable = "visible";
    runtimeScenery.source.batch.visibilityValue = 1;
    require(!runtimeScenery.drawOpaque(noDepth, context, *magentaView, identity, sceneryDraw),
            "Diligent scenery ignored script visibility with no script state");
    sceneryVariables.set("visible", 1.0F);
    sceneryVariables.set("scroll", 0.0F);
    sceneryVariables.set("opacity", 1.0F);
    sceneryVariables.set("item_lamp", 1.0F);
    runtimeItem->lightmapTextureName = "decoded-pattern.tga";
    runtimeScenery.textures.lightmap.reset();
    runtimeScenery.textures.nightmap.reset();
    runtimeScenery.textures.bumpmap.reset();
    sceneryDraw.variables = &sceneryVariables;
    sceneryDraw.weatherBrightness = 1.0;
    DiligentLightingParameters darkSceneryLighting;
    darkSceneryLighting.directionalEnabled = true;
    darkSceneryLighting.ambientIntensity = 0.0F;
    darkSceneryLighting.diffuseIntensity = 0.0F;
    darkSceneryLighting.lightDirectionView = {0.0F, 0.0F, -1.0F};
    context.SetRenderTargets(1, &targetView, nullptr, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    require(runtimeScenery.drawOpaque(noDepth, context, *magentaView, identity, sceneryDraw,
                                      darkSceneryLighting),
            "Diligent scenery did not submit a script-visible selected material");
    mapped = readback();
    const bool itemSceneryCorrect = pixelIs(mapped, 32, 32, {255, 0, 0});
    context.UnmapTextureSubresource(staging, 0, 0);
    require(itemSceneryCorrect, "Diligent scenery selected-item material was not rendered");
    sceneryVariables.set("variant", 0.0F);
    context.SetRenderTargets(1, &targetView, nullptr, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    require(runtimeScenery.drawOpaque(noDepth, context, *magentaView, identity, sceneryDraw,
                                      darkSceneryLighting),
            "Diligent scenery did not submit the base material");
    mapped = readback();
    const bool baseSceneryCorrect = pixelIs(mapped, 32, 32, {0, 0, 0});
    context.UnmapTextureSubresource(staging, 0, 0);
    require(baseSceneryCorrect, "Diligent scenery did not switch away from the item lightmap");
    sceneryDraw.variables = nullptr;
    sceneryDraw.weatherBrightness.reset();
    bool missingSceneryImageRejected = false;
    try {
        PreparedSceneryAsset missingImage = preparedScenery;
        missingImage.decodedTextures.clear();
        DiligentSceneryAsset invalid(device, missingImage);
    } catch (const std::invalid_argument&) {
        missingSceneryImageRejected = true;
    }
    require(missingSceneryImageRejected,
            "Diligent scenery upload accepted a path without its prepared image");
    bool inconsistentSceneryGeometryRejected = false;
    try {
        PreparedSceneryAsset badGeometry = preparedScenery;
        ++badGeometry.batches[0].batch.vertexCount;
        DiligentSceneryAsset invalid(device, badGeometry);
    } catch (const std::invalid_argument&) {
        inconsistentSceneryGeometryRejected = true;
    }
    require(inconsistentSceneryGeometryRejected,
            "Diligent scenery upload accepted an inconsistent geometry count");
    bool missingSceneryMeshRejected = false;
    try {
        PreparedSceneryAsset missingMesh = preparedScenery;
        missingMesh.meshes.clear();
        DiligentSceneryAsset invalid(device, missingMesh);
    } catch (const std::invalid_argument&) {
        missingSceneryMeshRejected = true;
    }
    require(missingSceneryMeshRejected, "Diligent scenery upload accepted a missing source mesh");
    PreparedSceneryAsset failedMesh = preparedScenery;
    failedMesh.meshes["failed"].preparationError =
        std::make_exception_ptr(std::runtime_error("mesh preparation failed"));
    DiligentSceneryAsset withFailedMesh(device, failedMesh);
    require(withFailedMesh.diagnostics().size() == 2,
            "Diligent scenery upload did not report an unreferenced failed mesh");
    {
        std::ofstream treeConfig(textureDirectory / "tree.sco", std::ios::binary);
        treeConfig << "[tree]\ndecoded-pattern.tga\n10\n10\n1\n1\n";
        require(static_cast<bool>(treeConfig), "Failed to write prepared tree configuration");
    }
    PreparedSceneryAsset preparedTree =
        prepareSceneryAsset(textureDirectory / "tree.sco", textureDirectory, sharedSceneryTextures);
    require(preparedTree.treeBatches.size() == 1, "CPU scenery did not prepare the tree fixture");
    preparedTree.treeBatches.push_back(preparedTree.treeBatches.front());
    preparedTree.treeBatches.back().texturePath.clear();
    preparedTree.treeBatches.back().textureName = "missing-tree.tga";
    DiligentSceneryAsset directTrees(device, preparedTree);
    require(
        directTrees.trees().size() == 2 && directTrees.trees()[0].mesh &&
            directTrees.trees()[0].texture && !directTrees.trees()[1].mesh &&
            !directTrees.trees()[1].texture && directTrees.diagnostics().size() == 1,
        "Diligent scenery upload did not retain tree resources and missing-texture diagnostics");
    Matrix4 treeTransform = identity;
    treeTransform[0] = 0.1;
    treeTransform[6] = 0.1;
    treeTransform[9] = 0.2;
    treeTransform[13] = -1.0;
    treeTransform[14] = 0.5;
    DiligentSceneryOpaqueDraw treeDraw;
    treeDraw.seasonalTexture = cyanView;
    treeDraw.reverseWinding = true;
    context.SetRenderTargets(1, &targetView, nullptr, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    require(directTrees.trees()[0].draw(noDepth, context, *magentaView, treeTransform, treeDraw),
            "Diligent scenery tree did not submit");
    mapped = readback();
    const auto* treePixels = static_cast<const std::uint8_t*>(mapped.pData);
    bool treeSeasonOverrideCorrect = false;
    for (Uint32 y = 0; y < 64 && !treeSeasonOverrideCorrect; ++y) {
        for (Uint32 x = 0; x < 64; ++x) {
            const auto* pixel = treePixels + y * mapped.Stride + x * 4;
            if (pixel[0] < 35 && pixel[1] > 220 && pixel[2] > 220) {
                treeSeasonOverrideCorrect = true;
                break;
            }
        }
    }
    context.UnmapTextureSubresource(staging, 0, 0);
    require(treeSeasonOverrideCorrect, "Diligent scenery tree did not use its seasonal texture");
    require(!directTrees.trees()[1].draw(noDepth, context, *magentaView, treeTransform, treeDraw),
            "Diligent scenery tree submitted an asset with missing geometry");
    treeDraw.visibilityFade = 0.0F;
    require(!directTrees.trees()[0].draw(noDepth, context, *magentaView, treeTransform, treeDraw),
            "Diligent scenery tree submitted with zero visibility");
    preparedScenery = {};
    preparedTree = {};
    failedMesh = {};
    context.SetRenderTargets(1, &targetView, nullptr, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    require(directScenery.batches()[0].drawOpaque(noDepth, context, *magentaView, identity,
                                                  sceneryDraw),
            "Diligent scenery could not submit after releasing prepared CPU data");
    mapped = readback();
    const bool sceneryOutlivedPreparation = pixelIs(mapped, 32, 32, {255, 0, 0});
    context.UnmapTextureSubresource(staging, 0, 0);
    require(sceneryOutlivedPreparation,
            "Diligent scenery resources did not outlive CPU preparation data");

    const auto samplePatternCenter = [&](bool nearest, bool blend) {
        DiligentMaterialState sampledMaterial;
        sampledMaterial.nearestBaseTexture = nearest;
        if (blend) {
            sampledMaterial.alphaMode = DiligentAlphaMode::Blend;
            sampledMaterial.depthWrite = false;
        }
        context.SetRenderTargets(1, &targetView, nullptr,
                                 RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        noDepth.draw(context, full, *patternView, identity, sampledMaterial);
        const auto sample = readback();
        const auto* pixel =
            static_cast<const std::uint8_t*>(sample.pData) + 32 * sample.Stride + 32 * 4;
        const std::array<std::uint8_t, 3> color = {pixel[0], pixel[1], pixel[2]};
        context.UnmapTextureSubresource(staging, 0, 0);
        return color;
    };
    const auto linearSample = samplePatternCenter(false, false);
    const auto nearestSample = samplePatternCenter(true, false);
    const auto nearestBlendedSample = samplePatternCenter(true, true);
    require(std::all_of(linearSample.begin(), linearSample.end(),
                        [](std::uint8_t channel) { return channel > 60 && channel < 200; }) &&
                std::all_of(nearestSample.begin(), nearestSample.end(),
                            [](std::uint8_t channel) { return channel < 35 || channel > 220; }) &&
                nearestBlendedSample == nearestSample,
            "Diligent base-texture nearest and linear filtering did not differ as expected");

    const auto sampleDynamicMapCenter = [&](bool textMap, bool nearest) {
        DiligentMaterialState sampledMaterial;
        if (textMap) {
            sampledMaterial.textMap = patternView;
            sampledMaterial.nearestTextTexture = nearest;
        } else {
            sampledMaterial.freeMap = patternView;
            sampledMaterial.nearestFreeTexture = nearest;
        }
        context.SetRenderTargets(1, &targetView, nullptr,
                                 RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        noDepth.draw(context, full, *magentaView, identity, sampledMaterial);
        const auto sample = readback();
        const auto* pixel =
            static_cast<const std::uint8_t*>(sample.pData) + 32 * sample.Stride + 32 * 4;
        const std::array<std::uint8_t, 3> color = {pixel[0], pixel[1], pixel[2]};
        context.UnmapTextureSubresource(staging, 0, 0);
        return color;
    };
    const auto linearFreeMapSample = sampleDynamicMapCenter(false, false);
    const auto nearestFreeMapSample = sampleDynamicMapCenter(false, true);
    const auto linearTextMapSample = sampleDynamicMapCenter(true, false);
    const auto nearestTextMapSample = sampleDynamicMapCenter(true, true);
    const auto isInterpolated = [](const std::array<std::uint8_t, 3>& color) {
        return color[0] > 60 && color[0] < 200 && color[2] > 60 && color[2] < 200;
    };
    const auto isPointSampled = [](const std::array<std::uint8_t, 3>& color) {
        const auto isExtreme = [](std::uint8_t channel) { return channel < 35 || channel > 220; };
        return isExtreme(color[0]) && isExtreme(color[2]);
    };
    require(isInterpolated(linearFreeMapSample) && isPointSampled(nearestFreeMapSample) &&
                isInterpolated(linearTextMapSample) && isPointSampled(nearestTextMapSample),
            "Diligent script/text map filters did not honor their independent nearest settings");

    struct AddressCase {
        TextureAddressMode uMode;
        TextureAddressMode vMode;
        float u;
        float v;
        std::array<std::uint8_t, 3> color;
    };
    const std::array addressCases = {
        AddressCase{
            TextureAddressMode::Repeat, TextureAddressMode::Repeat, -0.25F, 0.25F, {0, 255, 0}},
        AddressCase{
            TextureAddressMode::Clamp, TextureAddressMode::Repeat, -0.25F, 0.25F, {255, 0, 0}},
        AddressCase{
            TextureAddressMode::Border, TextureAddressMode::Repeat, -0.25F, 0.25F, {0, 0, 0}},
        AddressCase{
            TextureAddressMode::Mirror, TextureAddressMode::Repeat, -0.25F, 0.25F, {255, 0, 0}},
        AddressCase{
            TextureAddressMode::Mirror, TextureAddressMode::Repeat, 1.75F, 0.25F, {255, 0, 0}},
        AddressCase{
            TextureAddressMode::MirrorOnce, TextureAddressMode::Repeat, 1.75F, 0.25F, {0, 255, 0}},
        AddressCase{
            TextureAddressMode::Repeat, TextureAddressMode::Clamp, 0.25F, 1.25F, {0, 0, 255}},
        AddressCase{
            TextureAddressMode::Repeat, TextureAddressMode::Repeat, 0.25F, 1.25F, {255, 0, 0}},
        AddressCase{
            TextureAddressMode::Repeat, TextureAddressMode::Border, 0.25F, 1.25F, {0, 0, 0}},
        AddressCase{
            TextureAddressMode::Repeat, TextureAddressMode::Mirror, 0.25F, 1.75F, {255, 0, 0}},
        AddressCase{
            TextureAddressMode::Repeat, TextureAddressMode::MirrorOnce, 0.25F, 1.75F, {0, 0, 255}},
        AddressCase{
            TextureAddressMode::Repeat, TextureAddressMode::Repeat, -0.25F, 0.25F, {0, 255, 0}},
    };
    for (const auto& addressCase : addressCases) {
        auto addressVertices = fullVertices;
        for (auto& vertex : addressVertices) {
            vertex.u = addressCase.u;
            vertex.v = addressCase.v;
        }
        DiligentTexturedMesh addressMesh(device, addressVertices);
        DiligentMaterialState addressMaterial;
        addressMaterial.nearestBaseTexture = true;
        addressMaterial.addressU = addressCase.uMode;
        addressMaterial.addressV = addressCase.vMode;
        context.SetRenderTargets(1, &targetView, nullptr,
                                 RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        noDepth.draw(context, addressMesh, *patternView, identity, addressMaterial);
        mapped = readback();
        const bool addressCorrect = pixelIs(mapped, 32, 32, addressCase.color);
        context.UnmapTextureSubresource(staging, 0, 0);
        require(addressCorrect, "Diligent texture address mode sampled the wrong texel");
    }
    DiligentMaterialState invalidAddress;
    invalidAddress.addressU = static_cast<TextureAddressMode>(999);
    bool invalidAddressRejected = false;
    try {
        noDepth.draw(context, full, *patternView, identity, invalidAddress);
    } catch (const std::invalid_argument&) {
        invalidAddressRejected = true;
    }
    require(invalidAddressRejected, "Diligent accepted an invalid texture address mode");

    require(noDepth.maximumAnisotropy() == 1 || noDepth.maximumAnisotropy() == 2 ||
                noDepth.maximumAnisotropy() == 4 || noDepth.maximumAnisotropy() == 8 ||
                noDepth.maximumAnisotropy() == 16,
            "Diligent reported an unsupported maximum anisotropy level");
    DiligentMaterialState anisotropicMaterial;
    anisotropicMaterial.anisotropy = 16;
    context.SetRenderTargets(1, &targetView, nullptr, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    noDepth.draw(context, full, *patternView, identity, anisotropicMaterial);
    mapped = readback();
    const bool anisotropicDrawCompleted = mapped.pData != nullptr;
    context.UnmapTextureSubresource(staging, 0, 0);
    require(anisotropicDrawCompleted,
            "Diligent failed to create or submit an anisotropic sampler variant");

    DiligentMaterialState invalidAnisotropy;
    invalidAnisotropy.anisotropy = 3;
    bool invalidAnisotropyRejected = false;
    try {
        noDepth.draw(context, full, *patternView, identity, invalidAnisotropy);
    } catch (const std::invalid_argument&) {
        invalidAnisotropyRejected = true;
    }
    require(invalidAnisotropyRejected, "Diligent accepted an invalid anisotropy level");

    const std::array<Image, 2> addressArrayImages = {patternImage, patternImage};
    DiligentTexture2DArray addressArray(device, addressArrayImages);
    auto addressArrayVertices = fullVertices;
    for (auto& vertex : addressArrayVertices) {
        vertex.u = -0.25F;
        vertex.v = 0.25F;
        vertex.layer = 1.0F;
    }
    DiligentTexturedMesh addressArrayMesh(device, addressArrayVertices);
    DiligentMaterialState addressArrayMaterial;
    addressArrayMaterial.nearestBaseTexture = true;
    addressArrayMaterial.addressU = TextureAddressMode::Clamp;
    context.SetRenderTargets(1, &targetView, nullptr, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    noDepth.draw(context, addressArrayMesh, addressArray, identity, addressArrayMaterial);
    mapped = readback();
    const bool arrayAddressCorrect = pixelIs(mapped, 32, 32, {255, 0, 0});
    context.UnmapTextureSubresource(staging, 0, 0);
    require(arrayAddressCorrect, "Diligent array texture did not apply its U address mode");

    DiligentDynamicTexture2D dynamicTexture(device, context, 1, 1, magenta);
    const auto drawDynamic = [&] {
        context.SetRenderTargets(1, &targetView, nullptr,
                                 RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        noDepth.draw(context, full, dynamicTexture.shaderResourceView(), identity);
        auto dynamicMapped = readback();
        return dynamicMapped;
    };
    mapped = drawDynamic();
    const bool initialDynamicCorrect = pixelIs(mapped, 32, 32, {255, 0, 255});
    context.UnmapTextureSubresource(staging, 0, 0);
    require(initialDynamicCorrect, "Diligent dynamic texture initial upload failed");

    auto* originalView = &dynamicTexture.shaderResourceView();
    dynamicTexture.update(context, 1, 1, cyan);
    require(&dynamicTexture.shaderResourceView() == originalView,
            "Same-size dynamic texture update replaced its view");
    mapped = drawDynamic();
    const bool updatedDynamicCorrect = pixelIs(mapped, 32, 32, {0, 255, 255});
    context.UnmapTextureSubresource(staging, 0, 0);
    require(updatedDynamicCorrect, "Diligent dynamic texture update was not visible");

    const std::array<std::uint8_t, 8> resizedPixels = {255, 0, 0, 255, 255, 0, 0, 255};
    RefCntAutoPtr<ITextureView> retainedView{originalView};
    dynamicTexture.update(context, 2, 1, resizedPixels);
    require(dynamicTexture.width() == 2 && dynamicTexture.height() == 1 &&
                &dynamicTexture.shaderResourceView() != retainedView.RawPtr(),
            "Diligent dynamic texture resize did not replace its view and dimensions");
    mapped = drawDynamic();
    const bool resizedDynamicCorrect = pixelIs(mapped, 32, 32, {255, 0, 0});
    context.UnmapTextureSubresource(staging, 0, 0);
    require(resizedDynamicCorrect, "Resized Diligent dynamic texture was not visible");

    bool invalidDynamicUpdate = false;
    try {
        dynamicTexture.update(context, 2, 2, cyan);
    } catch (const std::invalid_argument&) {
        invalidDynamicUpdate = true;
    }
    require(invalidDynamicUpdate && dynamicTexture.width() == 2 && dynamicTexture.height() == 1,
            "Diligent dynamic texture accepted invalid data or changed dimensions on failure");
    mapped = drawDynamic();
    const bool failedUpdatePreservedTexture = pixelIs(mapped, 32, 32, {255, 0, 0});
    context.UnmapTextureSubresource(staging, 0, 0);
    require(failedUpdatePreservedTexture, "Failed dynamic texture update changed its contents");
    dynamicTexture.update(context, 1, 1, magenta);
    dynamicTexture.update(context, 1, 1, cyan);
    dynamicTexture.update(context, 2, 1, resizedPixels);
    mapped = drawDynamic();
    const bool queuedUpdatesCorrect = pixelIs(mapped, 32, 32, {255, 0, 0});
    context.UnmapTextureSubresource(staging, 0, 0);
    require(queuedUpdatesCorrect, "Queued Diligent dynamic texture updates were not ordered");
    bool invalidDynamicDimensions = false;
    try {
        DiligentDynamicTexture2D rejected(device, context, 0, 1, {});
    } catch (const std::invalid_argument&) {
        invalidDynamicDimensions = true;
    }
    require(invalidDynamicDimensions, "Diligent dynamic texture accepted zero width");

    DiligentRuntimeTexture2D runtimeTexture(device);
    require(!runtimeTexture.initialized() && runtimeTexture.shaderResourceView() == nullptr,
            "Diligent runtime texture exposed a resource before its first snapshot");
    runtimeTexture.synchronize(context, 1, 1, magenta, false, 1);
    auto* runtimeTextureView = runtimeTexture.shaderResourceView();
    require(runtimeTexture.initialized() && runtimeTextureView != nullptr &&
                runtimeTexture.width() == 1 && runtimeTexture.height() == 1 &&
                !runtimeTexture.filtered() && runtimeTexture.revision() == 1,
            "Diligent runtime texture did not retain its first snapshot");
    runtimeTexture.synchronize(context, 1, 1, {}, true, 1);
    require(runtimeTexture.shaderResourceView() == runtimeTextureView &&
                runtimeTexture.filtered() && runtimeTexture.revision() == 1,
            "Diligent runtime texture did not update filter state without reuploading pixels");
    runtimeTexture.synchronize(context, 1, 1, cyan, true, 2);
    require(runtimeTexture.shaderResourceView() == runtimeTextureView &&
                runtimeTexture.revision() == 2,
            "Diligent runtime texture did not update a changed content revision");
    context.SetRenderTargets(1, &targetView, nullptr, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    noDepth.draw(context, full, *runtimeTexture.shaderResourceView(), identity);
    mapped = readback();
    const bool runtimeTexturePixelsCorrect = pixelIs(mapped, 32, 32, {0, 255, 255});
    context.UnmapTextureSubresource(staging, 0, 0);
    require(runtimeTexturePixelsCorrect,
            "Diligent runtime texture content revision did not reach its shader resource");
    runtimeTexture.synchronize(context, 2, 1, resizedPixels, false, 3);
    require(runtimeTexture.width() == 2 && runtimeTexture.height() == 1 &&
                runtimeTexture.revision() == 3 && !runtimeTexture.filtered(),
            "Diligent runtime texture did not resize from a new script snapshot");
    bool invalidRuntimeSnapshotRejected = false;
    try {
        runtimeTexture.synchronize(context, 2, 2, cyan, true, 4);
    } catch (const std::invalid_argument&) {
        invalidRuntimeSnapshotRejected = true;
    }
    require(invalidRuntimeSnapshotRejected && runtimeTexture.width() == 2 &&
                runtimeTexture.height() == 1 && runtimeTexture.revision() == 3 &&
                !runtimeTexture.filtered(),
            "Diligent runtime texture did not preserve state after a rejected snapshot");

    VehicleConfig scriptedTextureConfig;
    Variables scriptedTextureVariables(ScriptObjectKind::Vehicle);
    SimulationState scriptedTextureSimulation;
    ScriptRuntime scriptedTextureRuntime(scriptedTextureConfig, scriptedTextureVariables,
                                          scriptedTextureSimulation);
    scriptedTextureRuntime.configureScriptTextures({ModelScriptTexture{1, 1, 1, {}}});
    openbus::app::DiligentScriptTexture diligentScriptTexture(
        device, openbus::app::DiligentScriptTextureKind::Script, 1);
    require(diligentScriptTexture.synchronize(scriptedTextureRuntime, context) &&
                diligentScriptTexture.texture().initialized() &&
                diligentScriptTexture.texture().width() == 1 &&
                diligentScriptTexture.texture().height() == 1,
            "Diligent script texture bridge did not upload a configured runtime slot");
    const std::uint64_t initialScriptTextureRevision =
        diligentScriptTexture.texture().revision();
    require(diligentScriptTexture.synchronize(scriptedTextureRuntime, context) &&
                diligentScriptTexture.texture().revision() == initialScriptTextureRevision,
            "Diligent script texture bridge did not retain an unchanged runtime revision");
    scriptedTextureRuntime.configureScriptTextures({ModelScriptTexture{1, 1, 1, {}}});
    require(diligentScriptTexture.synchronize(scriptedTextureRuntime, context) &&
                diligentScriptTexture.texture().revision() != initialScriptTextureRevision,
            "Diligent script texture bridge did not upload a changed runtime revision");
    openbus::app::DiligentScriptTexture missingDiligentScriptTexture(
        device, openbus::app::DiligentScriptTextureKind::Script, 99);
    require(!missingDiligentScriptTexture.synchronize(scriptedTextureRuntime, context) &&
                !missingDiligentScriptTexture.texture().initialized(),
            "Diligent script texture bridge accepted an unconfigured slot");
    scriptedTextureRuntime.configureTextTextures(
        {ModelTextTexture{2, false, {"display_text", "unloaded-font", "1", "1", "0"}}});
    openbus::app::DiligentScriptTexture diligentTextTexture(
        device, openbus::app::DiligentScriptTextureKind::Text, 2);
    require(diligentTextTexture.synchronize(scriptedTextureRuntime, context) &&
                diligentTextTexture.texture().initialized() &&
                diligentTextTexture.texture().width() == 1 &&
                diligentTextTexture.texture().height() == 1,
            "Diligent text texture bridge did not upload a configured text slot");

    ParsedObj parsedModel;
    parsedModel.positions = {
        {-0.9, -0.8, 0.0}, {-0.1, -0.8, 0.0}, {-0.5, 0.8, 0.0},
        {0.1, -0.8, 0.0},  {0.9, -0.8, 0.0},  {0.5, 0.8, 0.0},
    };
    parsedModel.texCoords = {{0.0, 0.0}, {1.0, 0.0}, {0.5, 1.0}};
    parsedModel.hasTransform = true;
    parsedModel.transform[12] = 0.125;
    parsedModel.materials["left"].textureName = "decoded-pattern.tga";
    parsedModel.materials["left"].color = {0.75, 0.25, 0.5};
    parsedModel.materials["left"].specular = {0.25, 0.5, 0.75};
    parsedModel.materials["left"].specularPower = 4.0;
    parsedModel.materials["left"].alpha = 0.5;
    parsedModel.materials["right"].textureName = "decoded-pattern.tga";
    ObjTriangle leftTriangle;
    leftTriangle.indices = {ObjIndex{1, 1, 0}, ObjIndex{2, 2, 0}, ObjIndex{3, 3, 0}};
    leftTriangle.material = "left";
    ObjTriangle rightTriangle;
    rightTriangle.indices = {ObjIndex{4, 1, 0}, ObjIndex{5, 2, 0}, ObjIndex{6, 3, 0}};
    rightTriangle.material = "right";
    parsedModel.triangles = {leftTriangle, rightTriangle};
    DiligentObjModel importedModel(device, parsedModel, textureDirectory);
    const auto importedSubmeshes = importedModel.submeshes();
    require(importedSubmeshes.size() == 2,
            "Diligent OBJ import did not create one batch per material");
    const DiligentMaterialState importedLeftMaterial = importedSubmeshes[0].materialState();
    const DiligentMaterialState importedRightMaterial = importedSubmeshes[1].materialState();
    require(importedModel.hasTransform() && importedModel.transform()[12] == 0.125 &&
                importedSubmeshes[0].materialName == "left" &&
                importedSubmeshes[0].material.textureName == "decoded-pattern.tga" &&
                importedSubmeshes[0].baseTexture != nullptr &&
                importedSubmeshes[0].baseTexture == importedSubmeshes[1].baseTexture &&
                importedSubmeshes[1].materialName == "right" &&
                importedSubmeshes[1].mesh != nullptr &&
                importedLeftMaterial.baseColor[0] == 0.75F &&
                importedLeftMaterial.baseColor[3] == 0.5F &&
                importedLeftMaterial.specularColor[1] == 0.5F &&
                importedLeftMaterial.specularPower == 4.0F &&
                importedLeftMaterial.alphaMode == DiligentAlphaMode::Blend &&
                !importedLeftMaterial.depthWrite &&
                importedRightMaterial.alphaMode == DiligentAlphaMode::Opaque &&
                importedRightMaterial.depthWrite,
            "Diligent OBJ import did not preserve transforms or map material state");
    ParsedObj missingTextureModel = parsedModel;
    missingTextureModel.materials["left"].textureName = "missing.tga";
    bool missingMaterialTextureRejected = false;
    try {
        DiligentObjModel rejectedModel(device, missingTextureModel, textureDirectory);
    } catch (const std::runtime_error&) {
        missingMaterialTextureRejected = true;
    }
    require(missingMaterialTextureRejected,
            "Diligent OBJ import silently accepted a missing material texture");
    DiligentObjSubmesh invalidAlphaMaterial;
    invalidAlphaMaterial.material.alpha = 1.5;
    bool invalidAlphaRejected = false;
    try {
        static_cast<void>(invalidAlphaMaterial.materialState());
    } catch (const std::invalid_argument&) {
        invalidAlphaRejected = true;
    }
    require(invalidAlphaRejected, "Diligent OBJ material mapping accepted invalid alpha");
    context.SetRenderTargets(1, &targetView, nullptr, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    for (const DiligentObjSubmesh& submesh : importedSubmeshes) {
        submesh.draw(noDepth, context, *magentaView, identity);
    }
    mapped = readback();
    const auto pixelHasMaterial = [&mapped](Uint32 x, Uint32 y) {
        const auto* pixel =
            static_cast<const std::uint8_t*>(mapped.pData) + y * mapped.Stride + x * 4;
        return pixel[0] > 20 || pixel[1] > 20 || pixel[2] > 20;
    };
    const bool importedGeometryCorrect =
        pixelHasMaterial(16, 32) && pixelHasMaterial(48, 32) && pixelIs(mapped, 32, 32, {0, 0, 0});
    context.UnmapTextureSubresource(staging, 0, 0);
    require(importedGeometryCorrect,
            "Diligent OBJ submesh draw did not bind imported material textures");

    DiligentObjModel fallbackModel(device, parsedModel);
    const auto fallbackSubmeshes = fallbackModel.submeshes();
    require(fallbackSubmeshes.size() == 2 && !fallbackSubmeshes[0].baseTexture,
            "Diligent OBJ import unexpectedly loaded textures without a texture root");
    context.SetRenderTargets(1, &targetView, nullptr, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    fallbackSubmeshes[0].draw(noDepth, context, *magentaView, identity);
    mapped = readback();
    const bool fallbackTextureCorrect =
        pixelHasMaterial(16, 32) && pixelIs(mapped, 48, 32, {0, 0, 0});
    context.UnmapTextureSubresource(staging, 0, 0);
    require(fallbackTextureCorrect, "Diligent OBJ submesh did not use its fallback texture");

    ModelAnimation translationAnimation;
    translationAnimation.type = "anim_trans";
    const Matrix4 translatedAnimation =
        openbus::rendering::modelAnimationTransform(translationAnimation, 2.0);
    require(translatedAnimation[12] == 0.0 && translatedAnimation[13] == -2.0 &&
                translatedAnimation[14] == 0.0,
            "Backend-neutral vehicle translation animation changed axis convention");
    ModelAnimation rotationAnimation;
    rotationAnimation.type = "anim_rot";
    rotationAnimation.originOperations.push_back(
        {ModelAnimationOriginType::Translation, {1.0, 0.0, 0.0}});
    const Matrix4 rotatedAnimation =
        openbus::rendering::modelAnimationTransform(rotationAnimation, 90.0);
    require(std::abs(rotatedAnimation[12] - 1.0) < 1.0e-9 &&
                std::abs(rotatedAnimation[14] - 1.0) < 1.0e-9,
            "Backend-neutral vehicle rotation animation changed pivot handling");
    openbus::rendering::ModelAnimationTransformBuilder animationBuilder;
    animationBuilder.append(translationAnimation, 2.0);
    animationBuilder.append(rotationAnimation, 90.0);
    const Matrix4 expectedAnimationComposition =
        openbus::rendering::multiplyMatrix4(rotatedAnimation, translatedAnimation);
    for (std::size_t index = 0; index < expectedAnimationComposition.size(); ++index) {
        require(std::abs(animationBuilder.matrix()[index] - expectedAnimationComposition[index]) <
                    1.0e-9,
                "Backend-neutral vehicle animation builder changed part transform order");
    }

    ParsedObj coordinateModel;
    coordinateModel.positions = {{0.15, 0.15, 0.0}, {0.85, 0.15, 0.0}, {0.15, 0.85, 0.0}};
    coordinateModel.texCoords = {{0.25, 0.25}};
    coordinateModel.normals = {{1.0, 0.0, 0.0}};
    ObjTriangle coordinateTriangle;
    coordinateTriangle.indices = {ObjIndex{1, 1, 1}, ObjIndex{2, 1, 1}, ObjIndex{3, 1, 1}};
    coordinateTriangle.material = "scenery";
    coordinateModel.triangles = {coordinateTriangle};
    coordinateModel.materials["scenery"].materialIndex = 7;
    const auto sceneryGeometry = openbus::map::prepareMapSceneryGeometry(coordinateModel);
    const auto sceneryVertices = sceneryGeometry.verticesByMaterial.find(7);
    require(sceneryGeometry.invalidPositionIndices == 0 &&
                sceneryVertices != sceneryGeometry.verticesByMaterial.end() &&
                sceneryVertices->second.size() == 3 && sceneryVertices->second[0].v == 0.75F &&
                sceneryVertices->second[0].nx == 1.0F,
            "Shared scenery preparation did not preserve material, UV, or normal data");
    DiligentTexturedMesh sceneryMesh(device, sceneryVertices->second);
    DiligentLightingParameters sceneryLighting;
    sceneryLighting.directionalEnabled = true;
    sceneryLighting.lightDirectionView = {1.0F, 0.0F, 0.0F};
    sceneryLighting.ambientIntensity = 0.0F;
    sceneryLighting.diffuseIntensity = 1.0F;
    context.SetRenderTargets(1, &targetView, nullptr, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    noDepth.draw(context, sceneryMesh, *patternView, identity, {}, sceneryLighting);
    mapped = readback();
    const bool sceneryGeometryCorrect =
        pixelIs(mapped, 42, 22, {0, 0, 255}) && pixelIs(mapped, 42, 42, {0, 0, 0});
    context.UnmapTextureSubresource(staging, 0, 0);
    require(sceneryGeometryCorrect, "Diligent did not render the shared scenery geometry directly");

    ParsedObj invalidScenery = coordinateModel;
    invalidScenery.triangles[0].indices[0].position = 4;
    invalidScenery.triangles[0].indices[1].texCoord = 0;
    invalidScenery.triangles[0].indices[1].normal = 0;
    const auto invalidSceneryGeometry = openbus::map::prepareMapSceneryGeometry(invalidScenery);
    const auto& remainingSceneryVertices = invalidSceneryGeometry.verticesByMaterial.at(7);
    require(invalidSceneryGeometry.invalidPositionIndices == 1 &&
                remainingSceneryVertices.size() == 2 && remainingSceneryVertices[0].u == 0.0F &&
                remainingSceneryVertices[0].v == 0.0F && remainingSceneryVertices[0].nz == 1.0F &&
                remainingSceneryVertices[0].r == 1.0F,
            "Shared scenery preparation changed invalid-index or fallback behavior");

    DiligentObjModel parsedFrameModel(device, coordinateModel);
    DiligentObjImportOptions vehicleOptions;
    vehicleOptions.coordinateFrame = DiligentObjCoordinateFrame::Vehicle;
    vehicleOptions.flipTextureV = true;
    DiligentObjModel vehicleFrameModel(device, coordinateModel, {}, vehicleOptions);
    context.SetRenderTargets(1, &targetView, nullptr, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    parsedFrameModel.submeshes()[0].draw(noDepth, context, *patternView, identity);
    mapped = readback();
    const bool parsedFrameCorrect =
        pixelIs(mapped, 42, 22, {255, 0, 0}) && pixelIs(mapped, 42, 42, {0, 0, 0});
    context.UnmapTextureSubresource(staging, 0, 0);
    require(parsedFrameCorrect, "Diligent OBJ import changed the parsed frame or UV orientation");

    DiligentLightingParameters vehicleLighting;
    vehicleLighting.directionalEnabled = true;
    vehicleLighting.lightDirectionView = {0.0F, -1.0F, 0.0F};
    vehicleLighting.ambientIntensity = 0.0F;
    vehicleLighting.diffuseIntensity = 1.0F;
    context.SetRenderTargets(1, &targetView, nullptr, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    vehicleFrameModel.submeshes()[0].draw(noDepth, context, *patternView, identity,
                                          vehicleLighting);
    mapped = readback();
    const bool vehicleFrameCorrect =
        pixelIs(mapped, 42, 42, {0, 0, 255}) && pixelIs(mapped, 42, 22, {0, 0, 0});
    context.UnmapTextureSubresource(staging, 0, 0);
    require(vehicleFrameCorrect,
            "Diligent OBJ vehicle frame did not transform geometry, normals, or UVs");

    const std::array<const ObjTriangle*, 1> vehicleTriangles = {&coordinateModel.triangles[0]};
    const auto vehicleVertices = prepareVehicleMeshGeometry(coordinateModel, vehicleTriangles);
    require(vehicleVertices.size() == 3 && vehicleVertices[0].x == 0.15F &&
                vehicleVertices[0].y == -0.15F && vehicleVertices[0].v == 0.75F &&
                vehicleVertices[0].nx == 0.0F && vehicleVertices[0].ny == -1.0F &&
                vehicleVertices[0].r == 1.0F,
            "Shared vehicle geometry changed coordinate, UV, normal, or color conventions");
    DiligentTexturedMesh vehicleMesh(device, vehicleVertices);
    context.SetRenderTargets(1, &targetView, nullptr, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    noDepth.draw(context, vehicleMesh, *patternView, identity, {}, vehicleLighting);
    mapped = readback();
    const bool directVehicleGeometryCorrect =
        pixelIs(mapped, 42, 42, {0, 0, 255}) && pixelIs(mapped, 42, 22, {0, 0, 0});
    context.UnmapTextureSubresource(staging, 0, 0);
    require(directVehicleGeometryCorrect,
            "Diligent did not render shared vehicle geometry directly");

    DiligentVehicleBatchSource preparedVehicleBatch;
    preparedVehicleBatch.materialIndex = 42;
    preparedVehicleBatch.vertices = vehicleVertices;
    DiligentVehicleAsset preparedVehicleAsset(
        device, std::span<const DiligentVehicleBatchSource>(&preparedVehicleBatch, 1));
    require(preparedVehicleAsset.submeshes().size() == 1 &&
                preparedVehicleAsset.submeshes()[0].materialIndex == 42,
            "Diligent vehicle asset did not retain production-prepared batch metadata");
    context.SetRenderTargets(1, &targetView, nullptr, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    preparedVehicleAsset.draw(noDepth, context, *patternView, identity, identity, vehicleLighting);
    mapped = readback();
    const bool preparedVehicleAssetCorrect =
        pixelIs(mapped, 42, 42, {0, 0, 255}) && pixelIs(mapped, 42, 22, {0, 0, 0});
    context.UnmapTextureSubresource(staging, 0, 0);
    require(preparedVehicleAssetCorrect,
            "Diligent vehicle asset did not render production-prepared geometry");

    std::array<DiligentVehicleBatchSource, 2> animatedVehicleBatches = {
        preparedVehicleBatch, preparedVehicleBatch};
    animatedVehicleBatches[1].materialIndex = 43;
    animatedVehicleBatches[1].animationGroupIndex = 1;
    DiligentVehicleAsset animatedVehicleAsset(device, animatedVehicleBatches);
    Matrix4 shiftedGroupMvp = identity;
    shiftedGroupMvp[12] = 0.5;
    const std::array<DiligentVehicleGroupTransform, 2> groupTransforms = {{
        {0, identity, identity, true, std::nullopt, std::nullopt},
        {1, shiftedGroupMvp, identity, true, std::nullopt, std::nullopt},
    }};
    MotionHistory vehicleMotionHistory;
    vehicleMotionHistory.beginFrame(1, 1.0, 84, 84, identity, identity);
    const auto firstVehicleGroupTransform = DiligentVehicleGroupTransform::fromMotionHistory(
        0, vehicleMotionHistory.frame(), vehicleMotionHistory, {9001, 4},
        openbus::rendering::translationMatrix({-0.25, 0.0, 0.0}));
    require(firstVehicleGroupTransform.motion.has_value() &&
                !firstVehicleGroupTransform.motion->historyValid,
            "Diligent vehicle animation group incorrectly used first-frame history");
    vehicleMotionHistory.completeFrame();
    vehicleMotionHistory.beginFrame(2, 2.0, 84, 84, identity, identity);
    const auto secondVehicleGroupTransform = DiligentVehicleGroupTransform::fromMotionHistory(
        0, vehicleMotionHistory.frame(), vehicleMotionHistory, {9001, 4},
        openbus::rendering::translationMatrix({0.25, 0.0, 0.0}));
    require(secondVehicleGroupTransform.motion->historyValid &&
                secondVehicleGroupTransform.motion->previousModelViewProjection[12] == -0.25 &&
                secondVehicleGroupTransform.modelViewProjection[12] == 0.25,
            "Diligent vehicle animation group did not use rendered-frame transforms");
    vehicleMotionHistory.completeFrame();
    MotionHistory animatedVehicleMotionHistory;
    ModelAnimation animatedPart;
    animatedPart.type = "anim_trans";
    const std::array<ModelAnimation, 1> partAnimations = {animatedPart};
    const std::array<double, 1> firstAnimationAmounts = {0.5};
    const std::array<double, 1> secondAnimationAmounts = {1.5};
    animatedVehicleMotionHistory.beginFrame(1, 1.0, 84, 84, identity, identity);
    const auto firstAnimatedGroup = DiligentVehicleGroupTransform::fromAnimationsAndHistory(
        2, animatedVehicleMotionHistory.frame(), animatedVehicleMotionHistory, {9002, 2},
        identity, partAnimations, firstAnimationAmounts);
    require(firstAnimatedGroup.modelView[13] == -0.5 && firstAnimatedGroup.motion.has_value() &&
                !firstAnimatedGroup.motion->historyValid,
            "Diligent vehicle did not compose its first animation transform");
    animatedVehicleMotionHistory.completeFrame();
    animatedVehicleMotionHistory.beginFrame(2, 2.0, 84, 84, identity, identity);
    const auto secondAnimatedGroup = DiligentVehicleGroupTransform::fromAnimationsAndHistory(
        2, animatedVehicleMotionHistory.frame(), animatedVehicleMotionHistory, {9002, 2},
        identity, partAnimations, secondAnimationAmounts);
    require(secondAnimatedGroup.modelView[13] == -1.5 &&
                secondAnimatedGroup.motion->previousModelViewProjection[13] == -0.5 &&
                secondAnimatedGroup.motion->historyValid,
            "Diligent vehicle did not retain the previous rendered animation transform");
    animatedVehicleMotionHistory.completeFrame();
    context.SetRenderTargets(1, &targetView, nullptr, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    animatedVehicleAsset.draw(noDepth, context, *patternView, identity, identity,
                              vehicleLighting, nullptr, {}, nullptr, groupTransforms);
    mapped = readback();
    const bool animatedVehicleGroupsCorrect =
        pixelIs(mapped, 42, 42, {0, 0, 255}) && pixelIs(mapped, 63, 42, {0, 0, 255});
    context.UnmapTextureSubresource(staging, 0, 0);
    require(animatedVehicleGroupsCorrect,
            "Diligent vehicle animation groups did not use independent model transforms");

    ParsedObj fallbackVehicle = coordinateModel;
    fallbackVehicle.normals.clear();
    fallbackVehicle.texCoords.clear();
    fallbackVehicle.vertexColors = {{0.25, 0.5, 0.75, 0.5}};
    const std::array<const ObjTriangle*, 1> fallbackTriangles = {&fallbackVehicle.triangles[0]};
    const auto fallbackVertices = prepareVehicleMeshGeometry(fallbackVehicle, fallbackTriangles);
    require(fallbackVertices.size() == 3 && fallbackVertices[0].nz == 1.0F &&
                fallbackVertices[0].u == 0.0F && fallbackVertices[0].v == 0.0F &&
                fallbackVertices[0].r == 0.25F && fallbackVertices[0].a == 0.5F &&
                fallbackVertices[1].r == 1.0F,
            "Shared vehicle geometry changed missing normal, UV, or color behavior");
    ObjTriangle reversedTriangle = fallbackVehicle.triangles[0];
    std::swap(reversedTriangle.indices[0], reversedTriangle.indices[1]);
    const std::array<const ObjTriangle*, 2> orderedTriangles = {&reversedTriangle,
                                                                &fallbackVehicle.triangles[0]};
    const auto orderedVehicleVertices =
        prepareVehicleMeshGeometry(fallbackVehicle, orderedTriangles);
    require(orderedVehicleVertices.size() == 6 && orderedVehicleVertices[0].y == -0.85F &&
                orderedVehicleVertices[1].y == -0.15F && orderedVehicleVertices[3].y == -0.15F &&
                orderedVehicleVertices[4].y == -0.85F,
            "Shared vehicle geometry changed triangle order");
    ParsedObj degenerateVehicle = fallbackVehicle;
    degenerateVehicle.positions[1] = degenerateVehicle.positions[0];
    const std::array<const ObjTriangle*, 1> degenerateTriangles = {&degenerateVehicle.triangles[0]};
    const auto degenerateVertices =
        prepareVehicleMeshGeometry(degenerateVehicle, degenerateTriangles);
    require(degenerateVertices[0].nx == 0.0F && degenerateVertices[0].ny == 0.0F &&
                degenerateVertices[0].nz == 1.0F,
            "Shared vehicle geometry changed degenerate face-normal behavior");
    fallbackVehicle.triangles[0].indices[1].position = 0;
    const auto invalidVehicleVertices =
        prepareVehicleMeshGeometry(fallbackVehicle, fallbackTriangles);
    require(invalidVehicleVertices.size() == 2 && invalidVehicleVertices[0].nz == 1.0F,
            "Shared vehicle geometry changed invalid-position behavior");
    const std::array<const ObjTriangle*, 1> nullTriangles = {nullptr};
    bool nullTriangleRejected = false;
    try {
        static_cast<void>(prepareVehicleMeshGeometry(coordinateModel, nullTriangles));
    } catch (const std::invalid_argument&) {
        nullTriangleRejected = true;
    }
    require(nullTriangleRejected, "Shared vehicle geometry accepted a null triangle");

    ParsedObj invalidModel;
    invalidModel.positions = parsedModel.positions;
    invalidModel.triangles = {leftTriangle};
    invalidModel.triangles[0].indices[2].position = 7;
    bool invalidModelRejected = false;
    try {
        DiligentObjModel rejectedModel(device, invalidModel);
    } catch (const std::invalid_argument&) {
        invalidModelRejected = true;
    }
    require(invalidModelRejected, "Diligent OBJ import accepted an out-of-range vertex index");

    DiligentMaterialState flippedTexture;
    flippedTexture.flipTextureY = true;
    context.SetRenderTargets(1, &targetView, nullptr, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    noDepth.draw(context, full, compressedRedTexture, identity);
    mapped = readback();
    const bool compressedTextureCorrect = pixelIs(mapped, 32, 32, {255, 0, 0});
    context.UnmapTextureSubresource(staging, 0, 0);
    require(compressedTextureCorrect, "Diligent compressed DDS texture did not sample correctly");

    CompressedDds malformedDds;
    malformedDds.width = 4;
    malformedDds.height = 4;
    malformedDds.levels.emplace_back(15, 0);
    bool malformedCompressedTextureRejected = false;
    try {
        DiligentTexture2D invalidCompressedTexture(device, malformedDds);
    } catch (const std::invalid_argument&) {
        malformedCompressedTextureRejected = true;
    }
    require(malformedCompressedTextureRejected,
            "Diligent compressed texture accepted a truncated BC3 mip level");

    context.SetRenderTargets(1, &targetView, nullptr, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    noDepth.draw(context, full, *patternTexture, identity, flippedTexture);
    mapped = readback();
    const bool flippedTextureCorrect =
        pixelIs(mapped, 19, 19, {0, 0, 255}) && pixelIs(mapped, 44, 19, {255, 255, 255}) &&
        pixelIs(mapped, 19, 44, {255, 0, 0}) && pixelIs(mapped, 44, 44, {0, 255, 0});
    context.UnmapTextureSubresource(staging, 0, 0);
    require(flippedTextureCorrect, "Textured mesh did not apply its vertical UV flip");

    DiligentMaterialState offsetTexture;
    offsetTexture.textureOffset = {0.25F, 0.25F};
    context.SetRenderTargets(1, &targetView, nullptr, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    noDepth.draw(context, full, *patternTexture, identity, offsetTexture);
    mapped = readback();
    const bool textureOffsetCorrect = pixelIs(mapped, 32, 32, {255, 255, 255});
    context.UnmapTextureSubresource(staging, 0, 0);
    require(textureOffsetCorrect, "Textured mesh did not apply its UV offset");

    auto scaledVertices = fullVertices;
    for (auto& vertex : scaledVertices) {
        vertex.u = 0.25F;
        vertex.v = 0.5F;
    }
    DiligentTexturedMesh scaledMesh(device, scaledVertices);
    DiligentMaterialState scaledTexture;
    scaledTexture.textureScale = {2.0F, 1.0F};
    scaledTexture.textureOffset = {0.25F, 0.0F};
    context.SetRenderTargets(1, &targetView, nullptr, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    noDepth.draw(context, scaledMesh, *scaleView, identity, scaledTexture);
    mapped = readback();
    const auto* scaledPixel =
        static_cast<const std::uint8_t*>(mapped.pData) + 32 * mapped.Stride + 32 * 4;
    const bool textureScaleCorrect = scaledPixel[0] >= 115 && scaledPixel[0] <= 140 &&
                                     scaledPixel[1] < 35 && scaledPixel[2] >= 115 &&
                                     scaledPixel[2] <= 140;
    context.UnmapTextureSubresource(staging, 0, 0);
    require(textureScaleCorrect,
            "Textured mesh did not apply UV offset before scale with repeat addressing");

    context.SetRenderTargets(1, &targetView, nullptr, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    Matrix4 translated = identity;
    translated[12] = 0.5;
    translated[13] = 0.5;
    noDepth.draw(context, near, *magentaView, translated);
    mapped = readback();
    const bool transformCorrect =
        pixelIs(mapped, 48, 16, {255, 0, 255}) && pixelIs(mapped, 32, 32, {0, 0, 0});
    context.UnmapTextureSubresource(staging, 0, 0);
    require(transformCorrect, "Textured mesh transform or vertical orientation is incorrect");

    auto tintedVertices = nearVertices;
    for (auto& vertex : tintedVertices) {
        vertex.r = 0.5f;
        vertex.g = 0.25f;
        vertex.b = 0.75f;
        vertex.a = 0.5f;
    }
    DiligentTexturedMesh tinted(device, tintedVertices);
    const std::array<std::uint8_t, 4> white = {255, 255, 255, 255};
    const std::array<std::uint8_t, 4> black = {0, 0, 0, 255};
    const std::array<std::uint8_t, 4> red = {255, 0, 0, 255};
    const std::array<std::uint8_t, 4> tangentXNormal = {255, 128, 128, 255};
    const std::array<std::uint8_t, 8> verticalTransmapPixels = {
        255, 255, 255, 0, 255, 255, 255, 255,
    };
    auto whiteTexture = makeTexture("White", 1, 1, white.data());
    auto blackTexture = makeTexture("Black", 1, 1, black.data());
    auto redTexture = makeTexture("Red", 1, 1, red.data());
    auto verticalTransmapTexture =
        makeTexture("Vertical alpha transmap", 1, 2, verticalTransmapPixels.data());
    auto tangentXNormalTexture = makeTexture("Tangent X normal", 1, 1, tangentXNormal.data());
    auto* whiteView = whiteTexture->GetDefaultView(TEXTURE_VIEW_SHADER_RESOURCE);
    auto* blackView = blackTexture->GetDefaultView(TEXTURE_VIEW_SHADER_RESOURCE);
    auto* redView = redTexture->GetDefaultView(TEXTURE_VIEW_SHADER_RESOURCE);
    auto* verticalTransmapView =
        verticalTransmapTexture->GetDefaultView(TEXTURE_VIEW_SHADER_RESOURCE);
    auto* tangentXNormalView = tangentXNormalTexture->GetDefaultView(TEXTURE_VIEW_SHADER_RESOURCE);
    require(whiteView != nullptr && blackView != nullptr && redView != nullptr &&
                verticalTransmapView != nullptr && tangentXNormalView != nullptr,
            "Textured mesh test texture view is missing");
    DiligentMaterialState transmapBlend;
    transmapBlend.alphaMode = DiligentAlphaMode::Blend;
    transmapBlend.depthWrite = false;
    transmapBlend.transMap = translucentView;
    constexpr float blueBackground[] = {0.0F, 0.0F, 1.0F, 1.0F};
    context.SetRenderTargets(1, &targetView, nullptr, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(targetView, blueBackground,
                              RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    noDepth.draw(context, full, *whiteView, identity, transmapBlend);
    mapped = readback();
    const auto* transmapBlendPixel =
        static_cast<const std::uint8_t*>(mapped.pData) + 32 * mapped.Stride + 32 * 4;
    const bool transmapAlphaCorrect = transmapBlendPixel[0] >= 120 &&
                                      transmapBlendPixel[0] <= 136 &&
                                      transmapBlendPixel[1] >= 120 &&
                                      transmapBlendPixel[1] <= 136 && transmapBlendPixel[2] >= 240;
    context.UnmapTextureSubresource(staging, 0, 0);
    require(transmapAlphaCorrect, "Diligent transmap alpha did not control material blending");
    transmapBlend.transMap = transparentView;
    context.SetRenderTargets(1, &targetView, nullptr, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(targetView, blueBackground,
                              RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    noDepth.draw(context, full, *whiteView, identity, transmapBlend);
    mapped = readback();
    const bool transparentTransmapCorrect = pixelIs(mapped, 32, 32, {0, 0, 255});
    context.UnmapTextureSubresource(staging, 0, 0);
    require(transparentTransmapCorrect,
            "Diligent transparent transmap alpha did not preserve the background");
    transmapBlend.transMap = verticalTransmapView;
    transmapBlend.flipTransmapY = false;
    context.SetRenderTargets(1, &targetView, nullptr, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(targetView, blueBackground,
                              RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    noDepth.draw(context, full, *whiteView, identity, transmapBlend);
    mapped = readback();
    const auto* normalTransmapTop =
        static_cast<const std::uint8_t*>(mapped.pData) + 16 * mapped.Stride + 32 * 4;
    const auto* normalTransmapBottom =
        static_cast<const std::uint8_t*>(mapped.pData) + 48 * mapped.Stride + 32 * 4;
    const auto topAlphaWithoutFlip = normalTransmapTop[0];
    const auto bottomAlphaWithoutFlip = normalTransmapBottom[0];
    context.UnmapTextureSubresource(staging, 0, 0);
    require(std::abs(static_cast<int>(topAlphaWithoutFlip) -
                     static_cast<int>(bottomAlphaWithoutFlip)) > 80,
            "Diligent transmap UVs did not sample both vertical alpha regions");
    transmapBlend.flipTransmapY = true;
    context.SetRenderTargets(1, &targetView, nullptr, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(targetView, blueBackground,
                              RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    noDepth.draw(context, full, *whiteView, identity, transmapBlend);
    mapped = readback();
    const auto* flippedTransmapTop =
        static_cast<const std::uint8_t*>(mapped.pData) + 16 * mapped.Stride + 32 * 4;
    const auto* flippedTransmapBottom =
        static_cast<const std::uint8_t*>(mapped.pData) + 48 * mapped.Stride + 32 * 4;
    const bool transmapFlipCorrect = std::abs(static_cast<int>(flippedTransmapTop[0]) -
                                              static_cast<int>(bottomAlphaWithoutFlip)) <= 16 &&
                                     std::abs(static_cast<int>(flippedTransmapBottom[0]) -
                                              static_cast<int>(topAlphaWithoutFlip)) <= 16;
    context.UnmapTextureSubresource(staging, 0, 0);
    require(transmapFlipCorrect, "Diligent transmap did not apply its vertical UV flip");

    DiligentTexturedMeshRenderer linearDepthRenderer(device, targetDesc.Format, depthDesc.Format,
                                                     TEX_FORMAT_R32_FLOAT);
    DiligentTexturedMeshRenderer motionAndDepthRenderer(device, targetDesc.Format, depthDesc.Format,
                                                        TEX_FORMAT_RG32_FLOAT, TEX_FORMAT_R8_UNORM,
                                                        TEX_FORMAT_R32_FLOAT);
    TextureDesc motionOutputDesc = targetDesc;
    motionOutputDesc.Name = "Textured mesh motion-vector target";
    motionOutputDesc.Format = TEX_FORMAT_RG32_FLOAT;
    RefCntAutoPtr<ITexture> motionOutput;
    device.CreateTexture(motionOutputDesc, nullptr, &motionOutput);
    require(static_cast<bool>(motionOutput), "Failed to create motion-vector test target");
    TextureDesc validityOutputDesc = targetDesc;
    validityOutputDesc.Name = "Textured mesh motion-validity target";
    validityOutputDesc.Format = TEX_FORMAT_R8_UNORM;
    RefCntAutoPtr<ITexture> validityOutput;
    device.CreateTexture(validityOutputDesc, nullptr, &validityOutput);
    require(static_cast<bool>(validityOutput), "Failed to create motion-validity test target");
    auto* motionOutputView = motionOutput->GetDefaultView(TEXTURE_VIEW_RENDER_TARGET);
    auto* validityOutputView = validityOutput->GetDefaultView(TEXTURE_VIEW_RENDER_TARGET);
    require(motionOutputView != nullptr && validityOutputView != nullptr,
            "Textured mesh motion output view is missing");
    const auto readLinearDepth = [&]() {
        context.SetRenderTargets(0, nullptr, nullptr, RESOURCE_STATE_TRANSITION_MODE_NONE);
        CopyTextureAttribs depthCopy;
        depthCopy.pSrcTexture = linearDepthTarget;
        depthCopy.SrcTextureTransitionMode = RESOURCE_STATE_TRANSITION_MODE_TRANSITION;
        depthCopy.pDstTexture = linearDepthStaging;
        depthCopy.DstTextureTransitionMode = RESOURCE_STATE_TRANSITION_MODE_TRANSITION;
        context.CopyTexture(depthCopy);
        context.WaitForIdle();
        MappedTextureSubresource mappedLinearDepth;
        context.MapTextureSubresource(linearDepthStaging, 0, 0, MAP_READ, MAP_FLAG_DO_NOT_WAIT,
                                      nullptr, mappedLinearDepth);
        require(mappedLinearDepth.pData != nullptr,
                "Failed to map textured mesh linear-depth readback");
        const auto* linearDepthPixel = static_cast<const std::uint8_t*>(mappedLinearDepth.pData) +
                                       32 * mappedLinearDepth.Stride + 32 * sizeof(float);
        float centerLinearDepth = 0.0F;
        std::memcpy(&centerLinearDepth, linearDepthPixel, sizeof(centerLinearDepth));
        context.UnmapTextureSubresource(linearDepthStaging, 0, 0);
        return centerLinearDepth;
    };
    ITextureView* linearDepthTargets[] = {targetView, linearDepthView};
    context.SetRenderTargets(2, linearDepthTargets, depthView,
                             RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    constexpr float clearLinearDepth[] = {0.0F, 0.0F, 0.0F, 0.0F};
    context.ClearRenderTarget(linearDepthView, clearLinearDepth,
                              RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearDepthStencil(depthView, CLEAR_DEPTH_FLAG, 1.0F, 0,
                              RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    linearDepthRenderer.draw(context, near, *whiteView, identity);
    const float centerLinearDepth = readLinearDepth();
    require(std::fabs(centerLinearDepth - 0.75F) < 0.01F,
            "Diligent linear-depth output did not contain positive view-space depth");
    ITextureView* motionDepthTargets[] = {targetView, motionOutputView, validityOutputView,
                                          linearDepthView};
    context.SetRenderTargets(4, motionDepthTargets, depthView,
                             RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    constexpr float clearMotion[] = {0.0F, 0.0F, 0.0F, 0.0F};
    context.ClearRenderTarget(motionOutputView, clearMotion,
                              RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(validityOutputView, clearMotion,
                              RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(linearDepthView, clearLinearDepth,
                              RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearDepthStencil(depthView, CLEAR_DEPTH_FLAG, 1.0F, 0,
                              RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    DiligentMotionParameters invalidMotionHistory;
    invalidMotionHistory.previousModelViewProjection = identity;
    invalidMotionHistory.viewportWidth = size;
    invalidMotionHistory.viewportHeight = size;
    DiligentMaterialState nearestMotionMaterial;
    nearestMotionMaterial.nearestBaseTexture = true;
    nearestMotionMaterial.cullBackFaces = true;
    nearestMotionMaterial.depthMode = DiligentDepthMode::LessEqual;
    nearestMotionMaterial.addressU = TextureAddressMode::Clamp;
    nearestMotionMaterial.snowCoverage = 1.0F;
    nearestMotionMaterial.environmentMap = redView;
    nearestMotionMaterial.environmentStrength = 0.5F;
    nearestMotionMaterial.environmentBrightness = 0.4F;
    auto reversedNearVertices = nearVertices;
    std::swap(reversedNearVertices[1], reversedNearVertices[2]);
    std::swap(reversedNearVertices[4], reversedNearVertices[5]);
    DiligentTexturedMesh reversedNear(device, reversedNearVertices);
    motionAndDepthRenderer.drawMotion(context, reversedNear, *whiteView, identity,
                                      invalidMotionHistory, nearestMotionMaterial);
    const float combinedLinearDepth = readLinearDepth();
    require(std::fabs(combinedLinearDepth - 0.75F) < 0.01F,
            "Diligent combined motion/depth pass did not preserve linear depth");
    mapped = readback();
    const auto* weatherMotionPixel =
        static_cast<const std::uint8_t*>(mapped.pData) + 32 * mapped.Stride + 32 * 4;
    const bool weatherMotionColor = weatherMotionPixel[0] >= 165 && weatherMotionPixel[0] <= 167 &&
                                    weatherMotionPixel[1] >= 116 && weatherMotionPixel[1] <= 119 &&
                                    weatherMotionPixel[2] >= 119 && weatherMotionPixel[2] <= 121;
    context.UnmapTextureSubresource(staging, 0, 0);
    require(weatherMotionColor, "Diligent motion pass did not apply snow and environment shading");

    context.SetRenderTargets(4, motionDepthTargets, depthView,
                             RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(motionOutputView, clearMotion,
                              RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(validityOutputView, clearMotion,
                              RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(linearDepthView, clearLinearDepth,
                              RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearDepthStencil(depthView, CLEAR_DEPTH_FLAG, 1.0F, 0,
                              RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    DiligentMotionParameters sceneryMotion = invalidMotionHistory;
    sceneryMotion.historyValid = true;
    require(directScenery.batches()[0].drawOpaque(motionAndDepthRenderer, context, *magentaView,
                                                  identity, sceneryDraw, {}, &sceneryMotion),
            "Diligent scenery motion submission skipped a visible batch");
    mapped = readback();
    const bool sceneryMotionColor = pixelIs(mapped, 32, 32, {255, 0, 0});
    context.UnmapTextureSubresource(staging, 0, 0);
    require(sceneryMotionColor, "Diligent scenery motion submission did not render its material");
    TextureDesc validityStagingDesc = validityOutputDesc;
    validityStagingDesc.Name = "Scenery motion validity staging";
    validityStagingDesc.Usage = USAGE_STAGING;
    validityStagingDesc.BindFlags = BIND_NONE;
    validityStagingDesc.CPUAccessFlags = CPU_ACCESS_READ;
    RefCntAutoPtr<ITexture> validityStaging;
    device.CreateTexture(validityStagingDesc, nullptr, &validityStaging);
    require(static_cast<bool>(validityStaging), "Failed to create scenery motion staging texture");
    CopyTextureAttribs validityCopy;
    validityCopy.pSrcTexture = validityOutput;
    validityCopy.SrcTextureTransitionMode = RESOURCE_STATE_TRANSITION_MODE_TRANSITION;
    validityCopy.pDstTexture = validityStaging;
    validityCopy.DstTextureTransitionMode = RESOURCE_STATE_TRANSITION_MODE_TRANSITION;
    context.CopyTexture(validityCopy);
    context.WaitForIdle();
    MappedTextureSubresource mappedValidity;
    context.MapTextureSubresource(validityStaging, 0, 0, MAP_READ, MAP_FLAG_DO_NOT_WAIT, nullptr,
                                  mappedValidity);
    require(mappedValidity.pData != nullptr, "Failed to map scenery motion validity");
    const auto* validityPixel =
        static_cast<const std::uint8_t*>(mappedValidity.pData) + 32 * mappedValidity.Stride + 32;
    const bool sceneryMotionValid = *validityPixel == 255;
    context.UnmapTextureSubresource(validityStaging, 0, 0);
    require(sceneryMotionValid, "Diligent scenery did not write valid rendered-frame motion");

    context.SetRenderTargets(1, &targetView, nullptr, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    noDepth.draw(context, full, *cyanView, identity);
    noDepth.draw(context, tinted, *whiteView, identity);
    mapped = readback();
    const auto* center =
        static_cast<const std::uint8_t*>(mapped.pData) + 32 * mapped.Stride + 32 * 4;
    const bool colorCorrect = center[0] >= 126 && center[0] <= 129 && center[1] >= 63 &&
                              center[1] <= 65 && center[2] >= 190 && center[2] <= 192 &&
                              center[3] >= 126 && center[3] <= 129;
    context.UnmapTextureSubresource(staging, 0, 0);
    require(colorCorrect, "Textured mesh vertex color or depth-disabled drawing is incorrect");

    auto lightingVertices = fullVertices;
    for (auto& vertex : lightingVertices) {
        vertex.nx = 0.0F;
        vertex.ny = 0.0F;
        vertex.nz = 1.0F;
    }
    DiligentTexturedMesh lightingMesh(device, lightingVertices);
    const auto readLitCenter = [&](DiligentLightingParameters lighting,
                                   DiligentMaterialState material,
                                   ITextureView* normalMap = nullptr) {
        context.SetRenderTargets(1, &targetView, nullptr,
                                 RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        noDepth.draw(context, lightingMesh, *whiteView, identity, material, lighting, normalMap);
        const auto litImage = readback();
        const auto* litPixel =
            static_cast<const std::uint8_t*>(litImage.pData) + 32 * litImage.Stride + 32 * 4;
        const std::array<std::uint8_t, 4> result = {litPixel[0], litPixel[1], litPixel[2],
                                                    litPixel[3]};
        context.UnmapTextureSubresource(staging, 0, 0);
        return result;
    };
    const auto unlitColor = readLitCenter({}, {});
    require(unlitColor[0] >= 250 && unlitColor[1] >= 250 && unlitColor[2] >= 250,
            "Disabled Diligent directional lighting changed the unlit output");

    DiligentMaterialState ambientMaterial;
    ambientMaterial.baseColor = {0.2F, 0.4F, 0.6F, 1.0F};
    ambientMaterial.whiteAmbient = true;
    const auto unlitWhiteAmbient = readLitCenter({}, ambientMaterial);
    require(unlitWhiteAmbient[0] >= 50 && unlitWhiteAmbient[0] <= 52 &&
                unlitWhiteAmbient[1] >= 101 && unlitWhiteAmbient[1] <= 103 &&
                unlitWhiteAmbient[2] >= 152 && unlitWhiteAmbient[2] <= 154,
            "Disabled lighting incorrectly applied white ambient");
    DiligentLightingParameters halfAmbient;
    halfAmbient.directionalEnabled = true;
    halfAmbient.ambientIntensity = 0.5F;
    halfAmbient.diffuseIntensity = 0.0F;
    const auto whiteAmbientColor = readLitCenter(halfAmbient, ambientMaterial);
    require(whiteAmbientColor[0] >= 126 && whiteAmbientColor[0] <= 129 &&
                whiteAmbientColor[1] >= 126 && whiteAmbientColor[1] <= 129 &&
                whiteAmbientColor[2] >= 126 && whiteAmbientColor[2] <= 129,
            "Diligent scenery did not preserve white ambient reflectance");
    ambientMaterial.whiteAmbient = false;
    const auto tintedAmbientColor = readLitCenter(halfAmbient, ambientMaterial);
    require(tintedAmbientColor[0] >= 24 && tintedAmbientColor[0] <= 27 &&
                tintedAmbientColor[1] >= 50 && tintedAmbientColor[1] <= 52 &&
                tintedAmbientColor[2] >= 75 && tintedAmbientColor[2] <= 78,
            "Diligent ordinary ambient lighting changed while adding white ambient");
    ambientMaterial.whiteAmbient = true;
    ambientMaterial.snowCoverage = 1.0F;
    const auto snowyAmbientColor = readLitCenter(halfAmbient, ambientMaterial);
    require(snowyAmbientColor[0] >= 114 && snowyAmbientColor[0] <= 116 &&
                snowyAmbientColor[1] >= 117 && snowyAmbientColor[1] <= 119 &&
                snowyAmbientColor[2] >= 119 && snowyAmbientColor[2] <= 121,
            "Diligent snow did not alter white ambient reflectance");

    DiligentMaterialState snowy;
    snowy.baseColor = {0.2F, 0.2F, 0.2F, 0.75F};
    snowy.snowCoverage = 0.5F;
    const auto halfSnow = readLitCenter({}, snowy);
    require(halfSnow[0] >= 139 && halfSnow[0] <= 142 && halfSnow[1] >= 142 && halfSnow[1] <= 144 &&
                halfSnow[2] >= 144 && halfSnow[2] <= 147 && halfSnow[3] >= 190 &&
                halfSnow[3] <= 192,
            "Diligent snow coverage did not blend surface color while preserving alpha");
    snowy.snowCoverage = 2.0F;
    const auto fullSnow = readLitCenter({}, snowy);
    require(fullSnow[0] >= 229 && fullSnow[0] <= 231 && fullSnow[1] >= 234 && fullSnow[1] <= 236 &&
                fullSnow[2] >= 239 && fullSnow[2] <= 241,
            "Diligent snow coverage did not clamp like the production shader");
    snowy.snowCoverage = -1.0F;
    const auto noSnow = readLitCenter({}, snowy);
    require(noSnow[0] >= 50 && noSnow[0] <= 52 && noSnow[1] >= 50 && noSnow[1] <= 52 &&
                noSnow[2] >= 50 && noSnow[2] <= 52,
            "Diligent negative snow coverage did not clamp to zero");

    DiligentMaterialState environment;
    environment.baseColor = {0.2F, 0.2F, 0.2F, 0.75F};
    environment.environmentMap = redView;
    environment.environmentStrength = 0.5F;
    environment.environmentBrightness = 0.4F;
    const auto environmentColor = readLitCenter({}, environment);
    require(environmentColor[0] >= 75 && environmentColor[0] <= 78 && environmentColor[1] >= 24 &&
                environmentColor[1] <= 27 && environmentColor[2] >= 24 &&
                environmentColor[2] <= 27 && environmentColor[3] >= 190 &&
                environmentColor[3] <= 192,
            "Diligent environment map did not apply strength and weather brightness");
    environment.environmentStrength = 0.0F;
    const auto environmentDisabled = readLitCenter({}, environment);
    require(environmentDisabled[0] >= 50 && environmentDisabled[0] <= 52 &&
                environmentDisabled[1] >= 50 && environmentDisabled[1] <= 52 &&
                environmentDisabled[2] >= 50 && environmentDisabled[2] <= 52,
            "Diligent zero-strength environment map changed the surface");
    environment.environmentStrength = 2.0F;
    const auto clampedEnvironment = readLitCenter({}, environment);
    require(clampedEnvironment[0] >= 101 && clampedEnvironment[0] <= 103 &&
                clampedEnvironment[1] <= 1 && clampedEnvironment[2] <= 1,
            "Diligent environment strength did not clamp to one");
    environment.environmentMap = nullptr;
    require(readLitCenter({}, environment) == environmentDisabled,
            "Diligent missing environment map changed the surface");
    environment.environmentMap = patternView;
    environment.environmentStrength = 1.0F;
    environment.environmentBrightness = 1.0F;
    DiligentLightingParameters tiltedEnvironment;
    tiltedEnvironment.normalTransform[8] = 0.382683432;
    tiltedEnvironment.normalTransform[10] = 0.923879533;
    const auto reflectedColor = readLitCenter(tiltedEnvironment, environment);
    require(reflectedColor[1] > reflectedColor[0] + 45 &&
                reflectedColor[1] > reflectedColor[2] + 45,
            "Diligent environment map did not use the view-space reflection direction");
    const auto reflectedMappedColor =
        readLitCenter(tiltedEnvironment, environment, tangentXNormalView);
    require(reflectedMappedColor == reflectedColor,
            "Diligent environment mapping used the normal-mapped instead of geometric normal");
    bool invalidEnvironmentRejected = false;
    try {
        environment.environmentMap = textureArrayView;
        static_cast<void>(readLitCenter({}, environment));
    } catch (const std::invalid_argument&) {
        invalidEnvironmentRejected = true;
    }
    require(invalidEnvironmentRejected, "Diligent accepted a non-2D environment view");
    bool invalidSnowRejected = false;
    try {
        snowy.snowCoverage = std::numeric_limits<float>::quiet_NaN();
        static_cast<void>(readLitCenter({}, snowy));
    } catch (const std::invalid_argument&) {
        invalidSnowRejected = true;
    }
    require(invalidSnowRejected, "Diligent accepted non-finite snow coverage");
    bool invalidEnvironmentBrightnessRejected = false;
    try {
        environment.environmentMap = redView;
        environment.environmentBrightness = -1.0F;
        static_cast<void>(readLitCenter({}, environment));
    } catch (const std::invalid_argument&) {
        invalidEnvironmentBrightnessRejected = true;
    }
    require(invalidEnvironmentBrightnessRejected,
            "Diligent accepted negative environment brightness");

    DiligentMaterialState detailMaterial;
    detailMaterial.detailMap = scaleView;
    const auto defaultDetailScaleColor = readLitCenter({}, detailMaterial);
    require(defaultDetailScaleColor[0] >= 110 && defaultDetailScaleColor[0] <= 145 &&
                defaultDetailScaleColor[2] >= 110 && defaultDetailScaleColor[2] <= 145,
            "Diligent detail texture did not multiply the base color");
    detailMaterial.detailMapScale[0] = 0.25F;
    const auto scaledDetailColor = readLitCenter({}, detailMaterial);
    require(scaledDetailColor[0] >= 240 && scaledDetailColor[1] <= 10 && scaledDetailColor[2] <= 10,
            "Diligent detail texture did not apply its independent UV scale");

    DiligentLightingParameters specularLighting;
    specularLighting.directionalEnabled = true;
    specularLighting.ambientIntensity = 0.0F;
    specularLighting.diffuseColor = {0.0F, 0.0F, 0.0F};
    specularLighting.diffuseIntensity = 1.0F;
    DiligentMaterialState reflectiveMaterial;
    reflectiveMaterial.baseColor = {0.2F, 0.2F, 0.2F, 1.0F};
    reflectiveMaterial.specularColor = {0.5F, 0.5F, 0.5F};
    const auto specularFrontColor = readLitCenter(specularLighting, reflectiveMaterial);
    require(specularFrontColor[0] >= 120 && specularFrontColor[1] >= 120 &&
                specularFrontColor[2] >= 120,
            "Diligent material did not produce directional Blinn-Phong highlights");
    const auto normalMappedSpecularColor =
        readLitCenter(specularLighting, reflectiveMaterial, tangentXNormalView);
    require(std::abs(static_cast<int>(normalMappedSpecularColor[0]) -
                     static_cast<int>(specularFrontColor[0])) <= 10,
            "Diligent normal mapping changed the production geometric-normal specular highlight");
    specularLighting.weatherBrightness = 0.5F;
    const auto halfWeatherSpecularColor = readLitCenter(specularLighting, reflectiveMaterial);
    require(halfWeatherSpecularColor[0] >= specularFrontColor[0] / 2 - 8 &&
                halfWeatherSpecularColor[0] <= specularFrontColor[0] / 2 + 8,
            "Diligent specular intensity did not follow production weather brightness");
    specularLighting.weatherBrightness = 1.0F;
    specularLighting.lightDirectionView = {0.0F, 0.0F, -1.0F};
    const auto specularBackColor = readLitCenter(specularLighting, reflectiveMaterial);
    require(specularBackColor[0] <= 10 && specularBackColor[1] <= 10 && specularBackColor[2] <= 10,
            "Diligent material produced a specular highlight on a back-facing surface");

    DiligentLightingParameters emissionLighting;
    emissionLighting.directionalEnabled = true;
    emissionLighting.ambientIntensity = 0.0F;
    emissionLighting.lightDirectionView = {0.0F, 0.0F, -1.0F};
    DiligentMaterialState selfLitMaterial;
    selfLitMaterial.baseColor = {0.2F, 0.2F, 0.2F, 1.0F};
    selfLitMaterial.emissionColor = {1.0F, 1.0F, 1.0F};
    const auto selfLitColor = readLitCenter(emissionLighting, selfLitMaterial);
    require(selfLitColor[0] >= 48 && selfLitColor[0] <= 54 && selfLitColor[1] >= 48 &&
                selfLitColor[1] <= 54 && selfLitColor[2] >= 48 && selfLitColor[2] <= 54,
            "Diligent self-illumination did not preserve the unlit base color");

    DiligentLightingParameters directionalLighting;
    directionalLighting.directionalEnabled = true;
    directionalLighting.ambientIntensity = 0.0F;
    directionalLighting.diffuseIntensity = 1.0F;
    const auto frontLitColor = readLitCenter(directionalLighting, {});
    require(frontLitColor[0] >= 240 && frontLitColor[1] >= 240 && frontLitColor[2] >= 240,
            "Diligent Lambert lighting did not illuminate a front-facing surface");

    directionalLighting.lightDirectionView = {0.0F, 0.0F, -1.0F};
    const auto backLitColor = readLitCenter(directionalLighting, {});
    require(backLitColor[0] <= 10 && backLitColor[1] <= 10 && backLitColor[2] <= 10,
            "Diligent Lambert lighting did not attenuate a back-facing surface");

    directionalLighting.lightDirectionView = {1.0F, 0.0F, 0.0F};
    directionalLighting.normalTransform = {
        0.0, 0.0, -1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    };
    const auto transformedNormalColor = readLitCenter(directionalLighting, {});
    require(transformedNormalColor[0] >= 240 && transformedNormalColor[1] >= 240 &&
                transformedNormalColor[2] >= 240,
            "Diligent directional lighting did not apply the supplied normal transform");

    DiligentLightingParameters darkLighting;
    darkLighting.directionalEnabled = true;
    darkLighting.ambientIntensity = 0.0F;
    darkLighting.diffuseIntensity = 1.0F;
    darkLighting.lightDirectionView = {0.0F, 0.0F, -1.0F};

    auto fixedUvVertices = lightingVertices;
    for (auto& vertex : fixedUvVertices) {
        vertex.u = 0.25F;
        vertex.v = 0.25F;
    }
    DiligentTexturedMesh fixedUvMesh(device, fixedUvVertices);
    const auto readFixedUv = [&](const DiligentMaterialState& material,
                                 const DiligentLightingParameters& lighting,
                                 ITextureView* normalMap = nullptr) {
        context.SetRenderTargets(1, &targetView, nullptr,
                                 RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        noDepth.draw(context, fixedUvMesh, *whiteView, identity, material, lighting, normalMap);
        const auto image = readback();
        const auto* pixel =
            static_cast<const std::uint8_t*>(image.pData) + 32 * image.Stride + 32 * 4;
        const std::array<std::uint8_t, 4> result = {pixel[0], pixel[1], pixel[2], pixel[3]};
        context.UnmapTextureSubresource(staging, 0, 0);
        return result;
    };
    DiligentMaterialState auxiliaryUv;
    auxiliaryUv.textureScale = {2.0F, 2.0F};
    auxiliaryUv.textureOffset = {0.5F, 0.5F};
    auxiliaryUv.flipTextureY = true;
    auxiliaryUv.detailMap = patternView;
    const auto transformedDetail = readFixedUv(auxiliaryUv, {});
    require(transformedDetail[0] <= 5 && transformedDetail[1] >= 250 && transformedDetail[2] <= 5,
            "Diligent detail map did not use flipped, offset, independently scaled UVs");
    auxiliaryUv.detailMapScale[0] = 3.0F;
    const auto scaledTransformedDetail = readFixedUv(auxiliaryUv, {});
    require(scaledTransformedDetail[0] >= 250 && scaledTransformedDetail[1] <= 5 &&
                scaledTransformedDetail[2] <= 5,
            "Diligent detail scale was not applied after the material UV transform");
    auxiliaryUv.detailMap = nullptr;
    auxiliaryUv.lightMap = patternView;
    const auto transformedLightmap = readFixedUv(auxiliaryUv, darkLighting);
    require(transformedLightmap[0] <= 5 && transformedLightmap[1] >= 250 &&
                transformedLightmap[2] <= 5,
            "Diligent lightmap did not follow the transformed material UVs");
    auxiliaryUv.lightMap = nullptr;
    auxiliaryUv.nightMap = patternView;
    const auto transformedNightmap = readFixedUv(auxiliaryUv, darkLighting);
    require(transformedNightmap[0] <= 5 && transformedNightmap[1] >= 250 &&
                transformedNightmap[2] <= 5,
            "Diligent nightmap did not follow the transformed material UVs");
    const std::array<std::uint8_t, 16> alphaUvPattern = {
        255, 255, 255, 0, 255, 255, 255, 255, 255, 255, 255, 0, 255, 255, 255, 0,
    };
    auto alphaUvTexture = makeTexture("Transformed alpha", 2, 2, alphaUvPattern.data());
    auxiliaryUv.nightMap = nullptr;
    auxiliaryUv.transMap = alphaUvTexture->GetDefaultView(TEXTURE_VIEW_SHADER_RESOURCE);
    require(auxiliaryUv.transMap != nullptr, "Transformed alpha test view is missing");
    auxiliaryUv.alphaMode = DiligentAlphaMode::Cutout;
    const auto transformedTransmap = readFixedUv(auxiliaryUv, {});
    require(transformedTransmap[0] >= 250 && transformedTransmap[1] >= 250 &&
                transformedTransmap[2] >= 250,
            "Diligent transmap did not follow the transformed material UVs");
    const auto transformedBumpCutout = readFixedUv(auxiliaryUv, {}, tangentXNormalView);
    require(transformedBumpCutout[0] >= 250 && transformedBumpCutout[1] >= 250 &&
                transformedBumpCutout[2] >= 250,
            "Diligent normal-mapped transmap discarded a visible fragment");
    auxiliaryUv.transMapScale[0] = 3.0F;
    const auto scaledTransformedTransmap = readFixedUv(auxiliaryUv, {});
    require(scaledTransformedTransmap[0] <= 5 && scaledTransformedTransmap[1] <= 5 &&
                scaledTransformedTransmap[2] <= 5,
            "Diligent transmap scale was not applied after the material UV transform");
    auxiliaryUv.transMapScale[0] = 1.0F;
    auxiliaryUv.flipTransmapY = true;
    const auto flippedTransmap = readFixedUv(auxiliaryUv, {});
    require(flippedTransmap[0] <= 5 && flippedTransmap[1] <= 5 && flippedTransmap[2] <= 5,
            "Diligent transmap-specific flip did not follow the material UV transform");
    const auto flippedBumpCutout = readFixedUv(auxiliaryUv, {}, tangentXNormalView);
    require(flippedBumpCutout[0] <= 5 && flippedBumpCutout[1] <= 5 && flippedBumpCutout[2] <= 5,
            "Diligent normal-mapped transmap did not discard a transparent fragment");

    DiligentMotionParameters validMotionHistory = invalidMotionHistory;
    validMotionHistory.historyValid = true;
    const auto motionDepthForTransmap = [&](bool flipTransmapY) {
        auxiliaryUv.flipTransmapY = flipTransmapY;
        context.SetRenderTargets(4, motionDepthTargets, depthView,
                                 RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        context.ClearRenderTarget(motionOutputView, clearMotion,
                                  RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        context.ClearRenderTarget(validityOutputView, clearMotion,
                                  RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        context.ClearRenderTarget(linearDepthView, clearLinearDepth,
                                  RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        context.ClearDepthStencil(depthView, CLEAR_DEPTH_FLAG, 1.0F, 0,
                                  RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        motionAndDepthRenderer.drawMotion(context, fixedUvMesh, *whiteView, identity,
                                          validMotionHistory, auxiliaryUv);
        return readLinearDepth();
    };
    require(std::fabs(motionDepthForTransmap(false) - 0.5F) < 0.01F &&
                motionDepthForTransmap(true) == 0.0F,
            "Diligent motion pass did not apply transformed transmap cutout");

    DiligentMaterialState whiteLightmapMaterial;
    whiteLightmapMaterial.lightMap = whiteView;
    const auto whiteLightmapColor = readLitCenter(darkLighting, whiteLightmapMaterial);
    require(whiteLightmapColor[0] >= 240 && whiteLightmapColor[1] >= 240 &&
                whiteLightmapColor[2] >= 240,
            "Diligent white lightmap did not restore the unlit base color");
    DiligentMaterialState blackLightmapMaterial;
    blackLightmapMaterial.lightMap = blackView;
    const auto blackLightmapColor = readLitCenter(darkLighting, blackLightmapMaterial);
    require(blackLightmapColor[0] <= 10 && blackLightmapColor[1] <= 10 &&
                blackLightmapColor[2] <= 10,
            "Diligent black lightmap did not preserve directional shading");

    DiligentMaterialState nightOverlayMaterial;
    nightOverlayMaterial.nightMap = redView;
    const auto nightOverlayColor = readLitCenter({}, nightOverlayMaterial);
    require(nightOverlayColor[0] >= 240 && nightOverlayColor[1] <= 10 && nightOverlayColor[2] <= 10,
            "Diligent nightmap did not add its self-illumination overlay");
    nightOverlayMaterial.nightMapStrength = 0.0F;
    const auto disabledNightOverlay = readLitCenter({}, nightOverlayMaterial);
    require(disabledNightOverlay[0] >= 240 && disabledNightOverlay[1] >= 240 &&
                disabledNightOverlay[2] >= 240,
            "Diligent zero-strength nightmap changed the base color");
    bool invalidLightmapStrengthRejected = false;
    try {
        DiligentMaterialState invalidLightmap;
        invalidLightmap.lightMap = whiteView;
        invalidLightmap.lightMapStrength = 1.01F;
        noDepth.draw(context, lightingMesh, *whiteView, identity, invalidLightmap);
    } catch (const std::invalid_argument&) {
        invalidLightmapStrengthRejected = true;
    }
    require(invalidLightmapStrengthRejected,
            "Diligent renderer accepted an out-of-range lightmap strength");
    bool invalidNightmapViewRejected = false;
    try {
        DiligentMaterialState invalidNightmap;
        invalidNightmap.nightMap = textureArrayView;
        noDepth.draw(context, lightingMesh, *whiteView, identity, invalidNightmap);
    } catch (const std::invalid_argument&) {
        invalidNightmapViewRejected = true;
    }
    require(invalidNightmapViewRejected, "Diligent renderer accepted a texture-array nightmap");
    bool invalidTransmapScaleRejected = false;
    try {
        DiligentMaterialState invalidTransmap;
        invalidTransmap.transMap = translucentView;
        invalidTransmap.transMapScale[0] = std::numeric_limits<float>::quiet_NaN();
        noDepth.draw(context, lightingMesh, *whiteView, identity, invalidTransmap);
    } catch (const std::invalid_argument&) {
        invalidTransmapScaleRejected = true;
    }
    require(invalidTransmapScaleRejected, "Diligent renderer accepted a non-finite transmap scale");
    bool invalidDetailMapViewRejected = false;
    try {
        DiligentMaterialState invalidDetailMap;
        invalidDetailMap.detailMap = textureArrayView;
        noDepth.draw(context, lightingMesh, *whiteView, identity, invalidDetailMap);
    } catch (const std::invalid_argument&) {
        invalidDetailMapViewRejected = true;
    }
    require(invalidDetailMapViewRejected, "Diligent renderer accepted a texture-array detail map");
    bool invalidDetailMapScaleRejected = false;
    try {
        DiligentMaterialState invalidDetailMap;
        invalidDetailMap.detailMap = scaleView;
        invalidDetailMap.detailMapScale[1] = std::numeric_limits<float>::infinity();
        noDepth.draw(context, lightingMesh, *whiteView, identity, invalidDetailMap);
    } catch (const std::invalid_argument&) {
        invalidDetailMapScaleRejected = true;
    }
    require(invalidDetailMapScaleRejected,
            "Diligent renderer accepted a non-finite detail-map scale");

    DiligentLightingParameters normalMapLighting;
    normalMapLighting.directionalEnabled = true;
    normalMapLighting.ambientIntensity = 0.0F;
    normalMapLighting.diffuseIntensity = 1.0F;
    normalMapLighting.lightDirectionView = {1.0F, 0.0F, 0.0F};
    const auto unperturbedNormalColor = readLitCenter(normalMapLighting, {});
    const auto mappedNormalColor = readLitCenter(normalMapLighting, {}, tangentXNormalView);
    require(unperturbedNormalColor[0] <= 10 && mappedNormalColor[0] >= 240,
            "Diligent tangent-space normal map did not change directional lighting");
    auto normalUvVertices = lightingVertices;
    for (auto& vertex : normalUvVertices) {
        vertex.u = 0.05F + vertex.u * 0.4F;
        vertex.v = 0.05F + vertex.v * 0.4F;
    }
    DiligentTexturedMesh normalUvMesh(device, normalUvVertices);
    const std::array<std::uint8_t, 16> normalUvPattern = {
        128, 128, 255, 255, 255, 128, 128, 255, 128, 128, 255, 255, 128, 128, 255, 255,
    };
    auto normalUvTexture = makeTexture("Transformed normal", 2, 2, normalUvPattern.data());
    auto* normalUvView = normalUvTexture->GetDefaultView(TEXTURE_VIEW_SHADER_RESOURCE);
    require(normalUvView != nullptr, "Transformed normal map test view is missing");
    DiligentMaterialState normalUvMaterial;
    normalUvMaterial.textureScale = {2.0F, 2.0F};
    normalUvMaterial.textureOffset = {0.5F, 0.0F};
    context.SetRenderTargets(1, &targetView, nullptr, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    noDepth.draw(context, normalUvMesh, *whiteView, identity, normalUvMaterial, normalMapLighting,
                 normalUvView);
    mapped = readback();
    const bool transformedNormal = pixelIs(mapped, 32, 32, {255, 255, 255});
    context.UnmapTextureSubresource(staging, 0, 0);
    require(transformedNormal,
            "Diligent normal map did not use offset UVs independent of base texture scale");
    DiligentMaterialState disabledNormalMap;
    disabledNormalMap.normalMapStrength = 0.0F;
    const auto zeroStrengthColor =
        readLitCenter(normalMapLighting, disabledNormalMap, tangentXNormalView);
    require(zeroStrengthColor[0] <= 10,
            "Diligent zero-strength normal map changed the geometric normal");

    DiligentMaterialState invalidNormalStrength;
    invalidNormalStrength.normalMapStrength = 1.01F;
    bool invalidNormalStrengthRejected = false;
    try {
        noDepth.draw(context, lightingMesh, *whiteView, identity, invalidNormalStrength, {},
                     tangentXNormalView);
    } catch (const std::invalid_argument&) {
        invalidNormalStrengthRejected = true;
    }
    require(invalidNormalStrengthRejected,
            "Diligent renderer accepted an out-of-range normal-map strength");
    bool invalidNormalMapViewRejected = false;
    try {
        noDepth.draw(context, lightingMesh, *whiteView, identity, {}, {}, textureArrayView);
    } catch (const std::invalid_argument&) {
        invalidNormalMapViewRejected = true;
    }
    require(invalidNormalMapViewRejected, "Diligent renderer accepted a texture-array normal map");

    DiligentPointLight frontPointLight;
    frontPointLight.positionView = {0.0F, 0.0F, 1.0F};
    frontPointLight.color = {1.0F, 0.0F, 0.0F};
    frontPointLight.radius = 2.0F;
    frontPointLight.intensity = 1.0F;
    noDepth.setPointLights(context, std::span{&frontPointLight, 1});
    DiligentMaterialState dimMaterial;
    dimMaterial.baseColor = {0.2F, 0.2F, 0.2F, 1.0F};
    const auto pointLitColor = readLitCenter({}, dimMaterial);
    require(pointLitColor[0] > pointLitColor[1] + 30,
            "Diligent point lighting did not add view-space front-facing illumination");

    DiligentPointLight backPointLight = frontPointLight;
    backPointLight.positionView = {0.0F, 0.0F, -1.0F};
    noDepth.setPointLights(context, std::span{&backPointLight, 1});
    const auto backPointLitColor = readLitCenter({}, dimMaterial);
    require(backPointLitColor[0] < pointLitColor[0] - 25,
            "Diligent point lighting did not attenuate a back-facing surface");

    std::array<DiligentPointLight, 2> multiplePointLights = {
        frontPointLight,
        frontPointLight,
    };
    multiplePointLights[1].color = {0.0F, 0.0F, 1.0F};
    noDepth.setPointLights(context, multiplePointLights);
    const auto multiplePointLitColor = readLitCenter({}, dimMaterial);
    require(multiplePointLitColor[0] > multiplePointLitColor[1] + 30 &&
                multiplePointLitColor[2] > multiplePointLitColor[1] + 30,
            "Diligent point-light buffer did not render multiple colored lights");

    DiligentPointLight transformedPointLight = frontPointLight;
    transformedPointLight.positionView = {0.0F, 0.0F, 0.5F};
    transformedPointLight.radius = 0.75F;
    transformedPointLight.intensity = 4.0F;
    noDepth.setPointLights(context, std::span{&transformedPointLight, 1});
    const auto untransformedPointColor = readLitCenter({}, dimMaterial);
    DiligentLightingParameters translatedObject;
    translatedObject.objectToView[14] = 0.5;
    const auto transformedPointColor = readLitCenter(translatedObject, dimMaterial);
    require(transformedPointColor[0] > untransformedPointColor[0] + 30,
            "Diligent point lighting did not use the supplied object-to-view transform");
    noDepth.setPointLights(context, {});

    DiligentMaterialState interiorLightMaterial = dimMaterial;
    interiorLightMaterial.useInteriorLights = true;
    interiorLightMaterial.interiorLightCount = 1;
    interiorLightMaterial.interiorLightPositionsView[0] = {0.0F, 0.0F, 1.0F};
    interiorLightMaterial.interiorLightColors[0] = {0.0F, 1.0F, 0.0F};
    interiorLightMaterial.interiorLightStrengths[0] = 4.0F;
    const auto interiorLitColor = readLitCenter({}, interiorLightMaterial);
    require(interiorLitColor[1] > interiorLitColor[0] + 30,
            "Diligent vehicle interior lighting did not add view-space material illumination");

    DiligentPointLight greenPointLight = frontPointLight;
    greenPointLight.color = {0.0F, 1.0F, 0.0F};
    noDepth.setPointLights(context, std::span{&greenPointLight, 1});
    DiligentMaterialState litNightmapMaterial;
    litNightmapMaterial.nightMap = redView;
    const auto nightmapAndPointLightColor = readLitCenter({}, litNightmapMaterial);
    require(nightmapAndPointLightColor[1] > 25,
            "Diligent nightmap incorrectly overwrote later dynamic point-light illumination");
    noDepth.setPointLights(context, {});

    DiligentSpotLight frontSpotLight;
    frontSpotLight.positionView = {0.0F, 0.0F, 1.0F};
    frontSpotLight.directionView = {0.0F, 0.0F, -1.0F};
    frontSpotLight.color = {0.0F, 1.0F, 0.0F};
    frontSpotLight.range = 3.0F;
    frontSpotLight.intensity = 10.0F;
    noDepth.setSpotLights(context, std::span{&frontSpotLight, 1});
    const auto spotLitColor = readLitCenter({}, dimMaterial);
    require(spotLitColor[1] > spotLitColor[0] + 10,
            "Diligent spotlight did not illuminate within its inner cone");

    DiligentSpotLight outsideCone = frontSpotLight;
    outsideCone.directionView = {0.0F, 0.0F, 1.0F};
    noDepth.setSpotLights(context, std::span{&outsideCone, 1});
    const auto outsideConeColor = readLitCenter({}, dimMaterial);
    require(outsideConeColor[1] < spotLitColor[1] - 10,
            "Diligent spotlight illuminated a surface outside its cone");

    std::array<DiligentSpotLight, 2> multipleSpotLights = {
        frontSpotLight,
        frontSpotLight,
    };
    multipleSpotLights[1].color = {1.0F, 0.0F, 0.0F};
    noDepth.setSpotLights(context, multipleSpotLights);
    const auto multipleSpotLitColor = readLitCenter({}, dimMaterial);
    require(multipleSpotLitColor[0] > multipleSpotLitColor[2] + 10 &&
                multipleSpotLitColor[1] > multipleSpotLitColor[2] + 10,
            "Diligent spotlight buffer did not render multiple colored lights");
    noDepth.setSpotLights(context, {});

    auto repeatedVertices = nearVertices;
    for (auto& vertex : repeatedVertices) {
        vertex.u = -0.25f;
        vertex.v = 1.25f;
    }
    DiligentTexturedMesh repeated(device, repeatedVertices);
    context.SetRenderTargets(1, &targetView, nullptr, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    noDepth.draw(context, repeated, *patternView, identity);
    mapped = readback();
    const bool repeatCorrect = pixelIs(mapped, 32, 32, {0, 255, 0});
    context.UnmapTextureSubresource(staging, 0, 0);
    require(repeatCorrect, "Textured mesh sampler did not repeat UV coordinates");

    std::array<ModelVertex, 4> indexedVertices = {
        fullVertices[2],
        fullVertices[0],
        fullVertices[5],
        fullVertices[1],
    };
    std::array<std::uint32_t, 6> indices = {1, 3, 0, 1, 0, 2};
    DiligentTexturedMesh indexed(device, indexedVertices, indices);
    indexedVertices[0].x = 100.0f;
    indices.fill(0);

    const auto rejectsIndices = [&](std::span<const ModelVertex> vertices,
                                    std::span<const std::uint32_t> candidate) {
        try {
            DiligentTexturedMesh invalid(device, vertices, candidate);
        } catch (const std::invalid_argument&) {
            return true;
        }
        return false;
    };
    constexpr std::array<std::uint32_t, 3> validIndices = {0, 1, 2};
    constexpr std::array<std::uint32_t, 3> pastEnd = {0, 1, 4};
    constexpr std::array<std::uint32_t, 3> maxIndex = {
        0,
        1,
        std::numeric_limits<std::uint32_t>::max(),
    };
    require(rejectsIndices({}, validIndices), "Indexed mesh accepted an empty vertex list");
    require(rejectsIndices(indexedVertices, {}), "Indexed mesh accepted an empty index list");
    require(rejectsIndices(indexedVertices, std::span{validIndices}.first(2)),
            "Indexed mesh accepted an incomplete triangle");
    require(rejectsIndices(indexedVertices, pastEnd),
            "Indexed mesh accepted an index at the vertex count");
    require(rejectsIndices(indexedVertices, maxIndex), "Indexed mesh accepted an extreme index");

    context.SetRenderTargets(1, &targetView, nullptr, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    noDepth.draw(context, indexed, *patternView, identity);
    noDepth.draw(context, near, *magentaView, identity);
    mapped = readback();
    const bool indexedCorrect =
        pixelIs(mapped, 19, 19, {255, 0, 0}) && pixelIs(mapped, 44, 19, {0, 255, 0}) &&
        pixelIs(mapped, 19, 44, {0, 0, 255}) && pixelIs(mapped, 44, 44, {255, 255, 255}) &&
        pixelIs(mapped, 32, 32, {255, 0, 255});
    context.UnmapTextureSubresource(staging, 0, 0);
    require(indexedCorrect,
            "Indexed geometry, immutable uploads or subsequent non-indexed draw failed");

    context.SetRenderTargets(1, &targetView, depthView, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearDepthStencil(depthView, CLEAR_DEPTH_FLAG, 1, 0,
                              RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    renderer.draw(context, indexed, *patternView, identity);
    renderer.draw(context, far, *cyanView, identity);
    renderer.draw(context, near, *magentaView, identity);
    mapped = readback();
    const bool indexedDepthCorrect =
        pixelIs(mapped, 19, 19, {255, 0, 0}) && pixelIs(mapped, 32, 32, {255, 0, 255});
    context.UnmapTextureSubresource(staging, 0, 0);
    require(indexedDepthCorrect, "Indexed and non-indexed depth testing disagreed");

    std::vector<ModelVertex> largeVertices(65539);
    largeVertices[65536] = {-0.8f, -0.8f, 0, 0, 0, 0, 0, 0, 1};
    largeVertices[65537] = {0.8f, -0.8f, 0, 0, 0, 0, 0, 0, 1};
    largeVertices[65538] = {0.0f, 0.8f, 0, 0, 0, 0, 0, 0, 1};
    constexpr std::array<std::uint32_t, 3> largeIndices = {65536, 65537, 65538};
    DiligentTexturedMesh largeMesh(device, largeVertices, largeIndices);
    context.SetRenderTargets(1, &targetView, nullptr, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    noDepth.draw(context, largeMesh, *magentaView, identity);
    mapped = readback();
    const bool largeIndexCorrect =
        pixelIs(mapped, 32, 32, {255, 0, 255}) && pixelIs(mapped, 1, 1, {0, 0, 0});
    context.UnmapTextureSubresource(staging, 0, 0);
    require(largeIndexCorrect, "Indexed mesh did not use 32-bit vertex offsets");

    using namespace openbus::map;
    MapSplinePlacement spline;
    spline.geometryValid = true;
    spline.length = 10.0;
    MapSplineProfileSection section;
    section.points = {{-2.0, 0.0, 0.0, 0.1}, {2.0, 0.0, 1.0, 0.1}};
    const auto samples = tessellateMapSpline(0.0, 0.0, 0.0, spline.length, 0.0);
    RoadMeshGeometry road;
    appendMapRoadProfile(road, spline, samples, section, {}, std::nullopt, {});
    require(!road.vertices.empty() && road.indices.size() > road.vertices.size() &&
                road.minX < road.maxX && road.minY < road.maxY,
            "Generated road did not produce indexed geometry");
    DiligentTexturedMesh roadMesh(device, road.vertices, road.indices);
    Matrix4 roadProjection = identity;
    roadProjection[0] = 0.4;
    roadProjection[5] = 0.16;
    roadProjection[13] = -0.8;
    context.SetRenderTargets(1, &targetView, nullptr, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    noDepth.draw(context, near, *magentaView, identity);
    noDepth.draw(context, roadMesh, *cyanView, roadProjection);
    mapped = readback();
    const bool roadCorrect =
        pixelIs(mapped, 32, 32, {0, 255, 255}) && pixelIs(mapped, 32, 12, {0, 255, 255}) &&
        pixelIs(mapped, 32, 52, {0, 255, 255}) && pixelIs(mapped, 1, 1, {0, 0, 0});
    context.UnmapTextureSubresource(staging, 0, 0);
    require(roadCorrect, "Generated indexed road did not render as expected");

    const auto terrain = prepareMapTerrainMesh(TerrainGrid{1, {0.0F, 0.0F, 0.0F, 0.0F}}, 2.0);
    DiligentTexturedMesh terrainMesh(device, terrain.vertices, terrain.indices);
    Matrix4 surfaceProjection = identity;
    surfaceProjection[0] = 0.8;
    surfaceProjection[5] = 0.8;
    surfaceProjection[12] = -0.8;
    surfaceProjection[13] = -0.8;
    context.SetRenderTargets(1, &targetView, nullptr, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    noDepth.draw(context, terrainMesh, *patternView, surfaceProjection);
    mapped = readback();
    const bool terrainCorrect =
        pixelIs(mapped, 19, 19, {0, 0, 255}) && pixelIs(mapped, 44, 19, {255, 255, 255}) &&
        pixelIs(mapped, 19, 44, {255, 0, 0}) && pixelIs(mapped, 44, 44, {0, 255, 0}) &&
        pixelIs(mapped, 1, 1, {0, 0, 0});
    context.UnmapTextureSubresource(staging, 0, 0);
    require(terrainCorrect, "Production-prepared indexed terrain did not render correctly");

    const auto water = prepareMapWaterMesh(WaterData{{WaterSurface{{0, 0, 0, 0}}}}, -2, 3, 2.0);
    DiligentTexturedMesh waterMesh(device, water.vertices, water.indices);
    context.SetRenderTargets(1, &targetView, nullptr, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    noDepth.draw(context, waterMesh, *patternView, surfaceProjection);
    mapped = readback();
    const bool waterCorrect =
        pixelIs(mapped, 19, 19, {0, 0, 255}) && pixelIs(mapped, 44, 19, {255, 255, 255}) &&
        pixelIs(mapped, 19, 44, {255, 0, 0}) && pixelIs(mapped, 44, 44, {0, 255, 0}) &&
        pixelIs(mapped, 1, 1, {0, 0, 0});
    context.UnmapTextureSubresource(staging, 0, 0);
    require(waterCorrect, "Production-prepared indexed water did not render correctly");

    DiligentMaterialState cutout;
    cutout.alphaMode = DiligentAlphaMode::Cutout;
    context.SetRenderTargets(1, &targetView, depthView, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearDepthStencil(depthView, CLEAR_DEPTH_FLAG, 1, 0,
                              RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    renderer.draw(context, full, *magentaView, identity);
    renderer.draw(context, full, *transparentView, identity, cutout);
    mapped = readback();
    const bool cutoutCorrect = pixelIs(mapped, 32, 32, {255, 0, 255});
    context.UnmapTextureSubresource(staging, 0, 0);
    require(cutoutCorrect, "Cutout alpha did not preserve the underlying surface");

    DiligentMaterialState noZwriteBlend;
    noZwriteBlend.blendEnabled = true;
    noZwriteBlend.depthWrite = false;
    noZwriteBlend.baseColor[3] = 0.5F;
    context.SetRenderTargets(1, &targetView, nullptr, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    noDepth.draw(context, full, *magentaView, identity, noZwriteBlend);
    mapped = readback();
    const auto* noZwriteBlendPixel =
        static_cast<const std::uint8_t*>(mapped.pData) + 32 * mapped.Stride + 32 * 4;
    const bool noZwriteOpaqueMaterialBlended = noZwriteBlendPixel[0] >= 126 &&
                                               noZwriteBlendPixel[0] <= 129 &&
                                               noZwriteBlendPixel[1] == 0 &&
                                               noZwriteBlendPixel[2] >= 126 &&
                                               noZwriteBlendPixel[2] <= 129;
    context.UnmapTextureSubresource(staging, 0, 0);
    require(noZwriteOpaqueMaterialBlended,
            "Diligent no-Z-write opaque material did not use source-alpha blending");

    DiligentMaterialState noZwriteCutout;
    noZwriteCutout.alphaMode = DiligentAlphaMode::Cutout;
    noZwriteCutout.blendEnabled = true;
    noZwriteCutout.depthWrite = false;
    context.SetRenderTargets(1, &targetView, nullptr, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    noDepth.draw(context, full, *translucentView, identity, noZwriteCutout);
    mapped = readback();
    const auto* noZwriteCutoutPixel =
        static_cast<const std::uint8_t*>(mapped.pData) + 32 * mapped.Stride + 32 * 4;
    const bool noZwriteCutoutBlended = noZwriteCutoutPixel[0] < 5 &&
                                       noZwriteCutoutPixel[1] >= 126 &&
                                       noZwriteCutoutPixel[1] <= 129 &&
                                       noZwriteCutoutPixel[2] < 5;
    context.UnmapTextureSubresource(staging, 0, 0);
    require(noZwriteCutoutBlended,
            "Diligent no-Z-write cutout did not preserve its alpha cutoff and blend");

    context.SetRenderTargets(1, &targetView, depthView, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearDepthStencil(depthView, CLEAR_DEPTH_FLAG, 1, 0,
                              RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    renderer.draw(context, full, *magentaView, identity);
    renderer.draw(context, near, *translucentView, identity, cutout);
    mapped = readback();
    const bool cutoutOpaquePixelCorrect = pixelIs(mapped, 32, 32, {0, 255, 0});
    context.UnmapTextureSubresource(staging, 0, 0);
    require(cutoutOpaquePixelCorrect, "Cutout alpha incorrectly discarded a fragment above 0.5");

    DiligentMaterialState tintedMaterial;
    tintedMaterial.baseColor = {0.5F, 0.25F, 1.0F, 1.0F};
    context.SetRenderTargets(1, &targetView, nullptr, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    noDepth.draw(context, near, *magentaView, identity, tintedMaterial);
    mapped = readback();
    const auto* tintedPixel =
        static_cast<const std::uint8_t*>(mapped.pData) + 32 * mapped.Stride + 32 * 4;
    const bool tintCorrect = tintedPixel[0] >= 126 && tintedPixel[0] <= 129 &&
                             tintedPixel[1] < 10 && tintedPixel[2] > 240;
    context.UnmapTextureSubresource(staging, 0, 0);
    require(tintCorrect, "Base material color did not multiply the sampled texture");

    DiligentMaterialState fadedCutout;
    fadedCutout.alphaMode = DiligentAlphaMode::Cutout;
    fadedCutout.visibilityFade = 0.5F;
    context.SetRenderTargets(1, &targetView, depthView, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearDepthStencil(depthView, CLEAR_DEPTH_FLAG, 1, 0,
                              RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    renderer.draw(context, full, *magentaView, identity);
    renderer.draw(context, near, *translucentView, identity, fadedCutout);
    mapped = readback();
    const bool fadedCutoutCorrect = pixelIs(mapped, 32, 32, {255, 0, 255});
    context.UnmapTextureSubresource(staging, 0, 0);
    require(fadedCutoutCorrect, "Visibility fade was not applied to cutout alpha");

    fadedCutout.visibilityFade = 2.0F;
    context.SetRenderTargets(1, &targetView, depthView, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearDepthStencil(depthView, CLEAR_DEPTH_FLAG, 1, 0,
                              RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    renderer.draw(context, full, *magentaView, identity);
    renderer.draw(context, near, *translucentView, identity, fadedCutout);
    mapped = readback();
    const bool clampedFadeCorrect = pixelIs(mapped, 32, 32, {0, 255, 0});
    context.UnmapTextureSubresource(staging, 0, 0);
    require(clampedFadeCorrect, "Visibility fade was not clamped to the OpenGL range");

    context.SetRenderTargets(1, &targetView, depthView, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearDepthStencil(depthView, CLEAR_DEPTH_FLAG, 1, 0,
                              RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    renderer.draw(context, full, *magentaView, identity);
    renderer.draw(context, near, *transparentView, identity);
    mapped = readback();
    const bool opaqueCorrect = pixelIs(mapped, 32, 32, {0, 255, 0});
    context.UnmapTextureSubresource(staging, 0, 0);
    require(opaqueCorrect, "Opaque material incorrectly discarded or blended zero-alpha pixels");

    DiligentMaterialState nearOpaque;
    nearOpaque.alphaMode = DiligentAlphaMode::NearOpaque;
    context.SetRenderTargets(1, &targetView, depthView, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearDepthStencil(depthView, CLEAR_DEPTH_FLAG, 1, 0,
                              RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    renderer.draw(context, full, *magentaView, identity);
    renderer.draw(context, full, *translucentView, identity, nearOpaque);
    mapped = readback();
    const bool nearOpaqueCorrect = pixelIs(mapped, 32, 32, {255, 0, 255});
    context.UnmapTextureSubresource(staging, 0, 0);
    require(nearOpaqueCorrect, "Near-opaque alpha did not apply its 0.99 cutoff");

    DiligentMaterialState blend;
    blend.alphaMode = DiligentAlphaMode::Blend;
    blend.depthWrite = false;
    context.SetRenderTargets(1, &targetView, depthView, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearDepthStencil(depthView, CLEAR_DEPTH_FLAG, 1, 0,
                              RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    renderer.draw(context, full, *magentaView, identity);
    renderer.draw(context, near, *translucentView, identity, blend);
    mapped = readback();
    const auto* blendedPixel =
        static_cast<const std::uint8_t*>(mapped.pData) + 32 * mapped.Stride + 32 * 4;
    const bool blendCorrect = blendedPixel[0] >= 126 && blendedPixel[0] <= 129 &&
                              blendedPixel[1] >= 126 && blendedPixel[1] <= 129 &&
                              blendedPixel[2] >= 126 && blendedPixel[2] <= 129;
    context.UnmapTextureSubresource(staging, 0, 0);
    require(blendCorrect, "Alpha blending did not combine source and target colors");

    context.SetRenderTargets(1, &targetView, depthView, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearDepthStencil(depthView, CLEAR_DEPTH_FLAG, 1, 0,
                              RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    renderer.draw(context, near, *translucentView, identity, blend);
    renderer.draw(context, far, *cyanView, identity);
    mapped = readback();
    const bool blendNoDepthWrite = pixelIs(mapped, 32, 32, {0, 255, 255});
    context.UnmapTextureSubresource(staging, 0, 0);
    require(blendNoDepthWrite, "Blended material unexpectedly wrote depth when disabled");

    blend.depthWrite = true;
    context.SetRenderTargets(1, &targetView, depthView, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearDepthStencil(depthView, CLEAR_DEPTH_FLAG, 1, 0,
                              RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    renderer.draw(context, near, *translucentView, identity, blend);
    renderer.draw(context, far, *cyanView, identity);
    mapped = readback();
    const auto* depthWritingBlendPixel =
        static_cast<const std::uint8_t*>(mapped.pData) + 32 * mapped.Stride + 32 * 4;
    const bool blendDepthWrite = depthWritingBlendPixel[1] > 100 && depthWritingBlendPixel[2] < 10;
    context.UnmapTextureSubresource(staging, 0, 0);
    require(blendDepthWrite, "Blended material did not honor its enabled depth writes");

    auto layerOneVertices = nearVertices;
    for (auto& vertex : layerOneVertices) {
        vertex.layer = 1.0F;
    }
    DiligentTexturedMesh layerOneMesh(device, layerOneVertices);
    context.SetRenderTargets(1, &targetView, nullptr, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    noDepth.draw(context, layerOneMesh, textureArray, identity);
    mapped = readback();
    const bool arrayLayerCorrect = pixelIs(mapped, 32, 32, {0, 0, 255});
    context.UnmapTextureSubresource(staging, 0, 0);
    require(arrayLayerCorrect, "Textured mesh did not sample the ModelVertex array layer");

    context.SetRenderTargets(1, &targetView, nullptr, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    noDepth.draw(context, layerOneMesh, *textureArrayView, identity);
    mapped = readback();
    const bool rawArrayViewCorrect = pixelIs(mapped, 32, 32, {0, 0, 255});
    context.UnmapTextureSubresource(staging, 0, 0);
    require(rawArrayViewCorrect, "Textured mesh did not accept a native Diligent array view");

    context.SetRenderTargets(1, &targetView, nullptr, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    noDepth.draw(context, near, singleLayerArray, identity);
    mapped = readback();
    const bool singleArrayLayerCorrect = pixelIs(mapped, 32, 32, {255, 0, 0});
    context.UnmapTextureSubresource(staging, 0, 0);
    require(singleArrayLayerCorrect, "Textured mesh did not sample a single-layer array");

    auto invalidLayerVertices = nearVertices;
    invalidLayerVertices.front().layer = 0.5F;
    DiligentTexturedMesh invalidLayerMesh(device, invalidLayerVertices);
    bool invalidArrayCoordinateRejected = false;
    try {
        noDepth.draw(context, invalidLayerMesh, textureArray, identity);
    } catch (const std::invalid_argument&) {
        invalidArrayCoordinateRejected = true;
    }
    require(invalidArrayCoordinateRejected,
            "Textured mesh accepted a fractional texture-array layer coordinate");
    bool missingArrayLayerRejected = false;
    try {
        noDepth.draw(context, layerOneMesh, singleLayerArray, identity);
    } catch (const std::invalid_argument&) {
        missingArrayLayerRejected = true;
    }
    require(missingArrayLayerRejected,
            "Textured mesh accepted an out-of-range texture-array layer coordinate");

    DiligentSceneFrame sceneFrame(device, size, size, TEX_FORMAT_RGBA8_UNORM);
    MotionDataFrame sceneMetadata;
    sceneMetadata.frameId = 1;
    sceneMetadata.timestampSeconds = 1.0;
    sceneMetadata.width = static_cast<int>(size);
    sceneMetadata.height = static_cast<int>(size);
    sceneMetadata.historyValid = true;
    sceneFrame.beginScene(context, sceneMetadata);
    require(!sceneFrame.frameData().historyValid,
            "Diligent scene frame did not invalidate its initial history");
    bool resizeDuringSceneRejected = false;
    try {
        sceneFrame.resize(context, size / 2, size / 2);
    } catch (const std::logic_error&) {
        resizeDuringSceneRejected = true;
    }
    require(resizeDuringSceneRejected,
            "Diligent scene frame accepted resize during scene submission");
    sceneFrame.finishScene(context, *targetView);
    bool beginBeforeCompletionRejected = false;
    try {
        sceneFrame.beginScene(context, sceneMetadata);
    } catch (const std::logic_error&) {
        beginBeforeCompletionRejected = true;
    }
    require(beginBeforeCompletionRejected,
            "Diligent scene frame replaced a composited frame before completion");
    sceneFrame.completeFrame();

    sceneMetadata.frameId = 2;
    sceneMetadata.timestampSeconds = 1.016;
    sceneFrame.beginScene(context, sceneMetadata);
    require(sceneFrame.frameData().historyValid,
            "Diligent scene frame discarded valid consecutive-frame history");
    sceneFrame.finishScene(context, *targetView);
    sceneFrame.completeFrame();

    sceneFrame.resize(context, size / 2, size / 2);
    sceneMetadata.frameId = 3;
    sceneMetadata.timestampSeconds = 1.032;
    sceneMetadata.width = static_cast<int>(size / 2);
    sceneMetadata.height = static_cast<int>(size / 2);
    sceneFrame.beginScene(context, sceneMetadata);
    require(!sceneFrame.frameData().historyValid,
            "Diligent scene frame retained temporal history after resize");
    sceneFrame.finishScene(context, *targetView);
    sceneFrame.completeFrame();

    auto mapTexture = std::make_shared<Image>(Image{1, 1, {220, 180, 120, 255}});
    openbus::map::MapTerrainMesh terrainGeometry;
    terrainGeometry.vertices.assign(nearVertices.begin(), nearVertices.end());
    terrainGeometry.indices = {0, 1, 2, 3, 4, 5};
    terrainGeometry.minimumHeight = -2.0F;
    terrainGeometry.maximumHeight = 5.0F;
    openbus::map::MapWaterMesh waterGeometry;
    waterGeometry.vertices.assign(nearVertices.begin(), nearVertices.end());
    waterGeometry.indices = {0, 1, 2, 3, 4, 5};
    waterGeometry.minimumHeight = -1.0;
    waterGeometry.maximumHeight = 1.0;
    waterGeometry.meanHeight = 0.25;
    openbus::map::RoadMeshGeometry roadGeometry;
    roadGeometry.vertices.assign(nearVertices.begin(), nearVertices.end());
    roadGeometry.indices = {0, 1, 2, 3, 4, 5};
    roadGeometry.tileX = 7;
    roadGeometry.tileY = -2;
    roadGeometry.minX = 2100.0;
    roadGeometry.minY = -600.0;
    roadGeometry.maxX = 2102.0;
    roadGeometry.maxY = -598.0;
    roadGeometry.texturePath = "road.tga";
    roadGeometry.minZ = -0.5;
    roadGeometry.maxZ = 0.5;
    roadGeometry.alphaMode = 2;
    roadGeometry.textured = true;
    roadGeometry.supportsMoisture = true;
    roadGeometry.weatherClassified = true;
    std::array<DiligentMapRoadSource, 2> roadSources = {{
        {&roadGeometry, mapTexture, {}},
        {&roadGeometry, mapTexture, {}},
    }};
    DiligentMapTileAsset mapTile(device, 7, -2, &terrainGeometry, mapTexture, &waterGeometry,
                                 mapTexture, roadSources);
    const Matrix4 mapTileTransform = mapTile.worldTransform(300.0);
    Matrix4 mapViewProjection = identity;
    mapViewProjection[0] = 2.0;
    mapViewProjection[5] = 3.0;
    mapViewProjection[12] = 10.0;
    mapViewProjection[13] = -5.0;
    const Matrix4 mapTileMvp = mapTile.modelViewProjection(mapViewProjection, 300.0);
    require(mapTile.tileX() == 7 && mapTile.tileY() == -2 && mapTile.terrain() != nullptr &&
                mapTileTransform[12] == 2100.0 && mapTileTransform[13] == -600.0 &&
                mapTileMvp[12] == 4210.0 && mapTileMvp[13] == -1805.0 &&
                mapTile.water() != nullptr && mapTile.roads().size() == roadSources.size() &&
                mapTile.roads()[0].tileX == 7 && mapTile.roads()[0].tileY == -2 &&
                mapTile.roads()[0].texturePath == std::filesystem::path("road.tga") &&
                mapTile.roads()[0].supportsMoisture && mapTile.roads()[0].weatherClassified &&
                mapTile.roads()[0].material.alphaMode == DiligentAlphaMode::Blend &&
                mapTile.terrain()->texture == mapTile.water()->texture &&
                mapTile.terrain()->texture == mapTile.roads()[0].texture,
            "Diligent map tile did not preserve geometry metadata or share repeated textures");
    bool invalidMapTileSizeRejected = false;
    try {
        static_cast<void>(mapTile.worldTransform(0.0));
    } catch (const std::invalid_argument&) {
        invalidMapTileSizeRejected = true;
    }
    require(invalidMapTileSizeRejected,
            "Diligent map tile accepted an invalid world-transform tile size");
    auto mipMapTexture =
        std::make_shared<Image>(Image{4, 4, std::vector<std::uint8_t>(4 * 4 * 4, 255)});
    const std::array<DiligentMapRoadSource, 1> mipRoadSources = {{
        {&roadGeometry, mipMapTexture, {}},
    }};
    DiligentMapTileAsset mipMapTile(device, 7, -2, &terrainGeometry, mipMapTexture,
                                    &waterGeometry, mipMapTexture, mipRoadSources, true);
    require(mipMapTile.terrain()->texture->shaderResourceView().GetTexture()->GetDesc().MipLevels ==
                    3 &&
                mipMapTile.water()->texture->shaderResourceView().GetTexture()->GetDesc().MipLevels ==
                    3 &&
                mipMapTile.roads()[0].texture->shaderResourceView().GetTexture()->GetDesc()
                        .MipLevels == 3,
            "Diligent map tile did not generate mips for terrain, water and road textures");

    auto terrainOverlay = std::make_shared<Image>(Image{1, 1, {80, 120, 160, 255}});
    auto terrainTransmap = std::make_shared<Image>(Image{1, 1, {255, 255, 255, 128}});
    auto terrainDetail = std::make_shared<Image>(Image{1, 1, {200, 210, 220, 255}});
    std::array<DiligentMapTerrainLayerSource, 2> terrainLayerSources = {{
        {mapTexture, {}, {}, {}},
        {terrainOverlay, terrainTransmap, terrainDetail, {}},
    }};
    terrainLayerSources[0].material.textureScale = {2.0F, 2.0F};
    terrainLayerSources[1].material.alphaMode = DiligentAlphaMode::Blend;
    terrainLayerSources[1].material.transMapScale = {4.0F, 3.0F};
    terrainLayerSources[1].material.flipTransmapY = true;
    terrainLayerSources[1].material.detailMapScale = {8.0F, 6.0F};
    DiligentMapTileAsset layeredMapTile(device, 7, -2, &terrainGeometry, terrainLayerSources,
                                        nullptr, {}, {});
    const auto layeredTerrain = layeredMapTile.terrainLayers();
    require(layeredTerrain.size() == terrainLayerSources.size() &&
                layeredTerrain[0].mesh == layeredTerrain[1].mesh &&
                layeredTerrain[0].texture != layeredTerrain[1].texture &&
                layeredTerrain[1].transMapTexture != nullptr &&
                layeredTerrain[1].detailMapTexture != nullptr &&
                layeredTerrain[1].material.alphaMode == DiligentAlphaMode::Blend &&
                layeredTerrain[1].material.transMap ==
                    &layeredTerrain[1].transMapTexture->shaderResourceView() &&
                layeredTerrain[1].material.detailMap ==
                    &layeredTerrain[1].detailMapTexture->shaderResourceView() &&
                layeredTerrain[1].material.transMapScale ==
                    terrainLayerSources[1].material.transMapScale &&
                layeredTerrain[1].material.detailMapScale ==
                    terrainLayerSources[1].material.detailMapScale &&
                layeredTerrain[1].material.flipTransmapY,
            "Diligent map tile did not preserve ordered terrain layer material data");
    DiligentMapTerrainLayerSource mipTerrainLayer;
    mipTerrainLayer.texture = mipMapTexture;
    mipTerrainLayer.transMap = mipMapTexture;
    mipTerrainLayer.detailMap = mipMapTexture;
    const std::array<DiligentMapTerrainLayerSource, 1> mipTerrainLayers = {mipTerrainLayer};
    DiligentMapTileAsset mipLayeredMapTile(device, 7, -2, &terrainGeometry, mipTerrainLayers,
                                           nullptr, {}, {}, true);
    const auto& mipLayer = mipLayeredMapTile.terrainLayers()[0];
    require(mipLayer.texture->shaderResourceView().GetTexture()->GetDesc().MipLevels == 3 &&
                mipLayer.transMapTexture->shaderResourceView().GetTexture()->GetDesc().MipLevels ==
                    3 &&
                mipLayer.detailMapTexture->shaderResourceView().GetTexture()->GetDesc().MipLevels ==
                    3,
            "Diligent layered map tile did not generate mips for its material textures");
    const auto sampleTerrainPixel = [&] {
        const MappedTextureSubresource mapped = readback();
        const auto* pixel = static_cast<const std::uint8_t*>(mapped.pData) +
                            (size / 2) * mapped.Stride + (size / 2) * 4;
        const std::array<std::uint8_t, 3> color = {pixel[0], pixel[1], pixel[2]};
        context.UnmapTextureSubresource(staging, 0, 0);
        return color;
    };
    const auto clearTerrainTarget = [&] {
        context.SetRenderTargets(1, &targetView, nullptr, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        context.ClearRenderTarget(targetView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    };
    const auto terrainFilePath = textureDirectory / "diligent-terrain.terrain";
    std::array<std::uint8_t, 20> terrainFileBytes = {};
    const auto writeLittleEndianU32 = [&](std::size_t offset, std::uint32_t value) {
        terrainFileBytes[offset] = static_cast<std::uint8_t>(value & 0xFFU);
        terrainFileBytes[offset + 1] = static_cast<std::uint8_t>((value >> 8U) & 0xFFU);
        terrainFileBytes[offset + 2] = static_cast<std::uint8_t>((value >> 16U) & 0xFFU);
        terrainFileBytes[offset + 3] = static_cast<std::uint8_t>((value >> 24U) & 0xFFU);
    };
    writeLittleEndianU32(0, 1);
    for (std::size_t index = 0; index < 4; ++index) {
        writeLittleEndianU32(4 + index * 4, std::bit_cast<std::uint32_t>(0.0F));
    }
    {
        std::ofstream output(terrainFilePath, std::ios::binary);
        output.write(reinterpret_cast<const char*>(terrainFileBytes.data()),
                     static_cast<std::streamsize>(terrainFileBytes.size()));
        require(static_cast<bool>(output), "Failed to write Diligent map terrain fixture");
    }
    const auto waterDirectory = textureDirectory / "texture";
    std::filesystem::create_directory(waterDirectory);
    const auto waterTexturePath = waterDirectory / "water.tga";
    std::filesystem::copy_file(imagePath, waterTexturePath,
                               std::filesystem::copy_options::overwrite_existing);
    const auto waterFilePath = textureDirectory / "diligent-water.water";
    {
        std::ofstream output(waterFilePath, std::ios::binary);
        output.write(reinterpret_cast<const char*>(terrainFileBytes.data()),
                     static_cast<std::streamsize>(terrainFileBytes.size()));
        require(static_cast<bool>(output), "Failed to write Diligent map water fixture");
    }
    const auto splineDirectory = textureDirectory / "Splines";
    std::filesystem::create_directories(splineDirectory);
    const auto splineTexturePath = splineDirectory / "road.tga";
    std::filesystem::copy_file(imagePath, splineTexturePath,
                               std::filesystem::copy_options::overwrite_existing);
    const auto splineProfilePath = splineDirectory / "road.sli";
    {
        std::ofstream output(splineProfilePath);
        output << "[texture]\nroad.tga\n"
                  "[matl_alpha]\n1\n"
                  "[profile]\n0\n"
                  "[profilepnt]\n-1\n0\n0\n0\n"
                  "[profilepnt]\n1\n0\n1\n0\n";
        require(static_cast<bool>(output), "Failed to write Diligent road profile fixture");
    }
    const auto roadMapTextPath = textureDirectory / "diligent-road.map";
    {
        std::ofstream output(roadMapTextPath);
        output << "Diligent road tile\n[version]\n14\n[terrain]\n"
                  "[spline]\n0\nSplines\\road.sli\n1\n0\n0\n0\n0\n0\n0\n"
                  "10\n0\n0\n0\n0\n0\n0\n0\n0\n\n";
        require(static_cast<bool>(output), "Failed to write Diligent map spline fixture");
    }
    openbus::map::MapDefinition fileBackedMap;
    fileBackedMap.rootPath = textureDirectory;
    fileBackedMap.groundTextures = {{"decoded-pattern.tga", "", {}},
                                    {"decoded-pattern.tga", "", {}}};
    openbus::map::MapTileReference fileBackedTile;
    fileBackedTile.x = 7;
    fileBackedTile.y = -2;
    fileBackedTile.terrainPath = terrainFilePath;
    fileBackedTile.hasTerrainFile = true;
    fileBackedTile.waterPath = waterFilePath;
    fileBackedTile.hasWaterFile = true;
    fileBackedTile.groundTextureSidecars.push_back({"1", imagePath});
    fileBackedMap.tiles.push_back(fileBackedTile);
    openbus::map::MapDefinition roadFileBackedMap = fileBackedMap;
    roadFileBackedMap.tiles[0].textPath = roadMapTextPath;
    const openbus::map::MapTileData roadMapData =
        openbus::map::loadMapTile(roadFileBackedMap.tiles[0]);
    require(roadMapData.splines.size() == 1 && roadMapData.splines[0].geometryValid,
            "Diligent map road fixture did not parse a valid spline");
    const openbus::map::MapSplineProfile roadProfile =
        openbus::map::loadMapSplineProfile(splineProfilePath);
    auto roadConnectionIndex =
        std::make_shared<openbus::map::MapRoadConnectionIndex>(roadFileBackedMap.coordinates);
    roadConnectionIndex->addSpline(
        roadMapData.splines[0], roadFileBackedMap.tiles[0].x, roadFileBackedMap.tiles[0].y,
        openbus::map::mapSplineProfileCantHalfWidth(roadProfile));
    roadConnectionIndex->finalize();
    DiligentMapTileSet fileBackedTileSet(device, fileBackedMap.coordinates.tileSize);
    require(fileBackedTileSet.updateTerrainWindow(fileBackedMap, 2101.0, -599.0, 0,
                                                  textureDirectory, 0) == 1,
            "Diligent map tile set did not load selected terrain from map metadata");
    const auto fileBackedLayers = fileBackedTileSet.activeTiles()[0]->terrainLayers();
    require(fileBackedLayers.size() == 2 && fileBackedLayers[0].mesh == fileBackedLayers[1].mesh &&
                fileBackedLayers[0].texture == fileBackedLayers[1].texture &&
                fileBackedLayers[1].transMapTexture != nullptr &&
                fileBackedLayers[1].material.flipTransmapY &&
                fileBackedLayers[1].material.transMapScale[0] == 1.0F &&
                fileBackedLayers[1].material.transMapScale[1] == 1.0F &&
                fileBackedTileSet.activeTiles()[0]->water() != nullptr &&
                fileBackedTileSet.activeTiles()[0]->water()->texture != nullptr &&
                fileBackedTileSet.activeTiles()[0]->water()->material.alphaMode ==
                    DiligentAlphaMode::Blend &&
                fileBackedTileSet.activeTiles()[0]->water()->material.baseColor[3] == 0.62F &&
                !fileBackedTileSet.activeTiles()[0]->water()->material.depthWrite,
            "Diligent file-backed terrain did not load layered terrain and transparent water");
    require(fileBackedTileSet.updateTerrainWindow(fileBackedMap, 2101.0, -599.0, 0,
                                                  textureDirectory, 0) == 0,
            "Diligent map tile set redundantly reloaded a cached map tile");
    DiligentMapTileSet roadFileBackedTileSet(device, roadFileBackedMap.coordinates.tileSize);
    require(roadFileBackedTileSet.updateTerrainWindow(
                roadFileBackedMap, 2101.0, -599.0, 0, textureDirectory, 0, roadConnectionIndex) ==
                1 &&
                roadFileBackedTileSet.activeTiles()[0]->roads().size() == 1 &&
                roadFileBackedTileSet.activeTiles()[0]->roads()[0].texture != nullptr &&
                roadFileBackedTileSet.activeTiles()[0]->roads()[0].alphaMode ==
                    static_cast<int>(DiligentAlphaMode::Cutout),
            "Diligent file-backed map tile did not prepare and upload its spline-profile road");
    auto changedRoadConnectionIndex =
        std::make_shared<openbus::map::MapRoadConnectionIndex>(roadFileBackedMap.coordinates);
    auto changedRoadSpline = roadMapData.splines[0];
    changedRoadSpline.length += 1.0;
    changedRoadConnectionIndex->addSpline(
        changedRoadSpline, roadFileBackedMap.tiles[0].x, roadFileBackedMap.tiles[0].y,
        openbus::map::mapSplineProfileCantHalfWidth(roadProfile));
    changedRoadConnectionIndex->finalize();
    require(roadFileBackedTileSet.updateTerrainWindow(
                roadFileBackedMap, 2101.0, -599.0, 0, textureDirectory, 0,
                changedRoadConnectionIndex) == 1,
            "Diligent map tile set did not invalidate tiles when road joins changed");
    require(roadFileBackedTileSet.updateTerrainWindow(
                roadFileBackedMap, 2101.0, -599.0, 0, textureDirectory, 0, roadConnectionIndex) ==
                1,
            "Diligent map tile set did not restore the original road source");
    const auto roadOverlayPath = textureDirectory / "active-chrono-road.map";
    {
        std::ofstream output(roadOverlayPath);
        output << "Diligent Chrono road overlay\n[version]\n14\n";
        require(static_cast<bool>(output), "Failed to write Diligent Chrono road overlay fixture");
    }
    openbus::map::MapDefinition chronoRoadMap = roadFileBackedMap;
    const openbus::map::MapTileReference& roadTile = roadFileBackedMap.tiles[0];
    chronoRoadMap.tiles[0].chronoBaseAssets = openbus::map::MapChronoBaseAssets{
        roadTile.textPath,          roadTile.terrainPath,          roadTile.hasTerrainFile,
        roadTile.waterPath,         roadTile.hasWaterFile,         roadTile.lightmapPath,
        roadTile.hasLightmapFile,   roadTile.groundTextureSidecars};
    chronoRoadMap.tiles[0].chronoOverlayPaths.push_back(roadOverlayPath);
    require(roadFileBackedTileSet.updateTerrainWindow(
                chronoRoadMap, 2101.0, -599.0, 0, textureDirectory, 0, roadConnectionIndex) == 1,
            "Diligent map tile set did not invalidate a changed Chrono road overlay list");
    openbus::map::MapDefinition chronoTerrainMap = fileBackedMap;
    const auto chronoTerrainPath = textureDirectory / "active-chrono.terrain";
    std::filesystem::copy_file(terrainFilePath, chronoTerrainPath,
                               std::filesystem::copy_options::overwrite_existing);
    chronoTerrainMap.tiles[0].terrainPath = chronoTerrainPath;
    chronoTerrainMap.tiles[0].chronoBaseAssets = openbus::map::MapChronoBaseAssets{};
    chronoTerrainMap.tiles[0].chronoOverlayPaths.push_back(textureDirectory / "active-chrono.map");
    require(fileBackedTileSet.updateTerrainWindow(chronoTerrainMap, 2101.0, -599.0, 0,
                                                  textureDirectory, 0) == 1 &&
                fileBackedTileSet.tileCount() == 1 &&
                fileBackedTileSet.activeTiles()[0]->terrainLayers().size() == 2,
            "Diligent map tile set did not replace cached terrain with selected chrono layers");
    require(fileBackedTileSet.updateTerrainWindow(chronoTerrainMap, 2101.0, -599.0, 0,
                                                  textureDirectory, 0) == 0,
            "Diligent map tile set redundantly reloaded an unchanged chrono terrain tile");

    DiligentMapTileSet asyncTileSet(device, fileBackedMap.coordinates.tileSize);
    require(asyncTileSet.requestTerrainWindow(fileBackedMap, 2101.0, -599.0, 0,
                                              textureDirectory, 0) == 1,
            "Diligent map tile set did not schedule CPU terrain preparation");
    std::size_t asynchronouslyPublished = 0;
    const auto asyncDeadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (asynchronouslyPublished == 0 && std::chrono::steady_clock::now() < asyncDeadline) {
        asynchronouslyPublished = asyncTileSet.publishReadyTerrainTiles();
        if (asynchronouslyPublished == 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    require(asynchronouslyPublished == 1 && asyncTileSet.tileCount() == 1 &&
                asyncTileSet.activeTiles().size() == 1 &&
                asyncTileSet.activeTiles()[0]->water() != nullptr,
            "Diligent map tile set did not publish prepared terrain and water on the caller thread");
    DiligentMapTileSet asyncRoadTileSet(device, roadFileBackedMap.coordinates.tileSize);
    require(asyncRoadTileSet.requestTerrainWindow(
                roadFileBackedMap, 2101.0, -599.0, 0, textureDirectory, 0, roadConnectionIndex) ==
                1,
            "Diligent map tile set did not schedule file-backed road preparation");
    std::size_t asyncRoadPublished = 0;
    const auto asyncRoadDeadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (asyncRoadPublished == 0 &&
           std::chrono::steady_clock::now() < asyncRoadDeadline) {
        asyncRoadPublished = asyncRoadTileSet.publishReadyTerrainTiles();
        if (asyncRoadPublished == 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    require(asyncRoadPublished == 1 && asyncRoadTileSet.activeTiles().size() == 1 &&
                asyncRoadTileSet.activeTiles()[0]->roads().size() == 1 &&
                asyncRoadTileSet.activeTiles()[0]->roads()[0].texture != nullptr,
            "Diligent map tile set did not publish asynchronously prepared spline-profile roads");
    require(asyncTileSet.requestTerrainWindow(fileBackedMap, 2101.0, -599.0, 0,
                                              textureDirectory, 0) == 0,
            "Diligent map tile set rescheduled a cached terrain tile");
    require(asyncTileSet.requestTerrainWindow(chronoTerrainMap, 2101.0, -599.0, 0,
                                              textureDirectory, 0) == 1,
            "Diligent map tile set did not schedule a changed chrono terrain source");
    std::size_t chronoPublished = 0;
    const auto chronoDeadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (chronoPublished == 0 && std::chrono::steady_clock::now() < chronoDeadline) {
        chronoPublished = asyncTileSet.publishReadyTerrainTiles();
        if (chronoPublished == 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    require(chronoPublished == 1 &&
                asyncTileSet.requestTerrainWindow(chronoTerrainMap, 2101.0, -599.0, 0,
                                                  textureDirectory, 0) == 0,
            "Diligent map tile set did not publish and cache changed chrono terrain");
    require(asyncTileSet.requestTerrainWindow(fileBackedMap, 2101.0, -599.0, 0,
                                              textureDirectory, 0) == 1 &&
                asyncTileSet.updateTerrainWindow(chronoTerrainMap, 2101.0, -599.0, 0,
                                                 textureDirectory, 0) == 0,
            "Diligent synchronous terrain update did not supersede pending window work");
    for (int attempt = 0; attempt < 100; ++attempt) {
        asyncTileSet.publishReadyTerrainTiles();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    require(asyncTileSet.requestTerrainWindow(chronoTerrainMap, 2101.0, -599.0, 0,
                                              textureDirectory, 0) == 0,
            "A stale asynchronous terrain result replaced a synchronous chrono selection");

    openbus::map::MapDefinition multiTileAsyncMap = fileBackedMap;
    for (int tileX = 8; tileX < 13; ++tileX) {
        openbus::map::MapTileReference tile = fileBackedTile;
        tile.x = tileX;
        multiTileAsyncMap.tiles.push_back(std::move(tile));
    }
    DiligentMapTileSet boundedAsyncTileSet(device, multiTileAsyncMap.coordinates.tileSize);
    require(boundedAsyncTileSet.requestTerrainWindow(multiTileAsyncMap, 2101.0, -599.0, 5,
                                                     textureDirectory, 0) == 4,
            "Diligent map tile set did not cap concurrent terrain preparation at four jobs");
    const auto multiTileDeadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (boundedAsyncTileSet.tileCount() < multiTileAsyncMap.tiles.size() &&
           std::chrono::steady_clock::now() < multiTileDeadline) {
        boundedAsyncTileSet.publishReadyTerrainTiles();
        if (boundedAsyncTileSet.tileCount() < multiTileAsyncMap.tiles.size()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    require(boundedAsyncTileSet.tileCount() == multiTileAsyncMap.tiles.size() &&
                boundedAsyncTileSet.activeTiles().size() == multiTileAsyncMap.tiles.size(),
            "Diligent map tile set did not drain its bounded terrain-preparation queue");

    openbus::map::MapDefinition invalidAsyncMap = fileBackedMap;
    invalidAsyncMap.tiles[0].terrainPath = textureDirectory / "missing-terrain.terrain";
    DiligentMapTileSet invalidAsyncTileSet(device, invalidAsyncMap.coordinates.tileSize);
    require(invalidAsyncTileSet.requestTerrainWindow(invalidAsyncMap, 2101.0, -599.0, 0,
                                                     textureDirectory, 0) == 1,
            "Diligent map tile set did not schedule invalid terrain preparation");
    bool preparationFailureSurfaced = false;
    const auto failureDeadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (!preparationFailureSurfaced && std::chrono::steady_clock::now() < failureDeadline) {
        try {
            invalidAsyncTileSet.publishReadyTerrainTiles();
        } catch (const std::runtime_error&) {
            preparationFailureSurfaced = true;
        }
        if (!preparationFailureSurfaced) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    require(preparationFailureSurfaced && invalidAsyncTileSet.tileCount() == 0,
            "Diligent map tile set did not propagate terrain preparation failures");

    openbus::map::MapDefinition projectedFileMap = fileBackedMap;
    projectedFileMap.coordinates = openbus::map::MapCoordinates::forRows(true, {10940});
    openbus::map::MapTileReference projectedFileTile = fileBackedTile;
    projectedFileTile.x = -494;
    projectedFileTile.y = 10940;
    DiligentMapTileSet projectedTileSet(device, projectedFileMap.coordinates);
    projectedTileSet.addTerrainTileFromFiles(projectedFileMap, projectedFileTile, textureDirectory,
                                             0);
    const double projectedTileSize = projectedFileMap.coordinates.tileSize;
    projectedTileSet.updateActiveTiles(-494.0 * projectedTileSize + 1.0,
                                       10940.0 * projectedTileSize + 1.0, 0);
    const Matrix4 projectedTileTransform =
        projectedTileSet.activeTiles()[0]->worldTransform(projectedTileSize);
    require(projectedTileTransform[12] == -494.0 * projectedTileSize &&
                projectedTileTransform[13] == 10940.0 * projectedTileSize,
            "Diligent file-backed tile did not preserve projected map-grid placement");
    clearTerrainTarget();
    require(fileBackedTileSet.drawWater(noDepth, context, *whiteView, identity) == 1,
            "Diligent map tile set failed to submit its active water mesh");
    openbus::map::RoadMeshGeometry fileRoadGeometry = roadGeometry;
    fileRoadGeometry.texturePath = imagePath;
    const std::array<DiligentMapRoadSource, 1> fileRoadSources = {{
        {&fileRoadGeometry, {}, {}},
    }};
    DiligentMapTileAsset fileRoadAsset(device, 7, -2, nullptr,
                                       std::span<const DiligentMapTerrainLayerSource>{}, nullptr,
                                       {}, fileRoadSources);
    require(fileRoadAsset.roads().size() == 1 && fileRoadAsset.roads()[0].texture != nullptr,
            "Diligent map road asset did not load its prepared texture path");
    DiligentMapTileSet mapTileSet(device, 300.0);
    mapTileSet.addTile(7, -2, &terrainGeometry, terrainLayerSources, nullptr, {}, roadSources);
    mapTileSet.addTile(8, -2, &terrainGeometry, terrainLayerSources, nullptr, {}, {});
    mapTileSet.updateActiveTiles(2101.0, -599.0, 0);
    require(mapTileSet.tileCount() == 2 && mapTileSet.activeTiles().size() == 1 &&
                mapTileSet.activeTiles()[0]->tileX() == 7 &&
                mapTileSet.activeTiles()[0]->tileY() == -2,
            "Diligent map tile set selected tiles outside the active tile window");
    mapTileSet.updateActiveTiles(2101.0, -599.0, 1);
    require(mapTileSet.activeTiles().size() == 2 &&
                mapTileSet.activeTiles()[0]->tileX() == 7 &&
                mapTileSet.activeTiles()[1]->tileX() == 8,
            "Diligent map tile set did not keep its active tile window sorted");
    mapTileSet.updateActiveTiles(2101.0, -599.0, 0);
    clearTerrainTarget();
    require(mapTileSet.drawTerrain(noDepth, context, *whiteView, identity) ==
                terrainLayerSources.size(),
            "Diligent map tile set failed to submit its active terrain layers");
    DiligentMaterialState roadPassMaterial;
    roadPassMaterial.alphaMode = DiligentAlphaMode::Blend;
    roadPassMaterial.depthMode = DiligentDepthMode::Disabled;
    roadPassMaterial.depthWrite = false;
    clearTerrainTarget();
    require(mapTileSet.drawRoads(DiligentMapRoadPass::Opaque, noDepth, context, *whiteView,
                                 identity, &roadPassMaterial) == 0 &&
                mapTileSet.drawRoads(DiligentMapRoadPass::Transparent, noDepth, context,
                                     *whiteView, identity, &roadPassMaterial) ==
                    roadSources.size(),
            "Diligent map tile set did not filter and submit transparent road batches");
    require(mapTileSet.removeInactiveTiles() == 1 && mapTileSet.tileCount() == 1 &&
                mapTileSet.activeTiles().size() == 1,
            "Diligent map tile set failed to release inactive tile assets");
    mapTileSet.removeTile(7, -2);
    require(mapTileSet.tileCount() == 0 && mapTileSet.activeTiles().empty(),
            "Removing a Diligent map tile left a stale active-tile reference");
    clearTerrainTarget();
    require(layeredTerrain[0].draw(noDepth, context, *whiteView, identity),
            "Diligent map tile failed to render its base terrain layer");
    const auto baseTerrainPixel = sampleTerrainPixel();
    clearTerrainTarget();
    require(layeredTerrain[1].draw(noDepth, context, *whiteView, identity),
            "Diligent map tile failed to render its transmap terrain layer");
    const auto overlayTerrainPixel = sampleTerrainPixel();
    clearTerrainTarget();
    require(layeredTerrain[0].draw(noDepth, context, *whiteView, identity) &&
                layeredTerrain[1].draw(noDepth, context, *whiteView, identity),
            "Diligent map tile failed to compose its terrain material layers");
    const auto composedTerrainPixel = sampleTerrainPixel();
    constexpr float transmapAlpha = 128.0F / 255.0F;
    for (std::size_t channel = 0; channel < composedTerrainPixel.size(); ++channel) {
        const float expected = overlayTerrainPixel[channel] +
                               (1.0F - transmapAlpha) * baseTerrainPixel[channel];
        require(std::abs(static_cast<float>(composedTerrainPixel[channel]) - expected) <= 4.0F,
                "Diligent map terrain transmap did not alpha-compose over its base layer");
    }
    bool foreignRoadTileRejected = false;
    roadGeometry.tileX = 8;
    try {
        DiligentMapTileAsset invalidMapTile(device, 7, -2, nullptr,
                                            std::shared_ptr<const Image>{}, nullptr, {},
                                            roadSources);
    } catch (const std::invalid_argument&) {
        foreignRoadTileRejected = true;
    }
    require(foreignRoadTileRejected, "Diligent map tile accepted road geometry from another tile");

    DiligentMaterialState mapMaterial;
    mapMaterial.depthMode = DiligentDepthMode::Disabled;
    context.SetRenderTargets(1, &targetView, nullptr, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    require(mapTile.terrain()->draw(noDepth, context, *whiteView, identity, &mapMaterial) &&
                mapTile.water()->draw(noDepth, context, *whiteView, identity, &mapMaterial) &&
                mapTile.roads()[0].draw(noDepth, context, *whiteView, identity, &mapMaterial) &&
                mapTile.roads()[1].draw(noDepth, context, *whiteView, identity, &mapMaterial) &&
                layeredTerrain[0].draw(noDepth, context, *whiteView, identity) &&
                layeredTerrain[1].draw(noDepth, context, *whiteView, identity),
            "Diligent map tile failed to submit layered terrain, water, or ordered road batches");
}
