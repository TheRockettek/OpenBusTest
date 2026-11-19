#include "DiligentPrimitiveRendering.h"

#include "assets/O3DLoader.h"
#include "render/diligent/DiligentPrimitiveRenderer.h"
#include "render/diligent/DiligentTexturedMesh.h"

#include <Common/interface/RefCntAutoPtr.hpp>
#include <Graphics/GraphicsEngine/interface/DeviceContext.h>
#include <Graphics/GraphicsEngine/interface/RenderDevice.h>
#include <Graphics/GraphicsEngine/interface/Texture.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

namespace {

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

} // namespace

void verifyDiligentPrimitiveRendering(Diligent::IRenderDevice& device,
                                      Diligent::IDeviceContext& context,
                                      const std::filesystem::path* meshPath) {
    using namespace Diligent;
    using namespace openbus::rendering;

    constexpr Uint32 size = 64;
    TextureDesc colorDesc;
    colorDesc.Name = "Primitive test color";
    colorDesc.Type = RESOURCE_DIM_TEX_2D;
    colorDesc.Width = size;
    colorDesc.Height = size;
    colorDesc.MipLevels = 1;
    colorDesc.Format = TEX_FORMAT_RGBA8_UNORM;
    colorDesc.BindFlags = BIND_RENDER_TARGET;
    RefCntAutoPtr<ITexture> color;
    device.CreateTexture(colorDesc, nullptr, &color);
    require(static_cast<bool>(color), "Failed to create primitive test color target");

    TextureDesc depthDesc = colorDesc;
    depthDesc.Name = "Primitive test depth";
    depthDesc.Format = TEX_FORMAT_D32_FLOAT;
    depthDesc.BindFlags = BIND_DEPTH_STENCIL;
    RefCntAutoPtr<ITexture> depth;
    device.CreateTexture(depthDesc, nullptr, &depth);
    require(static_cast<bool>(depth), "Failed to create primitive test depth target");

    TextureDesc stagingDesc = colorDesc;
    stagingDesc.Name = "Primitive test readback";
    stagingDesc.BindFlags = BIND_NONE;
    stagingDesc.Usage = USAGE_STAGING;
    stagingDesc.CPUAccessFlags = CPU_ACCESS_READ;
    RefCntAutoPtr<ITexture> staging;
    device.CreateTexture(stagingDesc, nullptr, &staging);
    require(static_cast<bool>(staging), "Failed to create primitive test readback target");

    ITextureView* colorView = color->GetDefaultView(TEXTURE_VIEW_RENDER_TARGET);
    ITextureView* depthView = depth->GetDefaultView(TEXTURE_VIEW_DEPTH_STENCIL);
    require(colorView != nullptr && depthView != nullptr,
            "Primitive test target views are missing");
    context.SetRenderTargets(1, &colorView, depthView, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    const float clear[] = {0, 0, 0, 1};
    context.ClearRenderTarget(colorView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearDepthStencil(depthView, CLEAR_DEPTH_FLAG, 1.0f, 0,
                              RESOURCE_STATE_TRANSITION_MODE_TRANSITION);

    DiligentPrimitiveRenderer renderer(device, colorDesc.Format, depthDesc.Format);
    const std::array<PrimitiveVertex, 3> nearTriangle = {{
        {-0.4f, -0.5f, -0.5f, 1, 0, 0},
        {0.4f, -0.5f, -0.5f, 1, 0, 0},
        {0.0f, 0.5f, -0.5f, 1, 0, 0},
    }};
    const std::array<PrimitiveVertex, 3> farTriangle = {{
        {-0.8f, -0.8f, 0.5f, 0, 0, 1},
        {0.8f, -0.8f, 0.5f, 0, 0, 1},
        {0.0f, 0.8f, 0.5f, 0, 0, 1},
    }};
    const Matrix4 identity = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    Matrix4 translated = identity;
    translated[12] = 0.25;
    translated[13] = 0.35;
    renderer.draw(context, nearTriangle, translated, translated);
    renderer.draw(context, farTriangle, identity, identity);
    auto clippedTriangle = farTriangle;
    for (auto& vertex : clippedTriangle) {
        vertex.z = -1.1f;
        vertex.g = 1.0f;
        vertex.b = 0.0f;
    }
    renderer.draw(context, clippedTriangle, identity, identity);
    for (auto& vertex : clippedTriangle) {
        vertex.z = 1.1f;
    }
    renderer.draw(context, clippedTriangle, identity, identity);
    renderer.draw(context, {}, identity, identity);

    bool rejectedInvalidCount = false;
    try {
        renderer.draw(context, std::span{nearTriangle}.first(2), identity, identity);
    } catch (const std::invalid_argument&) {
        rejectedInvalidCount = true;
    }
    require(rejectedInvalidCount, "Primitive renderer accepted an incomplete triangle");
    Matrix4 invalid = identity;
    invalid[0] = std::numeric_limits<double>::quiet_NaN();
    bool rejectedInvalidMatrix = false;
    try {
        renderer.draw(context, nearTriangle, invalid, identity);
    } catch (const std::invalid_argument&) {
        rejectedInvalidMatrix = true;
    }
    require(rejectedInvalidMatrix, "Primitive renderer accepted a non-finite transform");

    context.SetRenderTargets(0, nullptr, nullptr, RESOURCE_STATE_TRANSITION_MODE_NONE);
    CopyTextureAttribs copy;
    copy.pSrcTexture = color;
    copy.SrcTextureTransitionMode = RESOURCE_STATE_TRANSITION_MODE_TRANSITION;
    copy.pDstTexture = staging;
    copy.DstTextureTransitionMode = RESOURCE_STATE_TRANSITION_MODE_TRANSITION;
    context.CopyTexture(copy);
    context.WaitForIdle();

    MappedTextureSubresource mapped;
    context.MapTextureSubresource(staging, 0, 0, MAP_READ, MAP_FLAG_DO_NOT_WAIT, nullptr, mapped);
    require(mapped.pData != nullptr, "Failed to map primitive test readback");
    std::size_t redCount = 0;
    std::size_t blueCount = 0;
    std::size_t greenCount = 0;
    std::uint64_t redX = 0;
    std::uint64_t redY = 0;
    const auto* pixels = static_cast<const std::uint8_t*>(mapped.pData);
    for (Uint32 y = 0; y < size; ++y) {
        const auto* row = pixels + y * mapped.Stride;
        for (Uint32 x = 0; x < size; ++x) {
            const auto* pixel = row + x * 4;
            if (pixel[0] > 240 && pixel[1] < 10 && pixel[2] < 10) {
                ++redCount;
                redX += x;
                redY += y;
            } else if (pixel[0] < 10 && pixel[1] < 10 && pixel[2] > 240) {
                ++blueCount;
            } else if (pixel[1] > 240) {
                ++greenCount;
            }
        }
    }
    context.UnmapTextureSubresource(staging, 0, 0);
    require(redCount > 200 && blueCount > 300,
            "Primitive readback did not contain both depth-tested triangles");
    require(redX > redCount * (size / 2 + 3),
            "Primitive readback did not reflect the translated transform");
    require(redY < redCount * (size / 2 - 3),
            "Primitive readback did not preserve the OpenGL vertical orientation");
    require(greenCount == 0, "Primitive renderer did not clip OpenGL near/far depth");

    DiligentPrimitiveRenderer rendererWithoutDepth(device, colorDesc.Format, TEX_FORMAT_UNKNOWN);
    context.SetRenderTargets(1, &colorView, nullptr, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(colorView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    rendererWithoutDepth.draw(context, nearTriangle, translated, translated);
    rendererWithoutDepth.draw(context, farTriangle, identity, identity);
    context.SetRenderTargets(0, nullptr, nullptr, RESOURCE_STATE_TRANSITION_MODE_NONE);
    context.CopyTexture(copy);
    context.WaitForIdle();
    context.MapTextureSubresource(staging, 0, 0, MAP_READ, MAP_FLAG_DO_NOT_WAIT, nullptr, mapped);
    require(mapped.pData != nullptr, "Failed to map depth-disabled primitive readback");
    const auto* center = static_cast<const std::uint8_t*>(mapped.pData) +
                         (size / 2) * mapped.Stride + (size / 2) * 4;
    const bool depthDisabled = center[0] < 10 && center[2] > 240;
    context.UnmapTextureSubresource(staging, 0, 0);
    require(depthDisabled, "Primitive renderer did not disable depth without a depth target");

    if (meshPath == nullptr) {
        return;
    }
    const auto mesh = O3DLoader::parse(*meshPath);
    require(mesh && !mesh->positions.empty() && !mesh->triangles.empty(),
            "Failed to load the O3D primitive rendering fixture");
    std::array<double, 3> minimum = {std::numeric_limits<double>::max(),
                                     std::numeric_limits<double>::max(),
                                     std::numeric_limits<double>::max()};
    std::array<double, 3> maximum = {std::numeric_limits<double>::lowest(),
                                     std::numeric_limits<double>::lowest(),
                                     std::numeric_limits<double>::lowest()};
    for (const auto& position : mesh->positions) {
        const std::array<double, 3> components = {position.x, position.y, position.z};
        for (std::size_t axis = 0; axis < 3; ++axis) {
            require(std::isfinite(components[axis]), "O3D fixture contains a non-finite vertex");
            minimum[axis] = std::min(minimum[axis], components[axis]);
            maximum[axis] = std::max(maximum[axis], components[axis]);
        }
    }
    std::array<std::size_t, 3> axes = {0, 1, 2};
    std::sort(axes.begin(), axes.end(), [&](std::size_t left, std::size_t right) {
        return maximum[left] - minimum[left] > maximum[right] - minimum[right];
    });
    const double extent = maximum[axes[0]] - minimum[axes[0]];
    require(extent > 0.0 && maximum[axes[1]] > minimum[axes[1]],
            "O3D fixture cannot be projected into two dimensions");
    require(mesh->triangles.size() <= std::numeric_limits<std::size_t>::max() / 3,
            "O3D fixture triangle count overflows");
    std::vector<PrimitiveVertex> vertices;
    vertices.reserve(mesh->triangles.size() * 3);
    std::vector<ModelVertex> texturedVertices;
    texturedVertices.reserve(mesh->triangles.size() * 3);
    std::size_t textureCoordinateCount = 0;
    for (const auto& triangle : mesh->triangles) {
        for (const auto& index : triangle.indices) {
            require(index.position > 0 &&
                        static_cast<std::size_t>(index.position) <= mesh->positions.size(),
                    "O3D fixture contains an invalid vertex index");
            const auto& position = mesh->positions[static_cast<std::size_t>(index.position - 1)];
            const std::array<double, 3> components = {position.x, position.y, position.z};
            const auto project = [&](std::size_t axis) {
                return static_cast<float>(
                    (components[axis] - (minimum[axis] + maximum[axis]) * 0.5) * 1.6 / extent);
            };
            vertices.push_back({project(axes[0]), project(axes[1]), 0.0f, 0.0f, 1.0f, 0.0f});
            float u = 0.0f;
            float v = 0.0f;
            if (index.texCoord > 0 &&
                static_cast<std::size_t>(index.texCoord) <= mesh->texCoords.size()) {
                const auto& uv = mesh->texCoords[static_cast<std::size_t>(index.texCoord - 1)];
                u = static_cast<float>(uv.u);
                v = static_cast<float>(1.0 - uv.v);
                ++textureCoordinateCount;
            }
            texturedVertices.push_back({project(axes[0]), project(axes[1]), 0.0f, u, v,
                                        0.0f, 0.0f, 0.0f, 1.0f});
        }
    }

    context.SetRenderTargets(1, &colorView, depthView, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(colorView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearDepthStencil(depthView, CLEAR_DEPTH_FLAG, 1.0f, 0,
                              RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    renderer.draw(context, vertices, identity, identity);
    context.SetRenderTargets(0, nullptr, nullptr, RESOURCE_STATE_TRANSITION_MODE_NONE);
    context.CopyTexture(copy);
    context.WaitForIdle();
    context.MapTextureSubresource(staging, 0, 0, MAP_READ, MAP_FLAG_DO_NOT_WAIT, nullptr, mapped);
    require(mapped.pData != nullptr, "Failed to map O3D primitive test readback");
    std::size_t meshPixels = 0;
    pixels = static_cast<const std::uint8_t*>(mapped.pData);
    for (Uint32 y = 0; y < size; ++y) {
        const auto* row = pixels + y * mapped.Stride;
        for (Uint32 x = 0; x < size; ++x) {
            if (row[x * 4 + 1] > 240) {
                ++meshPixels;
            }
        }
    }
    context.UnmapTextureSubresource(staging, 0, 0);
    require(meshPixels > 20, "O3D geometry did not render into the Diligent target");

    require(textureCoordinateCount > 0, "O3D fixture has no usable texture coordinates");
    const std::array<std::uint8_t, 16> texels = {
        255, 0, 0, 255, 0, 255, 0, 255, 0, 0, 255, 255, 255, 255, 255, 255,
    };
    TextureDesc textureDesc;
    textureDesc.Name = "O3D test texture";
    textureDesc.Type = RESOURCE_DIM_TEX_2D;
    textureDesc.Width = 2;
    textureDesc.Height = 2;
    textureDesc.MipLevels = 1;
    textureDesc.Format = TEX_FORMAT_RGBA8_UNORM;
    textureDesc.Usage = USAGE_IMMUTABLE;
    textureDesc.BindFlags = BIND_SHADER_RESOURCE;
    TextureSubResData textureSubresource;
    textureSubresource.pData = texels.data();
    textureSubresource.Stride = 8;
    TextureData textureData;
    textureData.pSubResources = &textureSubresource;
    textureData.NumSubresources = 1;
    RefCntAutoPtr<ITexture> texture;
    device.CreateTexture(textureDesc, &textureData, &texture);
    require(static_cast<bool>(texture), "Failed to create O3D test texture");
    auto* textureView = texture->GetDefaultView(TEXTURE_VIEW_SHADER_RESOURCE);
    require(textureView != nullptr, "O3D test texture view is missing");
    DiligentTexturedMesh texturedMesh(device, texturedVertices);
    DiligentTexturedMeshRenderer texturedRenderer(device, colorDesc.Format, depthDesc.Format);
    context.SetRenderTargets(1, &colorView, depthView, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(colorView, clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearDepthStencil(depthView, CLEAR_DEPTH_FLAG, 1.0f, 0,
                              RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    texturedRenderer.draw(context, texturedMesh, *textureView, identity);
    context.SetRenderTargets(0, nullptr, nullptr, RESOURCE_STATE_TRANSITION_MODE_NONE);
    context.CopyTexture(copy);
    context.WaitForIdle();
    context.MapTextureSubresource(staging, 0, 0, MAP_READ, MAP_FLAG_DO_NOT_WAIT, nullptr, mapped);
    require(mapped.pData != nullptr, "Failed to map textured O3D test readback");
    std::size_t texturedPixels = 0;
    pixels = static_cast<const std::uint8_t*>(mapped.pData);
    for (Uint32 y = 0; y < size; ++y) {
        const auto* row = pixels + y * mapped.Stride;
        for (Uint32 x = 0; x < size; ++x) {
            if (row[x * 4] > 5 || row[x * 4 + 1] > 5 || row[x * 4 + 2] > 5) {
                ++texturedPixels;
            }
        }
    }
    context.UnmapTextureSubresource(staging, 0, 0);
    require(texturedPixels > 20, "Textured O3D geometry did not render into the Diligent target");
}
