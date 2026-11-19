#include "render/diligent/DiligentDynamicTexture2D.h"

#include "assets/TextureLoader.h"

#include <Common/interface/RefCntAutoPtr.hpp>
#include <Graphics/GraphicsEngine/interface/DeviceContext.h>
#include <Graphics/GraphicsEngine/interface/RenderDevice.h>
#include <Graphics/GraphicsEngine/interface/Texture.h>
#include <Graphics/GraphicsEngine/interface/TextureView.h>

#include <stdexcept>
#include <utility>

namespace openbus::rendering {
namespace {

void validateImage(int width, int height, std::span<const std::uint8_t> pixels) {
    std::size_t expectedBytes = 0;
    if (width <= 0 || height <= 0 ||
        !checkedTextureBufferSize(static_cast<std::size_t>(width), static_cast<std::size_t>(height),
                                  4, expectedBytes) ||
        pixels.size() != expectedBytes) {
        throw std::invalid_argument("Diligent dynamic texture requires valid RGBA8 pixel data");
    }
}

struct TextureResource {
    Diligent::RefCntAutoPtr<Diligent::ITexture> texture;
    Diligent::RefCntAutoPtr<Diligent::ITextureView> view;
};

TextureResource createTexture(Diligent::IRenderDevice& device, Diligent::IDeviceContext& context,
                              int width, int height, std::span<const std::uint8_t> pixels) {
    using namespace Diligent;
    TextureDesc desc;
    desc.Name = "OpenBus dynamic RGBA texture";
    desc.Type = RESOURCE_DIM_TEX_2D;
    desc.Width = static_cast<Uint32>(width);
    desc.Height = static_cast<Uint32>(height);
    desc.MipLevels = 1;
    desc.Format = TEX_FORMAT_RGBA8_UNORM;
    desc.Usage = USAGE_DEFAULT;
    desc.BindFlags = BIND_SHADER_RESOURCE;

    TextureSubResData subresource;
    subresource.pData = pixels.data();
    subresource.Stride = static_cast<Uint32>(width) * 4U;
    TextureData data;
    data.pSubResources = &subresource;
    data.NumSubresources = 1;
    data.pContext = &context;
    TextureResource result;
    device.CreateTexture(desc, &data, &result.texture);
    if (!result.texture) {
        throw std::runtime_error("Failed to create Diligent dynamic texture");
    }
    result.view = result.texture->GetDefaultView(TEXTURE_VIEW_SHADER_RESOURCE);
    if (!result.view) {
        throw std::runtime_error("Failed to create Diligent dynamic texture shader view");
    }
    return result;
}

} // namespace

struct DiligentDynamicTexture2D::Impl {
    Diligent::RefCntAutoPtr<Diligent::IRenderDevice> device;
    TextureResource resource;
    int width;
    int height;

    Impl(Diligent::IRenderDevice& renderDevice, Diligent::IDeviceContext& context, int imageWidth,
         int imageHeight, std::span<const std::uint8_t> pixels)
        : device(&renderDevice), width(imageWidth), height(imageHeight) {
        validateImage(width, height, pixels);
        resource = createTexture(renderDevice, context, width, height, pixels);
    }
};

DiligentDynamicTexture2D::DiligentDynamicTexture2D(Diligent::IRenderDevice& device,
                                                   Diligent::IDeviceContext& context, int width,
                                                   int height, std::span<const std::uint8_t> pixels)
    : impl_(std::make_unique<Impl>(device, context, width, height, pixels)) {}

DiligentDynamicTexture2D::~DiligentDynamicTexture2D() = default;

void DiligentDynamicTexture2D::update(Diligent::IDeviceContext& context, int width, int height,
                                      std::span<const std::uint8_t> pixels) {
    using namespace Diligent;
    validateImage(width, height, pixels);
    if (width != impl_->width || height != impl_->height) {
        auto resource = createTexture(*impl_->device, context, width, height, pixels);
        impl_->resource = std::move(resource);
        impl_->width = width;
        impl_->height = height;
        return;
    }

    TextureSubResData data;
    data.pData = pixels.data();
    data.Stride = static_cast<Uint32>(width) * 4U;
    const Box region{0, static_cast<Uint32>(width), 0, static_cast<Uint32>(height), 0, 1};
    context.UpdateTexture(impl_->resource.texture, 0, 0, region, data,
                          RESOURCE_STATE_TRANSITION_MODE_NONE,
                          RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
}

int DiligentDynamicTexture2D::width() const noexcept {
    return impl_->width;
}

int DiligentDynamicTexture2D::height() const noexcept {
    return impl_->height;
}

Diligent::ITextureView& DiligentDynamicTexture2D::shaderResourceView() const noexcept {
    return *impl_->resource.view;
}

} // namespace openbus::rendering
