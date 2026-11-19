#include "render/diligent/DiligentSpotlightShadowMaps.h"

#include <Common/interface/RefCntAutoPtr.hpp>
#include <Graphics/GraphicsEngine/interface/DeviceContext.h>
#include <Graphics/GraphicsEngine/interface/RenderDevice.h>
#include <Graphics/GraphicsEngine/interface/Texture.h>
#include <Graphics/GraphicsEngine/interface/TextureView.h>

#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace openbus::rendering {

struct DiligentSpotlightShadowMaps::Impl {
    Diligent::Uint32 resolution;
    Diligent::Uint32 capacity;
    Diligent::RefCntAutoPtr<Diligent::ITexture> texture;
    Diligent::RefCntAutoPtr<Diligent::ITextureView> shaderResourceView;
    std::vector<Diligent::RefCntAutoPtr<Diligent::ITextureView>> depthLayers;

    Impl(Diligent::IRenderDevice& device, Diligent::Uint32 requestedResolution,
         Diligent::Uint32 requestedCapacity)
        : resolution{requestedResolution}, capacity{requestedCapacity} {
        using namespace Diligent;
        if (resolution == 0 || capacity == 0 ||
            capacity > DiligentSpotlightShadowMaps::MAX_LIGHTS) {
            throw std::invalid_argument("Invalid Diligent spotlight shadow-map dimensions");
        }

        const auto formatInfo = device.GetTextureFormatInfoExt(TEX_FORMAT_D32_FLOAT);
        if (!(formatInfo.BindFlags & BIND_DEPTH_STENCIL) ||
            !(formatInfo.BindFlags & BIND_SHADER_RESOURCE)) {
            throw std::invalid_argument(
                "Diligent device does not support sampled D32 spotlight shadow maps");
        }

        TextureDesc textureDesc;
        textureDesc.Name = "OpenBus Diligent spotlight shadow-map array";
        textureDesc.Type = RESOURCE_DIM_TEX_2D_ARRAY;
        textureDesc.Width = resolution;
        textureDesc.Height = resolution;
        textureDesc.ArraySize = capacity;
        textureDesc.MipLevels = 1;
        textureDesc.SampleCount = 1;
        textureDesc.Format = TEX_FORMAT_D32_FLOAT;
        textureDesc.Usage = USAGE_DEFAULT;
        textureDesc.BindFlags = BIND_DEPTH_STENCIL | BIND_SHADER_RESOURCE;
        device.CreateTexture(textureDesc, nullptr, &texture);
        if (!texture) {
            throw std::runtime_error("Failed to create Diligent spotlight shadow-map array");
        }

        shaderResourceView = texture->GetDefaultView(TEXTURE_VIEW_SHADER_RESOURCE);
        if (!shaderResourceView) {
            throw std::runtime_error("Failed to create Diligent spotlight shadow-map SRV");
        }

        depthLayers.reserve(capacity);
        for (Uint32 index = 0; index < capacity; ++index) {
            TextureViewDesc viewDesc;
            viewDesc.ViewType = TEXTURE_VIEW_DEPTH_STENCIL;
            viewDesc.TextureDim = RESOURCE_DIM_TEX_2D_ARRAY;
            viewDesc.Format = TEX_FORMAT_D32_FLOAT;
            viewDesc.MostDetailedMip = 0;
            viewDesc.NumMipLevels = 1;
            viewDesc.FirstArraySlice = index;
            viewDesc.NumArraySlices = 1;

            RefCntAutoPtr<ITextureView> view;
            texture->CreateView(viewDesc, &view);
            if (!view) {
                throw std::runtime_error("Failed to create Diligent spotlight shadow-map layer " +
                                         std::to_string(index));
            }
            depthLayers.push_back(std::move(view));
        }
    }

    Diligent::ITextureView& depthLayer(Diligent::Uint32 lightIndex) const {
        if (lightIndex >= capacity) {
            throw std::out_of_range("Diligent spotlight shadow-map index is out of range");
        }
        return *depthLayers[lightIndex];
    }

    void clearLayer(Diligent::IDeviceContext& context, Diligent::Uint32 lightIndex,
                    float clearDepth) const {
        if (!std::isfinite(clearDepth) || clearDepth < 0.0F || clearDepth > 1.0F) {
            throw std::invalid_argument("Diligent spotlight shadow clear depth must be in [0, 1]");
        }
        context.ClearDepthStencil(&depthLayer(lightIndex), Diligent::CLEAR_DEPTH_FLAG, clearDepth,
                                  0, Diligent::RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    }
};

DiligentSpotlightShadowMaps::DiligentSpotlightShadowMaps(Diligent::IRenderDevice& device,
                                                         Diligent::Uint32 resolution,
                                                         Diligent::Uint32 capacity)
    : impl_{std::make_unique<Impl>(device, resolution, capacity)} {}

DiligentSpotlightShadowMaps::~DiligentSpotlightShadowMaps() = default;

Diligent::Uint32 DiligentSpotlightShadowMaps::resolution() const noexcept {
    return impl_->resolution;
}

Diligent::Uint32 DiligentSpotlightShadowMaps::capacity() const noexcept {
    return impl_->capacity;
}

Diligent::ITextureView& DiligentSpotlightShadowMaps::depthLayer(Diligent::Uint32 lightIndex) const {
    return impl_->depthLayer(lightIndex);
}

Diligent::ITextureView& DiligentSpotlightShadowMaps::shaderResourceView() const noexcept {
    return *impl_->shaderResourceView;
}

void DiligentSpotlightShadowMaps::clearLayer(Diligent::IDeviceContext& context,
                                             Diligent::Uint32 lightIndex, float clearDepth) const {
    impl_->clearLayer(context, lightIndex, clearDepth);
}

} // namespace openbus::rendering
