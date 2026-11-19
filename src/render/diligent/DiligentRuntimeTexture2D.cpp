#include "render/diligent/DiligentRuntimeTexture2D.h"

#include "render/diligent/DiligentDynamicTexture2D.h"

#include <Graphics/GraphicsEngine/interface/RenderDevice.h>

namespace openbus::rendering {

DiligentRuntimeTexture2D::DiligentRuntimeTexture2D(Diligent::IRenderDevice& device)
    : device_(&device) {}

DiligentRuntimeTexture2D::~DiligentRuntimeTexture2D() = default;

void DiligentRuntimeTexture2D::synchronize(Diligent::IDeviceContext& context, int width, int height,
                                           std::span<const std::uint8_t> pixels, bool filtered,
                                           std::uint64_t revision) {
    if (texture_ && hasRevision_ && width == width_ && height == height_ && revision == revision_) {
        filtered_ = filtered;
        return;
    }

    if (texture_) {
        texture_->update(context, width, height, pixels);
    } else {
        texture_ =
            std::make_unique<DiligentDynamicTexture2D>(*device_, context, width, height, pixels);
    }
    width_ = width;
    height_ = height;
    filtered_ = filtered;
    revision_ = revision;
    hasRevision_ = true;
}

bool DiligentRuntimeTexture2D::initialized() const noexcept {
    return static_cast<bool>(texture_);
}

int DiligentRuntimeTexture2D::width() const noexcept {
    return width_;
}

int DiligentRuntimeTexture2D::height() const noexcept {
    return height_;
}

bool DiligentRuntimeTexture2D::filtered() const noexcept {
    return filtered_;
}

std::uint64_t DiligentRuntimeTexture2D::revision() const noexcept {
    return revision_;
}

Diligent::ITextureView* DiligentRuntimeTexture2D::shaderResourceView() const noexcept {
    return texture_ ? &texture_->shaderResourceView() : nullptr;
}

} // namespace openbus::rendering
