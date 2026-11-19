#pragma once

#include "assets/TextureLoader.h"

#include <cstdint>
#include <memory>
#include <span>

namespace gli {
class texture;
}

namespace Diligent {
struct IRenderDevice;
struct ITextureView;
} // namespace Diligent

namespace openbus::rendering {

// Immutable 2D array uploaded from equally sized decoded images or supported GLI arrays.
class DiligentTexture2DArray {
  public:
    // Uploads supported block-compressed GLI arrays without expanding their mip chains.
    DiligentTexture2DArray(Diligent::IRenderDevice& device, const gli::texture& texture);
    DiligentTexture2DArray(Diligent::IRenderDevice& device, std::span<const Image> layers);
    DiligentTexture2DArray(Diligent::IRenderDevice& device,
                           std::span<const std::shared_ptr<const Image>> layers);
    ~DiligentTexture2DArray();
    DiligentTexture2DArray(const DiligentTexture2DArray&) = delete;
    DiligentTexture2DArray& operator=(const DiligentTexture2DArray&) = delete;

    [[nodiscard]] Diligent::ITextureView& shaderResourceView() const;
    [[nodiscard]] std::uint32_t layerCount() const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    [[nodiscard]] Diligent::IRenderDevice& owningDevice() const;
    friend class DiligentTexturedMeshRenderer;
};

} // namespace openbus::rendering
