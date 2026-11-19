#pragma once

#include <Graphics/GraphicsEngine/interface/GraphicsTypes.h>

#include <array>
#include <memory>

namespace Diligent {
struct IDeviceContext;
struct IRenderDevice;
struct ITextureView;
} // namespace Diligent

namespace openbus::rendering {

// Single-sample offscreen color/depth target for a reflection camera. The color SRV can be bound
// by later scene passes after rendering; the caller owns camera setup and pass-state restoration.
class DiligentReflectionTarget final {
  public:
    DiligentReflectionTarget(
        Diligent::IRenderDevice& device, Diligent::Uint32 size,
        Diligent::TEXTURE_FORMAT colorFormat = Diligent::TEX_FORMAT_RGBA8_UNORM,
        Diligent::TEXTURE_FORMAT depthFormat = Diligent::TEX_FORMAT_D32_FLOAT);
    ~DiligentReflectionTarget();
    DiligentReflectionTarget(const DiligentReflectionTarget&) = delete;
    DiligentReflectionTarget& operator=(const DiligentReflectionTarget&) = delete;

    void resize(Diligent::Uint32 size);
    void bind(Diligent::IDeviceContext& context) const;
    void clear(Diligent::IDeviceContext& context,
               const std::array<float, 4>& color = {0.0F, 0.0F, 0.0F, 1.0F},
               float depth = 1.0F) const;

    [[nodiscard]] Diligent::Uint32 size() const noexcept;
    [[nodiscard]] Diligent::TEXTURE_FORMAT colorFormat() const noexcept;
    [[nodiscard]] Diligent::TEXTURE_FORMAT depthFormat() const noexcept;
    [[nodiscard]] Diligent::ITextureView& colorRenderTarget() const;
    [[nodiscard]] Diligent::ITextureView& colorTexture() const;
    [[nodiscard]] Diligent::ITextureView& depthStencilTarget() const;
    [[nodiscard]] Diligent::ITextureView& depthTexture() const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace openbus::rendering
