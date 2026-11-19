#pragma once

#include <cstdint>
#include <memory>
#include <span>

namespace Diligent {
struct IDeviceContext;
struct IRenderDevice;
struct ITextureView;
} // namespace Diligent

namespace openbus::rendering {

// Updateable single-mip RGBA8 texture for engine-owned dynamic image data.
// The supplied immediate context must belong to the texture's device. Updates
// and subsequent draws must be ordered on that context, or synchronized by the
// caller when multiple contexts are used.
class DiligentDynamicTexture2D final {
  public:
    DiligentDynamicTexture2D(Diligent::IRenderDevice& device, Diligent::IDeviceContext& context,
                             int width, int height, std::span<const std::uint8_t> pixels);
    ~DiligentDynamicTexture2D();
    DiligentDynamicTexture2D(const DiligentDynamicTexture2D&) = delete;
    DiligentDynamicTexture2D& operator=(const DiligentDynamicTexture2D&) = delete;

    // Replaces the texture and its view when dimensions change. Any borrowed
    // view must be reacquired after such an update; consumers retaining an old
    // view must own a reference and synchronize its outstanding use.
    void update(Diligent::IDeviceContext& context, int width, int height,
                std::span<const std::uint8_t> pixels);

    [[nodiscard]] int width() const noexcept;
    [[nodiscard]] int height() const noexcept;
    [[nodiscard]] Diligent::ITextureView& shaderResourceView() const noexcept;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace openbus::rendering
