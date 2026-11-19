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

class DiligentDynamicTexture2D;

// Revision-aware GPU storage for script and text texture snapshots. The owner
// remains responsible for obtaining snapshots from ScriptRuntime and setting
// material filtering/addressing state.
class DiligentRuntimeTexture2D final {
  public:
    explicit DiligentRuntimeTexture2D(Diligent::IRenderDevice& device);
    ~DiligentRuntimeTexture2D();
    DiligentRuntimeTexture2D(const DiligentRuntimeTexture2D&) = delete;
    DiligentRuntimeTexture2D& operator=(const DiligentRuntimeTexture2D&) = delete;

    // Updates only when the content revision or dimensions change. The filtered
    // flag is retained independently because sampler state may change without pixels.
    void synchronize(Diligent::IDeviceContext& context, int width, int height,
                     std::span<const std::uint8_t> pixels, bool filtered, std::uint64_t revision);

    [[nodiscard]] bool initialized() const noexcept;
    [[nodiscard]] int width() const noexcept;
    [[nodiscard]] int height() const noexcept;
    [[nodiscard]] bool filtered() const noexcept;
    [[nodiscard]] std::uint64_t revision() const noexcept;
    [[nodiscard]] Diligent::ITextureView* shaderResourceView() const noexcept;

  private:
    Diligent::IRenderDevice* device_ = nullptr;
    std::unique_ptr<DiligentDynamicTexture2D> texture_;
    int width_ = 0;
    int height_ = 0;
    bool filtered_ = false;
    bool hasRevision_ = false;
    std::uint64_t revision_ = 0;
};

} // namespace openbus::rendering
