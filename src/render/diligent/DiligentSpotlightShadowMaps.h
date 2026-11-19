#pragma once

#include <Graphics/GraphicsEngine/interface/GraphicsTypes.h>

#include <cstdint>
#include <memory>

namespace Diligent {
struct IDeviceContext;
struct IRenderDevice;
struct ITextureView;
} // namespace Diligent

namespace openbus::rendering {

// One depth-array slice per scheduled spotlight lets a render pass clear a
// single light without clearing neighboring lights' shadow data.
class DiligentSpotlightShadowMaps final {
  public:
    static constexpr Diligent::Uint32 MAX_LIGHTS = 64;

    DiligentSpotlightShadowMaps(Diligent::IRenderDevice& device, Diligent::Uint32 resolution,
                                Diligent::Uint32 capacity);
    ~DiligentSpotlightShadowMaps();
    DiligentSpotlightShadowMaps(const DiligentSpotlightShadowMaps&) = delete;
    DiligentSpotlightShadowMaps& operator=(const DiligentSpotlightShadowMaps&) = delete;

    [[nodiscard]] Diligent::Uint32 resolution() const noexcept;
    [[nodiscard]] Diligent::Uint32 capacity() const noexcept;
    [[nodiscard]] Diligent::ITextureView& depthLayer(Diligent::Uint32 lightIndex) const;
    [[nodiscard]] Diligent::ITextureView& shaderResourceView() const noexcept;
    void clearLayer(Diligent::IDeviceContext& context, Diligent::Uint32 lightIndex,
                    float clearDepth = 1.0F) const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace openbus::rendering
