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

struct DiligentMultisampling {
    Diligent::Uint32 samples = 1;
};

struct DiligentMultisamplingSelection {
    DiligentMultisampling multisampling;
    bool useFxaaFallback = false;
};

// Selects the requested sample count or the next supported lower count. Returns FXAA fallback
// when none of the multisampled scene attachments support at least 2x sampling.
[[nodiscard]] DiligentMultisamplingSelection
selectDiligentMultisampling(Diligent::IRenderDevice& device, bool includeLinearDepth,
                            DiligentMultisampling requested = {});
[[nodiscard]] DiligentMultisamplingSelection
selectDiligentMultisamplingFromSupport(Diligent::Uint32 supportedSampleCounts,
                                       DiligentMultisampling requested = {});

// Color-only MSAA scene attachments. Motion, validity, and depth metadata are not resolved:
// temporal consumers need a separate visibility-aware policy, not hardware averaging.
class DiligentMultisampleTargets final {
  public:
    DiligentMultisampleTargets(Diligent::IRenderDevice& device, Diligent::Uint32 width,
                               Diligent::Uint32 height, Diligent::Uint32 samples);
    ~DiligentMultisampleTargets();
    DiligentMultisampleTargets(const DiligentMultisampleTargets&) = delete;
    DiligentMultisampleTargets& operator=(const DiligentMultisampleTargets&) = delete;

    // Resizing waits for the immediate context before atomically replacing attachments.
    void resize(Diligent::IDeviceContext& immediateContext, Diligent::Uint32 width,
                Diligent::Uint32 height);
    void bind(Diligent::IDeviceContext& context) const;
    void clear(Diligent::IDeviceContext& context, const std::array<float, 4>& color,
               float depth = 1.0F) const;
    // Unbinds attachments and resolves color to the single-sample shader-resource view.
    void resolve(Diligent::IDeviceContext& context) const;
    [[nodiscard]] Diligent::ITextureView& resolvedColor() const;
    [[nodiscard]] Diligent::Uint32 width() const noexcept;
    [[nodiscard]] Diligent::Uint32 height() const noexcept;
    [[nodiscard]] Diligent::Uint32 samples() const noexcept;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace openbus::rendering
