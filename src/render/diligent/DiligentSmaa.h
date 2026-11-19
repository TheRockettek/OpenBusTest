#pragma once

#include <Graphics/GraphicsEngine/interface/GraphicsTypes.h>
#include <memory>

namespace Diligent {
struct IDeviceContext;
struct IRenderDevice;
struct ITextureView;
} // namespace Diligent

namespace openbus::rendering {

// Three-pass spatial SMAA using the repository's original shader and lookup tables.
// The caller supplies distinct single-sample 2D views and owns frame synchronization.
class DiligentSmaa final {
  public:
    DiligentSmaa(Diligent::IRenderDevice& device, Diligent::TEXTURE_FORMAT targetFormat);
    ~DiligentSmaa();
    DiligentSmaa(const DiligentSmaa&) = delete;
    DiligentSmaa& operator=(const DiligentSmaa&) = delete;

    void draw(Diligent::IDeviceContext& context, Diligent::ITextureView& source,
              Diligent::ITextureView& destination);

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace openbus::rendering
