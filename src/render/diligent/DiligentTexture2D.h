#pragma once

#include "assets/TextureLoader.h"

#include <filesystem>
#include <memory>

namespace gli {
class texture;
}

namespace Diligent {
struct IRenderDevice;
struct ITexture;
struct ITextureView;
} // namespace Diligent

namespace openbus::rendering {

// Immutable Diligent 2D textures uploaded from decoded RGBA8 images or BC1-BC7
// compressed mip chains. Decoded images can opt into area-averaged mip generation.
// The texture and shader-resource view retain their device resources.
class DiligentTexture2D {
  public:
    DiligentTexture2D(Diligent::IRenderDevice& device, const Image& image,
                      bool generateMipmaps = false);
    DiligentTexture2D(Diligent::IRenderDevice& device, const CompressedDds& image);
    DiligentTexture2D(Diligent::IRenderDevice& device, const gli::texture& image);
    ~DiligentTexture2D();
    DiligentTexture2D(const DiligentTexture2D&) = delete;
    DiligentTexture2D& operator=(const DiligentTexture2D&) = delete;

    // Loads through the engine texture-asset loader. generateMipmaps affects decoded images
    // only; compressed inputs retain their authored mip chains. Failures are reported as
    // exceptions.
    [[nodiscard]] static std::unique_ptr<DiligentTexture2D> load(Diligent::IRenderDevice& device,
                                                                 const std::filesystem::path& path,
                                                                 bool generateMipmaps = false);

    [[nodiscard]] Diligent::ITextureView& shaderResourceView() const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    [[nodiscard]] Diligent::IRenderDevice& owningDevice() const;
    friend class DiligentTexturedMeshRenderer;
};

} // namespace openbus::rendering
