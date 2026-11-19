#include "render/diligent/DiligentTexture2DArray.h"
#include "render/diligent/DiligentTextureCompression.h"

#include <gli/texture2d_array.hpp>

#include <Common/interface/RefCntAutoPtr.hpp>
#include <Graphics/GraphicsEngine/interface/RenderDevice.h>
#include <Graphics/GraphicsEngine/interface/Texture.h>
#include <Graphics/GraphicsEngine/interface/TextureView.h>

#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace openbus::rendering {

struct DiligentTexture2DArray::Impl {
    Diligent::RefCntAutoPtr<Diligent::IRenderDevice> device;
    Diligent::RefCntAutoPtr<Diligent::ITexture> texture;
    Diligent::RefCntAutoPtr<Diligent::ITextureView> shaderResource;
    std::uint32_t layerCount = 0;

    Impl(Diligent::IRenderDevice& renderDevice, const gli::texture& source)
        : device(&renderDevice) {
        using namespace Diligent;
        if (source.empty() || source.target() != gli::TARGET_2D_ARRAY || source.layers() == 0 ||
            source.layers() > std::numeric_limits<Uint32>::max() || source.faces() != 1 ||
            source.levels() == 0 || source.levels() > std::numeric_limits<Uint32>::max()) {
            throw std::invalid_argument("Diligent GLI texture array has an invalid shape");
        }

        const auto dimensions = source.extent(0);
        if (dimensions.x <= 0 || dimensions.y <= 0 || dimensions.z != 1 ||
            dimensions.x > std::numeric_limits<Uint32>::max() ||
            dimensions.y > std::numeric_limits<Uint32>::max()) {
            throw std::invalid_argument("Diligent GLI texture array has invalid dimensions");
        }
        const Uint32 width = static_cast<Uint32>(dimensions.x);
        const Uint32 height = static_cast<Uint32>(dimensions.y);
        const Uint32 arraySize = static_cast<Uint32>(source.layers());
        const Uint32 mipLevels = static_cast<Uint32>(source.levels());
        if (source.layers() > std::numeric_limits<std::size_t>::max() / source.levels() ||
            source.layers() * source.levels() > std::numeric_limits<Uint32>::max()) {
            throw std::length_error("Diligent GLI texture array has too many subresources");
        }

        TEXTURE_FORMAT format = TEX_FORMAT_UNKNOWN;
        const char* formatName = nullptr;
        Uint32 bytesPerBlock = 0;
        bool expandDxt1Alpha = false;
        bool dxt1Srgb = false;
        switch (source.format()) {
        case gli::FORMAT_RGB_DXT1_UNORM_BLOCK8:
            format = TEX_FORMAT_BC1_UNORM;
            formatName = "BC1 texture array";
            bytesPerBlock = 8;
            break;
        case gli::FORMAT_RGB_DXT1_SRGB_BLOCK8:
            format = TEX_FORMAT_BC1_UNORM_SRGB;
            formatName = "BC1 sRGB texture array";
            bytesPerBlock = 8;
            break;
        case gli::FORMAT_RGBA_DXT1_UNORM_BLOCK8:
            formatName = "BC1 alpha texture array";
            bytesPerBlock = 8;
            expandDxt1Alpha = true;
            break;
        case gli::FORMAT_RGBA_DXT1_SRGB_BLOCK8:
            formatName = "BC1 alpha sRGB texture array";
            bytesPerBlock = 8;
            expandDxt1Alpha = true;
            dxt1Srgb = true;
            break;
        case gli::FORMAT_RGBA_DXT3_UNORM_BLOCK16:
            format = TEX_FORMAT_BC2_UNORM;
            formatName = "BC2 texture array";
            bytesPerBlock = 16;
            break;
        case gli::FORMAT_RGBA_DXT3_SRGB_BLOCK16:
            format = TEX_FORMAT_BC2_UNORM_SRGB;
            formatName = "BC2 sRGB texture array";
            bytesPerBlock = 16;
            break;
        case gli::FORMAT_RGBA_DXT5_UNORM_BLOCK16:
            format = TEX_FORMAT_BC3_UNORM;
            formatName = "BC3 texture array";
            bytesPerBlock = 16;
            break;
        case gli::FORMAT_RGBA_DXT5_SRGB_BLOCK16:
            format = TEX_FORMAT_BC3_UNORM_SRGB;
            formatName = "BC3 sRGB texture array";
            bytesPerBlock = 16;
            break;
        case gli::FORMAT_R_ATI1N_UNORM_BLOCK8:
            format = TEX_FORMAT_BC4_UNORM;
            formatName = "BC4 texture array";
            bytesPerBlock = 8;
            break;
        case gli::FORMAT_R_ATI1N_SNORM_BLOCK8:
            format = TEX_FORMAT_BC4_SNORM;
            formatName = "BC4 signed texture array";
            bytesPerBlock = 8;
            break;
        case gli::FORMAT_RG_ATI2N_UNORM_BLOCK16:
            format = TEX_FORMAT_BC5_UNORM;
            formatName = "BC5 texture array";
            bytesPerBlock = 16;
            break;
        case gli::FORMAT_RG_ATI2N_SNORM_BLOCK16:
            format = TEX_FORMAT_BC5_SNORM;
            formatName = "BC5 signed texture array";
            bytesPerBlock = 16;
            break;
        case gli::FORMAT_RGB_BP_UFLOAT_BLOCK16:
            format = TEX_FORMAT_BC6H_UF16;
            formatName = "BC6H texture array";
            bytesPerBlock = 16;
            break;
        case gli::FORMAT_RGB_BP_SFLOAT_BLOCK16:
            format = TEX_FORMAT_BC6H_SF16;
            formatName = "BC6H signed texture array";
            bytesPerBlock = 16;
            break;
        case gli::FORMAT_RGBA_BP_UNORM_BLOCK16:
            format = TEX_FORMAT_BC7_UNORM;
            formatName = "BC7 texture array";
            bytesPerBlock = 16;
            break;
        case gli::FORMAT_RGBA_BP_SRGB_BLOCK16:
            format = TEX_FORMAT_BC7_UNORM_SRGB;
            formatName = "BC7 sRGB texture array";
            bytesPerBlock = 16;
            break;
        default:
            throw std::invalid_argument("Diligent compressed texture array format is unsupported");
        }

        Uint32 largestDimension = (std::max)(width, height);
        std::size_t maximumMipLevels = 1;
        for (; largestDimension > 1; largestDimension >>= 1U) {
            ++maximumMipLevels;
        }
        if (source.levels() > maximumMipLevels) {
            throw std::invalid_argument("Diligent compressed texture array has too many mips");
        }

        std::size_t totalBytes = 0;
        std::size_t totalDecodedBytes = 0;
        for (std::size_t mip = 0; mip < source.levels(); ++mip) {
            const auto extent = source.extent(mip);
            if (extent.x <= 0 || extent.y <= 0 || extent.z != 1) {
                throw std::invalid_argument(
                    "Diligent compressed texture array has invalid mip dimensions");
            }
            const std::size_t blockColumns = (static_cast<std::size_t>(extent.x) + 3U) / 4U;
            const std::size_t blockRows = (static_cast<std::size_t>(extent.y) + 3U) / 4U;
            if (blockColumns > std::numeric_limits<std::size_t>::max() / bytesPerBlock ||
                blockRows >
                    std::numeric_limits<std::size_t>::max() / (blockColumns * bytesPerBlock)) {
                throw std::length_error("Diligent compressed texture array mip size overflow");
            }
            const std::size_t rowBytes = blockColumns * bytesPerBlock;
            const std::size_t bytesPerLayer = rowBytes * blockRows;
            if (rowBytes > std::numeric_limits<Uint32>::max() ||
                bytesPerLayer > MAX_TEXTURE_BUFFER_BYTES / source.layers() ||
                totalBytes > MAX_TEXTURE_BUFFER_BYTES - bytesPerLayer * source.layers()) {
                throw std::length_error(
                    "Diligent compressed texture array exceeds the upload limit");
            }
            totalBytes += bytesPerLayer * source.layers();
            if (expandDxt1Alpha) {
                std::size_t decodedBytesPerLayer = 0;
                if (!checkedTextureBufferSize(static_cast<std::size_t>(extent.x),
                                              static_cast<std::size_t>(extent.y), 4,
                                              decodedBytesPerLayer) ||
                    decodedBytesPerLayer > MAX_TEXTURE_BUFFER_BYTES / source.layers() ||
                    totalDecodedBytes >
                        MAX_TEXTURE_BUFFER_BYTES - decodedBytesPerLayer * source.layers()) {
                    throw std::length_error("Diligent decoded BC1 array exceeds the upload limit");
                }
                totalDecodedBytes += decodedBytesPerLayer * source.layers();
            }
        }
        if (source.size() < totalBytes) {
            throw std::invalid_argument("Diligent compressed texture array data is incomplete");
        }

        std::vector<gli::texture2d> decodedLayers;
        if (expandDxt1Alpha) {
            const gli::texture2d_array typedSource(source);
            decodedLayers.reserve(source.layers());
            for (std::size_t layer = 0; layer < source.layers(); ++layer) {
                decodedLayers.push_back(
                    diligent_detail::decodeDxt1AlphaMipChain(typedSource[layer], dxt1Srgb));
            }
        }

        std::vector<TextureSubResData> subresources;
        subresources.reserve(source.layers() * source.levels());
        for (std::size_t layer = 0; layer < source.layers(); ++layer) {
            for (std::size_t mip = 0; mip < source.levels(); ++mip) {
                Uint32 rowStride = 0;
                const void* layerData = nullptr;
                if (expandDxt1Alpha) {
                    const gli::texture2d& decoded = decodedLayers[layer];
                    const auto extent = decoded.extent(mip);
                    if (extent.x <= 0 || static_cast<std::size_t>(extent.x) >
                                             std::numeric_limits<Uint32>::max() / 4U) {
                        throw std::invalid_argument("Diligent decoded BC1 array mip is invalid");
                    }
                    rowStride = static_cast<Uint32>(extent.x) * 4U;
                    layerData = decoded.data(0, 0, mip);
                } else {
                    const auto extent = source.extent(mip);
                    const std::size_t blockColumns = (static_cast<std::size_t>(extent.x) + 3U) / 4U;
                    const std::size_t blockRows = (static_cast<std::size_t>(extent.y) + 3U) / 4U;
                    const std::size_t rowBytes = blockColumns * bytesPerBlock;
                    const std::size_t bytesPerLayer = rowBytes * blockRows;
                    if (rowBytes > std::numeric_limits<Uint32>::max() ||
                        source.data(layer, 0, mip) == nullptr || source.size(mip) < bytesPerLayer) {
                        throw std::invalid_argument(
                            "Diligent compressed texture array mip data is incomplete");
                    }
                    rowStride = static_cast<Uint32>(rowBytes);
                    layerData = source.data(layer, 0, mip);
                }
                if (layerData == nullptr) {
                    throw std::invalid_argument("Diligent texture array has missing mip data");
                }
                TextureSubResData subresource;
                subresource.pData = layerData;
                subresource.Stride = rowStride;
                subresources.push_back(subresource);
            }
        }

        TextureDesc desc;
        desc.Name = formatName;
        desc.Type = RESOURCE_DIM_TEX_2D_ARRAY;
        desc.Width = width;
        desc.Height = height;
        desc.ArraySize = arraySize;
        desc.MipLevels = mipLevels;
        desc.Format = expandDxt1Alpha
                          ? (dxt1Srgb ? TEX_FORMAT_RGBA8_UNORM_SRGB : TEX_FORMAT_RGBA8_UNORM)
                          : format;
        desc.Usage = USAGE_IMMUTABLE;
        desc.BindFlags = BIND_SHADER_RESOURCE;
        TextureData data;
        data.pSubResources = subresources.data();
        data.NumSubresources = static_cast<Uint32>(subresources.size());
        renderDevice.CreateTexture(desc, &data, &texture);
        if (!texture) {
            throw std::runtime_error(std::string("Failed to create Diligent ") + formatName);
        }
        shaderResource = texture->GetDefaultView(TEXTURE_VIEW_SHADER_RESOURCE);
        if (!shaderResource) {
            throw std::runtime_error(std::string("Failed to create Diligent ") + formatName +
                                     " shader-resource view");
        }
        layerCount = arraySize;
    }

    static std::vector<const Image*> imagePointers(std::span<const Image> layers) {
        std::vector<const Image*> pointers;
        pointers.reserve(layers.size());
        for (const Image& image : layers) {
            pointers.push_back(&image);
        }
        return pointers;
    }

    static std::vector<const Image*>
    imagePointers(std::span<const std::shared_ptr<const Image>> layers) {
        std::vector<const Image*> pointers;
        pointers.reserve(layers.size());
        for (const auto& image : layers) {
            if (!image) {
                throw std::invalid_argument("Diligent texture array contains a null layer");
            }
            pointers.push_back(image.get());
        }
        return pointers;
    }

    Impl(Diligent::IRenderDevice& renderDevice, std::span<const Image> layers)
        : Impl(renderDevice, imagePointers(layers)) {}

    Impl(Diligent::IRenderDevice& renderDevice,
         std::span<const std::shared_ptr<const Image>> layers)
        : Impl(renderDevice, imagePointers(layers)) {}

    Impl(Diligent::IRenderDevice& renderDevice, std::vector<const Image*> layers)
        : device(&renderDevice) {
        if (layers.empty() || layers.size() > std::numeric_limits<Diligent::Uint32>::max()) {
            throw std::invalid_argument("Diligent texture array has an invalid layer count");
        }
        layerCount = static_cast<std::uint32_t>(layers.size());
        const Image& first = *layers.front();
        if (first.width <= 0 || first.height <= 0) {
            throw std::invalid_argument("Diligent texture array has invalid layer dimensions");
        }
        std::size_t layerBytes = 0;
        if (!checkedTextureBufferSize(static_cast<std::size_t>(first.width),
                                      static_cast<std::size_t>(first.height), 4, layerBytes) ||
            layerBytes > MAX_TEXTURE_BUFFER_BYTES / layers.size()) {
            throw std::length_error("Diligent texture array exceeds the decoded texture limit");
        }
        const std::size_t layerCount = layers.size();
        std::vector<Diligent::TextureSubResData> subresources;
        subresources.reserve(layerCount);
        for (const Image* imagePointer : layers) {
            if (imagePointer == nullptr) {
                throw std::invalid_argument("Diligent texture array contains a null layer");
            }
            const Image& image = *imagePointer;
            std::size_t imageBytes = 0;
            if (image.width != first.width || image.height != first.height ||
                !checkedTextureBufferSize(static_cast<std::size_t>(image.width),
                                          static_cast<std::size_t>(image.height), 4, imageBytes) ||
                imageBytes != image.rgba.size() || imageBytes != layerBytes) {
                throw std::invalid_argument(
                    "Diligent texture array layers must be equally sized RGBA images");
            }
            Diligent::TextureSubResData subresource;
            subresource.pData = image.rgba.data();
            subresource.Stride = static_cast<Diligent::Uint32>(first.width) * 4;
            subresources.push_back(subresource);
        }

        Diligent::TextureDesc desc;
        desc.Name = "OpenBus decoded RGBA texture array";
        desc.Type = Diligent::RESOURCE_DIM_TEX_2D_ARRAY;
        desc.Width = static_cast<Diligent::Uint32>(first.width);
        desc.Height = static_cast<Diligent::Uint32>(first.height);
        desc.ArraySize = static_cast<Diligent::Uint32>(layerCount);
        desc.MipLevels = 1;
        desc.Format = Diligent::TEX_FORMAT_RGBA8_UNORM;
        desc.Usage = Diligent::USAGE_IMMUTABLE;
        desc.BindFlags = Diligent::BIND_SHADER_RESOURCE;
        Diligent::TextureData data;
        data.pSubResources = subresources.data();
        data.NumSubresources = static_cast<Diligent::Uint32>(layerCount);
        renderDevice.CreateTexture(desc, &data, &texture);
        if (!texture) {
            throw std::runtime_error("Failed to create Diligent RGBA texture array");
        }
        shaderResource = texture->GetDefaultView(Diligent::TEXTURE_VIEW_SHADER_RESOURCE);
        if (!shaderResource) {
            throw std::runtime_error("Failed to create Diligent RGBA array shader-resource view");
        }
    }
};

DiligentTexture2DArray::DiligentTexture2DArray(Diligent::IRenderDevice& device,
                                               const gli::texture& texture)
    : impl_(std::make_unique<Impl>(device, texture)) {}

DiligentTexture2DArray::DiligentTexture2DArray(Diligent::IRenderDevice& device,
                                               std::span<const Image> layers)
    : impl_(std::make_unique<Impl>(device, layers)) {}

DiligentTexture2DArray::DiligentTexture2DArray(Diligent::IRenderDevice& device,
                                               std::span<const std::shared_ptr<const Image>> layers)
    : impl_(std::make_unique<Impl>(device, layers)) {}

DiligentTexture2DArray::~DiligentTexture2DArray() = default;

Diligent::ITextureView& DiligentTexture2DArray::shaderResourceView() const {
    return *impl_->shaderResource;
}

std::uint32_t DiligentTexture2DArray::layerCount() const {
    return impl_->layerCount;
}

Diligent::IRenderDevice& DiligentTexture2DArray::owningDevice() const {
    return *impl_->device;
}

} // namespace openbus::rendering
