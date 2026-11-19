#include "render/diligent/DiligentTexture2D.h"

#include "assets/TextureAssetLoader.h"
#include "core/Utf8Path.h"
#include "render/diligent/DiligentTextureCompression.h"

#include <Common/interface/RefCntAutoPtr.hpp>
#include <Graphics/GraphicsEngine/interface/RenderDevice.h>
#include <Graphics/GraphicsEngine/interface/Texture.h>
#include <Graphics/GraphicsEngine/interface/TextureView.h>
#include <gli/gli.hpp>

#include <algorithm>
#include <array>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace openbus::rendering {
namespace {

Image downsampleMip(const Image& source) {
    Image mip;
    mip.width = std::max(1, source.width / 2);
    mip.height = std::max(1, source.height / 2);
    std::size_t byteCount = 0;
    if (!checkedTextureBufferSize(static_cast<std::size_t>(mip.width),
                                  static_cast<std::size_t>(mip.height), 4, byteCount)) {
        throw std::length_error("Diligent generated mip level exceeds the upload limit");
    }
    mip.rgba.resize(byteCount);
    for (int y = 0; y < mip.height; ++y) {
        const std::uint64_t destinationYBegin =
            static_cast<std::uint64_t>(y) * static_cast<std::uint64_t>(source.height);
        const std::uint64_t destinationYEnd =
            static_cast<std::uint64_t>(y + 1) * static_cast<std::uint64_t>(source.height);
        const int sourceYBegin =
            static_cast<int>(destinationYBegin / static_cast<std::uint64_t>(mip.height));
        const int sourceYEnd =
            static_cast<int>((destinationYEnd + static_cast<std::uint64_t>(mip.height) - 1) /
                             static_cast<std::uint64_t>(mip.height));
        for (int x = 0; x < mip.width; ++x) {
            const std::uint64_t destinationXBegin =
                static_cast<std::uint64_t>(x) * static_cast<std::uint64_t>(source.width);
            const std::uint64_t destinationXEnd =
                static_cast<std::uint64_t>(x + 1) * static_cast<std::uint64_t>(source.width);
            const int sourceXBegin =
                static_cast<int>(destinationXBegin / static_cast<std::uint64_t>(mip.width));
            const int sourceXEnd =
                static_cast<int>((destinationXEnd + static_cast<std::uint64_t>(mip.width) - 1) /
                                 static_cast<std::uint64_t>(mip.width));
            std::array<std::uint64_t, 4> sums = {};
            std::uint64_t totalWeight = 0;
            for (int sourceY = sourceYBegin; sourceY < sourceYEnd; ++sourceY) {
                const std::uint64_t sourceYBeginScaled =
                    static_cast<std::uint64_t>(sourceY) * static_cast<std::uint64_t>(mip.height);
                const std::uint64_t sourceYEndScaled = static_cast<std::uint64_t>(sourceY + 1) *
                                                       static_cast<std::uint64_t>(mip.height);
                const std::uint64_t overlapY = std::min(destinationYEnd, sourceYEndScaled) -
                                               std::max(destinationYBegin, sourceYBeginScaled);
                for (int sourceX = sourceXBegin; sourceX < sourceXEnd; ++sourceX) {
                    const std::uint64_t sourceXBeginScaled =
                        static_cast<std::uint64_t>(sourceX) * static_cast<std::uint64_t>(mip.width);
                    const std::uint64_t sourceXEndScaled = static_cast<std::uint64_t>(sourceX + 1) *
                                                           static_cast<std::uint64_t>(mip.width);
                    const std::uint64_t overlapX = std::min(destinationXEnd, sourceXEndScaled) -
                                                   std::max(destinationXBegin, sourceXBeginScaled);
                    const std::uint64_t weight = overlapX * overlapY;
                    const std::size_t sourceOffset = (static_cast<std::size_t>(sourceY) *
                                                          static_cast<std::size_t>(source.width) +
                                                      static_cast<std::size_t>(sourceX)) *
                                                     4;
                    for (std::size_t channel = 0; channel < sums.size(); ++channel) {
                        sums[channel] += source.rgba[sourceOffset + channel] * weight;
                    }
                    totalWeight += weight;
                }
            }
            const std::size_t destinationOffset =
                (static_cast<std::size_t>(y) * static_cast<std::size_t>(mip.width) +
                 static_cast<std::size_t>(x)) *
                4;
            for (std::size_t channel = 0; channel < sums.size(); ++channel) {
                mip.rgba[destinationOffset + channel] =
                    static_cast<std::uint8_t>((sums[channel] + totalWeight / 2) / totalWeight);
            }
        }
    }
    return mip;
}

} // namespace

struct DiligentTexture2D::Impl {
    Diligent::RefCntAutoPtr<Diligent::IRenderDevice> device;
    Diligent::RefCntAutoPtr<Diligent::ITexture> texture;
    Diligent::RefCntAutoPtr<Diligent::ITextureView> shaderResource;

    Impl(Diligent::IRenderDevice& renderDevice, const Image& image, bool generateMipmaps)
        : device(&renderDevice) {
        std::size_t expectedBytes = 0;
        if (image.width <= 0 || image.height <= 0 ||
            !checkedTextureBufferSize(static_cast<std::size_t>(image.width),
                                      static_cast<std::size_t>(image.height), 4, expectedBytes) ||
            image.rgba.size() != expectedBytes) {
            throw std::invalid_argument("Diligent texture requires a valid decoded RGBA image");
        }
        Diligent::TextureDesc desc;
        desc.Name = "OpenBus decoded RGBA texture";
        desc.Type = Diligent::RESOURCE_DIM_TEX_2D;
        desc.Width = static_cast<Diligent::Uint32>(image.width);
        desc.Height = static_cast<Diligent::Uint32>(image.height);
        std::vector<Image> mipImages;
        if (generateMipmaps) {
            std::size_t maximumMipLevels = 1;
            for (int dimension = std::max(image.width, image.height); dimension > 1;
                 dimension >>= 1) {
                ++maximumMipLevels;
            }
            mipImages.reserve(maximumMipLevels - 1);
            std::size_t totalBytes = image.rgba.size();
            const Image* previous = &image;
            while (previous->width > 1 || previous->height > 1) {
                Image mip = downsampleMip(*previous);
                if (mip.rgba.size() > MAX_TEXTURE_BUFFER_BYTES - totalBytes) {
                    throw std::length_error(
                        "Diligent generated mip chain exceeds the upload limit");
                }
                totalBytes += mip.rgba.size();
                mipImages.push_back(std::move(mip));
                previous = &mipImages.back();
            }
        }
        desc.MipLevels = static_cast<Diligent::Uint32>(mipImages.size() + 1);
        desc.Format = Diligent::TEX_FORMAT_RGBA8_UNORM;
        desc.Usage = Diligent::USAGE_IMMUTABLE;
        desc.BindFlags = Diligent::BIND_SHADER_RESOURCE;

        std::vector<Diligent::TextureSubResData> subresources;
        subresources.reserve(mipImages.size() + 1);
        const auto appendSubresource = [&](const Image& mip) {
            Diligent::TextureSubResData subresource;
            subresource.pData = mip.rgba.data();
            subresource.Stride = static_cast<Diligent::Uint32>(mip.width) * 4;
            subresources.push_back(subresource);
        };
        appendSubresource(image);
        for (const Image& mip : mipImages) {
            appendSubresource(mip);
        }
        Diligent::TextureData data;
        data.pSubResources = subresources.data();
        data.NumSubresources = static_cast<Diligent::Uint32>(subresources.size());
        renderDevice.CreateTexture(desc, &data, &texture);
        if (!texture) {
            throw std::runtime_error("Failed to create Diligent RGBA texture");
        }
        shaderResource = texture->GetDefaultView(Diligent::TEXTURE_VIEW_SHADER_RESOURCE);
        if (!shaderResource) {
            throw std::runtime_error("Failed to create Diligent RGBA shader-resource view");
        }
    }

    Impl(Diligent::IRenderDevice& renderDevice, const CompressedDds& image)
        : device(&renderDevice) {
        using namespace Diligent;
        if (image.width <= 0 || image.height <= 0 || image.levels.empty() ||
            image.levels.size() > std::numeric_limits<Uint32>::max()) {
            throw std::invalid_argument(
                "Diligent compressed texture has invalid dimensions or mips");
        }

        Uint32 largestDimension = static_cast<Uint32>(std::max(image.width, image.height));
        std::size_t maximumMipLevels = 1;
        for (; largestDimension > 1; largestDimension >>= 1U) {
            ++maximumMipLevels;
        }
        if (image.levels.size() > maximumMipLevels) {
            throw std::invalid_argument("Diligent compressed texture has too many mip levels");
        }

        std::vector<TextureSubResData> subresources;
        subresources.reserve(image.levels.size());
        std::size_t totalBytes = 0;
        for (std::size_t mip = 0; mip < image.levels.size(); ++mip) {
            const Uint32 width =
                std::max(1U, static_cast<Uint32>(image.width) >> static_cast<Uint32>(mip));
            const Uint32 height =
                std::max(1U, static_cast<Uint32>(image.height) >> static_cast<Uint32>(mip));
            const std::size_t blockColumns = (static_cast<std::size_t>(width) + 3U) / 4U;
            const std::size_t blockRows = (static_cast<std::size_t>(height) + 3U) / 4U;
            constexpr std::size_t bytesPerBlock = 16;
            if (blockColumns >
                std::numeric_limits<std::size_t>::max() / bytesPerBlock / blockRows) {
                throw std::length_error("Diligent compressed texture mip size overflow");
            }
            const std::size_t rowBytes = blockColumns * bytesPerBlock;
            const std::size_t levelBytes = rowBytes * blockRows;
            const auto& level = image.levels[mip];
            if (level.size() != levelBytes) {
                throw std::invalid_argument(
                    "Diligent BC3 mip data does not match its block dimensions");
            }
            if (levelBytes > MAX_TEXTURE_BUFFER_BYTES ||
                totalBytes > MAX_TEXTURE_BUFFER_BYTES - levelBytes ||
                rowBytes > std::numeric_limits<Uint32>::max()) {
                throw std::length_error("Diligent compressed texture exceeds the upload limit");
            }
            totalBytes += levelBytes;
            TextureSubResData subresource;
            subresource.pData = level.data();
            subresource.Stride = static_cast<Uint32>(rowBytes);
            subresources.push_back(subresource);
        }

        TextureDesc desc;
        desc.Name = "OpenBus BC3 compressed DDS texture";
        desc.Type = RESOURCE_DIM_TEX_2D;
        desc.Width = static_cast<Uint32>(image.width);
        desc.Height = static_cast<Uint32>(image.height);
        desc.MipLevels = static_cast<Uint32>(image.levels.size());
        desc.Format = TEX_FORMAT_BC3_UNORM;
        desc.Usage = USAGE_IMMUTABLE;
        desc.BindFlags = BIND_SHADER_RESOURCE;
        TextureData data;
        data.pSubResources = subresources.data();
        data.NumSubresources = static_cast<Uint32>(subresources.size());
        renderDevice.CreateTexture(desc, &data, &texture);
        if (!texture) {
            throw std::runtime_error("Failed to create Diligent BC3 texture");
        }
        shaderResource = texture->GetDefaultView(TEXTURE_VIEW_SHADER_RESOURCE);
        if (!shaderResource) {
            throw std::runtime_error("Failed to create Diligent BC3 shader-resource view");
        }
    }

    Impl(Diligent::IRenderDevice& renderDevice, const gli::texture& image) : device(&renderDevice) {
        using namespace Diligent;
        if (image.empty() || image.target() != gli::TARGET_2D || image.layers() != 1 ||
            image.faces() != 1 || image.levels() == 0 ||
            image.levels() > std::numeric_limits<Uint32>::max()) {
            throw std::invalid_argument(
                "Diligent compressed texture requires a non-array 2D texture");
        }

        TEXTURE_FORMAT format = TEX_FORMAT_UNKNOWN;
        const char* formatName = nullptr;
        bool expandDxt1Alpha = false;
        bool dxt1Srgb = false;
        switch (image.format()) {
        case gli::FORMAT_RGB_DXT1_UNORM_BLOCK8:
            format = TEX_FORMAT_BC1_UNORM;
            formatName = "BC1";
            break;
        case gli::FORMAT_RGB_DXT1_SRGB_BLOCK8:
            format = TEX_FORMAT_BC1_UNORM_SRGB;
            formatName = "BC1 sRGB";
            break;
        case gli::FORMAT_RGBA_DXT1_UNORM_BLOCK8:
            expandDxt1Alpha = true;
            formatName = "BC1 alpha";
            break;
        case gli::FORMAT_RGBA_DXT1_SRGB_BLOCK8:
            expandDxt1Alpha = true;
            dxt1Srgb = true;
            formatName = "BC1 alpha sRGB";
            break;
        case gli::FORMAT_RGBA_DXT3_UNORM_BLOCK16:
            format = TEX_FORMAT_BC2_UNORM;
            formatName = "BC2";
            break;
        case gli::FORMAT_RGBA_DXT3_SRGB_BLOCK16:
            format = TEX_FORMAT_BC2_UNORM_SRGB;
            formatName = "BC2 sRGB";
            break;
        case gli::FORMAT_RGBA_DXT5_UNORM_BLOCK16:
            format = TEX_FORMAT_BC3_UNORM;
            formatName = "BC3";
            break;
        case gli::FORMAT_RGBA_DXT5_SRGB_BLOCK16:
            format = TEX_FORMAT_BC3_UNORM_SRGB;
            formatName = "BC3 sRGB";
            break;
        case gli::FORMAT_R_ATI1N_UNORM_BLOCK8:
            format = TEX_FORMAT_BC4_UNORM;
            formatName = "BC4";
            break;
        case gli::FORMAT_R_ATI1N_SNORM_BLOCK8:
            format = TEX_FORMAT_BC4_SNORM;
            formatName = "BC4 signed";
            break;
        case gli::FORMAT_RG_ATI2N_UNORM_BLOCK16:
            format = TEX_FORMAT_BC5_UNORM;
            formatName = "BC5";
            break;
        case gli::FORMAT_RG_ATI2N_SNORM_BLOCK16:
            format = TEX_FORMAT_BC5_SNORM;
            formatName = "BC5 signed";
            break;
        case gli::FORMAT_RGB_BP_UFLOAT_BLOCK16:
            format = TEX_FORMAT_BC6H_UF16;
            formatName = "BC6H unsigned float";
            break;
        case gli::FORMAT_RGB_BP_SFLOAT_BLOCK16:
            format = TEX_FORMAT_BC6H_SF16;
            formatName = "BC6H signed float";
            break;
        case gli::FORMAT_RGBA_BP_UNORM_BLOCK16:
            format = TEX_FORMAT_BC7_UNORM;
            formatName = "BC7";
            break;
        case gli::FORMAT_RGBA_BP_SRGB_BLOCK16:
            format = TEX_FORMAT_BC7_UNORM_SRGB;
            formatName = "BC7 sRGB";
            break;
        default:
            throw std::invalid_argument("Diligent compressed texture format is not supported");
        }

        const auto dimensions = image.extent(0);
        if (dimensions.x <= 0 || dimensions.y <= 0 ||
            dimensions.x > std::numeric_limits<Uint32>::max() ||
            dimensions.y > std::numeric_limits<Uint32>::max()) {
            throw std::invalid_argument("Diligent compressed texture has invalid dimensions");
        }
        Uint32 largestDimension = static_cast<Uint32>(std::max(dimensions.x, dimensions.y));
        std::size_t maximumMipLevels = 1;
        for (; largestDimension > 1; largestDimension >>= 1U) {
            ++maximumMipLevels;
        }
        if (image.levels() > maximumMipLevels) {
            throw std::invalid_argument("Diligent compressed texture has too many mip levels");
        }

        if (expandDxt1Alpha) {
            // Diligent's BC1 format maps to RGB-only Vulkan BC1, so expand DXT1 alpha to preserve
            // it.
            const gli::texture2d decoded =
                diligent_detail::decodeDxt1AlphaMipChain(image, dxt1Srgb);
            if (decoded.empty() || decoded.target() != gli::TARGET_2D ||
                decoded.levels() != image.levels() || decoded.extent(0).x != dimensions.x ||
                decoded.extent(0).y != dimensions.y) {
                throw std::runtime_error("Failed to decode Diligent BC1 alpha texture");
            }

            std::vector<TextureSubResData> subresources;
            subresources.reserve(decoded.levels());
            std::size_t totalBytes = 0;
            for (std::size_t mip = 0; mip < decoded.levels(); ++mip) {
                const auto extent = decoded.extent(mip);
                if (extent.x <= 0 || extent.y <= 0 ||
                    static_cast<std::size_t>(extent.x) >
                        std::numeric_limits<std::size_t>::max() / 4U) {
                    throw std::length_error("Diligent decoded BC1 mip dimensions overflow");
                }
                const std::size_t rowBytes = static_cast<std::size_t>(extent.x) * 4U;
                if (rowBytes > std::numeric_limits<std::size_t>::max() / extent.y) {
                    throw std::length_error("Diligent decoded BC1 mip size overflow");
                }
                const std::size_t levelBytes = rowBytes * extent.y;
                if (decoded.size(mip) != levelBytes || decoded.data(0, 0, mip) == nullptr) {
                    throw std::invalid_argument(
                        "Diligent decoded BC1 mip data does not match its dimensions");
                }
                if (levelBytes > MAX_TEXTURE_BUFFER_BYTES ||
                    totalBytes > MAX_TEXTURE_BUFFER_BYTES - levelBytes ||
                    rowBytes > std::numeric_limits<Uint32>::max()) {
                    throw std::length_error(
                        "Diligent decoded BC1 texture exceeds the upload limit");
                }
                totalBytes += levelBytes;
                TextureSubResData subresource;
                subresource.pData = decoded.data(0, 0, mip);
                subresource.Stride = static_cast<Uint32>(rowBytes);
                subresources.push_back(subresource);
            }

            TextureDesc desc;
            desc.Name = formatName;
            desc.Type = RESOURCE_DIM_TEX_2D;
            desc.Width = static_cast<Uint32>(dimensions.x);
            desc.Height = static_cast<Uint32>(dimensions.y);
            desc.MipLevels = static_cast<Uint32>(decoded.levels());
            desc.Format = dxt1Srgb ? TEX_FORMAT_RGBA8_UNORM_SRGB : TEX_FORMAT_RGBA8_UNORM;
            desc.Usage = USAGE_IMMUTABLE;
            desc.BindFlags = BIND_SHADER_RESOURCE;
            TextureData data;
            data.pSubResources = subresources.data();
            data.NumSubresources = static_cast<Uint32>(subresources.size());
            renderDevice.CreateTexture(desc, &data, &texture);
            if (!texture) {
                throw std::runtime_error(std::string("Failed to create Diligent ") + formatName +
                                         " texture");
            }
            shaderResource = texture->GetDefaultView(TEXTURE_VIEW_SHADER_RESOURCE);
            if (!shaderResource) {
                throw std::runtime_error(std::string("Failed to create Diligent ") + formatName +
                                         " shader-resource view");
            }
            return;
        }

        const auto blockExtent = gli::block_extent(image.format());
        const std::size_t bytesPerBlock = gli::block_size(image.format());
        if (blockExtent.x <= 0 || blockExtent.y <= 0 || bytesPerBlock == 0) {
            throw std::invalid_argument("Diligent compressed texture has invalid block metadata");
        }
        const std::size_t blockWidth = static_cast<std::size_t>(blockExtent.x);
        const std::size_t blockHeight = static_cast<std::size_t>(blockExtent.y);

        std::vector<TextureSubResData> subresources;
        subresources.reserve(image.levels());
        std::size_t totalBytes = 0;
        for (std::size_t mip = 0; mip < image.levels(); ++mip) {
            const auto extent = image.extent(mip);
            if (extent.x <= 0 || extent.y <= 0) {
                throw std::invalid_argument(
                    "Diligent compressed texture has invalid mip dimensions");
            }
            const std::size_t blockColumns =
                (static_cast<std::size_t>(extent.x) + blockWidth - 1) / blockWidth;
            const std::size_t blockRows =
                (static_cast<std::size_t>(extent.y) + blockHeight - 1) / blockHeight;
            if (blockColumns > std::numeric_limits<std::size_t>::max() / bytesPerBlock ||
                blockColumns * bytesPerBlock >
                    std::numeric_limits<std::size_t>::max() / blockRows) {
                throw std::length_error("Diligent compressed texture mip size overflow");
            }
            const std::size_t rowBytes = blockColumns * bytesPerBlock;
            const std::size_t levelBytes = rowBytes * blockRows;
            if (image.size(mip) != levelBytes || image.data(0, 0, mip) == nullptr) {
                throw std::invalid_argument(
                    "Diligent compressed texture mip data does not match its block dimensions");
            }
            if (levelBytes > MAX_TEXTURE_BUFFER_BYTES ||
                totalBytes > MAX_TEXTURE_BUFFER_BYTES - levelBytes ||
                rowBytes > std::numeric_limits<Uint32>::max()) {
                throw std::length_error("Diligent compressed texture exceeds the upload limit");
            }
            totalBytes += levelBytes;
            TextureSubResData subresource;
            subresource.pData = image.data(0, 0, mip);
            subresource.Stride = static_cast<Uint32>(rowBytes);
            subresources.push_back(subresource);
        }

        TextureDesc desc;
        desc.Name = formatName;
        desc.Type = RESOURCE_DIM_TEX_2D;
        desc.Width = static_cast<Uint32>(dimensions.x);
        desc.Height = static_cast<Uint32>(dimensions.y);
        desc.MipLevels = static_cast<Uint32>(image.levels());
        desc.Format = format;
        desc.Usage = USAGE_IMMUTABLE;
        desc.BindFlags = BIND_SHADER_RESOURCE;
        TextureData data;
        data.pSubResources = subresources.data();
        data.NumSubresources = static_cast<Uint32>(subresources.size());
        renderDevice.CreateTexture(desc, &data, &texture);
        if (!texture) {
            throw std::runtime_error(std::string("Failed to create Diligent ") + formatName +
                                     " compressed texture");
        }
        shaderResource = texture->GetDefaultView(TEXTURE_VIEW_SHADER_RESOURCE);
        if (!shaderResource) {
            throw std::runtime_error(std::string("Failed to create Diligent ") + formatName +
                                     " shader-resource view");
        }
    }
};

DiligentTexture2D::DiligentTexture2D(Diligent::IRenderDevice& device, const Image& image,
                                     bool generateMipmaps)
    : impl_(std::make_unique<Impl>(device, image, generateMipmaps)) {}
DiligentTexture2D::DiligentTexture2D(Diligent::IRenderDevice& device, const CompressedDds& image)
    : impl_(std::make_unique<Impl>(device, image)) {}
DiligentTexture2D::DiligentTexture2D(Diligent::IRenderDevice& device, const gli::texture& image)
    : impl_(std::make_unique<Impl>(device, image)) {}

std::unique_ptr<DiligentTexture2D> DiligentTexture2D::load(Diligent::IRenderDevice& device,
                                                           const std::filesystem::path& path,
                                                           bool generateMipmaps) {
    TextureAsset asset;
    if (!loadTextureAsset(path, asset)) {
        throw std::runtime_error("Failed to load Diligent texture asset: " +
                                 openbus::pathToUtf8(path));
    }
    if (asset.compressedDds) {
        return std::make_unique<DiligentTexture2D>(device, *asset.compressedDds);
    }
    if (asset.compressedTexture) {
        return std::make_unique<DiligentTexture2D>(device, *asset.compressedTexture);
    }
    if (asset.image.width <= 0 || asset.image.height <= 0 || asset.image.rgba.empty()) {
        throw std::runtime_error("Texture loader returned no supported image data for Diligent: " +
                                 openbus::pathToUtf8(path));
    }
    return std::make_unique<DiligentTexture2D>(device, asset.image, generateMipmaps);
}

DiligentTexture2D::~DiligentTexture2D() = default;

Diligent::ITextureView& DiligentTexture2D::shaderResourceView() const {
    return *impl_->shaderResource;
}

Diligent::IRenderDevice& DiligentTexture2D::owningDevice() const {
    return *impl_->device;
}

} // namespace openbus::rendering
