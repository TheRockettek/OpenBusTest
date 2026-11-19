#pragma once

#include "render/CameraMath.h"

#include <Common/interface/RefCntAutoPtr.hpp>
#include <Graphics/GraphicsEngine/interface/RenderDevice.h>
#include <Graphics/GraphicsEngine/interface/Shader.h>

#include <array>
#include <bit>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace openbus::rendering::diligent_detail {

inline void validateSamples(Diligent::IRenderDevice& device, Diligent::TEXTURE_FORMAT color,
                            Diligent::TEXTURE_FORMAT depth, Diligent::Uint32 samples) {
    using namespace Diligent;
    if (!std::has_single_bit(samples) || samples > SAMPLE_COUNT_MAX ||
        color <= TEX_FORMAT_UNKNOWN || color >= TEX_FORMAT_NUM_FORMATS ||
        depth >= TEX_FORMAT_NUM_FORMATS) {
        throw std::invalid_argument("Invalid Diligent render-target format or sample count");
    }
    const auto& colorInfo = device.GetTextureFormatInfoExt(color);
    if (!(colorInfo.BindFlags & BIND_RENDER_TARGET) || !(colorInfo.SampleCounts & samples)) {
        throw std::invalid_argument("Diligent color format does not support requested samples");
    }
    if (depth != TEX_FORMAT_UNKNOWN) {
        const auto& depthInfo = device.GetTextureFormatInfoExt(depth);
        if (!(depthInfo.BindFlags & BIND_DEPTH_STENCIL) || !(depthInfo.SampleCounts & samples)) {
            throw std::invalid_argument("Diligent depth format does not support requested samples");
        }
    }
}

inline Diligent::RefCntAutoPtr<Diligent::IShader> createShader(Diligent::IRenderDevice& device,
                                                               Diligent::SHADER_TYPE type,
                                                               const char* name, const char* source,
                                                               bool combinedTextureSamplers = false,
                                                               const char* entryPoint = "main") {
    Diligent::ShaderCreateInfo info;
    info.Desc.Name = name;
    info.Desc.ShaderType = type;
    info.Desc.UseCombinedTextureSamplers = combinedTextureSamplers;
    info.SourceLanguage = Diligent::SHADER_SOURCE_LANGUAGE_HLSL;
    info.EntryPoint = entryPoint;
    info.Source = source;
    Diligent::RefCntAutoPtr<Diligent::IShader> shader;
    device.CreateShader(info, &shader);
    if (!shader) {
        throw std::runtime_error(std::string("Failed to compile Diligent shader: ") + name);
    }
    return shader;
}

inline std::array<float, 16> matrixRows(const Matrix4& matrix) {
    std::array<float, 16> rows{};
    for (std::size_t row = 0; row < 4; ++row) {
        for (std::size_t column = 0; column < 4; ++column) {
            const double value = matrix[column * 4 + row];
            if (!std::isfinite(value) ||
                std::abs(value) > static_cast<double>(std::numeric_limits<float>::max())) {
                throw std::invalid_argument("Diligent transform is not finite float data");
            }
            rows[row * 4 + column] = static_cast<float>(value);
        }
    }
    return rows;
}

inline std::array<float, 16> clipTransformRows(const Matrix4& matrix, float minimumNdcZ) {
    if (!std::isfinite(minimumNdcZ)) {
        throw std::invalid_argument("Diligent device reported invalid NDC depth");
    }
    std::array<float, 16> rows = matrixRows(matrix);
    const float zScale = (1.0f - minimumNdcZ) * 0.5f;
    const float zBias = (1.0f + minimumNdcZ) * 0.5f;
    for (std::size_t column = 0; column < 4; ++column) {
        rows[8 + column] = rows[8 + column] * zScale + rows[12 + column] * zBias;
        if (!std::isfinite(rows[8 + column])) {
            throw std::invalid_argument("Diligent depth transform overflowed");
        }
    }
    return rows;
}

} // namespace openbus::rendering::diligent_detail
