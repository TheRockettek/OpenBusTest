#include "render/diligent/DiligentWeatherPrecipitation.h"

#include "core/WeatherState.h"
#include "render/diligent/DiligentRenderUtils.h"

#include <Common/interface/RefCntAutoPtr.hpp>
#include <Graphics/GraphicsEngine/interface/Buffer.h>
#include <Graphics/GraphicsEngine/interface/DeviceContext.h>
#include <Graphics/GraphicsEngine/interface/PipelineState.h>
#include <Graphics/GraphicsEngine/interface/RenderDevice.h>
#include <Graphics/GraphicsEngine/interface/Shader.h>
#include <Graphics/GraphicsEngine/interface/ShaderResourceVariable.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace openbus::rendering {
namespace {

constexpr char vertexShaderSource[] = R"(
cbuffer PrecipitationParameters
{
    float4 ViewRow0;
    float4 ViewRow1;
    float4 ViewRow2;
    float4 ProjectionRow0;
    float4 ProjectionRow1;
    float4 ProjectionRow2;
    float4 ProjectionRow3;
    float4 EyeAndOffsetX;
    float4 OffsetWind;
    float4 Weather;
    float4 FogCameraAndWorldUpX;
    float4 Raster;
};

struct VertexOutput
{
    float4 Position : SV_POSITION;
    float3 ViewPosition : TEXCOORD0;
    float2 DiscCoordinate : TEXCOORD1;
};

uint hashBits(uint x)
{
    x ^= x >> 16;
    x *= 0x7feb352du;
    x ^= x >> 15;
    x *= 0x846ca68bu;
    return x ^ (x >> 16);
}

float randomValue(uint x)
{
    return float(hashBits(x) & 0x00ffffffu) / 16777216.0;
}

float positiveMod(float value, float period)
{
    return value - period * floor(value / period);
}

VertexOutput main(uint vertexId : SV_VertexID)
{
    uint type = (uint)Weather.x;
    uint particleVertexCount = type == 1 ? 2u : 6u;
    uint particleId = vertexId / particleVertexCount + 1u;
    float3 seed = float3(randomValue(particleId * 3u),
                         randomValue(particleId * 3u + 1u),
                         randomValue(particleId * 3u + 2u)) * 24.0;
    float3 position = float3(
        positiveMod(seed.x + EyeAndOffsetX.w - EyeAndOffsetX.x + 12.0, 24.0) - 12.0,
        positiveMod(seed.y + OffsetWind.x - EyeAndOffsetX.y + 12.0, 24.0) - 12.0,
        positiveMod(seed.z + OffsetWind.y - EyeAndOffsetX.z + 12.0, 24.0) - 12.0);
    float2 discCoordinate = float2(0.0, 0.0);
    if (type == 1)
    {
        if ((vertexId & 1u) != 0u)
            position += float3(OffsetWind.zw * 0.025, -0.32);
    }
    else
    {
        uint cornerIndex = vertexId % 6u;
        if (cornerIndex == 0u || cornerIndex == 3u)
            discCoordinate = float2(-1.0, -1.0);
        else if (cornerIndex == 1u)
            discCoordinate = float2(1.0, -1.0);
        else if (cornerIndex == 2u || cornerIndex == 4u)
            discCoordinate = float2(1.0, 1.0);
        else
            discCoordinate = float2(-1.0, 1.0);
    }

    float3 viewPosition = float3(
        dot(ViewRow0.xyz, position),
        dot(ViewRow1.xyz, position),
        dot(ViewRow2.xyz, position));
    float4 viewPoint = float4(viewPosition, 1.0);
    float4 clipPosition = float4(
        dot(ProjectionRow0, viewPoint),
        dot(ProjectionRow1, viewPoint),
        dot(ProjectionRow2, viewPoint),
        dot(ProjectionRow3, viewPoint));
    if (type == 2u)
    {
        clipPosition.xy += discCoordinate * float2(2.0 * Raster.x, 2.0 * Raster.y) *
                           clipPosition.w;
    }

    VertexOutput output;
    output.Position = clipPosition;
    output.ViewPosition = viewPosition;
    output.DiscCoordinate = discCoordinate;
    return output;
}
)";

constexpr char pixelShaderSource[] = R"(
cbuffer PrecipitationParameters
{
    float4 ViewRow0;
    float4 ViewRow1;
    float4 ViewRow2;
    float4 ProjectionRow0;
    float4 ProjectionRow1;
    float4 ProjectionRow2;
    float4 ProjectionRow3;
    float4 EyeAndOffsetX;
    float4 OffsetWind;
    float4 Weather;
    float4 FogCameraAndWorldUpX;
    float4 Raster;
};

struct PixelOutput
{
    float4 Color : SV_TARGET0;
    float2 Motion : SV_TARGET1;
    float Validity : SV_TARGET2;
    float LinearDepth : SV_TARGET3;
};

float fogTransmission(float3 viewPosition)
{
    float fraction = 1.0;
    float layerHeight = Weather.w;
    if (layerHeight >= 0.0)
    {
        float3 worldUp = FogCameraAndWorldUpX.yzw;
        float deltaHeight = dot(worldUp, viewPosition);
        float start = FogCameraAndWorldUpX.x - layerHeight;
        float end = start + deltaHeight;
        if (start > 0.0 && end > 0.0)
            fraction = 0.0;
        else if ((start > 0.0) != (end > 0.0))
        {
            float crossing = abs(deltaHeight) > 1.0e-6
                                 ? clamp(-start / deltaHeight, 0.0, 1.0)
                                 : 0.0;
            fraction = start > 0.0 ? crossing : 1.0 - crossing;
        }
    }
    return exp(-Weather.z * length(viewPosition) * fraction);
}

PixelOutput main(float3 viewPosition : TEXCOORD0, float2 discCoordinate : TEXCOORD1)
{
    if (Weather.x == 2.0 && dot(discCoordinate, discCoordinate) > 1.0)
        discard;
    float rain = Weather.x == 1.0 ? 1.0 : 0.0;
    float3 color = rain > 0.5 ? float3(0.68, 0.76, 0.84) : float3(0.95, 0.97, 1.0);
    float transmission = fogTransmission(viewPosition);
    float alpha = (rain > 0.5 ? 0.34 : 0.8) * transmission;
    float brightness = Weather.y;

    PixelOutput output;
    output.Color = float4(color * lerp(0.3, 1.0, brightness), alpha);
    output.Motion = float2(0.0, 0.0);
    output.Validity = 0.0;
    output.LinearDepth = max(-viewPosition.z, 0.0);
    return output;
}
)";

constexpr char pixelShaderSourceWithoutLinearDepth[] = R"(
cbuffer PrecipitationParameters
{
    float4 ViewRow0;
    float4 ViewRow1;
    float4 ViewRow2;
    float4 ProjectionRow0;
    float4 ProjectionRow1;
    float4 ProjectionRow2;
    float4 ProjectionRow3;
    float4 EyeAndOffsetX;
    float4 OffsetWind;
    float4 Weather;
    float4 FogCameraAndWorldUpX;
    float4 Raster;
};

struct PixelOutput
{
    float4 Color : SV_TARGET0;
    float2 Motion : SV_TARGET1;
    float Validity : SV_TARGET2;
};

float fogTransmission(float3 viewPosition)
{
    float fraction = 1.0;
    float layerHeight = Weather.w;
    if (layerHeight >= 0.0)
    {
        float3 worldUp = FogCameraAndWorldUpX.yzw;
        float deltaHeight = dot(worldUp, viewPosition);
        float start = FogCameraAndWorldUpX.x - layerHeight;
        float end = start + deltaHeight;
        if (start > 0.0 && end > 0.0)
            fraction = 0.0;
        else if ((start > 0.0) != (end > 0.0))
        {
            float crossing = abs(deltaHeight) > 1.0e-6
                                 ? clamp(-start / deltaHeight, 0.0, 1.0)
                                 : 0.0;
            fraction = start > 0.0 ? crossing : 1.0 - crossing;
        }
    }
    return exp(-Weather.z * length(viewPosition) * fraction);
}

PixelOutput main(float3 viewPosition : TEXCOORD0, float2 discCoordinate : TEXCOORD1)
{
    if (Weather.x == 2.0 && dot(discCoordinate, discCoordinate) > 1.0)
        discard;
    float rain = Weather.x == 1.0 ? 1.0 : 0.0;
    float3 color = rain > 0.5 ? float3(0.68, 0.76, 0.84) : float3(0.95, 0.97, 1.0);
    float transmission = fogTransmission(viewPosition);
    float alpha = (rain > 0.5 ? 0.34 : 0.8) * transmission;
    float brightness = Weather.y;

    PixelOutput output;
    output.Color = float4(color * lerp(0.3, 1.0, brightness), alpha);
    output.Motion = float2(0.0, 0.0);
    output.Validity = 0.0;
    return output;
}
)";

struct alignas(16) PrecipitationParameters {
    std::array<float, 4> viewRow0;
    std::array<float, 4> viewRow1;
    std::array<float, 4> viewRow2;
    std::array<float, 4> projectionRow0;
    std::array<float, 4> projectionRow1;
    std::array<float, 4> projectionRow2;
    std::array<float, 4> projectionRow3;
    std::array<float, 4> eyeAndOffsetX;
    std::array<float, 4> offsetWind;
    std::array<float, 4> weather;
    std::array<float, 4> fogCameraAndWorldUpX;
    std::array<float, 4> raster;
};

static_assert(sizeof(PrecipitationParameters) == sizeof(float) * 48);

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

double wrappedOffset(double seconds, double velocity) {
    return velocity == 0.0 ? 0.0 : std::fmod(seconds, 24.0 / std::abs(velocity)) * velocity;
}

} // namespace

struct DiligentWeatherPrecipitation::Impl {
    Diligent::RefCntAutoPtr<Diligent::IRenderDevice> device;
    Diligent::RefCntAutoPtr<Diligent::IPipelineState> rainPipeline;
    Diligent::RefCntAutoPtr<Diligent::IPipelineState> snowPipeline;
    Diligent::RefCntAutoPtr<Diligent::IShaderResourceBinding> rainBindings;
    Diligent::RefCntAutoPtr<Diligent::IShaderResourceBinding> snowBindings;
    Diligent::RefCntAutoPtr<Diligent::IBuffer> parameters;
    bool includeLinearDepth = true;

    Impl(Diligent::IRenderDevice& renderDevice, Diligent::TEXTURE_FORMAT colorFormat,
         DiligentMultisampling multisampling, bool withLinearDepth)
        : device(&renderDevice), includeLinearDepth(withLinearDepth) {
        using namespace Diligent;
        if (colorFormat == TEX_FORMAT_UNKNOWN) {
            throw std::invalid_argument("Diligent precipitation color format must be specified");
        }
        diligent_detail::validateSamples(renderDevice, colorFormat, TEX_FORMAT_D32_FLOAT,
                                         multisampling.samples);
        auto vertexShader =
            diligent_detail::createShader(renderDevice, SHADER_TYPE_VERTEX,
                                          "OpenBus weather precipitation VS", vertexShaderSource);
        auto pixelShader = diligent_detail::createShader(
            renderDevice, SHADER_TYPE_PIXEL, "OpenBus weather precipitation PS",
            includeLinearDepth ? pixelShaderSource : pixelShaderSourceWithoutLinearDepth);

        const auto createPipeline = [&](PRIMITIVE_TOPOLOGY topology, const char* name,
                                        RefCntAutoPtr<IPipelineState>& pipeline,
                                        RefCntAutoPtr<IShaderResourceBinding>& bindings) {
            GraphicsPipelineStateCreateInfo info;
            info.PSODesc.Name = name;
            info.PSODesc.PipelineType = PIPELINE_TYPE_GRAPHICS;
            info.GraphicsPipeline.NumRenderTargets = includeLinearDepth ? 4 : 3;
            info.GraphicsPipeline.RTVFormats[0] = colorFormat;
            info.GraphicsPipeline.RTVFormats[1] = TEX_FORMAT_RG32_FLOAT;
            info.GraphicsPipeline.RTVFormats[2] = TEX_FORMAT_R8_UNORM;
            if (includeLinearDepth) {
                info.GraphicsPipeline.RTVFormats[3] = TEX_FORMAT_R32_FLOAT;
            }
            info.GraphicsPipeline.DSVFormat = TEX_FORMAT_D32_FLOAT;
            info.GraphicsPipeline.SmplDesc.Count = multisampling.samples;
            info.GraphicsPipeline.PrimitiveTopology = topology;
            info.GraphicsPipeline.RasterizerDesc.CullMode = CULL_MODE_NONE;
            info.GraphicsPipeline.DepthStencilDesc.DepthEnable = True;
            info.GraphicsPipeline.DepthStencilDesc.DepthWriteEnable = False;
            info.GraphicsPipeline.DepthStencilDesc.DepthFunc = COMPARISON_FUNC_LESS_EQUAL;
            auto& colorBlend = info.GraphicsPipeline.BlendDesc.RenderTargets[0];
            colorBlend.BlendEnable = True;
            colorBlend.SrcBlend = BLEND_FACTOR_SRC_ALPHA;
            colorBlend.DestBlend = BLEND_FACTOR_INV_SRC_ALPHA;
            colorBlend.BlendOp = BLEND_OPERATION_ADD;
            colorBlend.SrcBlendAlpha = BLEND_FACTOR_ONE;
            colorBlend.DestBlendAlpha = BLEND_FACTOR_INV_SRC_ALPHA;
            colorBlend.BlendOpAlpha = BLEND_OPERATION_ADD;
            info.pVS = vertexShader;
            info.pPS = pixelShader;
            renderDevice.CreateGraphicsPipelineState(info, &pipeline);
            require(static_cast<bool>(pipeline),
                    "Failed to create Diligent precipitation pipeline");
            auto* vertexVariable =
                pipeline->GetStaticVariableByName(SHADER_TYPE_VERTEX, "PrecipitationParameters");
            auto* pixelVariable =
                pipeline->GetStaticVariableByName(SHADER_TYPE_PIXEL, "PrecipitationParameters");
            require(vertexVariable != nullptr && pixelVariable != nullptr,
                    "Diligent precipitation parameter binding is missing");
            vertexVariable->Set(parameters);
            pixelVariable->Set(parameters);
            pipeline->CreateShaderResourceBinding(&bindings, true);
            require(static_cast<bool>(bindings),
                    "Failed to create Diligent precipitation shader bindings");
        };

        BufferDesc bufferInfo;
        bufferInfo.Name = "OpenBus weather precipitation parameters";
        bufferInfo.Size = sizeof(PrecipitationParameters);
        bufferInfo.Usage = USAGE_DYNAMIC;
        bufferInfo.BindFlags = BIND_UNIFORM_BUFFER;
        bufferInfo.CPUAccessFlags = CPU_ACCESS_WRITE;
        renderDevice.CreateBuffer(bufferInfo, nullptr, &parameters);
        require(static_cast<bool>(parameters),
                "Failed to create Diligent precipitation parameter buffer");
        createPipeline(PRIMITIVE_TOPOLOGY_LINE_LIST, "OpenBus rain particles", rainPipeline,
                       rainBindings);
        createPipeline(PRIMITIVE_TOPOLOGY_TRIANGLE_LIST, "OpenBus snow particles", snowPipeline,
                       snowBindings);
    }
};

DiligentWeatherPrecipitation::DiligentWeatherPrecipitation(Diligent::IRenderDevice& device,
                                                           Diligent::TEXTURE_FORMAT colorFormat,
                                                           DiligentMultisampling multisampling,
                                                           bool includeLinearDepth)
    : impl_(std::make_unique<Impl>(device, colorFormat, multisampling, includeLinearDepth)) {}

DiligentWeatherPrecipitation::~DiligentWeatherPrecipitation() = default;

bool DiligentWeatherPrecipitation::draw(
    Diligent::IDeviceContext& context, const Matrix4& view, const Matrix4& projection,
    Diligent::Uint32 targetWidth, Diligent::Uint32 targetHeight, const DiligentWeatherFog& fog,
    int type, double rate, double simulationSeconds, std::optional<double> windDirectionDegrees,
    std::optional<double> windSpeedMetersPerSecond, std::optional<double> brightness) {
    WeatherState weather;
    weather.precipitationType = type;
    weather.precipitationRate = rate;
    weather.windDirectionDegrees = windDirectionDegrees;
    weather.windSpeedMetersPerSecond = windSpeedMetersPerSecond;
    weather.brightness = brightness;
    validateWeatherState(weather);
    if (!std::isfinite(simulationSeconds) || simulationSeconds < 0.0) {
        throw std::invalid_argument("Diligent precipitation time must be finite and non-negative");
    }
    if (type == 0 || rate == 0.0) {
        return false;
    }
    const auto finiteMatrix = [](const Matrix4& matrix) {
        return std::all_of(matrix.begin(), matrix.end(),
                           [](double value) { return std::isfinite(value); });
    };
    if (!finiteMatrix(view) || !finiteMatrix(projection) || targetWidth == 0 || targetHeight == 0 ||
        !std::isfinite(fog.density) || fog.density < 0.0F || !std::isfinite(fog.layerHeight) ||
        (fog.layerHeight < 0.0F && fog.layerHeight != -1.0F) || !std::isfinite(fog.cameraHeight) ||
        !std::all_of(
            fog.worldUpView.begin(), fog.worldUpView.end(),
            [](float value) { return std::isfinite(value); })) {
        throw std::invalid_argument("Diligent precipitation camera or fog is invalid");
    }
    if (!std::all_of(projection.begin(), projection.end(), [](double value) {
            return std::abs(value) <= static_cast<double>(std::numeric_limits<float>::max());
        })) {
        throw std::invalid_argument("Diligent precipitation projection exceeds float range");
    }

    const std::array<double, 3> eye = {
        -(view[0] * view[12] + view[1] * view[13] + view[2] * view[14]),
        -(view[4] * view[12] + view[5] * view[13] + view[6] * view[14]),
        -(view[8] * view[12] + view[9] * view[13] + view[10] * view[14])};
    if (!std::all_of(eye.begin(), eye.end(), [](double value) { return std::isfinite(value); })) {
        throw std::invalid_argument("Diligent precipitation camera position is invalid");
    }
    const double direction = windDirectionDegrees.value_or(0.0) * 3.14159265358979323846 / 180.0;
    const double speed = windDirectionDegrees && windSpeedMetersPerSecond
                             ? std::min(*windSpeedMetersPerSecond, 50.0)
                             : 0.0;
    const double windX = -std::sin(direction) * speed;
    const double windY = -std::cos(direction) * speed;
    const double fallSpeed = type == 1 ? -9.0 : -1.2;
    const float sceneBrightness = static_cast<float>(brightness.value_or(1.0));
    const auto viewRows = diligent_detail::matrixRows(view);
    const auto projectionRows = diligent_detail::matrixRows(projection);
    const PrecipitationParameters values{
        {viewRows[0], viewRows[1], viewRows[2], viewRows[3]},
        {viewRows[4], viewRows[5], viewRows[6], viewRows[7]},
        {viewRows[8], viewRows[9], viewRows[10], viewRows[11]},
        {projectionRows[0], projectionRows[1], projectionRows[2], projectionRows[3]},
        {projectionRows[4], projectionRows[5], projectionRows[6], projectionRows[7]},
        {projectionRows[8], projectionRows[9], projectionRows[10], projectionRows[11]},
        {projectionRows[12], projectionRows[13], projectionRows[14], projectionRows[15]},
        {static_cast<float>(std::fmod(eye[0], 24.0)), static_cast<float>(std::fmod(eye[1], 24.0)),
         static_cast<float>(std::fmod(eye[2], 24.0)),
         static_cast<float>(wrappedOffset(simulationSeconds, windX))},
        {static_cast<float>(wrappedOffset(simulationSeconds, windY)),
         static_cast<float>(wrappedOffset(simulationSeconds, fallSpeed)), static_cast<float>(windX),
         static_cast<float>(windY)},
        {static_cast<float>(type), sceneBrightness, fog.density, fog.layerHeight},
        {fog.cameraHeight, fog.worldUpView[0], fog.worldUpView[1], fog.worldUpView[2]},
        {2.0F / static_cast<float>(targetWidth), 2.0F / static_cast<float>(targetHeight), 0.0F,
         0.0F}};

    const std::size_t particleCount =
        static_cast<std::size_t>(std::lround(rate * (type == 1 ? 12000.0 : 6000.0)));
    if (particleCount == 0) {
        return false;
    }
    const std::size_t vertices = particleCount * (type == 1 ? 2U : 6U);
    if (vertices > std::numeric_limits<Diligent::Uint32>::max()) {
        throw std::length_error("Diligent precipitation vertex count exceeds supported range");
    }

    void* mapped = nullptr;
    context.MapBuffer(impl_->parameters, Diligent::MAP_WRITE, Diligent::MAP_FLAG_DISCARD, mapped);
    require(mapped != nullptr, "Failed to map Diligent precipitation parameters");
    std::memcpy(mapped, &values, sizeof(values));
    context.UnmapBuffer(impl_->parameters, Diligent::MAP_WRITE);

    const auto& pipeline = type == 1 ? impl_->rainPipeline : impl_->snowPipeline;
    const auto& bindings = type == 1 ? impl_->rainBindings : impl_->snowBindings;
    context.SetPipelineState(pipeline);
    context.CommitShaderResources(bindings, Diligent::RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    Diligent::DrawAttribs attributes;
    attributes.NumVertices = static_cast<Diligent::Uint32>(vertices);
    context.Draw(attributes);
    return true;
}

} // namespace openbus::rendering
