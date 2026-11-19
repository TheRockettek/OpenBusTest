#include "render/diligent/DiligentWeatherClouds.h"

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
#include <stdexcept>

namespace openbus::rendering {
namespace {

constexpr char vertexShaderSource[] = R"(
struct VertexOutput
{
    float4 Position : SV_POSITION;
    float2 PositionNdc : TEXCOORD0;
};
VertexOutput main(uint vertexId : SV_VertexID)
{
    float2 position = float2((vertexId << 1) & 2, vertexId & 2);
    VertexOutput output;
    output.PositionNdc = float2(position.x * 2.0 - 1.0, 1.0 - position.y * 2.0);
    output.Position = float4(output.PositionNdc, 0.0, 1.0);
    return output;
}
)";

constexpr char pixelShaderSource[] = R"(
cbuffer CloudParameters
{
    float4 ViewRow0;
    float4 ViewRow1;
    float4 ViewRow2;
    float4 ProjectionAndFog;
    float4 CloudOriginHeightStyle;
    float4 CloudBrightnessBackground;
    float4 FogCameraAndWorldUpX;
};

struct PixelOutput
{
    float4 Color : SV_TARGET0;
    float2 Motion : SV_TARGET1;
    float Validity : SV_TARGET2;
    float LinearDepth : SV_TARGET3;
};

float hash2(float2 p)
{
    p -= 1024.0 * floor(p / 1024.0);
    return frac(sin(dot(p, float2(127.1, 311.7))) * 43758.5453);
}

float noise2(float2 p)
{
    float2 cell = floor(p);
    float2 f = frac(p);
    f = f * f * (3.0 - 2.0 * f);
    return lerp(lerp(hash2(cell), hash2(cell + float2(1.0, 0.0)), f.x),
                lerp(hash2(cell + float2(0.0, 1.0)), hash2(cell + float2(1.0, 1.0)), f.x),
                f.y);
}

float fogTransmission(float3 viewPosition)
{
    float fraction = 1.0;
    float layerHeight = ProjectionAndFog.w;
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
    return exp(-ProjectionAndFog.z * length(viewPosition) * fraction);
}

PixelOutput main(float2 positionNdc : TEXCOORD0)
{
    float3 viewRay = normalize(float3(positionNdc / ProjectionAndFog.xy, -1.0));
    float3 worldRay = normalize(float3(
        dot(float3(ViewRow0.x, ViewRow1.x, ViewRow2.x), viewRay),
        dot(float3(ViewRow0.y, ViewRow1.y, ViewRow2.y), viewRay),
        dot(float3(ViewRow0.z, ViewRow1.z, ViewRow2.z), viewRay)));

    float coverage = 0.0;
    float shading = 0.0;
    float cloudHeight = CloudOriginHeightStyle.z;
    if (worldRay.z > 0.01 && cloudHeight > 0.0)
    {
        float distanceToLayer = cloudHeight / worldRay.z;
        float2 origin = CloudOriginHeightStyle.xy;
        float2 p = (origin + worldRay.xy * distanceToLayer) / 1800.0;
        float noise = 0.5333 * noise2(p) + 0.2667 * noise2(p * 2.0)
                    + 0.1333 * noise2(p * 4.0) + 0.0667 * noise2(p * 8.0);
        int style = (int)CloudOriginHeightStyle.w;
        float threshold = style == 0 ? 0.62 : style == 1 ? 0.53 :
                          style == 2 ? 0.43 : 0.24;
        coverage = smoothstep(threshold, threshold + 0.12, noise);
        coverage *= smoothstep(0.01, 0.12, worldRay.z);
        coverage *= fogTransmission(viewRay * distanceToLayer);
        shading = noise;
    }

    float brightness = CloudBrightnessBackground.x;
    float3 background = CloudBrightnessBackground.yzw;
    float3 cloudColor = lerp(float3(0.66, 0.69, 0.73), float3(1.0, 1.0, 1.0), shading) *
                        brightness;
    PixelOutput output;
    output.Color = float4(lerp(background, cloudColor, coverage), 1.0);
    output.Motion = float2(0.0, 0.0);
    output.Validity = 0.0;
    output.LinearDepth = 0.0;
    return output;
}
)";

constexpr char pixelShaderSourceWithoutLinearDepth[] = R"(
cbuffer CloudParameters
{
    float4 ViewRow0;
    float4 ViewRow1;
    float4 ViewRow2;
    float4 ProjectionAndFog;
    float4 CloudOriginHeightStyle;
    float4 CloudBrightnessBackground;
    float4 FogCameraAndWorldUpX;
};

struct PixelOutput
{
    float4 Color : SV_TARGET0;
    float2 Motion : SV_TARGET1;
    float Validity : SV_TARGET2;
};

float hash2(float2 p)
{
    p -= 1024.0 * floor(p / 1024.0);
    return frac(sin(dot(p, float2(127.1, 311.7))) * 43758.5453);
}

float noise2(float2 p)
{
    float2 cell = floor(p);
    float2 f = frac(p);
    f = f * f * (3.0 - 2.0 * f);
    return lerp(lerp(hash2(cell), hash2(cell + float2(1.0, 0.0)), f.x),
                lerp(hash2(cell + float2(0.0, 1.0)), hash2(cell + float2(1.0, 1.0)), f.x),
                f.y);
}

float fogTransmission(float3 viewPosition)
{
    float fraction = 1.0;
    float layerHeight = ProjectionAndFog.w;
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
    return exp(-ProjectionAndFog.z * length(viewPosition) * fraction);
}

PixelOutput main(float2 positionNdc : TEXCOORD0)
{
    float3 viewRay = normalize(float3(positionNdc / ProjectionAndFog.xy, -1.0));
    float3 worldRay = normalize(float3(
        dot(float3(ViewRow0.x, ViewRow1.x, ViewRow2.x), viewRay),
        dot(float3(ViewRow0.y, ViewRow1.y, ViewRow2.y), viewRay),
        dot(float3(ViewRow0.z, ViewRow1.z, ViewRow2.z), viewRay)));
    float coverage = 0.0;
    float shading = 0.0;
    float cloudHeight = CloudOriginHeightStyle.z;
    if (worldRay.z > 0.01 && cloudHeight > 0.0)
    {
        float distanceToLayer = cloudHeight / worldRay.z;
        float2 p = (CloudOriginHeightStyle.xy + worldRay.xy * distanceToLayer) / 1800.0;
        float noise = 0.5333 * noise2(p) + 0.2667 * noise2(p * 2.0)
                    + 0.1333 * noise2(p * 4.0) + 0.0667 * noise2(p * 8.0);
        int style = (int)CloudOriginHeightStyle.w;
        float threshold = style == 0 ? 0.62 : style == 1 ? 0.53 :
                          style == 2 ? 0.43 : 0.24;
        coverage = smoothstep(threshold, threshold + 0.12, noise);
        coverage *= smoothstep(0.01, 0.12, worldRay.z);
        coverage *= fogTransmission(viewRay * distanceToLayer);
        shading = noise;
    }
    float brightness = CloudBrightnessBackground.x;
    float3 background = CloudBrightnessBackground.yzw;
    float3 cloudColor = lerp(float3(0.66, 0.69, 0.73), float3(1.0, 1.0, 1.0), shading) *
                        brightness;
    PixelOutput output;
    output.Color = float4(lerp(background, cloudColor, coverage), 1.0);
    output.Motion = float2(0.0, 0.0);
    output.Validity = 0.0;
    return output;
}
)";

constexpr char pixelShaderSourceColorOnly[] = R"(
cbuffer CloudParameters
{
    float4 ViewRow0;
    float4 ViewRow1;
    float4 ViewRow2;
    float4 ProjectionAndFog;
    float4 CloudOriginHeightStyle;
    float4 CloudBrightnessBackground;
    float4 FogCameraAndWorldUpX;
};

float hash2(float2 p)
{
    p -= 1024.0 * floor(p / 1024.0);
    return frac(sin(dot(p, float2(127.1, 311.7))) * 43758.5453);
}

float noise2(float2 p)
{
    float2 cell = floor(p);
    float2 f = frac(p);
    f = f * f * (3.0 - 2.0 * f);
    return lerp(lerp(hash2(cell), hash2(cell + float2(1.0, 0.0)), f.x),
                lerp(hash2(cell + float2(0.0, 1.0)), hash2(cell + float2(1.0, 1.0)), f.x),
                f.y);
}

float fogTransmission(float3 viewPosition)
{
    float fraction = 1.0;
    float layerHeight = ProjectionAndFog.w;
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
    return exp(-ProjectionAndFog.z * length(viewPosition) * fraction);
}

float4 main(float2 positionNdc : TEXCOORD0) : SV_TARGET0
{
    float3 viewRay = normalize(float3(positionNdc / ProjectionAndFog.xy, -1.0));
    float3 worldRay = normalize(float3(
        dot(float3(ViewRow0.x, ViewRow1.x, ViewRow2.x), viewRay),
        dot(float3(ViewRow0.y, ViewRow1.y, ViewRow2.y), viewRay),
        dot(float3(ViewRow0.z, ViewRow1.z, ViewRow2.z), viewRay)));

    float coverage = 0.0;
    float shading = 0.0;
    float cloudHeight = CloudOriginHeightStyle.z;
    if (worldRay.z > 0.01 && cloudHeight > 0.0)
    {
        float distanceToLayer = cloudHeight / worldRay.z;
        float2 p = (CloudOriginHeightStyle.xy + worldRay.xy * distanceToLayer) / 1800.0;
        float noise = 0.5333 * noise2(p) + 0.2667 * noise2(p * 2.0)
                    + 0.1333 * noise2(p * 4.0) + 0.0667 * noise2(p * 8.0);
        int style = (int)CloudOriginHeightStyle.w;
        float threshold = style == 0 ? 0.62 : style == 1 ? 0.53 :
                          style == 2 ? 0.43 : 0.24;
        coverage = smoothstep(threshold, threshold + 0.12, noise);
        coverage *= smoothstep(0.01, 0.12, worldRay.z);
        coverage *= fogTransmission(viewRay * distanceToLayer);
        shading = noise;
    }
    float brightness = CloudBrightnessBackground.x;
    float3 background = CloudBrightnessBackground.yzw;
    float3 cloudColor = lerp(float3(0.66, 0.69, 0.73), float3(1.0, 1.0, 1.0), shading) *
                        brightness;
    return float4(lerp(background, cloudColor, coverage), 1.0);
}
)";

struct alignas(16) CloudParameters {
    std::array<float, 4> viewRow0;
    std::array<float, 4> viewRow1;
    std::array<float, 4> viewRow2;
    std::array<float, 4> projectionAndFog;
    std::array<float, 4> cloudOriginHeightStyle;
    std::array<float, 4> cloudBrightnessBackground;
    std::array<float, 4> fogCameraAndWorldUpX;
};

static_assert(sizeof(CloudParameters) == sizeof(float) * 28);

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

} // namespace

struct DiligentWeatherClouds::Impl {
    Diligent::RefCntAutoPtr<Diligent::IRenderDevice> device;
    Diligent::RefCntAutoPtr<Diligent::IPipelineState> pipeline;
    Diligent::RefCntAutoPtr<Diligent::IShaderResourceBinding> bindings;
    Diligent::RefCntAutoPtr<Diligent::IBuffer> parameters;
    bool includeLinearDepth = true;
    bool colorOnly = false;

    Impl(Diligent::IRenderDevice& renderDevice, Diligent::TEXTURE_FORMAT colorFormat,
         DiligentMultisampling multisampling, bool withLinearDepth, bool withColorOnly)
        : device(&renderDevice), includeLinearDepth(withLinearDepth), colorOnly(withColorOnly) {
        using namespace Diligent;
        if (colorFormat == TEX_FORMAT_UNKNOWN) {
            throw std::invalid_argument("Diligent cloud color format must be specified");
        }
        diligent_detail::validateSamples(renderDevice, colorFormat, TEX_FORMAT_D32_FLOAT,
                                         multisampling.samples);
        auto vertexShader = diligent_detail::createShader(
            renderDevice, SHADER_TYPE_VERTEX, "OpenBus weather cloud VS", vertexShaderSource);
        auto pixelShader = diligent_detail::createShader(
            renderDevice, SHADER_TYPE_PIXEL, "OpenBus weather cloud PS",
            colorOnly            ? pixelShaderSourceColorOnly
            : includeLinearDepth ? pixelShaderSource
                                 : pixelShaderSourceWithoutLinearDepth);

        GraphicsPipelineStateCreateInfo info;
        info.PSODesc.Name = "OpenBus procedural weather clouds";
        info.PSODesc.PipelineType = PIPELINE_TYPE_GRAPHICS;
        info.GraphicsPipeline.NumRenderTargets = colorOnly ? 1 : includeLinearDepth ? 4 : 3;
        info.GraphicsPipeline.RTVFormats[0] = colorFormat;
        if (!colorOnly) {
            info.GraphicsPipeline.RTVFormats[1] = TEX_FORMAT_RG32_FLOAT;
            info.GraphicsPipeline.RTVFormats[2] = TEX_FORMAT_R8_UNORM;
            if (includeLinearDepth) {
                info.GraphicsPipeline.RTVFormats[3] = TEX_FORMAT_R32_FLOAT;
            }
        }
        info.GraphicsPipeline.DSVFormat = TEX_FORMAT_D32_FLOAT;
        info.GraphicsPipeline.SmplDesc.Count = multisampling.samples;
        info.GraphicsPipeline.PrimitiveTopology = PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
        info.GraphicsPipeline.RasterizerDesc.CullMode = CULL_MODE_NONE;
        info.GraphicsPipeline.DepthStencilDesc.DepthEnable = False;
        info.GraphicsPipeline.DepthStencilDesc.DepthWriteEnable = False;
        info.pVS = vertexShader;
        info.pPS = pixelShader;
        renderDevice.CreateGraphicsPipelineState(info, &pipeline);
        require(static_cast<bool>(pipeline), "Failed to create Diligent weather cloud pipeline");

        BufferDesc bufferInfo;
        bufferInfo.Name = "OpenBus weather cloud parameters";
        bufferInfo.Size = sizeof(CloudParameters);
        bufferInfo.Usage = USAGE_DYNAMIC;
        bufferInfo.BindFlags = BIND_UNIFORM_BUFFER;
        bufferInfo.CPUAccessFlags = CPU_ACCESS_WRITE;
        renderDevice.CreateBuffer(bufferInfo, nullptr, &parameters);
        require(static_cast<bool>(parameters), "Failed to create Diligent cloud parameter buffer");
        auto* variable = pipeline->GetStaticVariableByName(SHADER_TYPE_PIXEL, "CloudParameters");
        require(variable != nullptr, "Diligent cloud parameter binding is missing");
        variable->Set(parameters);
        pipeline->CreateShaderResourceBinding(&bindings, true);
        require(static_cast<bool>(bindings), "Failed to create Diligent cloud shader bindings");
    }
};

DiligentWeatherClouds::DiligentWeatherClouds(Diligent::IRenderDevice& device,
                                             Diligent::TEXTURE_FORMAT colorFormat,
                                             DiligentMultisampling multisampling,
                                             bool includeLinearDepth, bool colorOnly)
    : impl_(std::make_unique<Impl>(device, colorFormat, multisampling, includeLinearDepth,
                                   colorOnly)) {}

DiligentWeatherClouds::~DiligentWeatherClouds() = default;

bool DiligentWeatherClouds::draw(Diligent::IDeviceContext& context, const Matrix4& view,
                                 const Matrix4& projection, const DiligentWeatherFog& fog,
                                 std::optional<WeatherCloudType> type,
                                 std::optional<double> baseAboveGroundMeters,
                                 std::optional<double> windDirectionDegrees,
                                 std::optional<double> windSpeedMetersPerSecond,
                                 std::optional<double> brightness, double groundHeightMeters,
                                 double simulationSeconds) {
    if (!type) {
        return false;
    }

    WeatherState weather;
    weather.cloudType = type;
    weather.cloudBaseAboveGroundMeters = baseAboveGroundMeters;
    weather.windDirectionDegrees = windDirectionDegrees;
    weather.windSpeedMetersPerSecond = windSpeedMetersPerSecond;
    weather.brightness = brightness;
    validateWeatherState(weather);

    const auto finiteMatrix = [](const Matrix4& matrix) {
        return std::all_of(matrix.begin(), matrix.end(),
                           [](double value) { return std::isfinite(value); });
    };
    if (!finiteMatrix(view) || !finiteMatrix(projection) || !std::isfinite(groundHeightMeters) ||
        !std::isfinite(simulationSeconds)) {
        throw std::invalid_argument("Diligent cloud camera, ground, or time is invalid");
    }
    if (!std::isfinite(fog.density) || fog.density < 0.0F || !std::isfinite(fog.layerHeight) ||
        (fog.layerHeight < 0.0F && fog.layerHeight != -1.0F) || !std::isfinite(fog.cameraHeight) ||
        !std::all_of(
            fog.worldUpView.begin(), fog.worldUpView.end(),
            [](float value) { return std::isfinite(value); })) {
        throw std::invalid_argument("Diligent cloud fog parameters are invalid");
    }
    if (!std::isfinite(projection[0]) || !std::isfinite(projection[5]) || projection[0] == 0.0 ||
        projection[5] == 0.0) {
        throw std::invalid_argument("Diligent cloud projection scale is invalid");
    }

    const std::array<double, 3> eye = {
        -(view[0] * view[12] + view[1] * view[13] + view[2] * view[14]),
        -(view[4] * view[12] + view[5] * view[13] + view[6] * view[14]),
        -(view[8] * view[12] + view[9] * view[13] + view[10] * view[14])};
    const double direction = windDirectionDegrees.value_or(0.0) * 3.14159265358979323846 / 180.0;
    const double distance = windDirectionDegrees && windSpeedMetersPerSecond
                                ? *windSpeedMetersPerSecond * simulationSeconds
                                : 0.0;
    constexpr double cloudPeriodMeters = 1800.0 * 1024.0;
    const double originX = eye[0] + std::sin(direction) * distance;
    const double originY = eye[1] + std::cos(direction) * distance;
    const double height = groundHeightMeters + baseAboveGroundMeters.value_or(1500.0) - eye[2];
    if (!std::isfinite(originX) || !std::isfinite(originY) || !std::isfinite(height)) {
        throw std::invalid_argument("Diligent cloud wind or height is invalid");
    }
    const double clampedHeight =
        std::clamp(height, -MAX_CLOUD_BASE_ABOVE_GROUND_METERS, MAX_CLOUD_BASE_ABOVE_GROUND_METERS);
    const float sceneBrightness = static_cast<float>(brightness.value_or(1.0));
    const auto viewRows = diligent_detail::matrixRows(view);
    const CloudParameters values{
        {viewRows[0], viewRows[1], viewRows[2], viewRows[3]},
        {viewRows[4], viewRows[5], viewRows[6], viewRows[7]},
        {viewRows[8], viewRows[9], viewRows[10], viewRows[11]},
        {static_cast<float>(projection[0]), static_cast<float>(projection[5]), fog.density,
         fog.layerHeight},
        {static_cast<float>(std::fmod(originX, cloudPeriodMeters)),
         static_cast<float>(std::fmod(originY, cloudPeriodMeters)),
         static_cast<float>(clampedHeight), static_cast<float>(*type)},
        {sceneBrightness, 0.45F * sceneBrightness, 0.65F * sceneBrightness,
         0.88F * sceneBrightness},
        {fog.cameraHeight, fog.worldUpView[0], fog.worldUpView[1], fog.worldUpView[2]}};

    void* mapped = nullptr;
    context.MapBuffer(impl_->parameters, Diligent::MAP_WRITE, Diligent::MAP_FLAG_DISCARD, mapped);
    require(mapped != nullptr, "Failed to map Diligent weather cloud parameters");
    std::memcpy(mapped, &values, sizeof(values));
    context.UnmapBuffer(impl_->parameters, Diligent::MAP_WRITE);

    context.SetPipelineState(impl_->pipeline);
    context.CommitShaderResources(impl_->bindings,
                                  Diligent::RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    Diligent::DrawAttribs attributes;
    attributes.NumVertices = 3;
    context.Draw(attributes);
    return true;
}

} // namespace openbus::rendering
