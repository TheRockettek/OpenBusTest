#include "render/diligent/DiligentTexturedMesh.h"
#include "render/diligent/DiligentRenderUtils.h"
#include "render/diligent/DiligentTexture2D.h"
#include "render/diligent/DiligentTexture2DArray.h"
#include "render/diligent/DiligentMotionRenderTargets.h"
#include "render/diligent/DiligentSpotlightShadowMaps.h"

#include <Common/interface/RefCntAutoPtr.hpp>
#include <Graphics/GraphicsEngine/interface/Buffer.h>
#include <Graphics/GraphicsEngine/interface/BufferView.h>
#include <Graphics/GraphicsEngine/interface/DeviceContext.h>
#include <Graphics/GraphicsEngine/interface/PipelineState.h>
#include <Graphics/GraphicsEngine/interface/ShaderResourceBinding.h>
#include <Graphics/GraphicsEngine/interface/ShaderResourceVariable.h>
#include <Graphics/GraphicsEngine/interface/Texture.h>
#include <Graphics/GraphicsEngine/interface/TextureView.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iterator>
#include <limits>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

namespace openbus::rendering {
namespace {

constexpr char vertexSource[] = R"(
cbuffer ModelTransform
{
    float4 Row0, Row1, Row2, Row3;
    float4 PreviousRow0, PreviousRow1, PreviousRow2, PreviousRow3;
    float4 ViewRow0, ViewRow1, ViewRow2, ViewRow3;
    float4 NormalRow0, NormalRow1, NormalRow2, NormalRow3;
    float4 MotionViewport;
    float4 MotionParameters;
};
cbuffer HumanBones
{
    float4 CurrentBones[52];
    float4 PreviousBones[52];
};
struct VertexInput
{
    float3 Position : ATTRIB0;
    float3 TexCoord : ATTRIB1;
    float4 Color : ATTRIB2;
    float3 Normal : ATTRIB3;
};
struct VertexOutput
{
    float4 Position : SV_POSITION;
    float3 TexCoord : TEXCOORD0;
    float4 Color : COLOR0;
    float4 PreviousClipPosition : TEXCOORD1;
    nointerpolation float4 Viewport : TEXCOORD2;
    nointerpolation float4 Motion : TEXCOORD3;
    float4 CurrentClipPosition : TEXCOORD4;
    float3 Normal : TEXCOORD5;
    float3 ViewPosition : TEXCOORD6;
};
VertexOutput transformVertex(VertexInput input, float3 currentPosition,
                             float3 previousPosition, float3 currentNormal)
{
    float4 position = float4(currentPosition, 1.0);
    float4 previousPosition4 = float4(previousPosition, 1.0);
    VertexOutput output;
    output.Position = float4(dot(Row0, position), dot(Row1, position),
                             dot(Row2, position), dot(Row3, position));
    output.CurrentClipPosition = output.Position;
    output.PreviousClipPosition =
        float4(dot(PreviousRow0, previousPosition4), dot(PreviousRow1, previousPosition4),
               dot(PreviousRow2, previousPosition4), dot(PreviousRow3, previousPosition4));
    output.ViewPosition =
        float3(dot(ViewRow0, position), dot(ViewRow1, position), dot(ViewRow2, position));
    output.TexCoord = input.TexCoord;
    output.Color = input.Color;
    output.Viewport = MotionViewport;
    output.Motion = MotionParameters;
    float4 normal = float4(currentNormal, 0.0);
    output.Normal = float3(dot(NormalRow0, normal), dot(NormalRow1, normal),
                           dot(NormalRow2, normal));
    return output;
}
VertexOutput main(VertexInput input)
{
    return transformVertex(input, input.Position, input.Position, input.Normal);
}
struct SkinVertexInput
{
    float3 Position : ATTRIB0;
    float3 TexCoord : ATTRIB1;
    float4 Color : ATTRIB2;
    float3 Normal : ATTRIB3;
    float4 Weights0 : ATTRIB4;
    float4 Weights1 : ATTRIB5;
    float4 Weights2 : ATTRIB6;
    float4 Weights3 : ATTRIB7;
};
VertexOutput mainSkinned(SkinVertexInput input)
{
    VertexInput base;
    base.Position = input.Position;
    base.TexCoord = input.TexCoord;
    base.Color = input.Color;
    base.Normal = input.Normal;
    float weights[13] = {
        input.Weights0.x, input.Weights0.y, input.Weights0.z, input.Weights0.w,
        input.Weights1.x, input.Weights1.y, input.Weights1.z, input.Weights1.w,
        input.Weights2.x, input.Weights2.y, input.Weights2.z, input.Weights2.w,
        input.Weights3.x
    };
    float3 current = 0.0, previous = 0.0, normal = 0.0;
    float total = 0.0;
    float4 position = float4(input.Position, 1.0);
    float4 sourceNormal = float4(input.Normal, 0.0);
    for (int i = 0; i < 13; ++i)
    {
        if (weights[i] > 0.0)
        {
            int row = i * 4;
            current += float3(dot(CurrentBones[row], position),
                              dot(CurrentBones[row + 1], position),
                              dot(CurrentBones[row + 2], position)) * weights[i];
            previous += float3(dot(PreviousBones[row], position),
                               dot(PreviousBones[row + 1], position),
                               dot(PreviousBones[row + 2], position)) * weights[i];
            normal += float3(dot(CurrentBones[row], sourceNormal),
                             dot(CurrentBones[row + 1], sourceNormal),
                             dot(CurrentBones[row + 2], sourceNormal)) * weights[i];
            total += weights[i];
        }
    }
    if (total > 0.0)
    {
        current /= total;
        previous /= total;
        normal /= total;
    }
    else
    {
        current = input.Position;
        previous = input.Position;
        normal = input.Normal;
    }
    return transformVertex(base, current, previous, normal);
}
)";

constexpr char shadowVertexSource[] = R"(
cbuffer ShadowTransform
{
    float4 Row0, Row1, Row2, Row3;
};
struct VertexInput
{
    float3 Position : ATTRIB0;
    float3 TexCoord : ATTRIB1;
};
struct VertexOutput
{
    float4 Position : SV_POSITION;
    float2 TexCoord : TEXCOORD0;
    float ArrayLayer : TEXCOORD1;
};
VertexOutput main(VertexInput input)
{
    float4 position = float4(input.Position, 1.0);
    VertexOutput output;
    output.Position = float4(dot(Row0, position), dot(Row1, position),
                             dot(Row2, position), dot(Row3, position));
    output.TexCoord = input.TexCoord.xy;
    output.ArrayLayer = input.TexCoord.z;
    return output;
}
)";

constexpr char shadowSkinnedVertexSource[] = R"(
cbuffer ShadowTransform
{
    float4 Row0, Row1, Row2, Row3;
};
cbuffer HumanBones
{
    float4 CurrentBones[52];
    float4 PreviousBones[52];
};
struct VertexInput
{
    float3 Position : ATTRIB0;
    float3 TexCoord : ATTRIB1;
    float4 Weights0 : ATTRIB4;
    float4 Weights1 : ATTRIB5;
    float4 Weights2 : ATTRIB6;
    float4 Weights3 : ATTRIB7;
};
struct VertexOutput
{
    float4 Position : SV_POSITION;
    float2 TexCoord : TEXCOORD0;
    float ArrayLayer : TEXCOORD1;
};
VertexOutput main(VertexInput input)
{
    float weights[13] = {
        input.Weights0.x, input.Weights0.y, input.Weights0.z, input.Weights0.w,
        input.Weights1.x, input.Weights1.y, input.Weights1.z, input.Weights1.w,
        input.Weights2.x, input.Weights2.y, input.Weights2.z, input.Weights2.w,
        input.Weights3.x
    };
    float4 sourcePosition = float4(input.Position, 1.0);
    float3 skinnedPosition = 0.0;
    float totalWeight = 0.0;
    for (int i = 0; i < 13; ++i)
    {
        if (weights[i] > 0.0)
        {
            int row = i * 4;
            skinnedPosition += float3(dot(CurrentBones[row], sourcePosition),
                                      dot(CurrentBones[row + 1], sourcePosition),
                                      dot(CurrentBones[row + 2], sourcePosition)) * weights[i];
            totalWeight += weights[i];
        }
    }
    if (totalWeight > 0.0)
    {
        skinnedPosition /= totalWeight;
    }
    else
    {
        skinnedPosition = input.Position;
    }
    float4 position = float4(skinnedPosition, 1.0);
    VertexOutput output;
    output.Position = float4(dot(Row0, position), dot(Row1, position),
                             dot(Row2, position), dot(Row3, position));
    output.TexCoord = input.TexCoord.xy;
    output.ArrayLayer = input.TexCoord.z;
    return output;
}
)";

constexpr char shadowPixelSource[] = R"(
Texture2D ShadowAlphaTexture;
SamplerState ShadowAlphaTexture_sampler;
cbuffer ShadowMaterial
{
    float4 TextureTransform;
    float4 AlphaParameters;
};
void main(float4 position : SV_POSITION, float2 texCoord : TEXCOORD0)
{
    if (AlphaParameters.x > 0.5)
    {
        float2 uv = texCoord * TextureTransform.xy + TextureTransform.zw;
        if (ShadowAlphaTexture.Sample(ShadowAlphaTexture_sampler, uv).a < AlphaParameters.y)
        {
            discard;
        }
    }
}
)";

constexpr char shadowArrayPixelSource[] = R"(
Texture2DArray ShadowAlphaTextureArray;
SamplerState ShadowAlphaTextureArray_sampler;
cbuffer ShadowMaterial
{
    float4 TextureTransform;
    float4 AlphaParameters;
};
void main(float4 position : SV_POSITION, float2 texCoord : TEXCOORD0,
          float arrayLayer : TEXCOORD1)
{
    if (AlphaParameters.x > 0.5)
    {
        float2 uv = texCoord * TextureTransform.xy + TextureTransform.zw;
        float selectedLayer =
            AlphaParameters.w > 0.5 ? arrayLayer : AlphaParameters.z;
        if (ShadowAlphaTextureArray.Sample(
                ShadowAlphaTextureArray_sampler, float3(uv, selectedLayer)).a <
            AlphaParameters.y)
        {
            discard;
        }
    }
}
)";

constexpr char pixelSource[] = R"(
Texture2D BaseTexture;
Texture2DArray BaseTextureArray;
Texture2D ArrayReflectionTexture0;
Texture2D ArrayReflectionTexture1;
Texture2D ArrayReflectionTexture2;
Texture2D ArrayReflectionTexture3;
Texture2D ArrayReflectionTexture4;
Texture2D ArrayReflectionTexture5;
Texture2D ArrayReflectionTexture6;
Texture2D ArrayReflectionTexture7;
Texture2DArray<float> SpotlightShadowMaps;
Texture2D NormalTexture;
Texture2D LightmapTexture;
Texture2D NightmapTexture;
Texture2D TransmapTexture;
Texture2D DetailTexture;
Texture2D EnvironmentTexture;
Texture2D FreeTexture;
Texture2D TextTexture;
SamplerState BaseTexture_sampler;
SamplerState BaseTextureArray_sampler;
SamplerState ArrayReflectionTexture0_sampler;
SamplerState ArrayReflectionTexture1_sampler;
SamplerState ArrayReflectionTexture2_sampler;
SamplerState ArrayReflectionTexture3_sampler;
SamplerState ArrayReflectionTexture4_sampler;
SamplerState ArrayReflectionTexture5_sampler;
SamplerState ArrayReflectionTexture6_sampler;
SamplerState ArrayReflectionTexture7_sampler;
SamplerState SpotlightShadowMaps_sampler;
SamplerState NormalTexture_sampler;
SamplerState LightmapTexture_sampler;
SamplerState NightmapTexture_sampler;
SamplerState TransmapTexture_sampler;
SamplerState DetailTexture_sampler;
SamplerState EnvironmentTexture_sampler;
SamplerState FreeTexture_sampler;
SamplerState TextTexture_sampler;
struct PointLightData
{
    float4 PositionRadius;
    float4 ColorIntensity;
};
StructuredBuffer<PointLightData> PointLights;
struct SpotLightData
{
    float4 PositionRange;
    float4 DirectionInner;
    float4 ColorOuter;
    float4 Parameters;
    float4 ShadowRow0;
    float4 ShadowRow1;
    float4 ShadowRow2;
    float4 ShadowRow3;
    uint4 ShadowInfo;
};
StructuredBuffer<SpotLightData> SpotLights;
float spotlightShadowVisibility(SpotLightData spotLight, float3 viewPosition)
{
    if (spotLight.ShadowInfo.x == 0)
    {
        return 1.0;
    }

    float4 position = float4(viewPosition, 1.0);
    float4 shadowClip = float4(dot(spotLight.ShadowRow0, position),
                               dot(spotLight.ShadowRow1, position),
                               dot(spotLight.ShadowRow2, position),
                               dot(spotLight.ShadowRow3, position));
    if (shadowClip.w <= 0.000001)
    {
        return 1.0;
    }
    float3 shadowNdc = shadowClip.xyz / shadowClip.w;
    if (any(abs(shadowNdc.xy) > 1.0) || shadowNdc.z < 0.0 || shadowNdc.z > 1.0)
    {
        return 1.0;
    }

    uint width, height, layers;
    SpotlightShadowMaps.GetDimensions(width, height, layers);
    if (width == 0 || height == 0 || spotLight.ShadowInfo.y >= layers)
    {
        return 1.0;
    }
    float2 uv = float2(shadowNdc.x * 0.5 + 0.5, 0.5 - shadowNdc.y * 0.5);
    float2 texel = 1.0 / float2(width, height);
    float visible = 0.0;
    for (int y = -1; y <= 1; ++y)
    {
        for (int x = -1; x <= 1; ++x)
        {
            float storedDepth = SpotlightShadowMaps.SampleLevel(
                SpotlightShadowMaps_sampler,
                float3(uv + float2(x, y) * texel, spotLight.ShadowInfo.y), 0.0);
            visible += shadowNdc.z - spotLight.Parameters.y <= storedDepth ? 1.0 : 0.0;
        }
    }
    return visible / 9.0;
}
struct PixelInput
{
    float4 Position : SV_POSITION;
    bool FrontFace : SV_IsFrontFace;
    float3 TexCoord : TEXCOORD0;
    float4 Color : COLOR0;
    float4 PreviousClipPosition : TEXCOORD1;
    nointerpolation float4 Viewport : TEXCOORD2;
    nointerpolation float4 Motion : TEXCOORD3;
    float4 CurrentClipPosition : TEXCOORD4;
    float3 Normal : TEXCOORD5;
    float3 ViewPosition : TEXCOORD6;
};
cbuffer ModelMaterial
{
    float4 BaseColor;
    float4 AlphaParameters;
    float4 TextureTransform;
    float4 LightDirectionAmbient;
    float4 LightDiffuseParameters;
    float4 SpecularParameters;
    float4 EmissionParameters;
    float4 NormalMapParameters;
    float4 LightmapParameters;
    float4 NightmapParameters;
    float4 TransmapParameters;
    float4 DetailmapParameters;
    float4 WeatherParameters;
    float4 EnvironmentParameters;
    float4 DynamicTextureParameters;
    float4 FogColorDensity;
    float4 FogLayerCamera;
    float4 FogWorldUp;
    float4 ArrayReflectionLayerMask0;
    float4 ArrayReflectionLayerMask1;
    uint4 PointLightParameters;
    uint4 SpotLightParameters;
    uint4 InteriorLightParameters;
    float4 InteriorLightPositions[4];
    float4 InteriorLightColors[4];
    float4 InteriorLightStrengths;
};
float4 sampleBaseTextureArray(float2 uv, float3 texCoord)
{
    float2 reflectionMaterialUv = texCoord.xy;
    reflectionMaterialUv.y = 1.0 - reflectionMaterialUv.y;
    float2 reflectionUv =
        (reflectionMaterialUv + TextureTransform.zw) * TextureTransform.xy;
    if (ArrayReflectionLayerMask0.x > 0.5 && abs(texCoord.z - 0.0) < 0.25)
    {
        return ArrayReflectionTexture0.Sample(ArrayReflectionTexture0_sampler, reflectionUv);
    }
    if (ArrayReflectionLayerMask0.y > 0.5 && abs(texCoord.z - 1.0) < 0.25)
    {
        return ArrayReflectionTexture1.Sample(ArrayReflectionTexture1_sampler, reflectionUv);
    }
    if (ArrayReflectionLayerMask0.z > 0.5 && abs(texCoord.z - 2.0) < 0.25)
    {
        return ArrayReflectionTexture2.Sample(ArrayReflectionTexture2_sampler, reflectionUv);
    }
    if (ArrayReflectionLayerMask0.w > 0.5 && abs(texCoord.z - 3.0) < 0.25)
    {
        return ArrayReflectionTexture3.Sample(ArrayReflectionTexture3_sampler, reflectionUv);
    }
    if (ArrayReflectionLayerMask1.x > 0.5 && abs(texCoord.z - 4.0) < 0.25)
    {
        return ArrayReflectionTexture4.Sample(ArrayReflectionTexture4_sampler, reflectionUv);
    }
    if (ArrayReflectionLayerMask1.y > 0.5 && abs(texCoord.z - 5.0) < 0.25)
    {
        return ArrayReflectionTexture5.Sample(ArrayReflectionTexture5_sampler, reflectionUv);
    }
    if (ArrayReflectionLayerMask1.z > 0.5 && abs(texCoord.z - 6.0) < 0.25)
    {
        return ArrayReflectionTexture6.Sample(ArrayReflectionTexture6_sampler, reflectionUv);
    }
    if (ArrayReflectionLayerMask1.w > 0.5 && abs(texCoord.z - 7.0) < 0.25)
    {
        return ArrayReflectionTexture7.Sample(ArrayReflectionTexture7_sampler, reflectionUv);
    }
    return BaseTextureArray.Sample(BaseTextureArray_sampler, float3(uv, texCoord.z));
}
float4 shade(PixelInput input)
{
    if (WeatherParameters.z > 0.5 && !input.FrontFace)
    {
        discard;
    }
    float2 materialUv = input.TexCoord.xy;
    if (AlphaParameters.w > 0.5)
    {
        materialUv.y = 1.0 - materialUv.y;
    }
    materialUv += TextureTransform.zw;
    float2 uv = materialUv * TextureTransform.xy;
    float4 sampled;
    if (AlphaParameters.z > 0.5)
    {
        sampled = sampleBaseTextureArray(uv, input.TexCoord);
    }
    else
    {
        sampled = BaseTexture.Sample(BaseTexture_sampler, uv);
    }
    if (NormalMapParameters.w > 0.5)
    {
        sampled.rgb = float3(1.0, 1.0, 1.0);
    }
    float4 vertexColor = NormalMapParameters.z > 0.5 ? input.Color : float4(1.0, 1.0, 1.0, 1.0);
    float3 ambientReflectance = sampled.rgb * vertexColor.rgb;
    float4 surface = sampled * vertexColor * BaseColor;
    bool useFreeTexture = DynamicTextureParameters.x > 0.5;
    bool useTextTexture = DynamicTextureParameters.y > 0.5;
    float4 dynamicTextureSample = float4(1.0, 1.0, 1.0, 1.0);
    if (useTextTexture)
    {
        dynamicTextureSample = TextTexture.Sample(TextTexture_sampler, input.TexCoord.xy);
    }
    else if (useFreeTexture)
    {
        dynamicTextureSample = FreeTexture.Sample(FreeTexture_sampler, uv);
    }
    surface *= dynamicTextureSample;
    ambientReflectance *= dynamicTextureSample.rgb;
    bool dynamicTexture = useFreeTexture || useTextTexture;
    if (useTextTexture && surface.a <= 0.01)
    {
        discard;
    }
    if (DetailmapParameters.x > 0.5)
    {
        float2 detailUv = materialUv * DetailmapParameters.yz;
        float3 detail = DetailTexture.Sample(DetailTexture_sampler, detailUv).rgb;
        surface.rgb *= detail;
        ambientReflectance *= detail;
    }
    surface.rgb = lerp(surface.rgb, float3(0.90, 0.92, 0.94), saturate(WeatherParameters.x));
    ambientReflectance = lerp(ambientReflectance, float3(0.90, 0.92, 0.94),
                              saturate(WeatherParameters.x));
    if (TransmapParameters.x > 0.5 && !useTextTexture)
    {
        float2 transmapUv = materialUv * TransmapParameters.yz;
        if (TransmapParameters.w > 0.5)
        {
            transmapUv.y = 1.0 - transmapUv.y;
        }
        float transmapAlpha = TransmapTexture.Sample(TransmapTexture_sampler, transmapUv).a;
        surface.a = vertexColor.a * BaseColor.a * transmapAlpha;
    }
    // Keep normal-map derivatives outside fragment discard divergence.
    if (AlphaParameters.x > 0.0 && NormalMapParameters.x < 0.5)
    {
        clip(surface.a * AlphaParameters.y - AlphaParameters.x);
    }
    float normalLength = length(input.Normal);
    float3 surfaceNormal =
        normalLength > 0.000001 ? input.Normal / normalLength : float3(0.0, 0.0, 0.0);
    if (NormalMapParameters.x > 0.5)
    {
        float3 positionDerivativeX = ddx(input.ViewPosition);
        float3 positionDerivativeY = ddy(input.ViewPosition);
        float2 coordinateDerivativeX = ddx(materialUv);
        float2 coordinateDerivativeY = ddy(materialUv);
        float determinant = coordinateDerivativeX.x * coordinateDerivativeY.y -
                            coordinateDerivativeX.y * coordinateDerivativeY.x;
        if (abs(determinant) > 0.00001)
        {
            float3 tangent = (positionDerivativeX * coordinateDerivativeY.y -
                              positionDerivativeY * coordinateDerivativeX.y) / determinant;
            float3 bitangent = (-positionDerivativeX * coordinateDerivativeY.x +
                                positionDerivativeY * coordinateDerivativeX.x) / determinant;
            float tangentLength = length(tangent);
            float bitangentLength = length(bitangent);
            if (tangentLength > 0.00001 && bitangentLength > 0.00001)
            {
                tangent /= tangentLength;
                bitangent /= bitangentLength;
                float3 mappedNormal = NormalTexture.Sample(NormalTexture_sampler, materialUv).xyz *
                                          2.0 -
                                      1.0;
                mappedNormal.xy *= saturate(NormalMapParameters.y);
                float3 perturbedNormal =
                    tangent * mappedNormal.x + bitangent * mappedNormal.y +
                    surfaceNormal * mappedNormal.z;
                float perturbedLength = length(perturbedNormal);
                if (perturbedLength > 0.00001)
                {
                    surfaceNormal = perturbedNormal / perturbedLength;
                }
            }
        }
    }
    if (AlphaParameters.x > 0.0 && NormalMapParameters.x > 0.5)
    {
        clip(surface.a * AlphaParameters.y - AlphaParameters.x);
    }
    float diffuse = max(dot(surfaceNormal, normalize(LightDirectionAmbient.xyz)), 0.0);
    float3 lighting = LightDirectionAmbient.w.xxx +
                      LightDiffuseParameters.rgb * (LightDiffuseParameters.w * diffuse);
    float3 diffuseReflectance = surface.rgb;
    float3 directionalColor =
        dynamicTexture
            ? diffuseReflectance *
                  (DynamicTextureParameters.z > 0.5 ? DynamicTextureParameters.w : 1.0)
            : diffuseReflectance * clamp(lighting, 0.0, 1.0);
    if (WeatherParameters.y > 0.5 && PointLightParameters.y != 0)
    {
        directionalColor = saturate(ambientReflectance * LightDirectionAmbient.w +
                                    diffuseReflectance * LightDiffuseParameters.rgb *
                                        (LightDiffuseParameters.w * diffuse));
    }
    float3 lightmapRestore = float3(0.0, 0.0, 0.0);
    if (LightmapParameters.x > 0.5 && !useTextTexture)
    {
        lightmapRestore = saturate(
            LightmapTexture.Sample(LightmapTexture_sampler, materialUv).rgb *
            LightmapParameters.y);
    }
    float3 emissionRestore = saturate(EmissionParameters.rgb);
    float3 restoreColor = emissionRestore + lightmapRestore * (1.0 - emissionRestore);
    surface.rgb = lerp(directionalColor, diffuseReflectance, restoreColor);
    if (!useTextTexture && PointLightParameters.y != 0)
    {
        float viewLength = length(input.ViewPosition);
        float3 viewDirection = viewLength > 0.000001
                                   ? -input.ViewPosition / viewLength
                                   : float3(0.0, 0.0, 1.0);
        float geometricNormalLength = length(input.Normal);
        float3 geometricNormal = geometricNormalLength > 0.000001
                                     ? input.Normal / geometricNormalLength
                                     : float3(0.0, 0.0, 1.0);
        float3 lightDirection = normalize(LightDirectionAmbient.xyz);
        float geometricDiffuse = max(dot(geometricNormal, lightDirection), 0.0);
        float3 halfVector = lightDirection + viewDirection;
        float halfLength = length(halfVector);
        if (geometricDiffuse > 0.0 && halfLength > 0.000001)
        {
            float specular = pow(max(dot(geometricNormal, halfVector / halfLength), 0.0),
                                 max(SpecularParameters.w, 1.0));
            surface.rgb += SpecularParameters.rgb * specular * WeatherParameters.w;
        }
    }
    float3 localLightReflectance = diffuseReflectance;
    float3 localLightContribution = float3(0.0, 0.0, 0.0);
    bool dynamicLightingEnabled = !useTextTexture || DynamicTextureParameters.z > 0.5;
    for (uint index = 0; dynamicLightingEnabled && index < PointLightParameters.x; ++index)
    {
        PointLightData pointLight = PointLights[index];
        float3 lightVector = pointLight.PositionRadius.xyz - input.ViewPosition;
        float distanceSquared = dot(lightVector, lightVector);
        if (distanceSquared > 0.00000001)
        {
            float distance = sqrt(distanceSquared);
            float3 pointLightDirection = lightVector / distance;
            float pointDiffuse = max(dot(surfaceNormal, pointLightDirection), 0.0);
            float relativeDistance = distance / pointLight.PositionRadius.w;
            float falloff = relativeDistance <= 1.0
                                ? 1.0
                                : 1.0 / (relativeDistance * relativeDistance);
            float illumination = falloff * (0.12 + 0.88 * pointDiffuse) *
                                 pointLight.ColorIntensity.w;
            localLightContribution +=
                localLightReflectance * pointLight.ColorIntensity.rgb * illumination;
        }
    }
    for (uint index = 0; dynamicLightingEnabled && index < SpotLightParameters.x; ++index)
    {
        SpotLightData spotLight = SpotLights[index];
        float3 lightToSurface = input.ViewPosition - spotLight.PositionRange.xyz;
        float distanceSquared = dot(lightToSurface, lightToSurface);
        if (distanceSquared > 0.00000001)
        {
            float distance = sqrt(distanceSquared);
            float relativeDistance = distance / spotLight.PositionRange.w;
            if (relativeDistance < 4.0)
            {
                float3 ray = lightToSurface / distance;
                float angle = dot(ray, spotLight.DirectionInner.xyz);
                float cone = spotLight.DirectionInner.w > spotLight.ColorOuter.w
                                 ? saturate((angle - spotLight.ColorOuter.w) /
                                            (spotLight.DirectionInner.w - spotLight.ColorOuter.w))
                                 : (angle >= spotLight.DirectionInner.w ? 1.0 : 0.0);
                float attenuation =
                    1.0 / (0.2 + relativeDistance + 40.0 * relativeDistance * relativeDistance);
                float spotDiffuse = max(dot(surfaceNormal, -ray), 0.0);
                float illumination = cone * cone * attenuation *
                                     (0.1 + 0.3 * spotDiffuse) *
                                     spotLight.Parameters.x *
                                     spotlightShadowVisibility(spotLight, input.ViewPosition);
                localLightContribution +=
                    localLightReflectance * spotLight.ColorOuter.rgb * illumination;
            }
        }
    }
    if (NightmapParameters.x > 0.5 && !useTextTexture)
    {
        float3 nightmap =
            NightmapTexture.Sample(NightmapTexture_sampler, materialUv).rgb;
        float coverage = max(max(nightmap.r, nightmap.g), nightmap.b);
        float3 nightColor = nightmap * BaseColor.rgb * vertexColor.rgb;
        surface.rgb = lerp(surface.rgb, nightColor,
                           saturate(NightmapParameters.y * coverage));
    }
    if (InteriorLightParameters.x != 0 && !useTextTexture)
    {
        float geometricNormalLength = length(input.Normal);
        float3 geometricNormal = geometricNormalLength > 0.000001
                                     ? input.Normal / geometricNormalLength
                                     : float3(0.0, 0.0, 1.0);
        for (uint index = 0; index < InteriorLightParameters.x; ++index)
        {
            float3 lightVector = InteriorLightPositions[index].xyz - input.ViewPosition;
            float distanceSquared = dot(lightVector, lightVector);
            if (distanceSquared > 0.00000001)
            {
                float distance = sqrt(distanceSquared);
                float3 lightDirection = lightVector / distance;
                float diffuse = max(dot(geometricNormal, lightDirection), 0.0);
                float attenuation = 1.0 / (1.0 + 0.12 * distanceSquared);
                float illumination =
                    attenuation * diffuse * max(InteriorLightStrengths[index], 0.0) * 0.25;
                surface.rgb += diffuseReflectance * InteriorLightColors[index].rgb * illumination;
            }
        }
        surface.rgb = saturate(surface.rgb);
    }
    surface.rgb += localLightContribution * (1.0 - lightmapRestore);
    if (EnvironmentParameters.x > 0.5 && !useTextTexture)
    {
        float viewLength = length(input.ViewPosition);
        float3 viewDirection = viewLength > 0.000001
                                   ? -input.ViewPosition / viewLength
                                   : float3(0.0, 0.0, 1.0);
        float geometricNormalLength = length(input.Normal);
        float3 geometricNormal = geometricNormalLength > 0.000001
                                     ? input.Normal / geometricNormalLength
                                     : float3(0.0, 0.0, 1.0);
        float3 reflection = reflect(-viewDirection, geometricNormal);
        float denominator = 2.0 * sqrt(max(reflection.z + 1.0, 0.0001));
        float2 coordinates = reflection.xy / denominator + float2(0.5, 0.5);
        float3 environment = EnvironmentTexture.Sample(EnvironmentTexture_sampler, coordinates).rgb *
                             EnvironmentParameters.z;
        surface.rgb = lerp(surface.rgb, environment, saturate(EnvironmentParameters.y));
    }
    if (FogColorDensity.w > 0.0)
    {
        float fraction = 1.0;
        if (FogLayerCamera.x >= 0.0)
        {
            float deltaHeight = dot(float3(FogLayerCamera.zw, FogWorldUp.x),
                                    input.ViewPosition);
            float start = FogLayerCamera.y - FogLayerCamera.x;
            float end = start + deltaHeight;
            if (start > 0.0 && end > 0.0)
            {
                fraction = 0.0;
            }
            else if ((start > 0.0) != (end > 0.0))
            {
                float crossing = clamp(-start / deltaHeight, 0.0, 1.0);
                fraction = start <= 0.0 ? crossing : 1.0 - crossing;
            }
        }
        float transmission =
            exp(-FogColorDensity.w * length(input.ViewPosition) * fraction);
        surface.rgb = lerp(FogColorDensity.rgb, surface.rgb, transmission);
    }
    surface.a *= saturate(AlphaParameters.y);
    return surface;
}
float4 main(PixelInput input) : SV_TARGET0
{
    return shade(input);
}
struct DepthPixelOutput
{
    float4 Color : SV_TARGET0;
    float LinearDepth : SV_TARGET1;
};
DepthPixelOutput mainDepth(PixelInput input)
{
    DepthPixelOutput output;
    output.Color = shade(input);
    output.LinearDepth = max(-input.ViewPosition.z, 0.0);
    return output;
}
struct MotionPixelOutput
{
    float4 Color : SV_TARGET0;
    float2 Motion : SV_TARGET1;
    float Validity : SV_TARGET2;
};
void computeMotion(PixelInput input, out float2 motion, out float validity)
{
    bool previousPositionValid =
        input.Motion.x > 0.5 && input.PreviousClipPosition.w > 0.000001 &&
        input.CurrentClipPosition.w > 0.000001;
    float2 previousNdc =
        previousPositionValid
            ? input.PreviousClipPosition.xy / input.PreviousClipPosition.w
            : float2(0.0, 0.0);
    float2 currentNdc = previousPositionValid
                            ? input.CurrentClipPosition.xy / input.CurrentClipPosition.w
                            : float2(0.0, 0.0);
    motion = previousPositionValid
                 ? (previousNdc - currentNdc) * (input.Viewport.zw * 0.5)
                 : float2(0.0, 0.0);
    validity = previousPositionValid ? 1.0 : 0.0;
}
MotionPixelOutput mainMotion(PixelInput input)
{
    MotionPixelOutput output;
    output.Color = shade(input);
    computeMotion(input, output.Motion, output.Validity);
    return output;
}
struct MotionDepthPixelOutput
{
    float4 Color : SV_TARGET0;
    float2 Motion : SV_TARGET1;
    float Validity : SV_TARGET2;
    float LinearDepth : SV_TARGET3;
};
MotionDepthPixelOutput mainMotionDepth(PixelInput input)
{
    MotionDepthPixelOutput output;
    output.Color = shade(input);
    computeMotion(input, output.Motion, output.Validity);
    output.LinearDepth = max(-input.ViewPosition.z, 0.0);
    return output;
}
)";

float alphaCutoff(DiligentAlphaMode mode) {
    switch (mode) {
    case DiligentAlphaMode::Opaque:
    case DiligentAlphaMode::Blend:
        return 0.0F;
    case DiligentAlphaMode::Cutout:
        return 0.5F;
    case DiligentAlphaMode::NearOpaque:
    case DiligentAlphaMode::NearOpaqueBlend:
        return 0.99F;
    }
    throw std::invalid_argument("Unsupported Diligent material alpha mode");
}

void updateDynamicBuffer(Diligent::IDeviceContext& context, Diligent::IBuffer& buffer,
                         const void* data, Diligent::Uint32 size, const char* errorMessage) {
    void* mapped = nullptr;
    context.MapBuffer(&buffer, Diligent::MAP_WRITE, Diligent::MAP_FLAG_DISCARD, mapped);
    if (mapped == nullptr) {
        throw std::runtime_error(errorMessage);
    }
    std::memcpy(mapped, data, size);
    context.UnmapBuffer(&buffer, Diligent::MAP_WRITE);
}

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void createFallbackTexture(Diligent::IRenderDevice& device, bool textureArray,
                           Diligent::RefCntAutoPtr<Diligent::ITexture>& texture,
                           Diligent::RefCntAutoPtr<Diligent::ITextureView>& view,
                           std::array<std::uint8_t, 4> pixel = {255, 255, 255, 255}) {
    using namespace Diligent;
    std::array<TextureSubResData, 2> subresources{};
    const Uint32 layerCount = textureArray ? 2 : 1;
    for (Uint32 layer = 0; layer < layerCount; ++layer) {
        subresources[layer].pData = pixel.data();
        subresources[layer].Stride = 4;
    }
    TextureDesc desc;
    desc.Name =
        textureArray ? "OpenBus fallback white texture array" : "OpenBus fallback white texture";
    desc.Type = textureArray ? RESOURCE_DIM_TEX_2D_ARRAY : RESOURCE_DIM_TEX_2D;
    desc.Width = 1;
    desc.Height = 1;
    desc.ArraySize = layerCount;
    desc.MipLevels = 1;
    desc.Format = TEX_FORMAT_RGBA8_UNORM;
    desc.Usage = USAGE_IMMUTABLE;
    desc.BindFlags = BIND_SHADER_RESOURCE;
    TextureData data;
    data.pSubResources = subresources.data();
    data.NumSubresources = layerCount;
    device.CreateTexture(desc, &data, &texture);
    require(static_cast<bool>(texture), "Failed to create fallback Diligent texture");
    view = texture->GetDefaultView(TEXTURE_VIEW_SHADER_RESOURCE);
    require(static_cast<bool>(view), "Failed to create fallback Diligent texture view");
}

Matrix4 multiplyMatrices(const Matrix4& left, const Matrix4& right) {
    Matrix4 result = {};
    for (std::size_t column = 0; column < 4; ++column) {
        for (std::size_t row = 0; row < 4; ++row) {
            for (std::size_t inner = 0; inner < 4; ++inner) {
                result[row + column * 4] += left[row + inner * 4] * right[inner + column * 4];
            }
        }
    }
    return result;
}

struct alignas(16) DiligentGpuPointLight {
    std::array<float, 4> positionRadius = {};
    std::array<float, 4> colorIntensity = {};
};
static_assert(sizeof(DiligentGpuPointLight) == 8 * sizeof(float));

struct alignas(16) DiligentGpuSpotLight {
    std::array<float, 4> positionRange = {};
    std::array<float, 4> directionInner = {};
    std::array<float, 4> colorOuter = {};
    std::array<float, 4> parameters = {};
    std::array<float, 4> shadowRow0 = {};
    std::array<float, 4> shadowRow1 = {};
    std::array<float, 4> shadowRow2 = {};
    std::array<float, 4> shadowRow3 = {};
    std::array<std::uint32_t, 4> shadowInfo = {};
};
static_assert(sizeof(DiligentGpuSpotLight) == 36 * sizeof(float));

template <typename Element> class DiligentDynamicStructuredBuffer {
  public:
    explicit DiligentDynamicStructuredBuffer(const char* name) : name_(name) {}

    void ensureCapacity(Diligent::IRenderDevice& device, std::size_t requiredCapacity) {
        using namespace Diligent;
        if (requiredCapacity <= capacity_) {
            return;
        }
        constexpr std::size_t maxCapacity = std::numeric_limits<Uint32>::max() / sizeof(Element);
        if (requiredCapacity > maxCapacity) {
            throw std::length_error("Diligent structured buffer exceeds supported size");
        }
        Uint32 newCapacity = std::max<Uint32>(capacity_, 1);
        while (newCapacity < requiredCapacity) {
            if (newCapacity > maxCapacity / 2) {
                newCapacity = static_cast<Uint32>(requiredCapacity);
                break;
            }
            newCapacity *= 2;
        }

        BufferDesc desc;
        desc.Name = name_;
        desc.Size = static_cast<Uint32>(newCapacity * sizeof(Element));
        desc.Usage = USAGE_DYNAMIC;
        desc.BindFlags = BIND_SHADER_RESOURCE;
        desc.CPUAccessFlags = CPU_ACCESS_WRITE;
        desc.Mode = BUFFER_MODE_STRUCTURED;
        desc.ElementByteStride = static_cast<Uint32>(sizeof(Element));
        RefCntAutoPtr<IBuffer> newBuffer;
        device.CreateBuffer(desc, nullptr, &newBuffer);
        require(static_cast<bool>(newBuffer), "Failed to create Diligent structured buffer");
        RefCntAutoPtr<IBufferView> newView;
        newView = newBuffer->GetDefaultView(BUFFER_VIEW_SHADER_RESOURCE);
        require(static_cast<bool>(newView), "Failed to create Diligent structured buffer view");
        buffer_ = std::move(newBuffer);
        view_ = std::move(newView);
        capacity_ = newCapacity;
    }

    void upload(Diligent::IDeviceContext& context, std::span<const Element> elements,
                const char* errorMessage) {
        if (elements.empty()) {
            return;
        }
        if (elements.size() > std::numeric_limits<Diligent::Uint32>::max() / sizeof(Element)) {
            throw std::length_error("Diligent structured-buffer upload exceeds supported size");
        }
        const auto uploadSize = static_cast<Diligent::Uint32>(elements.size() * sizeof(Element));
        updateDynamicBuffer(context, *buffer_, elements.data(), uploadSize, errorMessage);
    }

    [[nodiscard]] Diligent::IBufferView* view() const noexcept {
        return view_.RawPtr();
    }

  private:
    const char* name_;
    Diligent::RefCntAutoPtr<Diligent::IBuffer> buffer_;
    Diligent::RefCntAutoPtr<Diligent::IBufferView> view_;
    Diligent::Uint32 capacity_ = 0;
};

struct alignas(16) DiligentMaterialParameters {
    std::array<float, 80> material = {};
    std::array<std::uint32_t, 4> pointLights = {};
    std::array<std::uint32_t, 4> spotLights = {};
    std::array<std::uint32_t, 4> interiorLights = {};
    std::array<std::array<float, 4>, DILIGENT_MAX_INTERIOR_LIGHTS> interiorLightPositions = {};
    std::array<std::array<float, 4>, DILIGENT_MAX_INTERIOR_LIGHTS> interiorLightColors = {};
    std::array<float, 4> interiorLightStrengths = {};
};
static_assert(sizeof(DiligentMaterialParameters) == 128 * sizeof(float));

// Keep this field order aligned with the ModelTransform HLSL constant buffer.
struct alignas(16) DiligentModelTransform {
    std::array<float, 16> currentRows = {};
    std::array<float, 16> previousRows = {};
    std::array<float, 16> viewRows = {};
    std::array<float, 16> normalRows = {};
    std::array<float, 4> viewport = {};
    std::array<float, 4> motion = {};
};
static_assert(sizeof(DiligentModelTransform) == 72 * sizeof(float));

struct alignas(16) DiligentShadowTransform {
    std::array<float, 16> rows = {};
};
static_assert(sizeof(DiligentShadowTransform) == 16 * sizeof(float));

struct alignas(16) DiligentShadowMaterial {
    std::array<float, 4> textureTransform = {};
    std::array<float, 4> alphaParameters = {};
};
static_assert(sizeof(DiligentShadowMaterial) == 8 * sizeof(float));

struct alignas(16) DiligentHumanBones {
    std::array<float, HUMAN_BONE_COUNT * 16> current = {};
    std::array<float, HUMAN_BONE_COUNT * 16> previous = {};
};

} // namespace

DiligentMotionParameters DiligentMotionParameters::fromHistory(const MotionDataFrame& frame,
                                                               const MotionTransform& transform) {
    if (frame.width <= 0 || frame.height <= 0) {
        throw std::invalid_argument("Diligent motion history requires positive frame dimensions");
    }
    DiligentMotionParameters parameters;
    parameters.previousModelViewProjection =
        multiplyMatrices(frame.previousProjection, transform.previousModelView);
    parameters.viewportWidth = frame.width;
    parameters.viewportHeight = frame.height;
    parameters.historyValid = frame.historyValid && transform.valid;
    return parameters;
}

struct DiligentTexturedMesh::Impl {
    Diligent::RefCntAutoPtr<Diligent::IRenderDevice> device;
    Diligent::RefCntAutoPtr<Diligent::IBuffer> vertices;
    Diligent::RefCntAutoPtr<Diligent::IBuffer> indices;
    Diligent::RefCntAutoPtr<Diligent::IBuffer> skinWeights;
    Diligent::Uint32 count = 0;
    float maximumArrayLayer = 0.0F;
    bool hasValidArrayLayers = true;
    bool skinned = false;

    Impl(Diligent::IRenderDevice& renderDevice, std::span<const ModelVertex> source,
         std::span<const std::uint32_t> sourceIndices, bool indexed,
         std::span<const HumanVertexWeights> weights = {}, bool skinned = false)
        : device(&renderDevice), skinned(skinned) {
        using namespace Diligent;
        if (source.empty() || (!indexed && source.size() % 3 != 0)) {
            throw std::invalid_argument("Diligent mesh requires a nonempty triangle list");
        }
        if (indexed && (sourceIndices.empty() || sourceIndices.size() % 3 != 0)) {
            throw std::invalid_argument(
                "Diligent indexed mesh requires a nonempty triangle index list");
        }
        if (source.size() > std::numeric_limits<Uint32>::max() / sizeof(ModelVertex)) {
            throw std::length_error("Diligent mesh vertex buffer exceeds supported size");
        }
        for (const ModelVertex& vertex : source) {
            const float attributes[] = {vertex.x,     vertex.y,  vertex.z,  vertex.u,  vertex.v,
                                        vertex.layer, vertex.nx, vertex.ny, vertex.nz, vertex.r,
                                        vertex.g,     vertex.b,  vertex.a};
            if (!std::all_of(std::begin(attributes), std::end(attributes),
                             [](float value) { return std::isfinite(value); })) {
                throw std::invalid_argument("Diligent mesh contains non-finite vertex data");
            }
            if (vertex.layer < 0.0F || std::floor(vertex.layer) != vertex.layer) {
                hasValidArrayLayers = false;
            }
            maximumArrayLayer = std::max(maximumArrayLayer, vertex.layer);
        }
        if (sourceIndices.size() > std::numeric_limits<Uint32>::max() / sizeof(std::uint32_t)) {
            throw std::length_error("Diligent mesh index buffer exceeds supported size");
        }
        if (indexed && std::any_of(sourceIndices.begin(), sourceIndices.end(),
                                   [&](std::uint32_t index) { return index >= source.size(); })) {
            throw std::invalid_argument("Diligent mesh index is outside the vertex buffer");
        }
        if (skinned &&
            (weights.size() != source.size() ||
             weights.size() > std::numeric_limits<Uint32>::max() / sizeof(HumanVertexWeights))) {
            throw std::invalid_argument("Diligent skin weights must match the vertex count");
        }
        if (skinned) {
            for (const auto& vertexWeights : weights) {
                if (!std::all_of(vertexWeights.begin(), vertexWeights.end(), [](float weight) {
                        return std::isfinite(weight) && weight >= 0;
                    })) {
                    throw std::invalid_argument(
                        "Diligent skin weights must be finite and nonnegative");
                }
            }
        }
        BufferDesc info;
        info.Name = "OpenBus immutable model vertices";
        info.Size = source.size_bytes();
        info.Usage = USAGE_IMMUTABLE;
        info.BindFlags = BIND_VERTEX_BUFFER;
        BufferData data;
        data.pData = source.data();
        data.DataSize = source.size_bytes();
        renderDevice.CreateBuffer(info, &data, &vertices);
        require(static_cast<bool>(vertices), "Failed to create Diligent mesh buffer");
        if (indexed) {
            info.Name = "OpenBus immutable model indices";
            info.Size = sourceIndices.size_bytes();
            info.BindFlags = BIND_INDEX_BUFFER;
            data.pData = sourceIndices.data();
            data.DataSize = sourceIndices.size_bytes();
            renderDevice.CreateBuffer(info, &data, &indices);
            require(static_cast<bool>(indices), "Failed to create Diligent mesh index buffer");
        }
        if (skinned) {
            info.Name = "OpenBus human skin weights";
            info.Size = weights.size_bytes();
            info.BindFlags = BIND_VERTEX_BUFFER;
            data.pData = weights.data();
            data.DataSize = weights.size_bytes();
            renderDevice.CreateBuffer(info, &data, &skinWeights);
            require(static_cast<bool>(skinWeights), "Failed to create Diligent skin-weight buffer");
        }
        count = static_cast<Uint32>(indexed ? sourceIndices.size() : source.size());
    }
};

struct DiligentTexturedMeshRenderer::Impl {
    static constexpr std::size_t addressModeCount = 5;
    static constexpr std::array<Diligent::Uint32, 5> anisotropyLevels = {1, 2, 4, 8, 16};
    struct Pipeline {
        Diligent::RefCntAutoPtr<Diligent::IPipelineState> state;
        Diligent::RefCntAutoPtr<Diligent::IShaderResourceBinding> binding;
    };

    Diligent::RefCntAutoPtr<Diligent::IRenderDevice> device;
    std::unordered_map<std::size_t, Pipeline> pipelines;
    std::unordered_map<std::size_t, Pipeline> motionPipelines;
    Diligent::RefCntAutoPtr<Diligent::IShader> vertexShader;
    Diligent::RefCntAutoPtr<Diligent::IShader> skinnedVertexShader;
    Diligent::RefCntAutoPtr<Diligent::IShader> pixelShader;
    Diligent::RefCntAutoPtr<Diligent::IShader> motionPixelShader;
    Diligent::RefCntAutoPtr<Diligent::IShader> shadowVertexShader;
    Diligent::RefCntAutoPtr<Diligent::IShader> shadowSkinnedVertexShader;
    Diligent::RefCntAutoPtr<Diligent::IShader> shadowPixelShader;
    Diligent::RefCntAutoPtr<Diligent::IShader> shadowArrayPixelShader;
    Pipeline shadowPipeline;
    Pipeline shadowArrayPipeline;
    Pipeline shadowSkinnedPipeline;
    Pipeline shadowArraySkinnedPipeline;
    Pipeline shadowCutoutPipeline;
    Pipeline shadowArrayCutoutPipeline;
    Pipeline shadowSkinnedCutoutPipeline;
    Pipeline shadowArraySkinnedCutoutPipeline;
    Diligent::TEXTURE_FORMAT colorFormat;
    Diligent::TEXTURE_FORMAT depthFormat;
    Diligent::TEXTURE_FORMAT motionVectorFormat;
    Diligent::TEXTURE_FORMAT motionValidityFormat;
    Diligent::TEXTURE_FORMAT linearDepthFormat;
    Diligent::Uint32 sampleCount;
    Diligent::RefCntAutoPtr<Diligent::IBuffer> transform;
    Diligent::RefCntAutoPtr<Diligent::IBuffer> material;
    Diligent::RefCntAutoPtr<Diligent::IBuffer> bones;
    Diligent::RefCntAutoPtr<Diligent::IBuffer> shadowTransform;
    Diligent::RefCntAutoPtr<Diligent::IBuffer> shadowMaterial;
    DiligentDynamicStructuredBuffer<DiligentGpuPointLight> pointLightBuffer{
        "OpenBus dynamic view-space point lights"};
    DiligentDynamicStructuredBuffer<DiligentGpuSpotLight> spotLightBuffer{
        "OpenBus dynamic view-space spot lights"};
    Diligent::Uint32 pointLightCount = 0;
    Diligent::Uint32 spotLightCount = 0;
    Diligent::RefCntAutoPtr<Diligent::ITexture> fallback2DTexture;
    Diligent::RefCntAutoPtr<Diligent::ITextureView> fallback2DView;
    Diligent::RefCntAutoPtr<Diligent::ITexture> fallbackArrayTexture;
    Diligent::RefCntAutoPtr<Diligent::ITextureView> fallbackArrayView;
    Diligent::ITextureView* spotlightShadowMapView = nullptr;
    Diligent::RefCntAutoPtr<Diligent::ITexture> fallbackNormalTexture;
    Diligent::RefCntAutoPtr<Diligent::ITextureView> fallbackNormalView;
    Diligent::RefCntAutoPtr<Diligent::ITexture> fallbackNightmapTexture;
    Diligent::RefCntAutoPtr<Diligent::ITextureView> fallbackNightmapView;
    float minimumNdcZ = 0.0f;
    Diligent::Uint32 maximumAnisotropy = 1;
    Diligent::Uint32 defaultAnisotropy = 1;
    bool hasDefaultAnisotropy = false;
    bool motionEnabled = false;
    bool linearDepthEnabled = false;

    Impl(Diligent::IRenderDevice& renderDevice, Diligent::TEXTURE_FORMAT colorFormat,
         Diligent::TEXTURE_FORMAT depthFormat, Diligent::TEXTURE_FORMAT motionVectorFormat,
         Diligent::TEXTURE_FORMAT motionValidityFormat, Diligent::TEXTURE_FORMAT linearDepthFormat,
         Diligent::Uint32 samples = 1)
        : device(&renderDevice), colorFormat(colorFormat), depthFormat(depthFormat),
          motionVectorFormat(motionVectorFormat), motionValidityFormat(motionValidityFormat),
          linearDepthFormat(linearDepthFormat), sampleCount(samples),
          minimumNdcZ(renderDevice.GetDeviceInfo().NDC.MinZ) {
        using namespace Diligent;
        const Uint32 deviceMaximumAnisotropy =
            std::max<Uint32>(1, renderDevice.GetAdapterInfo().Sampler.MaxAnisotropy);
        for (const Uint32 level : anisotropyLevels) {
            if (level <= deviceMaximumAnisotropy) {
                maximumAnisotropy = level;
            }
        }
        if (colorFormat == TEX_FORMAT_UNKNOWN) {
            throw std::invalid_argument("Diligent mesh color format must be specified");
        }
        diligent_detail::validateSamples(renderDevice, colorFormat, depthFormat, sampleCount);
        if ((motionVectorFormat == TEX_FORMAT_UNKNOWN) !=
            (motionValidityFormat == TEX_FORMAT_UNKNOWN)) {
            throw std::invalid_argument(
                "Diligent motion vectors and validity formats must be specified together");
        }
        motionEnabled = motionVectorFormat != TEX_FORMAT_UNKNOWN;
        linearDepthEnabled = linearDepthFormat != TEX_FORMAT_UNKNOWN;
        if (motionEnabled) {
            diligent_detail::validateSamples(renderDevice, motionVectorFormat, TEX_FORMAT_UNKNOWN,
                                             sampleCount);
            diligent_detail::validateSamples(renderDevice, motionValidityFormat, TEX_FORMAT_UNKNOWN,
                                             sampleCount);
        }
        if (linearDepthEnabled) {
            diligent_detail::validateSamples(renderDevice, linearDepthFormat, TEX_FORMAT_UNKNOWN,
                                             sampleCount);
        }
        if (!std::isfinite(minimumNdcZ)) {
            throw std::runtime_error("Diligent device reported invalid NDC depth");
        }
        vertexShader = diligent_detail::createShader(
            renderDevice, SHADER_TYPE_VERTEX, "OpenBus textured mesh VS", vertexSource, true);
        skinnedVertexShader = diligent_detail::createShader(renderDevice, SHADER_TYPE_VERTEX,
                                                            "OpenBus skinned mesh VS", vertexSource,
                                                            true, "mainSkinned");
        pixelShader = diligent_detail::createShader(renderDevice, SHADER_TYPE_PIXEL,
                                                    "OpenBus textured mesh PS", pixelSource, true,
                                                    linearDepthEnabled ? "mainDepth" : "main");
        if (motionEnabled) {
            motionPixelShader = diligent_detail::createShader(
                renderDevice, SHADER_TYPE_PIXEL, "OpenBus motion textured mesh PS", pixelSource,
                true, linearDepthEnabled ? "mainMotionDepth" : "mainMotion");
        }
        shadowVertexShader = diligent_detail::createShader(
            renderDevice, SHADER_TYPE_VERTEX, "OpenBus spotlight shadow VS", shadowVertexSource);
        shadowSkinnedVertexShader = diligent_detail::createShader(
            renderDevice, SHADER_TYPE_VERTEX, "OpenBus skinned spotlight shadow VS",
            shadowSkinnedVertexSource);
        shadowPixelShader = diligent_detail::createShader(
            renderDevice, SHADER_TYPE_PIXEL, "OpenBus spotlight shadow PS", shadowPixelSource);
        shadowArrayPixelShader = diligent_detail::createShader(renderDevice, SHADER_TYPE_PIXEL,
                                                               "OpenBus spotlight shadow array PS",
                                                               shadowArrayPixelSource);

        BufferDesc transformDesc;
        transformDesc.Name = "OpenBus model transform";
        transformDesc.Size = sizeof(DiligentModelTransform);
        transformDesc.Usage = USAGE_DYNAMIC;
        transformDesc.BindFlags = BIND_UNIFORM_BUFFER;
        transformDesc.CPUAccessFlags = CPU_ACCESS_WRITE;
        renderDevice.CreateBuffer(transformDesc, nullptr, &transform);
        require(static_cast<bool>(transform), "Failed to create Diligent model transform buffer");
        transformDesc.Name = "OpenBus material and lighting parameters";
        transformDesc.Size = sizeof(DiligentMaterialParameters);
        renderDevice.CreateBuffer(transformDesc, nullptr, &material);
        require(static_cast<bool>(material), "Failed to create Diligent material buffer");
        transformDesc.Name = "OpenBus human bone palettes";
        transformDesc.Size = sizeof(DiligentHumanBones);
        renderDevice.CreateBuffer(transformDesc, nullptr, &bones);
        require(static_cast<bool>(bones), "Failed to create Diligent human bone buffer");
        transformDesc.Name = "OpenBus spotlight shadow transform";
        transformDesc.Size = sizeof(DiligentShadowTransform);
        renderDevice.CreateBuffer(transformDesc, nullptr, &shadowTransform);
        require(static_cast<bool>(shadowTransform),
                "Failed to create Diligent spotlight shadow transform buffer");
        transformDesc.Name = "OpenBus spotlight shadow material";
        transformDesc.Size = sizeof(DiligentShadowMaterial);
        renderDevice.CreateBuffer(transformDesc, nullptr, &shadowMaterial);
        require(static_cast<bool>(shadowMaterial),
                "Failed to create Diligent spotlight shadow material buffer");
        pointLightBuffer.ensureCapacity(renderDevice, 1);
        spotLightBuffer.ensureCapacity(renderDevice, 1);

        const LayoutElement shadowElements[] = {
            {0, 0, 3, VT_FLOAT32, False, offsetof(ModelVertex, x), sizeof(ModelVertex)},
            {1, 0, 3, VT_FLOAT32, False, offsetof(ModelVertex, u), sizeof(ModelVertex)}};
        const LayoutElement shadowSkinnedElements[] = {
            {0, 0, 3, VT_FLOAT32, False, offsetof(ModelVertex, x), sizeof(ModelVertex)},
            {1, 0, 3, VT_FLOAT32, False, offsetof(ModelVertex, u), sizeof(ModelVertex)},
            {4, 1, 4, VT_FLOAT32, False, 0, sizeof(HumanVertexWeights)},
            {5, 1, 4, VT_FLOAT32, False, 4 * sizeof(float), sizeof(HumanVertexWeights)},
            {6, 1, 4, VT_FLOAT32, False, 8 * sizeof(float), sizeof(HumanVertexWeights)},
            {7, 1, 4, VT_FLOAT32, False, 12 * sizeof(float), sizeof(HumanVertexWeights)}};
        SamplerDesc shadowSampler;
        shadowSampler.AddressU = TEXTURE_ADDRESS_WRAP;
        shadowSampler.AddressV = TEXTURE_ADDRESS_WRAP;
        const auto createShadowPipeline = [&](const char* pipelineName, const char* textureName,
                                              const char* samplerName,
                                              Diligent::IShader* pixelShader, bool skinned,
                                              bool cutout, Pipeline& pipeline) {
            const ShaderResourceVariableDesc variables[] = {
                {SHADER_TYPE_VERTEX, "ShadowTransform", SHADER_RESOURCE_VARIABLE_TYPE_STATIC},
                {SHADER_TYPE_PIXEL, "ShadowMaterial", SHADER_RESOURCE_VARIABLE_TYPE_STATIC},
                {SHADER_TYPE_PIXEL, textureName, SHADER_RESOURCE_VARIABLE_TYPE_DYNAMIC},
                {SHADER_TYPE_VERTEX, "HumanBones", SHADER_RESOURCE_VARIABLE_TYPE_STATIC}};
            const ImmutableSamplerDesc samplers[] = {
                {SHADER_TYPE_PIXEL, samplerName, shadowSampler}};
            GraphicsPipelineStateCreateInfo info;
            info.PSODesc.Name = pipelineName;
            info.PSODesc.PipelineType = PIPELINE_TYPE_GRAPHICS;
            info.PSODesc.ResourceLayout.Variables = variables;
            info.PSODesc.ResourceLayout.NumVariables =
                skinned ? static_cast<Uint32>(std::size(variables)) : 3;
            info.PSODesc.ResourceLayout.ImmutableSamplers = samplers;
            info.PSODesc.ResourceLayout.NumImmutableSamplers =
                static_cast<Uint32>(std::size(samplers));
            info.GraphicsPipeline.NumRenderTargets = 0;
            info.GraphicsPipeline.DSVFormat = TEX_FORMAT_D32_FLOAT;
            info.GraphicsPipeline.SmplDesc.Count = 1;
            info.GraphicsPipeline.PrimitiveTopology = PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
            info.GraphicsPipeline.RasterizerDesc.CullMode = CULL_MODE_NONE;
            if (cutout) {
                info.GraphicsPipeline.RasterizerDesc.DepthBias = 1;
                info.GraphicsPipeline.RasterizerDesc.SlopeScaledDepthBias = 1.0F;
            }
            info.GraphicsPipeline.DepthStencilDesc.DepthEnable = True;
            info.GraphicsPipeline.DepthStencilDesc.DepthWriteEnable = True;
            info.GraphicsPipeline.DepthStencilDesc.DepthFunc = COMPARISON_FUNC_LESS;
            info.GraphicsPipeline.InputLayout.LayoutElements =
                skinned ? shadowSkinnedElements : shadowElements;
            info.GraphicsPipeline.InputLayout.NumElements =
                skinned ? static_cast<Uint32>(std::size(shadowSkinnedElements))
                        : static_cast<Uint32>(std::size(shadowElements));
            info.pVS = skinned ? shadowSkinnedVertexShader : shadowVertexShader;
            info.pPS = pixelShader;
            renderDevice.CreateGraphicsPipelineState(info, &pipeline.state);
            require(static_cast<bool>(pipeline.state),
                    "Failed to create Diligent spotlight shadow pipeline");
            auto* transformVariable =
                pipeline.state->GetStaticVariableByName(SHADER_TYPE_VERTEX, "ShadowTransform");
            auto* materialVariable =
                pipeline.state->GetStaticVariableByName(SHADER_TYPE_PIXEL, "ShadowMaterial");
            auto* boneVariable =
                skinned ? pipeline.state->GetStaticVariableByName(SHADER_TYPE_VERTEX, "HumanBones")
                        : nullptr;
            require(transformVariable != nullptr && materialVariable != nullptr &&
                        (!skinned || boneVariable != nullptr),
                    "Diligent spotlight shadow constant-buffer binding is missing");
            transformVariable->Set(shadowTransform);
            materialVariable->Set(shadowMaterial);
            if (boneVariable != nullptr) {
                boneVariable->Set(bones);
            }
            pipeline.state->CreateShaderResourceBinding(&pipeline.binding, true);
            require(static_cast<bool>(pipeline.binding),
                    "Failed to create Diligent spotlight shadow shader bindings");
        };
        createShadowPipeline("OpenBus 2D spotlight shadow casters", "ShadowAlphaTexture",
                             "ShadowAlphaTexture_sampler", shadowPixelShader, false, false,
                             shadowPipeline);
        createShadowPipeline("OpenBus array spotlight shadow casters", "ShadowAlphaTextureArray",
                             "ShadowAlphaTextureArray_sampler", shadowArrayPixelShader, false,
                             false, shadowArrayPipeline);
        createShadowPipeline("OpenBus skinned 2D spotlight shadow casters", "ShadowAlphaTexture",
                             "ShadowAlphaTexture_sampler", shadowPixelShader, true, false,
                             shadowSkinnedPipeline);
        createShadowPipeline("OpenBus skinned array spotlight shadow casters",
                             "ShadowAlphaTextureArray", "ShadowAlphaTextureArray_sampler",
                             shadowArrayPixelShader, true, false, shadowArraySkinnedPipeline);
        createShadowPipeline("OpenBus 2D spotlight cutout shadow casters", "ShadowAlphaTexture",
                             "ShadowAlphaTexture_sampler", shadowPixelShader, false, true,
                             shadowCutoutPipeline);
        createShadowPipeline("OpenBus array spotlight cutout shadow casters",
                             "ShadowAlphaTextureArray", "ShadowAlphaTextureArray_sampler",
                             shadowArrayPixelShader, false, true, shadowArrayCutoutPipeline);
        createShadowPipeline("OpenBus skinned 2D spotlight cutout shadow casters",
                             "ShadowAlphaTexture", "ShadowAlphaTexture_sampler", shadowPixelShader,
                             true, true, shadowSkinnedCutoutPipeline);
        createShadowPipeline("OpenBus skinned array spotlight cutout shadow casters",
                             "ShadowAlphaTextureArray", "ShadowAlphaTextureArray_sampler",
                             shadowArrayPixelShader, true, true, shadowArraySkinnedCutoutPipeline);

        createFallbackTexture(renderDevice, false, fallback2DTexture, fallback2DView);
        createFallbackTexture(renderDevice, true, fallbackArrayTexture, fallbackArrayView);
        createFallbackTexture(renderDevice, false, fallbackNormalTexture, fallbackNormalView,
                              {128, 128, 255, 255});
        createFallbackTexture(renderDevice, false, fallbackNightmapTexture, fallbackNightmapView,
                              {0, 0, 0, 255});
    }

    static std::pair<std::size_t, Diligent::TEXTURE_ADDRESS_MODE>
    textureAddress(TextureAddressMode mode) {
        using namespace Diligent;
        switch (mode) {
        case TextureAddressMode::Repeat:
            return {0, TEXTURE_ADDRESS_WRAP};
        case TextureAddressMode::Clamp:
            return {1, TEXTURE_ADDRESS_CLAMP};
        case TextureAddressMode::Border:
            return {2, TEXTURE_ADDRESS_BORDER};
        case TextureAddressMode::Mirror:
            return {3, TEXTURE_ADDRESS_MIRROR};
        case TextureAddressMode::MirrorOnce:
            return {4, TEXTURE_ADDRESS_MIRROR_ONCE};
        }
        throw std::invalid_argument("Invalid Diligent material texture address mode");
    }

    Diligent::Uint32 selectedAnisotropy(const DiligentMaterialState& materialState) const {
        const Diligent::Uint32 requestedAnisotropy =
            hasDefaultAnisotropy ? defaultAnisotropy : materialState.anisotropy;
        const auto requested =
            std::find(anisotropyLevels.begin(), anisotropyLevels.end(), requestedAnisotropy);
        if (requested == anisotropyLevels.end()) {
            throw std::invalid_argument("Invalid Diligent material anisotropy level");
        }
        if (materialState.nearestBaseTexture) {
            return 1;
        }
        return std::min(requestedAnisotropy, maximumAnisotropy);
    }

    Pipeline& pipelineFor(const DiligentMaterialState& materialState, bool motion, bool skinned) {
        using namespace Diligent;
        const auto [uIndex, addressU] = textureAddress(materialState.addressU);
        const auto [vIndex, addressV] = textureAddress(materialState.addressV);
        const Uint32 anisotropy = selectedAnisotropy(materialState);
        const auto anisotropyLevel =
            std::find(anisotropyLevels.begin(), anisotropyLevels.end(), anisotropy);
        const std::size_t anisotropyIndex =
            static_cast<std::size_t>(std::distance(anisotropyLevels.begin(), anisotropyLevel));
        std::size_t depthIndex = 0;
        switch (materialState.depthMode) {
        case DiligentDepthMode::Disabled:
            break;
        case DiligentDepthMode::Less:
            depthIndex = 1;
            break;
        case DiligentDepthMode::LessEqual:
            depthIndex = 2;
            break;
        default:
            throw std::invalid_argument("Invalid Diligent depth mode");
        }
        const std::size_t baseVariant =
            (((uIndex * addressModeCount + vIndex) * 3U + depthIndex) * 256U) |
            (diligentMaterialBlends(materialState) ? 2U : 0U) |
            (materialState.depthWrite ? 1U : 0U) | (materialState.nearestBaseTexture ? 4U : 0U) |
            (skinned ? 8U : 0U) | (materialState.colorWriteEnabled ? 0U : 16U) |
            (materialState.nearestFreeTexture ? 32U : 0U) |
            (materialState.nearestTextTexture ? 64U : 0U) |
            (materialState.alphaMode == DiligentAlphaMode::Cutout ? 128U : 0U);
        const std::size_t variant = baseVariant * anisotropyLevels.size() + anisotropyIndex;
        Pipeline& cached = motion ? motionPipelines[variant] : pipelines[variant];
        if (cached.state) {
            return cached;
        }

        const LayoutElement elements[] = {
            {0, 0, 3, VT_FLOAT32, False, offsetof(ModelVertex, x), sizeof(ModelVertex)},
            {1, 0, 3, VT_FLOAT32, False, offsetof(ModelVertex, u), sizeof(ModelVertex)},
            {2, 0, 4, VT_FLOAT32, False, offsetof(ModelVertex, r), sizeof(ModelVertex)},
            {3, 0, 3, VT_FLOAT32, False, offsetof(ModelVertex, nx), sizeof(ModelVertex)},
            {4, 1, 4, VT_FLOAT32, False, 0, sizeof(HumanVertexWeights)},
            {5, 1, 4, VT_FLOAT32, False, 4 * sizeof(float), sizeof(HumanVertexWeights)},
            {6, 1, 4, VT_FLOAT32, False, 8 * sizeof(float), sizeof(HumanVertexWeights)},
            {7, 1, 4, VT_FLOAT32, False, 12 * sizeof(float), sizeof(HumanVertexWeights)},
        };
        ShaderResourceVariableDesc variables[] = {
            {SHADER_TYPE_PIXEL, "BaseTexture", SHADER_RESOURCE_VARIABLE_TYPE_DYNAMIC},
            {SHADER_TYPE_PIXEL, "BaseTextureArray", SHADER_RESOURCE_VARIABLE_TYPE_DYNAMIC},
            {SHADER_TYPE_PIXEL, "ArrayReflectionTexture0", SHADER_RESOURCE_VARIABLE_TYPE_DYNAMIC},
            {SHADER_TYPE_PIXEL, "ArrayReflectionTexture1", SHADER_RESOURCE_VARIABLE_TYPE_DYNAMIC},
            {SHADER_TYPE_PIXEL, "ArrayReflectionTexture2", SHADER_RESOURCE_VARIABLE_TYPE_DYNAMIC},
            {SHADER_TYPE_PIXEL, "ArrayReflectionTexture3", SHADER_RESOURCE_VARIABLE_TYPE_DYNAMIC},
            {SHADER_TYPE_PIXEL, "ArrayReflectionTexture4", SHADER_RESOURCE_VARIABLE_TYPE_DYNAMIC},
            {SHADER_TYPE_PIXEL, "ArrayReflectionTexture5", SHADER_RESOURCE_VARIABLE_TYPE_DYNAMIC},
            {SHADER_TYPE_PIXEL, "ArrayReflectionTexture6", SHADER_RESOURCE_VARIABLE_TYPE_DYNAMIC},
            {SHADER_TYPE_PIXEL, "ArrayReflectionTexture7", SHADER_RESOURCE_VARIABLE_TYPE_DYNAMIC},
            {SHADER_TYPE_PIXEL, "SpotlightShadowMaps", SHADER_RESOURCE_VARIABLE_TYPE_DYNAMIC},
            {SHADER_TYPE_PIXEL, "NormalTexture", SHADER_RESOURCE_VARIABLE_TYPE_DYNAMIC},
            {SHADER_TYPE_PIXEL, "LightmapTexture", SHADER_RESOURCE_VARIABLE_TYPE_DYNAMIC},
            {SHADER_TYPE_PIXEL, "NightmapTexture", SHADER_RESOURCE_VARIABLE_TYPE_DYNAMIC},
            {SHADER_TYPE_PIXEL, "TransmapTexture", SHADER_RESOURCE_VARIABLE_TYPE_DYNAMIC},
            {SHADER_TYPE_PIXEL, "DetailTexture", SHADER_RESOURCE_VARIABLE_TYPE_DYNAMIC},
            {SHADER_TYPE_PIXEL, "EnvironmentTexture", SHADER_RESOURCE_VARIABLE_TYPE_DYNAMIC},
            {SHADER_TYPE_PIXEL, "FreeTexture", SHADER_RESOURCE_VARIABLE_TYPE_DYNAMIC},
            {SHADER_TYPE_PIXEL, "TextTexture", SHADER_RESOURCE_VARIABLE_TYPE_DYNAMIC},
            {SHADER_TYPE_PIXEL, "PointLights", SHADER_RESOURCE_VARIABLE_TYPE_DYNAMIC},
            {SHADER_TYPE_PIXEL, "SpotLights", SHADER_RESOURCE_VARIABLE_TYPE_DYNAMIC},
        };
        SamplerDesc auxiliarySampler;
        auxiliarySampler.AddressU = TEXTURE_ADDRESS_WRAP;
        auxiliarySampler.AddressV = TEXTURE_ADDRESS_WRAP;
        SamplerDesc baseSampler = auxiliarySampler;
        SamplerDesc shadowMapSampler;
        shadowMapSampler.AddressU = TEXTURE_ADDRESS_CLAMP;
        shadowMapSampler.AddressV = TEXTURE_ADDRESS_CLAMP;
        shadowMapSampler.MinFilter = FILTER_TYPE_POINT;
        shadowMapSampler.MagFilter = FILTER_TYPE_POINT;
        shadowMapSampler.MipFilter = FILTER_TYPE_POINT;
        SamplerDesc freeSampler = auxiliarySampler;
        SamplerDesc textSampler = auxiliarySampler;
        baseSampler.AddressU = addressU;
        baseSampler.AddressV = addressV;
        if (materialState.nearestFreeTexture) {
            freeSampler.MinFilter = FILTER_TYPE_POINT;
            freeSampler.MagFilter = FILTER_TYPE_POINT;
            freeSampler.MipFilter = FILTER_TYPE_POINT;
        }
        if (materialState.nearestTextTexture) {
            textSampler.MinFilter = FILTER_TYPE_POINT;
            textSampler.MagFilter = FILTER_TYPE_POINT;
            textSampler.MipFilter = FILTER_TYPE_POINT;
        }
        if (anisotropy > 1) {
            baseSampler.MinFilter = FILTER_TYPE_ANISOTROPIC;
            baseSampler.MagFilter = FILTER_TYPE_ANISOTROPIC;
            baseSampler.MipFilter = FILTER_TYPE_ANISOTROPIC;
            baseSampler.MaxAnisotropy = anisotropy;
        } else if (materialState.nearestBaseTexture) {
            baseSampler.MinFilter = FILTER_TYPE_POINT;
            baseSampler.MagFilter = FILTER_TYPE_POINT;
            baseSampler.MipFilter = FILTER_TYPE_POINT;
        }
        const ImmutableSamplerDesc samplers[] = {
            {SHADER_TYPE_PIXEL, "BaseTexture", baseSampler},
            {SHADER_TYPE_PIXEL, "BaseTextureArray", baseSampler},
            {SHADER_TYPE_PIXEL, "ArrayReflectionTexture0", baseSampler},
            {SHADER_TYPE_PIXEL, "ArrayReflectionTexture1", baseSampler},
            {SHADER_TYPE_PIXEL, "ArrayReflectionTexture2", baseSampler},
            {SHADER_TYPE_PIXEL, "ArrayReflectionTexture3", baseSampler},
            {SHADER_TYPE_PIXEL, "ArrayReflectionTexture4", baseSampler},
            {SHADER_TYPE_PIXEL, "ArrayReflectionTexture5", baseSampler},
            {SHADER_TYPE_PIXEL, "ArrayReflectionTexture6", baseSampler},
            {SHADER_TYPE_PIXEL, "ArrayReflectionTexture7", baseSampler},
            {SHADER_TYPE_PIXEL, "SpotlightShadowMaps", shadowMapSampler},
            {SHADER_TYPE_PIXEL, "NormalTexture", auxiliarySampler},
            {SHADER_TYPE_PIXEL, "LightmapTexture", auxiliarySampler},
            {SHADER_TYPE_PIXEL, "NightmapTexture", auxiliarySampler},
            {SHADER_TYPE_PIXEL, "TransmapTexture", auxiliarySampler},
            {SHADER_TYPE_PIXEL, "DetailTexture", auxiliarySampler},
            {SHADER_TYPE_PIXEL, "EnvironmentTexture", auxiliarySampler},
            {SHADER_TYPE_PIXEL, "FreeTexture", freeSampler},
            {SHADER_TYPE_PIXEL, "TextTexture", textSampler},
        };
        GraphicsPipelineStateCreateInfo info;
        info.PSODesc.Name =
            motion ? "OpenBus textured mesh with motion data" : "OpenBus textured mesh";
        info.PSODesc.PipelineType = PIPELINE_TYPE_GRAPHICS;
        info.PSODesc.ResourceLayout.Variables = variables;
        info.PSODesc.ResourceLayout.NumVariables = static_cast<Uint32>(std::size(variables));
        info.PSODesc.ResourceLayout.ImmutableSamplers = samplers;
        info.PSODesc.ResourceLayout.NumImmutableSamplers = static_cast<Uint32>(std::size(samplers));
        info.GraphicsPipeline.RTVFormats[0] = colorFormat;
        info.GraphicsPipeline.NumRenderTargets =
            motion ? (linearDepthEnabled ? 4 : 3) : (linearDepthEnabled ? 2 : 1);
        if (!materialState.colorWriteEnabled) {
            for (Uint32 index = 0; index < info.GraphicsPipeline.NumRenderTargets; ++index) {
                info.GraphicsPipeline.BlendDesc.RenderTargets[index].RenderTargetWriteMask =
                    COLOR_MASK_NONE;
            }
        }
        if (motion) {
            info.GraphicsPipeline.RTVFormats[1] = motionVectorFormat;
            info.GraphicsPipeline.RTVFormats[2] = motionValidityFormat;
        }
        if (linearDepthEnabled) {
            info.GraphicsPipeline.RTVFormats[motion ? 3 : 1] = linearDepthFormat;
        }
        info.GraphicsPipeline.DSVFormat = depthFormat;
        info.GraphicsPipeline.SmplDesc.Count = sampleCount;
        info.GraphicsPipeline.PrimitiveTopology = PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
        info.GraphicsPipeline.RasterizerDesc.CullMode = CULL_MODE_NONE;
        info.GraphicsPipeline.RasterizerDesc.FrontCounterClockwise = False;
        if (materialState.alphaMode == DiligentAlphaMode::Cutout && materialState.depthWrite) {
            info.GraphicsPipeline.RasterizerDesc.DepthBias = -1;
            info.GraphicsPipeline.RasterizerDesc.SlopeScaledDepthBias = -1.0F;
        }
        info.GraphicsPipeline.DepthStencilDesc.DepthEnable =
            depthFormat != TEX_FORMAT_UNKNOWN &&
            materialState.depthMode != DiligentDepthMode::Disabled;
        info.GraphicsPipeline.DepthStencilDesc.DepthWriteEnable =
            info.GraphicsPipeline.DepthStencilDesc.DepthEnable && materialState.depthWrite;
        info.GraphicsPipeline.DepthStencilDesc.DepthFunc =
            materialState.depthMode == DiligentDepthMode::LessEqual ? COMPARISON_FUNC_LESS_EQUAL
                                                                    : COMPARISON_FUNC_LESS;
        info.GraphicsPipeline.InputLayout.LayoutElements = elements;
        info.GraphicsPipeline.InputLayout.NumElements = skinned ? 8 : 4;
        if (diligentMaterialBlends(materialState)) {
            auto& blend = info.GraphicsPipeline.BlendDesc.RenderTargets[0];
            blend.BlendEnable = True;
            blend.SrcBlend = BLEND_FACTOR_SRC_ALPHA;
            blend.DestBlend = BLEND_FACTOR_INV_SRC_ALPHA;
            blend.BlendOp = BLEND_OPERATION_ADD;
            blend.SrcBlendAlpha = BLEND_FACTOR_SRC_ALPHA;
            blend.DestBlendAlpha = BLEND_FACTOR_INV_SRC_ALPHA;
            blend.BlendOpAlpha = BLEND_OPERATION_ADD;
        }
        info.pVS = skinned ? skinnedVertexShader.RawPtr() : vertexShader.RawPtr();
        info.pPS = motion ? motionPixelShader.RawPtr() : pixelShader.RawPtr();

        Pipeline created;
        device->CreateGraphicsPipelineState(info, &created.state);
        require(static_cast<bool>(created.state),
                "Failed to create Diligent textured mesh pipeline");
        auto* transformVariable =
            created.state->GetStaticVariableByName(SHADER_TYPE_VERTEX, "ModelTransform");
        auto* materialVariable =
            created.state->GetStaticVariableByName(SHADER_TYPE_PIXEL, "ModelMaterial");
        require(transformVariable != nullptr && materialVariable != nullptr,
                "Diligent model uniform binding is missing");
        transformVariable->Set(transform);
        materialVariable->Set(material);
        if (skinned) {
            auto* boneVariable =
                created.state->GetStaticVariableByName(SHADER_TYPE_VERTEX, "HumanBones");
            require(boneVariable != nullptr, "Diligent human bone binding is missing");
            boneVariable->Set(bones);
        }
        created.state->CreateShaderResourceBinding(&created.binding, true);
        require(static_cast<bool>(created.binding),
                "Failed to create Diligent model shader bindings");
        cached = std::move(created);
        return cached;
    }
};

DiligentTexturedMesh::DiligentTexturedMesh(Diligent::IRenderDevice& device,
                                           std::span<const ModelVertex> vertices)
    : impl_(std::make_unique<Impl>(device, vertices, std::span<const std::uint32_t>{}, false)) {}
DiligentTexturedMesh::DiligentTexturedMesh(Diligent::IRenderDevice& device,
                                           std::span<const ModelVertex> vertices,
                                           std::span<const std::uint32_t> indices)
    : impl_(std::make_unique<Impl>(device, vertices, indices, true)) {}
DiligentTexturedMesh::DiligentTexturedMesh(Diligent::IRenderDevice& device,
                                           std::span<const ModelVertex> vertices,
                                           std::span<const HumanVertexWeights> weights)
    : impl_(std::make_unique<Impl>(device, vertices, std::span<const std::uint32_t>{}, false,
                                   weights, true)) {}
DiligentTexturedMesh::DiligentTexturedMesh(Diligent::IRenderDevice& device,
                                           std::span<const ModelVertex> vertices,
                                           std::span<const std::uint32_t> indices,
                                           std::span<const HumanVertexWeights> weights)
    : impl_(std::make_unique<Impl>(device, vertices, indices, true, weights, true)) {}
DiligentTexturedMesh::~DiligentTexturedMesh() = default;

DiligentTexturedMeshRenderer::DiligentTexturedMeshRenderer(Diligent::IRenderDevice& device,
                                                           Diligent::TEXTURE_FORMAT colorFormat,
                                                           Diligent::TEXTURE_FORMAT depthFormat)
    : impl_(std::make_unique<Impl>(device, colorFormat, depthFormat, Diligent::TEX_FORMAT_UNKNOWN,
                                   Diligent::TEX_FORMAT_UNKNOWN, Diligent::TEX_FORMAT_UNKNOWN)) {}
DiligentTexturedMeshRenderer::DiligentTexturedMeshRenderer(Diligent::IRenderDevice& device,
                                                           Diligent::TEXTURE_FORMAT colorFormat,
                                                           Diligent::TEXTURE_FORMAT depthFormat,
                                                           DiligentMultisampling multisampling)
    : impl_(std::make_unique<Impl>(device, colorFormat, depthFormat, Diligent::TEX_FORMAT_UNKNOWN,
                                   Diligent::TEX_FORMAT_UNKNOWN, Diligent::TEX_FORMAT_UNKNOWN,
                                   multisampling.samples)) {}
DiligentTexturedMeshRenderer::DiligentTexturedMeshRenderer(
    Diligent::IRenderDevice& device, Diligent::TEXTURE_FORMAT colorFormat,
    Diligent::TEXTURE_FORMAT depthFormat, Diligent::TEXTURE_FORMAT linearDepthFormat)
    : impl_(std::make_unique<Impl>(device, colorFormat, depthFormat, Diligent::TEX_FORMAT_UNKNOWN,
                                   Diligent::TEX_FORMAT_UNKNOWN, linearDepthFormat)) {}
DiligentTexturedMeshRenderer::DiligentTexturedMeshRenderer(
    Diligent::IRenderDevice& device, Diligent::TEXTURE_FORMAT colorFormat,
    Diligent::TEXTURE_FORMAT depthFormat, Diligent::TEXTURE_FORMAT motionVectorFormat,
    Diligent::TEXTURE_FORMAT motionValidityFormat)
    : impl_(std::make_unique<Impl>(device, colorFormat, depthFormat, motionVectorFormat,
                                   motionValidityFormat, Diligent::TEX_FORMAT_UNKNOWN)) {}
DiligentTexturedMeshRenderer::DiligentTexturedMeshRenderer(
    Diligent::IRenderDevice& device, Diligent::TEXTURE_FORMAT colorFormat,
    Diligent::TEXTURE_FORMAT depthFormat, Diligent::TEXTURE_FORMAT motionVectorFormat,
    Diligent::TEXTURE_FORMAT motionValidityFormat, DiligentMultisampling multisampling)
    : impl_(std::make_unique<Impl>(device, colorFormat, depthFormat, motionVectorFormat,
                                   motionValidityFormat, Diligent::TEX_FORMAT_UNKNOWN,
                                   multisampling.samples)) {}
DiligentTexturedMeshRenderer::DiligentTexturedMeshRenderer(
    Diligent::IRenderDevice& device, Diligent::TEXTURE_FORMAT colorFormat,
    Diligent::TEXTURE_FORMAT depthFormat, Diligent::TEXTURE_FORMAT motionVectorFormat,
    Diligent::TEXTURE_FORMAT motionValidityFormat, Diligent::TEXTURE_FORMAT linearDepthFormat)
    : impl_(std::make_unique<Impl>(device, colorFormat, depthFormat, motionVectorFormat,
                                   motionValidityFormat, linearDepthFormat)) {}
DiligentTexturedMeshRenderer::DiligentTexturedMeshRenderer(
    Diligent::IRenderDevice& device, Diligent::TEXTURE_FORMAT colorFormat,
    Diligent::TEXTURE_FORMAT depthFormat, Diligent::TEXTURE_FORMAT motionVectorFormat,
    Diligent::TEXTURE_FORMAT motionValidityFormat, Diligent::TEXTURE_FORMAT linearDepthFormat,
    DiligentMultisampling multisampling)
    : impl_(std::make_unique<Impl>(device, colorFormat, depthFormat, motionVectorFormat,
                                   motionValidityFormat, linearDepthFormat,
                                   multisampling.samples)) {}
DiligentTexturedMeshRenderer::~DiligentTexturedMeshRenderer() = default;

Diligent::Uint32 DiligentTexturedMeshRenderer::maximumAnisotropy() const noexcept {
    return impl_->maximumAnisotropy;
}

void DiligentTexturedMeshRenderer::setDefaultAnisotropy(Diligent::Uint32 anisotropy) {
    if (std::find(Impl::anisotropyLevels.begin(), Impl::anisotropyLevels.end(), anisotropy) ==
        Impl::anisotropyLevels.end()) {
        throw std::invalid_argument("Invalid Diligent default anisotropy level");
    }
    impl_->defaultAnisotropy = anisotropy;
    impl_->hasDefaultAnisotropy = true;
}

void DiligentTexturedMeshRenderer::setPointLights(Diligent::IDeviceContext& context,
                                                  std::span<const DiligentPointLight> pointLights) {
    if (pointLights.empty()) {
        impl_->pointLightCount = 0;
        return;
    }

    std::vector<DiligentGpuPointLight> gpuPointLights;
    gpuPointLights.reserve(pointLights.size());
    for (const DiligentPointLight& pointLight : pointLights) {
        const auto isFinite = [](float value) { return std::isfinite(value); };
        if (!std::all_of(pointLight.positionView.begin(), pointLight.positionView.end(),
                         isFinite) ||
            !std::all_of(pointLight.color.begin(), pointLight.color.end(), isFinite) ||
            !std::isfinite(pointLight.radius) || !std::isfinite(pointLight.intensity) ||
            pointLight.radius <= 0.0F || pointLight.intensity < 0.0F ||
            std::any_of(
                pointLight.color.begin(), pointLight.color.end(),
                [](float value) { return value < 0.0F; })) {
            throw std::invalid_argument("Diligent point light contains invalid parameters");
        }
        gpuPointLights.push_back({
            {pointLight.positionView[0], pointLight.positionView[1], pointLight.positionView[2],
             pointLight.radius},
            {pointLight.color[0], pointLight.color[1], pointLight.color[2], pointLight.intensity},
        });
    }

    impl_->pointLightBuffer.ensureCapacity(*impl_->device, gpuPointLights.size());
    impl_->pointLightBuffer.upload(context, gpuPointLights,
                                   "Failed to map Diligent point-light buffer");
    impl_->pointLightCount = static_cast<Diligent::Uint32>(gpuPointLights.size());
}

void DiligentTexturedMeshRenderer::setSpotLights(Diligent::IDeviceContext& context,
                                                 std::span<const DiligentSpotLight> spotLights) {
    if (spotLights.empty()) {
        impl_->spotLightCount = 0;
        return;
    }

    std::vector<DiligentGpuSpotLight> gpuSpotLights;
    gpuSpotLights.reserve(spotLights.size());
    constexpr float degreesToRadians = std::numbers::pi_v<float> / 180.0F;
    for (const DiligentSpotLight& spotLight : spotLights) {
        const auto isFinite = [](float value) { return std::isfinite(value); };
        if (!std::all_of(spotLight.positionView.begin(), spotLight.positionView.end(), isFinite) ||
            !std::all_of(spotLight.directionView.begin(), spotLight.directionView.end(),
                         isFinite) ||
            !std::all_of(spotLight.color.begin(), spotLight.color.end(), isFinite) ||
            !isFinite(spotLight.range) || !isFinite(spotLight.intensity) ||
            !isFinite(spotLight.innerAngleDegrees) || !isFinite(spotLight.outerAngleDegrees) ||
            spotLight.range <= 0.0F || spotLight.intensity < 0.0F ||
            spotLight.innerAngleDegrees < 0.0F ||
            spotLight.outerAngleDegrees < spotLight.innerAngleDegrees ||
            spotLight.outerAngleDegrees > 180.0F || !isFinite(spotLight.shadowBias) ||
            spotLight.shadowBias < 0.0F || spotLight.shadowBias > 1.0F ||
            std::any_of(spotLight.color.begin(), spotLight.color.end(),
                        [](float value) { return value < 0.0F; })) {
            throw std::invalid_argument("Diligent spotlight contains invalid parameters");
        }
        const float directionLength = std::hypot(
            spotLight.directionView[0], spotLight.directionView[1], spotLight.directionView[2]);
        if (!isFinite(directionLength) || directionLength <= 0.000001F) {
            throw std::invalid_argument("Diligent spotlight direction must be non-zero");
        }
        std::array<float, 16> shadowRows = {};
        if (spotLight.shadowValid) {
            if (spotLight.shadowMapIndex >= DiligentSpotlightShadowMaps::MAX_LIGHTS ||
                impl_->spotlightShadowMapView == nullptr) {
                throw std::invalid_argument(
                    "Diligent spotlight shadow layer or texture array is unavailable");
            }
            shadowRows = diligent_detail::matrixRows(spotLight.shadowClipFromView);
        }
        const float innerCos = std::cos(spotLight.innerAngleDegrees * degreesToRadians);
        const float outerCos = std::cos(spotLight.outerAngleDegrees * degreesToRadians);
        DiligentGpuSpotLight gpuSpotLight{
            {spotLight.positionView[0], spotLight.positionView[1], spotLight.positionView[2],
             spotLight.range},
            {spotLight.directionView[0] / directionLength,
             spotLight.directionView[1] / directionLength,
             spotLight.directionView[2] / directionLength, innerCos},
            {spotLight.color[0], spotLight.color[1], spotLight.color[2], outerCos},
            {spotLight.intensity, spotLight.shadowBias, 0.0F, 0.0F}};
        std::copy_n(shadowRows.begin(), 4, gpuSpotLight.shadowRow0.begin());
        std::copy_n(shadowRows.begin() + 4, 4, gpuSpotLight.shadowRow1.begin());
        std::copy_n(shadowRows.begin() + 8, 4, gpuSpotLight.shadowRow2.begin());
        std::copy_n(shadowRows.begin() + 12, 4, gpuSpotLight.shadowRow3.begin());
        gpuSpotLight.shadowInfo = {spotLight.shadowValid ? 1U : 0U, spotLight.shadowMapIndex, 0U,
                                   0U};
        gpuSpotLights.push_back(gpuSpotLight);
    }

    impl_->spotLightBuffer.ensureCapacity(*impl_->device, gpuSpotLights.size());
    impl_->spotLightBuffer.upload(context, gpuSpotLights,
                                  "Failed to map Diligent spotlight buffer");
    impl_->spotLightCount = static_cast<Diligent::Uint32>(gpuSpotLights.size());
}

void DiligentTexturedMeshRenderer::setSpotlightShadowMapView(Diligent::ITextureView* shadowMaps) {
    if (shadowMaps != nullptr &&
        shadowMaps->GetDesc().TextureDim != Diligent::RESOURCE_DIM_TEX_2D_ARRAY) {
        throw std::invalid_argument("Diligent spotlight shadows require a 2D-array shader view");
    }
    impl_->spotlightShadowMapView = shadowMaps;
}

std::size_t DiligentTexturedMeshRenderer::drawOpaqueShadowLayer(
    Diligent::IDeviceContext& context, DiligentMotionRenderTargets& sceneTargets,
    DiligentSpotlightShadowMaps& shadowMaps, Diligent::Uint32 lightIndex,
    std::span<const DiligentShadowCaster> casters) {
    struct PreparedCaster {
        const DiligentTexturedMesh* mesh;
        DiligentShadowTransform transform;
        DiligentShadowMaterial material;
        Diligent::ITextureView* alphaTexture;
        bool alphaTextureArray;
        bool alphaCutout;
        std::optional<DiligentHumanBones> bones;
    };
    std::vector<PreparedCaster> preparedCasters;
    preparedCasters.reserve(casters.size());
    for (const DiligentShadowCaster& caster : casters) {
        if (caster.mesh == nullptr) {
            throw std::invalid_argument("Diligent spotlight shadow caster mesh is null");
        }
        if (caster.mesh->impl_->device.RawPtr() != impl_->device.RawPtr()) {
            throw std::invalid_argument(
                "Diligent spotlight shadow caster belongs to a different device");
        }
        const auto finite = [](float value) { return std::isfinite(value); };
        if (!std::all_of(caster.textureTransform.begin(), caster.textureTransform.end(), finite) ||
            !finite(caster.alphaCutoff) || caster.alphaCutoff < 0.0F || caster.alphaCutoff > 1.0F ||
            (caster.alphaCutoff > 0.0F && caster.alphaTexture == nullptr)) {
            throw std::invalid_argument("Diligent spotlight shadow alpha material is invalid");
        }
        bool alphaTextureArray = false;
        if (caster.alphaTexture != nullptr) {
            const auto& viewDesc = caster.alphaTexture->GetDesc();
            if (viewDesc.ViewType != Diligent::TEXTURE_VIEW_SHADER_RESOURCE ||
                caster.alphaTexture->GetTexture()->GetDesc().SampleCount != 1) {
                throw std::invalid_argument(
                    "Diligent spotlight shadow alpha texture must be a single-sample shader view");
            }
            if (viewDesc.TextureDim == Diligent::RESOURCE_DIM_TEX_2D) {
                if (caster.alphaArrayLayer != 0 || caster.useVertexArrayLayer) {
                    throw std::invalid_argument(
                        "Diligent 2D shadow alpha texture cannot select an array layer");
                }
            } else if (viewDesc.TextureDim == Diligent::RESOURCE_DIM_TEX_2D_ARRAY) {
                const auto& textureDesc = caster.alphaTexture->GetTexture()->GetDesc();
                const Diligent::Uint32 firstSlice = viewDesc.FirstArraySlice;
                const Diligent::Uint32 viewLayerCount =
                    viewDesc.NumArraySlices == 0 ? textureDesc.GetArraySize() - firstSlice
                                                 : viewDesc.NumArraySlices;
                const bool invalidLayer = caster.useVertexArrayLayer
                                              ? (!caster.mesh->impl_->hasValidArrayLayers ||
                                                 caster.mesh->impl_->maximumArrayLayer >=
                                                     static_cast<float>(viewLayerCount))
                                              : (caster.alphaArrayLayer >= viewLayerCount ||
                                                 caster.alphaArrayLayer > (1U << 24));
                if (invalidLayer) {
                    throw std::invalid_argument(
                        "Diligent spotlight shadow alpha-array layer is outside the view");
                }
                alphaTextureArray = true;
            } else {
                throw std::invalid_argument(
                    "Diligent spotlight shadow alpha texture must be 2D or a 2D array");
            }
        } else if (caster.alphaArrayLayer != 0 || caster.useVertexArrayLayer) {
            throw std::invalid_argument(
                "Diligent spotlight shadow caster without an alpha texture cannot select a layer");
        }
        const bool skinned = caster.mesh->impl_->skinned;
        if (skinned != (caster.currentPose != nullptr)) {
            throw std::invalid_argument(
                "Diligent skinned shadow caster requires exactly one current bone palette");
        }
        std::optional<DiligentHumanBones> currentBones;
        if (skinned) {
            currentBones.emplace();
            for (std::size_t bone = 0; bone < HUMAN_BONE_COUNT; ++bone) {
                const auto rows = diligent_detail::matrixRows((*caster.currentPose)[bone]);
                std::copy(rows.begin(), rows.end(), currentBones->current.begin() + bone * 16);
                std::copy(rows.begin(), rows.end(), currentBones->previous.begin() + bone * 16);
            }
        }
        preparedCasters.push_back(
            {caster.mesh,
             {diligent_detail::clipTransformRows(caster.modelViewProjection, impl_->minimumNdcZ)},
             {caster.textureTransform,
              {caster.alphaCutoff > 0.0F ? 1.0F : 0.0F, caster.alphaCutoff,
               static_cast<float>(caster.alphaArrayLayer),
               caster.useVertexArrayLayer ? 1.0F : 0.0F}},
             caster.alphaTexture,
             alphaTextureArray,
             caster.alphaCutoff > 0.0F,
             std::move(currentBones)});
    }

    shadowMaps.clearLayer(context, lightIndex);
    struct RestoreSceneTargets {
        Diligent::IDeviceContext& context;
        DiligentMotionRenderTargets& targets;
        ~RestoreSceneTargets() {
            targets.bind(context);
            const Diligent::Viewport viewport{0.0F,
                                              0.0F,
                                              static_cast<float>(targets.width()),
                                              static_cast<float>(targets.height()),
                                              0.0F,
                                              1.0F};
            context.SetViewports(1, &viewport, targets.width(), targets.height());
        }
    } restore{context, sceneTargets};

    Diligent::ITextureView& depthLayer = shadowMaps.depthLayer(lightIndex);
    context.SetRenderTargets(0, nullptr, &depthLayer,
                             Diligent::RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    const Diligent::Uint32 resolution = shadowMaps.resolution();
    const Diligent::Viewport shadowViewport{
        0.0F, 0.0F, static_cast<float>(resolution), static_cast<float>(resolution), 0.0F, 1.0F};
    context.SetViewports(1, &shadowViewport, resolution, resolution);
    for (const PreparedCaster& caster : preparedCasters) {
        const bool skinned = caster.bones.has_value();
        auto& pipeline =
            caster.alphaCutout
                ? (skinned ? (caster.alphaTextureArray ? impl_->shadowArraySkinnedCutoutPipeline
                                                       : impl_->shadowSkinnedCutoutPipeline)
                           : (caster.alphaTextureArray ? impl_->shadowArrayCutoutPipeline
                                                       : impl_->shadowCutoutPipeline))
                : (skinned ? (caster.alphaTextureArray ? impl_->shadowArraySkinnedPipeline
                                                       : impl_->shadowSkinnedPipeline)
                           : (caster.alphaTextureArray ? impl_->shadowArrayPipeline
                                                       : impl_->shadowPipeline));
        const char* textureResource =
            caster.alphaTextureArray ? "ShadowAlphaTextureArray" : "ShadowAlphaTexture";
        context.SetPipelineState(pipeline.state);
        auto* alphaTextureVariable =
            pipeline.binding->GetVariableByName(Diligent::SHADER_TYPE_PIXEL, textureResource);
        require(alphaTextureVariable != nullptr,
                "Diligent spotlight shadow alpha-texture binding is missing");
        updateDynamicBuffer(context, *impl_->shadowTransform, &caster.transform,
                            sizeof(caster.transform),
                            "Failed to map Diligent spotlight shadow transform");
        updateDynamicBuffer(context, *impl_->shadowMaterial, &caster.material,
                            sizeof(caster.material),
                            "Failed to map Diligent spotlight shadow material");
        if (caster.bones) {
            updateDynamicBuffer(context, *impl_->bones, &*caster.bones, sizeof(*caster.bones),
                                "Failed to map Diligent spotlight shadow bone palette");
        }
        Diligent::ITextureView* fallbackTexture = caster.alphaTextureArray
                                                      ? impl_->fallbackArrayView.RawPtr()
                                                      : impl_->fallback2DView.RawPtr();
        alphaTextureVariable->Set(caster.alphaTexture != nullptr ? caster.alphaTexture
                                                                 : fallbackTexture);
        context.CommitShaderResources(pipeline.binding,
                                      Diligent::RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        Diligent::IBuffer* vertexBuffers[] = {caster.mesh->impl_->vertices,
                                              caster.mesh->impl_->skinWeights};
        context.SetVertexBuffers(0, skinned ? 2 : 1, vertexBuffers, nullptr,
                                 Diligent::RESOURCE_STATE_TRANSITION_MODE_TRANSITION,
                                 Diligent::SET_VERTEX_BUFFERS_FLAG_RESET);
        if (caster.mesh->impl_->indices) {
            context.SetIndexBuffer(caster.mesh->impl_->indices, 0,
                                   Diligent::RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
            Diligent::DrawIndexedAttribs attributes;
            attributes.NumIndices = caster.mesh->impl_->count;
            attributes.IndexType = Diligent::VT_UINT32;
            context.DrawIndexed(attributes);
        } else {
            Diligent::DrawAttribs attributes;
            attributes.NumVertices = caster.mesh->impl_->count;
            context.Draw(attributes);
        }
    }
    return preparedCasters.size();
}

void DiligentTexturedMeshRenderer::draw(Diligent::IDeviceContext& context,
                                        const DiligentTexturedMesh& mesh,
                                        Diligent::ITextureView& texture,
                                        const Matrix4& modelViewProjection,
                                        DiligentMaterialState material,
                                        const DiligentLightingParameters& lighting,
                                        Diligent::ITextureView* normalMap) {
    const auto dimension = texture.GetDesc().TextureDim;
    if (dimension != Diligent::RESOURCE_DIM_TEX_2D &&
        dimension != Diligent::RESOURCE_DIM_TEX_2D_ARRAY) {
        throw std::invalid_argument("Diligent model requires a 2D or 2D-array shader view");
    }
    drawImpl(context, mesh, texture, dimension == Diligent::RESOURCE_DIM_TEX_2D_ARRAY,
             modelViewProjection, material, lighting, nullptr, normalMap);
}

void DiligentTexturedMeshRenderer::drawMotion(
    Diligent::IDeviceContext& context, const DiligentTexturedMesh& mesh,
    Diligent::ITextureView& texture, const Matrix4& modelViewProjection,
    const DiligentMotionParameters& motion, DiligentMaterialState material,
    const DiligentLightingParameters& lighting, Diligent::ITextureView* normalMap) {
    const auto dimension = texture.GetDesc().TextureDim;
    if (dimension != Diligent::RESOURCE_DIM_TEX_2D &&
        dimension != Diligent::RESOURCE_DIM_TEX_2D_ARRAY) {
        throw std::invalid_argument("Diligent model requires a 2D or 2D-array shader view");
    }
    drawImpl(context, mesh, texture, dimension == Diligent::RESOURCE_DIM_TEX_2D_ARRAY,
             modelViewProjection, material, lighting, &motion, normalMap);
}

void DiligentTexturedMeshRenderer::drawSkinned(
    Diligent::IDeviceContext& context, const DiligentTexturedMesh& mesh,
    Diligent::ITextureView& texture, const Matrix4& modelViewProjection,
    const DiligentSkinDraw& skin, DiligentMaterialState material,
    const DiligentLightingParameters& lighting, Diligent::ITextureView* normalMap) {
    const auto dimension = texture.GetDesc().TextureDim;
    if (dimension != Diligent::RESOURCE_DIM_TEX_2D &&
        dimension != Diligent::RESOURCE_DIM_TEX_2D_ARRAY) {
        throw std::invalid_argument("Diligent model requires a 2D or 2D-array shader view");
    }
    drawImpl(context, mesh, texture, dimension == Diligent::RESOURCE_DIM_TEX_2D_ARRAY,
             modelViewProjection, material, lighting, nullptr, normalMap, &skin);
}

void DiligentTexturedMeshRenderer::drawSkinnedMotion(
    Diligent::IDeviceContext& context, const DiligentTexturedMesh& mesh,
    Diligent::ITextureView& texture, const Matrix4& modelViewProjection,
    const DiligentMotionParameters& motion, const DiligentSkinDraw& skin,
    DiligentMaterialState material, const DiligentLightingParameters& lighting,
    Diligent::ITextureView* normalMap) {
    const auto dimension = texture.GetDesc().TextureDim;
    if (dimension != Diligent::RESOURCE_DIM_TEX_2D &&
        dimension != Diligent::RESOURCE_DIM_TEX_2D_ARRAY) {
        throw std::invalid_argument("Diligent model requires a 2D or 2D-array shader view");
    }
    drawImpl(context, mesh, texture, dimension == Diligent::RESOURCE_DIM_TEX_2D_ARRAY,
             modelViewProjection, material, lighting, &motion, normalMap, &skin);
}

void DiligentTexturedMeshRenderer::drawImpl(
    Diligent::IDeviceContext& context, const DiligentTexturedMesh& mesh,
    Diligent::ITextureView& texture, bool textureArray, const Matrix4& modelViewProjection,
    DiligentMaterialState material, const DiligentLightingParameters& lighting,
    const DiligentMotionParameters* motion, Diligent::ITextureView* normalMap,
    const DiligentSkinDraw* skin) {
    using namespace Diligent;
    if (mesh.impl_->device.RawPtr() != impl_->device.RawPtr()) {
        throw std::invalid_argument("Diligent mesh belongs to a different device");
    }
    if (skin != nullptr && !mesh.impl_->skinWeights) {
        throw std::invalid_argument("Diligent skinned draw requires mesh skin weights");
    }
    if (motion != nullptr) {
        if (!impl_->motionEnabled) {
            throw std::logic_error(
                "Diligent motion draws require motion target formats at renderer creation");
        }
        if (motion->viewportWidth <= 0 || motion->viewportHeight <= 0) {
            throw std::invalid_argument("Diligent motion draw requires a positive viewport");
        }
    }
    if (texture.GetDesc().ViewType != TEXTURE_VIEW_SHADER_RESOURCE ||
        texture.GetDesc().TextureDim !=
            (textureArray ? RESOURCE_DIM_TEX_2D_ARRAY : RESOURCE_DIM_TEX_2D) ||
        texture.GetTexture()->GetDesc().SampleCount != 1) {
        throw std::invalid_argument(
            "Diligent model requires a single-sample 2D or 2D-array shader-resource view");
    }
    if (normalMap != nullptr && (normalMap->GetDesc().ViewType != TEXTURE_VIEW_SHADER_RESOURCE ||
                                 normalMap->GetDesc().TextureDim != RESOURCE_DIM_TEX_2D ||
                                 normalMap->GetTexture()->GetDesc().SampleCount != 1)) {
        throw std::invalid_argument(
            "Diligent normal map requires a single-sample 2D shader-resource view");
    }
    const auto validateAuxiliaryMap = [](ITextureView* view, const char* message) {
        if (view != nullptr && (view->GetDesc().ViewType != TEXTURE_VIEW_SHADER_RESOURCE ||
                                view->GetDesc().TextureDim != RESOURCE_DIM_TEX_2D ||
                                view->GetTexture()->GetDesc().SampleCount != 1)) {
            throw std::invalid_argument(message);
        }
    };
    const bool hasArrayReflection = std::any_of(
        material.arrayReflectionTextures.begin(), material.arrayReflectionTextures.end(),
        [](const ITextureView* view) { return view != nullptr; });
    if (hasArrayReflection && !textureArray) {
        throw std::invalid_argument(
            "Diligent material-array reflection requires a 2D-array base texture");
    }
    for (ITextureView* reflectionTexture : material.arrayReflectionTextures) {
        validateAuxiliaryMap(
            reflectionTexture,
            "Diligent material-array reflection requires a single-sample 2D shader view");
    }
    validateAuxiliaryMap(material.lightMap,
                         "Diligent lightmap requires a single-sample 2D shader-resource view");
    validateAuxiliaryMap(material.nightMap,
                         "Diligent nightmap requires a single-sample 2D shader-resource view");
    validateAuxiliaryMap(material.transMap,
                         "Diligent transmap requires a single-sample 2D shader-resource view");
    validateAuxiliaryMap(material.detailMap,
                         "Diligent detail map requires a single-sample 2D shader-resource view");
    validateAuxiliaryMap(
        material.environmentMap,
        "Diligent environment map requires a single-sample 2D shader-resource view");
    validateAuxiliaryMap(material.freeMap,
                         "Diligent free texture requires a single-sample 2D shader-resource view");
    validateAuxiliaryMap(material.textMap,
                         "Diligent text texture requires a single-sample 2D shader-resource view");
    if (textureArray) {
        const auto& viewDesc = texture.GetDesc();
        const auto& textureDesc = texture.GetTexture()->GetDesc();
        const Uint32 firstSlice = viewDesc.FirstArraySlice;
        const Uint32 viewLayerCount = viewDesc.NumArraySlices == 0
                                          ? textureDesc.GetArraySize() - firstSlice
                                          : viewDesc.NumArraySlices;
        if (!mesh.impl_->hasValidArrayLayers ||
            mesh.impl_->maximumArrayLayer >= static_cast<float>(viewLayerCount)) {
            throw std::invalid_argument(
                "Diligent mesh array coordinates must select an available zero-based layer");
        }
    }
    const auto rows = diligent_detail::clipTransformRows(modelViewProjection, impl_->minimumNdcZ);
    const auto previousRows =
        motion != nullptr ? diligent_detail::clipTransformRows(motion->previousModelViewProjection,
                                                               impl_->minimumNdcZ)
                          : rows;
    DiligentModelTransform transformParameters;
    transformParameters.currentRows = rows;
    transformParameters.previousRows = previousRows;
    transformParameters.viewRows = diligent_detail::matrixRows(lighting.objectToView);
    transformParameters.normalRows = diligent_detail::matrixRows(lighting.normalTransform);
    transformParameters.viewport = {
        0.0F,
        0.0F,
        motion != nullptr ? static_cast<float>(motion->viewportWidth) : 1.0F,
        motion != nullptr ? static_cast<float>(motion->viewportHeight) : 1.0F,
    };
    transformParameters.motion[0] =
        motion != nullptr && motion->historyValid && (skin == nullptr || skin->previous != nullptr)
            ? 1.0F
            : 0.0F;
    const auto isFinite = [](auto value) { return std::isfinite(value); };
    if (!std::all_of(material.baseColor.begin(), material.baseColor.end(), isFinite) ||
        !std::all_of(material.textureScale.begin(), material.textureScale.end(), isFinite) ||
        !std::all_of(material.textureOffset.begin(), material.textureOffset.end(), isFinite) ||
        !std::all_of(material.transMapScale.begin(), material.transMapScale.end(), isFinite) ||
        !std::all_of(material.detailMapScale.begin(), material.detailMapScale.end(), isFinite) ||
        !std::all_of(material.specularColor.begin(), material.specularColor.end(), isFinite) ||
        !std::all_of(material.emissionColor.begin(), material.emissionColor.end(), isFinite) ||
        !std::all_of(material.interiorLightPositionsView.begin(),
                     material.interiorLightPositionsView.end(),
                     [&](const auto& position) {
                         return std::all_of(position.begin(), position.end(), isFinite);
                     }) ||
        !std::all_of(
            material.interiorLightColors.begin(), material.interiorLightColors.end(),
            [&](const auto& color) { return std::all_of(color.begin(), color.end(), isFinite); }) ||
        !std::all_of(material.interiorLightStrengths.begin(), material.interiorLightStrengths.end(),
                     isFinite) ||
        !isFinite(material.specularPower) || !isFinite(material.normalMapStrength) ||
        !isFinite(material.lightMapStrength) || !isFinite(material.nightMapStrength) ||
        !isFinite(material.visibilityFade) || !isFinite(material.snowCoverage) ||
        !isFinite(material.environmentStrength) || !isFinite(material.environmentBrightness) ||
        !isFinite(material.dynamicTextureBrightness) || !isFinite(lighting.weatherBrightness) ||
        !std::all_of(lighting.weatherFog.color.begin(), lighting.weatherFog.color.end(),
                     isFinite) ||
        !isFinite(lighting.weatherFog.density) || !isFinite(lighting.weatherFog.layerHeight) ||
        !isFinite(lighting.weatherFog.cameraHeight) ||
        !std::all_of(lighting.weatherFog.worldUpView.begin(), lighting.weatherFog.worldUpView.end(),
                     isFinite)) {
        throw std::invalid_argument(
            "Diligent material contains non-finite color, UV, lighting, weather, or fade data");
    }
    if (material.specularPower <= 0.0F || material.normalMapStrength < 0.0F ||
        material.normalMapStrength > 1.0F || material.lightMapStrength < 0.0F ||
        material.lightMapStrength > 1.0F || material.nightMapStrength < 0.0F ||
        material.nightMapStrength > 1.0F || material.environmentBrightness < 0.0F ||
        material.interiorLightCount > DILIGENT_MAX_INTERIOR_LIGHTS ||
        lighting.weatherBrightness < 0.0F || lighting.weatherBrightness > 1.0F ||
        std::any_of(material.interiorLightColors.begin(), material.interiorLightColors.end(),
                    [](const auto& color) {
                        return std::any_of(color.begin(), color.end(), [](float value) {
                            return value < 0.0F || value > 1.0F;
                        });
                    }) ||
        std::any_of(material.interiorLightStrengths.begin(), material.interiorLightStrengths.end(),
                    [](float value) { return value < 0.0F; }) ||
        material.dynamicTextureBrightness < 0.0F ||
        std::any_of(
            material.specularColor.begin(), material.specularColor.end(),
            [](float value) { return value < 0.0F; }) ||
        std::any_of(
            material.emissionColor.begin(), material.emissionColor.end(),
            [](float value) { return value < 0.0F || value > 1.0F; })) {
        throw std::invalid_argument("Diligent material specular, emission, map-strength, or "
                                    "weather values are out of range");
    }
    if (lighting.weatherFog.density < 0.0F ||
        (lighting.weatherFog.layerHeight < 0.0F && lighting.weatherFog.layerHeight != -1.0F) ||
        std::any_of(
            lighting.weatherFog.color.begin(), lighting.weatherFog.color.end(),
            [](float value) { return value < 0.0F; }) ||
        (lighting.weatherFog.layerHeight >= 0.0F &&
         std::hypot(lighting.weatherFog.worldUpView[0], lighting.weatherFog.worldUpView[1],
                    lighting.weatherFog.worldUpView[2]) <= 0.000001F)) {
        throw std::invalid_argument("Diligent weather-fog parameters are out of range");
    }
    float lightDirectionLength = 1.0F;
    if (lighting.directionalEnabled) {
        if (!std::all_of(lighting.lightDirectionView.begin(), lighting.lightDirectionView.end(),
                         isFinite) ||
            !std::all_of(lighting.diffuseColor.begin(), lighting.diffuseColor.end(), isFinite) ||
            !isFinite(lighting.ambientIntensity) || !isFinite(lighting.diffuseIntensity)) {
            throw std::invalid_argument("Diligent directional lighting contains non-finite data");
        }
        lightDirectionLength =
            std::hypot(lighting.lightDirectionView[0], lighting.lightDirectionView[1],
                       lighting.lightDirectionView[2]);
        if (!isFinite(lightDirectionLength) || lightDirectionLength <= 0.000001F ||
            lighting.ambientIntensity < 0.0F || lighting.ambientIntensity > 1.0F ||
            lighting.diffuseIntensity < 0.0F ||
            std::any_of(
                lighting.diffuseColor.begin(), lighting.diffuseColor.end(),
                [](float value) { return value < 0.0F; }) ||
            !std::all_of(lighting.normalTransform.begin(), lighting.normalTransform.end(),
                         isFinite)) {
            throw std::invalid_argument(
                "Diligent directional lighting parameters are out of range or non-finite");
        }
    }
    DiligentMaterialParameters materialParameters;
    materialParameters.material = {
        material.baseColor[0],
        material.baseColor[1],
        material.baseColor[2],
        material.baseColor[3],
        alphaCutoff(material.alphaMode),
        std::clamp(material.visibilityFade, 0.0F, 1.0F),
        textureArray ? 1.0F : 0.0F,
        material.flipTextureY ? 1.0F : 0.0F,
        material.textureScale[0],
        material.textureScale[1],
        material.textureOffset[0],
        material.textureOffset[1],
        lighting.directionalEnabled ? lighting.lightDirectionView[0] / lightDirectionLength : 0.0F,
        lighting.directionalEnabled ? lighting.lightDirectionView[1] / lightDirectionLength : 0.0F,
        lighting.directionalEnabled ? lighting.lightDirectionView[2] / lightDirectionLength : 1.0F,
        lighting.directionalEnabled ? lighting.ambientIntensity : 1.0F,
        lighting.directionalEnabled ? lighting.diffuseColor[0] : 0.0F,
        lighting.directionalEnabled ? lighting.diffuseColor[1] : 0.0F,
        lighting.directionalEnabled ? lighting.diffuseColor[2] : 0.0F,
        lighting.directionalEnabled ? lighting.diffuseIntensity : 0.0F,
        material.specularColor[0],
        material.specularColor[1],
        material.specularColor[2],
        material.specularPower,
        material.emissionColor[0],
        material.emissionColor[1],
        material.emissionColor[2],
        0.0F,
        normalMap != nullptr ? 1.0F : 0.0F,
        material.normalMapStrength,
        material.useVertexColor ? 1.0F : 0.0F,
        material.textureAlphaOnly ? 1.0F : 0.0F,
        material.lightMap != nullptr ? 1.0F : 0.0F,
        material.lightMapStrength,
        0.0F,
        0.0F,
        material.nightMap != nullptr ? 1.0F : 0.0F,
        material.nightMapStrength,
        0.0F,
        0.0F,
        material.transMap != nullptr ? 1.0F : 0.0F,
        material.transMapScale[0],
        material.transMapScale[1],
        material.flipTransmapY ? 1.0F : 0.0F,
        material.detailMap != nullptr ? 1.0F : 0.0F,
        material.detailMapScale[0],
        material.detailMapScale[1],
        0.0F,
        material.snowCoverage,
        material.whiteAmbient ? 1.0F : 0.0F,
        material.cullBackFaces ? 1.0F : 0.0F,
        lighting.weatherBrightness,
        material.environmentMap != nullptr ? 1.0F : 0.0F,
        material.environmentStrength,
        material.environmentBrightness,
        0.0F,
        material.freeMap != nullptr ? 1.0F : 0.0F,
        material.textMap != nullptr ? 1.0F : 0.0F,
        material.weatherLitDynamicTexture ? 1.0F : 0.0F,
        material.dynamicTextureBrightness,
        lighting.weatherFog.color[0],
        lighting.weatherFog.color[1],
        lighting.weatherFog.color[2],
        lighting.weatherFog.density,
        lighting.weatherFog.layerHeight,
        lighting.weatherFog.cameraHeight,
        lighting.weatherFog.worldUpView[0],
        lighting.weatherFog.worldUpView[1],
        lighting.weatherFog.worldUpView[2],
        0.0F,
        0.0F,
        0.0F,
        material.arrayReflectionTextures[0] != nullptr ? 1.0F : 0.0F,
        material.arrayReflectionTextures[1] != nullptr ? 1.0F : 0.0F,
        material.arrayReflectionTextures[2] != nullptr ? 1.0F : 0.0F,
        material.arrayReflectionTextures[3] != nullptr ? 1.0F : 0.0F,
        material.arrayReflectionTextures[4] != nullptr ? 1.0F : 0.0F,
        material.arrayReflectionTextures[5] != nullptr ? 1.0F : 0.0F,
        material.arrayReflectionTextures[6] != nullptr ? 1.0F : 0.0F,
        material.arrayReflectionTextures[7] != nullptr ? 1.0F : 0.0F,
    };
    materialParameters.pointLights[0] = impl_->pointLightCount;
    materialParameters.pointLights[1] = lighting.directionalEnabled ? 1U : 0U;
    materialParameters.spotLights[0] = impl_->spotLightCount;
    materialParameters.interiorLights[0] =
        material.useInteriorLights ? material.interiorLightCount : 0U;
    for (std::size_t index = 0; index < DILIGENT_MAX_INTERIOR_LIGHTS; ++index) {
        std::copy(material.interiorLightPositionsView[index].begin(),
                  material.interiorLightPositionsView[index].end(),
                  materialParameters.interiorLightPositions[index].begin());
        std::copy(material.interiorLightColors[index].begin(),
                  material.interiorLightColors[index].end(),
                  materialParameters.interiorLightColors[index].begin());
        materialParameters.interiorLightStrengths[index] = material.interiorLightStrengths[index];
    }
    if (skin != nullptr) {
        DiligentHumanBones bones;
        const auto& previousBones = skin->previous != nullptr ? *skin->previous : skin->current;
        for (std::size_t bone = 0; bone < HUMAN_BONE_COUNT; ++bone) {
            const auto currentRows = diligent_detail::matrixRows(skin->current[bone]);
            const auto previousRows = diligent_detail::matrixRows(previousBones[bone]);
            std::copy(currentRows.begin(), currentRows.end(), bones.current.begin() + bone * 16);
            std::copy(previousRows.begin(), previousRows.end(), bones.previous.begin() + bone * 16);
        }
        updateDynamicBuffer(context, *impl_->bones, &bones, sizeof(bones),
                            "Failed to map Diligent human bone palettes");
    }
    updateDynamicBuffer(context, *impl_->transform, &transformParameters,
                        sizeof(transformParameters), "Failed to map Diligent model transform");
    updateDynamicBuffer(context, *impl_->material, &materialParameters, sizeof(materialParameters),
                        "Failed to map Diligent material parameters");
    auto& pipeline = impl_->pipelineFor(material, motion != nullptr, skin != nullptr);
    auto& binding = pipeline.binding;
    auto* textureVariable = binding->GetVariableByName(SHADER_TYPE_PIXEL, "BaseTexture");
    auto* arrayVariable = binding->GetVariableByName(SHADER_TYPE_PIXEL, "BaseTextureArray");
    constexpr std::array<const char*, DILIGENT_MAX_MATERIAL_ARRAY_LAYERS>
        arrayReflectionVariableNames = {"ArrayReflectionTexture0", "ArrayReflectionTexture1",
                                        "ArrayReflectionTexture2", "ArrayReflectionTexture3",
                                        "ArrayReflectionTexture4", "ArrayReflectionTexture5",
                                        "ArrayReflectionTexture6", "ArrayReflectionTexture7"};
    std::array<IShaderResourceVariable*, DILIGENT_MAX_MATERIAL_ARRAY_LAYERS>
        arrayReflectionVariables = {};
    for (std::size_t index = 0; index < arrayReflectionVariables.size(); ++index) {
        arrayReflectionVariables[index] =
            binding->GetVariableByName(SHADER_TYPE_PIXEL, arrayReflectionVariableNames[index]);
    }
    auto* shadowMapVariable = binding->GetVariableByName(SHADER_TYPE_PIXEL, "SpotlightShadowMaps");
    auto* normalMapVariable = binding->GetVariableByName(SHADER_TYPE_PIXEL, "NormalTexture");
    auto* lightMapVariable = binding->GetVariableByName(SHADER_TYPE_PIXEL, "LightmapTexture");
    auto* nightMapVariable = binding->GetVariableByName(SHADER_TYPE_PIXEL, "NightmapTexture");
    auto* transMapVariable = binding->GetVariableByName(SHADER_TYPE_PIXEL, "TransmapTexture");
    auto* detailMapVariable = binding->GetVariableByName(SHADER_TYPE_PIXEL, "DetailTexture");
    auto* environmentVariable = binding->GetVariableByName(SHADER_TYPE_PIXEL, "EnvironmentTexture");
    auto* freeTextureVariable = binding->GetVariableByName(SHADER_TYPE_PIXEL, "FreeTexture");
    auto* textTextureVariable = binding->GetVariableByName(SHADER_TYPE_PIXEL, "TextTexture");
    auto* pointLightVariable = binding->GetVariableByName(SHADER_TYPE_PIXEL, "PointLights");
    auto* spotLightVariable = binding->GetVariableByName(SHADER_TYPE_PIXEL, "SpotLights");
    require(textureVariable != nullptr && arrayVariable != nullptr &&
                std::all_of(
                    arrayReflectionVariables.begin(), arrayReflectionVariables.end(),
                    [](const IShaderResourceVariable* variable) { return variable != nullptr; }) &&
                shadowMapVariable != nullptr && normalMapVariable != nullptr &&
                lightMapVariable != nullptr && nightMapVariable != nullptr &&
                transMapVariable != nullptr && detailMapVariable != nullptr &&
                environmentVariable != nullptr && freeTextureVariable != nullptr &&
                textTextureVariable != nullptr && pointLightVariable != nullptr &&
                spotLightVariable != nullptr,
            "Diligent model shader resource binding is missing");
    textureVariable->Set(textureArray ? impl_->fallback2DView.RawPtr() : &texture);
    arrayVariable->Set(textureArray ? &texture : impl_->fallbackArrayView.RawPtr());
    for (std::size_t index = 0; index < arrayReflectionVariables.size(); ++index) {
        arrayReflectionVariables[index]->Set(material.arrayReflectionTextures[index] != nullptr
                                                 ? material.arrayReflectionTextures[index]
                                                 : impl_->fallback2DView.RawPtr());
    }
    shadowMapVariable->Set(impl_->spotlightShadowMapView != nullptr
                               ? impl_->spotlightShadowMapView
                               : impl_->fallbackArrayView.RawPtr());
    normalMapVariable->Set(normalMap != nullptr ? normalMap : impl_->fallbackNormalView.RawPtr());
    lightMapVariable->Set(material.lightMap != nullptr ? material.lightMap
                                                       : impl_->fallback2DView.RawPtr());
    nightMapVariable->Set(material.nightMap != nullptr ? material.nightMap
                                                       : impl_->fallbackNightmapView.RawPtr());
    transMapVariable->Set(material.transMap != nullptr ? material.transMap
                                                       : impl_->fallback2DView.RawPtr());
    detailMapVariable->Set(material.detailMap != nullptr ? material.detailMap
                                                         : impl_->fallback2DView.RawPtr());
    environmentVariable->Set(material.environmentMap != nullptr ? material.environmentMap
                                                                : impl_->fallback2DView.RawPtr());
    freeTextureVariable->Set(material.freeMap != nullptr ? material.freeMap
                                                         : impl_->fallback2DView.RawPtr());
    textTextureVariable->Set(material.textMap != nullptr ? material.textMap
                                                         : impl_->fallback2DView.RawPtr());
    pointLightVariable->Set(impl_->pointLightBuffer.view());
    spotLightVariable->Set(impl_->spotLightBuffer.view());
    IBuffer* buffers[] = {mesh.impl_->vertices, mesh.impl_->skinWeights};
    context.SetVertexBuffers(0, skin != nullptr ? 2 : 1, buffers, nullptr,
                             RESOURCE_STATE_TRANSITION_MODE_TRANSITION,
                             SET_VERTEX_BUFFERS_FLAG_RESET);
    context.SetPipelineState(pipeline.state);
    context.CommitShaderResources(binding, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    if (mesh.impl_->indices) {
        context.SetIndexBuffer(mesh.impl_->indices, 0, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        DrawIndexedAttribs attributes;
        attributes.NumIndices = mesh.impl_->count;
        attributes.IndexType = VT_UINT32;
        context.DrawIndexed(attributes);
    } else {
        DrawAttribs attributes;
        attributes.NumVertices = mesh.impl_->count;
        context.Draw(attributes);
    }
}

void DiligentTexturedMeshRenderer::draw(Diligent::IDeviceContext& context,
                                        const DiligentTexturedMesh& mesh,
                                        const DiligentTexture2D& texture,
                                        const Matrix4& modelViewProjection,
                                        DiligentMaterialState material,
                                        const DiligentLightingParameters& lighting,
                                        Diligent::ITextureView* normalMap) {
    if (&texture.owningDevice() != impl_->device.RawPtr()) {
        throw std::invalid_argument("Diligent texture belongs to a different device");
    }
    drawImpl(context, mesh, texture.shaderResourceView(), false, modelViewProjection, material,
             lighting, nullptr, normalMap);
}

void DiligentTexturedMeshRenderer::drawMotion(
    Diligent::IDeviceContext& context, const DiligentTexturedMesh& mesh,
    const DiligentTexture2D& texture, const Matrix4& modelViewProjection,
    const DiligentMotionParameters& motion, DiligentMaterialState material,
    const DiligentLightingParameters& lighting, Diligent::ITextureView* normalMap) {
    if (&texture.owningDevice() != impl_->device.RawPtr()) {
        throw std::invalid_argument("Diligent texture belongs to a different device");
    }
    drawImpl(context, mesh, texture.shaderResourceView(), false, modelViewProjection, material,
             lighting, &motion, normalMap);
}

void DiligentTexturedMeshRenderer::draw(Diligent::IDeviceContext& context,
                                        const DiligentTexturedMesh& mesh,
                                        const DiligentTexture2DArray& texture,
                                        const Matrix4& modelViewProjection,
                                        DiligentMaterialState material,
                                        const DiligentLightingParameters& lighting,
                                        Diligent::ITextureView* normalMap) {
    if (&texture.owningDevice() != impl_->device.RawPtr()) {
        throw std::invalid_argument("Diligent texture array belongs to a different device");
    }
    drawImpl(context, mesh, texture.shaderResourceView(), true, modelViewProjection, material,
             lighting, nullptr, normalMap);
}

void DiligentTexturedMeshRenderer::drawMotion(
    Diligent::IDeviceContext& context, const DiligentTexturedMesh& mesh,
    const DiligentTexture2DArray& texture, const Matrix4& modelViewProjection,
    const DiligentMotionParameters& motion, DiligentMaterialState material,
    const DiligentLightingParameters& lighting, Diligent::ITextureView* normalMap) {
    if (&texture.owningDevice() != impl_->device.RawPtr()) {
        throw std::invalid_argument("Diligent texture array belongs to a different device");
    }
    drawImpl(context, mesh, texture.shaderResourceView(), true, modelViewProjection, material,
             lighting, &motion, normalMap);
}

} // namespace openbus::rendering
