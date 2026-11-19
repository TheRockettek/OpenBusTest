#define GLFW_INCLUDE_NONE
#define GLFW_EXPOSE_NATIVE_WIN32
#include <GLFW/glfw3.h>
#include <GLFW/glfw3native.h>
#include <vulkan/vulkan.h>

#include <Common/interface/RefCntAutoPtr.hpp>
#include <Graphics/GraphicsEngine/interface/PipelineState.h>
#include <Graphics/GraphicsEngine/interface/RenderDevice.h>
#include <Graphics/GraphicsEngine/interface/Shader.h>
#include <Graphics/GraphicsEngine/interface/SwapChain.h>
#include <Graphics/GraphicsEngineVulkan/interface/CommandQueueVk.h>
#include <Graphics/GraphicsEngineVulkan/interface/DeviceContextVk.h>
#include <Graphics/GraphicsEngineVulkan/interface/EngineFactoryVk.h>
#include <Graphics/GraphicsEngineVulkan/interface/RenderDeviceVk.h>
#include <Graphics/GraphicsEngineVulkan/interface/TextureVk.h>
#include <imgui.h>

#include "app/DiligentImGuiLayer.h"
#include "render/diligent/DiligentVulkanFrame.h"
#include "render/diligent/DiligentColorPresent.h"
#include "render/diligent/DiligentMapTileAsset.h"
#include "render/diligent/DiligentSceneFrame.h"
#include "render/diligent/DiligentMotionRenderTargets.h"
#include "render/diligent/DiligentTexture2D.h"
#include "DiligentPrimitiveRendering.h"
#include "DiligentColorPresent.h"
#include "DiligentImGuiRendering.h"
#include "DiligentMotionDataRendering.h"
#include "DiligentTexturedMeshRendering.h"

#include <atomic>
#include <exception>
#include <filesystem>
#include <iostream>
#include <memory>
#include <stdexcept>

namespace {

std::atomic<unsigned int> diligentErrors{0};

void DILIGENT_CALL_TYPE reportDiligentMessage(Diligent::DEBUG_MESSAGE_SEVERITY severity,
                                              const Diligent::Char* message,
                                              const Diligent::Char* function,
                                              const Diligent::Char* file, int line) {
    if (severity >= Diligent::DEBUG_MESSAGE_SEVERITY_ERROR) {
        ++diligentErrors;
    }
    if (severity >= Diligent::DEBUG_MESSAGE_SEVERITY_WARNING) {
        std::cerr << "Diligent "
                  << (severity >= Diligent::DEBUG_MESSAGE_SEVERITY_ERROR ? "error" : "warning")
                  << ": " << (message != nullptr ? message : "(no message)");
        if (function != nullptr) {
            std::cerr << " [" << function;
            if (file != nullptr) {
                std::cerr << ", " << file << ':' << line;
            }
            std::cerr << ']';
        }
        std::cerr << '\n';
    }
}

constexpr char kVertexShader[] = R"(
struct VertexOutput
{
    float4 Position : SV_POSITION;
    float3 Color : COLOR0;
};

void main(uint VertexId : SV_VertexID, out VertexOutput Output)
{
    float2 Positions[3] = {
        float2(-0.65, -0.55),
        float2( 0.00,  0.65),
        float2( 0.65, -0.55)
    };
    float3 Colors[3] = {
        float3(1.0, 0.1, 0.1),
        float3(0.1, 1.0, 0.1),
        float3(0.1, 0.2, 1.0)
    };

    Output.Position = float4(Positions[VertexId], 0.0, 1.0);
    Output.Color = Colors[VertexId];
}
)";

constexpr char kPixelShader[] = R"(
struct PixelInput
{
    float4 Position : SV_POSITION;
    float3 Color : COLOR0;
};

float4 main(PixelInput Input) : SV_TARGET
{
    return float4(Input.Color, 1.0);
}
)";

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

Diligent::RefCntAutoPtr<Diligent::IShader> createShader(Diligent::IRenderDevice* device,
                                                        Diligent::SHADER_TYPE type,
                                                        const char* name, const char* source) {
    Diligent::ShaderCreateInfo createInfo;
    createInfo.SourceLanguage = Diligent::SHADER_SOURCE_LANGUAGE_HLSL;
    createInfo.Desc.ShaderType = type;
    createInfo.Desc.Name = name;
    createInfo.EntryPoint = "main";
    createInfo.Source = source;

    Diligent::RefCntAutoPtr<Diligent::IShader> shader;
    device->CreateShader(createInfo, &shader);
    require(static_cast<bool>(shader), "Diligent failed to compile a probe shader.");
    return shader;
}

Diligent::RefCntAutoPtr<Diligent::IPipelineState>
createPipeline(Diligent::IRenderDevice* device, Diligent::TEXTURE_FORMAT colorFormat) {
    auto vertexShader =
        createShader(device, Diligent::SHADER_TYPE_VERTEX, "Probe triangle VS", kVertexShader);
    auto pixelShader =
        createShader(device, Diligent::SHADER_TYPE_PIXEL, "Probe triangle PS", kPixelShader);

    Diligent::GraphicsPipelineStateCreateInfo createInfo;
    createInfo.PSODesc.Name = "Diligent Vulkan interop probe";
    createInfo.PSODesc.PipelineType = Diligent::PIPELINE_TYPE_GRAPHICS;
    createInfo.GraphicsPipeline.NumRenderTargets = 1;
    createInfo.GraphicsPipeline.RTVFormats[0] = colorFormat;
    createInfo.GraphicsPipeline.PrimitiveTopology = Diligent::PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
    createInfo.GraphicsPipeline.RasterizerDesc.CullMode = Diligent::CULL_MODE_NONE;
    createInfo.GraphicsPipeline.DepthStencilDesc.DepthEnable = Diligent::False;
    createInfo.pVS = vertexShader;
    createInfo.pPS = pixelShader;

    Diligent::RefCntAutoPtr<Diligent::IPipelineState> pipeline;
    device->CreateGraphicsPipelineState(createInfo, &pipeline);
    require(static_cast<bool>(pipeline), "Diligent failed to create the triangle pipeline.");
    return pipeline;
}

void drawTriangle(Diligent::IDeviceContext* context, Diligent::IPipelineState* pipeline,
                  Diligent::ITextureView* renderTarget) {
    context->SetRenderTargets(1, &renderTarget, nullptr,
                              Diligent::RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    const auto& desc = renderTarget->GetTexture()->GetDesc();
    const Diligent::Viewport viewport{0.0F, 0.0F, static_cast<float>(desc.Width),
                                      static_cast<float>(desc.Height), 0.0F, 1.0F};
    context->SetViewports(1, &viewport, desc.Width, desc.Height);
    const float clearColor[] = {0.04f, 0.04f, 0.06f, 1.0f};
    context->ClearRenderTarget(renderTarget, clearColor,
                               Diligent::RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context->SetPipelineState(pipeline);

    Diligent::DrawAttribs drawAttributes;
    drawAttributes.NumVertices = 3;
    context->Draw(drawAttributes);
}

void runProbe(GLFWwindow* window, const std::filesystem::path* meshPath) {
    using namespace Diligent;

    IEngineFactoryVk* factory = GetEngineFactoryVk();
    require(factory != nullptr, "Diligent Vulkan engine factory is unavailable.");
    factory->SetMessageCallback(reportDiligentMessage);

    NativeWindow nativeWindow;
    nativeWindow.hWnd = glfwGetWin32Window(window);
    SwapChainDesc swapChainDesc;
    swapChainDesc.Width = 320;
    swapChainDesc.Height = 240;
    swapChainDesc.ColorBufferFormat = TEX_FORMAT_RGBA8_UNORM;
    openbus::rendering::DiligentVulkanFrame frame(nativeWindow, swapChainDesc);
    openbus::rendering::RendererFramePresenter& presenter = frame;
    IRenderDevice& device = frame.device();
    IDeviceContext& context = frame.immediateContext();

    glfwSetWindowSize(window, 256, 192);
    glfwPollEvents();
    int resizedWidth = 0;
    int resizedHeight = 0;
    glfwGetFramebufferSize(window, &resizedWidth, &resizedHeight);
    require(resizedWidth > 0 && resizedHeight > 0,
            "GLFW reported an invalid framebuffer size after resizing");
    presenter.resize(static_cast<Uint32>(resizedWidth), static_cast<Uint32>(resizedHeight));
    require(frame.swapChain().GetDesc().Width == static_cast<Uint32>(resizedWidth) &&
                frame.swapChain().GetDesc().Height == static_cast<Uint32>(resizedHeight),
            "Diligent Vulkan frame did not resize its swap chain");
    glfwSetWindowSize(window, static_cast<int>(swapChainDesc.Width),
                      static_cast<int>(swapChainDesc.Height));
    glfwPollEvents();
    presenter.resize(swapChainDesc.Width, swapChainDesc.Height);
    presenter.resize(0, swapChainDesc.Height);
    require(presenter.isSuspended(), "Diligent Vulkan frame did not suspend at zero width");
    bool backBufferUnavailable = false;
    try {
        static_cast<void>(frame.backBufferRenderTarget());
    } catch (const std::logic_error&) {
        backBufferUnavailable = true;
    }
    require(backBufferUnavailable,
            "Diligent Vulkan frame exposed the back buffer while suspended");
    presenter.present();
    presenter.resize(swapChainDesc.Width, swapChainDesc.Height);
    require(!presenter.isSuspended(),
            "Diligent Vulkan frame did not resume after a nonzero resize");

    verifyDiligentPrimitiveRendering(device, context, meshPath);
    verifyDiligentTexturedMeshRendering(device, context);
    verifyDiligentMotionDataRendering(device, context);
    verifyDiligentColorPresent(device, context);
    verifyDiligentImGuiRendering(device, context);

    const TEXTURE_FORMAT colorFormat = frame.swapChain().GetDesc().ColorBufferFormat;
    TextureDesc textureDesc;
    textureDesc.Name = "Diligent POC offscreen color target";
    textureDesc.Type = RESOURCE_DIM_TEX_2D;
    textureDesc.Width = swapChainDesc.Width;
    textureDesc.Height = swapChainDesc.Height;
    textureDesc.MipLevels = 1;
    textureDesc.Format = colorFormat;
    textureDesc.BindFlags = BIND_RENDER_TARGET | BIND_SHADER_RESOURCE;

    ITexture* rawOffscreen = nullptr;
    device.CreateTexture(textureDesc, nullptr, &rawOffscreen);
    RefCntAutoPtr<ITexture> offscreen;
    offscreen.Attach(rawOffscreen);
    require(static_cast<bool>(offscreen), "Diligent failed to create the offscreen color target.");

    ITextureView* offscreenRtv = offscreen->GetDefaultView(TEXTURE_VIEW_RENDER_TARGET);
    require(offscreenRtv != nullptr, "Diligent failed to create the offscreen render-target view.");
    auto pipeline = createPipeline(&device, colorFormat);

    RefCntAutoPtr<IRenderDeviceVk> vkDevice{&device, IID_RenderDeviceVk};
    RefCntAutoPtr<IDeviceContextVk> vkContext{&context, IID_DeviceContextVk};
    RefCntAutoPtr<ITextureVk> vkOffscreen{offscreen.RawPtr(), IID_TextureVk};
    require(vkDevice && vkContext && vkOffscreen,
            "Diligent Vulkan native interfaces are unavailable.");

    const VkDevice nativeDevice = vkDevice->GetVkDevice();
    const VkCommandBuffer nativeCommandBuffer = vkContext->GetVkCommandBuffer();
    const VkImage nativeImage = vkOffscreen->GetVkImage();
    require(nativeDevice != VK_NULL_HANDLE, "Diligent returned a null Vulkan device.");
    require(nativeCommandBuffer != VK_NULL_HANDLE,
            "Diligent returned a null Vulkan command buffer.");
    require(nativeImage != VK_NULL_HANDLE, "Diligent returned a null Vulkan image.");

    VkQueue nativeQueue = VK_NULL_HANDLE;
    ICommandQueue* commandQueue = context.LockCommandQueue();
    if (commandQueue != nullptr) {
        {
            RefCntAutoPtr<ICommandQueueVk> vkQueue{commandQueue, IID_CommandQueueVk};
            if (vkQueue) {
                nativeQueue = vkQueue->GetVkQueue();
            }
        }
        context.UnlockCommandQueue();
    }
    require(nativeQueue != VK_NULL_HANDLE, "Diligent returned a null Vulkan queue.");

    drawTriangle(&context, pipeline, offscreenRtv);
    vkContext->TransitionImageLayout(offscreen, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    require(vkOffscreen->GetLayout() == VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
            "Diligent did not track the offscreen Vulkan image layout handoff.");

    openbus::rendering::DiligentColorPresent presentColor(device, colorFormat);
    auto* offscreenSrv = offscreen->GetDefaultView(TEXTURE_VIEW_SHADER_RESOURCE);
    require(offscreenSrv != nullptr, "Diligent offscreen shader-resource view is missing.");
    presentColor.draw(context, *offscreenSrv, frame.backBufferRenderTarget());
    ImGuiContext* previousGuiContext = ImGui::GetCurrentContext();
    bool invalidGuiFormatRejected = false;
    try {
        DiligentImGuiLayer invalidGui(*window, device, TEX_FORMAT_UNKNOWN);
    } catch (const std::invalid_argument&) {
        invalidGuiFormatRejected = true;
    }
    require(invalidGuiFormatRejected && ImGui::GetCurrentContext() == previousGuiContext,
            "Diligent ImGui initialization did not clean up after failure");
    DiligentImGuiLayer gui(*window, device, colorFormat);
    gui.rebuildFontTexture();
    openbus::rendering::    DiligentTexture2D guiTexture(
        device, openbus::rendering::Image{1, 1, {255, 255, 255, 255}});
    const ImTextureID guiTextureId = gui.registerTexture(guiTexture.shaderResourceView());

    constexpr openbus::rendering::Matrix4 identityMatrix = {
        1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    };
    openbus::rendering::DiligentSceneFrame sceneFrame(
        device, swapChainDesc.Width / 2, swapChainDesc.Height / 2, colorFormat, true);
    openbus::rendering::MotionHistory motionHistory;
    openbus::map::MapTerrainMesh preparedTerrain;
    preparedTerrain.vertices = {
        {-0.65F, -0.55F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 1.0F},
        {0.65F, -0.55F, 0.0F, 1.0F, 1.0F, 0.0F, 0.0F, 0.0F, 1.0F},
        {0.0F, 0.65F, 0.0F, 0.5F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F},
    };
    preparedTerrain.indices = {0, 1, 2};
    auto preparedTerrainTexture =
        std::make_shared<openbus::rendering::Image>(
            openbus::rendering::Image{1, 1, {220, 180, 120, 255}});
    openbus::rendering::DiligentMapTileAsset mapTile(
        device, 0, 0, &preparedTerrain, preparedTerrainTexture, nullptr, {}, {});
    const auto renderFrame = [&](std::uint64_t frameId, double timestampSeconds,
                                 Uint32 renderWidth, Uint32 renderHeight,
                                 openbus::rendering::DiligentColorFilter filter,
                                 bool expectedHistoryValidity,
                                 openbus::rendering::DiligentDebugView debugView) {
        motionHistory.beginFrame(frameId, timestampSeconds, static_cast<int>(renderWidth),
                                 static_cast<int>(renderHeight), identityMatrix, identityMatrix,
                                 static_cast<int>(swapChainDesc.Width),
                                 static_cast<int>(swapChainDesc.Height));
        sceneFrame.beginScene(context, motionHistory.frame());
        require(sceneFrame.frameData().historyValid == expectedHistoryValidity,
                "Diligent scene frame did not preserve the expected temporal history state");
        require(mapTile.terrain() != nullptr &&
                    mapTile.terrain()->draw(sceneFrame.meshes(), context,
                                            guiTexture.shaderResourceView(), identityMatrix),
                "Diligent prepared map terrain did not submit through the scene frame");
        sceneFrame.finishScene(context, frame.backBufferRenderTarget(), filter, debugView);
        gui.beginFrame(false);
        require((ImGui::GetIO().ConfigFlags & ImGuiConfigFlags_NoMouseCursorChange) != 0,
                "Diligent ImGui layer did not preserve cursor policy");
        ImGui::GetForegroundDrawList()->AddText({8.0F, 8.0F}, IM_COL32_WHITE, "Diligent UI");
        ImGui::GetForegroundDrawList()->AddImage(guiTextureId, {8.0F, 24.0F}, {16.0F, 32.0F});
        gui.render(context, frame.backBufferRenderTarget());
        frame.present();
        sceneFrame.completeFrame();
        motionHistory.completeFrame();
    };

    frame.setVerticalSync(false);
    renderFrame(1, 1.0, swapChainDesc.Width / 2, swapChainDesc.Height / 2,
                openbus::rendering::DiligentColorFilter::Smaa, false,
                openbus::rendering::DiligentDebugView::None);
    renderFrame(2, 1.016, swapChainDesc.Width / 2, swapChainDesc.Height / 2,
                openbus::rendering::DiligentColorFilter::None, true,
                openbus::rendering::DiligentDebugView::None);
    sceneFrame.resize(context, swapChainDesc.Width / 3, swapChainDesc.Height / 3);
    renderFrame(3, 1.032, swapChainDesc.Width / 3, swapChainDesc.Height / 3,
                openbus::rendering::DiligentColorFilter::Fxaa, false,
                openbus::rendering::DiligentDebugView::None);
    renderFrame(4, 1.048, swapChainDesc.Width / 3, swapChainDesc.Height / 3,
                openbus::rendering::DiligentColorFilter::None, true,
                openbus::rendering::DiligentDebugView::MotionValidity);
    renderFrame(5, 1.064, swapChainDesc.Width / 3, swapChainDesc.Height / 3,
                openbus::rendering::DiligentColorFilter::None, true,
                openbus::rendering::DiligentDebugView::LinearDepth);
    frame.waitForIdle();
    gui.unregisterTexture(guiTextureId);

    std::cout
        << "Diligent Vulkan primitive, alpha-mode, indexed and non-indexed textured mesh readback,"
           " motion-vector output"
        << (meshPath != nullptr ? " and O3D geometry/UVs" : "")
        << ", generated road, prepared map-tile submission, triangle, offscreen target, native "
           "device/queue/command-buffer/image handles, "
           "layout handoff, scene history, scaled SMAA/FXAA composition, motion and linear-depth "
           "debug visualization, UI overlay, and multi-frame presentation passed.\n";
}

} // namespace

int main(int argc, char** argv) {
    if (argc > 2) {
        std::cerr << "Usage: OpenBusDiligentVulkanProbe [O3D fixture path]\n";
        return 1;
    }
    const std::filesystem::path meshPath =
        argc == 2 ? std::filesystem::path(argv[1]) : std::filesystem::path{};
    if (glfwInit() != GLFW_TRUE || glfwVulkanSupported() != GLFW_TRUE) {
        std::cerr << "GLFW could not initialize Vulkan for the Diligent probe.\n";
        glfwTerminate();
        return 77;
    }

    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    GLFWwindow* window = glfwCreateWindow(320, 240, "Diligent Vulkan Probe", nullptr, nullptr);
    if (window == nullptr) {
        std::cerr << "GLFW could not create a Vulkan-compatible window.\n";
        glfwTerminate();
        return 77;
    }

    int result = 0;
    try {
        runProbe(window, argc == 2 ? &meshPath : nullptr);
        if (diligentErrors.load() != 0) {
            std::cerr << "Diligent Vulkan probe reported engine errors.\n";
            result = 1;
        }
    } catch (const std::exception& error) {
        std::cerr << "Diligent Vulkan probe failed: " << error.what() << '\n';
        result = 1;
    }

    glfwDestroyWindow(window);
    glfwTerminate();
    return result;
}
