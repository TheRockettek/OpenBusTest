#include "render/VulkanRenderer.h"

#include <volk.h>
#include <GLFW/glfw3.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace openbus::rendering {
namespace {

constexpr char PORTABILITY_SUBSET_EXTENSION[] = "VK_KHR_portability_subset";

void checkVk(VkResult result, const char* operation) {
    if (result != VK_SUCCESS) {
        throw std::runtime_error(std::string(operation) + " failed with VkResult " +
                                 std::to_string(static_cast<int>(result)));
    }
}

struct QueueFamilies {
    std::optional<std::uint32_t> graphics;
    std::optional<std::uint32_t> present;

    [[nodiscard]] bool complete() const noexcept {
        return graphics.has_value() && present.has_value();
    }
};

bool hasDeviceExtension(VkPhysicalDevice device, const char* requestedExtension) {
    std::uint32_t count = 0;
    checkVk(vkEnumerateDeviceExtensionProperties(device, nullptr, &count, nullptr),
            "vkEnumerateDeviceExtensionProperties");
    std::vector<VkExtensionProperties> extensions(count);
    checkVk(vkEnumerateDeviceExtensionProperties(device, nullptr, &count, extensions.data()),
            "vkEnumerateDeviceExtensionProperties");
    return std::any_of(extensions.begin(), extensions.end(),
                       [requestedExtension](const auto& extension) {
                           return std::string_view(extension.extensionName) == requestedExtension;
                       });
}

bool hasInstanceExtension(const char* requestedExtension) {
    std::uint32_t count = 0;
    checkVk(vkEnumerateInstanceExtensionProperties(nullptr, &count, nullptr),
            "vkEnumerateInstanceExtensionProperties");
    std::vector<VkExtensionProperties> extensions(count);
    checkVk(vkEnumerateInstanceExtensionProperties(nullptr, &count, extensions.data()),
            "vkEnumerateInstanceExtensionProperties");
    return std::any_of(extensions.begin(), extensions.end(),
                       [requestedExtension](const auto& extension) {
                           return std::string_view(extension.extensionName) == requestedExtension;
                       });
}

} // namespace

struct VulkanRenderer::Impl {
    explicit Impl(GLFWwindow* requestedWindow) : window(requestedWindow) {
        if (window == nullptr) {
            throw std::invalid_argument("VulkanRenderer requires a GLFW window");
        }
        try {
            initialize();
        } catch (...) {
            destroy();
            throw;
        }
    }

    ~Impl() {
        destroy();
    }

    void initialize() {
        checkVk(volkInitialize(), "volkInitialize");
        volkInitialized = true;
        if (glfwVulkanSupported() != GLFW_TRUE) {
            throw std::runtime_error("GLFW reports that Vulkan is unavailable");
        }
        createInstance();
        checkVk(glfwCreateWindowSurface(instance, window, nullptr, &surface),
                "glfwCreateWindowSurface");
        choosePhysicalDevice();
        createDevice();
        createSwapchain();
        createFrameSynchronization();
    }

    void createInstance() {
        std::uint32_t extensionCount = 0;
        const char** extensions = glfwGetRequiredInstanceExtensions(&extensionCount);
        if (extensions == nullptr || extensionCount == 0) {
            throw std::runtime_error("GLFW did not provide Vulkan surface extensions");
        }

        VkApplicationInfo applicationInfo{};
        applicationInfo.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
        applicationInfo.pApplicationName = "OpenBus Vulkan Probe";
        applicationInfo.applicationVersion = VK_MAKE_API_VERSION(0, 0, 1, 0);
        applicationInfo.pEngineName = "OpenBus";
        applicationInfo.engineVersion = VK_MAKE_API_VERSION(0, 0, 1, 0);
        applicationInfo.apiVersion = VK_API_VERSION_1_2;

        VkInstanceCreateInfo createInfo{};
        createInfo.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
        createInfo.pApplicationInfo = &applicationInfo;
        std::vector<const char*> enabledExtensions(extensions, extensions + extensionCount);
        if (hasInstanceExtension(VK_KHR_PORTABILITY_ENUMERATION_EXTENSION_NAME)) {
            const bool alreadyEnabled = std::any_of(
                enabledExtensions.begin(), enabledExtensions.end(), [](const char* extension) {
                    return std::string_view(extension) ==
                           VK_KHR_PORTABILITY_ENUMERATION_EXTENSION_NAME;
                });
            if (!alreadyEnabled) {
                enabledExtensions.push_back(VK_KHR_PORTABILITY_ENUMERATION_EXTENSION_NAME);
            }
            createInfo.flags |= VK_INSTANCE_CREATE_ENUMERATE_PORTABILITY_BIT_KHR;
        }
        createInfo.enabledExtensionCount = static_cast<std::uint32_t>(enabledExtensions.size());
        createInfo.ppEnabledExtensionNames = enabledExtensions.data();
        checkVk(vkCreateInstance(&createInfo, nullptr, &instance), "vkCreateInstance");
        volkLoadInstance(instance);
    }

    QueueFamilies findQueueFamilies(VkPhysicalDevice candidate) const {
        std::uint32_t count = 0;
        vkGetPhysicalDeviceQueueFamilyProperties(candidate, &count, nullptr);
        std::vector<VkQueueFamilyProperties> families(count);
        vkGetPhysicalDeviceQueueFamilyProperties(candidate, &count, families.data());

        QueueFamilies result;
        for (std::uint32_t index = 0; index < count; ++index) {
            if (families[index].queueCount > 0 &&
                (families[index].queueFlags & VK_QUEUE_GRAPHICS_BIT) != 0) {
                result.graphics = index;
            }
            VkBool32 supportsPresent = VK_FALSE;
            checkVk(
                vkGetPhysicalDeviceSurfaceSupportKHR(candidate, index, surface, &supportsPresent),
                "vkGetPhysicalDeviceSurfaceSupportKHR");
            if (families[index].queueCount > 0 && supportsPresent == VK_TRUE) {
                result.present = index;
            }
            if (result.complete()) {
                break;
            }
        }
        return result;
    }

    bool supportsSwapchain(VkPhysicalDevice candidate) const {
        if (!hasDeviceExtension(candidate, VK_KHR_SWAPCHAIN_EXTENSION_NAME)) {
            return false;
        }
        std::uint32_t formatCount = 0;
        std::uint32_t modeCount = 0;
        checkVk(vkGetPhysicalDeviceSurfaceFormatsKHR(candidate, surface, &formatCount, nullptr),
                "vkGetPhysicalDeviceSurfaceFormatsKHR");
        checkVk(vkGetPhysicalDeviceSurfacePresentModesKHR(candidate, surface, &modeCount, nullptr),
                "vkGetPhysicalDeviceSurfacePresentModesKHR");
        return formatCount > 0 && modeCount > 0;
    }

    void choosePhysicalDevice() {
        std::uint32_t count = 0;
        checkVk(vkEnumeratePhysicalDevices(instance, &count, nullptr),
                "vkEnumeratePhysicalDevices");
        if (count == 0) {
            throw std::runtime_error("No Vulkan physical devices are available");
        }
        std::vector<VkPhysicalDevice> devices(count);
        checkVk(vkEnumeratePhysicalDevices(instance, &count, devices.data()),
                "vkEnumeratePhysicalDevices");

        for (VkPhysicalDevice candidate : devices) {
            VkPhysicalDeviceProperties properties{};
            vkGetPhysicalDeviceProperties(candidate, &properties);
            if (properties.apiVersion < VK_API_VERSION_1_2) {
                continue;
            }
            const QueueFamilies families = findQueueFamilies(candidate);
            if (!families.complete() || !supportsSwapchain(candidate)) {
                continue;
            }
            physicalDevice = candidate;
            queueFamilies = families;
            deviceName = properties.deviceName;
            deviceExtensions = {VK_KHR_SWAPCHAIN_EXTENSION_NAME};
            if (hasDeviceExtension(candidate, PORTABILITY_SUBSET_EXTENSION)) {
                deviceExtensions.push_back(PORTABILITY_SUBSET_EXTENSION);
            }
            break;
        }
        if (physicalDevice == VK_NULL_HANDLE) {
            throw std::runtime_error(
                "No Vulkan 1.2 device supports graphics, presentation, and swapchains");
        }
    }

    void createDevice() {
        const std::array uniqueFamilies{*queueFamilies.graphics, *queueFamilies.present};
        std::vector<std::uint32_t> queueFamilyIndices;
        for (std::uint32_t index : uniqueFamilies) {
            if (std::find(queueFamilyIndices.begin(), queueFamilyIndices.end(), index) ==
                queueFamilyIndices.end()) {
                queueFamilyIndices.push_back(index);
            }
        }

        constexpr float queuePriority = 1.0F;
        std::vector<VkDeviceQueueCreateInfo> queueCreateInfos;
        queueCreateInfos.reserve(queueFamilyIndices.size());
        for (std::uint32_t family : queueFamilyIndices) {
            VkDeviceQueueCreateInfo queueInfo{};
            queueInfo.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
            queueInfo.queueFamilyIndex = family;
            queueInfo.queueCount = 1;
            queueInfo.pQueuePriorities = &queuePriority;
            queueCreateInfos.push_back(queueInfo);
        }

        VkPhysicalDeviceFeatures supportedFeatures{};
        VkDeviceCreateInfo createInfo{};
        createInfo.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
        createInfo.queueCreateInfoCount = static_cast<std::uint32_t>(queueCreateInfos.size());
        createInfo.pQueueCreateInfos = queueCreateInfos.data();
        createInfo.pEnabledFeatures = &supportedFeatures;
        createInfo.enabledExtensionCount = static_cast<std::uint32_t>(deviceExtensions.size());
        createInfo.ppEnabledExtensionNames = deviceExtensions.data();
        checkVk(vkCreateDevice(physicalDevice, &createInfo, nullptr, &device), "vkCreateDevice");
        volkLoadDevice(device);
        vkGetDeviceQueue(device, *queueFamilies.graphics, 0, &graphicsQueue);
        vkGetDeviceQueue(device, *queueFamilies.present, 0, &presentQueue);
    }

    VkSurfaceFormatKHR chooseSurfaceFormat(const std::vector<VkSurfaceFormatKHR>& formats) const {
        constexpr VkFormat preferredFormat = VK_FORMAT_B8G8R8A8_SRGB;
        if (formats.size() == 1 && formats.front().format == VK_FORMAT_UNDEFINED) {
            return {VK_FORMAT_B8G8R8A8_UNORM, formats.front().colorSpace};
        }
        for (const auto& format : formats) {
            if (format.format == preferredFormat &&
                format.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR) {
                return format;
            }
        }
        return formats.front();
    }

    VkCompositeAlphaFlagBitsKHR chooseCompositeAlpha(VkCompositeAlphaFlagsKHR supported) const {
        constexpr std::array choices{
            VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR, VK_COMPOSITE_ALPHA_PRE_MULTIPLIED_BIT_KHR,
            VK_COMPOSITE_ALPHA_POST_MULTIPLIED_BIT_KHR, VK_COMPOSITE_ALPHA_INHERIT_BIT_KHR};
        for (VkCompositeAlphaFlagBitsKHR choice : choices) {
            if ((supported & choice) != 0) {
                return choice;
            }
        }
        throw std::runtime_error("Vulkan surface has no supported composite alpha mode");
    }

    void createSwapchain() {
        VkSurfaceCapabilitiesKHR capabilities{};
        checkVk(vkGetPhysicalDeviceSurfaceCapabilitiesKHR(physicalDevice, surface, &capabilities),
                "vkGetPhysicalDeviceSurfaceCapabilitiesKHR");
        if ((capabilities.supportedUsageFlags & VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT) == 0) {
            throw std::runtime_error(
                "Vulkan surface does not support color-attachment swapchain images");
        }

        std::uint32_t formatCount = 0;
        checkVk(
            vkGetPhysicalDeviceSurfaceFormatsKHR(physicalDevice, surface, &formatCount, nullptr),
            "vkGetPhysicalDeviceSurfaceFormatsKHR");
        std::vector<VkSurfaceFormatKHR> formats(formatCount);
        checkVk(vkGetPhysicalDeviceSurfaceFormatsKHR(physicalDevice, surface, &formatCount,
                                                     formats.data()),
                "vkGetPhysicalDeviceSurfaceFormatsKHR");
        if (formats.empty()) {
            throw std::runtime_error("Vulkan surface returned no swapchain formats");
        }
        const VkSurfaceFormatKHR surfaceFormat = chooseSurfaceFormat(formats);

        int framebufferWidth = 0;
        int framebufferHeight = 0;
        glfwGetFramebufferSize(window, &framebufferWidth, &framebufferHeight);
        if (capabilities.currentExtent.width != std::numeric_limits<std::uint32_t>::max()) {
            extent = capabilities.currentExtent;
        } else {
            extent.width =
                std::clamp(static_cast<std::uint32_t>(std::max(framebufferWidth, 1)),
                           capabilities.minImageExtent.width, capabilities.maxImageExtent.width);
            extent.height =
                std::clamp(static_cast<std::uint32_t>(std::max(framebufferHeight, 1)),
                           capabilities.minImageExtent.height, capabilities.maxImageExtent.height);
        }

        std::uint32_t imageCount = capabilities.minImageCount + 1;
        if (capabilities.maxImageCount > 0 && imageCount > capabilities.maxImageCount) {
            imageCount = capabilities.maxImageCount;
        }
        const std::array queueIndices{*queueFamilies.graphics, *queueFamilies.present};
        VkSwapchainCreateInfoKHR createInfo{};
        createInfo.sType = VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR;
        createInfo.surface = surface;
        createInfo.minImageCount = imageCount;
        createInfo.imageFormat = surfaceFormat.format;
        createInfo.imageColorSpace = surfaceFormat.colorSpace;
        createInfo.imageExtent = extent;
        createInfo.imageArrayLayers = 1;
        createInfo.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
        if (queueFamilies.graphics != queueFamilies.present) {
            createInfo.imageSharingMode = VK_SHARING_MODE_CONCURRENT;
            createInfo.queueFamilyIndexCount = static_cast<std::uint32_t>(queueIndices.size());
            createInfo.pQueueFamilyIndices = queueIndices.data();
        } else {
            createInfo.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
        }
        createInfo.preTransform = capabilities.currentTransform;
        createInfo.compositeAlpha = chooseCompositeAlpha(capabilities.supportedCompositeAlpha);
        createInfo.presentMode = VK_PRESENT_MODE_FIFO_KHR;
        createInfo.clipped = VK_TRUE;
        checkVk(vkCreateSwapchainKHR(device, &createInfo, nullptr, &swapchain),
                "vkCreateSwapchainKHR");

        checkVk(vkGetSwapchainImagesKHR(device, swapchain, &imageCount, nullptr),
                "vkGetSwapchainImagesKHR");
        swapchainImages.resize(imageCount);
        checkVk(vkGetSwapchainImagesKHR(device, swapchain, &imageCount, swapchainImages.data()),
                "vkGetSwapchainImagesKHR");
        swapchainFormat = surfaceFormat.format;
        createImageViews();
        createRenderPass();
        createFramebuffers();
        createCommandResources();
        createRenderFinishedSemaphores();
    }

    void createImageViews() {
        imageViews.resize(swapchainImages.size(), VK_NULL_HANDLE);
        for (std::size_t index = 0; index < swapchainImages.size(); ++index) {
            VkImageViewCreateInfo createInfo{};
            createInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
            createInfo.image = swapchainImages[index];
            createInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
            createInfo.format = swapchainFormat;
            createInfo.components = {VK_COMPONENT_SWIZZLE_IDENTITY, VK_COMPONENT_SWIZZLE_IDENTITY,
                                     VK_COMPONENT_SWIZZLE_IDENTITY, VK_COMPONENT_SWIZZLE_IDENTITY};
            createInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
            createInfo.subresourceRange.baseMipLevel = 0;
            createInfo.subresourceRange.levelCount = 1;
            createInfo.subresourceRange.baseArrayLayer = 0;
            createInfo.subresourceRange.layerCount = 1;
            checkVk(vkCreateImageView(device, &createInfo, nullptr, &imageViews[index]),
                    "vkCreateImageView");
        }
    }

    void createRenderPass() {
        VkAttachmentDescription colorAttachment{};
        colorAttachment.format = swapchainFormat;
        colorAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
        colorAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        colorAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        colorAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        colorAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        colorAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        colorAttachment.finalLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;

        VkAttachmentReference colorReference{};
        colorReference.attachment = 0;
        colorReference.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

        VkSubpassDescription subpass{};
        subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
        subpass.colorAttachmentCount = 1;
        subpass.pColorAttachments = &colorReference;

        VkSubpassDependency dependency{};
        dependency.srcSubpass = VK_SUBPASS_EXTERNAL;
        dependency.dstSubpass = 0;
        dependency.srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
        dependency.dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
        dependency.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;

        VkRenderPassCreateInfo createInfo{};
        createInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
        createInfo.attachmentCount = 1;
        createInfo.pAttachments = &colorAttachment;
        createInfo.subpassCount = 1;
        createInfo.pSubpasses = &subpass;
        createInfo.dependencyCount = 1;
        createInfo.pDependencies = &dependency;
        checkVk(vkCreateRenderPass(device, &createInfo, nullptr, &renderPass),
                "vkCreateRenderPass");
    }

    void createFramebuffers() {
        framebuffers.resize(imageViews.size(), VK_NULL_HANDLE);
        for (std::size_t index = 0; index < imageViews.size(); ++index) {
            VkFramebufferCreateInfo createInfo{};
            createInfo.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
            createInfo.renderPass = renderPass;
            createInfo.attachmentCount = 1;
            createInfo.pAttachments = &imageViews[index];
            createInfo.width = extent.width;
            createInfo.height = extent.height;
            createInfo.layers = 1;
            checkVk(vkCreateFramebuffer(device, &createInfo, nullptr, &framebuffers[index]),
                    "vkCreateFramebuffer");
        }
    }

    void createCommandResources() {
        if (commandPool == VK_NULL_HANDLE) {
            VkCommandPoolCreateInfo poolInfo{};
            poolInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
            poolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
            poolInfo.queueFamilyIndex = *queueFamilies.graphics;
            checkVk(vkCreateCommandPool(device, &poolInfo, nullptr, &commandPool),
                    "vkCreateCommandPool");
        }
        commandBuffers.resize(framebuffers.size(), VK_NULL_HANDLE);
        VkCommandBufferAllocateInfo allocateInfo{};
        allocateInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
        allocateInfo.commandPool = commandPool;
        allocateInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        allocateInfo.commandBufferCount = static_cast<std::uint32_t>(commandBuffers.size());
        checkVk(vkAllocateCommandBuffers(device, &allocateInfo, commandBuffers.data()),
                "vkAllocateCommandBuffers");
    }

    void createFrameSynchronization() {
        VkSemaphoreCreateInfo semaphoreInfo{};
        semaphoreInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;
        checkVk(vkCreateSemaphore(device, &semaphoreInfo, nullptr, &imageAvailableSemaphore),
                "vkCreateSemaphore");
        VkFenceCreateInfo fenceInfo{};
        fenceInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
        fenceInfo.flags = VK_FENCE_CREATE_SIGNALED_BIT;
        checkVk(vkCreateFence(device, &fenceInfo, nullptr, &inFlightFence), "vkCreateFence");
    }

    void createRenderFinishedSemaphores() {
        renderFinishedSemaphores.resize(swapchainImages.size(), VK_NULL_HANDLE);
        VkSemaphoreCreateInfo createInfo{};
        createInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;
        for (VkSemaphore& semaphore : renderFinishedSemaphores) {
            checkVk(vkCreateSemaphore(device, &createInfo, nullptr, &semaphore),
                    "vkCreateSemaphore");
        }
    }

    void destroySwapchainResources() noexcept {
        if (device != VK_NULL_HANDLE) {
            for (VkSemaphore semaphore : renderFinishedSemaphores) {
                if (semaphore != VK_NULL_HANDLE) {
                    vkDestroySemaphore(device, semaphore, nullptr);
                }
            }
            renderFinishedSemaphores.clear();
            if (commandPool != VK_NULL_HANDLE) {
                vkDestroyCommandPool(device, commandPool, nullptr);
                commandPool = VK_NULL_HANDLE;
            }
            commandBuffers.clear();
            for (VkFramebuffer framebuffer : framebuffers) {
                if (framebuffer != VK_NULL_HANDLE) {
                    vkDestroyFramebuffer(device, framebuffer, nullptr);
                }
            }
            framebuffers.clear();
            if (renderPass != VK_NULL_HANDLE) {
                vkDestroyRenderPass(device, renderPass, nullptr);
                renderPass = VK_NULL_HANDLE;
            }
            for (VkImageView imageView : imageViews) {
                if (imageView != VK_NULL_HANDLE) {
                    vkDestroyImageView(device, imageView, nullptr);
                }
            }
            imageViews.clear();
            swapchainImages.clear();
            if (swapchain != VK_NULL_HANDLE) {
                vkDestroySwapchainKHR(device, swapchain, nullptr);
                swapchain = VK_NULL_HANDLE;
            }
        }
    }

    void recreateSwapchain() {
        int width = 0;
        int height = 0;
        glfwGetFramebufferSize(window, &width, &height);
        while (width == 0 || height == 0) {
            if (glfwWindowShouldClose(window) == GLFW_TRUE) {
                return;
            }
            glfwWaitEvents();
            glfwGetFramebufferSize(window, &width, &height);
        }
        checkVk(vkDeviceWaitIdle(device), "vkDeviceWaitIdle");
        destroySwapchainResources();
        createSwapchain();
    }

    void recordCommandBuffer(VkCommandBuffer commandBuffer, std::uint32_t imageIndex) {
        VkCommandBufferBeginInfo beginInfo{};
        beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
        checkVk(vkBeginCommandBuffer(commandBuffer, &beginInfo), "vkBeginCommandBuffer");

        VkClearValue clearValue{};
        clearValue.color = {{0.015F, 0.045F, 0.09F, 1.0F}};
        VkRenderPassBeginInfo renderPassInfo{};
        renderPassInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
        renderPassInfo.renderPass = renderPass;
        renderPassInfo.framebuffer = framebuffers[imageIndex];
        renderPassInfo.renderArea = {{0, 0}, extent};
        renderPassInfo.clearValueCount = 1;
        renderPassInfo.pClearValues = &clearValue;
        vkCmdBeginRenderPass(commandBuffer, &renderPassInfo, VK_SUBPASS_CONTENTS_INLINE);
        vkCmdEndRenderPass(commandBuffer);
        checkVk(vkEndCommandBuffer(commandBuffer), "vkEndCommandBuffer");
    }

    void drawClearFrame() {
        int width = 0;
        int height = 0;
        glfwGetFramebufferSize(window, &width, &height);
        if (width <= 0 || height <= 0) {
            recreateSwapchain();
            return;
        }
        if (static_cast<std::uint32_t>(width) != extent.width ||
            static_cast<std::uint32_t>(height) != extent.height) {
            recreateSwapchain();
        }
        if (glfwWindowShouldClose(window) == GLFW_TRUE) {
            return;
        }

        checkVk(vkWaitForFences(device, 1, &inFlightFence, VK_TRUE,
                                std::numeric_limits<std::uint64_t>::max()),
                "vkWaitForFences");
        std::uint32_t imageIndex = 0;
        const VkResult acquireResult =
            vkAcquireNextImageKHR(device, swapchain, std::numeric_limits<std::uint64_t>::max(),
                                  imageAvailableSemaphore, VK_NULL_HANDLE, &imageIndex);
        if (acquireResult == VK_ERROR_OUT_OF_DATE_KHR) {
            recreateSwapchain();
            return;
        }
        if (acquireResult != VK_SUCCESS && acquireResult != VK_SUBOPTIMAL_KHR) {
            checkVk(acquireResult, "vkAcquireNextImageKHR");
        }

        VkCommandBuffer commandBuffer = commandBuffers[imageIndex];
        checkVk(vkResetCommandBuffer(commandBuffer, 0), "vkResetCommandBuffer");
        recordCommandBuffer(commandBuffer, imageIndex);
        checkVk(vkResetFences(device, 1, &inFlightFence), "vkResetFences");

        constexpr VkPipelineStageFlags waitStage = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
        VkSubmitInfo submitInfo{};
        submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
        submitInfo.waitSemaphoreCount = 1;
        submitInfo.pWaitSemaphores = &imageAvailableSemaphore;
        submitInfo.pWaitDstStageMask = &waitStage;
        submitInfo.commandBufferCount = 1;
        submitInfo.pCommandBuffers = &commandBuffer;
        submitInfo.signalSemaphoreCount = 1;
        submitInfo.pSignalSemaphores = &renderFinishedSemaphores[imageIndex];
        checkVk(vkQueueSubmit(graphicsQueue, 1, &submitInfo, inFlightFence), "vkQueueSubmit");

        VkPresentInfoKHR presentInfo{};
        presentInfo.sType = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR;
        presentInfo.waitSemaphoreCount = 1;
        presentInfo.pWaitSemaphores = &renderFinishedSemaphores[imageIndex];
        presentInfo.swapchainCount = 1;
        presentInfo.pSwapchains = &swapchain;
        presentInfo.pImageIndices = &imageIndex;
        const VkResult presentResult = vkQueuePresentKHR(presentQueue, &presentInfo);
        if (presentResult != VK_SUCCESS && presentResult != VK_SUBOPTIMAL_KHR &&
            presentResult != VK_ERROR_OUT_OF_DATE_KHR) {
            checkVk(presentResult, "vkQueuePresentKHR");
        }
        if (acquireResult == VK_SUBOPTIMAL_KHR || presentResult == VK_SUBOPTIMAL_KHR ||
            presentResult == VK_ERROR_OUT_OF_DATE_KHR) {
            recreateSwapchain();
        }
    }

    void destroy() noexcept {
        if (device != VK_NULL_HANDLE) {
            vkDeviceWaitIdle(device);
            destroySwapchainResources();
            if (imageAvailableSemaphore != VK_NULL_HANDLE) {
                vkDestroySemaphore(device, imageAvailableSemaphore, nullptr);
                imageAvailableSemaphore = VK_NULL_HANDLE;
            }
            if (inFlightFence != VK_NULL_HANDLE) {
                vkDestroyFence(device, inFlightFence, nullptr);
                inFlightFence = VK_NULL_HANDLE;
            }
            vkDestroyDevice(device, nullptr);
            device = VK_NULL_HANDLE;
        }
        if (surface != VK_NULL_HANDLE && instance != VK_NULL_HANDLE) {
            vkDestroySurfaceKHR(instance, surface, nullptr);
            surface = VK_NULL_HANDLE;
        }
        if (instance != VK_NULL_HANDLE) {
            vkDestroyInstance(instance, nullptr);
            instance = VK_NULL_HANDLE;
        }
        if (volkInitialized) {
            volkFinalize();
            volkInitialized = false;
        }
    }

    GLFWwindow* window = nullptr;
    bool volkInitialized = false;
    VkInstance instance = VK_NULL_HANDLE;
    VkSurfaceKHR surface = VK_NULL_HANDLE;
    VkPhysicalDevice physicalDevice = VK_NULL_HANDLE;
    VkDevice device = VK_NULL_HANDLE;
    QueueFamilies queueFamilies;
    VkQueue graphicsQueue = VK_NULL_HANDLE;
    VkQueue presentQueue = VK_NULL_HANDLE;
    VkSwapchainKHR swapchain = VK_NULL_HANDLE;
    VkFormat swapchainFormat = VK_FORMAT_UNDEFINED;
    VkExtent2D extent{};
    std::string deviceName;
    std::vector<const char*> deviceExtensions;
    std::vector<VkImage> swapchainImages;
    std::vector<VkImageView> imageViews;
    VkRenderPass renderPass = VK_NULL_HANDLE;
    std::vector<VkFramebuffer> framebuffers;
    VkCommandPool commandPool = VK_NULL_HANDLE;
    std::vector<VkCommandBuffer> commandBuffers;
    VkSemaphore imageAvailableSemaphore = VK_NULL_HANDLE;
    std::vector<VkSemaphore> renderFinishedSemaphores;
    VkFence inFlightFence = VK_NULL_HANDLE;
};

VulkanRenderer::VulkanRenderer(GLFWwindow* window) : impl_(std::make_unique<Impl>(window)) {}

VulkanRenderer::~VulkanRenderer() = default;

void VulkanRenderer::drawClearFrame() {
    impl_->drawClearFrame();
}

std::string_view VulkanRenderer::physicalDeviceName() const noexcept {
    return impl_->deviceName;
}

} // namespace openbus::rendering
