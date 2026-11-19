#include "render/VulkanRenderer.h"

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <exception>
#include <iostream>

int main() {
    if (glfwInit() != GLFW_TRUE) {
        std::cerr << "GLFW initialization failed\n";
        return 1;
    }

    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    GLFWwindow* window = glfwCreateWindow(320, 240, "OpenBus Vulkan Probe", nullptr, nullptr);
    if (window == nullptr) {
        std::cerr << "GLFW could not create a Vulkan-compatible window\n";
        glfwTerminate();
        return 1;
    }

    int result = 0;
    try {
        {
            openbus::rendering::VulkanRenderer renderer(window);
            for (int frame = 0; frame < 3; ++frame) {
                glfwPollEvents();
                renderer.drawClearFrame();
            }
            std::cout << "Vulkan swapchain clear/present passed on "
                      << renderer.physicalDeviceName() << '\n';
        }
    } catch (const std::exception& error) {
        std::cerr << "Vulkan backend probe failed: " << error.what() << '\n';
        result = 1;
    }

    glfwDestroyWindow(window);
    glfwTerminate();
    return result;
}
