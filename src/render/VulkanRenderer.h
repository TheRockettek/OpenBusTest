#pragma once

#include <memory>
#include <string_view>

struct GLFWwindow;

namespace openbus::rendering {

class VulkanRenderer {
  public:
    explicit VulkanRenderer(GLFWwindow* window);
    ~VulkanRenderer();

    VulkanRenderer(const VulkanRenderer&) = delete;
    VulkanRenderer& operator=(const VulkanRenderer&) = delete;
    VulkanRenderer(VulkanRenderer&&) = delete;
    VulkanRenderer& operator=(VulkanRenderer&&) = delete;

    void drawClearFrame();
    [[nodiscard]] std::string_view physicalDeviceName() const noexcept;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace openbus::rendering
