#pragma once

struct GLFWwindow;
struct ImGuiContext;

class ImGuiLayer {
  public:
    explicit ImGuiLayer(GLFWwindow* window);
    ~ImGuiLayer();

    ImGuiLayer(const ImGuiLayer&) = delete;
    ImGuiLayer& operator=(const ImGuiLayer&) = delete;

    void beginFrame(bool manageMouseCursor = true);
    void render();

  private:
    ImGuiContext* context_ = nullptr;
};
