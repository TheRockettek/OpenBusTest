#include "app/ImGuiLayer.h"

#include <GLFW/glfw3.h>
#include <imgui.h>

#include <iostream>

int main() {
    if (!glfwInit()) {
        return 77;
    }
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    GLFWwindow* window = glfwCreateWindow(64, 64, "ImGui cursor probe", nullptr, nullptr);
    if (window == nullptr) {
        glfwTerminate();
        return 77;
    }
    glfwMakeContextCurrent(window);

    bool passed = true;
    {
        ImGuiLayer layer(window);
        ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
        for (int frame = 0; frame < 4; ++frame) {
            glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_HIDDEN);
            layer.beginFrame(false);
            if ((ImGui::GetIO().ConfigFlags & ImGuiConfigFlags_NoMouseCursorChange) == 0 ||
                glfwGetInputMode(window, GLFW_CURSOR) != GLFW_CURSOR_HIDDEN) {
                std::cerr << "ImGui changed the game-owned cursor\n";
                passed = false;
            }
            layer.render();
        }

        layer.beginFrame();
        if ((ImGui::GetIO().ConfigFlags & ImGuiConfigFlags_NoMouseCursorChange) != 0 ||
            glfwGetInputMode(window, GLFW_CURSOR) != GLFW_CURSOR_NORMAL) {
            std::cerr << "ImGui did not regain cursor ownership for the UI\n";
            passed = false;
        }
        layer.render();

        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_HIDDEN);
        layer.beginFrame(false);
        if (glfwGetInputMode(window, GLFW_CURSOR) != GLFW_CURSOR_HIDDEN ||
            (ImGui::GetIO().ConfigFlags & ImGuiConfigFlags_NavEnableKeyboard) == 0) {
            std::cerr << "Cursor handoff changed cursor mode or unrelated configuration\n";
            passed = false;
        }
        layer.render();
    }

    glfwDestroyWindow(window);
    glfwTerminate();
    return passed ? 0 : 1;
}
