#include "app/CustomWeatherUi.h"
#include "core/Variables.h"
#include "physics/RoadWeather.h"

#include <imgui.h>

#include <array>
#include <cmath>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <utility>

int main() {
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.IniFilename = nullptr;
    io.DisplaySize = ImVec2(800.0F, 600.0F);
    io.DeltaTime = 1.0F / 60.0F;
    unsigned char* pixels = nullptr;
    int width = 0;
    int height = 0;
    io.Fonts->GetTexDataAsRGBA32(&pixels, &width, &height);

    SystemVariables system;
    auto weather = system.weather();
    weather.precipitationType = 2;
    weather.precipitationRate = 1.0;
    weather.temperatureCelsius = -5.0;
    weather.omsiGroundWetness = std::array<double, 3>{39.5, 0.0, 0.0};
    weather.omsiSnow = true;
    weather.omsiSnowOnRoad = true;
    system.setWeather(weather);
    system.seedRoadWeatherFromPreset();

    ImVec2 lastButtonMin;
    ImVec2 lastButtonMax;
    const auto frame = [&]() {
        ImGui::NewFrame();
        ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(800, 600), ImGuiCond_Always);
        ImGui::Begin("Surface weather probe", nullptr,
                     ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoSavedSettings |
                         ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove);
        const auto action = openbus::drawSurfaceWeatherControls(system.roadWeather());
        if (action) {
            system.applySurfaceWeatherAction(*action);
        } else {
            lastButtonMin = ImGui::GetItemRectMin();
            lastButtonMax = ImGui::GetItemRectMax();
        }
        ImGui::End();
        ImGui::Render();
        return action;
    };
    const auto clickButton = [&](int row) {
        frame();
        const float y = (lastButtonMin.y + lastButtonMax.y) * 0.5F -
                        static_cast<float>(2 - row) * ImGui::GetFrameHeightWithSpacing();
        io.AddMousePosEvent(lastButtonMin.x + 10.0F, y);
        io.AddMouseButtonEvent(0, true);
        frame();
        io.AddMouseButtonEvent(0, false);
        return frame();
    };
    bool passed = true;
    const auto check = [&](bool condition, const char* message) {
        if (!condition) {
            std::cerr << message << '\n';
            passed = false;
        }
    };
    const std::optional<CollisionSurface> asphalt = CollisionSurface{0, true, true};
    const std::optional<CollisionSurface> terrain =
        CollisionSurface{std::nullopt, false, false, true};
    const std::optional<CollisionSurface> authoredSlush = CollisionSurface{8, false, false};
    const auto grip = [&](const std::optional<CollisionSurface>& surface) {
        return openbus::physics::roadWeatherFrictionMultiplier(surface, system.roadWeather());
    };
    frame();
    check(system.roadWeather().water == 0.5 && system.roadWeather().snow == 1.0 &&
              system.roadWeather().groundSnow == 1.0 && system.roadWeather().freezeFactor == 1.0,
          "surface state changed without interaction");
    check(clickButton(0) == openbus::SurfaceWeatherAction::DryRoadWater &&
              system.roadWeather().water == 0.0 && system.roadWeather().snow == 1.0 &&
              system.roadWeather().groundSnow == 1.0 && system.roadWeather().freezeFactor == 1.0,
          "drying did not preserve snow and cold exposure");
    system.seedRoadWeatherFromPreset();
    check(clickButton(1) == openbus::SurfaceWeatherAction::ClearRoadSnow &&
              system.roadWeather().water == 0.5 && system.roadWeather().snow == 0.0 &&
              system.roadWeather().groundSnow == 1.0 && system.roadWeather().freezeFactor == 1.0,
          "clearing road snow changed other surface conditions");
    check(std::abs(grip(asphalt) - 0.65) < 1e-12 && std::abs(grip(terrain) - 0.45) < 1e-12,
          "road snow clearing did not retain frozen road water and snowy terrain grip");
    system.seedRoadWeatherFromPreset();
    check(clickButton(2) == openbus::SurfaceWeatherAction::ClearAllSnow &&
              system.roadWeather().water == 0.5 && system.roadWeather().snow == 0.0 &&
              system.roadWeather().groundSnow == 0.0 && system.roadWeather().freezeFactor == 1.0,
          "clearing snow changed water or cold exposure");
    check(std::abs(grip(asphalt) - 0.65) < 1e-12 && grip(terrain) == 1.0 &&
              std::abs(grip(authoredSlush) - 0.45) < 1e-12,
          "snow clearing did not preserve wet or authored-slush grip");
    check(system.weather().precipitationType == 2 && system.weather().precipitationRate == 1.0 &&
              system.weather().omsiSnow && system.weather().omsiSnowOnRoad &&
              system.weather().omsiGroundWetness == weather.omsiGroundWetness,
          "surface actions changed the current weather or preset metadata");
    system.advanceRoadWeather(600.0);
    check(system.roadWeather().snow > 0.0 && system.roadWeather().groundSnow > 0.0,
          "snow did not accumulate again after clearing");
    system.applySurfaceWeatherAction(openbus::SurfaceWeatherAction::DryRoadWater);
    system.setPrecipitation(1, 1.0);
    system.advanceRoadWeather(600.0);
    check(system.roadWeather().water > 0.0, "rain did not accumulate again after drying");

    const auto beforeInvalid = system.roadWeather();
    bool rejected = false;
    try {
        system.applySurfaceWeatherAction(static_cast<openbus::SurfaceWeatherAction>(-1));
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    check(rejected && system.roadWeather().water == beforeInvalid.water &&
              system.roadWeather().snow == beforeInvalid.snow &&
              system.roadWeather().groundSnow == beforeInvalid.groundSnow &&
              system.roadWeather().freezeFactor == beforeInvalid.freezeFactor,
          "invalid action was not rejected without changing conditions");

    io.DisplaySize = ImVec2(320, 240);
    bool openPopup = true;
    bool scrollToBottom = false;
    float scrollMax = 0.0F;
    float scrollY = 0.0F;
    ImVec2 closeMin;
    ImVec2 closeMax;
    const auto modalFrame = [&]() {
        ImGui::NewFrame();
        ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_Always);
        ImGui::SetNextWindowSize(io.DisplaySize, ImGuiCond_Always);
        ImGui::Begin("Pause menu probe", nullptr,
                     ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoSavedSettings);
        if (openPopup) {
            ImGui::OpenPopup("Surface conditions");
            openPopup = false;
        }
        ImGui::SetNextWindowPos(ImVec2(160, 120), ImGuiCond_Always, ImVec2(0.5F, 0.5F));
        ImGui::SetNextWindowSize(ImVec2(288, 216), ImGuiCond_Always);
        std::optional<openbus::SurfaceWeatherAction> action;
        bool closed = false;
        if (ImGui::BeginPopupModal("Surface conditions", nullptr, ImGuiWindowFlags_NoSavedSettings)) {
            scrollMax = ImGui::GetScrollMaxY();
            scrollY = ImGui::GetScrollY();
            if (scrollToBottom)
                ImGui::SetScrollY(scrollMax);
            action = openbus::drawSurfaceWeatherControls(system.roadWeather());
            if (action)
                system.applySurfaceWeatherAction(*action);
            lastButtonMin = ImGui::GetItemRectMin();
            lastButtonMax = ImGui::GetItemRectMax();
            if (ImGui::Button("Close")) {
                closed = true;
                ImGui::CloseCurrentPopup();
            }
            closeMin = ImGui::GetItemRectMin();
            closeMax = ImGui::GetItemRectMax();
            ImGui::EndPopup();
        }
        ImGui::End();
        ImGui::Render();
        return std::pair{action, closed};
    };
    modalFrame();
    modalFrame();
    scrollToBottom = true;
    modalFrame();
    modalFrame();
    scrollToBottom = false;
    check(scrollMax > 0.0F && scrollY > 0.0F && closeMax.y <= 228.0F,
          "surface modal controls were not reachable in a small viewport");
    io.AddMousePosEvent(lastButtonMin.x + 10.0F, (lastButtonMin.y + lastButtonMax.y) * 0.5F);
    io.AddMouseButtonEvent(0, true);
    modalFrame();
    io.AddMouseButtonEvent(0, false);
    check(modalFrame().first == openbus::SurfaceWeatherAction::ClearAllSnow &&
              system.roadWeather().snow == 0.0 && system.roadWeather().groundSnow == 0.0,
          "scrolled surface modal could not clear snow");
    modalFrame();
    io.AddMousePosEvent((closeMin.x + closeMax.x) * 0.5F, (closeMin.y + closeMax.y) * 0.5F);
    io.AddMouseButtonEvent(0, true);
    modalFrame();
    io.AddMouseButtonEvent(0, false);
    check(modalFrame().second, "scrolled surface modal could not be closed");
    ImGui::DestroyContext();
    return passed ? 0 : 1;
}
