#include "app/CustomWeatherUi.h"
#include "core/Metar.h"

#include <imgui.h>

#include <cmath>
#include <iostream>
#include <limits>

int main() {
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.IniFilename = nullptr;
    io.DisplaySize = ImVec2(800, 800);
    io.DeltaTime = 1.0F / 60.0F;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    unsigned char* pixels = nullptr;
    int width = 0;
    int height = 0;
    io.Fonts->GetTexDataAsRGBA32(&pixels, &width, &height);

    openbus::WeatherState draft;
    ImVec2 cancelMin;
    ImVec2 cancelMax;
    const auto frame = [&]() {
        ImGui::NewFrame();
        ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(800, 800), ImGuiCond_Always);
        ImGui::Begin("Custom weather probe", nullptr,
                     ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoSavedSettings |
                         ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove);
        const auto action = openbus::drawCustomWeatherEditor(draft, {});
        if (action == openbus::CustomWeatherAction::None) {
            cancelMin = ImGui::GetItemRectMin();
            cancelMax = ImGui::GetItemRectMax();
        }
        ImGui::End();
        ImGui::Render();
        return action;
    };
    const auto click = [&](float x, float y) {
        io.AddMousePosEvent(x, y);
        io.AddMouseButtonEvent(0, true);
        frame();
        io.AddMouseButtonEvent(0, false);
        return frame();
    };
    const auto key = [&](ImGuiKey value) {
        io.AddKeyEvent(value, true);
        frame();
        io.AddKeyEvent(value, false);
        frame();
    };
    frame();
    const ImGuiStyle& style = ImGui::GetStyle();
    constexpr const char* intro =
        "Start from the current weather. Enable an optional value to set "
        "it; disabled values remain unspecified.";
    const float introHeight = ImGui::CalcTextSize(intro, nullptr, false,
                                                  800.0F - 2.0F * style.WindowPadding.x).y;
    const float firstRow = style.WindowPadding.y + introHeight + style.ItemSpacing.y +
                           ImGui::GetFrameHeight() * 0.5F;
    const float rowStep = ImGui::GetFrameHeightWithSpacing();
    const auto edit = [&](int row, float x, const char* text) {
        click(x, firstRow + static_cast<float>(row) * rowStep);
        io.AddKeyEvent(ImGuiMod_Ctrl, true);
        io.AddKeyEvent(ImGuiKey_A, true);
        frame();
        io.AddKeyEvent(ImGuiKey_A, false);
        io.AddKeyEvent(ImGuiMod_Ctrl, false);
        frame();
        io.AddInputCharactersUTF8(text);
        frame();
        key(ImGuiKey_Enter);
    };
    const float inputX = style.WindowPadding.x + 25.0F;
    const float checkboxX = style.WindowPadding.x + ImGui::GetFrameHeight() * 0.5F;
    const float optionalInputX =
        style.WindowPadding.x + ImGui::GetFrameHeight() + style.ItemInnerSpacing.x +
        ImGui::CalcTextSize("Set").x + style.ItemSpacing.x + 25.0F;
    edit(1, inputX, "0.75");
    edit(2, inputX, "-4.5");
    edit(3, inputX, "3.5");

    bool passed = true;
    if (std::abs(draft.precipitationRate - 0.75) > 1e-6) {
        std::cerr << "Editing precipitation intensity did not update the draft: "
                  << draft.precipitationRate << '\n';
        passed = false;
    }
    if (draft.temperatureCelsius != -4.5 || draft.absoluteHumidityGramsPerCubicMeter != 3.5) {
        std::cerr << "Temperature or humidity input did not update the draft\n";
        passed = false;
    }
    const char* optionalValues[] = {"2500", "0.4", "270", "12", "400", "100", "995"};
    for (int index = 0; index < 7; ++index) {
        const int row = index + 4;
        click(checkboxX, firstRow + static_cast<float>(row) * rowStep);
        edit(row, optionalInputX, optionalValues[index]);
    }
    if (draft.visibilityMeters != 2500.0 || draft.brightness != 0.4 ||
        draft.windDirectionDegrees != 270.0 || draft.windSpeedMetersPerSecond != 12.0 ||
        draft.cloudBaseAboveGroundMeters != 400.0 || draft.fogTopAboveGroundMeters != 100.0 ||
        draft.atmosphericPressureHectopascals != 995.0) {
        const auto value = [](const std::optional<double>& field) { return field.value_or(-1.0); };
        std::cerr << "Optional inputs: visibility=" << value(draft.visibilityMeters)
                  << " brightness=" << value(draft.brightness)
                  << " direction=" << value(draft.windDirectionDegrees)
                  << " speed=" << value(draft.windSpeedMetersPerSecond)
                  << " cloud=" << value(draft.cloudBaseAboveGroundMeters)
                  << " fog=" << value(draft.fogTopAboveGroundMeters)
                  << " pressure=" << value(draft.atmosphericPressureHectopascals) << '\n';
        passed = false;
    }
    click(checkboxX, firstRow + 4.0F * rowStep);
    if (draft.visibilityMeters) {
        std::cerr << "Disabling visibility did not clear it\n";
        passed = false;
    }
    click(inputX, firstRow);
    key(ImGuiKey_DownArrow);
    key(ImGuiKey_DownArrow);
    key(ImGuiKey_Enter);
    if (draft.precipitationType != 2) {
        std::cerr << "Precipitation selection did not choose snow\n";
        passed = false;
    }
    click(inputX, firstRow + 11.0F * rowStep);
    for (int index = 0; index < 4; ++index)
        key(ImGuiKey_DownArrow);
    key(ImGuiKey_Enter);
    if (draft.cloudType != openbus::WeatherCloudType::Overcast1) {
        std::cerr << "Cloud selection did not choose overcast\n";
        passed = false;
    }
    click(style.WindowPadding.x + 80.0F, firstRow + 12.0F * rowStep);
    if (std::abs(openbus::relativeHumidityFromAbsolute(
                     draft.temperatureCelsius, draft.absoluteHumidityGramsPerCubicMeter) -
                 50.0) > 2.0) {
        std::cerr << "Relative humidity slider did not update absolute humidity\n";
        passed = false;
    }
    const float applyWidth = ImGui::CalcTextSize("Apply custom weather").x +
                             2.0F * style.FramePadding.x;
    const float applyX = cancelMin.x - style.ItemSpacing.x - applyWidth * 0.5F;
    const float buttonY = (cancelMin.y + cancelMax.y) * 0.5F;
    if (click(applyX, buttonY) != openbus::CustomWeatherAction::Apply) {
        std::cerr << "Apply button did not return the apply action\n";
        passed = false;
    }
    frame();
    if (click((cancelMin.x + cancelMax.x) * 0.5F, (cancelMin.y + cancelMax.y) * 0.5F) !=
        openbus::CustomWeatherAction::Cancel) {
        std::cerr << "Cancel button did not return the cancel action\n";
        passed = false;
    }
    const double preservedHumidity = draft.absoluteHumidityGramsPerCubicMeter;
    draft.temperatureCelsius = 51.0;
    frame();
    if (draft.absoluteHumidityGramsPerCubicMeter != preservedHumidity) {
        std::cerr << "Out-of-range temperature changed absolute humidity\n";
        passed = false;
    }
    draft.temperatureCelsius = 15.0;
    for (double humidity : {100.0, std::numeric_limits<double>::max()}) {
        draft.absoluteHumidityGramsPerCubicMeter = humidity;
        frame();
        if (draft.absoluteHumidityGramsPerCubicMeter != humidity) {
            std::cerr << "Supersaturated humidity was changed without interaction\n";
            passed = false;
        }
    }
    draft.absoluteHumidityGramsPerCubicMeter = 100.0;
    frame();
    click(style.WindowPadding.x + 80.0F, firstRow + 12.0F * rowStep);
    if (std::abs(openbus::relativeHumidityFromAbsolute(
                     draft.temperatureCelsius, draft.absoluteHumidityGramsPerCubicMeter) -
                 50.0) > 2.0) {
        std::cerr << "Relative humidity slider did not recover from supersaturation\n";
        passed = false;
    }
    draft.absoluteHumidityGramsPerCubicMeter = preservedHumidity;
    io.DisplaySize = ImVec2(640, 480);
    bool openPopup = true;
    bool scrollToBottom = false;
    float scrollMax = 0.0F;
    float scrollY = 0.0F;
    const std::string modalError =
        "Could not apply custom weather: Wind direction must be at least 0 and less than 360 "
        "degrees. Correct the entered value and apply the custom weather again.";
    const auto modalFrame = [&]() {
        ImGui::NewFrame();
        ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_Always);
        ImGui::SetNextWindowSize(io.DisplaySize, ImGuiCond_Always);
        ImGui::Begin("Pause menu probe", nullptr,
                     ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoSavedSettings);
        if (openPopup) {
            ImGui::OpenPopup("Custom weather");
            openPopup = false;
        }
        ImGui::SetNextWindowPos(ImVec2(320, 240), ImGuiCond_Always, ImVec2(0.5F, 0.5F));
        ImGui::SetNextWindowSize(ImVec2(576, 432), ImGuiCond_Always);
        auto action = openbus::CustomWeatherAction::None;
        if (ImGui::BeginPopupModal("Custom weather", nullptr, ImGuiWindowFlags_NoSavedSettings)) {
            scrollMax = ImGui::GetScrollMaxY();
            scrollY = ImGui::GetScrollY();
            if (scrollToBottom)
                ImGui::SetScrollY(scrollMax);
            action = openbus::drawCustomWeatherEditor(draft, modalError);
            if (action == openbus::CustomWeatherAction::None) {
                cancelMin = ImGui::GetItemRectMin();
                cancelMax = ImGui::GetItemRectMax();
            }
            ImGui::EndPopup();
        }
        ImGui::End();
        ImGui::Render();
        return action;
    };
    modalFrame();
    modalFrame();
    scrollToBottom = true;
    modalFrame();
    modalFrame();
    scrollToBottom = false;
    if ((scrollMax > 0.0F && scrollY <= 0.0F) || cancelMax.y > 456.0F) {
        std::cerr << "Custom weather modal cannot reach its buttons on a 640x480 viewport: "
                  << "scroll=" << scrollY << "/" << scrollMax << " button=" << cancelMax.y
                  << '\n';
        passed = false;
    }
    const float modalApplyX = cancelMin.x - style.ItemSpacing.x - applyWidth * 0.5F;
    const float modalButtonY = (cancelMin.y + cancelMax.y) * 0.5F;
    io.AddMousePosEvent(modalApplyX, modalButtonY);
    io.AddMouseButtonEvent(0, true);
    modalFrame();
    io.AddMouseButtonEvent(0, false);
    if (modalFrame() != openbus::CustomWeatherAction::Apply) {
        std::cerr << "Scrolled custom weather modal could not apply weather\n";
        passed = false;
    }
    ImGui::DestroyContext();
    return passed ? 0 : 1;
}
