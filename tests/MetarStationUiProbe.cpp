#include "app/MetarStationUi.h"

#include <imgui.h>

#include <iostream>
#include <string>
#include <vector>

int main() {
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.IniFilename = nullptr;
    io.DisplaySize = ImVec2(800, 600);
    io.DeltaTime = 1.0F / 60.0F;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    unsigned char* pixels = nullptr;
    int width = 0;
    int height = 0;
    io.Fonts->GetTexDataAsRGBA32(&pixels, &width, &height);

    openbus::MetarStationPicker picker;
    std::vector<openbus::MetarStation> stations = {
        {"EGLL", "Heathrow", "United Kingdom"},
        {"KJFK", "Kennedy", "United States"},
    };
    const auto frame = [&]() {
        ImGui::NewFrame();
        ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(800, 600), ImGuiCond_Always);
        ImGui::Begin("Station picker probe", nullptr,
                     ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoSavedSettings |
                         ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove);
        openbus::drawMetarStationPicker(picker, stations);
        ImGui::End();
        ImGui::Render();
    };
    const auto click = [&](float x, float y) {
        io.AddMousePosEvent(x, y);
        io.AddMouseButtonEvent(0, true);
        frame();
        io.AddMouseButtonEvent(0, false);
        frame();
    };
    const auto key = [&](ImGuiKey value) {
        io.AddKeyEvent(value, true);
        frame();
        io.AddKeyEvent(value, false);
        frame();
    };
    frame();
    const ImGuiStyle& style = ImGui::GetStyle();
    const float x = style.WindowPadding.x + 25.0F;
    const float firstRow = style.WindowPadding.y + ImGui::GetFrameHeight() * 0.5F;
    const float rowStep = ImGui::GetFrameHeightWithSpacing();
    const auto edit = [&](int row, const char* text) {
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
    bool passed = true;
    const auto check = [&](bool condition, const char* message) {
        if (!condition) {
            std::cerr << message << '\n';
            passed = false;
        }
    };
    edit(0, "United States");
    click(x, firstRow + rowStep);
    key(ImGuiKey_DownArrow);
    key(ImGuiKey_Enter);
    check(picker.selected == 1 && std::string(picker.code.data()) == "KJFK",
          "country search did not select the worldwide catalogue station");
    const std::vector<openbus::MetarStation> reversed = {stations[1], stations[0]};
    openbus::remapMetarStationSelection(picker, stations, reversed);
    check(picker.selected == 0, "station selection was not retained when the list changed");
    openbus::remapMetarStationSelection(picker, reversed, stations);
    check(picker.selected == 1, "station selection did not restore its position");
    const std::vector<openbus::MetarStation> removed = {stations[0]};
    openbus::remapMetarStationSelection(picker, stations, removed);
    check(!picker.selected && std::string(picker.code.data()) == "KJFK",
          "removed station did not retain its direct ICAO code");
    picker.selected = 1;
    edit(2, "egll");
    check(!picker.selected && std::string(picker.code.data()) == "EGLL",
          "direct ICAO input did not normalize the code or clear catalogue selection");
    stations.clear();
    edit(2, "yssy");
    check(!picker.selected && std::string(picker.code.data()) == "YSSY",
          "direct ICAO input did not work without a catalogue");
    edit(2, "abcde");
    check(std::string(picker.code.data()) == "ABCDE",
          "overlong station input was silently truncated to a different ICAO code");
    ImGui::DestroyContext();
    return passed ? 0 : 1;
}
