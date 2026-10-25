#include "app/RenderLoopInternal.h"
#include "app/ImGuiLayer.h"
#include "config/VehicleCatalog.h"
#include "core/Environment.h"
#include "core/Logger.h"

#include <imgui.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <ctime>
#include <numbers>
#include <system_error>

namespace {

std::string pathToUtf8(const std::filesystem::path& path) {
    const std::u8string utf8 = path.u8string();
    return std::string(reinterpret_cast<const char*>(utf8.data()), utf8.size());
}

std::vector<std::filesystem::path> discoverMaps(const std::filesystem::path& omsiRoot) {
    const std::filesystem::path mapsDirectory = omsiRoot / "maps";
    std::error_code error;
    if (!std::filesystem::is_directory(mapsDirectory, error) || error) {
        throw std::runtime_error("OMSI maps directory is unavailable: " + pathToUtf8(mapsDirectory));
    }

    std::vector<std::filesystem::path> maps;
    std::filesystem::directory_iterator map(mapsDirectory, error), end;
    for (; !error && map != end; map.increment(error)) {
        std::error_code entryError;
        if (map->is_directory(entryError) && !entryError &&
            std::filesystem::is_regular_file(map->path() / "global.cfg", entryError) &&
            !entryError) {
            maps.push_back(map->path());
        }
    }
    if (error) {
        throw std::runtime_error("Failed to list OMSI maps directory '" + pathToUtf8(mapsDirectory) +
                                 "': " + error.message());
    }
    std::sort(maps.begin(), maps.end());
    return maps;
}

std::string spawnLabel(std::size_t index, const openbus::map::MapEntryPoint& entryPoint) {
    return "[" + std::to_string(index) + "] " +
           (entryPoint.name.empty() ? std::string("(unnamed)") : entryPoint.name);
}

std::string normalizedText(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char character) {
        return static_cast<char>(std::tolower(character));
    });
    return value;
}

void copyToInputBuffer(std::array<char, 256>& buffer, const std::string& value) {
    const std::size_t count = std::min(value.size(), buffer.size() - 1);
    std::copy_n(value.data(), count, buffer.data());
    buffer[count] = '\0';
}

} // namespace

bool RenderLoop::runStartupMenu(const std::filesystem::path& omsiRoot,
                                StartupSelection& selection) {
    imguiEnabled_ = true;
    glfwSetInputMode(window_, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
    const std::vector<std::filesystem::path> maps = discoverMaps(omsiRoot);
    if (maps.empty()) {
        throw std::runtime_error("No OMSI maps containing global.cfg were found under '" +
                                 pathToUtf8(omsiRoot / "maps") + "'");
    }
    const std::filesystem::path vehiclesDirectory = omsiRoot / "Vehicles";
    const std::vector<openbus::config::VehicleCatalogEntry> vehicles =
        openbus::config::discoverVehicleCatalog(vehiclesDirectory);
    if (vehicles.empty()) {
        throw std::runtime_error("No OMSI .bus files were found under '" +
                                 pathToUtf8(vehiclesDirectory) + "'");
    }

    std::size_t selectedMapIndex = 0;
    const std::string configuredMap = openbus::getEnvironment("OPENBUS_MAP_PATH") == nullptr
                                          ? std::string{}
                                          : openbus::getEnvironment("OPENBUS_MAP_PATH");
    if (!configuredMap.empty()) {
        const auto configured =
            std::find_if(maps.begin(), maps.end(), [&](const std::filesystem::path& path) {
                return pathToUtf8(path.filename()) == configuredMap;
            });
        if (configured != maps.end()) {
            selectedMapIndex = static_cast<std::size_t>(configured - maps.begin());
        }
    }

    const auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::tm localTime = {};
#ifdef _WIN32
    localtime_s(&localTime, &now);
#else
    localtime_r(&now, &localTime);
#endif
    int year = localTime.tm_year + 1900;
    int month = localTime.tm_mon + 1;
    int day = localTime.tm_mday;
    int hour = localTime.tm_hour;
    int minute = localTime.tm_min;
    int selectedSpawn = 0;
    std::size_t selectedVehicleIndex = 0;
    std::size_t selectedAppearanceIndex = 0;
    std::size_t selectedDepotIndex = 0;
    std::size_t selectedRegistrationIndex = 0;
    std::optional<openbus::map::MapDefinition> selectedDefinition;
    std::string mapLoadError;
    std::optional<VehicleConfig> selectedVehicleConfig;
    std::vector<openbus::config::VehicleAppearance> appearances;
    std::vector<openbus::config::VehicleDepot> depots;
    std::string vehicleLoadError;
    std::array<char, 256> registrationInput = {};
    std::array<char, 256> vehicleNumberInput = {};

    const auto loadSelectedVehicle = [&] {
        selectedVehicleConfig.reset();
        appearances.clear();
        depots.clear();
        vehicleLoadError.clear();
        selectedAppearanceIndex = 0;
        selectedDepotIndex = 0;
        selectedRegistrationIndex = 0;
        copyToInputBuffer(registrationInput, std::string{});
        copyToInputBuffer(vehicleNumberInput, std::string{});
        try {
            selectedVehicleConfig = loadVehicleConfig(
                vehicles[selectedVehicleIndex].configurationPath, VehicleFileKind::Bus, false);
            appearances = openbus::config::discoverVehicleAppearances(
                vehicles[selectedVehicleIndex].configurationPath, vehiclesDirectory);
            depots = openbus::config::discoverVehicleDepots(
                vehicles[selectedVehicleIndex].configurationPath, vehiclesDirectory);
            copyToInputBuffer(registrationInput, selectedVehicleConfig->selectedRegistration);
            copyToInputBuffer(vehicleNumberInput, selectedVehicleConfig->selectedVehicleNumber);
        } catch (const std::exception& error) {
            vehicleLoadError = error.what();
            gameLog.Log("Unable to load bus '" +
                        pathToUtf8(vehicles[selectedVehicleIndex].configurationPath) +
                        "': " + error.what());
        }
    };
    loadSelectedVehicle();

    const auto loadSelectedMap = [&] {
        selectedDefinition.reset();
        selectedSpawn = 0;
        mapLoadError.clear();
        try {
            selectedDefinition = openbus::map::loadMapDefinition(maps[selectedMapIndex]);
        } catch (const std::exception& error) {
            mapLoadError = error.what();
            gameLog.Log("Unable to load map '" + pathToUtf8(maps[selectedMapIndex]) +
                        "': " + error.what());
        }
    };
    loadSelectedMap();

    while (!shouldClose()) {
        glfwPollEvents();
        if (shouldClose()) {
            return false;
        }
        const auto [framebufferWidth, framebufferHeight] = framebufferSize();
        const auto [windowWidth, windowHeight] = windowSize();
        glViewport(0, 0, framebufferWidth, framebufferHeight);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        imguiLayer_->beginFrame();

        ImGui::SetNextWindowSize(ImVec2(700.0F, 760.0F), ImGuiCond_Once);
        ImGui::SetNextWindowPos(
            ImVec2(static_cast<float>(windowWidth) * 0.5F, static_cast<float>(windowHeight) * 0.5F),
            ImGuiCond_Always, ImVec2(0.5F, 0.5F));
        ImGui::Begin("OpenBus setup", nullptr,
                     ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize);
        ImGui::TextUnformatted("Choose your bus, depot, and where and when to start.");
        ImGui::Spacing();

        std::vector<std::string> manufacturers;
        for (const openbus::config::VehicleCatalogEntry& vehicle : vehicles) {
            if (manufacturers.empty() ||
                normalizedText(manufacturers.back()) != normalizedText(vehicle.manufacturer)) {
                manufacturers.push_back(vehicle.manufacturer);
            }
        }
        const std::string selectedManufacturer = vehicles[selectedVehicleIndex].manufacturer;
        if (ImGui::BeginCombo("Manufacturer", selectedManufacturer.c_str())) {
            for (const std::string& manufacturer : manufacturers) {
                const bool selected =
                    normalizedText(manufacturer) == normalizedText(selectedManufacturer);
                if (ImGui::Selectable(manufacturer.c_str(), selected) && !selected) {
                    const auto nextVehicle =
                        std::find_if(vehicles.begin(), vehicles.end(),
                                     [&](const openbus::config::VehicleCatalogEntry& candidate) {
                                         return normalizedText(candidate.manufacturer) ==
                                                normalizedText(manufacturer);
                                     });
                    selectedVehicleIndex = static_cast<std::size_t>(nextVehicle - vehicles.begin());
                    loadSelectedVehicle();
                }
                if (selected) {
                    ImGui::SetItemDefaultFocus();
                }
            }
            ImGui::EndCombo();
        }

        const std::string selectedType = vehicles[selectedVehicleIndex].type;
        const std::string currentVehicleLabel =
            selectedType + " (" +
            pathToUtf8(vehicles[selectedVehicleIndex].configurationPath.filename()) + ")";
        if (ImGui::BeginCombo("Type", currentVehicleLabel.c_str())) {
            for (std::size_t index = 0; index < vehicles.size(); ++index) {
                if (normalizedText(vehicles[index].manufacturer) !=
                    normalizedText(vehicles[selectedVehicleIndex].manufacturer)) {
                    continue;
                }
                const std::string label =
                    vehicles[index].type + " (" +
                    pathToUtf8(vehicles[index].configurationPath.filename()) + ")";
                const bool selected = index == selectedVehicleIndex;
                if (ImGui::Selectable(label.c_str(), selected) && !selected) {
                    selectedVehicleIndex = index;
                    loadSelectedVehicle();
                }
                if (selected) {
                    ImGui::SetItemDefaultFocus();
                }
            }
            ImGui::EndCombo();
        }

        if (selectedVehicleConfig.has_value()) {
            std::string defaultAppearance = selectedVehicleConfig->friendlyDefaultPaint.empty()
                                                ? vehicles[selectedVehicleIndex].defaultAppearance
                                                : selectedVehicleConfig->friendlyDefaultPaint;
            if (defaultAppearance.empty()) {
                defaultAppearance = "Default";
            }
            const std::string currentAppearance =
                selectedAppearanceIndex == 0 ? defaultAppearance
                                             : appearances[selectedAppearanceIndex - 1].name;
            if (ImGui::BeginCombo("Appearance", currentAppearance.c_str())) {
                const bool defaultSelected = selectedAppearanceIndex == 0;
                if (ImGui::Selectable(defaultAppearance.c_str(), defaultSelected)) {
                    selectedAppearanceIndex = 0;
                }
                if (defaultSelected) {
                    ImGui::SetItemDefaultFocus();
                }
                for (std::size_t index = 0; index < appearances.size(); ++index) {
                    const bool selected = selectedAppearanceIndex == index + 1;
                    if (ImGui::Selectable(appearances[index].name.c_str(), selected)) {
                        selectedAppearanceIndex = index + 1;
                    }
                    if (selected) {
                        ImGui::SetItemDefaultFocus();
                    }
                }
                ImGui::EndCombo();
            }

            const std::string currentDepot =
                selectedDepotIndex == 0 ? "None" : depots[selectedDepotIndex - 1].name;
            if (ImGui::BeginCombo("Depot (HOF)", currentDepot.c_str())) {
                const bool noDepotSelected = selectedDepotIndex == 0;
                if (ImGui::Selectable("None", noDepotSelected)) {
                    selectedDepotIndex = 0;
                }
                if (noDepotSelected) {
                    ImGui::SetItemDefaultFocus();
                }
                for (std::size_t index = 0; index < depots.size(); ++index) {
                    const bool selected = selectedDepotIndex == index + 1;
                    const std::string label =
                        depots[index].name + " (" +
                        pathToUtf8(depots[index].path.filename()) + ")";
                    if (ImGui::Selectable(label.c_str(), selected)) {
                        selectedDepotIndex = index + 1;
                    }
                    if (selected) {
                        ImGui::SetItemDefaultFocus();
                    }
                }
                ImGui::EndCombo();
            }

            const std::size_t registrationCount = registrationOptionCount(*selectedVehicleConfig);
            if (registrationCount > 0) {
                selectedRegistrationIndex =
                    std::min(selectedRegistrationIndex, registrationCount - 1);
                VehicleConfig registration = *selectedVehicleConfig;
                selectRegistrationAtIndex(registration, selectedRegistrationIndex);
                const auto registrationLabel = [](const VehicleConfig& config) {
                    std::string label = config.selectedVehicleNumber;
                    if (!label.empty() && !config.selectedRegistration.empty()) {
                        label += " — ";
                    }
                    label += config.selectedRegistration;
                    return label.empty() ? std::string("(unnumbered)") : label;
                };
                const std::string currentRegistration = registrationLabel(registration);
                if (ImGui::BeginCombo("Number / registration", currentRegistration.c_str())) {
                    for (std::size_t index = 0; index < registrationCount; ++index) {
                        VehicleConfig option = *selectedVehicleConfig;
                        selectRegistrationAtIndex(option, index);
                        const std::string label = registrationLabel(option);
                        const bool selected = index == selectedRegistrationIndex;
                        if (ImGui::Selectable(label.c_str(), selected)) {
                            selectedRegistrationIndex = index;
                            copyToInputBuffer(registrationInput, option.selectedRegistration);
                            copyToInputBuffer(vehicleNumberInput, option.selectedVehicleNumber);
                        }
                        if (selected) {
                            ImGui::SetItemDefaultFocus();
                        }
                    }
                    ImGui::EndCombo();
                }
            }
            if (selectedVehicleConfig->registrationFree) {
                ImGui::SetNextItemWidth(-1.0F);
                ImGui::InputText(registrationCount > 0 ? "Registration override" : "Registration",
                                 registrationInput.data(), registrationInput.size());
            } else if (selectedVehicleConfig->registrationAutomatic) {
                ImGui::Text("Registration: automatic (%s)",
                            selectedVehicleConfig->registrationPrefix.c_str());
            } else {
                ImGui::TextUnformatted("Registration: not configured");
            }
            if (!vehicleLoadError.empty()) {
                ImGui::TextWrapped("Unable to load this bus: %s", vehicleLoadError.c_str());
            }
        }

        ImGui::Spacing();
        const std::string selectedMapName = pathToUtf8(maps[selectedMapIndex].filename());
        if (ImGui::BeginCombo("Map", selectedMapName.c_str())) {
            for (std::size_t index = 0; index < maps.size(); ++index) {
                const std::string label = pathToUtf8(maps[index].filename());
                const bool selected = index == selectedMapIndex;
                if (ImGui::Selectable(label.c_str(), selected) && !selected) {
                    selectedMapIndex = index;
                    loadSelectedMap();
                }
                if (selected) {
                    ImGui::SetItemDefaultFocus();
                }
            }
            ImGui::EndCombo();
        }
        ImGui::TextUnformatted("Date");
        ImGui::SetNextItemWidth(125.0F);
        ImGui::InputInt("Year", &year, 1, 1);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(105.0F);
        ImGui::InputInt("Month", &month, 1, 1);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(95.0F);
        ImGui::InputInt("Day", &day, 1, 1);
        ImGui::TextUnformatted("Time (24-hour)");
        ImGui::SetNextItemWidth(105.0F);
        ImGui::InputInt("Hour", &hour, 1, 1);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(105.0F);
        ImGui::InputInt("Minute", &minute, 1, 1);

        if (!mapLoadError.empty()) {
            ImGui::Spacing();
            ImGui::TextWrapped("Unable to load this map: %s", mapLoadError.c_str());
        } else if (selectedDefinition.has_value() && selectedDefinition->entryPoints.empty()) {
            ImGui::Spacing();
            ImGui::TextWrapped("This map does not define any spawn locations.");
        } else if (selectedDefinition.has_value()) {
            const auto& entryPoints = selectedDefinition->entryPoints;
            selectedSpawn = std::clamp(selectedSpawn, 0, static_cast<int>(entryPoints.size()) - 1);
            const std::string currentSpawn =
                spawnLabel(static_cast<std::size_t>(selectedSpawn),
                           entryPoints[static_cast<std::size_t>(selectedSpawn)]);
            if (ImGui::BeginCombo("Spawn location", currentSpawn.c_str())) {
                for (std::size_t index = 0; index < entryPoints.size(); ++index) {
                    const std::string label = spawnLabel(index, entryPoints[index]);
                    const bool selected = static_cast<int>(index) == selectedSpawn;
                    if (ImGui::Selectable(label.c_str(), selected)) {
                        selectedSpawn = static_cast<int>(index);
                    }
                    if (selected) {
                        ImGui::SetItemDefaultFocus();
                    }
                }
                ImGui::EndCombo();
            }
        }

        const std::chrono::year_month_day selectedDate{
            std::chrono::year{std::clamp(year, 1, 9999)},
            std::chrono::month{static_cast<unsigned>(std::clamp(month, 1, 12))},
            std::chrono::day{static_cast<unsigned>(std::clamp(day, 1, 31))}};
        const bool validDateTime = year >= 1 && year <= 9999 && month >= 1 && month <= 12 &&
                                   day >= 1 && day <= 31 && selectedDate.ok() && hour >= 0 &&
                                   hour < 24 && minute >= 0 && minute < 60;
        const bool canStart = selectedDefinition.has_value() &&
                              !selectedDefinition->entryPoints.empty() && validDateTime &&
                              selectedVehicleConfig.has_value() && vehicleLoadError.empty();
        if (!validDateTime) {
            ImGui::TextColored(ImVec4(1.0F, 0.45F, 0.35F, 1.0F), "Enter a valid date and time.");
        }
        ImGui::Spacing();
        ImGui::BeginDisabled(!canStart);
        const auto saveSelection = [&](bool startInBus) {
            selection.mapDirectory = maps[selectedMapIndex];
            selection.mapDefinition = std::move(*selectedDefinition);
            selection.vehicleConfigPath = vehicles[selectedVehicleIndex].configurationPath;
            selection.vehicleSetup = {};
            if (selectedAppearanceIndex > 0) {
                const openbus::config::VehicleAppearance& appearance =
                    appearances[selectedAppearanceIndex - 1];
                selection.vehicleSetup.ctcTextureReplacements = appearance.textureReplacements;
                selection.vehicleSetup.appearanceKey =
                    appearance.sourcePath.generic_string() + "|" + appearance.name;
            } else {
                selection.vehicleSetup.appearanceKey = "default";
            }
            if (selectedDepotIndex > 0) {
                const openbus::config::VehicleDepot& depot = depots[selectedDepotIndex - 1];
                selection.vehicleSetup.depotPath = depot.path;
                selection.vehicleSetup.depotName = depot.name;
            }
            if (registrationOptionCount(*selectedVehicleConfig) > 0) {
                VehicleConfig registration = *selectedVehicleConfig;
                selectRegistrationAtIndex(registration, selectedRegistrationIndex);
                selection.vehicleSetup.vehicleNumber = registration.selectedVehicleNumber;
                selection.vehicleSetup.registration = selectedVehicleConfig->registrationFree
                                                          ? std::string(registrationInput.data())
                                                          : registration.selectedRegistration;
            } else if (selectedVehicleConfig->registrationFree) {
                selection.vehicleSetup.registration = registrationInput.data();
                selection.vehicleSetup.vehicleNumber = vehicleNumberInput.data();
            }
            selection.startInBus = startInBus;
            selection.spawnEntryPoint = static_cast<std::size_t>(selectedSpawn);
            selection.year = year;
            selection.month = month;
            selection.day = day;
            selection.timeSeconds = static_cast<double>(hour * 3600 + minute * 60);
        };
        const float buttonWidth =
            (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) * 0.5F;
        if (ImGui::Button("Start in bus", ImVec2(buttonWidth, 42.0F))) {
            saveSelection(true);
            ImGui::EndDisabled();
            ImGui::End();
            imguiLayer_->render();
            glfwSwapBuffers(window_);
            return true;
        }
        ImGui::SameLine();
        if (ImGui::Button("Start on foot", ImVec2(buttonWidth, 42.0F))) {
            saveSelection(false);
            ImGui::EndDisabled();
            ImGui::End();
            imguiLayer_->render();
            glfwSwapBuffers(window_);
            return true;
        }
        ImGui::EndDisabled();
        ImGui::End();
        imguiLayer_->render();
        glfwSwapBuffers(window_);
    }
    return false;
}

void RenderLoop::SetNoBusSpawnPoint(const VehiclePlacement& placement) {
    onFootState_ = {placement.position, 0.0, false};
    freeCameraPose_.position = {placement.position[0], placement.position[1],
                                placement.position[2] + 1.7};
    freeCameraPose_.yawRadians = placement.yawDegrees * std::numbers::pi / 180.0;
    freeCameraPose_.pitchRadians = 0.0;
    cameraView_ = 0;
    freeCameraEnabled_ = true;
}

void RenderLoop::updateOnFoot(const BusSimulation& simulation, double elapsedSeconds) {
    if (playerVehicle_ != nullptr) {
        return;
    }
    const bool focused = glfwGetWindowAttrib(window_, GLFW_FOCUSED) == GLFW_TRUE;
    const bool acceptInput =
        focused && (mouseLookEnabled_ || !imguiEnabled_ || !ImGui::GetIO().WantCaptureKeyboard);
    const auto pressed = [&](int key) {
        return acceptInput && glfwGetKey(window_, key) == GLFW_PRESS;
    };
    const bool jump = pressed(GLFW_KEY_SPACE);
    simulation.updateOnFoot(
        onFootState_, elapsedSeconds, freeCameraPose_.yawRadians,
        static_cast<double>(pressed(GLFW_KEY_W)) - static_cast<double>(pressed(GLFW_KEY_S)),
        static_cast<double>(pressed(GLFW_KEY_D)) - static_cast<double>(pressed(GLFW_KEY_A)),
        jump && !previousJumpKeyState_);
    previousJumpKeyState_ = jump;
    freeCameraPose_.position = {onFootState_.feetPosition[0], onFootState_.feetPosition[1],
                                onFootState_.feetPosition[2] + 1.7};
}

std::array<double, 3> RenderLoop::streamingPosition(const BusSimulation& simulation) const {
    return playerVehicle_ == nullptr ? onFootState_.feetPosition
                                     : simulation.chassisPose().position;
}

bool RenderLoop::consumeSpawnBusRequest() {
    const bool requested = spawnBusRequestPending_;
    spawnBusRequestPending_ = false;
    return requested;
}

void RenderLoop::drawOnFootUi() {
    if (!imguiEnabled_ || playerVehicle_ != nullptr || renderingReflection_ || captureMode_) {
        return;
    }
    ImGui::SetNextWindowPos(ImVec2(20.0F, 20.0F), ImGuiCond_Always);
    ImGui::SetNextWindowBgAlpha(0.82F);
    ImGui::Begin("On foot", nullptr,
                 ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoCollapse |
                     ImGuiWindowFlags_NoNav);
    ImGui::TextUnformatted("You are not in a bus.");
    if (ImGui::Button("Spawn selected bus", ImVec2(220.0F, 36.0F))) {
        spawnBusRequestPending_ = true;
    }
    ImGui::TextUnformatted("WASD: walk | Space: jump | F4: toggle mouse look");
    ImGui::TextUnformatted("Press F4 to release the cursor and use this panel.");
    ImGui::End();
}
