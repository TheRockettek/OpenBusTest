#include "app/RenderLoopInternal.h"
#include "app/ImGuiLayer.h"
#include "config/VehicleCatalog.h"
#include "core/Environment.h"
#include "core/Logger.h"
#include "core/Utf8Path.h"

#include <imgui.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <ctime>
#include <numbers>
#include <string>
#include <system_error>

namespace {

using openbus::pathToUtf8;

std::vector<std::filesystem::path> discoverMaps(const std::filesystem::path& omsiRoot) {
    openbus::rendering::TraceScope trace("startup", "startup.discoverMaps");
    const std::filesystem::path mapsDirectory = omsiRoot / "maps";
    std::error_code error;
    if (!std::filesystem::is_directory(mapsDirectory, error) || error) {
        throw std::runtime_error("OMSI maps directory is unavailable: " +
                                 pathToUtf8(mapsDirectory));
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
        throw std::runtime_error("Failed to list OMSI maps directory '" +
                                 pathToUtf8(mapsDirectory) + "': " + error.message());
    }
    std::sort(maps.begin(), maps.end());
    return maps;
}

std::string spawnLabel(std::size_t index, const openbus::map::MapEntryPoint& entryPoint) {
    return "[" + std::to_string(index) + "] " +
           (entryPoint.name.empty() ? std::string("(unnamed)") : entryPoint.name);
}

void copyToInputBuffer(std::array<char, 256>& buffer, const std::string& value) {
    const std::size_t count = std::min(value.size(), buffer.size() - 1);
    std::copy_n(value.data(), count, buffer.data());
    buffer[count] = '\0';
}

std::string environmentValue(const char* name) {
    const char* value = openbus::getEnvironment(name);
    return value == nullptr ? std::string{} : std::string(value);
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
    vehiclesDirectory_ = vehiclesDirectory;
    {
        TraceScope catalogTrace("startup", "startup.discoverVehicleCatalog");
        vehicleCatalog_ = openbus::config::discoverVehicleCatalog(vehiclesDirectory);
    }
    if (vehicleCatalog_.empty()) {
        throw std::runtime_error("No OMSI .bus files were found under '" +
                                 pathToUtf8(vehiclesDirectory) + "'");
    }

    std::size_t selectedMapIndex = 0;
    const std::string configuredMap = environmentValue("OPENBUS_MAP_PATH");
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
    std::optional<openbus::map::MapDefinition> selectedDefinition;
    std::string mapLoadError;
    bool waitingForInitialMapWindow = false;
    std::optional<TraceScope> initialMapWaitTrace;
    bool restoreSetupDialogSize = false;

    const auto loadSelectedMap = [&] {
        selectedDefinition.reset();
        selectedSpawn = 0;
        mapLoadError.clear();
        try {
            TraceScope mapTrace("startup", "startup.loadSelectedMapDefinition");
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

        if (waitingForInitialMapWindow) {
            try {
                const VehiclePlacement& spawn =
                    selection.mapDefinition.entryPoints[selection.spawnEntryPoint].placement;
                updateMapStreaming(spawn.position[0], spawn.position[1]);
                if (initialMapWindowReady()) {
                    if (initialMapWaitTrace.has_value()) {
                        initialMapWaitTrace->finish();
                        initialMapWaitTrace.reset();
                    }
                    ImGui::SetNextWindowSize(ImVec2(480.0F, 120.0F), ImGuiCond_Always);
                    ImGui::SetNextWindowPos(ImVec2(static_cast<float>(windowWidth) * 0.5F,
                                                   static_cast<float>(windowHeight) * 0.5F),
                                            ImGuiCond_Always, ImVec2(0.5F, 0.5F));
                    ImGui::Begin("OpenBus setup", nullptr,
                                 ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize);
                    ImGui::TextUnformatted("Initial map tiles are ready.");
                    ImGui::End();
                    imguiLayer_->render();
                    glfwSwapBuffers(window_);
                    return true;
                }
            } catch (const std::exception& error) {
                if (initialMapWaitTrace.has_value()) {
                    initialMapWaitTrace->finish();
                    initialMapWaitTrace.reset();
                }
                waitingForInitialMapWindow = false;
                restoreSetupDialogSize = true;
                mapLoadError = error.what();
                selectedDefinition = selection.mapDefinition;
                gameLog.Log("Initial map tile loading failed: " + mapLoadError);
            }
            if (waitingForInitialMapWindow) {
                ImGui::SetNextWindowSize(ImVec2(480.0F, 120.0F), ImGuiCond_Always);
                ImGui::SetNextWindowPos(ImVec2(static_cast<float>(windowWidth) * 0.5F,
                                               static_cast<float>(windowHeight) * 0.5F),
                                        ImGuiCond_Always, ImVec2(0.5F, 0.5F));
                ImGui::Begin("OpenBus setup", nullptr,
                             ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize);
                ImGui::TextUnformatted("Loading initial map tiles...");
                ImGui::TextUnformatted("The setup menu will close when the spawn area is ready.");
                ImGui::End();
                imguiLayer_->render();
                glfwSwapBuffers(window_);
                continue;
            }
        }

        ImGui::SetNextWindowSize(ImVec2(700.0F, 760.0F),
                                 restoreSetupDialogSize ? ImGuiCond_Always : ImGuiCond_Once);
        restoreSetupDialogSize = false;
        ImGui::SetNextWindowPos(
            ImVec2(static_cast<float>(windowWidth) * 0.5F, static_cast<float>(windowHeight) * 0.5F),
            ImGuiCond_Always, ImVec2(0.5F, 0.5F));
        ImGui::Begin("OpenBus setup", nullptr,
                     ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize);
        ImGui::TextUnformatted("Choose where and when to start. Select your bus in the map.");
        ImGui::Spacing();

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
                              !selectedDefinition->entryPoints.empty() && validDateTime;
        if (!validDateTime) {
            ImGui::TextColored(ImVec4(1.0F, 0.45F, 0.35F, 1.0F), "Enter a valid date and time.");
        }
        ImGui::Spacing();
        ImGui::BeginDisabled(!canStart);
        const auto saveSelection = [&] {
            selection.mapDirectory = maps[selectedMapIndex];
            selection.mapDefinition = std::move(*selectedDefinition);
            selection.initialSimulationBusConfigPath = vehicleCatalog_.front().configurationPath;
            selection.simulationBusConfigCandidates.clear();
            selection.simulationBusConfigCandidates.reserve(vehicleCatalog_.size());
            for (const auto& vehicle : vehicleCatalog_) {
                selection.simulationBusConfigCandidates.push_back(vehicle.configurationPath);
            }
            selection.vehicleSetup = {};
            selection.spawnEntryPoint = static_cast<std::size_t>(selectedSpawn);
            selection.year = year;
            selection.month = month;
            selection.day = day;
            selection.timeSeconds = static_cast<double>(hour * 3600 + minute * 60);
        };
        if (ImGui::Button("Start map", ImVec2(-1.0F, 42.0F))) {
            saveSelection();
            try {
                TraceScope mapTrace("startup", "startup.beginInitialMapSceneLoad");
                SetMap(selection.mapDefinition, selection.spawnEntryPoint,
                       openbus::map::selectMapGroundTextureIndex(
                           selection.mapDefinition, environmentValue("OPENBUS_MAP_GROUNDTEX")),
                       omsiRoot, true);
                initialMapWaitTrace.emplace("startup", "startup.waitForInitialMapWindow");
                waitingForInitialMapWindow = true;
                mapLoadError.clear();
            } catch (const std::exception& error) {
                mapLoadError = error.what();
                selectedDefinition = selection.mapDefinition;
                gameLog.Log("Unable to start map loading: " + mapLoadError);
            }
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

std::optional<SpawnBusRequest> RenderLoop::consumeSpawnBusRequest() {
    if (!spawnBusRequestPending_ || vehicleCatalog_.empty() || !mapVehicleConfig_) {
        spawnBusRequestPending_ = false;
        return std::nullopt;
    }
    spawnBusRequestPending_ = false;
    spawnBusError_.clear();
    selectedMapVehicleIndex_ = std::min(selectedMapVehicleIndex_, vehicleCatalog_.size() - 1);
    SpawnBusRequest request;
    request.configurationPath = vehicleCatalog_[selectedMapVehicleIndex_].configurationPath;
    request.placement.position = onFootState_.feetPosition;
    request.placement.yawDegrees = freeCameraPose_.yawRadians * 180.0 / std::numbers::pi;
    if (selectedMapAppearanceIndex_ > 0) {
        const auto& appearance = mapVehicleAppearances_[selectedMapAppearanceIndex_ - 1];
        request.setupOptions.ctcTextureReplacements = appearance.textureReplacements;
        request.setupOptions.appearanceKey =
            appearance.sourcePath.generic_string() + "|" + appearance.name;
    } else {
        request.setupOptions.appearanceKey = "default";
    }
    if (selectedMapDepotIndex_ > 0) {
        const auto& depot = mapVehicleDepots_[selectedMapDepotIndex_ - 1];
        request.setupOptions.depotPath = depot.path;
        request.setupOptions.depotName = depot.name;
    }
    const std::size_t registrationCount = registrationOptionCount(*mapVehicleConfig_);
    if (registrationCount > 0) {
        VehicleConfig registration = *mapVehicleConfig_;
        selectRegistrationAtIndex(registration,
                                  std::min(selectedMapRegistrationIndex_, registrationCount - 1));
        request.setupOptions.vehicleNumber = registration.selectedVehicleNumber;
        request.setupOptions.registration = mapVehicleConfig_->registrationFree
                                                ? std::string(mapRegistrationInput_.data())
                                                : registration.selectedRegistration;
    } else if (mapVehicleConfig_->registrationFree) {
        request.setupOptions.registration = mapRegistrationInput_.data();
        request.setupOptions.vehicleNumber = mapVehicleNumberInput_.data();
    }
    return request;
}

void RenderLoop::SetSpawnBusError(std::string error) {
    spawnBusError_ = std::move(error);
}

void RenderLoop::loadMapVehicleOptions() {
    loadedMapVehicleIndex_ = selectedMapVehicleIndex_;
    mapVehicleConfig_.reset();
    mapVehicleAppearances_.clear();
    mapVehicleDepots_.clear();
    mapVehicleLoadError_.clear();
    selectedMapAppearanceIndex_ = 0;
    selectedMapDepotIndex_ = 0;
    selectedMapRegistrationIndex_ = 0;
    mapRegistrationInput_.fill('\0');
    mapVehicleNumberInput_.fill('\0');
    try {
        TraceScope vehicleTrace("startup", "startup.loadMapSelectedVehicleConfig");
        const auto& configPath = vehicleCatalog_[selectedMapVehicleIndex_].configurationPath;
        mapVehicleConfig_ = loadVehicleConfig(configPath, VehicleFileKind::Bus, false);
        mapVehicleAppearances_ =
            openbus::config::discoverVehicleAppearances(configPath, vehiclesDirectory_);
        mapVehicleDepots_ = openbus::config::discoverVehicleDepots(configPath, vehiclesDirectory_);
        copyToInputBuffer(mapRegistrationInput_, mapVehicleConfig_->selectedRegistration);
        copyToInputBuffer(mapVehicleNumberInput_, mapVehicleConfig_->selectedVehicleNumber);
    } catch (const std::exception& error) {
        mapVehicleConfig_.reset();
        mapVehicleLoadError_ = error.what();
        gameLog.Log("Unable to load bus '" +
                    pathToUtf8(vehicleCatalog_[selectedMapVehicleIndex_].configurationPath) +
                    "': " + error.what());
    }
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
    if (vehicleCatalog_.empty()) {
        ImGui::TextUnformatted("No bus configurations are available.");
    } else {
        selectedMapVehicleIndex_ = std::min(selectedMapVehicleIndex_, vehicleCatalog_.size() - 1);
        if (!loadedMapVehicleIndex_ || *loadedMapVehicleIndex_ != selectedMapVehicleIndex_) {
            loadMapVehicleOptions();
        }
        const auto vehicleLabel = [](const openbus::config::VehicleCatalogEntry& vehicle) {
            return vehicle.manufacturer + " " + vehicle.type + " (" +
                   pathToUtf8(vehicle.configurationPath.filename()) + ")";
        };
        const std::string selectedVehicleLabel =
            vehicleLabel(vehicleCatalog_[selectedMapVehicleIndex_]);
        if (ImGui::BeginCombo("Bus", selectedVehicleLabel.c_str())) {
            for (std::size_t index = 0; index < vehicleCatalog_.size(); ++index) {
                const std::string label = vehicleLabel(vehicleCatalog_[index]);
                const bool selected = index == selectedMapVehicleIndex_;
                if (ImGui::Selectable(label.c_str(), selected)) {
                    selectedMapVehicleIndex_ = index;
                    loadMapVehicleOptions();
                }
                if (selected) {
                    ImGui::SetItemDefaultFocus();
                }
            }
            ImGui::EndCombo();
        }
        if (mapVehicleConfig_) {
            std::string defaultAppearance =
                mapVehicleConfig_->friendlyDefaultPaint.empty()
                    ? vehicleCatalog_[selectedMapVehicleIndex_].defaultAppearance
                    : mapVehicleConfig_->friendlyDefaultPaint;
            if (defaultAppearance.empty()) {
                defaultAppearance = "Default";
            }
            const std::string currentAppearance =
                selectedMapAppearanceIndex_ == 0
                    ? defaultAppearance
                    : mapVehicleAppearances_[selectedMapAppearanceIndex_ - 1].name;
            if (ImGui::BeginCombo("Appearance", currentAppearance.c_str())) {
                const bool defaultSelected = selectedMapAppearanceIndex_ == 0;
                if (ImGui::Selectable(defaultAppearance.c_str(), defaultSelected)) {
                    selectedMapAppearanceIndex_ = 0;
                }
                if (defaultSelected) {
                    ImGui::SetItemDefaultFocus();
                }
                for (std::size_t index = 0; index < mapVehicleAppearances_.size(); ++index) {
                    const bool selected = selectedMapAppearanceIndex_ == index + 1;
                    if (ImGui::Selectable(mapVehicleAppearances_[index].name.c_str(), selected)) {
                        selectedMapAppearanceIndex_ = index + 1;
                    }
                    if (selected) {
                        ImGui::SetItemDefaultFocus();
                    }
                }
                ImGui::EndCombo();
            }

            const std::string currentDepot =
                selectedMapDepotIndex_ == 0 ? "None"
                                            : mapVehicleDepots_[selectedMapDepotIndex_ - 1].name;
            if (ImGui::BeginCombo("Depot (HOF)", currentDepot.c_str())) {
                const bool noDepotSelected = selectedMapDepotIndex_ == 0;
                if (ImGui::Selectable("None", noDepotSelected)) {
                    selectedMapDepotIndex_ = 0;
                }
                if (noDepotSelected) {
                    ImGui::SetItemDefaultFocus();
                }
                for (std::size_t index = 0; index < mapVehicleDepots_.size(); ++index) {
                    const bool selected = selectedMapDepotIndex_ == index + 1;
                    const std::string label = mapVehicleDepots_[index].name + " (" +
                                              pathToUtf8(mapVehicleDepots_[index].path.filename()) +
                                              ")";
                    if (ImGui::Selectable(label.c_str(), selected)) {
                        selectedMapDepotIndex_ = index + 1;
                    }
                    if (selected) {
                        ImGui::SetItemDefaultFocus();
                    }
                }
                ImGui::EndCombo();
            }

            const std::size_t registrationCount = registrationOptionCount(*mapVehicleConfig_);
            if (registrationCount > 0) {
                selectedMapRegistrationIndex_ =
                    std::min(selectedMapRegistrationIndex_, registrationCount - 1);
                VehicleConfig registration = *mapVehicleConfig_;
                selectRegistrationAtIndex(registration, selectedMapRegistrationIndex_);
                const auto registrationLabel = [](const VehicleConfig& config) {
                    std::string label = config.selectedVehicleNumber;
                    if (!label.empty() && !config.selectedRegistration.empty()) {
                        label += " - ";
                    }
                    label += config.selectedRegistration;
                    return label.empty() ? std::string("(unnumbered)") : label;
                };
                const std::string currentRegistration = registrationLabel(registration);
                if (ImGui::BeginCombo("Number / registration", currentRegistration.c_str())) {
                    for (std::size_t index = 0; index < registrationCount; ++index) {
                        VehicleConfig option = *mapVehicleConfig_;
                        selectRegistrationAtIndex(option, index);
                        const std::string label = registrationLabel(option);
                        const bool selected = index == selectedMapRegistrationIndex_;
                        if (ImGui::Selectable(label.c_str(), selected)) {
                            selectedMapRegistrationIndex_ = index;
                            copyToInputBuffer(mapRegistrationInput_, option.selectedRegistration);
                            copyToInputBuffer(mapVehicleNumberInput_, option.selectedVehicleNumber);
                        }
                        if (selected) {
                            ImGui::SetItemDefaultFocus();
                        }
                    }
                    ImGui::EndCombo();
                }
            }
            if (mapVehicleConfig_->registrationFree) {
                ImGui::SetNextItemWidth(-1.0F);
                ImGui::InputText(registrationCount > 0 ? "Registration override" : "Registration",
                                 mapRegistrationInput_.data(), mapRegistrationInput_.size());
            } else if (mapVehicleConfig_->registrationAutomatic) {
                ImGui::Text("Registration: automatic (%s)",
                            mapVehicleConfig_->registrationPrefix.c_str());
            } else {
                ImGui::TextUnformatted("Registration: not configured");
            }
        }
        if (!mapVehicleLoadError_.empty()) {
            ImGui::TextWrapped("Unable to load bus options: %s", mapVehicleLoadError_.c_str());
        }
    }
    ImGui::BeginDisabled(vehicleCatalog_.empty() || !mapVehicleConfig_);
    if (ImGui::Button("Spawn selected bus", ImVec2(220.0F, 36.0F))) {
        spawnBusRequestPending_ = true;
    }
    ImGui::EndDisabled();
    if (!spawnBusError_.empty()) {
        ImGui::TextWrapped("Unable to spawn bus: %s", spawnBusError_.c_str());
    }
    ImGui::TextUnformatted("WASD: walk | Space: jump | F4: toggle mouse look");
    ImGui::TextUnformatted("Press F4 to release the cursor and use this panel.");
    ImGui::End();
}
