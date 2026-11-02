#include "app/RenderLoopInternal.h"
#include "app/ImGuiLayer.h"
#include "app/ScopedSwapInterval.h"
#include "app/SteamPresence.h"
#include "app/traffic/PlayerTimetableDiagnostics.h"
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
    const openbus::app::ScopedSwapInterval menuVsync(glfwSwapInterval, 1, vsyncEnabled_ ? 1 : 0);
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
            gameLog.Error("Unable to load map '" + pathToUtf8(maps[selectedMapIndex]) +
                          "': " + error.what());
        }
    };
    loadSelectedMap();

    while (!shouldClose()) {
        steamPresence_.runCallbacks();
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
                gameLog.Error("Initial map tile loading failed: " + mapLoadError);
            }
            if (waitingForInitialMapWindow) {
                ImGui::SetNextWindowSize(ImVec2(480.0F, 120.0F), ImGuiCond_Always);
                ImGui::SetNextWindowPos(ImVec2(static_cast<float>(windowWidth) * 0.5F,
                                               static_cast<float>(windowHeight) * 0.5F),
                                        ImGuiCond_Always, ImVec2(0.5F, 0.5F));
                ImGui::Begin("OpenBus setup", nullptr,
                             ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize);
                ImGui::TextUnformatted(
                    "Preparing map index, shared textures, and spawn-area assets...");
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
                gameLog.Error("Unable to start map loading: " + mapLoadError);
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

void RenderLoop::updateOnFoot(const World& world, double elapsedSeconds) {
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
    world.updateOnFoot(
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
    const auto registration = vehicleRegistrationSelectionAtIndex(
        *mapVehicleConfig_, selectedMapRegistrationIndex_, mapVehicleNumberInput_.data(),
        mapRegistrationInput_.data());
    request.setupOptions.vehicleNumber = registration.vehicleNumber;
    request.setupOptions.registration = registration.registration;
    return request;
}

void RenderLoop::SetSpawnBusError(std::string error) {
    spawnBusError_ = std::move(error);
}

void RenderLoop::showStartupStatus(const std::string& status) {
    if (shouldClose()) {
        return;
    }
    glfwPollEvents();
    const auto [framebufferWidth, framebufferHeight] = framebufferSize();
    const auto [windowWidth, windowHeight] = windowSize();
    glViewport(0, 0, framebufferWidth, framebufferHeight);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    imguiLayer_->beginFrame();
    ImGui::SetNextWindowSize(ImVec2(520.0F, 145.0F), ImGuiCond_Always);
    ImGui::SetNextWindowPos(
        ImVec2(static_cast<float>(windowWidth) * 0.5F, static_cast<float>(windowHeight) * 0.5F),
        ImGuiCond_Always, ImVec2(0.5F, 0.5F));
    ImGui::Begin("OpenBus startup", nullptr,
                 ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove);
    ImGui::TextUnformatted("Preparing your session");
    ImGui::Separator();
    ImGui::TextWrapped("%s", status.c_str());
    ImGui::TextUnformatted("Please wait...");
    ImGui::End();
    imguiLayer_->render();
    glfwSwapBuffers(window_);
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
        restoreMapVehiclePreference();
    } catch (const std::exception& error) {
        mapVehicleConfig_.reset();
        mapVehicleLoadError_ = error.what();
        gameLog.Error("Unable to load bus '" +
                      pathToUtf8(vehicleCatalog_[selectedMapVehicleIndex_].configurationPath) +
                      "': " + error.what());
    }
}

void RenderLoop::restoreMapVehiclePreference() {
    const std::string key = openbus::app::vehicleRegistrationPreferenceKey(
        vehicleCatalog_[selectedMapVehicleIndex_].configurationPath);
    const auto saved = vehicleRegistrationPreferences_.find(key);
    if (saved == vehicleRegistrationPreferences_.end()) {
        return;
    }

    const std::size_t count = registrationOptionCount(*mapVehicleConfig_);
    for (std::size_t index = 0; index < count; ++index) {
        VehicleConfig option = *mapVehicleConfig_;
        selectRegistrationAtIndex(option, index);
        if (option.selectedVehicleNumber == saved->second.vehicleNumber &&
            option.selectedRegistration == saved->second.registration) {
            selectedMapRegistrationIndex_ = index;
            break;
        }
    }
    if (mapVehicleConfig_->registrationFree) {
        copyToInputBuffer(mapVehicleNumberInput_, saved->second.vehicleNumber);
        copyToInputBuffer(mapRegistrationInput_, saved->second.registration);
    }
}

void RenderLoop::persistMapVehiclePreference() {
    if (!vehicleRegistrationPreferencesLoaded_ || !mapVehicleConfig_) {
        return;
    }

    openbus::app::VehicleRegistrationPreference preference;
    const std::size_t count = registrationOptionCount(*mapVehicleConfig_);
    if (count > 0) {
        VehicleConfig selected = *mapVehicleConfig_;
        selectRegistrationAtIndex(selected, std::min(selectedMapRegistrationIndex_, count - 1));
        preference.vehicleNumber = selected.selectedVehicleNumber;
        preference.registration = selected.selectedRegistration;
    }
    if (mapVehicleConfig_->registrationFree) {
        preference.vehicleNumber = mapVehicleNumberInput_.data();
        preference.registration = mapRegistrationInput_.data();
    } else if (count == 0) {
        return;
    }

    const std::string key = openbus::app::vehicleRegistrationPreferenceKey(
        vehicleCatalog_[selectedMapVehicleIndex_].configurationPath);
    vehicleRegistrationPreferences_[key] = std::move(preference);
    try {
        openbus::app::saveVehicleRegistrationPreferences(vehicleRegistrationPreferencesPath_,
                                                         vehicleRegistrationPreferences_);
        vehicleRegistrationPreferencesError_.clear();
    } catch (const std::exception& error) {
        vehicleRegistrationPreferencesError_ = error.what();
        gameLog.Error("Vehicle registration preferences were not saved: " +
                      vehicleRegistrationPreferencesError_);
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
                const bool registrationAutomatic = mapVehicleConfig_->registrationAutomatic &&
                                                   !mapVehicleConfig_->registrationFree;
                VehicleConfig registration = *mapVehicleConfig_;
                selectRegistrationAtIndex(registration, selectedMapRegistrationIndex_);
                const auto registrationLabel =
                    [registrationAutomatic](const VehicleConfig& config) {
                        std::string label = config.selectedVehicleNumber;
                        if (!registrationAutomatic && !label.empty() &&
                            !config.selectedRegistration.empty()) {
                            label += " - ";
                        }
                        if (!registrationAutomatic) {
                            label += config.selectedRegistration;
                        }
                        return label.empty() ? std::string("(unnumbered)") : label;
                    };
                const std::string currentRegistration = registrationLabel(registration);
                if (ImGui::BeginCombo(registrationAutomatic ? "Vehicle number"
                                                            : "Number / registration",
                                      currentRegistration.c_str())) {
                    for (std::size_t index = 0; index < registrationCount; ++index) {
                        VehicleConfig option = *mapVehicleConfig_;
                        selectRegistrationAtIndex(option, index);
                        const std::string label = registrationLabel(option);
                        const bool selected = index == selectedMapRegistrationIndex_;
                        if (ImGui::Selectable(label.c_str(), selected)) {
                            selectedMapRegistrationIndex_ = index;
                            copyToInputBuffer(mapRegistrationInput_, option.selectedRegistration);
                            copyToInputBuffer(mapVehicleNumberInput_, option.selectedVehicleNumber);
                            persistMapVehiclePreference();
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
                if (ImGui::InputText("Vehicle number override", mapVehicleNumberInput_.data(),
                                     mapVehicleNumberInput_.size())) {
                    persistMapVehiclePreference();
                }
                ImGui::SetNextItemWidth(-1.0F);
                if (ImGui::InputText(registrationCount > 0 ? "Registration override"
                                                           : "Registration",
                                     mapRegistrationInput_.data(), mapRegistrationInput_.size())) {
                    persistMapVehiclePreference();
                }
            } else if (mapVehicleConfig_->registrationAutomatic &&
                       !mapVehicleConfig_->registrationFree) {
                ImGui::Text("Registration: automatic (%s)",
                            mapVehicleConfig_->registrationPrefix.c_str());
            } else if (!mapVehicleConfig_->registrationFree) {
                ImGui::TextUnformatted("Registration: not configured");
            }
        }
        if (!mapVehicleLoadError_.empty()) {
            ImGui::TextWrapped("Unable to load bus options: %s", mapVehicleLoadError_.c_str());
        }
        if (!vehicleRegistrationPreferencesError_.empty()) {
            ImGui::TextWrapped("Vehicle preferences: %s",
                               vehicleRegistrationPreferencesError_.c_str());
        }
        if (!vehicleOdometerPreferencesError_.empty()) {
            ImGui::TextWrapped("Odometer preferences: %s",
                               vehicleOdometerPreferencesError_.c_str());
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
    ImGui::TextUnformatted("Select a scheduled trip in the timetable panel after spawning.");
    ImGui::TextUnformatted("WASD: walk | Space: jump | F4: toggle mouse look");
    ImGui::TextUnformatted("Press F4 to release the cursor and use this panel.");
    ImGui::End();
}

void RenderLoop::drawPlayerTimetableSelectionUi() {
    selectedPlayerTimetableSelection_.reset();
    if (ImGui::Checkbox("Drive a scheduled trip", &selectedPlayerUseTimetable_) &&
        !selectedPlayerUseTimetable_) {
        selectedPlayerTimetableSelection_.reset();
    }
    if (selectedPlayerUseTimetable_) {
        const auto& hof = playerVehicle_->scripts->hofData();
        if (playerTimetableCatalog_.update(passengerNetwork_.routes, passengerTimetableCourses_,
                                           hof)) {
            openbus::app::logPlayerTimetableCatalog(
                playerTimetableCatalog_, passengerNetwork_.routes, passengerTimetableCourses_,
                playerTimetableMapKeys_, playerTimetableMapPath_,
                playerVehicle_->variables.getString("hof_file"), hof,
                playerVehicle_->scripts->errors());
        }
        const auto& lines = playerTimetableCatalog_.lines;
        if (!lines.empty()) {
            if (std::ranges::find(lines, selectedPlayerLineName_,
                                  &openbus::app::PlayerTimetableCatalogLine::name) == lines.end()) {
                selectedPlayerLineName_ = lines.front().name;
                selectedPlayerCourseIndex_ = 0;
                selectedPlayerCourseTripIndex_ = 0;
                selectedPlayerFirstStopIndex_ = 0;
            }
            if (ImGui::BeginCombo("Line", selectedPlayerLineName_.c_str())) {
                for (const auto& line : lines) {
                    const bool selected = line.name == selectedPlayerLineName_;
                    if (ImGui::Selectable(line.name.c_str(), selected)) {
                        selectedPlayerLineName_ = line.name;
                        selectedPlayerCourseIndex_ = 0;
                        selectedPlayerCourseTripIndex_ = 0;
                        selectedPlayerFirstStopIndex_ = 0;
                    }
                    if (selected) {
                        ImGui::SetItemDefaultFocus();
                    }
                }
                ImGui::EndCombo();
            }
            const auto& line = *std::ranges::find(lines, selectedPlayerLineName_,
                                                  &openbus::app::PlayerTimetableCatalogLine::name);
            const auto& courses = line.courses;
            if (!courses.empty()) {
                if (std::ranges::find(courses, selectedPlayerCourseIndex_,
                                      &openbus::app::PlayerTimetableCatalogCourse::courseIndex) ==
                    courses.end()) {
                    selectedPlayerCourseIndex_ = courses.front().courseIndex;
                    selectedPlayerCourseTripIndex_ = 0;
                    selectedPlayerFirstStopIndex_ = 0;
                }
                const auto& displayedCourse =
                    passengerTimetableCourses_[selectedPlayerCourseIndex_];
                if (ImGui::BeginCombo("Tour / course", displayedCourse.name.c_str())) {
                    for (const auto& cachedCourse : courses) {
                        const std::size_t courseIndex = cachedCourse.courseIndex;
                        const auto& option = passengerTimetableCourses_[courseIndex];
                        const bool selected = courseIndex == selectedPlayerCourseIndex_;
                        if (ImGui::Selectable(option.name.c_str(), selected)) {
                            selectedPlayerCourseIndex_ = courseIndex;
                            selectedPlayerCourseTripIndex_ = 0;
                            selectedPlayerFirstStopIndex_ = 0;
                        }
                        if (selected) {
                            ImGui::SetItemDefaultFocus();
                        }
                    }
                    ImGui::EndCombo();
                }
                const auto& course = passengerTimetableCourses_[selectedPlayerCourseIndex_];
                const auto& trips =
                    std::ranges::find(courses, selectedPlayerCourseIndex_,
                                      &openbus::app::PlayerTimetableCatalogCourse::courseIndex)
                        ->trips;
                if (!trips.empty()) {
                    if (std::ranges::find(trips, selectedPlayerCourseTripIndex_,
                                          &openbus::app::PlayerTimetableCatalogTrip::tripIndex) ==
                        trips.end()) {
                        selectedPlayerCourseTripIndex_ = trips.front().tripIndex;
                        selectedPlayerFirstStopIndex_ = 0;
                    }
                    const auto& trip = course.trips[selectedPlayerCourseTripIndex_];
                    const auto cachedTrip =
                        std::ranges::find(trips, selectedPlayerCourseTripIndex_,
                                          &openbus::app::PlayerTimetableCatalogTrip::tripIndex);
                    const auto& route = passengerNetwork_.routes[cachedTrip->routeIndex];
                    {
                        const auto tripDisplayLabel = [](const std::string& tripName,
                                                         double departureMinutes) {
                            const int minuteOfDay = static_cast<int>(std::floor(departureMinutes));
                            const int minute = minuteOfDay % 60;
                            return (tripName.empty() ? std::string("Trip") : tripName) + " (" +
                                   std::to_string((minuteOfDay / 60) % 24) + ":" +
                                   (minute < 10 ? "0" : "") + std::to_string(minute) + ")";
                        };
                        const std::string tripLabel =
                            tripDisplayLabel(route.tripName, trip.departureMinutes);
                        if (ImGui::BeginCombo("Trip", tripLabel.c_str())) {
                            for (const auto& option : trips) {
                                const std::size_t tripIndex = option.tripIndex;
                                const auto& optionTrip = course.trips[tripIndex];
                                const auto& optionRoute =
                                    passengerNetwork_.routes[option.routeIndex];
                                const std::string label = tripDisplayLabel(
                                    optionRoute.tripName, optionTrip.departureMinutes);
                                const bool selected = tripIndex == selectedPlayerCourseTripIndex_;
                                if (ImGui::Selectable(label.c_str(), selected)) {
                                    selectedPlayerCourseTripIndex_ = tripIndex;
                                    selectedPlayerFirstStopIndex_ = 0;
                                }
                                if (selected) {
                                    ImGui::SetItemDefaultFocus();
                                }
                            }
                            ImGui::EndCombo();
                        }
                        const auto selectedTrip =
                            std::ranges::find(trips, selectedPlayerCourseTripIndex_,
                                              &openbus::app::PlayerTimetableCatalogTrip::tripIndex);
                        const auto& selectedRoute =
                            passengerNetwork_.routes[selectedTrip->routeIndex];
                        const auto missingStops = std::ranges::count_if(
                            selectedRoute.stopPoses, [](const auto& pose) { return !pose; });
                        if (missingStops != 0) {
                            ImGui::TextWrapped(
                                "%zu stops cannot be located or serviced on this trip.",
                                static_cast<std::size_t>(missingStops));
                        }
                        {
                            const std::size_t selectedStopIndex = std::min(
                                selectedPlayerFirstStopIndex_, selectedRoute.stops.size() - 1);
                            selectedPlayerFirstStopIndex_ = selectedStopIndex;
                            const std::string& stopLabel =
                                selectedRoute.stops[selectedPlayerFirstStopIndex_].name;
                            if (ImGui::BeginCombo("First stop", stopLabel.c_str())) {
                                for (std::size_t stopIndex = 0;
                                     stopIndex < selectedRoute.stops.size(); ++stopIndex) {
                                    const bool selected =
                                        stopIndex == selectedPlayerFirstStopIndex_;
                                    const bool unavailable = !selectedRoute.stopPoses.empty() &&
                                                             !selectedRoute.stopPoses[stopIndex];
                                    const std::string label = selectedRoute.stops[stopIndex].name +
                                        (unavailable ? " (location unavailable)" : "");
                                    if (ImGui::Selectable(label.c_str(), selected)) {
                                        selectedPlayerFirstStopIndex_ = stopIndex;
                                    }
                                    if (selected) {
                                        ImGui::SetItemDefaultFocus();
                                    }
                                }
                                ImGui::EndCombo();
                            }
                            ImGui::Checkbox("Drive as an additional bus",
                                            &selectedPlayerDriveAsAdditionalBus_);
                            selectedPlayerTimetableSelection_ =
                                SpawnBusRequest::TimetableSelection{
                                    selectedTrip->routeIndex, selectedPlayerCourseIndex_,
                                    selectedPlayerCourseTripIndex_, selectedPlayerFirstStopIndex_,
                                    selectedPlayerDriveAsAdditionalBus_};
                        }
                    }
                } else {
                    ImGui::TextUnformatted("No selectable trips are available for this line.");
                }
            } else {
                ImGui::TextUnformatted("No tours are available for this line.");
            }
        } else {
            if (playerVehicle_->variables.getString("hof_file").empty()) {
                ImGui::TextUnformatted("Select a Depot (HOF) when spawning the bus.");
            } else if (hof.routes.empty()) {
                ImGui::TextUnformatted("The selected HOF has no loaded routes.");
                for (const auto& error : playerVehicle_->scripts->errors()) {
                    ImGui::TextWrapped("%s", error.c_str());
                }
            } else if (passengerNetwork_.routes.empty()) {
                ImGui::TextWrapped(
                    "No player timetable routes were registered from the map schedule; "
                    "see the log for details.");
            } else {
                ImGui::TextWrapped("No player timetable routes match this HOF unambiguously; "
                                   "see the log for details.");
            }
        }
        ImGui::BeginDisabled(!selectedPlayerTimetableSelection_);
        if (ImGui::Button("Start selected trip")) {
            SetPlayerTimetableSelection(selectedPlayerTimetableSelection_);
            selectedPlayerUseTimetable_ = false;
        }
        ImGui::EndDisabled();
        ImGui::SameLine();
        if (ImGui::Button("Cancel selection")) {
            selectedPlayerUseTimetable_ = false;
            selectedPlayerTimetableSelection_.reset();
        }
    }
}

void RenderLoop::drawPlayerTimetableUi() {
    if (!imguiEnabled_ || playerVehicle_ == nullptr || renderingReflection_ || captureMode_) {
        return;
    }
    ImGui::SetNextWindowPos(ImVec2(20.0F, 20.0F), ImGuiCond_Always);
    ImGui::SetNextWindowBgAlpha(0.82F);
    ImGui::Begin("Player timetable", nullptr,
                 ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoCollapse |
                     ImGuiWindowFlags_NoNav);
    const auto drawConfiguredViewSelector = [this](const char* title,
                                                   const std::vector<VehicleViewEntry>& entries) {
        if (entries.empty()) {
            return;
        }

        int currentDriverViewIndex = -1;
        if (cameraView_ > 0 && static_cast<std::size_t>(cameraView_) <= vehicleCameras_.size()) {
            const std::size_t currentCameraIndex = static_cast<std::size_t>(cameraView_ - 1);
            for (std::size_t index = 0; index <= currentCameraIndex; ++index) {
                if (vehicleCameras_[index].kind == VehicleCameraKind::Driver) {
                    ++currentDriverViewIndex;
                }
            }
        }
        std::string preview = "Select view";
        for (const VehicleViewEntry& entry : entries) {
            if (entry.viewIndex != currentDriverViewIndex) {
                continue;
            }
            preview = entry.label.empty()
                          ? std::string(title) + " " + std::to_string(entry.viewIndex + 1)
                          : entry.label;
            break;
        }

        if (ImGui::BeginCombo(title, preview.c_str())) {
            for (std::size_t index = 0; index < entries.size(); ++index) {
                const VehicleViewEntry& entry = entries[index];
                const std::string label =
                    entry.label.empty()
                        ? std::string(title) + " " + std::to_string(entry.viewIndex + 1)
                        : entry.label;
                ImGui::PushID(static_cast<int>(index));
                if (ImGui::Selectable(label.c_str(), entry.viewIndex == currentDriverViewIndex) &&
                    !selectDriverCameraView(entry.viewIndex)) {
                    gameLog.Warning("Configured vehicle view has no matching driver camera: " +
                                    std::to_string(entry.viewIndex));
                }
                ImGui::PopID();
            }
            ImGui::EndCombo();
        }
    };
    drawConfiguredViewSelector("Schedule view", vehicleScheduleViews_);
    drawConfiguredViewSelector("Ticket-selling view", vehicleTicketSellingViews_);
    if (!playerTimetable_.state) {
        drawPlayerTimetableSelectionUi();
        ImGui::End();
        return;
    }
    const PlayerTimetableState& state = *playerTimetable_.state;
    if (state.selection.courseIndex >= passengerTimetableCourses_.size() ||
        state.selection.courseTripIndex >=
            passengerTimetableCourses_[state.selection.courseIndex].trips.size() ||
        state.selection.passengerRouteIndex >= passengerNetwork_.routes.size()) {
        throw std::logic_error("Active player timetable selection is outside its catalog");
    }
    const auto& course = passengerTimetableCourses_[state.selection.courseIndex];
    const auto& route = passengerNetwork_.routes[state.selection.passengerRouteIndex];
    ImGui::Text("Line %s | %s", state.lineName.c_str(), course.name.c_str());
    ImGui::Text("Trip: %s to %s", route.tripName.c_str(), route.destinationName.c_str());
    if (state.stopAfterCurrentTrip) {
        ImGui::TextUnformatted("Driving will stop at this trip's terminus.");
    } else if (ImGui::Button("Stop driving after this trip")) {
        playerTimetable_.state->stopAfterCurrentTrip = true;
    }
    if (ImGui::Button("Cancel trip now")) {
        SetPlayerTimetableSelection(std::nullopt);
        selectedPlayerUseTimetable_ = false;
        selectedPlayerTimetableSelection_.reset();
    }
    ImGui::End();
}
