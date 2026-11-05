#include "app/RenderLoopInternal.h"
#include "app/ImGuiLayer.h"
#include "app/ScopedSwapInterval.h"
#include "app/SteamPresence.h"
#include "app/traffic/PlayerTimetableDiagnostics.h"
#include "config/VehicleCatalog.h"
#include "core/Environment.h"
#include "core/Logger.h"
#include "core/Utf8Path.h"
#include "physics/BusConfiguration.h"

#include <imgui.h>

#include <algorithm>
#include <array>
#include <cctype>
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

void drawMapEntryPointCombo(const std::vector<openbus::map::MapEntryPoint>& entryPoints,
                            std::size_t& selectedIndex, const char* label) {
    if (entryPoints.empty()) {
        ImGui::TextUnformatted("This map has no spawn locations.");
        return;
    }
    selectedIndex = std::min(selectedIndex, entryPoints.size() - 1);
    const std::string current = spawnLabel(selectedIndex, entryPoints[selectedIndex]);
    if (ImGui::BeginCombo(label, current.c_str())) {
        for (std::size_t index = 0; index < entryPoints.size(); ++index) {
            const std::string option = spawnLabel(index, entryPoints[index]);
            const bool selected = index == selectedIndex;
            if (ImGui::Selectable(option.c_str(), selected)) {
                selectedIndex = index;
            }
            if (selected) {
                ImGui::SetItemDefaultFocus();
            }
        }
        ImGui::EndCombo();
    }
}

void copyToInputBuffer(std::array<char, 256>& buffer, const std::string& value) {
    const std::size_t count = std::min(value.size(), buffer.size() - 1);
    std::copy_n(value.data(), count, buffer.data());
    buffer[count] = '\0';
}

std::string lowercase(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char character) {
                       return static_cast<char>(std::tolower(character));
                   });
    return value;
}

bool matchesVehicleSearch(const openbus::config::VehicleCatalogEntry& vehicle,
                          const std::string& search) {
    if (search.empty()) {
        return true;
    }
    const std::string searchable = lowercase(
        vehicle.manufacturer + " " + vehicle.type + " " +
        pathToUtf8(vehicle.configurationPath.filename()));
    return searchable.find(search) != std::string::npos;
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
        vehicleDepotCatalog_ = openbus::config::discoverAllVehicleDepots(vehiclesDirectory);
    }
    if (vehicleCatalog_.empty()) {
        throw std::runtime_error("No OMSI .bus files were found under '" +
                                 pathToUtf8(vehiclesDirectory) + "'");
    }
    if (!vehicleDepotCatalog_.empty()) {
        selectedMapDepotIndex_ =
            std::min(selectedMapDepotIndex_, vehicleDepotCatalog_.size() - 1);
    } else if (mapHofSelectionMode_ == MapHofSelectionMode::Manual) {
        mapHofSelectionMode_ = MapHofSelectionMode::Automatic;
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

bool RenderLoop::mapBusIsNearby() const {
    if (mapBusVehicle_ == nullptr) {
        return false;
    }
    const double deltaX =
        onFootState_.feetPosition[0] - mapBusVehicle_->placement.position[0];
    const double deltaY =
        onFootState_.feetPosition[1] - mapBusVehicle_->placement.position[1];
    return deltaX * deltaX + deltaY * deltaY <= 25.0;
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
    if (!spawnBusRequestPending_ || vehicleCatalog_.empty() || !mapVehicleConfig_ ||
        mapBusVehicle_ != nullptr || mapEntryPoints_.empty()) {
        spawnBusRequestPending_ = false;
        return std::nullopt;
    }
    spawnBusRequestPending_ = false;
    spawnBusError_.clear();
    selectedMapVehicleIndex_ = std::min(selectedMapVehicleIndex_, vehicleCatalog_.size() - 1);
    SpawnBusRequest request;
    request.configurationPath = vehicleCatalog_[selectedMapVehicleIndex_].configurationPath;
    selectedMapSpawnEntryPointIndex_ =
        std::min(selectedMapSpawnEntryPointIndex_, mapEntryPoints_.size() - 1);
    request.placement = mapEntryPoints_[selectedMapSpawnEntryPointIndex_].placement;
    request.spawnInBus = mapSpawnInBus_;
    const auto applyAppearance = [&](const openbus::config::VehicleAppearance& appearance) {
        request.setupOptions.ctcTextureReplacements = appearance.textureReplacements;
        request.setupOptions.appearanceVariableReplacements = appearance.variableReplacements;
        request.setupOptions.appearanceKey =
            appearance.sourcePath.generic_string() + "|" + appearance.name;
    };
    if (selectedMapAppearanceIndex_ > 0) {
        applyAppearance(mapVehicleAppearances_[selectedMapAppearanceIndex_ - 1]);
    } else {
        const std::string defaultAppearance =
            !mapVehicleConfig_->friendlyDefaultPaint.empty()
                ? mapVehicleConfig_->friendlyDefaultPaint
                : vehicleCatalog_[selectedMapVehicleIndex_].defaultAppearance;
        const openbus::config::VehicleAppearance* appearance =
            openbus::config::findVehicleAppearance(mapVehicleAppearances_, defaultAppearance);
        if (appearance != nullptr) {
            applyAppearance(*appearance);
        } else {
            request.setupOptions.appearanceKey = "default";
        }
    }
    const openbus::config::VehicleDepot* depot = nullptr;
    if (mapHofSelectionMode_ == MapHofSelectionMode::Automatic && autoMapDepotIndex_ &&
        *autoMapDepotIndex_ < vehicleDepotCatalog_.size()) {
        depot = &vehicleDepotCatalog_[*autoMapDepotIndex_];
    } else if (mapHofSelectionMode_ == MapHofSelectionMode::Manual &&
               selectedMapDepotIndex_ < vehicleDepotCatalog_.size()) {
        depot = &vehicleDepotCatalog_[selectedMapDepotIndex_];
    }
    if (depot != nullptr) {
        request.setupOptions.depotPath = depot->path;
        request.setupOptions.depotName = depot->name;
    }
    const auto registration = vehicleRegistrationSelectionAtIndex(
        *mapVehicleConfig_, selectedMapRegistrationIndex_, mapVehicleNumberInput_.data(),
        mapRegistrationInput_.data());
    request.setupOptions.vehicleNumber = registration.vehicleNumber;
    request.setupOptions.registration = registration.registration;
    return request;
}

bool RenderLoop::consumeEnterMapBusRequest() {
    const bool requested = enterMapBusRequestPending_;
    enterMapBusRequestPending_ = false;
    return requested;
}

bool RenderLoop::consumeLeaveMapBusRequest() {
    const bool requested = leaveMapBusRequestPending_;
    leaveMapBusRequestPending_ = false;
    if (requested && playerTimetable_.state) {
        gameLog.Warning("Cannot leave the player bus during an active timetable trip.");
        return false;
    }
    return requested;
}

std::optional<VehiclePlacement> RenderLoop::consumeMapBusTeleportRequest() {
    std::optional<VehiclePlacement> request = mapBusTeleportRequestPending_;
    mapBusTeleportRequestPending_.reset();
    if (request && playerTimetable_.state) {
        gameLog.Warning("Cannot teleport the player bus during an active timetable trip.");
        return std::nullopt;
    }
    return request;
}

bool RenderLoop::consumeRemovePlayerBusRequest() {
    const bool requested = removePlayerBusRequestPending_;
    removePlayerBusRequestPending_ = false;
    return requested;
}

std::optional<MapCalendarRequest> RenderLoop::consumeMapCalendarRequest() {
    std::optional<MapCalendarRequest> request = mapCalendarRequestPending_;
    mapCalendarRequestPending_.reset();
    return request;
}

bool RenderLoop::consumeQuitToMenuRequest() {
    const bool requested = quitToMenuRequested_;
    quitToMenuRequested_ = false;
    return requested;
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
    mapVehicleLoadError_.clear();
    selectedMapAppearanceIndex_ = 0;
    selectedMapRegistrationIndex_ = 0;
    mapRegistrationInput_.fill('\0');
    mapVehicleNumberInput_.fill('\0');
    try {
        TraceScope vehicleTrace("startup", "startup.loadMapSelectedVehicleConfig");
        const auto& configPath = vehicleCatalog_[selectedMapVehicleIndex_].configurationPath;
        mapVehicleConfig_ = loadVehicleConfig(configPath, VehicleFileKind::Bus, false);
        const ModelConfig modelConfiguration = loadBusModelConfiguration(configPath);
        mapVehicleAppearances_ =
            openbus::config::discoverVehicleAppearances(
                configPath, vehiclesDirectory_, modelConfiguration.ctcTemplates);
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

void RenderLoop::updateAutomaticMapHofSelection() {
    TraceScope hofTrace("startup", "startup.autoMatchMapHof");
    autoMapDepotEvaluated_ = true;
    autoMapDepotIndex_.reset();
    autoMapDepotMatchedRouteCount_ = 0;
    if (vehicleDepotCatalog_.empty() || passengerNetwork_.routes.empty()) {
        return;
    }

    for (std::size_t index = 0; index < vehicleDepotCatalog_.size(); ++index) {
        const openbus::config::VehicleDepot& depot = vehicleDepotCatalog_[index];
        try {
            const openbus::config::HofData hof = openbus::config::loadHofData(depot.path);
            const std::size_t matchedRoutes =
                openbus::app::matchingPlayerTimetableRouteCount(passengerNetwork_.routes, hof);
            if (matchedRoutes > autoMapDepotMatchedRouteCount_) {
                autoMapDepotIndex_ = index;
                autoMapDepotMatchedRouteCount_ = matchedRoutes;
            }
        } catch (const std::exception& error) {
            gameLog.Warning("Skipping HOF during map auto-matching '" +
                            pathToUtf8(depot.path) + "': " + error.what());
        }
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
    if (mapBusVehicle_ != nullptr && !pauseMenuOpen_) {
        ImGui::SetNextWindowPos(ImVec2(20.0F, 80.0F), ImGuiCond_Always);
        ImGui::SetNextWindowBgAlpha(0.82F);
        ImGui::Begin("Parked bus", nullptr,
                     ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoCollapse |
                         ImGuiWindowFlags_NoNav);
        if (mapBusIsNearby()) {
            if (ImGui::Button("Enter bus")) {
                enterMapBusRequestPending_ = true;
            }
        } else {
            ImGui::TextUnformatted("Walk closer to the parked bus to enter.");
        }
        ImGui::End();
    }
    if (openSpawnPopup_) {
        ImGui::OpenPopup("Spawn a bus");
        openSpawnPopup_ = false;
    }
    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    const float popupWidth = std::min(640.0F, std::max(1.0F, viewport->WorkSize.x - 32.0F));
    const float popupHeight = std::min(560.0F, std::max(1.0F, viewport->WorkSize.y - 32.0F));
    const float overrideInputWidth = std::min(360.0F, std::max(1.0F, popupWidth - 40.0F));
    ImGui::SetNextWindowSize(ImVec2(popupWidth, popupHeight), ImGuiCond_Appearing);
    ImGui::SetNextWindowPos(ImVec2(viewport->WorkPos.x + viewport->WorkSize.x * 0.5F,
                                   viewport->WorkPos.y + viewport->WorkSize.y * 0.5F),
                            ImGuiCond_Appearing, ImVec2(0.5F, 0.5F));
    ImGui::SetNextWindowBgAlpha(0.9F);
    if (!ImGui::BeginPopupModal("Spawn a bus", nullptr,
                                ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoNav |
                                    ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoSavedSettings)) {
        return;
    }
    ImGui::TextUnformatted("You are not in a bus.");
    drawMapEntryPointCombo(mapEntryPoints_, selectedMapSpawnEntryPointIndex_,
                           "Bus spawn location");
    ImGui::Checkbox("Spawn in bus", &mapSpawnInBus_);
    if (vehicleCatalog_.empty()) {
        ImGui::TextUnformatted("No bus configurations are available.");
    } else {
        selectedMapVehicleIndex_ = std::min(selectedMapVehicleIndex_, vehicleCatalog_.size() - 1);
        if (!loadedMapVehicleIndex_ || *loadedMapVehicleIndex_ != selectedMapVehicleIndex_) {
            loadMapVehicleOptions();
        }
        ImGui::InputTextWithHint("Search buses", "Model, type, or file name",
                                 mapVehicleSearchInput_.data(), mapVehicleSearchInput_.size());
        ImGui::SameLine();
        if (ImGui::SmallButton("Clear")) {
            mapVehicleSearchInput_.fill('\0');
        }
        const std::string search = lowercase(mapVehicleSearchInput_.data());
        struct ModelChoice {
            std::string name;
        };
        std::vector<ModelChoice> matchingModels;
        for (const openbus::config::VehicleCatalogEntry& vehicle : vehicleCatalog_) {
            if (!matchesVehicleSearch(vehicle, search)) {
                continue;
            }
            const auto existing = std::find_if(
                matchingModels.begin(), matchingModels.end(),
                [&](const ModelChoice& model) { return model.name == vehicle.manufacturer; });
            if (existing == matchingModels.end()) {
                matchingModels.push_back({vehicle.manufacturer});
            }
        }
        std::string selectedModel = vehicleCatalog_[selectedMapVehicleIndex_].manufacturer;
        if (ImGui::BeginCombo("Model", selectedModel.c_str())) {
            if (matchingModels.empty()) {
                ImGui::TextDisabled("No matching models");
            }
            for (const ModelChoice& model : matchingModels) {
                const bool selected = model.name == selectedModel;
                if (ImGui::Selectable(model.name.c_str(), selected)) {
                    const auto matchingVehicle = std::find_if(
                        vehicleCatalog_.begin(), vehicleCatalog_.end(),
                        [&](const openbus::config::VehicleCatalogEntry& vehicle) {
                            return vehicle.manufacturer == model.name &&
                                   matchesVehicleSearch(vehicle, search);
                        });
                    if (matchingVehicle != vehicleCatalog_.end()) {
                        selectedMapVehicleIndex_ = static_cast<std::size_t>(
                            matchingVehicle - vehicleCatalog_.begin());
                        selectedModel = model.name;
                        loadMapVehicleOptions();
                    }
                }
                if (selected) {
                    ImGui::SetItemDefaultFocus();
                }
            }
            ImGui::EndCombo();
        }
        const auto vehicleTypeLabel = [](const openbus::config::VehicleCatalogEntry& vehicle) {
            return vehicle.type + " (" + pathToUtf8(vehicle.configurationPath.filename()) + ")";
        };
        const std::string selectedTypeLabel =
            vehicleTypeLabel(vehicleCatalog_[selectedMapVehicleIndex_]);
        if (ImGui::BeginCombo("Bus type", selectedTypeLabel.c_str())) {
            bool hasMatchingType = false;
            for (std::size_t index = 0; index < vehicleCatalog_.size(); ++index) {
                const openbus::config::VehicleCatalogEntry& vehicle = vehicleCatalog_[index];
                if (vehicle.manufacturer != selectedModel ||
                    !matchesVehicleSearch(vehicle, search)) {
                    continue;
                }
                hasMatchingType = true;
                const std::string label = vehicleTypeLabel(vehicle);
                const bool selected = index == selectedMapVehicleIndex_;
                if (ImGui::Selectable(label.c_str(), selected)) {
                    selectedMapVehicleIndex_ = index;
                    loadMapVehicleOptions();
                }
                if (selected) {
                    ImGui::SetItemDefaultFocus();
                }
            }
            if (!hasMatchingType) {
                ImGui::TextDisabled("No matching bus types");
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

            const auto depotLabel = [&](const openbus::config::VehicleDepot& depot) {
                std::filesystem::path relativePath = depot.path.lexically_relative(vehiclesDirectory_);
                if (relativePath.empty()) {
                    relativePath = depot.path;
                }
                return depot.name + " (" + pathToUtf8(relativePath) + ")";
            };
            std::string currentDepot = "Auto-match this map";
            if (mapHofSelectionMode_ == MapHofSelectionMode::None) {
                currentDepot = "No HOF";
            } else if (mapHofSelectionMode_ == MapHofSelectionMode::Manual &&
                       selectedMapDepotIndex_ < vehicleDepotCatalog_.size()) {
                currentDepot = depotLabel(vehicleDepotCatalog_[selectedMapDepotIndex_]);
            } else if (autoMapDepotIndex_ &&
                       *autoMapDepotIndex_ < vehicleDepotCatalog_.size()) {
                currentDepot = "Auto: " + depotLabel(vehicleDepotCatalog_[*autoMapDepotIndex_]);
            } else if (autoMapDepotEvaluated_) {
                currentDepot = "Auto: no matching HOF found";
            }
            if (ImGui::BeginCombo("Timetable HOF", currentDepot.c_str())) {
                const bool automaticSelected =
                    mapHofSelectionMode_ == MapHofSelectionMode::Automatic;
                if (ImGui::Selectable("Auto-match this map", automaticSelected)) {
                    mapHofSelectionMode_ = MapHofSelectionMode::Automatic;
                }
                if (automaticSelected) {
                    ImGui::SetItemDefaultFocus();
                }
                const bool noHofSelected = mapHofSelectionMode_ == MapHofSelectionMode::None;
                if (ImGui::Selectable("None (drive without a HOF)", noHofSelected)) {
                    mapHofSelectionMode_ = MapHofSelectionMode::None;
                }
                if (noHofSelected) {
                    ImGui::SetItemDefaultFocus();
                }
                if (!vehicleDepotCatalog_.empty()) {
                    ImGui::Separator();
                    ImGui::TextDisabled("Choose an HOF from any installed bus");
                }
                for (std::size_t index = 0; index < vehicleDepotCatalog_.size(); ++index) {
                    const bool selected = mapHofSelectionMode_ == MapHofSelectionMode::Manual &&
                                          selectedMapDepotIndex_ == index;
                    const std::string label = depotLabel(vehicleDepotCatalog_[index]);
                    if (ImGui::Selectable(label.c_str(), selected)) {
                        mapHofSelectionMode_ = MapHofSelectionMode::Manual;
                        selectedMapDepotIndex_ = index;
                    }
                    if (selected) {
                        ImGui::SetItemDefaultFocus();
                    }
                }
                ImGui::EndCombo();
            }
            if (mapHofSelectionMode_ == MapHofSelectionMode::Automatic) {
                if (autoMapDepotIndex_ && *autoMapDepotIndex_ < vehicleDepotCatalog_.size()) {
                    const std::string label = depotLabel(vehicleDepotCatalog_[*autoMapDepotIndex_]);
                    ImGui::TextWrapped("Auto-selected %s; it matches %zu map timetable routes.",
                                       label.c_str(), autoMapDepotMatchedRouteCount_);
                } else if (passengerNetwork_.routes.empty()) {
                    ImGui::TextWrapped(
                        "This map has no timetable routes to match automatically. You can still "
                        "spawn and drive, or choose an HOF manually.");
                } else if (autoMapDepotEvaluated_) {
                    ImGui::TextWrapped(
                        "No installed HOF matches the map timetable routes. You can still spawn "
                        "and drive; select an HOF manually if you want scheduled trips.");
                } else {
                    ImGui::TextUnformatted("HOF matching will run after map timetables load.");
                }
            } else if (mapHofSelectionMode_ == MapHofSelectionMode::Manual) {
                ImGui::TextUnformatted(
                    "This HOF is used regardless of which bus model you spawn.");
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
                ImGui::SetNextItemWidth(overrideInputWidth);
                if (ImGui::InputText("Vehicle number override", mapVehicleNumberInput_.data(),
                                     mapVehicleNumberInput_.size())) {
                    persistMapVehiclePreference();
                }
                ImGui::SetNextItemWidth(overrideInputWidth);
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
    ImGui::BeginDisabled(vehicleCatalog_.empty() || !mapVehicleConfig_ ||
                         mapBusVehicle_ != nullptr || mapEntryPoints_.empty());
    if (ImGui::Button(mapSpawnInBus_ ? "Spawn and enter bus" : "Spawn bus on foot",
                      ImVec2(220.0F, 36.0F))) {
        spawnBusRequestPending_ = true;
    }
    ImGui::EndDisabled();
    if (!spawnBusError_.empty()) {
        ImGui::TextWrapped("Unable to spawn bus: %s", spawnBusError_.c_str());
    }
    ImGui::TextUnformatted(
        "Use the trip panel at the top of the screen to start a scheduled trip.");
    ImGui::TextUnformatted("WASD: walk | Space: jump | F4: toggle mouse look");
    ImGui::TextUnformatted("Press F4 to release the cursor and use this panel.");
    if (ImGui::Button("Close")) {
        ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
}

void RenderLoop::drawPlayerTimetableSelectionUi() {
    selectedPlayerTimetableSelection_.reset();
    {
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
                    const auto cachedTrip =
                        std::ranges::find(trips, selectedPlayerCourseTripIndex_,
                                          &openbus::app::PlayerTimetableCatalogTrip::tripIndex);
                    {
                        const auto tripDisplayLabel = [](const std::string& tripName,
                                                         double departureMinutes) {
                            const int minuteOfDay = static_cast<int>(std::floor(departureMinutes));
                            const int minute = minuteOfDay % 60;
                            return (tripName.empty() ? std::string("Trip") : tripName) + " (" +
                                   std::to_string((minuteOfDay / 60) % 24) + ":" +
                                   (minute < 10 ? "0" : "") + std::to_string(minute) + ")";
                        };
                        const auto tripOptionLabel = [&](const auto& option) {
                            const auto& optionTrip = course.trips[option.tripIndex];
                            const auto& optionRoute =
                                passengerNetwork_.routes[option.routeIndex];
                            std::string label = tripDisplayLabel(
                                optionRoute.tripName, optionTrip.departureMinutes);
                            const std::size_t duplicateLabelCount = static_cast<std::size_t>(
                                std::ranges::count_if(trips, [&](const auto& candidate) {
                                    const auto& candidateTrip = course.trips[candidate.tripIndex];
                                    const auto& candidateRoute =
                                        passengerNetwork_.routes[candidate.routeIndex];
                                    return tripDisplayLabel(candidateRoute.tripName,
                                                            candidateTrip.departureMinutes) ==
                                           label;
                                }));
                            if (duplicateLabelCount > 1) {
                                label += " [" + optionRoute.timetableKey + " / profile " +
                                         std::to_string(optionTrip.profileIndex) + "]";
                            }
                            return label;
                        };
                        const std::string tripLabel = tripOptionLabel(*cachedTrip);
                        if (ImGui::BeginCombo("Trip", tripLabel.c_str())) {
                            for (const auto& option : trips) {
                                const std::size_t tripIndex = option.tripIndex;
                                const std::string label = tripOptionLabel(option);
                                const bool selected = tripIndex == selectedPlayerCourseTripIndex_;
                                ImGui::PushID(static_cast<int>(tripIndex));
                                if (ImGui::Selectable(label.c_str(), selected)) {
                                    selectedPlayerCourseTripIndex_ = tripIndex;
                                    selectedPlayerFirstStopIndex_ = 0;
                                }
                                if (selected) {
                                    ImGui::SetItemDefaultFocus();
                                }
                                ImGui::PopID();
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
                                    const std::string label =
                                        selectedRoute.stops[stopIndex].name +
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
                            selectedPlayerTimetableSelection_ = SpawnBusRequest::TimetableSelection{
                                selectedTrip->routeIndex, selectedPlayerCourseIndex_,
                                selectedPlayerCourseTripIndex_, selectedPlayerFirstStopIndex_,
                                selectedPlayerDriveAsAdditionalBus_};
                            const auto startingPose =
                                selectedRoute.stopPose(selectedPlayerFirstStopIndex_);
                            ImGui::BeginDisabled(!mapBusVehicle_ || !startingPose);
                            if (ImGui::Button("Teleport bus to selected starting point") &&
                                startingPose) {
                                mapBusTeleportRequestPending_ = VehiclePlacement{
                                    startingPose->position,
                                    openbus::map::mapTrafficHeadingToVehicleYawDegrees(
                                        startingPose->headingRadians),
                                    startingPose->pitchRadians * 180.0 / std::numbers::pi,
                                    startingPose->rollRadians * 180.0 / std::numbers::pi};
                            }
                            ImGui::EndDisabled();
                            if (!startingPose) {
                                ImGui::TextUnformatted(
                                    "The selected starting stop has no usable map position.");
                            } else if (!mapBusVehicle_) {
                                ImGui::TextUnformatted(
                                    "Teleport is available when using the parked map bus.");
                            }
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
                ImGui::TextUnformatted(
                    "Select a compatible Timetable HOF when spawning the bus.");
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
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndDisabled();
        ImGui::SameLine();
        if (ImGui::Button("Cancel selection")) {
            selectedPlayerTimetableSelection_.reset();
            ImGui::CloseCurrentPopup();
        }
    }
}

void RenderLoop::drawPlayerTimetableUi() {
    if (!imguiEnabled_ || playerVehicle_ == nullptr || renderingReflection_ || captureMode_) {
        return;
    }
    const auto drawTripSelectionPopup = [this] {
        if (openPlayerTimetablePopup_) {
            ImGui::OpenPopup("Drive a scheduled trip");
            openPlayerTimetablePopup_ = false;
        }
        if (ImGui::BeginPopupModal("Drive a scheduled trip", nullptr,
                                   ImGuiWindowFlags_AlwaysAutoResize |
                                       ImGuiWindowFlags_NoCollapse)) {
            drawPlayerTimetableSelectionUi();
            ImGui::EndPopup();
        }
    };
    if (!playerTimetable_.state) {
        drawTripSelectionPopup();
        return;
    }
    ImGui::SetNextWindowPos(ImVec2(20.0F, 20.0F), ImGuiCond_Always);
    ImGui::SetNextWindowBgAlpha(0.82F);
    ImGui::Begin("Player timetable", nullptr,
                 ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoCollapse |
                     ImGuiWindowFlags_NoNav);
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
        selectedPlayerTimetableSelection_.reset();
    }
    ImGui::End();
}

void RenderLoop::drawPauseMenuUi() {
    if (!imguiEnabled_ || !pauseMenuOpen_ || renderingReflection_ || captureMode_) {
        return;
    }

    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(ImVec2(viewport->WorkPos.x + viewport->WorkSize.x * 0.5F,
                                   viewport->WorkPos.y + viewport->WorkSize.y * 0.5F),
                            ImGuiCond_Always, ImVec2(0.5F, 0.5F));
    ImGui::SetNextWindowBgAlpha(0.95F);
    if (!ImGui::Begin("Pause Menu", nullptr,
                      ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoCollapse |
                          ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings)) {
        ImGui::End();
        return;
    }

    if (ImGui::Button("Resume", ImVec2(260.0F, 36.0F))) {
        pauseMenuOpen_ = false;
        setPausedState(false);
    }
    ImGui::Separator();
    if (playerTimetable_.state) {
        if (playerTimetable_.state->stopAfterCurrentTrip) {
            ImGui::TextUnformatted("Driving will stop at this trip's terminus.");
        } else if (ImGui::Button("Stop driving after this trip", ImVec2(260.0F, 0.0F))) {
            playerTimetable_.state->stopAfterCurrentTrip = true;
        }
        if (ImGui::Button("Stop driving immediately", ImVec2(260.0F, 0.0F))) {
            SetPlayerTimetableSelection(std::nullopt);
            selectedPlayerTimetableSelection_.reset();
        }
    } else if (playerVehicle_ != nullptr) {
        if (ImGui::Button("Leave Bus", ImVec2(260.0F, 36.0F))) {
            leaveMapBusRequestPending_ = true;
            pauseMenuOpen_ = false;
            setPausedState(false);
        }
        if (ImGui::Button("Remove Bus", ImVec2(260.0F, 36.0F))) {
            removePlayerBusRequestPending_ = true;
            pauseMenuOpen_ = false;
        }
    } else if (mapBusVehicle_ != nullptr) {
        if (mapBusIsNearby()) {
            if (ImGui::Button("Enter Bus", ImVec2(260.0F, 36.0F))) {
                enterMapBusRequestPending_ = true;
                pauseMenuOpen_ = false;
                setPausedState(false);
            }
        } else {
            ImGui::TextUnformatted("Walk closer to the parked bus to enter.");
        }
        if (ImGui::Button("Remove Bus", ImVec2(260.0F, 36.0F))) {
            removePlayerBusRequestPending_ = true;
            pauseMenuOpen_ = false;
        }
    } else if (ImGui::Button("Spawn Bus", ImVec2(260.0F, 36.0F))) {
        openSpawnPopup_ = true;
        pauseMenuOpen_ = false;
        setPausedState(false);
    }

    if (!playerTimetable_.state) {
        ImGui::Separator();
        drawMapEntryPointCombo(mapEntryPoints_, selectedMapSpawnEntryPointIndex_,
                               "Bus location");
        if (mapBusVehicle_ != nullptr) {
            if (ImGui::Button("Teleport Bus to selected location", ImVec2(260.0F, 36.0F))) {
                if (selectedMapSpawnEntryPointIndex_ < mapEntryPoints_.size()) {
                    mapBusTeleportRequestPending_ =
                        mapEntryPoints_[selectedMapSpawnEntryPointIndex_].placement;
                }
            }
        } else {
            ImGui::TextUnformatted("This location will be used next time you spawn a bus.");
        }
    }

    ImGui::Separator();
    if (ImGui::Button("Debug Menu", ImVec2(260.0F, 0.0F))) {
        debugMenuOpen_ = !debugMenuOpen_;
        debugCalendarInitialized_ = false;
    }
    if (ImGui::Button("Quit to Menu", ImVec2(260.0F, 0.0F))) {
        quitToMenuRequested_ = true;
        pauseMenuOpen_ = false;
    }
    if (ImGui::Button("Quit Game", ImVec2(260.0F, 0.0F))) {
        requestClose();
    }
    ImGui::TextUnformatted("Esc: resume or open/close this menu | P: pause/resume");
    ImGui::End();
}

void RenderLoop::drawDebugMenuUi() {
    if (!imguiEnabled_ || !debugMenuOpen_ || renderingReflection_ || captureMode_) {
        return;
    }

    SystemVariables& calendar = simulationState_.sharedVariables();
    if (!debugCalendarInitialized_) {
        debugCalendarYear_ = static_cast<int>(calendar.get("year"));
        debugCalendarMonth_ = static_cast<int>(calendar.get("month"));
        debugCalendarDay_ = static_cast<int>(calendar.get("day"));
        const int timeSeconds = static_cast<int>(calendar.get("time"));
        debugCalendarHour_ = timeSeconds / 3600;
        debugCalendarMinute_ = (timeSeconds % 3600) / 60;
        debugCalendarInitialized_ = true;
    }

    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(ImVec2(viewport->WorkPos.x + viewport->WorkSize.x - 20.0F,
                                   viewport->WorkPos.y + 20.0F),
                            ImGuiCond_FirstUseEver, ImVec2(1.0F, 0.0F));
    ImGui::SetNextWindowBgAlpha(0.95F);
    if (!ImGui::Begin("OpenBus Debug Menu", &debugMenuOpen_,
                      ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoCollapse |
                          ImGuiWindowFlags_NoSavedSettings)) {
        ImGui::End();
        return;
    }

    const int currentTime = static_cast<int>(calendar.get("time"));
    ImGui::Text("Current map time: %04d-%02d-%02d %02d:%02d",
                static_cast<int>(calendar.get("year")), static_cast<int>(calendar.get("month")),
                static_cast<int>(calendar.get("day")), currentTime / 3600,
                (currentTime % 3600) / 60);
    ImGui::Separator();
    ImGui::TextUnformatted("Set map date and time");
    ImGui::SetNextItemWidth(100.0F);
    ImGui::InputInt("Year", &debugCalendarYear_, 1, 1);
    ImGui::SetNextItemWidth(100.0F);
    ImGui::InputInt("Month", &debugCalendarMonth_, 1, 1);
    ImGui::SetNextItemWidth(100.0F);
    ImGui::InputInt("Day", &debugCalendarDay_, 1, 1);
    ImGui::SetNextItemWidth(100.0F);
    ImGui::InputInt("Hour", &debugCalendarHour_, 1, 1);
    ImGui::SetNextItemWidth(100.0F);
    ImGui::InputInt("Minute", &debugCalendarMinute_, 1, 1);

    const std::chrono::year_month_day selectedDate{
        std::chrono::year{std::clamp(debugCalendarYear_, 1, 9999)},
        std::chrono::month{static_cast<unsigned>(std::clamp(debugCalendarMonth_, 1, 12))},
        std::chrono::day{static_cast<unsigned>(std::clamp(debugCalendarDay_, 1, 31))}};
    const bool validDateTime = debugCalendarYear_ >= 1 && debugCalendarYear_ <= 9999 &&
                               debugCalendarMonth_ >= 1 && debugCalendarMonth_ <= 12 &&
                               debugCalendarDay_ >= 1 && debugCalendarDay_ <= 31 &&
                               selectedDate.ok() && debugCalendarHour_ >= 0 &&
                               debugCalendarHour_ < 24 && debugCalendarMinute_ >= 0 &&
                               debugCalendarMinute_ < 60;
    if (!validDateTime) {
        ImGui::TextColored(ImVec4(1.0F, 0.45F, 0.35F, 1.0F),
                           "Enter a valid date and time.");
    }
    ImGui::BeginDisabled(!validDateTime);
    if (ImGui::Button("Apply date and time")) {
        mapCalendarRequestPending_ = MapCalendarRequest{
            debugCalendarYear_, debugCalendarMonth_, debugCalendarDay_,
            static_cast<double>(debugCalendarHour_ * 3600 + debugCalendarMinute_ * 60)};
    }
    ImGui::EndDisabled();

    if (playerTimetable_.state &&
        playerTimetable_.state->selection.courseIndex < passengerTimetableCourses_.size()) {
        const auto& trips =
            passengerTimetableCourses_[playerTimetable_.state->selection.courseIndex].trips;
        const std::size_t tripIndex = playerTimetable_.state->selection.courseTripIndex;
        if (tripIndex < trips.size()) {
            const double departureSeconds = trips[tripIndex].departureMinutes * 60.0;
            if (std::isfinite(departureSeconds) && departureSeconds >= 0.0 &&
                departureSeconds < 86400.0 && departureSeconds > calendar.get("time")) {
                const int departureMinute =
                    static_cast<int>(std::floor(departureSeconds / 60.0));
                if (ImGui::Button("Skip to scheduled departure")) {
                    mapCalendarRequestPending_ = MapCalendarRequest{
                        static_cast<int>(calendar.get("year")),
                        static_cast<int>(calendar.get("month")),
                        static_cast<int>(calendar.get("day")),
                        static_cast<double>(departureMinute * 60)};
                    debugCalendarInitialized_ = false;
                }
            }
        }
    }
    ImGui::End();
}
