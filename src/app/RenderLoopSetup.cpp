#include "app/RenderLoopInternal.h"
#include "app/CustomWeatherUi.h"
#include "app/ImGuiLayer.h"
#include "app/ScopedSwapInterval.h"
#include "app/SteamPresence.h"
#include "app/traffic/PlayerTimetableDiagnostics.h"
#include "config/VehicleCatalog.h"
#include "core/Environment.h"
#include "core/Logger.h"
#include "core/Utf8Path.h"
#include "map/MapAssetValidation.h"
#include "physics/BusConfiguration.h"
#include "render/AssetRequestManager.h"

#include <imgui.h>

#include <algorithm>
#include <array>

#include <cctype>
#include <chrono>
#include <ctime>
#include <future>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <numbers>
#include <string>
#include <system_error>

namespace {

using openbus::pathToUtf8;

struct AssetValidationProgressState {
    std::mutex mutex;
    openbus::map::MapAssetValidationProgress progress;
};

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
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char character) {
        return static_cast<char>(std::tolower(character));
    });
    return value;
}

bool matchesVehicleSearch(const openbus::config::VehicleCatalogEntry& vehicle,
                          const std::string& search) {
    if (search.empty()) {
        return true;
    }
    const std::string searchable = lowercase(vehicle.manufacturer + " " + vehicle.type + " " +
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
        selectedMapDepotIndex_ = std::min(selectedMapDepotIndex_, vehicleDepotCatalog_.size() - 1);
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
    std::future<openbus::map::MapAssetValidationReport> assetValidationRequest;
    std::filesystem::path assetValidationMap;
    std::optional<openbus::map::MapAssetValidationReport> assetValidationReport;
    std::shared_ptr<AssetValidationProgressState> assetValidationProgress;
    std::string assetValidationError;
    bool showAssetValidationWindow = false;
    int selectedAssetFinding = -1;
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
            if (selectedDefinition->environment.startDateYyyyMmDd.has_value()) {
                const int startDate = *selectedDefinition->environment.startDateYyyyMmDd;
                year = startDate / 10000;
                month = (startDate / 100) % 100;
                day = startDate % 100;
            }
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
        if (assetValidationRequest.valid() &&
            assetValidationRequest.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            try {
                assetValidationReport = assetValidationRequest.get();
                if (!assetValidationReport->complete) {
                    gameLog.Warning("Map asset check is incomplete for '" +
                                    pathToUtf8(assetValidationMap) + "'.");
                }
            } catch (const std::exception& error) {
                assetValidationError = error.what();
                gameLog.Error("Map asset check failed for '" + pathToUtf8(assetValidationMap) +
                              "': " + error.what());
            }
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
        ImGui::SameLine();
        ImGui::BeginDisabled(assetValidationRequest.valid());
        if (ImGui::Button("Check assets")) {
            assetValidationMap = maps[selectedMapIndex];
            assetValidationReport.reset();
            assetValidationError.clear();
            selectedAssetFinding = -1;
            showAssetValidationWindow = true;
            try {
                const auto mapDirectory = assetValidationMap;
                const auto rootDirectory = omsiRoot;
                assetValidationProgress = std::make_shared<AssetValidationProgressState>();
                const auto progressState = assetValidationProgress;
                assetValidationRequest = assetRequestManager_->submit([mapDirectory, rootDirectory,
                                                                       progressState] {
                    openbus::map::MapAssetValidationOptions options;
                    options.onProgress = [progressState](const auto& progress) {
                        std::lock_guard lock(progressState->mutex);
                        progressState->progress = progress;
                    };
                    return openbus::map::validateMapAssets(mapDirectory, rootDirectory, options);
                });
            } catch (const std::exception& error) {
                assetValidationError = error.what();
                gameLog.Error("Could not start map asset check: " + assetValidationError);
            }
        }
        ImGui::EndDisabled();
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

        if (showAssetValidationWindow) {
            ImGui::SetNextWindowSize(ImVec2(760.0F, 600.0F), ImGuiCond_FirstUseEver);
            if (ImGui::Begin("Map asset check", &showAssetValidationWindow)) {
                ImGui::Text("Map: %s", pathToUtf8(assetValidationMap).c_str());
                if (assetValidationRequest.valid()) {
                    openbus::map::MapAssetValidationProgress progress;
                    if (assetValidationProgress) {
                        std::lock_guard lock(assetValidationProgress->mutex);
                        progress = assetValidationProgress->progress;
                    }
                    ImGui::Text("%s", progress.phase.empty() ? "Checking map assets..."
                                                             : progress.phase.c_str());
                    if (progress.total > 0) {
                        const float fraction = std::clamp(static_cast<float>(progress.completed) /
                                                              static_cast<float>(progress.total),
                                                          0.0F, 1.0F);
                        ImGui::ProgressBar(fraction, ImVec2(-1.0F, 0.0F));
                    } else if (progress.completed > 0) {
                        ImGui::Text("Processed %zu items", progress.completed);
                    }
                } else if (!assetValidationError.empty()) {
                    ImGui::TextColored(ImVec4(1.0F, 0.4F, 0.3F, 1.0F), "Check failed");
                    ImGui::TextWrapped("%s", assetValidationError.c_str());
                } else if (assetValidationReport.has_value()) {
                    const auto& report = *assetValidationReport;
                    ImGui::Text("Scanned %zu base tiles, %zu independent Chrono tiles, "
                                "%zu combined Chrono tiles, and %zu timetable dates.",
                                report.tilesScanned, report.chronoTilesScanned,
                                report.chronoCombinedTilesScanned,
                                report.chronoTimetableDatesScanned);
                    if (report.complete) {
                        ImGui::TextColored(ImVec4(0.35F, 0.85F, 0.4F, 1.0F),
                                           "Scan complete: %zu finding(s).",
                                           report.findings.size());
                    } else {
                        ImGui::TextColored(
                            ImVec4(1.0F, 0.75F, 0.25F, 1.0F),
                            "INCOMPLETE: a clean result does not certify full OMSI coverage.");
                        ImGui::Text("%zu finding(s).", report.findings.size());
                    }

                    std::map<std::string, std::size_t> findingCounts;
                    for (const auto& finding : report.findings) {
                        ++findingCounts[finding.category];
                    }
                    for (const auto& [category, count] : findingCounts) {
                        ImGui::SameLine();
                        ImGui::Text("%s: %zu", category.c_str(), count);
                    }

                    const int findingCount = static_cast<int>(
                        std::min(report.findings.size(),
                                 static_cast<std::size_t>((std::numeric_limits<int>::max)())));
                    ImGui::BeginChild("asset-check-findings", ImVec2(0.0F, -150.0F), true);
                    ImGuiListClipper clipper;
                    clipper.Begin(findingCount);
                    while (clipper.Step()) {
                        for (int index = clipper.DisplayStart; index < clipper.DisplayEnd;
                             ++index) {
                            const auto& finding = report.findings[static_cast<std::size_t>(index)];
                            const std::string label =
                                std::string(finding.unsupported ? "UNSUPPORTED " : "") +
                                finding.category + ": " +
                                (finding.reference.empty() ? finding.detail : finding.reference);
                            if (ImGui::Selectable(label.c_str(), selectedAssetFinding == index)) {
                                selectedAssetFinding = index;
                            }
                        }
                    }
                    ImGui::EndChild();
                    if (selectedAssetFinding >= 0 &&
                        static_cast<std::size_t>(selectedAssetFinding) < report.findings.size()) {
                        const auto& finding =
                            report.findings[static_cast<std::size_t>(selectedAssetFinding)];
                        if (finding.unsupported) {
                            ImGui::TextColored(ImVec4(1.0F, 0.75F, 0.25F, 1.0F),
                                               "UNSUPPORTED BY OPENBUS");
                        }
                        ImGui::TextWrapped("%s", finding.detail.c_str());
                        if (!finding.reference.empty()) {
                            ImGui::TextWrapped("Reference: %s", finding.reference.c_str());
                        }
                        ImGui::TextWrapped("Source: %s", pathToUtf8(finding.source).c_str());
                        if (!finding.context.empty()) {
                            ImGui::TextWrapped("Context: %s", pathToUtf8(finding.context).c_str());
                        }
                    }
                } else {
                    ImGui::TextWrapped(
                        "Select a map and click Check assets. This scan does not yet cover every "
                        "OMSI format; the README lists known unsupported content.");
                }
                if (ImGui::TreeNode("Known OpenBus support gaps")) {
                    ImGui::BulletText(
                        "Animated or skinned .x content, complete .sli semantics, cross-tile "
                        "repeater and attachment behavior, and scenery scripting are partial.");
                    ImGui::BulletText(
                        "Water rendering, dynamic terrain and lightmaps, and tile or seasonal "
                        "ground-texture rendering are not implemented.");
                    ImGui::BulletText(
                        "Chrono and timetable parity is incomplete; the checker also misses some "
                        "runtime repaint/free-texture variants, sound directives, and audio "
                        "formats beyond WAV.");
                    ImGui::TextWrapped(
                        "Unknown sections retained from global.cfg and tile files are shown as "
                        "UNSUPPORTED findings. This does not discover every unsupported "
                        "reference or certify complete OMSI compatibility. See the README for "
                        "the detailed support notes.");
                    ImGui::TreePop();
                }
            }
            ImGui::End();
        }
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
    const double deltaX = onFootState_.feetPosition[0] - mapBusVehicle_->placement.position[0];
    const double deltaY = onFootState_.feetPosition[1] - mapBusVehicle_->placement.position[1];
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
        mapVehicleAppearances_ = openbus::config::discoverVehicleAppearances(
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
            openbus::config::HofData hof;
            {
                TraceScope loadTrace("startup", "startup.autoMatchMapHof.loadHof");
                hof = openbus::config::loadHofData(depot.path);
            }
            TraceScope matchTrace("startup", "startup.autoMatchMapHof.matchRoutes");
            const std::size_t matchedRoutes =
                openbus::app::matchingPlayerTimetableRouteCount(passengerNetwork_.routes, hof);
            if (matchedRoutes > autoMapDepotMatchedRouteCount_) {
                autoMapDepotIndex_ = index;
                autoMapDepotMatchedRouteCount_ = matchedRoutes;
            }
        } catch (const std::exception& error) {
            gameLog.Warning("Skipping HOF during map auto-matching '" + pathToUtf8(depot.path) +
                            "': " + error.what());
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
    drawMapEntryPointCombo(mapEntryPoints_, selectedMapSpawnEntryPointIndex_, "Bus spawn location");
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
                    const auto matchingVehicle =
                        std::find_if(vehicleCatalog_.begin(), vehicleCatalog_.end(),
                                     [&](const openbus::config::VehicleCatalogEntry& vehicle) {
                                         return vehicle.manufacturer == model.name &&
                                                matchesVehicleSearch(vehicle, search);
                                     });
                    if (matchingVehicle != vehicleCatalog_.end()) {
                        selectedMapVehicleIndex_ =
                            static_cast<std::size_t>(matchingVehicle - vehicleCatalog_.begin());
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
                std::filesystem::path relativePath =
                    depot.path.lexically_relative(vehiclesDirectory_);
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
            } else if (autoMapDepotIndex_ && *autoMapDepotIndex_ < vehicleDepotCatalog_.size()) {
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
                ImGui::TextUnformatted("This HOF is used regardless of which bus model you spawn.");
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
                            const auto& optionRoute = passengerNetwork_.routes[option.routeIndex];
                            std::string label =
                                tripDisplayLabel(optionRoute.tripName, optionTrip.departureMinutes);
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
                ImGui::TextUnformatted("Select a compatible Timetable HOF when spawning the bus.");
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

void RenderLoop::previewDisplaySettings() {
    if (displayPreviewPending_)
        return;

    int windowX = 0;
    int windowY = 0;
    glfwGetWindowPos(window_, &windowX, &windowY);
    glfwGetWindowSize(window_, &displayPreviewPreviousWidth_, &displayPreviewPreviousHeight_);
    displayPreviewPreviousX_ = windowX;
    displayPreviewPreviousY_ = windowY;
    displayPreviewPreviousDecorated_ = glfwGetWindowAttrib(window_, GLFW_DECORATED) == GLFW_TRUE;
    displayPreviewPreviousMonitor_ = glfwGetWindowMonitor(window_);
    displayPreviewPreviousRefreshRate_ = GLFW_DONT_CARE;
    if (displayPreviewPreviousMonitor_ != nullptr) {
        const GLFWvidmode* currentMode = glfwGetVideoMode(displayPreviewPreviousMonitor_);
        if (currentMode != nullptr) {
            displayPreviewPreviousRefreshRate_ = currentMode->refreshRate;
        }
    }
    displayPreviewPreviousPreferences_ = activeGraphicsPreferences_;

    GLFWmonitor* targetMonitor = nullptr;
    int targetX = displayPreviewPreviousX_;
    int targetY = displayPreviewPreviousY_;
    int targetWidth = graphicsPreferences_.values.width;
    int targetHeight = graphicsPreferences_.values.height;
    int targetRefreshRate = GLFW_DONT_CARE;
    const auto& preferences = graphicsPreferences_.values;
    int monitorCount = 0;
    GLFWmonitor** monitors = glfwGetMonitors(&monitorCount);
    GLFWmonitor* selectedMonitor = nullptr;
    if (monitors != nullptr && monitorCount > 0) {
        const int monitorIndex =
            preferences.monitor < monitorCount ? std::max(preferences.monitor, 0) : 0;
        selectedMonitor = monitors[monitorIndex];
    }

    if (preferences.windowMode != "windowed") {
        if (selectedMonitor == nullptr) {
            displayPreviewError_ = "No display monitor is available for fullscreen modes.";
            return;
        }
        if (preferences.windowMode == "borderless") {
            const GLFWvidmode* desktopMode = glfwGetVideoMode(selectedMonitor);
            if (desktopMode == nullptr) {
                displayPreviewError_ = "The selected monitor has no available display mode.";
                return;
            }
            targetWidth = desktopMode->width;
            targetHeight = desktopMode->height;
            glfwGetMonitorPos(selectedMonitor, &targetX, &targetY);
        } else {
            targetMonitor = selectedMonitor;
            int modeCount = 0;
            const GLFWvidmode* modes = glfwGetVideoModes(targetMonitor, &modeCount);
            const GLFWvidmode* selectedMode = nullptr;
            for (int index = 0; modes != nullptr && index < modeCount; ++index) {
                if (modes[index].width == preferences.width &&
                    modes[index].height == preferences.height &&
                    modes[index].refreshRate == preferences.refreshRate) {
                    selectedMode = &modes[index];
                    break;
                }
            }
            if (selectedMode == nullptr) {
                displayPreviewError_ = "The selected resolution and refresh rate are not "
                                       "available on this monitor.";
                return;
            }
            targetWidth = selectedMode->width;
            targetHeight = selectedMode->height;
            targetRefreshRate = selectedMode->refreshRate;
            targetX = 0;
            targetY = 0;
        }
    } else if (selectedMonitor != nullptr) {
        const GLFWvidmode* desktopMode = glfwGetVideoMode(selectedMonitor);
        if (desktopMode != nullptr) {
            int monitorX = 0;
            int monitorY = 0;
            glfwGetMonitorPos(selectedMonitor, &monitorX, &monitorY);
            targetX = monitorX + (desktopMode->width - targetWidth) / 2;
            targetY = monitorY + (desktopMode->height - targetHeight) / 2;
        }
    }

    glfwSetWindowAttrib(window_, GLFW_DECORATED,
                        preferences.windowMode == "borderless" ? GLFW_FALSE : GLFW_TRUE);
    glfwSetWindowMonitor(window_, targetMonitor, targetX, targetY, targetWidth, targetHeight,
                         targetRefreshRate);

    int actualWidth = 0;
    int actualHeight = 0;
    glfwGetWindowSize(window_, &actualWidth, &actualHeight);
    const bool isExclusive = preferences.windowMode == "exclusive";
    if ((isExclusive && glfwGetWindowMonitor(window_) == nullptr) ||
        (!isExclusive && glfwGetWindowMonitor(window_) != nullptr) || actualWidth != targetWidth ||
        actualHeight != targetHeight) {
        revertDisplaySettingsPreview();
        displayPreviewError_ = "The display mode could not be applied; the previous mode was "
                               "restored.";
        return;
    }

    displayPreviewError_.clear();
    displayPreviewDeadline_ = glfwGetTime() + 15.0;
    displayPreviewPending_ = true;
    ImGui::OpenPopup("Confirm display settings");
}

void RenderLoop::revertDisplaySettingsPreview() {
    GLFWmonitor* restoreMonitor = displayPreviewPreviousMonitor_;
    if (restoreMonitor != nullptr) {
        int monitorCount = 0;
        GLFWmonitor** monitors = glfwGetMonitors(&monitorCount);
        const bool stillConnected =
            monitors != nullptr &&
            std::find(monitors, monitors + monitorCount, restoreMonitor) != monitors + monitorCount;
        if (!stillConnected)
            restoreMonitor = nullptr;
    }
    glfwSetWindowAttrib(window_, GLFW_DECORATED,
                        displayPreviewPreviousDecorated_ ? GLFW_TRUE : GLFW_FALSE);
    glfwSetWindowMonitor(
        window_, restoreMonitor, displayPreviewPreviousX_, displayPreviewPreviousY_,
        displayPreviewPreviousWidth_, displayPreviewPreviousHeight_,
        restoreMonitor == nullptr ? GLFW_DONT_CARE : displayPreviewPreviousRefreshRate_);
    auto& preferences = graphicsPreferences_.values;
    preferences.windowMode = displayPreviewPreviousPreferences_.windowMode;
    preferences.width = displayPreviewPreviousPreferences_.width;
    preferences.height = displayPreviewPreviousPreferences_.height;
    preferences.refreshRate = displayPreviewPreviousPreferences_.refreshRate;
    preferences.monitor = displayPreviewPreviousPreferences_.monitor;
    displayPreviewPending_ = false;
}

void RenderLoop::drawPauseMenuUi() {
    if (displayPreviewPending_ && !pauseMenuOpen_) {
        pauseMenuOpen_ = true;
        setPausedState(true);
    }
    if (!imguiEnabled_ || renderingReflection_ || captureMode_ ||
        (!pauseMenuOpen_ && !displayPreviewPending_)) {
        return;
    }

    if (!pauseMenuOpen_)
        return;

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

    if (displayPreviewPending_ && ImGui::BeginPopupModal("Confirm display settings", nullptr,
                                                         ImGuiWindowFlags_AlwaysAutoResize)) {
        if (glfwGetTime() >= displayPreviewDeadline_) {
            revertDisplaySettingsPreview();
            displayPreviewError_ = "Display settings were reverted because they were not "
                                   "confirmed in time.";
            ImGui::CloseCurrentPopup();
        } else {
            const double secondsRemaining = std::max(0.0, displayPreviewDeadline_ - glfwGetTime());
            ImGui::TextWrapped("Keep these display settings? They will revert automatically "
                               "unless confirmed.");
            ImGui::Text("Reverting in %.0f seconds.", secondsRemaining);
            if (ImGui::Button("Keep settings")) {
                activeGraphicsPreferences_.windowMode = graphicsPreferences_.values.windowMode;
                activeGraphicsPreferences_.width = graphicsPreferences_.values.width;
                activeGraphicsPreferences_.height = graphicsPreferences_.values.height;
                activeGraphicsPreferences_.refreshRate = graphicsPreferences_.values.refreshRate;
                activeGraphicsPreferences_.monitor = graphicsPreferences_.values.monitor;
                displayPreviewPending_ = false;
                displayPreviewError_.clear();
                ImGui::CloseCurrentPopup();
            }
            ImGui::SameLine();
            if (ImGui::Button("Revert now")) {
                revertDisplaySettingsPreview();
                displayPreviewError_ = "Display settings were reverted.";
                ImGui::CloseCurrentPopup();
            }
        }
        ImGui::EndPopup();
    }

    if (ImGui::Button("Resume", ImVec2(260.0F, 36.0F))) {
        pauseMenuOpen_ = false;
        setPausedState(false);
    }
    ImGui::Separator();
    if (ImGui::CollapsingHeader("Graphics")) {
        if (!graphicsPreferencesError_.empty()) {
            ImGui::TextWrapped("Graphics settings error: %s", graphicsPreferencesError_.c_str());
        }
        if (!graphicsPreferencesLoaded_) {
            ImGui::TextWrapped("Settings cannot be saved until the INI file can be loaded.");
        } else {
            auto& preferences = graphicsPreferences_.values;
            struct QualityPreset {
                const char* name;
                int reflectionSize;
                int reflectionInterval;
                double reflectionMaxFps;
                double mainCullPercent;
                double mirrorCullPercent;
                double renderScale;
                double vehicleTextureScale;
                double mapTextureScale;
            };
            constexpr std::array<QualityPreset, 3> qualityPresets = {{
                {"Low", 256, 3, 30.0, 0.25, 0.5, 0.5, 0.5, 0.5},
                {"Medium", 512, 2, 45.0, 0.1, 0.2, 0.75, 0.75, 0.75},
                {"High", 1024, 1, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0},
            }};
            const auto matchesPreset = [&preferences](const QualityPreset& preset) {
                return preferences.reflectionSize == preset.reflectionSize &&
                       preferences.reflectionInterval == preset.reflectionInterval &&
                       preferences.reflectionMaxFps == preset.reflectionMaxFps &&
                       preferences.minimumObjectSizeMainPercent == preset.mainCullPercent &&
                       preferences.minimumObjectSizeMirrorsPercent == preset.mirrorCullPercent &&
                       preferences.renderScale == preset.renderScale &&
                       preferences.vehicleTextureScale == preset.vehicleTextureScale &&
                       preferences.mapTextureScale == preset.mapTextureScale;
            };
            const QualityPreset* matchedPreset = nullptr;
            for (const auto& preset : qualityPresets) {
                if (matchesPreset(preset)) {
                    matchedPreset = &preset;
                    break;
                }
            }
            const char* presetLabel = matchedPreset == nullptr ? "Custom" : matchedPreset->name;
            if (ImGui::BeginCombo("Quality preset", presetLabel)) {
                for (const auto& preset : qualityPresets) {
                    if (ImGui::Selectable(preset.name, matchedPreset == &preset)) {
                        if (openbus::getEnvironment("OPENBUS_REFLECTION_SIZE") == nullptr)
                            preferences.reflectionSize = preset.reflectionSize;
                        if (openbus::getEnvironment("OPENBUS_REFLECTION_INTERVAL") == nullptr)
                            preferences.reflectionInterval = preset.reflectionInterval;
                        if (openbus::getEnvironment("OPENBUS_REFLECTION_MAX_FPS") == nullptr)
                            preferences.reflectionMaxFps = preset.reflectionMaxFps;
                        if (openbus::getEnvironment("OPENBUS_MIN_OBJECT_SIZE_MAIN_VIEW_PERCENT") ==
                            nullptr)
                            preferences.minimumObjectSizeMainPercent = preset.mainCullPercent;
                        if (openbus::getEnvironment("OPENBUS_MIN_OBJECT_SIZE_MIRRORS_PERCENT") ==
                            nullptr)
                            preferences.minimumObjectSizeMirrorsPercent = preset.mirrorCullPercent;
                        preferences.renderScale = preset.renderScale;
                        if (openbus::getEnvironment("OPENBUS_TEXTURE_SCALE") == nullptr)
                            preferences.vehicleTextureScale = preset.vehicleTextureScale;
                        preferences.mapTextureScale = preset.mapTextureScale;
                    }
                }
                ImGui::EndCombo();
            }
            ImGui::TextWrapped("Presets adjust implemented world render scale, reflection, and "
                               "visibility settings. Edit individual values for a custom "
                               "configuration.");
            const auto beginOverride = [](const char* name) {
                const bool overridden = openbus::getEnvironment(name) != nullptr;
                ImGui::BeginDisabled(overridden);
                return overridden;
            };
            const auto endOverride = [](bool overridden, const char* name) {
                ImGui::EndDisabled();
                if (overridden) {
                    ImGui::SameLine();
                    ImGui::TextDisabled("Overridden by %s", name);
                }
            };

            ImGui::TextUnformatted("Display");
            const char* modeLabel = preferences.windowMode == "borderless" ? "Borderless fullscreen"
                                    : preferences.windowMode == "exclusive" ? "Exclusive fullscreen"
                                                                            : "Windowed";
            if (ImGui::BeginCombo("Window mode", modeLabel)) {
                constexpr std::array<std::pair<const char*, const char*>, 3> modes = {{
                    {"Windowed", "windowed"},
                    {"Borderless fullscreen", "borderless"},
                    {"Exclusive fullscreen", "exclusive"},
                }};
                for (const auto& [label, value] : modes) {
                    if (ImGui::Selectable(label, preferences.windowMode == value)) {
                        preferences.windowMode = value;
                    }
                }
                ImGui::EndCombo();
            }
            ImGui::SameLine();
            ImGui::TextDisabled("Preview");
            int monitorCount = 0;
            GLFWmonitor** monitors = glfwGetMonitors(&monitorCount);
            if (monitorCount > 0) {
                const int maximumMonitorIndex = std::min(monitorCount - 1, 16);
                preferences.monitor = std::clamp(preferences.monitor, 0, maximumMonitorIndex);
                ImGui::SliderInt("Monitor", &preferences.monitor, 0, maximumMonitorIndex);
            } else {
                ImGui::TextDisabled("No display monitor detected");
            }
            ImGui::SameLine();
            ImGui::TextDisabled("Preview");
            if (preferences.windowMode == "windowed") {
                ImGui::InputInt("Window width", &preferences.width, 16, 64);
                preferences.width = std::clamp(preferences.width, 640, 16384);
                ImGui::SameLine();
                ImGui::TextDisabled("Preview");
                ImGui::InputInt("Window height", &preferences.height, 16, 64);
                preferences.height = std::clamp(preferences.height, 640, 16384);
                ImGui::SameLine();
                ImGui::TextDisabled("Preview");
                ImGui::TextDisabled("Refresh rate selection applies to exclusive fullscreen.");
            } else if (preferences.windowMode == "exclusive" && monitorCount > 0 &&
                       monitors != nullptr) {
                const int monitorIndex = std::clamp(preferences.monitor, 0, monitorCount - 1);
                int modeCount = 0;
                const GLFWvidmode* availableModes =
                    glfwGetVideoModes(monitors[monitorIndex], &modeCount);
                const bool currentModeAvailable =
                    availableModes != nullptr &&
                    std::any_of(availableModes, availableModes + modeCount,
                                [&preferences](const GLFWvidmode& mode) {
                                    return mode.width == preferences.width &&
                                           mode.height == preferences.height &&
                                           mode.refreshRate == preferences.refreshRate;
                                });
                bool hasSupportedMode = false;
                for (int index = 0; availableModes != nullptr && index < modeCount; ++index) {
                    const auto& mode = availableModes[index];
                    if (mode.width >= 640 && mode.width <= 16384 && mode.height >= 640 &&
                        mode.height <= 16384 && mode.refreshRate >= 24 &&
                        mode.refreshRate <= 1000) {
                        hasSupportedMode = true;
                        break;
                    }
                }
                const std::string selectedModeLabel =
                    currentModeAvailable ? std::to_string(preferences.width) + " x " +
                                               std::to_string(preferences.height) + " @ " +
                                               std::to_string(preferences.refreshRate) + " Hz"
                                         : "Choose a supported mode";
                if (ImGui::BeginCombo("Fullscreen resolution / refresh",
                                      selectedModeLabel.c_str())) {
                    std::vector<GLFWvidmode> uniqueModes;
                    for (int index = 0; availableModes != nullptr && index < modeCount; ++index) {
                        const auto& mode = availableModes[index];
                        if (mode.width < 640 || mode.width > 16384 || mode.height < 640 ||
                            mode.height > 16384 || mode.refreshRate < 24 ||
                            mode.refreshRate > 1000) {
                            continue;
                        }
                        const bool alreadyListed =
                            std::any_of(uniqueModes.begin(), uniqueModes.end(),
                                        [&mode](const GLFWvidmode& listed) {
                                            return listed.width == mode.width &&
                                                   listed.height == mode.height &&
                                                   listed.refreshRate == mode.refreshRate;
                                        });
                        if (alreadyListed)
                            continue;
                        uniqueModes.push_back(mode);
                        const std::string label = std::to_string(mode.width) + " x " +
                                                  std::to_string(mode.height) + " @ " +
                                                  std::to_string(mode.refreshRate) + " Hz";
                        const bool selected = mode.width == preferences.width &&
                                              mode.height == preferences.height &&
                                              mode.refreshRate == preferences.refreshRate;
                        if (ImGui::Selectable(label.c_str(), selected)) {
                            preferences.width = mode.width;
                            preferences.height = mode.height;
                            preferences.refreshRate = mode.refreshRate;
                        }
                    }
                    ImGui::EndCombo();
                }
                ImGui::SameLine();
                ImGui::TextDisabled("Preview");
                if (modeCount == 0 || availableModes == nullptr) {
                    ImGui::TextDisabled("No exclusive display modes are available.");
                } else if (!hasSupportedMode) {
                    ImGui::TextDisabled("No supported exclusive display modes are available.");
                }
            } else if (preferences.windowMode == "borderless") {
                ImGui::TextWrapped("Borderless fullscreen uses the selected monitor's desktop "
                                   "resolution.");
            } else {
                ImGui::TextDisabled("No display monitor is available for fullscreen modes.");
            }
            const bool displayPreferencesChanged =
                preferences.windowMode != activeGraphicsPreferences_.windowMode ||
                preferences.width != activeGraphicsPreferences_.width ||
                preferences.height != activeGraphicsPreferences_.height ||
                preferences.refreshRate != activeGraphicsPreferences_.refreshRate ||
                preferences.monitor != activeGraphicsPreferences_.monitor;
            if (displayPreferencesChanged && !displayPreviewPending_) {
                if (ImGui::Button("Preview display mode (15-second confirm)")) {
                    previewDisplaySettings();
                }
                ImGui::TextWrapped("If you do not preview the saved display changes, they will "
                                   "apply the next time OpenBus starts.");
            } else if (displayPreviewPending_) {
                ImGui::TextUnformatted("Display mode preview is active; confirm it in the "
                                       "countdown dialog.");
            }
            if (!displayPreviewError_.empty()) {
                ImGui::TextWrapped("Display preview: %s", displayPreviewError_.c_str());
            }
            bool overridden = beginOverride("OPENBUS_VSYNC");
            ImGui::Checkbox("VSync", &preferences.vsync);
            endOverride(overridden, "OPENBUS_VSYNC");
            ImGui::SameLine();
            ImGui::TextDisabled("Live");
            overridden = beginOverride("OPENBUS_FPS_CAP");
            ImGui::InputDouble("FPS cap (0 = unlimited)", &preferences.fpsCap, 1.0, 10.0, "%.2f");
            endOverride(overridden, "OPENBUS_FPS_CAP");
            ImGui::SameLine();
            ImGui::TextDisabled("Live");

            ImGui::Separator();
            ImGui::TextUnformatted("Anti-aliasing");
            const char* antiAliasingLabel = preferences.antiAliasingMode == "fxaa" ? "FXAA" : "Off";
            if (ImGui::BeginCombo("Anti-aliasing mode", antiAliasingLabel)) {
                if (ImGui::Selectable("Off", preferences.antiAliasingMode == "off")) {
                    preferences.antiAliasingMode = "off";
                }
                if (ImGui::Selectable("FXAA", preferences.antiAliasingMode == "fxaa")) {
                    preferences.antiAliasingMode = "fxaa";
                }
                ImGui::EndCombo();
            }
            ImGui::SameLine();
            ImGui::TextDisabled("Live on save");
            ImGui::TextDisabled("MSAA is not available until motion and depth buffers can be "
                                "resolved without mixing surfaces.");

            ImGui::Separator();
            ImGui::TextUnformatted("World render scale");
            float renderScalePercent = static_cast<float>(preferences.renderScale * 100.0);
            if (ImGui::SliderFloat("Render resolution", &renderScalePercent, 50.0F, 100.0F,
                                   "%.0f%%")) {
                preferences.renderScale = static_cast<double>(renderScalePercent) / 100.0;
            }
            ImGui::SameLine();
            ImGui::TextDisabled("Live on save");
            ImGui::TextWrapped("The 3D world renders at this scale and is upscaled before the "
                               "native-resolution interface. Motion and depth textures use the "
                               "scaled render resolution.");

            ImGui::Separator();
            ImGui::TextUnformatted("Reflections (restart required)");
            overridden = beginOverride("OPENBUS_REFLECTION_SIZE");
            const char* sizeLabel = preferences.reflectionSize == 256   ? "256"
                                    : preferences.reflectionSize == 512 ? "512"
                                                                        : "1024";
            if (ImGui::BeginCombo("Reflection resolution", sizeLabel)) {
                for (const int size : {256, 512, 1024}) {
                    const std::string label = std::to_string(size);
                    if (ImGui::Selectable(label.c_str(), preferences.reflectionSize == size)) {
                        preferences.reflectionSize = size;
                    }
                }
                ImGui::EndCombo();
            }
            endOverride(overridden, "OPENBUS_REFLECTION_SIZE");
            ImGui::SameLine();
            ImGui::TextDisabled("Restart required");
            overridden = beginOverride("OPENBUS_REFLECTION_INTERVAL");
            ImGui::InputInt("Reflection update interval", &preferences.reflectionInterval);
            endOverride(overridden, "OPENBUS_REFLECTION_INTERVAL");
            ImGui::SameLine();
            ImGui::TextDisabled("Restart required");
            overridden = beginOverride("OPENBUS_REFLECTION_MAX_FPS");
            ImGui::InputDouble("Reflection FPS cap (0 = unlimited)", &preferences.reflectionMaxFps,
                               1.0, 10.0, "%.2f");
            endOverride(overridden, "OPENBUS_REFLECTION_MAX_FPS");
            ImGui::SameLine();
            ImGui::TextDisabled("Restart required");

            ImGui::Separator();
            ImGui::TextUnformatted("Visibility");
            overridden = beginOverride("OPENBUS_MIN_OBJECT_SIZE_MAIN_VIEW_PERCENT");
            ImGui::InputDouble("Main minimum object size (%)",
                               &preferences.minimumObjectSizeMainPercent, 0.1, 1.0, "%.2f");
            endOverride(overridden, "OPENBUS_MIN_OBJECT_SIZE_MAIN_VIEW_PERCENT");
            ImGui::SameLine();
            ImGui::TextDisabled("Live");
            overridden = beginOverride("OPENBUS_MIN_OBJECT_SIZE_MIRRORS_PERCENT");
            ImGui::InputDouble("Mirror minimum object size (%)",
                               &preferences.minimumObjectSizeMirrorsPercent, 0.1, 1.0, "%.2f");
            endOverride(overridden, "OPENBUS_MIN_OBJECT_SIZE_MIRRORS_PERCENT");
            ImGui::SameLine();
            ImGui::TextDisabled("Live");

            ImGui::Separator();
            ImGui::TextUnformatted("Advanced rendering");
            overridden = beginOverride("OPENBUS_FRUSTUM_CULLING");
            ImGui::Checkbox("Vehicle part frustum culling", &preferences.vehicleFrustumCulling);
            endOverride(overridden, "OPENBUS_FRUSTUM_CULLING");
            ImGui::SameLine();
            ImGui::TextDisabled("Restart required");
            ImGui::TextWrapped("Disable only for troubleshooting missing vehicle parts; "
                               "rendering parts outside the view can increase CPU and GPU work.");

            ImGui::Separator();
            ImGui::TextUnformatted("Texture filtering and quality");
            overridden = beginOverride("OPENBUS_TEXTURE_SCALE");
            float vehicleTextureScalePercent =
                static_cast<float>(preferences.vehicleTextureScale * 100.0);
            if (ImGui::SliderFloat("Vehicle texture resolution", &vehicleTextureScalePercent, 25.0F,
                                   100.0F, "%.0f%%")) {
                preferences.vehicleTextureScale =
                    static_cast<double>(vehicleTextureScalePercent) / 100.0;
            }
            endOverride(overridden, "OPENBUS_TEXTURE_SCALE");
            ImGui::SameLine();
            ImGui::TextDisabled("Restart required");
            ImGui::TextWrapped("Scales decoded textures and selects an authored mip for "
                               "precompressed textures when available. Map textures and "
                               "precompressed textures without mip chains are unchanged.");
            float mapTextureScalePercent = static_cast<float>(preferences.mapTextureScale * 100.0);
            if (ImGui::SliderFloat("Map texture resolution", &mapTextureScalePercent, 25.0F, 100.0F,
                                   "%.0f%%")) {
                preferences.mapTextureScale = static_cast<double>(mapTextureScalePercent) / 100.0;
            }
            ImGui::SameLine();
            ImGui::TextDisabled("Restart required");
            ImGui::TextWrapped("Scales decoded map textures at upload. GPU texture storage falls "
                               "with pixel count, but decoded source images remain cached in CPU "
                               "memory.");
            ImGui::Checkbox("Generate mipmaps for decoded textures",
                            &preferences.generateTextureMipmaps);
            ImGui::SameLine();
            ImGui::TextDisabled("Restart required");
            ImGui::TextWrapped("Adds smoother minification for decoded vehicle and map textures "
                               "at roughly one-third more texture storage. Active anisotropic "
                               "filtering enables mipmaps automatically.");
            const std::string anisotropyLabel =
                preferences.anisotropicFiltering == 1
                    ? "Off"
                    : std::to_string(preferences.anisotropicFiltering) + "x";
            ImGui::BeginDisabled(!textureAnisotropySupported_);
            if (ImGui::BeginCombo("Anisotropic filtering", anisotropyLabel.c_str())) {
                for (const int factor : {1, 2, 4, 8, 16}) {
                    const std::string label = factor == 1 ? "Off" : std::to_string(factor) + "x";
                    if (ImGui::Selectable(label.c_str(),
                                          preferences.anisotropicFiltering == factor)) {
                        preferences.anisotropicFiltering = factor;
                    }
                }
                ImGui::EndCombo();
            }
            ImGui::EndDisabled();
            ImGui::SameLine();
            ImGui::TextDisabled("Restart required");
            if (!textureAnisotropySupported_) {
                ImGui::TextWrapped("Anisotropic filtering is not supported by this OpenGL "
                                   "device; the requested setting will be ignored.");
            } else {
                ImGui::Text("Device maximum: %.0fx; active: %.0fx", maximumTextureAnisotropy_,
                            appliedTextureAnisotropy_);
                if (preferences.anisotropicFiltering > maximumTextureAnisotropy_) {
                    ImGui::TextWrapped("The requested level exceeds this device's limit and will "
                                       "be clamped.");
                }
            }

            ImGui::BeginDisabled(displayPreferencesChanged || displayPreviewPending_);
            if (ImGui::Button("Save graphics settings")) {
                try {
                    auto diskPreferences =
                        openbus::core::loadGraphicsPreferences(graphicsPreferencesPath_);
                    auto edited = preferences;
                    if (openbus::getEnvironment("OPENBUS_VSYNC") != nullptr)
                        edited.vsync = graphicsPreferencesBaseline_.vsync;
                    if (openbus::getEnvironment("OPENBUS_FPS_CAP") != nullptr)
                        edited.fpsCap = graphicsPreferencesBaseline_.fpsCap;
                    if (openbus::getEnvironment("OPENBUS_REFLECTION_SIZE") != nullptr)
                        edited.reflectionSize = graphicsPreferencesBaseline_.reflectionSize;
                    if (openbus::getEnvironment("OPENBUS_REFLECTION_INTERVAL") != nullptr)
                        edited.reflectionInterval = graphicsPreferencesBaseline_.reflectionInterval;
                    if (openbus::getEnvironment("OPENBUS_REFLECTION_MAX_FPS") != nullptr)
                        edited.reflectionMaxFps = graphicsPreferencesBaseline_.reflectionMaxFps;
                    if (openbus::getEnvironment("OPENBUS_MIN_OBJECT_SIZE_MAIN_VIEW_PERCENT") !=
                        nullptr)
                        edited.minimumObjectSizeMainPercent =
                            graphicsPreferencesBaseline_.minimumObjectSizeMainPercent;
                    if (openbus::getEnvironment("OPENBUS_MIN_OBJECT_SIZE_MIRRORS_PERCENT") !=
                        nullptr)
                        edited.minimumObjectSizeMirrorsPercent =
                            graphicsPreferencesBaseline_.minimumObjectSizeMirrorsPercent;
                    if (openbus::getEnvironment("OPENBUS_TEXTURE_SCALE") != nullptr)
                        edited.vehicleTextureScale =
                            graphicsPreferencesBaseline_.vehicleTextureScale;
                    if (openbus::getEnvironment("OPENBUS_FRUSTUM_CULLING") != nullptr)
                        edited.vehicleFrustumCulling =
                            graphicsPreferencesBaseline_.vehicleFrustumCulling;
                    diskPreferences.values = openbus::core::mergeGraphicsPreferences(
                        graphicsPreferencesBaseline_, edited, diskPreferences.values);
                    openbus::core::saveGraphicsPreferences(graphicsPreferencesPath_,
                                                           diskPreferences);
                    graphicsPreferences_.values = diskPreferences.values;
                    graphicsPreferencesBaseline_ = diskPreferences.values;
                    graphicsPreferences_.lines = std::move(diskPreferences.lines);
                    if (openbus::getEnvironment("OPENBUS_VSYNC") == nullptr) {
                        vsyncEnabled_ = preferences.vsync;
                        glfwSwapInterval(vsyncEnabled_ ? 1 : 0);
                    }
                    if (openbus::getEnvironment("OPENBUS_FPS_CAP") == nullptr) {
                        frameTiming_.setFrameRateCap(preferences.fpsCap);
                    }
                    openbus::rendering::setFxaaEnabled(preferences.antiAliasingMode == "fxaa");
                    if (openbus::getEnvironment("OPENBUS_MIN_OBJECT_SIZE_MAIN_VIEW_PERCENT") ==
                        nullptr) {
                        mainViewMinimumObjectViewportFraction_ =
                            preferences.minimumObjectSizeMainPercent / 100.0;
                    }
                    if (openbus::getEnvironment("OPENBUS_MIN_OBJECT_SIZE_MIRRORS_PERCENT") ==
                        nullptr) {
                        reflectionMinimumObjectViewportFraction_ =
                            preferences.minimumObjectSizeMirrorsPercent / 100.0;
                    }
                    graphicsPreferencesError_.clear();
                } catch (const std::exception& error) {
                    graphicsPreferencesError_ = error.what();
                    gameLog.Error("Could not save graphics preferences: " +
                                  graphicsPreferencesError_);
                }
            }
            ImGui::EndDisabled();
            if (displayPreferencesChanged || displayPreviewPending_) {
                ImGui::TextDisabled("Preview and confirm or revert display changes before saving.");
            }
            ImGui::TextWrapped("Saved beside the OpenBus executable. Display changes use a "
                               "15-second confirmation; reflection, anisotropic filtering, "
                               "texture mipmaps, map and vehicle texture resolution, and "
                               "vehicle frustum-culling changes require a restart.");
        }
    }
    ImGui::Separator();
    if (ImGui::CollapsingHeader("Weather")) {
        if (!weatherPresetsLoaded_) {
            try {
                weatherPresets_ =
                    openbus::config::discoverWeatherPresets(weatherPresetRoot_ / "Weather");
                weatherPresetError_.clear();
                selectedWeatherPreset_ = 0;
            } catch (const std::exception& error) {
                weatherPresets_.clear();
                weatherPresetError_ =
                    "Could not load weather presets: " + std::string(error.what());
                gameLog.Error(weatherPresetError_);
            }
            weatherPresetsLoaded_ = true;
        }
        if (!weatherPresetError_.empty()) {
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0F, 0.4F, 0.4F, 1.0F));
            ImGui::TextWrapped("%s", weatherPresetError_.c_str());
            ImGui::PopStyleColor();
        }
        if (ImGui::Button("Reload presets"))
            weatherPresetsLoaded_ = false;
        if (!weatherPresets_.empty()) {
            selectedWeatherPreset_ = std::min(selectedWeatherPreset_, weatherPresets_.size() - 1);
        }
        const char* selectedPreset = weatherPresets_.empty()
                                         ? "No presets available"
                                         : weatherPresets_[selectedWeatherPreset_].name.c_str();
        if (ImGui::BeginCombo("Preset to apply", selectedPreset)) {
            for (std::size_t i = 0; i < weatherPresets_.size(); ++i) {
                ImGui::PushID(static_cast<int>(i));
                if (ImGui::Selectable(weatherPresets_[i].name.c_str(),
                                      i == selectedWeatherPreset_)) {
                    selectedWeatherPreset_ = i;
                }
                ImGui::PopID();
            }
            ImGui::EndCombo();
        }
        ImGui::BeginDisabled(weatherPresets_.empty());
        if (ImGui::Button("Apply preset")) {
            try {
                const auto loaded = openbus::config::loadWeatherPreset(
                    weatherPresets_[selectedWeatherPreset_].path);
                SetWeather(loaded.weather);
                simulationState_.sharedVariables().seedRoadWeatherFromPreset();
                weatherSelection_.discardPending();
                weatherUiMessage_ =
                    "Applied preset: " + weatherPresets_[selectedWeatherPreset_].name + ".";
                if (!loaded.warnings.empty()) {
                    weatherUiMessage_ += " Some fields are approximate or unsupported; see log.";
                    for (const auto& warning : loaded.warnings)
                        gameLog.Warning(warning);
                }
                weatherUiError_ = false;
                gameLog.Log(weatherUiMessage_);
            } catch (const std::exception& error) {
                weatherUiMessage_ = "Could not apply weather preset: " + std::string(error.what());
                weatherUiError_ = true;
                gameLog.Error(weatherUiMessage_);
            }
        }
        ImGui::EndDisabled();
        ImGui::SeparatorText("Custom weather");
        if (ImGui::Button("Configure custom weather")) {
            try {
                customWeatherDraft_ = simulationState_.sharedVariables().weather();
                customWeatherError_.clear();
                ImGui::OpenPopup("Custom weather");
            } catch (const std::exception& error) {
                weatherUiMessage_ = "Could not read current weather: " + std::string(error.what());
                weatherUiError_ = true;
                gameLog.Error(weatherUiMessage_);
            }
        }
        ImGui::SetNextWindowPos(ImVec2(viewport->WorkPos.x + viewport->WorkSize.x * 0.5F,
                                       viewport->WorkPos.y + viewport->WorkSize.y * 0.5F),
                                ImGuiCond_Appearing, ImVec2(0.5F, 0.5F));
        ImGui::SetNextWindowSize(ImVec2(std::min(650.0F, viewport->WorkSize.x * 0.9F),
                                        std::min(700.0F, viewport->WorkSize.y * 0.9F)),
                                 ImGuiCond_Appearing);
        if (ImGui::BeginPopupModal("Custom weather", nullptr,
                                   ImGuiWindowFlags_NoCollapse |
                                       ImGuiWindowFlags_NoSavedSettings)) {
            const auto action =
                openbus::drawCustomWeatherEditor(customWeatherDraft_, customWeatherError_);
            if (action == openbus::CustomWeatherAction::Apply) {
                try {
                    SetWeather(openbus::customWeatherFromDraft(customWeatherDraft_));
                    weatherSelection_.discardPending();
                    weatherUiMessage_ = "Applied custom weather.";
                    weatherUiError_ = false;
                    gameLog.Log(weatherUiMessage_);
                    ImGui::CloseCurrentPopup();
                } catch (const std::exception& error) {
                    customWeatherError_ =
                        "Could not apply custom weather: " + std::string(error.what());
                    gameLog.Error(customWeatherError_);
                }
            }
            if (action == openbus::CustomWeatherAction::Cancel)
                ImGui::CloseCurrentPopup();
            ImGui::EndPopup();
        }
        if (ImGui::Button("Manage accumulated surface conditions"))
            ImGui::OpenPopup("Surface conditions");
        ImGui::SetNextWindowPos(ImVec2(viewport->WorkPos.x + viewport->WorkSize.x * 0.5F,
                                       viewport->WorkPos.y + viewport->WorkSize.y * 0.5F),
                                ImGuiCond_Appearing, ImVec2(0.5F, 0.5F));
        ImGui::SetNextWindowSize(ImVec2(std::min(550.0F, viewport->WorkSize.x * 0.9F),
                                        std::min(350.0F, viewport->WorkSize.y * 0.9F)),
                                 ImGuiCond_Appearing);
        if (ImGui::BeginPopupModal("Surface conditions", nullptr,
                                   ImGuiWindowFlags_NoCollapse |
                                       ImGuiWindowFlags_NoSavedSettings)) {
            auto& system = simulationState_.sharedVariables();
            if (const auto action = openbus::drawSurfaceWeatherControls(system.roadWeather()))
                system.applySurfaceWeatherAction(*action);
            if (ImGui::Button("Close"))
                ImGui::CloseCurrentPopup();
            ImGui::EndPopup();
        }
        ImGui::SeparatorText("Fog layer");
        const auto currentFogHeight =
            simulationState_.sharedVariables().weatherFogTopAboveGroundMeters();
        bool useFogHeight = currentFogHeight.has_value();
        double fogHeight = currentFogHeight.value_or(100.0);
        ImGui::BeginDisabled(weatherSelection_.pending());
        const bool fogEnabledChanged = ImGui::Checkbox("Limit fog by height", &useFogHeight);
        ImGui::BeginDisabled(!useFogHeight);
        const bool fogHeightChanged =
            ImGui::InputDouble("Fog layer height (m)", &fogHeight, 0.0, 0.0, "%.1f",
                               ImGuiInputTextFlags_EnterReturnsTrue);
        ImGui::EndDisabled();
        ImGui::EndDisabled();
        if (fogEnabledChanged || fogHeightChanged) {
            try {
                auto weather = simulationState_.sharedVariables().weather();
                weather.fogTopAboveGroundMeters =
                    useFogHeight ? std::optional<double>{fogHeight} : std::nullopt;
                SetWeather(weather);
                weatherUiMessage_ = "Fog layer updated for the current weather.";
                weatherUiError_ = false;
                gameLog.Log(weatherUiMessage_);
            } catch (const std::exception& error) {
                weatherUiMessage_ = "Could not update fog layer: " + std::string(error.what());
                weatherUiError_ = true;
                gameLog.Error(weatherUiMessage_);
            }
        }
        ImGui::TextDisabled("New presets and METAR reports replace this override.");
        ImGui::SeparatorText("NOAA METAR");
        if (!weatherStationsLoaded_) {
            try {
                weatherStations_ = openbus::metarStations();
                weatherStationError_.clear();
            } catch (const std::exception& error) {
                weatherStationError_ =
                    "Could not load METAR stations: " + std::string(error.what());
                gameLog.Error(weatherStationError_);
            }
            weatherStationsLoaded_ = true;
            if (!activeMetarStations_.pending()) {
                try {
                    activeMetarStations_.request();
                } catch (const std::exception& error) {
                    weatherStationError_ += " Could not start NOAA station directory request: " +
                                            std::string(error.what());
                    gameLog.Error(weatherStationError_);
                }
            }
        }
        if (!weatherStationError_.empty()) {
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0F, 0.4F, 0.4F, 1.0F));
            ImGui::TextWrapped("%s", weatherStationError_.c_str());
            ImGui::PopStyleColor();
            if (ImGui::Button("Retry station catalogue"))
                weatherStationsLoaded_ = false;
        }
        if (activeMetarStations_.pending())
            ImGui::TextDisabled("Refreshing recently updated NOAA stations...");
        else if (activeMetarStations_.failed()) {
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0F, 0.4F, 0.4F, 1.0F));
            ImGui::TextWrapped("%s", activeMetarStations_.status().c_str());
            ImGui::PopStyleColor();
            if (recentWeatherStationsAvailable_)
                ImGui::TextWrapped("The previous activity list is retained.");
        } else if (recentWeatherStationsAvailable_)
            ImGui::TextWrapped("%s", activeMetarStations_.status().c_str());
        if (!recentWeatherStationsAvailable_)
            ImGui::TextWrapped("The full catalogue, if available, is shown until NOAA activity "
                               "is loaded.");
        ImGui::BeginDisabled(activeMetarStations_.pending());
        if (ImGui::Button("Refresh active stations")) {
            try {
                activeMetarStations_.request();
            } catch (const std::exception& error) {
                weatherStationError_ =
                    "Could not start NOAA station directory request: " + std::string(error.what());
                gameLog.Error(weatherStationError_);
            }
        }
        ImGui::EndDisabled();
        openbus::drawMetarStationPicker(weatherStationPicker_, recentWeatherStationsAvailable_
                                                                   ? recentWeatherStations_
                                                                   : weatherStations_);
        ImGui::BeginDisabled(weatherStationPicker_.code[0] == '\0' || weatherSelection_.pending());
        if (ImGui::Button("Fetch and apply METAR")) {
            try {
                weatherSelection_.requestMetar(weatherStationPicker_.code.data());
                weatherUiMessage_ = weatherSelection_.status();
                weatherUiError_ = false;
            } catch (const std::exception& error) {
                weatherUiMessage_ = "Could not start METAR request: " + std::string(error.what());
                weatherUiError_ = true;
                gameLog.Error(weatherUiMessage_);
            }
        }
        ImGui::EndDisabled();
        if (weatherSelection_.pending()) {
            ImGui::TextDisabled("A METAR request is in progress.");
        }
        if (!weatherUiMessage_.empty()) {
            if (weatherUiError_) {
                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0F, 0.4F, 0.4F, 1.0F));
                ImGui::TextWrapped("%s", weatherUiMessage_.c_str());
                ImGui::PopStyleColor();
            } else {
                ImGui::TextWrapped("%s", weatherUiMessage_.c_str());
            }
        }
        if (weatherSelection_.activeStation()) {
            ImGui::Text("Automatic refresh: %s every 10 minutes",
                        weatherSelection_.activeStation()->c_str());
            if (ImGui::Button("Stop automatic METAR updates")) {
                weatherSelection_.discardPending();
                weatherUiMessage_ = "Automatic METAR updates stopped; current weather retained.";
                weatherUiError_ = false;
                gameLog.Log(weatherUiMessage_);
            }
        }
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
        drawMapEntryPointCombo(mapEntryPoints_, selectedMapSpawnEntryPointIndex_, "Bus location");
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
    ImGui::SetNextWindowPos(
        ImVec2(viewport->WorkPos.x + viewport->WorkSize.x - 20.0F, viewport->WorkPos.y + 20.0F),
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
    const bool validDateTime =
        debugCalendarYear_ >= 1 && debugCalendarYear_ <= 9999 && debugCalendarMonth_ >= 1 &&
        debugCalendarMonth_ <= 12 && debugCalendarDay_ >= 1 && debugCalendarDay_ <= 31 &&
        selectedDate.ok() && debugCalendarHour_ >= 0 && debugCalendarHour_ < 24 &&
        debugCalendarMinute_ >= 0 && debugCalendarMinute_ < 60;
    if (!validDateTime) {
        ImGui::TextColored(ImVec4(1.0F, 0.45F, 0.35F, 1.0F), "Enter a valid date and time.");
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
                const int departureMinute = static_cast<int>(std::floor(departureSeconds / 60.0));
                if (ImGui::Button("Skip to scheduled departure")) {
                    mapCalendarRequestPending_ =
                        MapCalendarRequest{static_cast<int>(calendar.get("year")),
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
