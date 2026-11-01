#pragma once

#include <filesystem>
#include <optional>
#include <string>

namespace openbus::app {

class SteamPresence {
  public:
    SteamPresence() = default;
    ~SteamPresence();

    SteamPresence(const SteamPresence&) = delete;
    SteamPresence& operator=(const SteamPresence&) = delete;

    bool initializeFromEnvironment();
    bool initializeFromDirectory(const std::filesystem::path& directory);
    bool initialize(const std::filesystem::path& libraryPath);
    void runCallbacks() const;
    void shutdown();
    bool active() const noexcept { return active_; }

  private:
    using SteamError = char[1024];
    void restoreEnvironment();

    void* library_ = nullptr;
    int (*init_)(SteamError*) = nullptr;
    void (*callbacks_)() = nullptr;
    void (*shutdown_)() = nullptr;
    std::optional<std::filesystem::path::string_type> previousAppId_;
    std::optional<std::filesystem::path::string_type> previousGameId_;
    bool environmentChanged_ = false;
    bool active_ = false;
};

} // namespace openbus::app
