#pragma once

#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

namespace openbus::assets {

enum class VehicleTextureAccessPolicy { RuntimeCompatible, Strict };

class VehicleTextureResolver {
  public:
    explicit VehicleTextureResolver(
        bool logMissing = true,
        VehicleTextureAccessPolicy accessPolicy = VehicleTextureAccessPolicy::RuntimeCompatible)
        : logMissing_(logMissing), accessPolicy_(accessPolicy) {}
    std::filesystem::path find(const std::filesystem::path& root, const std::string& name);

  private:
    struct TextureIndex {
        std::mutex mutex;
        bool built = false;
        std::unordered_map<std::string, std::filesystem::path> byFileName;
        std::unordered_map<std::string, std::filesystem::path> byStem;
    };
    struct RootIndex {
        std::mutex mutex;
        bool built = false;
        std::unordered_map<std::string, std::filesystem::path> byStem;
    };

    bool logMissing_;
    VehicleTextureAccessPolicy accessPolicy_;
    std::mutex mutex_;
    std::unordered_map<std::string, std::filesystem::path> resolutions_;
    std::unordered_map<std::string, std::shared_ptr<TextureIndex>> textureIndexes_;
    std::unordered_map<std::string, std::shared_ptr<RootIndex>> rootIndexes_;
};

} // namespace openbus::assets
