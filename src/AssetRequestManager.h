#pragma once

#include "ObjLoader.h"
#include "TextureAssetLoader.h"

#include <filesystem>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace openbus::rendering {

class AssetRequestManager {
  public:
        using TextureHandle = unsigned int;
    using TextureResolver = std::function<std::filesystem::path(
        const std::filesystem::path&, const std::filesystem::path&, const std::string&)>;

    struct TextureRequest {
        std::filesystem::path root;
        std::filesystem::path path;
        std::string name;
        TextureResolver resolver;
        std::filesystem::path resolvedPath;
        std::shared_ptr<Image> image;
        std::shared_ptr<gli::texture> compressedTexture;
        std::shared_ptr<CompressedDds> compressedDds;
        bool complete = false;
        bool started = false;
        std::mutex mutex;
        std::shared_future<void> task;
    };

    struct TextureCacheEntry {
        std::shared_ptr<TextureRequest> request;
        TextureHandle texture = 0;
        bool textureArray = false;
        std::size_t textureArrayLayers = 1;
        bool uploadAttempted = false;
    };

    AssetRequestManager() = default;
    ~AssetRequestManager();

    AssetRequestManager(const AssetRequestManager&) = delete;
    AssetRequestManager& operator=(const AssetRequestManager&) = delete;

    std::shared_future<std::shared_ptr<ParsedObj>> requestObj(
        const std::filesystem::path& path);
    std::shared_ptr<TextureCacheEntry> requestTexture(
        const std::filesystem::path& root, const std::filesystem::path& path,
        const std::string& name, TextureResolver resolver);
    void startTextureRequest(const std::shared_ptr<TextureCacheEntry>& entry);
    void trackTexture(TextureHandle texture);
    void join();

  private:
    struct DecodedTexture {
        std::shared_ptr<Image> image;
        std::shared_ptr<gli::texture> compressedTexture;
        std::shared_ptr<CompressedDds> compressedDds;
    };

    static std::string normalizedPathKey(const std::filesystem::path& path);
    static std::string textureKey(const std::filesystem::path& root,
                                  const std::filesystem::path& path,
                                  const std::string& name);
    static std::string textureAliasKey(const std::filesystem::path& root,
                                       const std::filesystem::path& path,
                                       const std::string& name);
    void loadTextureRequest(const std::shared_ptr<TextureRequest>& request);

    std::mutex parsedObjMutex_;
    std::unordered_map<std::string, std::shared_future<std::shared_ptr<ParsedObj>>> parsedObjCache_;
    std::mutex textureMutex_;
    std::unordered_map<std::string, std::shared_ptr<TextureCacheEntry>> textureCache_;
    std::unordered_map<std::string, std::shared_ptr<TextureCacheEntry>> textureAliases_;
    std::mutex decodedTextureMutex_;
    std::unordered_map<std::string, std::shared_ptr<DecodedTexture>> decodedTextureCache_;
    std::vector<TextureHandle> textures_;
};

} // namespace openbus::rendering
