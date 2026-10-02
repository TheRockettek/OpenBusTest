#include "AssetRequestManager.h"

#include "O3DLoader.h"

#include "Logger.h"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <exception>
#include <stdexcept>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif
#include <GL/gl.h>

extern Logger gameLog;

namespace openbus::rendering {

namespace {

#ifdef _WIN32
class ComInitializer {
  public:
    ComInitializer() : initialized_(SUCCEEDED(CoInitializeEx(nullptr, COINIT_MULTITHREADED))) {}
    ~ComInitializer() {
        if (initialized_) {
            CoUninitialize();
        }
    }

    ComInitializer(const ComInitializer&) = delete;
    ComInitializer& operator=(const ComInitializer&) = delete;

  private:
    bool initialized_;
};
#endif

std::string lower(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char character) {
        return static_cast<char>(std::tolower(character));
    });
    return value;
}

} // namespace

AssetRequestManager::AssetRequestManager() {
    std::size_t workerCount = 4;
    if (const char* configuredWorkers = std::getenv("OPENBUS_ASSET_WORKERS")) {
        try {
            workerCount = static_cast<std::size_t>(
                std::clamp(std::stoi(configuredWorkers), 1,
                           static_cast<int>(std::thread::hardware_concurrency())));
        } catch (const std::exception&) {
            workerCount = 4;
        }
    }
    workers_.reserve(workerCount);
    for (std::size_t index = 0; index < workerCount; ++index) {
        workers_.emplace_back(&AssetRequestManager::workerLoop, this);
    }
}

AssetRequestManager::~AssetRequestManager() {
    join();
    {
        std::lock_guard<std::mutex> lock(workerMutex_);
        stoppingWorkers_ = true;
    }
    workerCondition_.notify_all();
    for (std::thread& worker : workers_) {
        if (worker.joinable()) {
            worker.join();
        }
    }
    if (!textures_.empty()) {
        glDeleteTextures(static_cast<GLsizei>(textures_.size()), textures_.data());
    }
}

void AssetRequestManager::enqueue(std::function<void()> task) {
    {
        std::lock_guard<std::mutex> lock(workerMutex_);
        if (stoppingWorkers_) {
            throw std::runtime_error("asset worker pool is stopping");
        }
        workerQueue_.push(std::move(task));
    }
    workerCondition_.notify_one();
}

void AssetRequestManager::workerLoop() {
    while (true) {
        std::function<void()> task;
        {
            std::unique_lock<std::mutex> lock(workerMutex_);
            workerCondition_.wait(lock,
                                  [this] { return stoppingWorkers_ || !workerQueue_.empty(); });
            if (stoppingWorkers_ && workerQueue_.empty()) {
                return;
            }
            task = std::move(workerQueue_.front());
            workerQueue_.pop();
        }
        task();
    }
}

std::string AssetRequestManager::normalizedPathKey(const std::filesystem::path& path) {
    return lower(std::filesystem::absolute(path).lexically_normal().generic_string());
}

std::string AssetRequestManager::textureKey(const std::filesystem::path& root,
                                            const std::filesystem::path& path,
                                            const std::string& name) {
    const std::filesystem::path identity = path.empty() ? root / name : path;
    return normalizedPathKey(identity);
}

std::string AssetRequestManager::textureAliasKey(const std::filesystem::path& root,
                                                 const std::filesystem::path& path,
                                                 const std::string& name) {
    const std::string normalizedRoot = normalizedPathKey(root);
    const std::filesystem::path identity = path.empty() ? std::filesystem::path(name) : path;
    return normalizedRoot + "|" + lower(identity.parent_path().generic_string()) + "/" +
           lower(identity.stem().generic_string());
}

std::shared_future<std::shared_ptr<ParsedObj>>
AssetRequestManager::requestObj(const std::filesystem::path& path, const std::string& bundleEntry) {
    const std::string key = normalizedPathKey(path) + "|" + lower(bundleEntry);
    std::lock_guard<std::mutex> lock(parsedObjMutex_);
    const auto cached = parsedObjCache_.find(key);
    if (cached != parsedObjCache_.end()) {
        return cached->second;
    }
    auto task =
        std::make_shared<std::packaged_task<std::shared_ptr<ParsedObj>()>>([path, bundleEntry] {
            std::string extension = path.extension().string();
            std::transform(
                extension.begin(), extension.end(), extension.begin(),
                [](unsigned char character) { return static_cast<char>(std::tolower(character)); });
            return extension == ".o3d" ? O3DLoader::parse(path)
                                       : ObjLoader::parse(path, bundleEntry);
        });
    std::shared_future<std::shared_ptr<ParsedObj>> future = task->get_future().share();
    enqueue([task] { (*task)(); });
    parsedObjCache_.emplace(key, future);
    return future;
}

std::shared_ptr<AssetRequestManager::TextureCacheEntry>
AssetRequestManager::requestTexture(const std::filesystem::path& root,
                                    const std::filesystem::path& path, const std::string& name,
                                    TextureResolver resolver) {
    const std::string key = textureKey(root, path, name);
    std::lock_guard<std::mutex> lock(textureMutex_);
    const auto found = textureCache_.find(key);
    if (found != textureCache_.end()) {
        return found->second;
    }
    const std::string alias = textureAliasKey(root, path, name);
    if (!alias.empty()) {
        const auto aliasFound = textureAliases_.find(alias);
        if (aliasFound != textureAliases_.end()) {
            textureCache_.emplace(key, aliasFound->second);
            return aliasFound->second;
        }
    }
    auto entry = std::make_shared<TextureCacheEntry>();
    entry->request = std::make_shared<TextureRequest>();
    entry->request->root = root;
    entry->request->path = path;
    entry->request->name = name;
    entry->request->resolver = std::move(resolver);
    textureCache_.emplace(key, entry);
    if (!alias.empty()) {
        textureAliases_.emplace(alias, entry);
    }
    return entry;
}

void AssetRequestManager::startTextureRequest(const std::shared_ptr<TextureCacheEntry>& entry) {
    {
        std::lock_guard<std::mutex> lock(entry->request->mutex);
        if (entry->request->started) {
            return;
        }
        entry->request->started = true;
    }
    try {
        auto task = std::make_shared<std::packaged_task<void()>>([this, request = entry->request] {
            try {
                loadTextureRequest(request);
            } catch (const std::exception& error) {
                gameLog.Log("Texture worker failed: " + std::string(error.what()));
            } catch (...) {
                gameLog.Log("Texture worker failed with an unknown error");
            }
            std::lock_guard<std::mutex> lock(request->mutex);
            request->complete = true;
        });
        entry->request->task = task->get_future().share();
        enqueue([task] { (*task)(); });
    } catch (const std::exception& error) {
        gameLog.Log("Failed to start texture worker: " + std::string(error.what()));
        std::lock_guard<std::mutex> lock(entry->request->mutex);
        entry->request->complete = true;
    }
}

void AssetRequestManager::loadTextureRequest(const std::shared_ptr<TextureRequest>& request) {
#ifdef _WIN32
    const ComInitializer com;
#endif
    std::filesystem::path resolved;
    if (request->resolver) {
        resolved = request->resolver(request->root, request->path, request->name);
    }
    if (resolved.empty() && !request->path.empty()) {
        const std::filesystem::path candidate = request->path;
        if (candidate.is_absolute() && std::filesystem::exists(candidate)) {
            resolved = candidate;
        } else if (request->name.empty() && std::filesystem::exists(candidate)) {
            resolved = candidate;
        }
    }
    const std::string resolvedKey = resolved.empty() ? std::string() : normalizedPathKey(resolved);
    if (!resolvedKey.empty()) {
        std::shared_ptr<DecodedTexture> cachedTexture;
        {
            std::lock_guard<std::mutex> lock(decodedTextureMutex_);
            const auto cached = decodedTextureCache_.find(resolvedKey);
            if (cached != decodedTextureCache_.end()) {
                cachedTexture = cached->second;
            }
        }
        if (cachedTexture) {
            std::lock_guard<std::mutex> requestLock(request->mutex);
            request->resolvedPath = resolved;
            request->image = cachedTexture->image;
            request->compressedTexture = cachedTexture->compressedTexture;
            request->compressedDds = cachedTexture->compressedDds;
            request->complete = true;
            return;
        }
    }
    TextureAsset asset;
    const bool textureLoaded = loadTextureAsset(resolved, asset);
    std::lock_guard<std::mutex> lock(request->mutex);
    if (textureLoaded) {
        request->resolvedPath = std::move(resolved);
        request->image = std::make_shared<Image>(std::move(asset.image));
        request->compressedTexture = std::move(asset.compressedTexture);
        request->compressedDds = std::move(asset.compressedDds);
        if (!resolvedKey.empty()) {
            auto decoded = std::make_shared<DecodedTexture>();
            decoded->image = request->image;
            decoded->compressedTexture = request->compressedTexture;
            decoded->compressedDds = request->compressedDds;
            std::lock_guard<std::mutex> cacheLock(decodedTextureMutex_);
            decodedTextureCache_.emplace(resolvedKey, std::move(decoded));
        }
    }
    request->complete = true;
}

void AssetRequestManager::trackTexture(TextureHandle texture) {
    if (texture != 0) {
        textures_.push_back(texture);
    }
}

void AssetRequestManager::join() {
    std::lock_guard<std::mutex> lock(textureMutex_);
    for (const auto& cache : textureCache_) {
        if (cache.second->request && cache.second->request->task.valid()) {
            cache.second->request->task.wait();
        }
    }
}

} // namespace openbus::rendering
