#include "AssetRequestManager.h"

#include "Logger.h"

#include <algorithm>
#include <cctype>
#include <exception>

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

std::string lower(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char character) {
        return static_cast<char>(std::tolower(character));
    });
    return value;
}

} // namespace

AssetRequestManager::~AssetRequestManager() {
    join();
    if (!textures_.empty()) {
        glDeleteTextures(static_cast<GLsizei>(textures_.size()), textures_.data());
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
    return normalizedRoot + "|" + lower(identity.stem().generic_string());
}

std::shared_future<std::shared_ptr<ParsedObj>>
AssetRequestManager::requestObj(const std::filesystem::path& path) {
    const std::string key = normalizedPathKey(path);
    std::lock_guard<std::mutex> lock(parsedObjMutex_);
    const auto cached = parsedObjCache_.find(key);
    if (cached != parsedObjCache_.end()) {
        return cached->second;
    }
    std::shared_future<std::shared_ptr<ParsedObj>> future =
        std::async(std::launch::async, &ObjLoader::parse, path).share();
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
        entry->request->task =
            std::async(std::launch::async, [this, request = entry->request] {
                try {
                    loadTextureRequest(request);
                } catch (const std::exception& error) {
                    gameLog.Log("Texture worker failed: " + std::string(error.what()));
                    std::lock_guard<std::mutex> lock(request->mutex);
                    request->complete = true;
                } catch (...) {
                    gameLog.Log("Texture worker failed with an unknown error");
                    std::lock_guard<std::mutex> lock(request->mutex);
                    request->complete = true;
                }
            }).share();
    } catch (const std::exception& error) {
        gameLog.Log("Failed to start texture worker: " + std::string(error.what()));
        std::lock_guard<std::mutex> lock(entry->request->mutex);
        entry->request->complete = true;
    }
}

void AssetRequestManager::loadTextureRequest(const std::shared_ptr<TextureRequest>& request) {
#ifdef _WIN32
    const HRESULT comResult = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
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
#ifdef _WIN32
            if (SUCCEEDED(comResult)) {
                CoUninitialize();
            }
#endif
            return;
        }
    }
    TextureAsset asset;
    const bool textureLoaded = loadTextureAsset(resolved, asset);
#ifdef _WIN32
    if (SUCCEEDED(comResult)) {
        CoUninitialize();
    }
#endif
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
