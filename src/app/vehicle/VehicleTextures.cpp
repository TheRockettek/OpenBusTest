// Texture lookup, asynchronous loading and GPU upload.

#include "app/vehicle/Vehicle.h"
#include "config/ScriptTextureLimits.h"

bool Vehicle::areTexturesLoaded() const {
    if (!areObjectsLoaded()) {
        return false;
    }
    std::unordered_set<const TextureCacheEntry*> trackedTextures;
    const auto requestComplete = [](const std::shared_ptr<TextureRequest>& request) {
        std::lock_guard<std::mutex> lock(request->mutex);
        return request->complete &&
               (!request->fallbackDecodeStarted || request->fallbackDecodeComplete);
    };
    const auto texturesLoadedInParts = [&](const std::vector<DisplayPart>& parts) {
        for (const DisplayPart& displayPart : parts) {
            for (const Batch& batch : displayPart.batches) {
                for (const MaterialTextureSource& source : batch.materialTextures) {
                    if (!source.textureCacheEntry) {
                        return false;
                    }
                    const TextureCacheEntry* sourceEntry = source.textureCacheEntry.get();
                    if (trackedTextures.insert(sourceEntry).second &&
                        !requestComplete(sourceEntry->request)) {
                        return false;
                    }
                }
                if (batch.materialTextures.empty() &&
                    (!batch.texturePath.empty() || !batch.textureName.empty())) {
                    if (!batch.textureCacheEntry) {
                        return false;
                    }
                    const TextureCacheEntry* entry = batch.textureCacheEntry.get();
                    if (trackedTextures.insert(entry).second && !requestComplete(entry->request)) {
                        return false;
                    }
                }
                for (const AuxiliaryTexture* auxiliary :
                     {&batch.lightmap, &batch.nightmap, &batch.transmap, &batch.bumpmap}) {
                    if (auxiliary->name.empty() ||
                        openbus::scripting::scriptTextureReferenceIndex(auxiliary->name) >= 0) {
                        continue;
                    }
                    if (!auxiliary->cacheEntry) {
                        return false;
                    }
                    const TextureCacheEntry* auxiliaryEntry = auxiliary->cacheEntry.get();
                    if (!trackedTextures.insert(auxiliaryEntry).second) {
                        continue;
                    }
                    if (!requestComplete(auxiliaryEntry->request)) {
                        return false;
                    }
                }
            }
        }
        return true;
    };
    if (!texturesLoadedInParts(displayLists)) {
        return false;
    }
    return true;
}

bool Vehicle::uploadTextureCacheEntry(const std::shared_ptr<TextureCacheEntry>& entry) {
    if (!entry) {
        return false;
    }
    const std::shared_ptr<TextureRequest>& request = entry->request;
    std::unique_lock<std::mutex> lock(request->mutex);
    if (!request->complete) {
        return false;
    }
    if (entry->uploadAttempted) {
        return true;
    }
    if (request->resolvedPath.empty()) {
        entry->uploadAttempted = true;
        return true;
    }

    if (request->compressedDds) {
        if (!entry->compressedUploadAttempted) {
            entry->texture = uploadCompressedDds(request->resolvedPath, *request->compressedDds);
            entry->compressedUploadAttempted = true;
            if (entry->texture == 0) {
                lock.unlock();
                assets->requestDxt5Fallback(request);
                return false;
            }
        } else if (entry->texture == 0) {
            if (!request->fallbackDecodeComplete) {
                return false;
            }
            if (request->image) {
                entry->texture = uploadTexture(request->resolvedPath, *request->image);
            }
        }
    } else if (request->compressedTexture) {
        entry->texture = uploadCompressedTexture(request->resolvedPath, *request->compressedTexture,
                                                 entry->textureArray, entry->textureArrayLayers);
    } else if (request->image) {
        entry->texture = uploadTexture(request->resolvedPath, *request->image);
    }

    entry->uploadAttempted = true;
    return true;
}

GLuint Vehicle::uploadTexture(const std::filesystem::path& path, const Image& image) {
    TraceScope trace("texture", "uploadTexture");
    static const bool verboseTextureUploadLogs =
        parseEnabledFlag(openbus::getEnvironment("OPENBUS_VERBOSE_TEXTURE_UPLOAD"));
    if (verboseTextureUploadLogs) {
        gameLog.Debug("Uploading texture from path: " + path.generic_string());
    }
    Image scaledImage;
    const Image* uploadImage = &image;
    if (textureScale < 0.999) {
        const int scaledWidth =
            std::max(1, static_cast<int>(std::lround(image.width * textureScale)));
        const int scaledHeight =
            std::max(1, static_cast<int>(std::lround(image.height * textureScale)));
        std::size_t scaledSize = 0;
        if (!openbus::rendering::checkedTextureBufferSize(static_cast<std::size_t>(scaledWidth),
                                                          static_cast<std::size_t>(scaledHeight), 4,
                                                          scaledSize)) {
            gameLog.Error("Texture scaling exceeds the decoded image size limit: " +
                        path.generic_string());
            return 0;
        }
        std::vector<std::uint8_t> scaled(scaledSize);
        for (int y = 0; y < scaledHeight; ++y) {
            const int sourceY = std::min(image.height - 1, static_cast<int>(y / textureScale));
            for (int x = 0; x < scaledWidth; ++x) {
                const int sourceX = std::min(image.width - 1, static_cast<int>(x / textureScale));
                const std::size_t source =
                    (static_cast<std::size_t>(sourceY) * image.width + sourceX) * 4;
                const std::size_t target = (static_cast<std::size_t>(y) * scaledWidth + x) * 4;
                std::copy_n(image.rgba.data() + source, 4, scaled.data() + target);
            }
        }
        scaledImage.width = scaledWidth;
        scaledImage.height = scaledHeight;
        scaledImage.rgba = std::move(scaled);
        uploadImage = &scaledImage;
    }
    GLuint texture = 0;
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, uploadImage->width, uploadImage->height, 0, GL_RGBA,
                 GL_UNSIGNED_BYTE, uploadImage->rgba.data());
    openbus::rendering::invalidateTextureBindings();
    assets->trackTexture(texture);
    return texture;
}

GLuint Vehicle::uploadCompressedTexture(const std::filesystem::path& path,
                                        const gli::texture& image, bool& textureArray,
                                        std::size_t& textureArrayLayers) {
    TraceScope trace("texture", "uploadCompressedTexture");
    textureArray = image.layers() > 1;
    textureArrayLayers = std::max<std::size_t>(1, image.layers());
    gli::gl translator(gli::gl::PROFILE_GL33);
    const gli::gl::format format = translator.translate(image.format(), image.swizzles());
    if (format.Internal == 0 || !gli::is_compressed(image.format())) {
        return 0;
    }
    if (textureArray && pglCompressedTexImage3D == nullptr) {
        return 0;
    }
    GLuint texture = 0;
    glGenTextures(1, &texture);
    const GLenum target = textureArray ? GL_TEXTURE_2D_ARRAY : GL_TEXTURE_2D;
    glBindTexture(target, texture);
    glTexParameteri(target, GL_TEXTURE_MIN_FILTER,
                    image.levels() > 1 ? GL_LINEAR_MIPMAP_LINEAR : GL_LINEAR);
    glTexParameteri(target, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(target, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(target, GL_TEXTURE_WRAP_T, GL_REPEAT);
    for (std::size_t level = 0; level < image.levels(); ++level) {
        const auto extent = image.extent(level);
        if (textureArray) {
            pglCompressedTexImage3D(GL_TEXTURE_2D_ARRAY, static_cast<GLint>(level), format.Internal,
                                    static_cast<GLsizei>(extent.x), static_cast<GLsizei>(extent.y),
                                    static_cast<GLsizei>(textureArrayLayers), 0,
                                    static_cast<GLsizei>(image.size(level)),
                                    image.data(0, 0, level));
        } else {
            pglCompressedTexImage2D(GL_TEXTURE_2D, static_cast<GLint>(level), format.Internal,
                                    static_cast<GLsizei>(extent.x), static_cast<GLsizei>(extent.y),
                                    0, static_cast<GLsizei>(image.size(level)),
                                    image.data(0, 0, level));
        }
    }
    openbus::rendering::invalidateTextureBindings();
    assets->trackTexture(texture);
    static const bool verboseTextureUploadLogs =
        parseEnabledFlag(openbus::getEnvironment("OPENBUS_VERBOSE_TEXTURE_UPLOAD"));
    if (verboseTextureUploadLogs) {
        gameLog.Debug("Uploaded compressed texture from path: " + path.generic_string());
    }
    return texture;
}

GLuint Vehicle::uploadCompressedDds(const std::filesystem::path& path, const CompressedDds& image) {
    TraceScope trace("texture", "uploadCompressedDds");
    if (pglCompressedTexImage2D == nullptr || image.levels.empty()) {
        return 0;
    }
    static const bool s3tcSupported =
        hasOpenGLExtension("GL_EXT_texture_compression_s3tc") || hasOpenGLExtension("GL_S3_s3tc");
    if (!s3tcSupported) {
        return 0;
    }
    GLuint texture = 0;
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
                    image.levels.size() > 1 ? GL_LINEAR_MIPMAP_LINEAR : GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    int levelWidth = image.width;
    int levelHeight = image.height;
    for (std::size_t level = 0; level < image.levels.size(); ++level) {
        const auto& data = image.levels[level];
        pglCompressedTexImage2D(GL_TEXTURE_2D, static_cast<GLint>(level),
                                GL_COMPRESSED_RGBA_S3TC_DXT5_EXT, levelWidth, levelHeight, 0,
                                static_cast<GLsizei>(data.size()), data.data());
        if (glGetError() != GL_NO_ERROR) {
            glDeleteTextures(1, &texture);
            openbus::rendering::invalidateTextureBindings();
            return 0;
        }
        levelWidth = std::max(1, levelWidth / 2);
        levelHeight = std::max(1, levelHeight / 2);
    }
    openbus::rendering::invalidateTextureBindings();
    assets->trackTexture(texture);
    static const bool verboseTextureUploadLogs =
        parseEnabledFlag(openbus::getEnvironment("OPENBUS_VERBOSE_TEXTURE_UPLOAD"));
    if (verboseTextureUploadLogs) {
        gameLog.Debug("Uploaded DXT5 texture from path: " + path.generic_string());
    }
    return texture;
}

std::filesystem::path Vehicle::findTexture(const std::filesystem::path& root,
                                           const std::string& name) {
    TraceScope trace("texture", "findTexture");
    const std::string cleaned = trim(name);
    if (cleaned.empty()) {
        return {};
    }

    static const bool verboseTextureLookupLogs =
        parseEnabledFlag(openbus::getEnvironment("OPENBUS_VERBOSE_TEXTURE_LOOKUP"));

    const auto isImageExtension = [](const std::string& extension) {
        static const std::array<std::string, 6> imageExtensions = {".tga", ".bmp", ".png",
                                                                   ".dds", ".jpg", ".jpeg"};
        return std::find(imageExtensions.begin(), imageExtensions.end(), extension) !=
               imageExtensions.end();
    };

    const auto textureFormatPriority = [](const std::filesystem::path& path) {
        const std::string extension = lower(path.extension().string());
        if (extension == ".dds") {
            return 3;
        }
        if (extension == ".tga") {
            return 2;
        }
        return 1;
    };

    const auto normalizedPathKey = [&](const std::filesystem::path& path) {
        return lower(std::filesystem::absolute(path).lexically_normal().generic_string());
    };

    static std::mutex resolutionMutex;
    static std::unordered_map<std::string, std::filesystem::path> resolutionCache;
    struct TextureDirectoryIndex {
        std::mutex mutex;
        bool built = false;
        std::unordered_map<std::string, std::filesystem::path> byFileName;
        std::unordered_map<std::string, std::filesystem::path> byStem;
    };
    struct RootTextureDirectoryIndex {
        std::mutex mutex;
        bool built = false;
        std::unordered_map<std::string, std::filesystem::path> byStem;
    };
    static std::unordered_map<std::string, std::shared_ptr<TextureDirectoryIndex>> textureIndexes;
    static std::unordered_map<std::string, std::shared_ptr<RootTextureDirectoryIndex>>
        rootTextureIndexes;

    const std::filesystem::path normalizedRequestPath =
        std::filesystem::path(cleaned).lexically_normal();
    const std::string requestPathKey = lower(normalizedRequestPath.generic_string());
    const std::string resolutionKey = normalizedPathKey(root) + "|" + requestPathKey;

    {
        std::lock_guard<std::mutex> lock(resolutionMutex);
        const auto cached = resolutionCache.find(resolutionKey);
        if (cached != resolutionCache.end()) {
            return cached->second;
        }
    }

    if (verboseTextureLookupLogs) {
        gameLog.Debug("Resolving texture with name: " + name + " in root: " + root.generic_string());
    }

    const auto cacheResult = [&](const std::filesystem::path& path) {
        std::lock_guard<std::mutex> lock(resolutionMutex);
        resolutionCache.emplace(resolutionKey, path);
        return path;
    };

    const auto preferCompressedSibling = [&](const std::filesystem::path& path) {
        if (lower(path.extension().string()) == ".dds") {
            return path;
        }
        std::filesystem::path compressed = path;
        compressed.replace_extension(".dds");
        return std::filesystem::exists(compressed) ? compressed : path;
    };

    const auto getIndexForTextureRoot = [&](const std::filesystem::path& textureRoot) {
        const std::string indexKey = normalizedPathKey(textureRoot);
        std::shared_ptr<TextureDirectoryIndex> index;
        {
            std::lock_guard<std::mutex> lock(resolutionMutex);
            const auto found = textureIndexes.find(indexKey);
            if (found != textureIndexes.end()) {
                index = found->second;
            } else {
                index = std::make_shared<TextureDirectoryIndex>();
                textureIndexes.emplace(indexKey, index);
            }
        }
        {
            std::lock_guard<std::mutex> lock(index->mutex);
            if (!index->built) {
                for (const auto& entry : std::filesystem::recursive_directory_iterator(
                         textureRoot, std::filesystem::directory_options::skip_permission_denied)) {
                    if (!entry.is_regular_file()) {
                        continue;
                    }
                    const std::string extension = lower(entry.path().extension().string());
                    if (!isImageExtension(extension)) {
                        continue;
                    }
                    const std::string fileName = lower(entry.path().filename().string());
                    const std::string stem = lower(entry.path().stem().string());
                    index->byFileName.emplace(fileName, entry.path());
                    const auto existingStem = index->byStem.find(stem);
                    if (existingStem == index->byStem.end() ||
                        textureFormatPriority(entry.path()) >
                            textureFormatPriority(existingStem->second)) {
                        index->byStem[stem] = entry.path();
                    }
                }
                index->built = true;
            }
        }
        return index;
    };

    const auto getIndexForRoot = [&](const std::filesystem::path& textureRoot) {
        const std::string indexKey = normalizedPathKey(textureRoot);
        std::shared_ptr<RootTextureDirectoryIndex> index;
        {
            std::lock_guard<std::mutex> lock(resolutionMutex);
            const auto found = rootTextureIndexes.find(indexKey);
            if (found != rootTextureIndexes.end()) {
                index = found->second;
            } else {
                index = std::make_shared<RootTextureDirectoryIndex>();
                rootTextureIndexes.emplace(indexKey, index);
            }
        }
        {
            std::lock_guard<std::mutex> lock(index->mutex);
            if (!index->built) {
                std::error_code error;
                std::filesystem::directory_iterator entry(
                    textureRoot, std::filesystem::directory_options::skip_permission_denied, error);
                const std::filesystem::directory_iterator end;
                for (; !error && entry != end; entry.increment(error)) {
                    std::error_code statusError;
                    if (!entry->is_regular_file(statusError) || statusError) {
                        continue;
                    }
                    const std::string extension = lower(entry->path().extension().string());
                    if (!isImageExtension(extension)) {
                        continue;
                    }
                    const std::string stem = lower(entry->path().stem().string());
                    const auto existing = index->byStem.find(stem);
                    if (existing == index->byStem.end() ||
                        textureFormatPriority(entry->path()) >
                            textureFormatPriority(existing->second)) {
                        index->byStem[stem] = entry->path();
                    }
                }
                index->built = true;
            }
        }
        return index;
    };

    std::filesystem::path direct = root / cleaned;
    if (std::filesystem::exists(direct)) {
        return cacheResult(preferCompressedSibling(direct));
    }

    std::vector<std::filesystem::path> textureRoots;
    for (std::filesystem::path ancestor = root; !ancestor.empty();
         ancestor = ancestor.parent_path()) {
        const std::filesystem::path textureRoot = ancestor / "Texture";
        if (std::filesystem::exists(textureRoot)) {
            textureRoots.push_back(textureRoot);
            direct = textureRoot / cleaned;
            if (std::filesystem::exists(direct)) {
                return cacheResult(preferCompressedSibling(direct));
            }
        }
        const std::filesystem::path parent = ancestor.parent_path();
        if (parent == ancestor) {
            break;
        }
    }

    const std::string basename = lower(std::filesystem::path(cleaned).filename().string());
    const std::string requestedStem = lower(std::filesystem::path(cleaned).stem().string());
    if (std::filesystem::exists(root)) {
        const std::shared_ptr<RootTextureDirectoryIndex> index = getIndexForRoot(root);
        std::lock_guard<std::mutex> lock(index->mutex);
        const auto preferredDirect = index->byStem.find(requestedStem);
        if (preferredDirect != index->byStem.end()) {
            if (verboseTextureLookupLogs) {
                gameLog.Debug("Found texture directly at path: " +
                            preferredDirect->second.generic_string());
            }
            return cacheResult(preferredDirect->second);
        }
    }

    for (const std::filesystem::path& textureRoot : textureRoots) {
        const std::shared_ptr<TextureDirectoryIndex> index = getIndexForTextureRoot(textureRoot);
        std::lock_guard<std::mutex> lock(index->mutex);
        const auto byFileName = index->byFileName.find(basename);
        if (byFileName != index->byFileName.end()) {
            if (verboseTextureLookupLogs) {
                gameLog.Debug("Found texture at indexed filename path: " +
                            byFileName->second.generic_string());
            }
            return cacheResult(byFileName->second);
        }
        const auto byStem = index->byStem.find(requestedStem);
        if (byStem != index->byStem.end()) {
            if (verboseTextureLookupLogs) {
                gameLog.Debug("Found texture at indexed stem path: " +
                            byStem->second.generic_string());
            }
            return cacheResult(byStem->second);
        }
    }

    textureLog.Warning("missing: " + cleaned);
    return cacheResult({});
}

std::shared_ptr<Vehicle::TextureCacheEntry> Vehicle::textureEntry(Batch& batch) {
    TraceScope phase("texture", "textureEntry");
    return assets->requestTexture(batch.textureRoot, batch.texturePath, batch.textureName,
                                  [](const std::filesystem::path& root,
                                     const std::filesystem::path& path, const std::string& name) {
                                      return Vehicle::findTexture(root, name.empty() ? path.string()
                                                                                     : name);
                                  });
}

void Vehicle::ensureAuxiliaryTexture(Batch& batch, AuxiliaryTexture& auxiliary) {
    const int scriptIndex = openbus::scripting::scriptTextureReferenceIndex(auxiliary.name);
    if (scriptIndex >= 0) {
        ScriptRuntime::ScriptTextureSnapshot snapshot;
        // An unchanged revision returns an empty pixel buffer; never upload from it.
        if (scripts &&
            scripts->copyScriptTextureIfChanged(scriptIndex, auxiliary.revision, snapshot) &&
            snapshot.width > 0 && snapshot.height > 0 &&
            snapshot.pixels.size() >= static_cast<std::size_t>(snapshot.width) *
                                          static_cast<std::size_t>(snapshot.height) * 4U) {
            uploadDynamicTexture(snapshot, auxiliary.texture, auxiliary.width, auxiliary.height,
                                 auxiliary.revision, batch.textureWrapS, batch.textureWrapT);
        }
        return;
    }
    if (auxiliary.name.empty() || auxiliary.loadAttempted) {
        return;
    }
    if (!auxiliary.cacheEntry) {
        auxiliary.cacheEntry = assets->requestTexture(
            batch.textureRoot, {}, auxiliary.name,
            [](const std::filesystem::path& root, const std::filesystem::path& path,
               const std::string& name) {
                return Vehicle::findTexture(root, name.empty() ? path.string() : name);
            });
    }
    startTextureRequest(auxiliary.cacheEntry);
    const std::shared_ptr<TextureRequest>& request = auxiliary.cacheEntry->request;
    bool requestComplete = false;
    {
        std::lock_guard<std::mutex> lock(request->mutex);
        requestComplete = request->complete;
    }
    if (!requestComplete) {
        return;
    }
    if (!uploadTextureCacheEntry(auxiliary.cacheEntry)) {
        return;
    }
    auxiliary.texture = auxiliary.cacheEntry->texture;
    auxiliary.loadAttempted = true;
}

void Vehicle::updateFreeTexture(Batch& batch) {
    if (batch.textTextureIndex < 0 && batch.scriptTextureIndex < 0 &&
        !batch.freeTextureName.empty() && !batch.freeTextureVariable.empty()) {
        const std::string selectedName = variables.getString(batch.freeTextureVariable);
        const std::string textureName = selectedName.empty() ? batch.freeTextureName : selectedName;
        if (batch.freeTextureAsset.name != textureName) {
            batch.freeTextureAsset = {};
            batch.freeTextureAsset.name = textureName;
        }
        ensureAuxiliaryTexture(batch, batch.freeTextureAsset);
        batch.freeTexture = batch.freeTextureAsset.texture;
        return;
    }
    if ((batch.freeTextureVariable.empty() && batch.scriptTextureIndex < 0 &&
         batch.textTextureIndex < 0) ||
        !scripts) {
        return;
    }
    ScriptRuntime::ScriptTextureSnapshot snapshot;
    int index = -1;
    bool textTexture = false;
    if (batch.textTextureIndex >= 0) {
        index = batch.textTextureIndex;
        textTexture = true;
    } else if (batch.scriptTextureIndex >= 0) {
        index = batch.scriptTextureIndex;
    } else {
        index = static_cast<int>(std::lround(variables.get(batch.freeTextureVariable)));
    }
    const bool textureAlreadyCached = batch.freeTexture != 0 && batch.freeTextureIndex == index;
    const auto copyTexture = [&] {
        if (textureAlreadyCached) {
            return textTexture ? scripts->copyTextTextureIfChanged(index, batch.freeTextureRevision,
                                                                   snapshot)
                               : scripts->copyScriptTextureIfChanged(
                                     index, batch.freeTextureRevision, snapshot);
        }
        return textTexture ? scripts->copyTextTexture(index, snapshot)
                           : scripts->copyScriptTexture(index, snapshot);
    };
    const bool copied = copyTexture();
    if (textureAlreadyCached && snapshot.pixels.empty()) {
        return;
    }
    if (index < 0 || !copied || snapshot.width <= 0 || snapshot.height <= 0 ||
        snapshot.pixels.empty()) {
        return;
    }
    if (batch.freeTexture != 0 && batch.freeTextureIndex == index &&
        batch.freeTextureRevision == snapshot.revision) {
        return;
    }
    uploadDynamicTexture(snapshot, batch.dynamicTexture, batch.freeTextureWidth,
                         batch.freeTextureHeight, batch.freeTextureRevision, batch.textureWrapS,
                         batch.textureWrapT);
    batch.freeTexture = batch.dynamicTexture;
    batch.freeTextureIndex = index;
}

void Vehicle::uploadDynamicTexture(const ScriptRuntime::ScriptTextureSnapshot& snapshot,
                                   GLuint& texture, int& width, int& height,
                                   std::uint64_t& revision, GLenum wrapS, GLenum wrapT) {
    const bool dimensionsChanged = width != snapshot.width || height != snapshot.height;
    const bool textureCreated = texture == 0;
    if (textureCreated) {
        glGenTextures(1, &texture);
        assets->trackTexture(texture);
    }
    glBindTexture(GL_TEXTURE_2D, texture);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    const GLint filter = snapshot.filtered ? GL_LINEAR : GL_NEAREST;
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, filter);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, filter);
    // Dynamic script/text textures replace the material texture but must
    // keep its address mode. OMSI O3D UVs commonly span 1..2; clamping
    // those out-of-range coordinates collapses glyph sampling to an edge.
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, wrapS);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, wrapT);
    if (textureCreated || dimensionsChanged) {
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, snapshot.width, snapshot.height, 0, GL_RGBA,
                     GL_UNSIGNED_BYTE, snapshot.pixels.data());
        width = snapshot.width;
        height = snapshot.height;
    } else {
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, snapshot.width, snapshot.height, GL_RGBA,
                        GL_UNSIGNED_BYTE, snapshot.pixels.data());
    }
    openbus::rendering::invalidateTextureBindings();
    revision = snapshot.revision;
}

void Vehicle::startTextureRequest(const std::shared_ptr<TextureCacheEntry>& entry) {
    assets->startTextureRequest(entry);
}

void Vehicle::ensureTexture(Batch& batch, bool visible) {
    if (batch.textureLoadAttempted && batch.textureChanges.empty()) {
        return;
    }
    TraceScope phase("texture", "ensureTexture");
    updateMaterialChange(batch);
    if (batch.textureName.empty() && batch.texturePath.empty()) {
        batch.textureLoadAttempted = true;
        batch.textured = false;
        return;
    }
    if (batch.textureLoadAttempted) {
        return;
    }
    if (!batch.textureCacheEntry) {
        batch.textureCacheEntry = textureEntry(batch);
        batch.textureRequest = batch.textureCacheEntry->request;
        batch.textureLoadStarted = true;
    }
    startTextureRequest(batch.textureCacheEntry);
    if (!visible) {
        return;
    }
    {
        std::lock_guard<std::mutex> lock(batch.textureCacheEntry->request->mutex);
        if (!batch.textureCacheEntry->request->complete) {
            return;
        }
        batch.texturePath = batch.textureCacheEntry->request->resolvedPath;
    }
    if (!batch.textureCacheEntry->uploadAttempted) {
        if (loadingPolicy.textureMode == AssetLoadingMode::Deferred) {
            constexpr auto textureUploadBudget = std::chrono::milliseconds(2);
            if (std::chrono::steady_clock::now() - textureUploadStart >= textureUploadBudget) {
                return;
            }
        }
        if (!uploadTextureCacheEntry(batch.textureCacheEntry)) {
            return;
        }
    }
    batch.texture = batch.textureCacheEntry->texture;
    batch.textureArray = batch.textureCacheEntry->textureArray;
    batch.textureArrayLayers = batch.textureCacheEntry->textureArrayLayers;
    batch.textureLoadAttempted = true;
    batch.textured = batch.texture != 0;
    if ((batch.texturePath.empty() || batch.texture == 0) &&
        !batch.textureCacheEntry->decodeFailureLogged) {
        batch.textureCacheEntry->decodeFailureLogged = true;
        textureLog.Error("decode-failed: " + batch.textureName +
                       " resolved=" + batch.texturePath.generic_string() +
                       " source=" + batch.sourceMeshPath.generic_string());
    }
    // if (!batch.environmentLoadAttempted && !batch.environmentTextureName.empty() &&
    //     batch.environmentStrength > 0.0) {
    //     batch.environmentLoadAttempted = true;
    //     Batch environmentBatch;
    //     environmentBatch.textureRoot = batch.textureRoot;
    //     environmentBatch.textureName = batch.environmentTextureName;
    //     ensureTexture(environmentBatch, visible);
    //     batch.environmentTexture = environmentBatch.texture;
    // }
}

void Vehicle::ensureMaterialTexture(MaterialTextureSource& source, bool visible) {
    if (source.textureLoadAttempted) {
        return;
    }
    TraceScope phase("texture", "ensureMaterialTexture");
    if (!source.textureCacheEntry) {
        source.textureCacheEntry = assets->requestTexture(
            source.textureRoot, source.texturePath, source.textureName,
            [](const std::filesystem::path& root, const std::filesystem::path& path,
               const std::string& name) {
                return Vehicle::findTexture(root, name.empty() ? path.string() : name);
            });
        source.textureRequest = source.textureCacheEntry->request;
        source.textureLoadStarted = true;
    }
    startTextureRequest(source.textureCacheEntry);
    {
        std::lock_guard<std::mutex> lock(source.textureCacheEntry->request->mutex);
        if (!source.textureCacheEntry->request->complete) {
            return;
        }
        source.texturePath = source.textureCacheEntry->request->resolvedPath;
    }
    if (!visible) {
        return;
    }
    if (!source.textureCacheEntry->uploadAttempted) {
        if (loadingPolicy.textureMode == AssetLoadingMode::Deferred) {
            constexpr auto textureUploadBudget = std::chrono::milliseconds(2);
            if (std::chrono::steady_clock::now() - textureUploadStart >= textureUploadBudget) {
                return;
            }
        }
        if (!uploadTextureCacheEntry(source.textureCacheEntry)) {
            return;
        }
    }
    source.texture = source.textureCacheEntry->texture;
    source.textureArray = source.textureCacheEntry->textureArray;
    source.textureArrayLayers = source.textureCacheEntry->textureArrayLayers;
    source.textureLoadAttempted = true;
}

void Vehicle::ensureEnvironmentTexture(Batch& batch, bool visible) {
    if (batch.environmentLoadAttempted || batch.environmentTextureName.empty() ||
        batch.environmentStrength <= 0.0) {
        return;
    }
    TraceScope phase("texture", "ensureEnvironmentTexture");
    if (!batch.environmentTextureCacheEntry) {
        Batch request;
        request.textureRoot = batch.textureRoot;
        request.textureName = batch.environmentTextureName;
        batch.environmentTextureCacheEntry = textureEntry(request);
    }
    const std::shared_ptr<TextureCacheEntry>& entry = batch.environmentTextureCacheEntry;
    startTextureRequest(entry);
    {
        std::lock_guard<std::mutex> lock(entry->request->mutex);
        if (!entry->request->complete) {
            return;
        }
    }
    if (!visible) {
        return;
    }
    if (!entry->uploadAttempted) {
        if (!uploadTextureCacheEntry(entry)) {
            return;
        }
    }
    batch.environmentTexture = entry->texture;
    batch.environmentLoadAttempted = true;
}

void Vehicle::preloadTextures() {
    TraceScope trace("texture", "Vehicle::preloadTextures");
    const auto preload = [&](std::vector<DisplayPart>& parts) {
        for (DisplayPart& part : parts) {
            for (Batch& batch : part.batches) {
                if (!batch.materialTextures.empty()) {
                    for (MaterialTextureSource& source : batch.materialTextures) {
                        ensureMaterialTexture(source,
                                              loadingPolicy.textureMode == AssetLoadingMode::Eager);
                    }
                } else if (!batch.texturePath.empty() || !batch.textureName.empty()) {
                    ensureTexture(batch, loadingPolicy.textureMode == AssetLoadingMode::Eager);
                }
                ensureEnvironmentTexture(batch,
                                         loadingPolicy.textureMode == AssetLoadingMode::Eager);
                ensureAuxiliaryTexture(batch, batch.lightmap);
                ensureAuxiliaryTexture(batch, batch.nightmap);
                ensureAuxiliaryTexture(batch, batch.transmap);
                ensureAuxiliaryTexture(batch, batch.bumpmap);
            }
        }
    };
    preload(displayLists);
}
