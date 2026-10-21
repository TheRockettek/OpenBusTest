// Texture lookup, asynchronous loading and GPU upload.

#include "app/vehicle/Vehicle.h"
#include "config/ScriptTextureLimits.h"

bool Vehicle::areTexturesLoaded() const {
    if (!areObjectsLoaded()) {
        return false;
    }
    std::unordered_set<const TextureCacheEntry*> trackedTextures;
    const auto texturesLoadedInParts = [&](const std::vector<DisplayPart>& parts) {
        for (const DisplayPart& displayPart : parts) {
            for (const Batch& batch : displayPart.batches) {
                for (const MaterialTextureSource& source : batch.materialTextures) {
                    if (!source.textureCacheEntry) {
                        return false;
                    }
                    const TextureCacheEntry* sourceEntry = source.textureCacheEntry.get();
                    if (trackedTextures.insert(sourceEntry).second) {
                        std::lock_guard<std::mutex> sourceLock(sourceEntry->request->mutex);
                        if (!sourceEntry->request->complete) {
                            return false;
                        }
                    }
                }
                if (batch.materialTextures.empty() &&
                    (!batch.texturePath.empty() || !batch.textureName.empty())) {
                    if (!batch.textureCacheEntry) {
                        return false;
                    }
                    const TextureCacheEntry* entry = batch.textureCacheEntry.get();
                    if (trackedTextures.insert(entry).second) {
                        std::lock_guard<std::mutex> lock(entry->request->mutex);
                        if (!entry->request->complete) {
                            return false;
                        }
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
                    std::lock_guard<std::mutex> auxiliaryLock(auxiliaryEntry->request->mutex);
                    if (!auxiliaryEntry->request->complete) {
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

GLuint Vehicle::uploadTexture(const std::filesystem::path& path, Image image) {
    TraceScope trace("texture", "uploadTexture");
    static const bool verboseTextureUploadLogs =
        parseEnabledFlag(openbus::getEnvironment("OPENBUS_VERBOSE_TEXTURE_UPLOAD"));
    if (verboseTextureUploadLogs) {
        gameLog.Log("Uploading texture from path: " + path.generic_string());
    }
    if (textureScale < 0.999) {
        const int scaledWidth =
            std::max(1, static_cast<int>(std::lround(image.width * textureScale)));
        const int scaledHeight =
            std::max(1, static_cast<int>(std::lround(image.height * textureScale)));
        std::size_t scaledSize = 0;
        if (!openbus::rendering::checkedTextureBufferSize(static_cast<std::size_t>(scaledWidth),
                                                          static_cast<std::size_t>(scaledHeight), 4,
                                                          scaledSize)) {
            gameLog.Log("Texture scaling exceeds the decoded image size limit: " +
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
        image.width = scaledWidth;
        image.height = scaledHeight;
        image.rgba = std::move(scaled);
    }
    GLuint texture = 0;
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, image.width, image.height, 0, GL_RGBA, GL_UNSIGNED_BYTE,
                 image.rgba.data());
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
        gameLog.Log("Uploaded compressed texture from path: " + path.generic_string());
    }
    return texture;
}

GLuint Vehicle::uploadCompressedDds(const std::filesystem::path& path, const CompressedDds& image) {
    TraceScope trace("texture", "uploadCompressedDds");
    if (pglCompressedTexImage2D == nullptr || image.levels.empty()) {
        return 0;
    }
    const GLubyte* extensionBytes = glGetString(GL_EXTENSIONS);
    const std::string extensions =
        extensionBytes != nullptr ? reinterpret_cast<const char*>(extensionBytes) : std::string();
    if (extensions.find("GL_EXT_texture_compression_s3tc") == std::string::npos &&
        extensions.find("GL_S3_s3tc") == std::string::npos) {
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
        gameLog.Log("Uploaded DXT5 texture from path: " + path.generic_string());
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
    static std::unordered_map<std::string, std::shared_ptr<TextureDirectoryIndex>> textureIndexes;

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
        gameLog.Log("Resolving texture with name: " + name + " in root: " + root.generic_string());
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
        std::filesystem::path preferredDirect;
        for (const auto& entry : std::filesystem::directory_iterator(
                 root, std::filesystem::directory_options::skip_permission_denied)) {
            if (entry.is_regular_file() && lower(entry.path().stem().string()) == requestedStem &&
                isImageExtension(lower(entry.path().extension().string()))) {
                if (preferredDirect.empty() ||
                    textureFormatPriority(entry.path()) > textureFormatPriority(preferredDirect)) {
                    preferredDirect = entry.path();
                }
            }
        }
        if (!preferredDirect.empty()) {
            if (verboseTextureLookupLogs) {
                gameLog.Log("Found texture directly at path: " + preferredDirect.generic_string());
            }
            return cacheResult(preferredDirect);
        }
    }

    for (const std::filesystem::path& textureRoot : textureRoots) {
        const std::shared_ptr<TextureDirectoryIndex> index = getIndexForTextureRoot(textureRoot);
        std::lock_guard<std::mutex> lock(index->mutex);
        const auto byFileName = index->byFileName.find(basename);
        if (byFileName != index->byFileName.end()) {
            if (verboseTextureLookupLogs) {
                gameLog.Log("Found texture at indexed filename path: " +
                            byFileName->second.generic_string());
            }
            return cacheResult(byFileName->second);
        }
        const auto byStem = index->byStem.find(requestedStem);
        if (byStem != index->byStem.end()) {
            if (verboseTextureLookupLogs) {
                gameLog.Log("Found texture at indexed stem path: " +
                            byStem->second.generic_string());
            }
            return cacheResult(byStem->second);
        }
    }

    textureLog.Log("missing: " + cleaned);
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
    std::lock_guard<std::mutex> lock(request->mutex);
    if (!auxiliary.cacheEntry->uploadAttempted) {
        auxiliary.cacheEntry->uploadAttempted = true;
        if (!request->resolvedPath.empty()) {
            if (request->compressedDds) {
                auxiliary.cacheEntry->texture =
                    uploadCompressedDds(request->resolvedPath, *request->compressedDds);
            } else if (request->compressedTexture) {
                auxiliary.cacheEntry->texture = uploadCompressedTexture(
                    request->resolvedPath, *request->compressedTexture,
                    auxiliary.cacheEntry->textureArray, auxiliary.cacheEntry->textureArrayLayers);
            } else if (request->image) {
                auxiliary.cacheEntry->texture =
                    uploadTexture(request->resolvedPath, *request->image);
            }
        }
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
    bool copied = false;
    if (batch.textTextureIndex >= 0) {
        index = batch.textTextureIndex;
        copied = scripts->copyTextTexture(index, snapshot);
    } else if (batch.scriptTextureIndex >= 0) {
        index = batch.scriptTextureIndex;
        copied = scripts->copyScriptTexture(index, snapshot);
    } else {
        index = static_cast<int>(std::lround(variables.get(batch.freeTextureVariable)));
        copied = scripts->copyScriptTexture(index, snapshot);
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
    bool requestComplete = false;
    {
        std::lock_guard<std::mutex> lock(batch.textureCacheEntry->request->mutex);
        requestComplete = batch.textureCacheEntry->request->complete;
    }
    if (!requestComplete) {
        return;
    }
    std::lock_guard<std::mutex> lock(batch.textureCacheEntry->request->mutex);
    if (!batch.textureCacheEntry->request->complete) {
        return;
    }
    if (!visible) {
        return;
    }
    batch.texturePath = batch.textureCacheEntry->request->resolvedPath;
    if (!batch.textureCacheEntry->uploadAttempted) {
        if (loadingPolicy.textureMode == AssetLoadingMode::Deferred) {
            constexpr auto textureUploadBudget = std::chrono::milliseconds(2);
            if (std::chrono::steady_clock::now() - textureUploadStart >= textureUploadBudget) {
                return;
            }
        }
        batch.textureCacheEntry->uploadAttempted = true;
        if (!batch.texturePath.empty()) {
            if (batch.textureCacheEntry->request->compressedDds) {
                batch.textureCacheEntry->texture = uploadCompressedDds(
                    batch.texturePath, *batch.textureCacheEntry->request->compressedDds);
                if (batch.textureCacheEntry->texture == 0) {
                    Image fallbackImage;
                    if (openbus::rendering::TextureLoader::readImage(batch.texturePath,
                                                                     fallbackImage)) {
                        batch.textureCacheEntry->texture =
                            uploadTexture(batch.texturePath, std::move(fallbackImage));
                    }
                }
            } else if (batch.textureCacheEntry->request->compressedTexture) {
                batch.textureCacheEntry->texture = uploadCompressedTexture(
                    batch.texturePath, *batch.textureCacheEntry->request->compressedTexture,
                    batch.textureCacheEntry->textureArray,
                    batch.textureCacheEntry->textureArrayLayers);
            } else {
                batch.textureCacheEntry->texture =
                    uploadTexture(batch.texturePath, *batch.textureCacheEntry->request->image);
            }
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
        textureLog.Log("decode-failed: " + batch.textureName +
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
    bool requestComplete = false;
    {
        std::lock_guard<std::mutex> lock(source.textureCacheEntry->request->mutex);
        requestComplete = source.textureCacheEntry->request->complete;
    }
    if (!requestComplete || !visible) {
        return;
    }
    std::lock_guard<std::mutex> lock(source.textureCacheEntry->request->mutex);
    if (!source.textureCacheEntry->request->complete) {
        return;
    }
    source.texturePath = source.textureCacheEntry->request->resolvedPath;
    if (!source.textureCacheEntry->uploadAttempted) {
        if (loadingPolicy.textureMode == AssetLoadingMode::Deferred) {
            constexpr auto textureUploadBudget = std::chrono::milliseconds(2);
            if (std::chrono::steady_clock::now() - textureUploadStart >= textureUploadBudget) {
                return;
            }
        }
        source.textureCacheEntry->uploadAttempted = true;
        if (!source.texturePath.empty()) {
            if (source.textureCacheEntry->request->compressedDds) {
                source.textureCacheEntry->texture = uploadCompressedDds(
                    source.texturePath, *source.textureCacheEntry->request->compressedDds);
                if (source.textureCacheEntry->texture == 0) {
                    Image fallbackImage;
                    if (openbus::rendering::TextureLoader::readImage(source.texturePath,
                                                                     fallbackImage)) {
                        source.textureCacheEntry->texture =
                            uploadTexture(source.texturePath, std::move(fallbackImage));
                    }
                }
            } else if (source.textureCacheEntry->request->compressedTexture) {
                source.textureCacheEntry->texture = uploadCompressedTexture(
                    source.texturePath, *source.textureCacheEntry->request->compressedTexture,
                    source.textureCacheEntry->textureArray,
                    source.textureCacheEntry->textureArrayLayers);
            } else if (source.textureCacheEntry->request->image) {
                source.textureCacheEntry->texture =
                    uploadTexture(source.texturePath, *source.textureCacheEntry->request->image);
            }
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
    bool requestComplete = false;
    {
        std::lock_guard<std::mutex> lock(entry->request->mutex);
        requestComplete = entry->request->complete;
    }
    if (!requestComplete || !visible) {
        return;
    }
    std::lock_guard<std::mutex> lock(entry->request->mutex);
    if (!entry->uploadAttempted) {
        entry->uploadAttempted = true;
        if (!entry->request->resolvedPath.empty()) {
            if (entry->request->compressedDds) {
                entry->texture = uploadCompressedDds(entry->request->resolvedPath,
                                                     *entry->request->compressedDds);
                if (entry->texture == 0) {
                    Image fallbackImage;
                    if (openbus::rendering::TextureLoader::readImage(entry->request->resolvedPath,
                                                                     fallbackImage)) {
                        entry->texture =
                            uploadTexture(entry->request->resolvedPath, std::move(fallbackImage));
                    }
                }
            } else if (entry->request->compressedTexture) {
                entry->texture = uploadCompressedTexture(
                    entry->request->resolvedPath, *entry->request->compressedTexture,
                    entry->textureArray, entry->textureArrayLayers);
            } else if (entry->request->image) {
                entry->texture =
                    uploadTexture(entry->request->resolvedPath, *entry->request->image);
            }
        }
    }
    batch.environmentTexture = entry->texture;
    batch.environmentLoadAttempted = true;
}

void Vehicle::preloadTextures() {
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
