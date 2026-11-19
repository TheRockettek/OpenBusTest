#include "app/vehicle/Vehicle.h"

#include "app/vehicle/DiligentVehicleSourceBridge.h"
#include "assets/VehicleTextureResolver.h"
#include "config/ScriptTextureLimits.h"

bool Vehicle::prepareDiligentTextureRequests() {
    if (displayLists.empty()) {
        return false;
    }

    const auto findTexture = [](const std::filesystem::path& root,
                                const std::filesystem::path& path, const std::string& name) {
        return Vehicle::findTexture(root, name.empty() ? path.string() : name);
    };
    const auto startRequest = [this](const std::shared_ptr<TextureCacheEntry>& entry) {
        if (entry) {
            startTextureRequest(entry);
        }
    };
    const auto startAuxiliaryRequest = [&](Batch& batch, AuxiliaryTexture& auxiliary) {
        if (auxiliary.name.empty() ||
            openbus::scripting::scriptTextureReferenceIndex(auxiliary.name) >= 0) {
            return;
        }
        if (!auxiliary.cacheEntry) {
            auxiliary.cacheEntry =
                assets->requestTexture(batch.textureRoot, {}, auxiliary.name, findTexture);
        }
        startRequest(auxiliary.cacheEntry);
    };

    for (DisplayPart& part : displayLists) {
        for (Batch& batch : part.batches) {
            updateMaterialChange(batch, false);

            if (batch.materialTextures.empty()) {
                if ((!batch.textureName.empty() || !batch.texturePath.empty()) &&
                    !batch.textureCacheEntry) {
                    batch.textureCacheEntry = textureEntry(batch);
                    batch.textureRequest = batch.textureCacheEntry->request;
                    batch.textureLoadStarted = true;
                }
                startRequest(batch.textureCacheEntry);
            } else {
                for (MaterialTextureSource& source : batch.materialTextures) {
                    if (!source.textureCacheEntry) {
                        source.textureCacheEntry =
                            assets->requestTexture(source.textureRoot, source.texturePath,
                                                   source.textureName, findTexture);
                        source.textureRequest = source.textureCacheEntry->request;
                        source.textureLoadStarted = true;
                    }
                    startRequest(source.textureCacheEntry);
                }
            }

            for (AuxiliaryTexture* auxiliary :
                 {&batch.lightmap, &batch.nightmap, &batch.transmap, &batch.bumpmap}) {
                startAuxiliaryRequest(batch, *auxiliary);
            }

            if (batch.environmentStrength > 0.0 && !batch.environmentTextureName.empty()) {
                if (!batch.environmentTextureCacheEntry) {
                    Batch request;
                    request.textureRoot = batch.textureRoot;
                    request.textureName = batch.environmentTextureName;
                    batch.environmentTextureCacheEntry = textureEntry(request);
                }
                startRequest(batch.environmentTextureCacheEntry);
            }

            if (batch.textTextureIndex < 0 && batch.scriptTextureIndex < 0 &&
                !batch.freeTextureName.empty() && !batch.freeTextureVariable.empty()) {
                const std::string selectedName = variables.getString(batch.freeTextureVariable);
                const std::string textureName =
                    selectedName.empty() ? batch.freeTextureName : selectedName;
                if (batch.freeTextureAsset.name != textureName) {
                    invalidateDiligentAsset();
                    batch.freeTextureAsset = {};
                    batch.freeTextureAsset.name = textureName;
                }
                startAuxiliaryRequest(batch, batch.freeTextureAsset);
            }
        }
    }

    return true;
}

bool Vehicle::prepareDiligentBatchSources(
    openbus::app::vehicle::DiligentVehicleBatchSourceSet& sources) {
    if (!prepareDiligentTextureRequests()) {
        sources = {};
        return false;
    }
    return openbus::app::vehicle::makeDiligentVehicleBatchSources(displayLists, sources);
}

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
    return texturesLoadedInParts(displayLists);
}

std::filesystem::path Vehicle::findTexture(const std::filesystem::path& root,
                                           const std::string& name) {
    static openbus::assets::VehicleTextureResolver resolver;
    return resolver.find(root, name);
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

void Vehicle::startTextureRequest(const std::shared_ptr<TextureCacheEntry>& entry) {
    assets->startTextureRequest(entry);
}

void Vehicle::preloadTextures() {
    TraceScope trace("texture", "Vehicle::preloadTextures");
    static_cast<void>(prepareDiligentTextureRequests());
}
