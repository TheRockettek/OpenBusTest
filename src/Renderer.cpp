#include "Renderer.h"

#include "BusConfigLoader.h"
#include "BusConfiguration.h"
#include "BusModelLoader.h"
#include "BusSimulation.h"
#include "CameraMath.h"
#include "Logger.h"
#include "ObjLoader.h"
#include "PerfTrace.h"
#include "RenderPrimitives.h"
#include "RoadFeatures.h"
#include "ScreenshotWriter.h"
#include "TextureLoader.h"
#include "Variables.h"

#ifndef GLFW_INCLUDE_NONE
#define GLFW_INCLUDE_NONE
#endif
#include <GLFW/glfw3.h>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif
#include <GL/gl.h>
#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <future>
#include <gli/gli.hpp>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#ifndef GL_ARRAY_BUFFER
#define GL_ARRAY_BUFFER 0x8892
#define GL_STATIC_DRAW 0x88E4
#endif

#ifndef GL_COMPRESSED_RGBA_S3TC_DXT5_EXT
#define GL_COMPRESSED_RGBA_S3TC_DXT5_EXT 0x83F3
#endif

#ifndef GL_TEXTURE_2D_ARRAY
#define GL_TEXTURE_2D_ARRAY 0x8C1A
#endif

#ifndef GL_TEXTURE0
#define GL_TEXTURE0 0x84C0
#define GL_TEXTURE1 0x84C1
#define GL_TEXTURE_ENV 0x2300
#define GL_TEXTURE_ENV_MODE 0x2200
#define GL_COMBINE 0x8570
#define GL_REPLACE 0x1E01
#define GL_MODULATE 0x2100
#define GL_COMBINE_RGB 0x8571
#define GL_COMBINE_ALPHA 0x8572
#define GL_SOURCE0_RGB 0x8580
#define GL_SOURCE0_ALPHA 0x8588
#define GL_SOURCE1_ALPHA 0x8589
#define GL_OPERAND0_RGB 0x8590
#define GL_OPERAND0_ALPHA 0x8598
#define GL_OPERAND1_ALPHA 0x8599
#define GL_PREVIOUS 0x8578
#define GL_SRC_ALPHA 0x0302
#define GL_TEXTURE 0x1702
#endif

Logger gameLog = Logger("Game");
Logger textureLog = Logger("Texture");
Logger wheelLog = Logger("Wheel");

namespace {

enum class RenderViewContext {
    PlayerExterior = 1,
    PlayerInterior = 2,
    NonPlayer = 4,
};

constexpr int viewpointMask(RenderViewContext context) {
    return static_cast<int>(context);
}

constexpr double MAN_DL05_MODEL_OFFSET_Z = -1.035;
using GenBuffersProc = void (*)(GLsizei, GLuint*);
using BindBufferProc = void (*)(GLenum, GLuint);
using BufferDataProc = void (*)(GLenum, std::ptrdiff_t, const void*, GLenum);
using DeleteBuffersProc = void (*)(GLsizei, const GLuint*);
using CompressedTexImage2DProc = void (*)(GLenum, GLint, GLenum, GLsizei, GLsizei, GLint, GLsizei,
                                          const void*);
using CompressedTexImage3DProc = void (*)(GLenum, GLint, GLenum, GLsizei, GLsizei, GLsizei, GLint,
                                          GLsizei, const void*);
using ActiveTextureProc = void (*)(GLenum);
GenBuffersProc pglGenBuffers = nullptr;
BindBufferProc pglBindBuffer = nullptr;
BufferDataProc pglBufferData = nullptr;
DeleteBuffersProc pglDeleteBuffers = nullptr;
CompressedTexImage2DProc pglCompressedTexImage2D = nullptr;
CompressedTexImage3DProc pglCompressedTexImage3D = nullptr;
ActiveTextureProc pglActiveTexture = nullptr;

bool loadBufferFunctions() {
    // VBO and compressed-texture entry points are loaded after the GLFW context
    // exists because legacy OpenGL headers do not expose them portably.
    pglGenBuffers = reinterpret_cast<GenBuffersProc>(glfwGetProcAddress("glGenBuffers"));
    pglBindBuffer = reinterpret_cast<BindBufferProc>(glfwGetProcAddress("glBindBuffer"));
    pglBufferData = reinterpret_cast<BufferDataProc>(glfwGetProcAddress("glBufferData"));
    pglDeleteBuffers = reinterpret_cast<DeleteBuffersProc>(glfwGetProcAddress("glDeleteBuffers"));
    pglCompressedTexImage2D =
        reinterpret_cast<CompressedTexImage2DProc>(glfwGetProcAddress("glCompressedTexImage2D"));
    pglCompressedTexImage3D =
        reinterpret_cast<CompressedTexImage3DProc>(glfwGetProcAddress("glCompressedTexImage3D"));
    pglActiveTexture = reinterpret_cast<ActiveTextureProc>(glfwGetProcAddress("glActiveTexture"));
    const bool available = pglGenBuffers && pglBindBuffer && pglBufferData && pglDeleteBuffers &&
                           pglCompressedTexImage2D && pglActiveTexture;
    if (!available) {
        gameLog.Log("Failed to load required OpenGL VBO functions");
    }
    return available;
}
} // namespace

using openbus::rendering::applyPose;
using openbus::rendering::drawBox;
using openbus::rendering::drawCenterOfGravityMarker;
using openbus::rendering::drawGround;
using openbus::rendering::drawWheel;
using openbus::rendering::lookAt;
using openbus::rendering::parseEnabledFlag;
using openbus::rendering::setPerspective;
using openbus::rendering::TraceScope;
using openbus::rendering::transformLocalPoint;
using ObjIndex = openbus::rendering::ObjIndex;
using ObjNormal = openbus::rendering::ObjNormal;
using ObjPosition = openbus::rendering::ObjPosition;
using ObjTexCoord = openbus::rendering::ObjTexCoord;
using ObjTriangle = openbus::rendering::ObjTriangle;
using ParsedObj = openbus::rendering::ParsedObj;
using Image = openbus::rendering::Image;
using CompressedDds = openbus::rendering::CompressedDds;

struct BusModel {
    // TODO: Split model parsing, texture loading, and rendering into focused components.
    ModelLoadingPolicy loadingPolicy;
    struct TextureRequest {
        // Decode work runs asynchronously; the mutex protects completion and
        // decoded payload ownership while the render thread uploads to GL.
        std::filesystem::path root;
        std::filesystem::path path;
        std::string name;
        std::filesystem::path resolvedPath;
        std::shared_ptr<Image> image;
        std::shared_ptr<gli::texture> compressedTexture;
        std::shared_ptr<CompressedDds> compressedDds;
        bool complete = false;
        bool started = false;
        std::mutex mutex;
        std::shared_future<void> task;
    };

    struct DecodedTexture {
        std::shared_ptr<Image> image;
        std::shared_ptr<gli::texture> compressedTexture;
        std::shared_ptr<CompressedDds> compressedDds;
    };

    struct TextureCacheEntry {
        std::shared_ptr<TextureRequest> request;
        GLuint texture = 0;
        bool textureArray = false;
        std::size_t textureArrayLayers = 1;
        bool uploadAttempted = false;
    };

    struct ObjRequest {
        std::shared_future<std::shared_ptr<openbus::rendering::ParsedObj>> future;
    };

    using MaterialState = openbus::rendering::BusModelMaterialState;
    using WheelAnimation = openbus::rendering::WheelAnimation;

    struct Part : openbus::rendering::BusModelPart {
        std::shared_ptr<ObjRequest> objRequest;
    };

    struct Vertex {
        float x, y, z, u, v, layer, nx, ny, nz;
    };

    struct Batch {
        GLuint buffer = 0;
        GLuint texture = 0;
        bool textureArray = false;
        std::size_t textureArrayLayers = 1;
        std::filesystem::path baseTexturePath;
        std::string baseTextureName;
        std::filesystem::path texturePath;
        std::filesystem::path textureRoot;
        std::string textureName;
        std::string environmentTextureName;
        GLuint environmentTexture = 0;
        double environmentStrength = 0.0;
        bool environmentLoadAttempted = false;
        std::shared_ptr<TextureCacheEntry> environmentTextureCacheEntry;
        std::array<double, 3> color = {0.65, 0.65, 0.65};
        bool textured = false;
        bool hasNormals = false;
        bool textureLoadAttempted = false;
        bool textureLoadStarted = false;
        std::shared_ptr<TextureRequest> textureRequest;
        std::shared_ptr<TextureCacheEntry> textureCacheEntry;
        int alphaMode = 0;
        bool noZwrite = false;
        std::string alphaScaleVariable;
        int baseTextureLayer = 0;
        int textureLayer = 0;
        std::vector<Vertex> vertices;
        std::vector<MaterialState::TextureChange> textureChanges;
        std::size_t vertexCount = 0;
    };

    struct DisplayPart {
        // Geometry metadata is immutable after loading; visibility is still
        // evaluated from live variables during each draw.
        std::vector<Batch> batches;
        int viewpoint;
        int renderType = 2;
        bool transparent;
        int lodIndex;
        std::string visibleVariable;
        int visibleValue = 0;
        std::array<double, 3> center;
        std::array<double, 3> size;
        double radius;
        std::size_t triangleCount;
    };

    struct WheelModel {
        WheelAnimation animation;
        std::vector<DisplayPart> parts;
    };

    std::vector<DisplayPart> displayLists;
    std::vector<WheelModel> wheelModels;
    Variables variables;
    std::vector<Part> pendingParts;
    std::vector<GLuint> textures;
    std::unordered_map<std::string, std::shared_ptr<TextureCacheEntry>> textureCache;
    std::unordered_map<std::string, std::shared_ptr<TextureCacheEntry>> textureAliases;
    std::mutex decodedTextureMutex;
    std::unordered_map<std::string, std::shared_ptr<DecodedTexture>> decodedTextureCache;
    std::mutex parsedObjMutex;
    std::unordered_map<std::string, std::shared_future<std::shared_ptr<ParsedObj>>> parsedObjCache;
    mutable std::unordered_set<std::size_t> loggedWheelBindings;
    double modelOffsetZ = MAN_DL05_MODEL_OFFSET_Z;
    double textureScale = 1;
    bool frustumCulling = true;
    std::vector<double> lodThresholds;
    std::size_t opaqueDisplayCount = 0;
    mutable std::size_t lastRenderedTriangles = 0;
    bool loaded = false;
    bool hasLoadedInitialView = false;
    bool loggedAllObjectsLoaded = false;
    int activeLod = -1;
    std::chrono::steady_clock::time_point textureUploadStart;

    void loadTextureRequest(const std::shared_ptr<TextureRequest>& request);

    void updateMaterialChange(Batch& batch) {
        TraceScope phase("texture", "updateMaterialChange");
        // Select the last active material change, then reset only the texture
        // request state so the new texture is resolved and uploaded.
        const MaterialState::TextureChange* selected = nullptr;
        for (const MaterialState::TextureChange& change : batch.textureChanges) {
            if (variables.get(change.activationVariable) != 0.0) {
                selected = &change;
            }
        }
        const std::filesystem::path texturePath =
            selected == nullptr ? batch.baseTexturePath : selected->texturePath;
        const std::string textureName =
            selected == nullptr ? batch.baseTextureName : selected->textureName;
        const int requestedLayer = selected == nullptr ? batch.baseTextureLayer : selected->layer;
        const int layer =
            batch.textureArray
                ? std::clamp(requestedLayer, 0, static_cast<int>(batch.textureArrayLayers) - 1)
                : 0;
        const bool sameRequestedTexture = textureName == batch.textureName &&
                                          (texturePath == batch.texturePath ||
                                           batch.textureLoadAttempted);
        if (sameRequestedTexture && layer == batch.textureLayer) {
            return;
        }
        batch.texturePath = texturePath;
        batch.textureName = textureName;
        batch.textureCacheEntry.reset();
        batch.textureRequest.reset();
        batch.texture = 0;
        batch.textured = false;
        batch.textureLoadAttempted = false;
        batch.textureLoadStarted = false;
        if (layer != batch.textureLayer) {
            batch.textureLayer = layer;
            for (Vertex& vertex : batch.vertices) {
                vertex.layer = static_cast<float>(layer);
            }
            if (!batch.vertices.empty()) {
                pglBindBuffer(GL_ARRAY_BUFFER, batch.buffer);
                pglBufferData(GL_ARRAY_BUFFER,
                              static_cast<std::ptrdiff_t>(batch.vertices.size() * sizeof(Vertex)),
                              batch.vertices.data(), GL_STATIC_DRAW);
                pglBindBuffer(GL_ARRAY_BUFFER, 0);
            }
        }
    }

    double alphaScale(const Batch& batch) const {
        if (batch.alphaScaleVariable.empty()) {
            return 1.0;
        }
        return std::clamp(variables.get(batch.alphaScaleVariable), 0.0, 1.0);
    }

    void updateFrameVariables(double timegap, double getTime, double mouseX, double mouseY) {
        // Frame-scoped values are refreshed before simulation and rendering run.
        variables.updateFrame(timegap, getTime, mouseX, mouseY);
    }

    void joinTextureWorkers() {
        for (const auto& cache : textureCache) {
            if (cache.second->request && cache.second->request->task.valid()) {
                cache.second->request->task.wait();
            }
        }
    }

#ifdef _WIN32
    bool comInitialized = false;
#endif

    explicit BusModel(BusVehicle vehicle, ModelLoadingPolicy policy) : loadingPolicy(policy) {
        if (const char* scale = std::getenv("OPENBUS_TEXTURE_SCALE")) {
            try {
                textureScale = std::clamp(std::stod(scale), 0.25, 1.0);
            } catch (const std::exception&) {
                textureScale = 1.0;
            }
        }
        if (const char* culling = std::getenv("OPENBUS_FRUSTUM_CULLING")) {
            frustumCulling = std::string(culling) == "1";
        }
#ifdef _WIN32
        const HRESULT comResult = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
        comInitialized = SUCCEEDED(comResult);
#endif
        std::filesystem::path relativeConfig;
        std::filesystem::path relativeModelRoot;
        if (vehicle == BusVehicle::SpE400Mmc) {
            relativeConfig = std::filesystem::path("SP_E400MMC") / "Model" / "Configuration Files" /
                             "E400MMC_ADL_10.9m_Voith_LowHeight.cfg";
            relativeModelRoot = std::filesystem::path("SP_E400MMC") / "Model" / "Converted" /
                                "E400MMC_ADL_10.9m_Voith_LowHeight_obj";
            modelOffsetZ = -1.02;
        } else {
            relativeConfig = std::filesystem::path("MAN_DL05") / "Model" / "DL05.cfg";
            relativeModelRoot = std::filesystem::path("MAN_DL05") / "Model" / "DL05";
        }
        const auto resolveAssetPath = [](const std::filesystem::path& relative) {
            const std::array<std::filesystem::path, 4> candidates = {
                relative, std::filesystem::current_path() / relative,
                std::filesystem::current_path().parent_path() / relative,
                std::filesystem::current_path().parent_path().parent_path() / relative};
            for (const auto& candidate : candidates) {
                if (std::filesystem::exists(candidate)) {
                    return candidate;
                }
            }
            return relative;
        };
        gameLog.Log("Loading bus model with config: " + resolveAssetPath(relativeConfig).string() +
                    " and model root: " + resolveAssetPath(relativeModelRoot).string());
        load(resolveAssetPath(relativeConfig), resolveAssetPath(relativeModelRoot));
        spawn();
    }

    ~BusModel() {
        joinTextureWorkers();
        const auto deleteBuffers = [](const std::vector<DisplayPart>& parts) {
            for (const DisplayPart& part : parts) {
                for (const Batch& batch : part.batches) {
                    if (batch.buffer) {
                        pglDeleteBuffers(1, &batch.buffer);
                    }
                }
            }
        };
        deleteBuffers(displayLists);
        for (const WheelModel& wheel : wheelModels) {
            deleteBuffers(wheel.parts);
        }
        if (!textures.empty()) {
            glDeleteTextures(static_cast<GLsizei>(textures.size()), textures.data());
        }
#ifdef _WIN32
        if (comInitialized) {
            CoUninitialize();
        }
#endif
    }

    void draw(RenderViewContext context) {
        TraceScope trace("render", "BusModel::draw");
        if (!loaded) {
            return;
        }
        // Texture uploads must happen on the OpenGL thread, so decoding and GL
        // upload are deliberately split between the worker and draw paths.
        textureUploadStart = std::chrono::steady_clock::now();

        const auto& modelView = openbus::rendering::modelViewMatrix();
        std::array<std::array<double, 4>, 6> frustumPlanes = {};
        std::array<double, 6> frustumPlaneLengths = {};

        {
            TraceScope phase("render", "BusModel::draw.setup");

            glDisable(GL_TEXTURE_2D);
            glDisable(GL_BLEND);
            glDisable(GL_ALPHA_TEST);
            glDepthMask(GL_TRUE);
            glEnable(GL_DEPTH_TEST);
            glEnable(GL_POLYGON_OFFSET_FILL);
            glPolygonOffset(-1.0f, -1.0f);
        }

        {
            if (frustumCulling) {
                TraceScope phase("render", "BusModel::draw.frustumSetup");
                const auto& projection = openbus::rendering::projectionMatrix();
                const std::array<std::array<double, 4>, 6> planeSigns = {{{{1.0, 0.0, 0.0, 1.0}},
                                                                          {{-1.0, 0.0, 0.0, 1.0}},
                                                                          {{0.0, 1.0, 0.0, 1.0}},
                                                                          {{0.0, -1.0, 0.0, 1.0}},
                                                                          {{0.0, 0.0, 1.0, 1.0}},
                                                                          {{0.0, 0.0, -1.0, 1.0}}}};
                for (std::size_t plane = 0; plane < planeSigns.size(); ++plane) {
                    const auto& signs = planeSigns[plane];
                    double lengthSquared = 0.0;
                    for (int column = 0; column < 4; ++column) {
                        double coefficient = 0.0;
                        for (int row = 0; row < 4; ++row) {
                            coefficient += signs[row] * projection[row + column * 4];
                        }
                        frustumPlanes[plane][column] = coefficient;
                        if (column < 3) {
                            lengthSquared += coefficient * coefficient;
                        }
                    }
                    frustumPlaneLengths[plane] = std::sqrt(lengthSquared);
                }
            }
        }

        const auto visible = [&](const DisplayPart& part) {
            // TraceScope phase("render", "BusModel::draw.visible");
            if (!part.visibleVariable.empty() &&
                variables.get(part.visibleVariable) != static_cast<double>(part.visibleValue)) {
                return false;
            }
            const double local[4] = {part.center[0], part.center[1], part.center[2] + modelOffsetZ,
                                     1.0};
            double eye[4] = {};
            for (int row = 0; row < 4; ++row) {
                for (int column = 0; column < 4; ++column) {
                    eye[row] += modelView[row + column * 4] * local[column];
                }
            }
            const double distance = std::sqrt(eye[0] * eye[0] + eye[1] * eye[1] + eye[2] * eye[2]);
            if (!lodThresholds.empty() && part.lodIndex >= 0) {
                std::size_t selectedLod = lodThresholds.size() - 1;
                for (std::size_t index = 0; index < lodThresholds.size(); ++index) {
                    const double boundary = 25.0 / std::max(lodThresholds[index], 0.001);
                    if (distance <= boundary) {
                        selectedLod = index;
                        break;
                    }
                }
                if (static_cast<std::size_t>(part.lodIndex) != selectedLod) {
                    return false;
                }
            }
            if (!frustumCulling) {
                return true;
            }
            for (std::size_t plane = 0; plane < frustumPlanes.size(); ++plane) {
                const double planeDistance =
                    frustumPlanes[plane][0] * eye[0] + frustumPlanes[plane][1] * eye[1] +
                    frustumPlanes[plane][2] * eye[2] + frustumPlanes[plane][3];
                if (planeDistance < -part.radius * frustumPlaneLengths[plane]) {
                    return false;
                }
            }
            return true;
        };
        const auto viewDepth = [&](const DisplayPart& part) {
            // TraceScope phase("render", "BusModel::draw.viewDepth");
            const double local[4] = {part.center[0], part.center[1], part.center[2] + modelOffsetZ,
                                     1.0};
            double eyeZ = 0.0;
            for (int column = 0; column < 4; ++column) {
                eyeZ += modelView[2 + column * 4] * local[column];
            }
            return -eyeZ;
        };
        struct TransparentBatch {
            Batch* batch;
            double depth;
            int renderType;
        };
        std::vector<DisplayPart*> opaqueParts;
        std::vector<TransparentBatch> noDepthOpaqueBatches;
        std::vector<TransparentBatch> transparentBatches;
        {
            TraceScope phase("render", "BusModel::draw.classifyParts");
            for (DisplayPart& part : displayLists) {
                const bool viewpointMatches =
                    part.viewpoint == 0 || (part.viewpoint & viewpointMask(context)) != 0;
                if (!viewpointMatches || !visible(part)) {
                    continue;
                }
                const bool hasOpaqueBatch =
                    std::any_of(part.batches.begin(), part.batches.end(), [](const Batch& batch) {
                        return batch.alphaMode == 0 && !batch.noZwrite;
                    });
                for (Batch& batch : part.batches) {
                    if (batch.alphaMode != 0 || batch.noZwrite) {
                            ensureTexture(batch);
                            transparentBatches.push_back(
                                {&batch, viewDepth(part), part.renderType});
                    }
                }
                if (hasOpaqueBatch) {
                    opaqueParts.push_back(&part);
                }
            }
        }
        {
            TraceScope phase("render", "BusModel::draw.sortBatches");
            std::sort(opaqueParts.begin(), opaqueParts.end(),
                      [&](const DisplayPart* first, const DisplayPart* second) {
                          return viewDepth(*first) < viewDepth(*second);
                      });
            std::stable_sort(transparentBatches.begin(), transparentBatches.end(),
                             [](const TransparentBatch& first, const TransparentBatch& second) {
                                 return first.renderType < second.renderType;
                             });
            // TODO: Sort transparent batches back-to-front within each render type.
            std::stable_sort(noDepthOpaqueBatches.begin(), noDepthOpaqueBatches.end(),
                             [](const TransparentBatch& first, const TransparentBatch& second) {
                                 return first.renderType < second.renderType;
                             });
            lastRenderedTriangles = 0;
            for (DisplayPart* part : opaqueParts) {
                lastRenderedTriangles += part->triangleCount;
            }
            for (const TransparentBatch& transparent : transparentBatches) {
                lastRenderedTriangles += transparent.batch->vertexCount / 3;
            }
        }
        glPushMatrix();
        glTranslated(0.0, 0.0, modelOffsetZ);
        {
            TraceScope phase("render", "BusModel::draw.opaquePass");
            for (DisplayPart* part : opaqueParts) {
                for (Batch& batch : part->batches) {
                    const double alpha = alphaScale(batch);
                    if (alpha <= 0.0) {
                        continue;
                    }
                    if (batch.alphaMode != 0 || batch.noZwrite) {
                        continue;
                    }
                    if (batch.alphaMode == 2 || batch.noZwrite) {
                        glEnable(GL_BLEND);
                        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
                        glDepthMask(GL_TRUE);
                    } else if (batch.alphaMode == 1) {
                        glDisable(GL_BLEND);
                        glEnable(GL_ALPHA_TEST);
                        glAlphaFunc(GL_GREATER, 0.5f);
                        glDepthMask(GL_FALSE);
                    } else {
                        glDisable(GL_BLEND);
                        glDisable(GL_ALPHA_TEST);
                        glDepthMask(GL_TRUE);
                    }
                    ensureTexture(batch);
                    if (batch.textured) {
                        glEnable(GL_TEXTURE_2D);
                        glBindTexture(batch.textureArray ? GL_TEXTURE_2D_ARRAY : GL_TEXTURE_2D,
                                      batch.texture);
                        glColor4d(1.0, 1.0, 1.0, alpha);
                    } else {
                        glDisable(GL_TEXTURE_2D);
                        glColor4d(batch.color[0], batch.color[1], batch.color[2], alpha);
                    }
                    pglBindBuffer(GL_ARRAY_BUFFER, batch.buffer);
                    glEnableClientState(GL_VERTEX_ARRAY);
                    glVertexPointer(3, GL_FLOAT, sizeof(Vertex), reinterpret_cast<const void*>(0));
                    if (batch.textured) {
                        glEnableClientState(GL_TEXTURE_COORD_ARRAY);
                        glTexCoordPointer(batch.textureArray ? 3 : 2, GL_FLOAT, sizeof(Vertex),
                                          reinterpret_cast<const void*>(3 * sizeof(float)));
                    }
                    glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(batch.vertexCount));
                    glDisableClientState(GL_TEXTURE_COORD_ARRAY);
                    glDisableClientState(GL_VERTEX_ARRAY);
                    drawEnvironmentMap(batch, alpha);
                }
            }
        }
        glDepthMask(GL_FALSE);
        glDisable(GL_BLEND);
        glDisable(GL_ALPHA_TEST);
        {
            TraceScope phase("render", "BusModel::draw.noDepthPass");
            for (const TransparentBatch& noDepthOpaque : noDepthOpaqueBatches) {
                Batch& batch = *noDepthOpaque.batch;
                const double alpha = alphaScale(batch);
                if (alpha <= 0.0) {
                    continue;
                }
                ensureTexture(batch);
                if (batch.textured) {
                    glEnable(GL_TEXTURE_2D);
                    glBindTexture(batch.textureArray ? GL_TEXTURE_2D_ARRAY : GL_TEXTURE_2D,
                                  batch.texture);
                    glColor4d(1.0, 1.0, 1.0, alpha);
                } else {
                    glDisable(GL_TEXTURE_2D);
                    glColor4d(batch.color[0], batch.color[1], batch.color[2], alpha);
                }
                pglBindBuffer(GL_ARRAY_BUFFER, batch.buffer);
                glEnableClientState(GL_VERTEX_ARRAY);
                glVertexPointer(3, GL_FLOAT, sizeof(Vertex), nullptr);
                if (batch.textured) {
                    glEnableClientState(GL_TEXTURE_COORD_ARRAY);
                    glTexCoordPointer(batch.textureArray ? 3 : 2, GL_FLOAT, sizeof(Vertex),
                                      reinterpret_cast<const void*>(3 * sizeof(float)));
                }
                glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(batch.vertexCount));
                glDisableClientState(GL_TEXTURE_COORD_ARRAY);
                glDisableClientState(GL_VERTEX_ARRAY);
                drawEnvironmentMap(batch, alpha);
            }
        }
        glDepthMask(GL_FALSE);
        {
            TraceScope phase("render", "BusModel::draw.transparentPass");
            for (const TransparentBatch& transparent : transparentBatches) {
                Batch& batch = *transparent.batch;
                const double alpha = alphaScale(batch);
                if (alpha <= 0.0) {
                    continue;
                }
                if (batch.alphaMode == 1) {
                    glDisable(GL_BLEND);
                    glEnable(GL_ALPHA_TEST);
                    glAlphaFunc(GL_GREATER, 0.5f);
                    glEnable(GL_POLYGON_OFFSET_FILL);
                    glPolygonOffset(-1.0f, -1.0f);
                } else if (batch.alphaMode == 2) {
                    glEnable(GL_BLEND);
                    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
                    glDisable(GL_ALPHA_TEST);
                    glDisable(GL_POLYGON_OFFSET_FILL);
                } else if (batch.noZwrite) {
                    glEnable(GL_BLEND);
                    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
                    glDisable(GL_ALPHA_TEST);
                    glDisable(GL_POLYGON_OFFSET_FILL);
                } else {
                    glDisable(GL_BLEND);
                    glDisable(GL_ALPHA_TEST);
                    glDisable(GL_POLYGON_OFFSET_FILL);
                }
                ensureTexture(batch);
                if (batch.textured) {
                    glEnable(GL_TEXTURE_2D);
                    glBindTexture(batch.textureArray ? GL_TEXTURE_2D_ARRAY : GL_TEXTURE_2D,
                                  batch.texture);
                    glColor4d(1.0, 1.0, 1.0, alpha);
                } else {
                    glDisable(GL_TEXTURE_2D);
                    glColor4d(batch.color[0], batch.color[1], batch.color[2], alpha);
                }
                pglBindBuffer(GL_ARRAY_BUFFER, batch.buffer);
                glEnableClientState(GL_VERTEX_ARRAY);
                glVertexPointer(3, GL_FLOAT, sizeof(Vertex), nullptr);
                if (batch.textured) {
                    glEnableClientState(GL_TEXTURE_COORD_ARRAY);
                    glTexCoordPointer(batch.textureArray ? 3 : 2, GL_FLOAT, sizeof(Vertex),
                                      reinterpret_cast<const void*>(3 * sizeof(float)));
                }
                glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(batch.vertexCount));
                glDisableClientState(GL_TEXTURE_COORD_ARRAY);
                glDisableClientState(GL_VERTEX_ARRAY);
                drawEnvironmentMap(batch, alpha);
            }
        }
        {
            TraceScope phase("render", "BusModel::draw.cleanup");
            glDisableClientState(GL_TEXTURE_COORD_ARRAY);
            glDisableClientState(GL_NORMAL_ARRAY);
            glDisableClientState(GL_VERTEX_ARRAY);
            pglBindBuffer(GL_ARRAY_BUFFER, 0);
            glDepthMask(GL_TRUE);
            glDisable(GL_TEXTURE_2D);
            glDisable(GL_BLEND);
            glDisable(GL_ALPHA_TEST);
            glDisable(GL_POLYGON_OFFSET_FILL);
            glDepthMask(GL_TRUE);
            glColor4d(1.0, 1.0, 1.0, 1.0);
        }
        glPopMatrix();
    }

    bool hasConfiguredWheels(std::size_t expectedWheelCount) const {
        static_cast<void>(expectedWheelCount);
        return std::any_of(wheelModels.begin(), wheelModels.end(),
                           [](const WheelModel& wheel) { return !wheel.parts.empty(); });
    }

    void drawConfiguredWheels(const BusSimulation& simulation, const BodyPose& chassis,
                              bool outsideView) {
        std::vector<bool> usedWheelIndices(simulation.wheelCount(), false);
        for (std::size_t modelIndex = 0; modelIndex < wheelModels.size(); ++modelIndex) {
            WheelModel& wheel = wheelModels[modelIndex];
            if (wheel.parts.empty() || !wheel.animation.hasOrigin) {
                continue;
            }
            std::array<double, 3> targetLocal = wheel.animation.origin;
            targetLocal[2] += modelOffsetZ;
            const std::array<double, 3> target = transformLocalPoint(chassis, targetLocal);
            std::size_t simulationIndex = 0;
            double closestDistance = std::numeric_limits<double>::max();
            for (std::size_t index = 0; index < simulation.wheelCount(); ++index) {
                if (usedWheelIndices[index]) {
                    continue;
                }
                const BodyPose pose = simulation.wheelPose(index);
                const double deltaX = pose.position[0] - target[0];
                const double deltaY = pose.position[1] - target[1];
                const double deltaZ = pose.position[2] - target[2];
                const double distance = deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ;
                if (distance < closestDistance) {
                    closestDistance = distance;
                    simulationIndex = index;
                }
            }
            if (closestDistance == std::numeric_limits<double>::max()) {
                continue;
            }
            usedWheelIndices[simulationIndex] = true;
            const BodyPose pose = simulation.wheelPose(simulationIndex);
            if (loggedWheelBindings.insert(modelIndex).second) {
                wheelLog.Log("cfgWheel=" + std::to_string(modelIndex) +
                             " odeWheel=" + std::to_string(simulationIndex) + " origin=(" +
                             std::to_string(wheel.animation.origin[0]) + ',' +
                             std::to_string(wheel.animation.origin[1]) + ',' +
                             std::to_string(wheel.animation.origin[2]) + ')' +
                             " rotationVar=" + wheel.animation.rotationVariable +
                             " suspensionVar=" + wheel.animation.suspensionVariable +
                             " steeringVar=" + wheel.animation.steeringVariable + " pose=(" +
                             std::to_string(pose.position[0]) + ',' +
                             std::to_string(pose.position[1]) + ',' +
                             std::to_string(pose.position[2]) + ")");
            }
            const std::array<double, 3> meshOrigin = wheel.animation.origin;
            double visualWheelWidth = 0.0;
            double visualWheelDiameter = 0.0;
            for (const DisplayPart& part : wheel.parts) {
                visualWheelWidth = std::max(visualWheelWidth, part.size[1]);
                visualWheelDiameter =
                    std::max(visualWheelDiameter, std::max(part.size[0], part.size[2]));
            }
            const BusAxle axle = simulation.axle(simulationIndex / 2);
            const double outerHalfTrack =
                axle.maxWidth > 0.0 ? axle.maxWidth * 0.5 : axle.trackWidth * 0.5;
            const double innerHalfTrack =
                axle.minWidth > 0.0 ? axle.minWidth * 0.5 : axle.trackWidth * 0.5;
            const double currentCenter = axle.trackWidth * 0.5;
            double desiredCenter = currentCenter;
            if (!axle.steerable && visualWheelWidth > 0.0) {
                desiredCenter = outerHalfTrack - visualWheelWidth * 0.5;
                desiredCenter = std::max(desiredCenter, innerHalfTrack + visualWheelWidth * 0.5);
            }
            const double sideSign = simulationIndex % 2 == 0 ? 1.0 : -1.0;
            const double lateralOffset = sideSign * (desiredCenter - currentCenter);
            const double diameterScale = visualWheelDiameter > 0.0 && axle.wheelDiameter > 0.0
                                             ? axle.wheelDiameter / visualWheelDiameter
                                             : 1.0;
            for (DisplayPart& part : wheel.parts) {
                const bool visibleOutside = part.viewpoint == 0 || (part.viewpoint & 1) != 0;
                const bool visibleInside = part.viewpoint == 0 || (part.viewpoint & 2) != 0;
                if ((outsideView && !visibleOutside) || (!outsideView && !visibleInside)) {
                    continue;
                }
                glPushMatrix();
                applyPose(pose);
                glRotated(90.0, 1.0, 0.0, 0.0);
                glTranslated(0.0, lateralOffset, 0.0);
                glScaled(diameterScale, diameterScale, diameterScale);
                glTranslated(-meshOrigin[0], -meshOrigin[1], -meshOrigin[2]);
                for (Batch& batch : part.batches) {
                    ensureTexture(batch);
                    if (batch.alphaMode == 2 || batch.noZwrite) {
                        glEnable(GL_BLEND);
                        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
                        glDepthMask(GL_FALSE);
                    } else if (batch.alphaMode == 1) {
                        glDisable(GL_BLEND);
                        glEnable(GL_ALPHA_TEST);
                        glAlphaFunc(GL_GREATER, 0.5f);
                        glDepthMask(GL_FALSE);
                    } else {
                        glDisable(GL_BLEND);
                        glDisable(GL_ALPHA_TEST);
                        glDepthMask(GL_TRUE);
                    }
                    if (batch.textured && !isWheelRubberTexture(batch.textureName)) {
                        glEnable(GL_TEXTURE_2D);
                        glBindTexture(batch.textureArray ? GL_TEXTURE_2D_ARRAY : GL_TEXTURE_2D,
                                      batch.texture);
                        glColor3d(1.0, 1.0, 1.0);
                    } else {
                        glDisable(GL_TEXTURE_2D);
                        if (isWheelRubberTexture(batch.textureName)) {
                            glColor3d(0.20, 0.20, 0.20);
                        } else {
                            glColor3d(batch.color[0], batch.color[1], batch.color[2]);
                        }
                    }
                    pglBindBuffer(GL_ARRAY_BUFFER, batch.buffer);
                    glEnableClientState(GL_VERTEX_ARRAY);
                    glVertexPointer(3, GL_FLOAT, sizeof(Vertex), nullptr);
                    if (batch.textured) {
                        glEnableClientState(GL_TEXTURE_COORD_ARRAY);
                        glTexCoordPointer(batch.textureArray ? 3 : 2, GL_FLOAT, sizeof(Vertex),
                                          reinterpret_cast<const void*>(3 * sizeof(float)));
                    } else {
                        glDisableClientState(GL_TEXTURE_COORD_ARRAY);
                    }
                    glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(batch.vertexCount));
                    glDisableClientState(GL_TEXTURE_COORD_ARRAY);
                    glDisableClientState(GL_VERTEX_ARRAY);
                }
                glPopMatrix();
            }
        }
        for (std::size_t index = 0; index < simulation.wheelCount(); ++index) {
            if (usedWheelIndices[index]) {
                continue;
            }
            const BodyPose pose = simulation.wheelPose(index);
            glPushMatrix();
            applyPose(pose);
            drawWheel(simulation.wheelRadius(index), simulation.wheelHalfWidth());
            glPopMatrix();
        }
        pglBindBuffer(GL_ARRAY_BUFFER, 0);
        glDepthMask(GL_TRUE);
        glDisable(GL_TEXTURE_2D);
        glDisable(GL_BLEND);
        glDisable(GL_ALPHA_TEST);
        glColor4d(1.0, 1.0, 1.0, 1.0);
    }

    std::size_t renderedTriangles() const {
        return lastRenderedTriangles;
    }

    bool areObjectsLoaded() const {
        return loaded && pendingParts.empty();
    }

    bool areTexturesLoaded() const {
        if (!areObjectsLoaded()) {
            return false;
        }
        std::unordered_set<const TextureCacheEntry*> trackedTextures;
        const auto texturesLoadedInParts = [&](const std::vector<DisplayPart>& parts) {
            for (const DisplayPart& displayPart : parts) {
                for (const Batch& batch : displayPart.batches) {
                    if (batch.texturePath.empty() && batch.textureName.empty()) {
                        continue;
                    }
                    if (!batch.textureCacheEntry) {
                        return false;
                    }
                    const TextureCacheEntry* entry = batch.textureCacheEntry.get();
                    if (!trackedTextures.insert(entry).second) {
                        continue;
                    }
                    std::lock_guard<std::mutex> lock(entry->request->mutex);
                    if (!entry->request->complete) {
                        return false;
                    }
                }
            }
            return true;
        };
        if (!texturesLoadedInParts(displayLists)) {
            return false;
        }
        for (const WheelModel& wheel : wheelModels) {
            if (!texturesLoadedInParts(wheel.parts)) {
                return false;
            }
        }
        return true;
    }

    bool isCaptureReady() const {
        return areObjectsLoaded() && areTexturesLoaded();
    }

    void spawn() {
        TraceScope trace("obj", "spawn");
        for (Part& part : pendingParts) {
            loadObj(part, parsedObjFuture(part.objPath).get());
        }
        pendingParts.clear();
        rebuildDisplayOrder();
        preloadTextures();
        loaded = !displayLists.empty();
        hasLoadedInitialView = true;
        if (!loggedAllObjectsLoaded && loaded) {
            std::size_t wheelPartCount = 0;
            for (const WheelModel& wheel : wheelModels) {
                wheelPartCount += wheel.parts.size();
            }
            gameLog.Log("All objects loaded. bodyParts=" + std::to_string(displayLists.size()) +
                        " wheelParts=" + std::to_string(wheelPartCount));
            loggedAllObjectsLoaded = true;
        }
    }

  private:
    static std::string trim(const std::string& value) {
        const std::size_t first = value.find_first_not_of(" \t\r\n");
        if (first == std::string::npos) {
            return {};
        }
        const std::size_t last = value.find_last_not_of(" \t\r\n");
        return value.substr(first, last - first + 1);
    }

    static std::string lower(std::string value) {
        std::transform(value.begin(), value.end(), value.begin(), [](unsigned char character) {
            return static_cast<char>(std::tolower(character));
        });
        return value;
    }

    static int parseInt(const std::string& value, int fallback) {
        try {
            return std::stoi(trim(value));
        } catch (const std::exception&) {
            return fallback;
        }
    }

    static double parseDouble(const std::string& value, double fallback) {
        try {
            return std::stod(trim(value));
        } catch (const std::exception&) {
            return fallback;
        }
    }

    static bool isWheelRubberTexture(const std::string& textureName) {
        const std::string stem = lower(std::filesystem::path(textureName).stem().string());
        return stem == "e4_wheel_tyre" || stem == "e4_wheel_tread";
    }

    GLuint uploadTexture(const std::filesystem::path& path, Image image) {
        TraceScope trace("texture", "uploadTexture");
        static const bool verboseTextureUploadLogs =
            parseEnabledFlag(std::getenv("OPENBUS_VERBOSE_TEXTURE_UPLOAD"));
        if (verboseTextureUploadLogs) {
            gameLog.Log("Uploading texture from path: " + path.generic_string());
        }
        if (textureScale < 0.999) {
            const int scaledWidth =
                std::max(1, static_cast<int>(std::lround(image.width * textureScale)));
            const int scaledHeight =
                std::max(1, static_cast<int>(std::lround(image.height * textureScale)));
            std::vector<std::uint8_t> scaled(static_cast<std::size_t>(scaledWidth) * scaledHeight *
                                             4);
            for (int y = 0; y < scaledHeight; ++y) {
                const int sourceY = std::min(image.height - 1, static_cast<int>(y / textureScale));
                for (int x = 0; x < scaledWidth; ++x) {
                    const int sourceX =
                        std::min(image.width - 1, static_cast<int>(x / textureScale));
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
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, image.width, image.height, 0, GL_RGBA,
                     GL_UNSIGNED_BYTE, image.rgba.data());
        textures.push_back(texture);
        return texture;
    }

    GLuint uploadCompressedTexture(const std::filesystem::path& path, const gli::texture& image,
                                   bool& textureArray, std::size_t& textureArrayLayers) {
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
                pglCompressedTexImage3D(
                    GL_TEXTURE_2D_ARRAY, static_cast<GLint>(level), format.Internal,
                    static_cast<GLsizei>(extent.x), static_cast<GLsizei>(extent.y),
                    static_cast<GLsizei>(textureArrayLayers), 0,
                    static_cast<GLsizei>(image.size(level)), image.data(0, 0, level));
            } else {
                pglCompressedTexImage2D(
                    GL_TEXTURE_2D, static_cast<GLint>(level), format.Internal,
                    static_cast<GLsizei>(extent.x), static_cast<GLsizei>(extent.y), 0,
                    static_cast<GLsizei>(image.size(level)), image.data(0, 0, level));
            }
        }
        textures.push_back(texture);
        static const bool verboseTextureUploadLogs =
            parseEnabledFlag(std::getenv("OPENBUS_VERBOSE_TEXTURE_UPLOAD"));
        if (verboseTextureUploadLogs) {
            gameLog.Log("Uploaded compressed texture from path: " + path.generic_string());
        }
        return texture;
    }

    GLuint uploadCompressedDds(const std::filesystem::path& path, const CompressedDds& image) {
        TraceScope trace("texture", "uploadCompressedDds");
        if (pglCompressedTexImage2D == nullptr || image.levels.empty()) {
            return 0;
        }
        const GLubyte* extensionBytes = glGetString(GL_EXTENSIONS);
        const std::string extensions = extensionBytes != nullptr
                                           ? reinterpret_cast<const char*>(extensionBytes)
                                           : std::string();
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
        textures.push_back(texture);
        static const bool verboseTextureUploadLogs =
            parseEnabledFlag(std::getenv("OPENBUS_VERBOSE_TEXTURE_UPLOAD"));
        if (verboseTextureUploadLogs) {
            gameLog.Log("Uploaded DXT5 texture from path: " + path.generic_string());
        }
        return texture;
    }

    std::filesystem::path findTexture(const std::filesystem::path& root, const std::string& name) {
        TraceScope trace("texture", "findTexture");
        const std::string cleaned = trim(name);
        if (cleaned.empty()) {
            return {};
        }

        static const bool verboseTextureLookupLogs =
            parseEnabledFlag(std::getenv("OPENBUS_VERBOSE_TEXTURE_LOOKUP"));

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
        static std::unordered_map<std::string, std::shared_ptr<TextureDirectoryIndex>>
            textureIndexes;

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
            gameLog.Log("Resolving texture with name: " + name +
                        " in root: " + root.generic_string());
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
                             textureRoot,
                             std::filesystem::directory_options::skip_permission_denied)) {
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
                if (entry.is_regular_file() &&
                    lower(entry.path().stem().string()) == requestedStem &&
                    isImageExtension(lower(entry.path().extension().string()))) {
                    if (preferredDirect.empty() || textureFormatPriority(entry.path()) >
                                                       textureFormatPriority(preferredDirect)) {
                        preferredDirect = entry.path();
                    }
                }
            }
            if (!preferredDirect.empty()) {
                if (verboseTextureLookupLogs) {
                    gameLog.Log("Found texture directly at path: " +
                                preferredDirect.generic_string());
                }
                return cacheResult(preferredDirect);
            }
        }

        for (const std::filesystem::path& textureRoot : textureRoots) {
            const std::shared_ptr<TextureDirectoryIndex> index =
                getIndexForTextureRoot(textureRoot);
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

    std::string textureCacheKey(const Batch& batch) const {
        const std::filesystem::path identity =
            batch.texturePath.empty() ? batch.textureRoot / batch.textureName : batch.texturePath;
        return lower(std::filesystem::absolute(identity).lexically_normal().generic_string());
    }

    std::shared_ptr<TextureCacheEntry> textureEntry(Batch& batch) {
        TraceScope phase("texture", "textureEntry");
        const std::string key = textureCacheKey(batch);
        const auto found = textureCache.find(key);
        if (found != textureCache.end()) {
            return found->second;
        }
        const std::string alias =
            lower(std::filesystem::path(batch.texturePath.empty() ? batch.textureName
                                                                  : batch.texturePath.string())
                      .stem()
                      .generic_string());
        if (!alias.empty()) {
            // TODO: Scope aliases by normalized texture root/path to avoid stem collisions.
            const auto aliasFound = textureAliases.find(alias);
            if (aliasFound != textureAliases.end()) {
                textureCache.emplace(key, aliasFound->second);
                return aliasFound->second;
            }
        }
        auto entry = std::make_shared<TextureCacheEntry>();
        entry->request = std::make_shared<TextureRequest>();
        entry->request->root = batch.textureRoot;
        entry->request->path = batch.texturePath;
        entry->request->name = batch.textureName;
        textureCache.emplace(key, entry);
        if (!alias.empty()) {
            textureAliases.emplace(alias, entry);
        }
        return entry;
    }

    void startTextureRequest(const std::shared_ptr<TextureCacheEntry>& entry) {
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

    void ensureTexture(Batch& batch, bool visible = true) {
        TraceScope phase("texture", "ensureTexture");
        updateMaterialChange(batch);
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
        if (batch.texturePath.empty() || batch.texture == 0) {
            textureLog.Log("decode-failed: " + batch.textureName +
                           " resolved=" + batch.texturePath.generic_string());
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

    void ensureEnvironmentTexture(Batch& batch, bool visible = true) {
        TraceScope phase("texture", "ensureEnvironmentTexture");
        if (batch.environmentLoadAttempted || batch.environmentTextureName.empty() ||
            batch.environmentStrength <= 0.0) {
            return;
        }
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
                        if (openbus::rendering::TextureLoader::readImage(
                                entry->request->resolvedPath, fallbackImage)) {
                            entry->texture =
                                uploadTexture(entry->request->resolvedPath, std::move(fallbackImage));
                        }
                    }
                } else if (entry->request->compressedTexture) {
                    entry->texture = uploadCompressedTexture(
                        entry->request->resolvedPath, *entry->request->compressedTexture,
                        entry->textureArray, entry->textureArrayLayers);
                } else if (entry->request->image) {
                    entry->texture = uploadTexture(entry->request->resolvedPath,
                                                   *entry->request->image);
                }
            }
        }
        batch.environmentTexture = entry->texture;
        batch.environmentLoadAttempted = true;
    }

    void drawEnvironmentMap(Batch& batch, double alpha) {
        ensureEnvironmentTexture(batch);
        if (batch.environmentTexture == 0 || !batch.hasNormals || alpha <= 0.0) {
            return;
        }
        const double reflectionStrength =
            std::clamp(alpha * batch.environmentStrength, 0.0, 1.0);
        if (reflectionStrength <= 0.0) {
            return;
        }

        const GLboolean blendEnabled = glIsEnabled(GL_BLEND);
        const GLboolean normalizeEnabled = glIsEnabled(GL_NORMALIZE);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glEnable(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, batch.environmentTexture);
        glDisableClientState(GL_TEXTURE_COORD_ARRAY);
        glEnableClientState(GL_NORMAL_ARRAY);
        glNormalPointer(GL_FLOAT, sizeof(Vertex),
                        reinterpret_cast<const void*>(6 * sizeof(float)));
        glEnable(GL_NORMALIZE);
        glTexGeni(GL_S, GL_TEXTURE_GEN_MODE, GL_SPHERE_MAP);
        glTexGeni(GL_T, GL_TEXTURE_GEN_MODE, GL_SPHERE_MAP);
        glEnable(GL_TEXTURE_GEN_S);
        glEnable(GL_TEXTURE_GEN_T);
        glColor4d(1.0, 1.0, 1.0, reflectionStrength);
        glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(batch.vertexCount));
        glDisable(GL_TEXTURE_GEN_S);
        glDisable(GL_TEXTURE_GEN_T);
        glDisableClientState(GL_NORMAL_ARRAY);
        if (!normalizeEnabled) {
            glDisable(GL_NORMALIZE);
        }
        if (!blendEnabled) {
            glDisable(GL_BLEND);
        }
        glColor4d(1.0, 1.0, 1.0, 1.0);
    }

    void preloadTextures() {
        const auto preload = [&](std::vector<DisplayPart>& parts) {
            for (DisplayPart& part : parts) {
                for (Batch& batch : part.batches) {
                    if (!batch.texturePath.empty() || !batch.textureName.empty()) {
                        ensureTexture(batch, loadingPolicy.textureMode == AssetLoadingMode::Eager);
                    }
                    ensureEnvironmentTexture(batch,
                                             loadingPolicy.textureMode == AssetLoadingMode::Eager);
                }
            }
        };
        preload(displayLists);
        for (WheelModel& wheel : wheelModels) {
            preload(wheel.parts);
        }
    }

    std::shared_future<std::shared_ptr<ParsedObj>>
    parsedObjFuture(const std::filesystem::path& path) {
        const std::string key =
            lower(std::filesystem::absolute(path).lexically_normal().generic_string());
        std::lock_guard<std::mutex> lock(parsedObjMutex);
        const auto cached = parsedObjCache.find(key);
        if (cached != parsedObjCache.end()) {
            return cached->second;
        }
        std::shared_future<std::shared_ptr<ParsedObj>> future =
            std::async(std::launch::async, &openbus::rendering::ObjLoader::parse, path).share();
        parsedObjCache.emplace(key, future);
        return future;
    }

    void loadObj(const Part& part, const std::shared_ptr<ParsedObj>& parsed) {
        TraceScope trace("obj", "loadObj");
        const std::string sourceStem = lower(part.objPath.stem().string());
        if (sourceStem == "shadow") {
            return;
        }
        static const bool verboseObjLoadLogs =
            parseEnabledFlag(std::getenv("OPENBUS_VERBOSE_OBJ_LOAD"));
        if (verboseObjLoadLogs) {
            gameLog.Log("Loading OBJ model from path: " + part.objPath.generic_string());
        }
        if (!parsed) {
            return;
        }
        const auto& positions = parsed->positions;
        const auto& normals = parsed->normals;
        const auto& texCoords = parsed->texCoords;
        const auto& triangles = parsed->triangles;
        const auto& materials = parsed->materials;
        const auto& boundsCenter = parsed->boundsCenter;
        const auto& boundsSize = parsed->boundsSize;
        const double boundsRadius = parsed->boundsRadius;

        WheelAnimation wheelAnimation = part.wheelAnimation;
        if (!wheelAnimation.rotationVariable.empty() && !wheelAnimation.hasOrigin) {
            wheelAnimation.origin = boundsCenter;
            wheelAnimation.hasOrigin = true;
        }
        std::vector<DisplayPart>* destination = &displayLists;
        if (!wheelAnimation.rotationVariable.empty()) {
            auto wheel = std::find_if(
                wheelModels.begin(), wheelModels.end(), [&](const WheelModel& candidate) {
                    const bool sameAnimation =
                        wheelAnimation.rotationVariable == candidate.animation.rotationVariable &&
                        wheelAnimation.suspensionVariable ==
                            candidate.animation.suspensionVariable &&
                        wheelAnimation.steeringVariable == candidate.animation.steeringVariable &&
                        wheelAnimation.hasOrigin == candidate.animation.hasOrigin &&
                        wheelAnimation.origin == candidate.animation.origin;
                    if (sameAnimation) {
                        return true;
                    }
                    return false;
                });
            if (wheel == wheelModels.end()) {
                wheelModels.push_back({wheelAnimation, {}});
                wheel = wheelModels.end() - 1;
            }
            destination = &wheel->parts;
        }
        auto makeBatch = [&](const std::vector<const ObjTriangle*>& source,
                             const std::filesystem::path& texturePath,
                             const std::string& textureName, const std::array<double, 3>& color,
                             const std::string& environmentTextureName, double environmentStrength,
                             int alphaMode, bool noZwrite, const std::string& alphaScaleVariable,
                             const std::vector<MaterialState::TextureChange>& textureChanges) {
            TraceScope batchTrace("obj", "loadObj.makeBatch");
            std::vector<Vertex> vertices;
            vertices.reserve(source.size() * 3);
            const auto convertPosition = [](const ObjPosition& position) {
                return std::array<double, 3>{position.z, -position.x, position.y};
            };
            const auto normalizeVector = [](std::array<double, 3> value) {
                const double length =
                    std::sqrt(value[0] * value[0] + value[1] * value[1] + value[2] * value[2]);
                if (length <= 1.0e-8) {
                    return std::array<double, 3>{0.0, 0.0, 1.0};
                }
                for (double& component : value) {
                    component /= length;
                }
                return value;
            };
            {
                TraceScope vertexTrace("obj", "loadObj.buildVertices");
                for (const ObjTriangle* triangle : source) {
                    std::array<double, 3> fallbackNormal = {0.0, 0.0, 1.0};
                    if (triangle->indices[0].position > 0 &&
                        triangle->indices[1].position > 0 &&
                        triangle->indices[2].position > 0 &&
                        triangle->indices[0].position <= static_cast<int>(positions.size()) &&
                        triangle->indices[1].position <= static_cast<int>(positions.size()) &&
                        triangle->indices[2].position <= static_cast<int>(positions.size())) {
                        const std::array<double, 3> first = convertPosition(
                            positions[static_cast<std::size_t>(triangle->indices[0].position - 1)]);
                        const std::array<double, 3> second = convertPosition(
                            positions[static_cast<std::size_t>(triangle->indices[1].position - 1)]);
                        const std::array<double, 3> third = convertPosition(
                            positions[static_cast<std::size_t>(triangle->indices[2].position - 1)]);
                        const std::array<double, 3> edgeA = {second[0] - first[0],
                                                             second[1] - first[1],
                                                             second[2] - first[2]};
                        const std::array<double, 3> edgeB = {third[0] - first[0],
                                                             third[1] - first[1],
                                                             third[2] - first[2]};
                        fallbackNormal = normalizeVector(
                            {edgeA[1] * edgeB[2] - edgeA[2] * edgeB[1],
                             edgeA[2] * edgeB[0] - edgeA[0] * edgeB[2],
                             edgeA[0] * edgeB[1] - edgeA[1] * edgeB[0]});
                    }
                    for (const ObjIndex& index : triangle->indices) {
                        if (index.position <= 0 ||
                            index.position > static_cast<int>(positions.size())) {
                            continue;
                        }
                        const ObjPosition& position =
                            positions[static_cast<std::size_t>(index.position - 1)];
                        std::array<double, 3> normal = fallbackNormal;
                        if (index.normal > 0 &&
                            index.normal <= static_cast<int>(normals.size())) {
                            const ObjNormal& sourceNormal =
                                normals[static_cast<std::size_t>(index.normal - 1)];
                            normal = normalizeVector(
                                {-sourceNormal.z, sourceNormal.x, -sourceNormal.y});
                        }
                        const ObjTexCoord* texCoord = nullptr;
                        if (index.texCoord > 0 &&
                            index.texCoord <= static_cast<int>(texCoords.size())) {
                            texCoord = &texCoords[static_cast<std::size_t>(index.texCoord - 1)];
                        }
                        vertices.push_back(
                            {static_cast<float>(position.z), static_cast<float>(-position.x),
                             static_cast<float>(position.y),
                             texCoord ? static_cast<float>(texCoord->u) : 0.0f,
                             texCoord ? static_cast<float>(1.0 - texCoord->v) : 0.0f, 0.0f,
                             static_cast<float>(normal[0]), static_cast<float>(normal[1]),
                             static_cast<float>(normal[2])});
                    }
                }
            }
            if (vertices.empty())
                return;
            Batch batch;
            batch.vertices = std::move(vertices);
            batch.baseTexturePath = texturePath;
            batch.baseTextureName = textureName;
            batch.texturePath = texturePath;
            batch.textureRoot = part.objPath.parent_path();
            batch.textureName = textureName;
            batch.environmentTextureName = environmentTextureName;
            batch.environmentStrength = environmentStrength;
            batch.color = color;
            batch.hasNormals = true;
            batch.alphaMode = alphaMode;
            batch.noZwrite = noZwrite;
            batch.alphaScaleVariable = alphaScaleVariable;
            batch.textureChanges = textureChanges;
            batch.vertexCount = batch.vertices.size();
            {
                TraceScope uploadTrace("obj", "loadObj.uploadVbo");
                pglGenBuffers(1, &batch.buffer);
                pglBindBuffer(GL_ARRAY_BUFFER, batch.buffer);
                pglBufferData(GL_ARRAY_BUFFER,
                              static_cast<std::ptrdiff_t>(batch.vertices.size() * sizeof(Vertex)),
                              batch.vertices.data(), GL_STATIC_DRAW);
                pglBindBuffer(GL_ARRAY_BUFFER, 0);
            }
            destination->back().batches.push_back(std::move(batch));
        };

        std::unordered_map<std::string, std::vector<const ObjTriangle*>> groups;
        std::unordered_map<std::string, std::filesystem::path> groupTextures;
        std::unordered_map<std::string, std::string> groupTextureNames;
        std::unordered_map<std::string, std::string> groupEnvironmentNames;
        std::unordered_map<std::string, double> groupEnvironmentStrengths;
        std::unordered_map<std::string, std::array<double, 3>> groupColors;
        std::unordered_map<std::string, MaterialState> groupStates;
        std::vector<std::string> groupOrder;
        std::unordered_map<std::string, const MaterialState*> materialStatesByFilename;
        std::unordered_map<std::string, const MaterialState*> materialStatesByStem;
        std::unordered_map<std::string, std::vector<const MaterialState*>>
            materialStatesByStemOccurrence;
        materialStatesByFilename.reserve(part.materialStates.size());
        materialStatesByStem.reserve(part.materialStates.size());
        for (const auto& entry : part.materialStates) {
            materialStatesByFilename.emplace(
                lower(std::filesystem::path(entry.first).filename().string()), &entry.second);
            materialStatesByStem.emplace(lower(std::filesystem::path(entry.first).stem().string()),
                                         &entry.second);
        }
        for (const MaterialState& state : part.materialStatesInOrder) {
            const std::filesystem::path statePath =
                state.textureName.empty() ? state.texturePath
                                          : std::filesystem::path(state.textureName);
            std::vector<const MaterialState*>& states =
                materialStatesByStemOccurrence[lower(statePath.stem().string())];
            if (state.materialIndex >= 0) {
                if (states.size() <= static_cast<std::size_t>(state.materialIndex)) {
                    states.resize(static_cast<std::size_t>(state.materialIndex) + 1, nullptr);
                }
                states[static_cast<std::size_t>(state.materialIndex)] = &state;
            } else {
                states.push_back(&state);
            }
        }
        std::vector<std::pair<int, std::string>> indexedMaterials;
        indexedMaterials.reserve(materials.size());
        for (const auto& entry : materials) {
            if (entry.second.materialIndex >= 0) {
                const std::filesystem::path materialPath =
                    entry.second.textureName.empty() ? entry.second.texturePath
                                                     : std::filesystem::path(entry.second.textureName);
                indexedMaterials.push_back(
                    {entry.second.materialIndex, lower(materialPath.stem().string())});
            }
        }
        std::sort(indexedMaterials.begin(), indexedMaterials.end(),
                  [](const auto& first, const auto& second) { return first.first < second.first; });
        std::unordered_map<std::string, int> textureOccurrences;
        std::unordered_map<int, int> materialOccurrenceByIndex;
        for (const auto& indexedMaterial : indexedMaterials) {
            const int occurrence = textureOccurrences[indexedMaterial.second]++;
            materialOccurrenceByIndex[indexedMaterial.first] = occurrence;
        }
        const std::filesystem::path fallbackTexturePath = part.texturePath;
        const std::string fallbackTextureName = part.textureName;
        std::size_t renderedTriangleCount = 0;
        bool hasTransparentMaterial = false;
        {
            TraceScope materialTrace("obj", "loadObj.groupMaterials");
            for (const auto& triangle : triangles) {
                ++renderedTriangleCount;
                const auto material = materials.find(triangle.material);
                std::filesystem::path texturePath =
                    material != materials.end() && !material->second.texturePath.empty()
                        ? material->second.texturePath
                        : fallbackTexturePath;
                std::string textureName =
                    material != materials.end() && !material->second.textureName.empty()
                        ? material->second.textureName
                        : fallbackTextureName;
                std::array<double, 3> color =
                    material != materials.end() ? material->second.color : part.color;
                const std::string key =
                    triangle.material.empty() ? "__fallback__" : triangle.material;
                if (groups.find(key) == groups.end()) {
                    groupOrder.push_back(key);
                }
                MaterialState state;
                bool matchedMaterialState = false;
                if (material != materials.end() && !part.materialStatesInOrder.empty() &&
                    material->second.materialIndex >= 0) {
                    const auto occurrence =
                        materialOccurrenceByIndex.find(material->second.materialIndex);
                    const std::filesystem::path materialPath =
                        material->second.textureName.empty()
                            ? material->second.texturePath
                            : std::filesystem::path(material->second.textureName);
                    const auto states = materialStatesByStemOccurrence.find(
                        lower(materialPath.stem().string()));
                    if (occurrence != materialOccurrenceByIndex.end() &&
                        states != materialStatesByStemOccurrence.end() &&
                        occurrence->second >= 0 &&
                        static_cast<std::size_t>(occurrence->second) < states->second.size() &&
                        states->second[static_cast<std::size_t>(occurrence->second)] != nullptr) {
                        state = *states->second[static_cast<std::size_t>(occurrence->second)];
                    }
                    matchedMaterialState = true;
                }
                if (!textureName.empty() || !texturePath.empty()) {
                    const std::filesystem::path statePath =
                        textureName.empty() ? texturePath : std::filesystem::path(textureName);
                    const std::string stateKey = lower(statePath.filename().string());
                    if (!matchedMaterialState) {
                        const auto exactState = materialStatesByFilename.find(stateKey);
                        if (exactState != materialStatesByFilename.end()) {
                            state = *exactState->second;
                        } else {
                            const std::string stateStem = lower(statePath.stem().string());
                            const auto matchingState = materialStatesByStem.find(stateStem);
                            if (matchingState != materialStatesByStem.end()) {
                                state = *matchingState->second;
                            } else {
                                const auto matchingMaterial =
                                    materialStatesByStem.find(lower(triangle.material));
                                if (matchingMaterial != materialStatesByStem.end()) {
                                    state = *matchingMaterial->second;
                                }
                            }
                        }
                    }
                }
                if (!state.textureName.empty()) {
                    textureName = state.textureName;
                }
                if (state.alphaMode == 0) {
                    state.noZwrite = false;
                }
                groups[key].push_back(&triangle);
                groupTextures[key] = texturePath;
                groupTextureNames[key] = textureName;
                groupEnvironmentNames[key] = state.environmentTextureName;
                groupEnvironmentStrengths[key] = state.environmentStrength;
                groupColors[key] = color;
                groupStates[key] = state;
                hasTransparentMaterial =
                    hasTransparentMaterial || state.alphaMode != 0 || state.noZwrite;
            }
        }
        destination->push_back({{},
                                part.viewpoint,
                                part.renderType,
                                hasTransparentMaterial,
                                part.lodIndex,
                                part.visibleVariable,
                                part.visibleValue,
                                boundsCenter,
                                boundsSize,
                                boundsRadius,
                                renderedTriangleCount});
        for (const std::string& key : groupOrder) {
            const MaterialState& state = groupStates[key];
            makeBatch(groups[key], groupTextures[key], groupTextureNames[key], groupColors[key],
                      groupEnvironmentNames[key], groupEnvironmentStrengths[key], state.alphaMode,
                      state.noZwrite, state.alphaScaleVariable, state.textureChanges);
        }
    }

    int lodForDistance(double distance) const {
        if (lodThresholds.empty()) {
            return -1;
        }
        for (std::size_t index = 0; index < lodThresholds.size(); ++index) {
            const double boundary = 25.0 / std::max(lodThresholds[index], 0.001);
            if (distance <= boundary) {
                return static_cast<int>(index);
            }
        }
        return static_cast<int>(lodThresholds.size() - 1);
    }

    void rebuildDisplayOrder() {
        std::stable_sort(displayLists.begin(), displayLists.end(),
                         [](const DisplayPart& first, const DisplayPart& second) {
                             return first.transparent < second.transparent;
                         });
        opaqueDisplayCount = 0;
        while (opaqueDisplayCount < displayLists.size() &&
               !displayLists[opaqueDisplayCount].transparent) {
            ++opaqueDisplayCount;
        }
    }

    // Load visible OBJ geometry immediately; texture decoding remains asynchronous.
    void ensureLoaded(RenderViewContext context, double viewDistance) {
        TraceScope trace("obj", "ensureLoaded");
        const int selectedLod = lodForDistance(viewDistance);
        activeLod = selectedLod;
        bool loadedPart = false;
        const bool immediateLoad =
            !hasLoadedInitialView || loadingPolicy.modelMode == AssetLoadingMode::Eager;
        const auto loadStart = std::chrono::steady_clock::now();
        constexpr auto loadBudget = std::chrono::milliseconds(4);
        static const std::size_t maxObjWorkers = [] {
            if (const char* value = std::getenv("OPENBUS_OBJ_WORKERS")) {
                try {
                    return static_cast<std::size_t>(std::clamp(std::stoi(value), 1, 64));
                } catch (const std::exception&) {
                }
            }
            const unsigned int concurrency = std::thread::hardware_concurrency();
            if (concurrency == 0) {
                return static_cast<std::size_t>(8);
            }
            return static_cast<std::size_t>(std::clamp(static_cast<int>(concurrency), 4, 32));
        }();
        static const std::size_t maxObjLoadsPerFrame = [] {
            if (const char* value = std::getenv("OPENBUS_OBJ_LOADS_PER_FRAME")) {
                try {
                    return static_cast<std::size_t>(std::clamp(std::stoi(value), 1, 128));
                } catch (const std::exception&) {
                }
            }
            return static_cast<std::size_t>(8);
        }();
        const std::size_t maxLoadsThisFrame =
            immediateLoad ? pendingParts.size() : maxObjLoadsPerFrame;
        const auto budgetReached = [&] {
            return !immediateLoad && std::chrono::steady_clock::now() - loadStart >= loadBudget;
        };

        std::size_t activeObjWorkers = 0;
        for (const Part& part : pendingParts) {
            if (!part.objRequest) {
                continue;
            }
            if (part.objRequest->future.wait_for(std::chrono::milliseconds(0)) !=
                std::future_status::ready) {
                ++activeObjWorkers;
            }
        }

        for (Part& part : pendingParts) {
            const bool viewpointMatches =
                part.viewpoint == 0 || (part.viewpoint & viewpointMask(context)) != 0;
            const bool lodMatches = selectedLod < 0 || part.lodIndex == selectedLod;
            if (part.objRequest || (viewpointMatches && lodMatches)) {
                continue;
            }
            if (activeObjWorkers >= maxObjWorkers) {
                continue;
            }
            part.objRequest = std::make_shared<ObjRequest>();
            part.objRequest->future = parsedObjFuture(part.objPath);
            ++activeObjWorkers;
        }

        std::size_t loadedThisFrame = 0;
        auto part = pendingParts.begin();
        while (part != pendingParts.end() && loadedThisFrame < maxLoadsThisFrame) {
            if (budgetReached() && loadedThisFrame > 0) {
                break;
            }
            const bool viewpointMatches =
                part->viewpoint == 0 || (part->viewpoint & viewpointMask(context)) != 0;
            const bool shouldLoad =
                viewpointMatches && (selectedLod < 0 || part->lodIndex == selectedLod);
            if (!shouldLoad) {
                ++part;
                continue;
            }
            std::shared_ptr<ParsedObj> parsed;
            if (!part->objRequest) {
                parsed = parsedObjFuture(part->objPath).get();
            } else {
                parsed = part->objRequest->future.get();
            }
            loadObj(*part, parsed);
            part = pendingParts.erase(part);
            loadedPart = true;
            ++loadedThisFrame;
        }

        auto backgroundPart = pendingParts.begin();
        while (backgroundPart != pendingParts.end() && loadedThisFrame < maxLoadsThisFrame) {
            if (budgetReached() && loadedThisFrame > 0) {
                break;
            }
            const bool viewpointMatches = backgroundPart->viewpoint == 0 ||
                                          (backgroundPart->viewpoint & viewpointMask(context)) != 0;
            const bool lodMatches = selectedLod < 0 || backgroundPart->lodIndex == selectedLod;
            if ((viewpointMatches && lodMatches) || !backgroundPart->objRequest ||
                backgroundPart->objRequest->future.wait_for(std::chrono::milliseconds(0)) !=
                    std::future_status::ready) {
                ++backgroundPart;
                continue;
            }
            const std::shared_ptr<ParsedObj> parsed = backgroundPart->objRequest->future.get();
            loadObj(*backgroundPart, parsed);
            backgroundPart = pendingParts.erase(backgroundPart);
            loadedPart = true;
            ++loadedThisFrame;
        }
        hasLoadedInitialView = true;
        if (loadedPart) {
            rebuildDisplayOrder();
        }
        loaded = !displayLists.empty() || !pendingParts.empty();
        if (!loggedAllObjectsLoaded && pendingParts.empty() && loaded) {
            std::size_t wheelPartCount = 0;
            for (const WheelModel& wheel : wheelModels) {
                wheelPartCount += wheel.parts.size();
            }
            gameLog.Log("All objects loaded. bodyParts=" + std::to_string(displayLists.size()) +
                        " wheelParts=" + std::to_string(wheelPartCount));
            loggedAllObjectsLoaded = true;
        }
    }

    void load(const std::filesystem::path& configPath, const std::filesystem::path& modelRoot) {
        auto result = openbus::rendering::loadBusModel(configPath, modelRoot, variables);
        lodThresholds = std::move(result.lodThresholds);
        for (const ConfigurationDiagnostic& diagnostic : result.diagnostics.entries) {
            const std::string severity =
                diagnostic.severity == ConfigurationDiagnostic::Severity::Error ? "error"
                                                                                : "warning";
            gameLog.Log("CFG " + severity + " line " + std::to_string(diagnostic.line) + " [" +
                        diagnostic.keyword + "]: " + diagnostic.message);
        }

        std::vector<Part> parts;
        parts.reserve(result.parts.size());
        for (auto& source : result.parts) {
            Part part;
            static_cast<openbus::rendering::BusModelPart&>(part) = std::move(source);
            parts.push_back(std::move(part));
        }
        pendingParts = std::move(parts);
        loaded = !pendingParts.empty();
    }
};

void BusModel::loadTextureRequest(const std::shared_ptr<TextureRequest>& request) {
    TraceScope trace("texture", "loadTextureRequest");
#ifdef _WIN32
    const HRESULT comResult = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
#endif
    std::filesystem::path resolved;
    const std::string logicalName = request->name.empty() ? request->path.string() : request->name;
    if (!logicalName.empty() && !std::filesystem::path(logicalName).is_absolute()) {
        resolved = findTexture(request->root, logicalName);
    }
    if (resolved.empty() && !request->path.empty()) {
        const std::filesystem::path candidate = request->path;
        if (candidate.is_absolute() && std::filesystem::exists(candidate)) {
            resolved = candidate;
        } else if (request->name.empty() && std::filesystem::exists(candidate)) {
            resolved = candidate;
        }
    }
    const std::string resolvedKey =
        resolved.empty()
            ? std::string()
            : lower(std::filesystem::absolute(resolved).lexically_normal().generic_string());
    if (!resolvedKey.empty()) {
        std::shared_ptr<DecodedTexture> cachedTexture;
        {
            std::lock_guard<std::mutex> lock(decodedTextureMutex);
            const auto cached = decodedTextureCache.find(resolvedKey);
            if (cached != decodedTextureCache.end()) {
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
    Image image;
    bool textureLoaded = false;
    std::shared_ptr<gli::texture> compressedTexture;
    std::shared_ptr<CompressedDds> compressedDds;
    if (!resolved.empty() && lower(resolved.extension().string()) == ".dds" &&
        openbus::rendering::TextureLoader::isSafeCompressedDds(resolved)) {
        if (openbus::rendering::TextureLoader::isDxt5Dds(resolved) &&
            !openbus::rendering::TextureLoader::isDdsTextureArray(resolved)) {
            compressedDds = std::make_shared<CompressedDds>();
            textureLoaded =
                openbus::rendering::TextureLoader::readDxt5CompressedDds(resolved, *compressedDds);
            if (!textureLoaded) {
                compressedDds.reset();
            }
        } else {
            try {
                gli::texture loadedTexture = gli::load(resolved.string());
                if (!loadedTexture.empty() && gli::is_compressed(loadedTexture.format())) {
                    compressedTexture = std::make_shared<gli::texture>(std::move(loadedTexture));
                    textureLoaded = true;
                }
            } catch (const std::exception& error) {
                gameLog.Log("GLI failed to load compressed texture " + resolved.generic_string() +
                            ": " + error.what());
            }
        }
    }
    if (!textureLoaded) {
        textureLoaded =
            !resolved.empty() && openbus::rendering::TextureLoader::readImage(resolved, image);
    }
#ifdef _WIN32
    if (SUCCEEDED(comResult)) {
        CoUninitialize();
    }
#endif
    std::lock_guard<std::mutex> lock(request->mutex);
    if (textureLoaded) {
        request->resolvedPath = std::move(resolved);
        request->image = std::make_shared<Image>(std::move(image));
        request->compressedTexture = std::move(compressedTexture);
        request->compressedDds = std::move(compressedDds);
        if (!resolvedKey.empty()) {
            auto decoded = std::make_shared<DecodedTexture>();
            decoded->image = request->image;
            decoded->compressedTexture = request->compressedTexture;
            decoded->compressedDds = request->compressedDds;
            std::lock_guard<std::mutex> cacheLock(decodedTextureMutex);
            decodedTextureCache.emplace(resolvedKey, std::move(decoded));
        }
    }
    request->complete = true;
}

void Renderer::scrollCallback(GLFWwindow* window, double, double yOffset) {
    auto* renderer = static_cast<Renderer*>(glfwGetWindowUserPointer(window));
    if (renderer == nullptr) {
        return;
    }
    renderer->cameraDistance_ = std::clamp(renderer->cameraDistance_ - yOffset * 2.0, 6.0, 80.0);
}

Renderer::Renderer(int width, int height, const char* title)
    : window_(nullptr) {
    TraceScope trace("startup", "Renderer::Renderer");
    if (!glfwInit()) {
        gameLog.Log("Failed to initialize GLFW");
        throw std::runtime_error("Failed to initialize GLFW");
    }
    window_ = glfwCreateWindow(width, height, title, nullptr, nullptr);
    if (!window_) {
        gameLog.Log("Failed to create OpenGL window");
        glfwTerminate();
        throw std::runtime_error("Failed to create OpenGL window");
    }
    glfwMakeContextCurrent(window_);
    glfwSetWindowUserPointer(window_, this);
    glfwSetScrollCallback(window_, &Renderer::scrollCallback);
    const char* vsyncSetting = std::getenv("OPENBUS_VSYNC");
    const bool vsyncEnabled = vsyncSetting == nullptr || (std::string(vsyncSetting) != "0" &&
                                                          std::string(vsyncSetting) != "off" &&
                                                          std::string(vsyncSetting) != "false");
    glfwSwapInterval(vsyncEnabled ? 1 : 0);
    gameLog.Log(std::string("VSync ") + (vsyncEnabled ? "enabled" : "disabled"));
    if (!loadBufferFunctions()) {
        glfwDestroyWindow(window_);
        window_ = nullptr;
        glfwTerminate();
        throw std::runtime_error("OpenGL VBO functions are unavailable");
    }
    glEnable(GL_DEPTH_TEST);
    glClearColor(0.45f, 0.65f, 0.88f, 1.0f);
    gameLog.Log("Renderer initialized");
}

Renderer::~Renderer() {
    gameLog.Log("Renderer shutting down");
    playerBusModel_ = nullptr;
    busModels_.clear();
    if (window_) {
        glfwDestroyWindow(window_);
    }
    glfwTerminate();
}

BusModel* Renderer::AddBusModel(BusVehicle vehicle, ModelLoadingPolicy loadingPolicy) {
    if (vehicleCameras_.empty()) {
        const VehicleConfig vehicleConfiguration = loadBusConfig(busConfigurationPathFor(vehicle));
        for (const VehicleCamera& camera : vehicleConfiguration.cameras) {
            if (camera.kind == VehicleCameraKind::Driver ||
                camera.kind == VehicleCameraKind::Passenger) {
                vehicleCameras_.push_back(camera);
            }
        }
        if (!vehicleCameras_.empty()) {
            int driverIndex = 0;
            int selectedCamera = -1;
            for (std::size_t index = 0; index < vehicleCameras_.size(); ++index) {
                if (vehicleCameras_[index].kind != VehicleCameraKind::Driver) {
                    continue;
                }
                if (driverIndex == vehicleConfiguration.standardDriverCamera) {
                    selectedCamera = static_cast<int>(index);
                    break;
                }
                ++driverIndex;
            }
            cameraView_ = selectedCamera >= 0 ? selectedCamera + 1 : 1;
        }
    }
    auto model = std::make_unique<BusModel>(vehicle, loadingPolicy);
    BusModel* result = model.get();
    busModels_.push_back(std::move(model));
    return result;
}

BusModel* Renderer::AddBusModel(BusVehicle vehicle, const std::array<double, 3>& spawnPosition,
                                ModelLoadingPolicy loadingPolicy) {
    static_cast<void>(spawnPosition);
    return AddBusModel(vehicle, loadingPolicy);
}

void Renderer::SetPlayerBusModel(BusModel* model) {
    playerBusModel_ = model;
}

bool Renderer::isExteriorView() const {
    if (cameraView_ == 0) {
        return true;
    }
    const VehicleCamera* camera = currentVehicleCamera();
    return camera != nullptr && camera->kind == VehicleCameraKind::Passenger &&
           camera->orbitDistance > 1.0;
}

const VehicleCamera* Renderer::currentVehicleCamera() const {
    if (cameraView_ <= 0 || static_cast<std::size_t>(cameraView_) > vehicleCameras_.size()) {
        return nullptr;
    }
    return &vehicleCameras_[static_cast<std::size_t>(cameraView_ - 1)];
}

void Renderer::selectVehicleCamera(int direction) {
    if (vehicleCameras_.empty() || direction == 0) {
        return;
    }
    const int cameraCount = static_cast<int>(vehicleCameras_.size());
    int next = cameraView_ == 0 ? (direction > 0 ? 0 : cameraCount - 1) : cameraView_ - 1;
    if (cameraView_ != 0) {
        next = (next + direction + cameraCount) % cameraCount;
    }
    cameraView_ = next + 1;
    viewLookYaw_ = 0.0;
    viewLookPitch_ = 0.0;
    gameLog.Log("Changed vehicle camera to " + std::to_string(next));
}

double Renderer::currentFieldOfView() const {
    const VehicleCamera* camera = currentVehicleCamera();
    return camera != nullptr && camera->fieldOfView > 0.0 ? camera->fieldOfView : 60.0;
}

bool Renderer::shouldClose() const {
    return glfwWindowShouldClose(window_) != 0;
}

void Renderer::requestClose() {
    if (window_ != nullptr) {
        glfwSetWindowShouldClose(window_, GLFW_TRUE);
    }
}

void Renderer::beginFrame() {
    TraceScope trace("frame", "Renderer::beginFrame");
    glfwPollEvents();
    const double currentTime = glfwGetTime();
    const double timegap =
        hasPreviousVariableTime_ ? std::max(0.0, currentTime - previousVariableTime_) : 0.0;
    previousVariableTime_ = currentTime;
    hasPreviousVariableTime_ = true;
    const std::array<int, 4> keys = {GLFW_KEY_W, GLFW_KEY_A, GLFW_KEY_S, GLFW_KEY_D};
    const std::array<const char*, 4> keyNames = {"W", "A", "S", "D"};
    for (std::size_t index = 0; index < keys.size(); ++index) {
        const bool pressed = glfwGetKey(window_, keys[index]) == GLFW_PRESS;
        if (pressed != previousKeyStates_[index]) {
            keyEvents_.push_back({keyNames[index], pressed, glfwGetTime()});
            gameLog.Log(std::string("Key ") + keyNames[index] +
                        (pressed ? " pressed" : " released"));
            previousKeyStates_[index] = pressed;
        }
    }
    for (std::size_t index = 0; index < previousViewKeyStates_.size(); ++index) {
        const int key = GLFW_KEY_0 + static_cast<int>(index);
        const bool pressed = glfwGetKey(window_, key) == GLFW_PRESS;
        if (pressed && !previousViewKeyStates_[index]) {
            if (index == 0 || index <= vehicleCameras_.size()) {
                cameraView_ = static_cast<int>(index);
                viewLookYaw_ = 0.0;
                viewLookPitch_ = 0.0;
                gameLog.Log("Changed camera view to " + std::to_string(cameraView_));
            }
        }
        previousViewKeyStates_[index] = pressed;
    }
    const std::array<int, 2> cameraNavigationKeys = {GLFW_KEY_LEFT, GLFW_KEY_RIGHT};
    for (std::size_t index = 0; index < cameraNavigationKeys.size(); ++index) {
        const bool pressed = glfwGetKey(window_, cameraNavigationKeys[index]) == GLFW_PRESS;
        if (pressed && !previousCameraNavigationStates_[index]) {
            selectVehicleCamera(index == 0 ? -1 : 1);
        }
        previousCameraNavigationStates_[index] = pressed;
    }
    const bool captureKeyPressed = glfwGetKey(window_, GLFW_KEY_F12) == GLFW_PRESS;
    if (captureKeyPressed && !previousCaptureKeyState_) {
        captureRequested_ = true;
    }
    previousCaptureKeyState_ = captureKeyPressed;
    int width = 1;
    int height = 1;
    glfwGetFramebufferSize(window_, &width, &height);
    double cursorX = 0.0;
    double cursorY = 0.0;
    glfwGetCursorPos(window_, &cursorX, &cursorY);
    if (playerBusModel_) {
        playerBusModel_->updateFrameVariables(timegap, currentTime, cursorX, cursorY);
    }
    const bool middleMouse = glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
    if (middleMouse && !draggingCamera_) {
        previousCursorX_ = cursorX;
        previousCursorY_ = cursorY;
    } else if (middleMouse) {
        const double cursorDeltaX = cursorX - previousCursorX_;
        const double cursorDeltaY = cursorY - previousCursorY_;
        if (isExteriorView()) {
            cameraYaw_ -= cursorDeltaX * 0.005;
            cameraPitch_ -= cursorDeltaY * 0.005;
            cameraPitch_ = std::clamp(cameraPitch_, -1.35, 1.35);
        } else {
            viewLookYaw_ -= cursorDeltaX * 0.005;
            viewLookPitch_ -= cursorDeltaY * 0.005;
            viewLookPitch_ = std::clamp(viewLookPitch_, -1.35, 1.35);
        }
    }
    draggingCamera_ = middleMouse;
    previousCursorX_ = cursorX;
    previousCursorY_ = cursorY;
    glViewport(0, 0, width, height);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    setPerspective(static_cast<double>(width), static_cast<double>(height), currentFieldOfView());
}

void Renderer::draw(const BusSimulation& simulation) {
    TraceScope trace("frame", "Renderer::draw");
    const BodyPose chassis = simulation.chassisPose();
    const ChassisCollisionBox collision = simulation.chassisCollisionBox();
    {
        TraceScope phase("render", "Renderer::draw.camera");
        if (cameraView_ == 0) {
            const std::array<double, 3> target = transformLocalPoint(
                chassis, {collision.offsetX, collision.offsetY, collision.offsetZ});
            const double targetX = target[0];
            const double targetY = target[1];
            const double targetZ = target[2];
            const double horizontalDistance = cameraDistance_ * std::cos(cameraPitch_);
            const double orbitYaw = simulation.yaw() + cameraYaw_;
            const double eyeX = targetX + horizontalDistance * std::cos(orbitYaw);
            const double eyeY = targetY + horizontalDistance * std::sin(orbitYaw);
            const double eyeZ = targetZ + cameraDistance_ * std::sin(cameraPitch_);
            lookAt(eyeX, eyeY, eyeZ, targetX, targetY, targetZ);
        } else if (const VehicleCamera* camera = currentVehicleCamera(); camera != nullptr) {
            constexpr double DEGREES_TO_RADIANS = 3.141592653589793 / 180.0;
            const double pan = -camera->pan * DEGREES_TO_RADIANS + viewLookYaw_;
            const double tilt = camera->tilt * DEGREES_TO_RADIANS + viewLookPitch_;
            const double modelOffsetZ =
                playerBusModel_ != nullptr ? playerBusModel_->modelOffsetZ : 0.0;
            const std::array<double, 3> centerLocal = {camera->position[1], -camera->position[0],
                                                       camera->position[2] + modelOffsetZ};
            const std::array<double, 3> direction = {
                std::cos(tilt) * std::cos(pan), std::cos(tilt) * std::sin(pan), std::sin(tilt)};
            const std::array<double, 3> eyeLocal = {
                centerLocal[0] - direction[0] * camera->orbitDistance,
                centerLocal[1] - direction[1] * camera->orbitDistance,
                centerLocal[2] - direction[2] * camera->orbitDistance};
            const std::array<double, 3> targetLocal = {eyeLocal[0] + direction[0] * 3.0,
                                                       eyeLocal[1] + direction[1] * 3.0,
                                                       eyeLocal[2] + direction[2] * 3.0};
            const std::array<double, 3> eye = transformLocalPoint(chassis, eyeLocal);
            const std::array<double, 3> target = transformLocalPoint(chassis, targetLocal);
            lookAt(eye[0], eye[1], eye[2], target[0], target[1], target[2]);
        } else {
            std::array<double, 3> eyeLocal = {5.65, 0.70, 0.70};
            std::array<double, 3> targetLocal = {8.65, 0.70, 0.72};
            switch (cameraView_) {
            case 2:
                eyeLocal = {4.25, -0.55, 0.78};
                targetLocal = {7.25, -0.55, 0.80};
                break;
            case 3:
                eyeLocal = {-2.75, 0.0, 0.72};
                targetLocal = {0.25, 0.0, 0.74};
                break;
            case 4:
                eyeLocal = {5.35, 1.05, 0.86};
                targetLocal = {5.35, 4.0, 0.82};
                break;
            case 5:
                eyeLocal = {5.35, -1.05, 0.86};
                targetLocal = {5.35, -4.0, 0.82};
                break;
            case 6:
                eyeLocal = {6.85, 0.0, 0.80};
                targetLocal = {3.85, 0.0, 0.80};
                break;
            case 7:
                eyeLocal = {-6.15, 0.0, 0.80};
                targetLocal = {-3.15, 0.0, 0.80};
                break;
            case 8:
                eyeLocal = {0.0, 1.05, 1.30};
                targetLocal = {0.0, 4.0, 1.30};
                break;
            case 9:
                eyeLocal = {0.0, -1.05, 1.30};
                targetLocal = {0.0, -4.0, 1.30};
                break;
            default:
                break;
            }
            const std::array<double, 3> eye = transformLocalPoint(chassis, eyeLocal);
            const double directionX = targetLocal[0] - eyeLocal[0];
            const double directionY = targetLocal[1] - eyeLocal[1];
            const double directionZ = targetLocal[2] - eyeLocal[2];
            const double horizontalLength = std::hypot(directionX, directionY);
            const double distance = std::sqrt(directionX * directionX + directionY * directionY +
                                              directionZ * directionZ);
            const double baseYaw = std::atan2(directionY, directionX);
            const double basePitch = std::atan2(directionZ, horizontalLength);
            const double lookYaw = baseYaw + viewLookYaw_;
            const double lookPitch = basePitch + viewLookPitch_;
            const std::array<double, 3> target = transformLocalPoint(
                chassis, {eyeLocal[0] + distance * std::cos(lookPitch) * std::cos(lookYaw),
                          eyeLocal[1] + distance * std::cos(lookPitch) * std::sin(lookYaw),
                          eyeLocal[2] + distance * std::sin(lookPitch)});
            lookAt(eye[0], eye[1], eye[2], target[0], target[1], target[2]);
        }
    }

    {
        TraceScope phase("render", "Renderer::draw.ground");
        glPushMatrix();
        if (!captureMode_) {
            drawGround(simulation.roadBumps());
        }
        glPopMatrix();
    }

    const RenderViewContext context =
        isExteriorView() ? RenderViewContext::PlayerExterior : RenderViewContext::PlayerInterior;

    {
        TraceScope phase("render", "Renderer::draw.model");
        if (playerBusModel_ && playerBusModel_->loaded && !playerBusModel_->displayLists.empty()) {
            glPushMatrix();
            applyPose(chassis);
            playerBusModel_->draw(context);
            glPopMatrix();
        } else {
            glPushMatrix();
            applyPose(chassis);
            glTranslated(collision.offsetX, collision.offsetY, collision.offsetZ);
            drawBox(collision.length, collision.width, collision.height, 0.85, 0.70, 0.08);
            glPopMatrix();
        }
    }

    if (playerBusModel_ && glfwGetTime() - lastStatsTitleTime_ > 0.25) {
        std::ostringstream title;
        title << "OpenBus - " << playerBusModel_->renderedTriangles() << " triangles";
        glfwSetWindowTitle(window_, title.str().c_str());
        lastStatsTitleTime_ = glfwGetTime();
    }

    {
        TraceScope phase("render", "Renderer::draw.overlays");
        // Render center of gravity marker
        const std::array<double, 3> centerOfGravity = simulation.centerOfGravity();
        glPushMatrix();
        glTranslated(centerOfGravity[0], centerOfGravity[1], centerOfGravity[2]);
        drawCenterOfGravityMarker(0.35);
        glPopMatrix();

        // Render axle lines
        glPushMatrix();
        glColor3d(0.20, 0.20, 0.20);
        for (std::size_t axleIndex = 0; axleIndex < simulation.axleCount(); ++axleIndex) {
            const BodyPose leftWheel = simulation.wheelPose(axleIndex * 2);
            const BodyPose rightWheel = simulation.wheelPose(axleIndex * 2 + 1);
            glBegin(GL_LINES);
            glVertex3dv(rightWheel.position.data());
            glVertex3dv(leftWheel.position.data());
            glEnd();
        }
        glPopMatrix();
    }

    {
        TraceScope phase("render", "Renderer::draw.wheels");
        if (playerBusModel_ && playerBusModel_->hasConfiguredWheels(simulation.wheelCount())) {
            playerBusModel_->drawConfiguredWheels(simulation, chassis, isExteriorView());
        } else {
            for (std::size_t index = 0; index < simulation.wheelCount(); ++index) {
                const BodyPose wheel = simulation.wheelPose(index);
                glPushMatrix();
                applyPose(wheel);
                drawWheel(simulation.wheelRadius(index), simulation.wheelHalfWidth());
                glPopMatrix();
            }
        }
    }
}

void Renderer::endFrame() {
    TraceScope trace("frame", "Renderer::endFrame");
    glfwSwapBuffers(window_);
}

void Renderer::captureViews(const BusSimulation& simulation,
                            const std::filesystem::path& directory) {
    std::filesystem::create_directories(directory);
    int width = 1;
    int height = 1;
    glfwGetFramebufferSize(window_, &width, &height);

    const int previousCameraView = cameraView_;
    const double previousCameraYaw = cameraYaw_;
    const double previousCameraPitch = cameraPitch_;
    const double previousCameraDistance = cameraDistance_;
    const double previousViewLookYaw = viewLookYaw_;
    const double previousViewLookPitch = viewLookPitch_;
    captureMode_ = true;
    cameraView_ = 0;
    cameraDistance_ = 24.0;
    viewLookYaw_ = 0.0;
    viewLookPitch_ = 0.0;

    struct CaptureView {
        const char* name;
        double yaw;
        double pitch;
        double distance;
    };
    const std::array<CaptureView, 4> views = {{{"three-quarter", 0.55, 0.08, 11.0},
                                               {"front", 0.0, 0.08, 10.5},
                                               {"left", 1.5707963267948966, 0.08, 10.5},
                                               {"right", -1.5707963267948966, 0.08, 10.5}}};
    for (const CaptureView& view : views) {
        cameraYaw_ = view.yaw;
        cameraPitch_ = view.pitch;
        cameraDistance_ = view.distance;
        glViewport(0, 0, width, height);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        setPerspective(static_cast<double>(width), static_cast<double>(height), 60.0);
        draw(simulation);
        glFinish();
        const std::filesystem::path path = directory / (std::string(view.name) + ".bmp");
        if (!openbus::rendering::saveFramebufferBmp(path, width, height)) {
            gameLog.Log("Failed to save screenshot: " + path.string());
        } else {
            gameLog.Log("Saved screenshot: " + path.string());
        }
    }

    cameraView_ = previousCameraView;
    cameraYaw_ = previousCameraYaw;
    cameraPitch_ = previousCameraPitch;
    cameraDistance_ = previousCameraDistance;
    viewLookYaw_ = previousViewLookYaw;
    viewLookPitch_ = previousViewLookPitch;
    captureMode_ = false;
}

bool Renderer::consumeCaptureRequest() {
    const bool requested = captureRequested_;
    captureRequested_ = false;
    return requested;
}

bool Renderer::isCaptureReady() const {
    return playerBusModel_ && playerBusModel_->isCaptureReady();
}

double Renderer::throttle() const {
    return glfwGetKey(window_, GLFW_KEY_W) == GLFW_PRESS ? 1.0 : 0.0;
}

double Renderer::steering() const {
    const bool left = glfwGetKey(window_, GLFW_KEY_A) == GLFW_PRESS;
    const bool right = glfwGetKey(window_, GLFW_KEY_D) == GLFW_PRESS;
    return static_cast<double>(right) - static_cast<double>(left);
}

double Renderer::brake() const {
    return glfwGetKey(window_, GLFW_KEY_S) == GLFW_PRESS ? 1.0 : 0.0;
}

std::vector<KeyEvent> Renderer::consumeKeyEvents() {
    std::vector<KeyEvent> events;
    events.swap(keyEvents_);
    return events;
}
