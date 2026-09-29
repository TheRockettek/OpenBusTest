#include "Renderer.h"

#include "AssetRequestManager.h"
#include "BusConfigLoader.h"
#include "BusModelLoader.h"
#include "BusSimulation.h"
#include "CameraMath.h"
#include "CoreRenderer.h"
#include "Logger.h"
#include "ObjLoader.h"
#include "OpenGLFunctions.h"
#include "PerfTrace.h"
#include "RenderPrimitives.h"
#include "RoadFeatures.h"
#include "ScriptRuntime.h"
#include "ScreenshotWriter.h"
#include "TextureAssetLoader.h"
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

#ifndef GL_CLAMP_TO_EDGE
#define GL_CLAMP_TO_EDGE 0x812F
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

using Matrix4 = openbus::rendering::Matrix4;

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
constexpr double ENVIRONMENT_MAP_OPACITY = 0.1;
constexpr int MAX_SCRIPT_CATCH_UP_TICKS = 8;

Matrix4 identityMatrix() {
    return {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
}

Matrix4 multiplyMatrix4(const Matrix4& left, const Matrix4& right) {
    Matrix4 result = {};
    for (int column = 0; column < 4; ++column) {
        for (int row = 0; row < 4; ++row) {
            for (int inner = 0; inner < 4; ++inner) {
                result[row + column * 4] += left[row + inner * 4] * right[inner + column * 4];
            }
        }
    }
    return result;
}

Matrix4 translationMatrix(const std::array<double, 3>& value) {
    Matrix4 result = identityMatrix();
    result[12] = value[0];
    result[13] = value[1];
    result[14] = value[2];
    return result;
}

Matrix4 rotationMatrix(double angleDegrees, double x, double y, double z) {
    const double length = std::sqrt(x * x + y * y + z * z);
    if (length <= 1.0e-12) {
        return identityMatrix();
    }
    x /= length;
    y /= length;
    z /= length;
    const double angle = angleDegrees * 3.141592653589793 / 180.0;
    const double cosine = std::cos(angle);
    const double sine = std::sin(angle);
    const double inverseCosine = 1.0 - cosine;
    return {cosine + x * x * inverseCosine,
            y * x * inverseCosine + z * sine,
            z * x * inverseCosine - y * sine,
            0.0,
            x * y * inverseCosine - z * sine,
            cosine + y * y * inverseCosine,
            z * y * inverseCosine + x * sine,
            0.0,
            x * z * inverseCosine + y * sine,
            y * z * inverseCosine - x * sine,
            cosine + z * z * inverseCosine,
            0.0,
            0.0,
            0.0,
            0.0,
            1.0};
}

bool invertAffineMatrix(const Matrix4& matrix, Matrix4& inverse) {
    const double a = matrix[0];
    const double b = matrix[4];
    const double c = matrix[8];
    const double d = matrix[1];
    const double e = matrix[5];
    const double f = matrix[9];
    const double g = matrix[2];
    const double h = matrix[6];
    const double i = matrix[10];
    const double determinant = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
    if (std::abs(determinant) <= 1.0e-12) {
        inverse = identityMatrix();
        return false;
    }
    const double scale = 1.0 / determinant;
    inverse = identityMatrix();
    inverse[0] = (e * i - f * h) * scale;
    inverse[4] = (c * h - b * i) * scale;
    inverse[8] = (b * f - c * e) * scale;
    inverse[1] = (f * g - d * i) * scale;
    inverse[5] = (a * i - c * g) * scale;
    inverse[9] = (c * d - a * f) * scale;
    inverse[2] = (d * h - e * g) * scale;
    inverse[6] = (b * g - a * h) * scale;
    inverse[10] = (a * e - b * d) * scale;
    inverse[12] = -(inverse[0] * matrix[12] + inverse[4] * matrix[13] + inverse[8] * matrix[14]);
    inverse[13] = -(inverse[1] * matrix[12] + inverse[5] * matrix[13] + inverse[9] * matrix[14]);
    inverse[14] = -(inverse[2] * matrix[12] + inverse[6] * matrix[13] + inverse[10] * matrix[14]);
    return true;
}

std::array<double, 3> applyMeshAxis(const std::array<double, 3>& renderAxis,
                                    const std::array<double, 9>& meshRotation) {
    const std::array<double, 3> sourceAxis = {-renderAxis[1], renderAxis[0], renderAxis[2]};
    const std::array<double, 3> rotatedSourceAxis = {
        meshRotation[0] * sourceAxis[0] + meshRotation[3] * sourceAxis[1] +
            meshRotation[6] * sourceAxis[2],
        meshRotation[1] * sourceAxis[0] + meshRotation[4] * sourceAxis[1] +
            meshRotation[7] * sourceAxis[2],
        meshRotation[2] * sourceAxis[0] + meshRotation[5] * sourceAxis[1] +
            meshRotation[8] * sourceAxis[2]};
    return {rotatedSourceAxis[1], -rotatedSourceAxis[0], rotatedSourceAxis[2]};
}

Matrix4 makeMeshTransform(const std::array<double, 9>& meshRotation,
                          const std::array<double, 3>& origin) {
    Matrix4 result = identityMatrix();
    const std::array<std::array<double, 3>, 3> axes = {std::array<double, 3>{1.0, 0.0, 0.0},
                                                       std::array<double, 3>{0.0, 1.0, 0.0},
                                                       std::array<double, 3>{0.0, 0.0, 1.0}};
    for (int column = 0; column < 3; ++column) {
        const std::array<double, 3> transformed = applyMeshAxis(axes[column], meshRotation);
        result[column * 4] = transformed[0];
        result[column * 4 + 1] = transformed[1];
        result[column * 4 + 2] = transformed[2];
    }
    result[12] = origin[0];
    result[13] = origin[1];
    result[14] = origin[2];
    return result;
}

} // namespace

using openbus::rendering::applyPose;
using AssetRequestManager = openbus::rendering::AssetRequestManager;
using openbus::rendering::drawBox;
using openbus::rendering::drawCenterOfGravityMarker;
using openbus::rendering::drawEnvironmentBatch;
using openbus::rendering::drawGround;
using openbus::rendering::drawModelBatch;
using openbus::rendering::drawWheel;
using openbus::rendering::lookAt;
using openbus::rendering::multiplyMatrix;
using openbus::rendering::parseEnabledFlag;
using openbus::rendering::popMatrix;
using openbus::rendering::pushMatrix;
using openbus::rendering::rotate;
using openbus::rendering::scale;
using openbus::rendering::setPerspective;
using openbus::rendering::TraceScope;
using openbus::rendering::transformLocalPoint;
using openbus::rendering::translate;
using ObjIndex = openbus::rendering::ObjIndex;
using ObjNormal = openbus::rendering::ObjNormal;
using ObjPosition = openbus::rendering::ObjPosition;
using ObjTexCoord = openbus::rendering::ObjTexCoord;
using ObjTriangle = openbus::rendering::ObjTriangle;
using ParsedObj = openbus::rendering::ParsedObj;
using Image = openbus::rendering::Image;
using CompressedDds = openbus::rendering::CompressedDds;

struct Vehicle {
    ModelLoadingPolicy loadingPolicy;
    using TextureRequest = AssetRequestManager::TextureRequest;
    using TextureCacheEntry = AssetRequestManager::TextureCacheEntry;

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

    struct AuxiliaryTexture {
        std::string name;
        GLuint texture = 0;
        bool loadAttempted = false;
        std::shared_ptr<TextureCacheEntry> cacheEntry;
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
        AuxiliaryTexture lightmap;
        AuxiliaryTexture nightmap;
        AuxiliaryTexture transmap;
        std::string lightmapStrengthVariable;
        std::string freeTextureVariable;
        std::string texcoordTransXVariable;
        std::string texcoordTransYVariable;
        GLuint freeTexture = 0;
        int freeTextureIndex = -1;
        int freeTextureWidth = 0;
        int freeTextureHeight = 0;
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
        struct AnimationState {
            double currentAmount = 0.0;
            double targetAmount = 0.0;
            bool initialized = false;
        };

        // Geometry metadata is immutable after loading; visibility is still
        // evaluated from live variables during each draw.
        std::vector<Batch> batches;
        int viewpoint;
        int renderType = 2;
        bool transparent;
        int lodIndex;
        std::string visibleVariable;
        int visibleValue = 0;
        std::string meshIdentifier;
        std::string animationParent;
        bool backFaceCulling = false;
        std::array<double, 3> center;
        std::array<double, 3> size;
        double radius;
        std::size_t triangleCount;
        std::vector<ModelAnimation> animations;
        std::vector<AnimationState> animationStates;
    };

    struct WheelModel {
        WheelAnimation animation;
        std::vector<DisplayPart> parts;
    };

    std::vector<DisplayPart> displayLists;
    std::vector<WheelModel> wheelModels;
    openbus::scripting::Vehicle variables;
    std::unique_ptr<ScriptRuntime> scripts;
    std::vector<Part> pendingParts;
    AssetRequestManager* assets;
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
    double animationTimeStep = 0.0;
    std::chrono::steady_clock::time_point textureUploadStart;

    void updateMaterialChange(Batch& batch) {
        TraceScope phase("texture", "updateMaterialChange");
        static const bool verboseMaterialChangeLogs =
            parseEnabledFlag(std::getenv("OPENBUS_VERBOSE_MATERIAL_CHANGES"));

        // Select the last active material change, then reset only the texture
        // request state so the new texture is resolved and uploaded.
        const MaterialState::TextureChange* selected = nullptr;
        for (const MaterialState::TextureChange& change : batch.textureChanges) {
            const double activationValue = variables.get(change.activationVariable);
            if (activationValue != 0.0) {
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
        const bool sameRequestedTexture =
            textureName == batch.textureName &&
            (texturePath == batch.texturePath || batch.textureLoadAttempted);
        if (sameRequestedTexture && layer == batch.textureLayer) {
            return;
        }
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

    void updateAnimationStates() {
        const double timeStep = std::clamp(animationTimeStep, 0.0, 0.25);
        for (DisplayPart& part : displayLists) {
            if (part.animationStates.size() != part.animations.size()) {
                part.animationStates.resize(part.animations.size());
            }
            for (std::size_t index = 0; index < part.animations.size(); ++index) {
                const ModelAnimation& animation = part.animations[index];
                DisplayPart::AnimationState& state = part.animationStates[index];
                if (animation.type.empty()) {
                    state.currentAmount = 0.0;
                    state.targetAmount = 0.0;
                    state.initialized = true;
                    continue;
                }
                const double targetAmount =
                    variables.get(animation.variable) * animation.scale + animation.offset;
                if (!state.initialized) {
                    state.currentAmount = targetAmount;
                    state.targetAmount = targetAmount;
                    state.initialized = true;
                    continue;
                }
                if (targetAmount != state.targetAmount) {
                    state.targetAmount = targetAmount;
                }
                double nextAmount = state.targetAmount;
                if (animation.delay > 0.0) {
                    const double interpolation = std::min(1.0, animation.delay * timeStep);
                    nextAmount = state.currentAmount +
                                 (state.targetAmount - state.currentAmount) * interpolation;
                }
                if (animation.maxSpeed > 0.0) {
                    const double maximumStep = animation.maxSpeed * timeStep;
                    state.currentAmount +=
                        std::clamp(nextAmount - state.currentAmount, -maximumStep, maximumStep);
                } else {
                    state.currentAmount = nextAmount;
                }
            }
        }
    }

    Matrix4 animationOriginMatrix(const ModelAnimation& animation) const {
        Matrix4 origin = identityMatrix();
        for (const ModelAnimationOrigin& operation : animation.originOperations) {
            Matrix4 operationMatrix = identityMatrix();
            switch (operation.type) {
            case ModelAnimationOriginType::Translation:
                operationMatrix = translationMatrix(operation.value);
                break;
            case ModelAnimationOriginType::RotationX:
                operationMatrix = rotationMatrix(-operation.value[0], 0.0, -1.0, 0.0);
                break;
            case ModelAnimationOriginType::RotationY:
                operationMatrix = rotationMatrix(-operation.value[0], 1.0, 0.0, 0.0);
                break;
            case ModelAnimationOriginType::RotationZ:
                operationMatrix = rotationMatrix(-operation.value[0], 0.0, 0.0, 1.0);
                break;
            case ModelAnimationOriginType::FromMesh:
                if (animation.hasMeshTransform) {
                    operationMatrix = animation.meshTransform;
                }
                break;
            }
            origin = multiplyMatrix4(origin, operationMatrix);
        }
        if (animation.originOperations.empty() && animation.hasOrigin) {
            origin = translationMatrix(animation.origin);
        }
        return origin;
    }

    Matrix4 animationTransform(const ModelAnimation& animation, double amount) const {
        const Matrix4 origin = animationOriginMatrix(animation);
        Matrix4 inverseOrigin = identityMatrix();
        invertAffineMatrix(origin, inverseOrigin);
        const Matrix4 local = animation.type == "anim_rot" ? rotationMatrix(-amount, 0.0, -1.0, 0.0)
                              : animation.type == "anim_trans"
                                  ? translationMatrix({0.0, -amount, 0.0})
                                  : identityMatrix();
        return multiplyMatrix4(multiplyMatrix4(origin, local), inverseOrigin);
    }

    Matrix4 animationTransformForPart(const DisplayPart& part,
                                      std::vector<const DisplayPart*>& active) const {
        Matrix4 local = identityMatrix();
        for (std::size_t index = 0; index < part.animations.size(); ++index) {
            const ModelAnimation& animation = part.animations[index];
            const double value = variables.get(animation.variable);
            const double amount = index < part.animationStates.size()
                                      ? part.animationStates[index].currentAmount
                                      : value * animation.scale + animation.offset;
            local = multiplyMatrix4(animationTransform(animation, amount), local);
        }
        if (part.animationParent.empty() ||
            std::find(active.begin(), active.end(), &part) != active.end()) {
            return local;
        }
        const DisplayPart* parent = nullptr;
        const auto findParent = [&](const std::vector<DisplayPart>& parts) {
            const auto found =
                std::find_if(parts.begin(), parts.end(), [&](const DisplayPart& candidate) {
                    return candidate.meshIdentifier == part.animationParent;
                });
            return found == parts.end() ? nullptr : &*found;
        };
        parent = findParent(displayLists);
        if (parent == nullptr) {
            for (const WheelModel& wheel : wheelModels) {
                parent = findParent(wheel.parts);
                if (parent != nullptr) {
                    break;
                }
            }
        }
        if (parent == nullptr) {
            return local;
        }
        active.push_back(&part);
        const Matrix4 parentTransform = animationTransformForPart(*parent, active);
        active.pop_back();
        return multiplyMatrix4(parentTransform, local);
    }

    void applyAnimations(const DisplayPart& part) const {
        std::vector<const DisplayPart*> active;
        multiplyMatrix(animationTransformForPart(part, active));
    }

    static void setBackFaceCulling(bool enabled) {
        glFrontFace(GL_CW);
        glCullFace(GL_BACK);
        if (enabled) {
            glEnable(GL_CULL_FACE);
        } else {
            glDisable(GL_CULL_FACE);
        }
    }

    void updateFrameVariables(bool isAiVehicle, double timeStep) {
        // Frame-scoped values are refreshed before simulation and rendering run.
        variables.updateFrame();
        variables.set("AI", isAiVehicle ? 1.0 : 0.0);
        animationTimeStep = timeStep;
    }

    void updateScripts(bool isAiVehicle) {
        if (scripts) {
            const std::size_t scriptErrorCount = scripts->errors().size();
            scripts->update(isAiVehicle);
            for (std::size_t index = scriptErrorCount; index < scripts->errors().size(); ++index) {
                gameLog.Log("Lua frame error: " + scripts->errors()[index]);
            }
        }
    }

    void updateSimulationVariables(const BusSimulation& simulation, double throttle,
                                   double steering, double brake) {
        simulation.updateVariables(variables, throttle, steering, brake);
    }

    void joinTextureWorkers() {
        assets->join();
    }

#ifdef _WIN32
    bool comInitialized = false;
#endif

    explicit Vehicle(BusVehicle vehicle, ModelLoadingPolicy policy, AssetRequestManager& manager,
                     SimulationState& simulationState)
        : loadingPolicy(policy), variables(), assets(&manager) {
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
            relativeModelRoot = std::filesystem::path("SP_E400MMC") / "Model" / "SP_E400MMC_obj";
            modelOffsetZ = -1.02;
        } else {
            relativeConfig = std::filesystem::path("MAN_DL05") / "Model" / "DL05.cfg";
            relativeModelRoot = std::filesystem::path("MAN_DL05") / "Model" / "DL05_obj";
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
        const VehicleConfig vehicleConfiguration = loadBusConfig(busConfigurationPathFor(vehicle));
        scripts = std::make_unique<ScriptRuntime>(vehicleConfiguration, variables, simulationState);
        for (const std::string& error : scripts->errors()) {
            gameLog.Log("Lua script error: " + error);
        }
        load(resolveAssetPath(relativeConfig), resolveAssetPath(relativeModelRoot));
        scripts->initialize();
        for (const std::string& error : scripts->errors()) {
            gameLog.Log("Lua initialization error: " + error);
        }
        spawn();
    }

    ~Vehicle() {
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
#ifdef _WIN32
        if (comInitialized) {
            CoUninitialize();
        }
#endif
    }

    void drawBatch(Batch& batch, double alpha, bool forceUntextured = false,
                   const std::array<double, 3>* overrideColor = nullptr) {
        TraceScope trace("render", "Vehicle::drawBatch");
        if (alpha <= 0.0) {
            return;
        }
        if (batch.alphaMode == 2 || batch.noZwrite) {
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            glDepthMask(GL_FALSE);
        } else if (batch.alphaMode == 1) {
            glDisable(GL_BLEND);
            glDepthMask(GL_FALSE);
            glEnable(GL_POLYGON_OFFSET_FILL);
            glPolygonOffset(-1.0f, -1.0f);
        } else {
            glDisable(GL_BLEND);
            glDepthMask(GL_TRUE);
        }
        ensureTexture(batch);
        ensureAuxiliaryTexture(batch, batch.lightmap);
        ensureAuxiliaryTexture(batch, batch.nightmap);
        ensureAuxiliaryTexture(batch, batch.transmap);
        updateFreeTexture(batch);
        const std::array<double, 3>& color =
            overrideColor == nullptr ? batch.color : *overrideColor;
        openbus::rendering::ModelMaterial material;
        material.texture = batch.texture;
        material.textureArray = batch.textureArray;
        material.textured = batch.textured && !forceUntextured;
        material.lightmap = batch.lightmap.texture;
        material.nightmap = batch.nightmap.texture;
        material.transmap = batch.transmap.texture;
        material.freeTexture = batch.freeTexture;
        material.useLightmap = !forceUntextured && material.lightmap != 0;
        material.useNightmap = !forceUntextured && material.nightmap != 0;
        material.useTransmap = !forceUntextured && material.transmap != 0;
        material.useFreeTexture = !forceUntextured && material.freeTexture != 0;
        material.lightmapStrength = static_cast<float>(
            batch.lightmapStrengthVariable.empty()
                ? 1.0
                : std::clamp(variables.get(batch.lightmapStrengthVariable), 0.0, 1.0));
        const double nightlight =
            std::max(variables.get("NightlightA"), 1.0 - variables.get("Envir_Brightness"));
        material.nightmapStrength = static_cast<float>(std::clamp(nightlight, 0.0, 1.0));
        material.texcoordOffsetX = static_cast<float>(
            batch.texcoordTransXVariable.empty() ? 0.0
                                                 : variables.get(batch.texcoordTransXVariable));
        material.texcoordOffsetY = static_cast<float>(
            batch.texcoordTransYVariable.empty() ? 0.0
                                                 : variables.get(batch.texcoordTransYVariable));
        drawModelBatch(batch.buffer, batch.vertexCount, material, color, alpha,
                       forceUntextured ? 0 : batch.alphaMode);
    }

    void draw(RenderViewContext context) {
        TraceScope trace("render", "Vehicle::draw");
        if (!loaded) {
            return;
        }
        updateAnimationStates();
        // Texture uploads must happen on the OpenGL thread, so decoding and GL
        // upload are deliberately split between the worker and draw paths.
        textureUploadStart = std::chrono::steady_clock::now();

        const auto& modelView = openbus::rendering::modelViewMatrix();
        std::array<std::array<double, 4>, 6> frustumPlanes = {};
        std::array<double, 6> frustumPlaneLengths = {};

        {
            TraceScope phase("render", "Vehicle::draw.setup");

            glDisable(GL_BLEND);
            glDepthMask(GL_TRUE);
            glEnable(GL_DEPTH_TEST);
            setBackFaceCulling(false);
            glEnable(GL_POLYGON_OFFSET_FILL);
            glPolygonOffset(-1.0f, -1.0f);
        }

        {
            if (frustumCulling) {
                TraceScope phase("render", "Vehicle::draw.frustumSetup");
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
            // TraceScope phase("render", "Vehicle::draw.visible");
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
            // TraceScope phase("render", "Vehicle::draw.viewDepth");
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
            DisplayPart* part;
            double depth;
            int renderType;
        };
        std::vector<DisplayPart*> opaqueParts;
        std::vector<TransparentBatch> noDepthOpaqueBatches;
        std::vector<TransparentBatch> transparentBatches;
        {
            TraceScope phase("render", "Vehicle::draw.classifyParts");
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
                        transparentBatches.push_back(
                            {&batch, &part, viewDepth(part), part.renderType});
                    }
                }
                if (hasOpaqueBatch) {
                    opaqueParts.push_back(&part);
                }
            }
        }
        {
            TraceScope phase("render", "Vehicle::draw.sortBatches");
            std::sort(opaqueParts.begin(), opaqueParts.end(),
                      [&](const DisplayPart* first, const DisplayPart* second) {
                          return viewDepth(*first) < viewDepth(*second);
                      });
            std::stable_sort(transparentBatches.begin(), transparentBatches.end(),
                             [](const TransparentBatch& first, const TransparentBatch& second) {
                                 if (first.renderType != second.renderType) {
                                     return first.renderType < second.renderType;
                                 }
                                 return first.depth > second.depth;
                             });
            std::stable_sort(noDepthOpaqueBatches.begin(), noDepthOpaqueBatches.end(),
                             [](const TransparentBatch& first, const TransparentBatch& second) {
                                 if (first.renderType != second.renderType) {
                                     return first.renderType < second.renderType;
                                 }
                                 return first.depth > second.depth;
                             });
            lastRenderedTriangles = 0;
            for (DisplayPart* part : opaqueParts) {
                lastRenderedTriangles += part->triangleCount;
            }
            for (const TransparentBatch& transparent : transparentBatches) {
                lastRenderedTriangles += transparent.batch->vertexCount / 3;
            }
        }
        pushMatrix();
        translate(0.0, 0.0, modelOffsetZ);
        {
            TraceScope phase("render", "Vehicle::draw.opaquePass");
            for (DisplayPart* part : opaqueParts) {
                pushMatrix();
                applyAnimations(*part);
                setBackFaceCulling(part->backFaceCulling);
                for (Batch& batch : part->batches) {
                    const double alpha = alphaScale(batch);
                    if (alpha <= 0.0) {
                        continue;
                    }
                    if (batch.alphaMode != 0 || batch.noZwrite) {
                        continue;
                    }
                    drawBatch(batch, alpha);
                    drawEnvironmentMap(batch, alpha);
                }
                popMatrix();
            }
        }
        glDepthMask(GL_FALSE);
        glDisable(GL_BLEND);
        {
            TraceScope phase("render", "Vehicle::draw.noDepthPass");
            for (const TransparentBatch& noDepthOpaque : noDepthOpaqueBatches) {
                Batch& batch = *noDepthOpaque.batch;
                pushMatrix();
                applyAnimations(*noDepthOpaque.part);
                setBackFaceCulling(noDepthOpaque.part->backFaceCulling);
                const double alpha = alphaScale(batch);
                if (alpha <= 0.0) {
                    popMatrix();
                    continue;
                }
                drawBatch(batch, alpha);
                drawEnvironmentMap(batch, alpha);
                popMatrix();
            }
        }
        glDepthMask(GL_FALSE);
        {
            TraceScope phase("render", "Vehicle::draw.transparentPass");
            for (const TransparentBatch& transparent : transparentBatches) {
                Batch& batch = *transparent.batch;
                pushMatrix();
                applyAnimations(*transparent.part);
                setBackFaceCulling(transparent.part->backFaceCulling);
                const double alpha = alphaScale(batch);
                if (alpha <= 0.0) {
                    popMatrix();
                    continue;
                }
                if (batch.alphaMode == 1) {
                    glDisable(GL_BLEND);
                    glEnable(GL_POLYGON_OFFSET_FILL);
                    glPolygonOffset(-1.0f, -1.0f);
                } else if (batch.alphaMode == 2) {
                    glEnable(GL_BLEND);
                    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
                    glDisable(GL_POLYGON_OFFSET_FILL);
                } else if (batch.noZwrite) {
                    glEnable(GL_BLEND);
                    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
                    glDisable(GL_POLYGON_OFFSET_FILL);
                } else {
                    glDisable(GL_BLEND);
                    glDisable(GL_POLYGON_OFFSET_FILL);
                }
                drawBatch(batch, alpha);
                drawEnvironmentMap(batch, alpha);
                popMatrix();
            }
        }
        {
            TraceScope phase("render", "Vehicle::draw.cleanup");
            glDepthMask(GL_TRUE);
            glDisable(GL_BLEND);
            glDisable(GL_POLYGON_OFFSET_FILL);
            setBackFaceCulling(false);
            glDepthMask(GL_TRUE);
        }
        popMatrix();
    }

    bool hasConfiguredWheels(std::size_t expectedWheelCount) const {
        static_cast<void>(expectedWheelCount);
        return std::any_of(wheelModels.begin(), wheelModels.end(),
                           [](const WheelModel& wheel) { return !wheel.parts.empty(); });
    }

    void applyWheelVariableCorrections(const WheelAnimation& animation,
                                       const BusSimulation& simulation,
                                       std::size_t simulationIndex) const {
        constexpr double RADIANS_TO_DEGREES = 57.29577951308232;
        const auto steeringVariableMatchesAxle = [&](const std::string& variable) {
            const std::string normalized = lower(variable);
            constexpr const char* prefix = "axle_steering_";
            constexpr std::size_t prefixLength = 14;
            if (normalized.rfind(prefix, 0) != 0) {
                return true;
            }
            const std::size_t separator = normalized.find('_', prefixLength);
            if (separator == std::string::npos) {
                return true;
            }
            const int variableAxle =
                parseInt(normalized.substr(prefixLength, separator - prefixLength), -1);
            return variableAxle < 0 ||
                   static_cast<std::size_t>(variableAxle) == simulationIndex / 2;
        };
        if (!animation.steeringVariable.empty() &&
            steeringVariableMatchesAxle(animation.steeringVariable)) {
            const double requested = variables.get(animation.steeringVariable);
            const double physical = simulation.wheelSteeringAngle(simulationIndex);
            const double scale =
                animation.steeringScale == 0.0 ? RADIANS_TO_DEGREES : animation.steeringScale;
            rotate((requested - physical) * scale, 0.0, 0.0, 1.0);
        }
        if (!animation.rotationVariable.empty()) {
            const double requested = variables.get(animation.rotationVariable);
            const double scale =
                animation.rotationScale == 0.0 ? RADIANS_TO_DEGREES : animation.rotationScale;
            rotate(-requested * scale, 0.0, 1.0, 0.0);
        }
        if (!animation.suspensionVariable.empty()) {
            const double requested = variables.get(animation.suspensionVariable);
            const double physical = simulation.wheelSuspensionCompression(simulationIndex);
            const double scale = animation.suspensionScale == 0.0 ? 1.0 : animation.suspensionScale;
            translate(0.0, 0.0, (requested - physical) * scale);
        }
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
            const BodyPose pose = simulation.wheelMountPose(simulationIndex);
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
            double visualWheelDiameter = 0.0;
            for (const DisplayPart& part : wheel.parts) {
                visualWheelDiameter =
                    std::max(visualWheelDiameter, std::max(part.size[0], part.size[2]));
            }
            const BusAxle axle = simulation.axle(simulationIndex / 2);
            const double sideSign = simulationIndex % 2 == 0 ? 1.0 : -1.0;
            const double currentCenter = sideSign * axle.trackWidth * 0.5;
            const double lateralOffset = wheel.animation.origin[1] - currentCenter;
            const double diameterScale = visualWheelDiameter > 0.0 && axle.wheelDiameter > 0.0
                                             ? axle.wheelDiameter / visualWheelDiameter
                                             : 1.0;
            for (DisplayPart& part : wheel.parts) {
                const bool visibleOutside = part.viewpoint == 0 || (part.viewpoint & 1) != 0;
                const bool visibleInside = part.viewpoint == 0 || (part.viewpoint & 2) != 0;
                if ((outsideView && !visibleOutside) || (!outsideView && !visibleInside)) {
                    continue;
                }
                pushMatrix();
                applyPose(pose);
                rotate(90.0, 1.0, 0.0, 0.0);
                applyWheelVariableCorrections(wheel.animation, simulation, simulationIndex);
                translate(0.0, lateralOffset, 0.0);
                scale(diameterScale, diameterScale, diameterScale);
                translate(-meshOrigin[0], -meshOrigin[1], -meshOrigin[2]);
                for (Batch& batch : part.batches) {
                    setBackFaceCulling(part.backFaceCulling);
                    if (isWheelRubberTexture(batch.textureName)) {
                        const std::array<double, 3> rubberColor = {0.20, 0.20, 0.20};
                        drawBatch(batch, alphaScale(batch), true, &rubberColor);
                    } else {
                        drawBatch(batch, alphaScale(batch));
                    }
                }
                popMatrix();
            }
        }
        setBackFaceCulling(false);
        for (std::size_t index = 0; index < simulation.wheelCount(); ++index) {
            if (usedWheelIndices[index]) {
                continue;
            }
            const BodyPose pose = simulation.wheelPose(index);
            pushMatrix();
            applyPose(pose);
            drawWheel(simulation.wheelRadius(index), simulation.wheelHalfWidth());
            popMatrix();
        }
        setBackFaceCulling(false);
        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);
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
                    if (!batch.texturePath.empty() || !batch.textureName.empty()) {
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
                         {&batch.lightmap, &batch.nightmap, &batch.transmap}) {
                        if (auxiliary->name.empty()) {
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
        openbus::rendering::invalidateTextureBindings();
        assets->trackTexture(texture);
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
        openbus::rendering::invalidateTextureBindings();
        assets->trackTexture(texture);
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
        openbus::rendering::invalidateTextureBindings();
        assets->trackTexture(texture);
        static const bool verboseTextureUploadLogs =
            parseEnabledFlag(std::getenv("OPENBUS_VERBOSE_TEXTURE_UPLOAD"));
        if (verboseTextureUploadLogs) {
            gameLog.Log("Uploaded DXT5 texture from path: " + path.generic_string());
        }
        return texture;
    }

    static std::filesystem::path findTexture(const std::filesystem::path& root,
                                             const std::string& name) {
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

    std::shared_ptr<TextureCacheEntry> textureEntry(Batch& batch) {
        TraceScope phase("texture", "textureEntry");
        return assets->requestTexture(
            batch.textureRoot, batch.texturePath, batch.textureName,
            [](const std::filesystem::path& root, const std::filesystem::path& path,
               const std::string& name) {
                return Vehicle::findTexture(root, name.empty() ? path.string() : name);
            });
    }

    void ensureAuxiliaryTexture(Batch& batch, AuxiliaryTexture& auxiliary) {
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
                    auxiliary.cacheEntry->texture =
                        uploadCompressedTexture(request->resolvedPath, *request->compressedTexture,
                                                auxiliary.cacheEntry->textureArray,
                                                auxiliary.cacheEntry->textureArrayLayers);
                } else if (request->image) {
                    auxiliary.cacheEntry->texture =
                        uploadTexture(request->resolvedPath, *request->image);
                }
            }
        }
        auxiliary.texture = auxiliary.cacheEntry->texture;
        auxiliary.loadAttempted = true;
    }

    void updateFreeTexture(Batch& batch) {
        if (batch.freeTextureVariable.empty() || !scripts) {
            return;
        }
        const int index = static_cast<int>(std::lround(variables.get(batch.freeTextureVariable)));
        ScriptRuntime::ScriptTextureSnapshot snapshot;
        if (index < 0 || !scripts->copyScriptTexture(index, snapshot) || snapshot.width <= 0 ||
            snapshot.height <= 0 || snapshot.pixels.empty()) {
            return;
        }
        if (batch.freeTexture == 0) {
            glGenTextures(1, &batch.freeTexture);
            assets->trackTexture(batch.freeTexture);
        }
        glBindTexture(GL_TEXTURE_2D, batch.freeTexture);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        if (batch.freeTextureWidth != snapshot.width ||
            batch.freeTextureHeight != snapshot.height) {
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, snapshot.width, snapshot.height, 0, GL_RGBA,
                         GL_UNSIGNED_BYTE, snapshot.pixels.data());
            batch.freeTextureWidth = snapshot.width;
            batch.freeTextureHeight = snapshot.height;
        } else {
            glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, snapshot.width, snapshot.height, GL_RGBA,
                            GL_UNSIGNED_BYTE, snapshot.pixels.data());
        }
        openbus::rendering::invalidateTextureBindings();
        batch.freeTextureIndex = index;
    }

    void startTextureRequest(const std::shared_ptr<TextureCacheEntry>& entry) {
        assets->startTextureRequest(entry);
    }

    void ensureTexture(Batch& batch, bool visible = true) {
        if (batch.textureLoadAttempted && batch.textureChanges.empty()) {
            return;
        }
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
                        if (openbus::rendering::TextureLoader::readImage(
                                entry->request->resolvedPath, fallbackImage)) {
                            entry->texture = uploadTexture(entry->request->resolvedPath,
                                                           std::move(fallbackImage));
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

    void drawEnvironmentMap(Batch& batch, double alpha) {
        TraceScope trace("render", "Vehicle::drawEnvironmentMap");
        ensureEnvironmentTexture(batch);
        if (batch.environmentTexture == 0 || !batch.hasNormals || alpha <= 0.0) {
            return;
        }
        const double reflectionStrength =
            std::clamp(alpha * batch.environmentStrength * ENVIRONMENT_MAP_OPACITY, 0.0, 1.0);
        if (reflectionStrength <= 0.0) {
            return;
        }

        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glDepthMask(GL_FALSE);
        drawEnvironmentBatch(batch.buffer, batch.vertexCount, batch.environmentTexture,
                             reflectionStrength);
        glDisable(GL_BLEND);
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
                    ensureAuxiliaryTexture(batch, batch.lightmap);
                    ensureAuxiliaryTexture(batch, batch.nightmap);
                    ensureAuxiliaryTexture(batch, batch.transmap);
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
        return assets->requestObj(path);
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

        std::vector<ModelAnimation> animations = part.animations;
        for (ModelAnimation& animation : animations) {
            if (animation.originFromMesh) {
                if (parsed->hasTransform) {
                    // The converter preserves OBJ vertices and stores this transform as
                    // metadata. Only its source-space translation supplies the mesh pivot;
                    // applying its orientation again would double-transform the mesh.
                    if (!animation.hasOrigin) {
                        animation.origin = {parsed->transform[13], -parsed->transform[12],
                                            parsed->transform[14]};
                        animation.hasOrigin = true;
                    }
                    animation.meshRotation = {
                        parsed->transform[0], parsed->transform[1], parsed->transform[2],
                        parsed->transform[4], parsed->transform[5], parsed->transform[6],
                        parsed->transform[8], parsed->transform[9], parsed->transform[10]};
                    animation.hasMeshRotation = true;
                    const std::array<double, 3> meshOrigin = {
                        parsed->transform[13], -parsed->transform[12], parsed->transform[14]};
                    animation.meshTransform = makeMeshTransform(animation.meshRotation, meshOrigin);
                    animation.hasMeshTransform = true;
                } else if (!animation.hasOrigin) {
                    animation.origin = boundsCenter;
                    animation.hasOrigin = true;
                }
            }
        }
        if (verboseObjLoadLogs && !animations.empty()) {
            for (const ModelAnimation& animation : animations) {
                gameLog.Log("Model animation: " + part.objPath.filename().string() +
                            " type=" + animation.type + " variable=" + animation.variable +
                            " scale=" + std::to_string(animation.scale) + " origin=(" +
                            std::to_string(animation.origin[0]) + ',' +
                            std::to_string(animation.origin[1]) + ',' +
                            std::to_string(animation.origin[2]) +
                            ") hasOrigin=" + (animation.hasOrigin ? "true" : "false") +
                            " value=" + std::to_string(variables.get(animation.variable)));
            }
        }

        WheelAnimation wheelAnimation = part.wheelAnimation;
        if (verboseObjLoadLogs && !wheelAnimation.rotationVariable.empty()) {
            gameLog.Log("Wheel OBJ animation: " + part.objPath.filename().string() +
                        " rotation=" + wheelAnimation.rotationVariable +
                        " suspension=" + wheelAnimation.suspensionVariable +
                        " steering=" + wheelAnimation.steeringVariable);
        }
        if (!wheelAnimation.rotationVariable.empty() && !wheelAnimation.hasOrigin) {
            const auto rotationAnimation = std::find_if(
                animations.begin(), animations.end(), [&](const ModelAnimation& animation) {
                    return animation.variable == wheelAnimation.rotationVariable &&
                           animation.hasOrigin;
                });
            if (rotationAnimation != animations.end()) {
                wheelAnimation.origin = rotationAnimation->origin;
                wheelAnimation.hasOrigin = true;
            } else {
                wheelAnimation.origin = boundsCenter;
                wheelAnimation.hasOrigin = true;
            }
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
                        (!wheelAnimation.hasOrigin ||
                         (std::abs(wheelAnimation.origin[0] - candidate.animation.origin[0]) <
                              1e-6 &&
                          std::abs(wheelAnimation.origin[1] - candidate.animation.origin[1]) <
                              1e-6 &&
                          std::abs(wheelAnimation.origin[2] - candidate.animation.origin[2]) <
                              1e-6));
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
                             const std::vector<MaterialState::TextureChange>& textureChanges,
                             const MaterialState& materialState) {
            TraceScope batchTrace("obj", "loadObj.makeBatch");
            std::vector<Vertex> vertices;
            vertices.reserve(source.size() * 3);
            const auto convertPosition = [](const ObjPosition& position) {
                return std::array<double, 3>{position.y, -position.x, position.z};
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
                    if (triangle->indices[0].position > 0 && triangle->indices[1].position > 0 &&
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
                        const std::array<double, 3> edgeA = {
                            second[0] - first[0], second[1] - first[1], second[2] - first[2]};
                        const std::array<double, 3> edgeB = {
                            third[0] - first[0], third[1] - first[1], third[2] - first[2]};
                        fallbackNormal =
                            normalizeVector({edgeA[1] * edgeB[2] - edgeA[2] * edgeB[1],
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
                        if (index.normal > 0 && index.normal <= static_cast<int>(normals.size())) {
                            const ObjNormal& sourceNormal =
                                normals[static_cast<std::size_t>(index.normal - 1)];
                            normal =
                                normalizeVector({sourceNormal.y, -sourceNormal.x, sourceNormal.z});
                        }
                        const ObjTexCoord* texCoord = nullptr;
                        if (index.texCoord > 0 &&
                            index.texCoord <= static_cast<int>(texCoords.size())) {
                            texCoord = &texCoords[static_cast<std::size_t>(index.texCoord - 1)];
                        }
                        vertices.push_back(
                            {static_cast<float>(position.y), static_cast<float>(-position.x),
                             static_cast<float>(position.z),
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
            batch.lightmap.name = materialState.lightmapTextureName;
            batch.nightmap.name = materialState.nightmapTextureName;
            batch.transmap.name = materialState.transmapTextureName;
            batch.lightmapStrengthVariable = materialState.lightmapStrengthVariable;
            batch.freeTextureVariable = materialState.freeTextureVariable;
            batch.texcoordTransXVariable = materialState.texcoordTransXVariable;
            batch.texcoordTransYVariable = materialState.texcoordTransYVariable;
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
            const std::filesystem::path statePath = state.textureName.empty()
                                                        ? state.texturePath
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
                    entry.second.textureName.empty()
                        ? entry.second.texturePath
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
                    const auto states =
                        materialStatesByStemOccurrence.find(lower(materialPath.stem().string()));
                    if (occurrence != materialOccurrenceByIndex.end() &&
                        states != materialStatesByStemOccurrence.end() && occurrence->second >= 0 &&
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
        DisplayPart displayPart;
        displayPart.viewpoint = part.viewpoint;
        displayPart.renderType = part.renderType;
        displayPart.transparent = hasTransparentMaterial;
        displayPart.lodIndex = part.lodIndex;
        displayPart.visibleVariable = part.visibleVariable;
        displayPart.visibleValue = part.visibleValue;
        displayPart.meshIdentifier = part.meshIdentifier;
        displayPart.animationParent = part.animationParent;
        displayPart.backFaceCulling = parsed->backFaceCulling;
        displayPart.center = boundsCenter;
        displayPart.size = boundsSize;
        displayPart.radius = boundsRadius;
        displayPart.triangleCount = renderedTriangleCount;
        displayPart.animations = std::move(animations);
        destination->push_back(std::move(displayPart));
        for (const std::string& key : groupOrder) {
            const MaterialState& state = groupStates[key];
            makeBatch(groups[key], groupTextures[key], groupTextureNames[key], groupColors[key],
                      groupEnvironmentNames[key], groupEnvironmentStrengths[key], state.alphaMode,
                      state.noZwrite, state.alphaScaleVariable, state.textureChanges, state);
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

void Renderer::scrollCallback(GLFWwindow* window, double, double yOffset) {
    auto* renderer = static_cast<Renderer*>(glfwGetWindowUserPointer(window));
    if (renderer == nullptr) {
        return;
    }
    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS) {
        renderer->fieldOfViewOffset_ =
            std::clamp(renderer->fieldOfViewOffset_ - yOffset * 2.0, -40.0, 60.0);
        return;
    }
    renderer->cameraDistance_ = std::clamp(renderer->cameraDistance_ - yOffset * 2.0, 6.0, 80.0);
}

Renderer::Renderer(int width, int height, const char* title)
    : window_(nullptr), assetRequestManager_(std::make_unique<AssetRequestManager>()) {
    TraceScope trace("startup", "Renderer::Renderer");
    if (const char* scriptRate = std::getenv("OPENBUS_SCRIPT_HZ")) {
        try {
            scriptRateHz_ = std::clamp(std::stod(scriptRate), 0.0, 1000.0);
        } catch (const std::exception&) {
            scriptRateHz_ = 0.0;
        }
    }
    if (scriptRateHz_ > 0.0) {
        gameLog.Log("Script rate limited to " + std::to_string(scriptRateHz_) + " Hz");
    } else {
        gameLog.Log("Scripts follow the render rate");
    }
    if (!glfwInit()) {
        gameLog.Log("Failed to initialize GLFW");
        throw std::runtime_error("Failed to initialize GLFW");
    }
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
#endif
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
    if (!loadOpenGLFunctions()) {
        glfwDestroyWindow(window_);
        window_ = nullptr;
        glfwTerminate();
        throw std::runtime_error("OpenGL VBO functions are unavailable");
    }
    if (!openbus::rendering::initializeCoreRenderer()) {
        glfwDestroyWindow(window_);
        window_ = nullptr;
        glfwTerminate();
        throw std::runtime_error("Failed to initialize core OpenGL renderer");
    }
    glEnable(GL_DEPTH_TEST);
    glClearColor(0.45f, 0.65f, 0.88f, 1.0f);
    gameLog.Log("Renderer initialized");
}

Renderer::~Renderer() {
    gameLog.Log("Renderer shutting down");
    playerVehicle_ = nullptr;
    assetRequestManager_->join();
    vehicles_.clear();
    openbus::rendering::shutdownCoreRenderer();
    assetRequestManager_.reset();
    if (window_) {
        glfwDestroyWindow(window_);
    }
    glfwTerminate();
}

Vehicle* Renderer::AddVehicle(BusVehicle vehicle, ModelLoadingPolicy loadingPolicy) {
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
    auto model =
        std::make_unique<Vehicle>(vehicle, loadingPolicy, *assetRequestManager_, simulationState_);
    Vehicle* result = model.get();
    vehicles_.push_back(std::move(model));
    return result;
}

Vehicle* Renderer::AddVehicle(BusVehicle vehicle, const std::array<double, 3>& spawnPosition,
                              ModelLoadingPolicy loadingPolicy) {
    static_cast<void>(spawnPosition);
    return AddVehicle(vehicle, loadingPolicy);
}

void Renderer::SetPlayerVehicle(Vehicle* model) {
    playerVehicle_ = model;
}

void Renderer::updatePlayerVariables(const BusSimulation& simulation, double throttle,
                                     double steering, double brake) {
    if (playerVehicle_ != nullptr) {
        playerVehicle_->updateSimulationVariables(simulation, throttle, steering, brake);
    }
    updateScripts();
}

void Renderer::updateScripts() {
    const double renderTimeStep = std::clamp(frameTimeStep_, 0.0, 0.25);
    if (scriptRateHz_ <= 0.0) {
        simulationState_.sharedVariables().set("Timegap", renderTimeStep);
        for (const std::unique_ptr<Vehicle>& vehicle : vehicles_) {
            vehicle->updateScripts(vehicle.get() != playerVehicle_);
        }
        return;
    }

    const double scriptTimeStep = 1.0 / scriptRateHz_;
    scriptAccumulator_ += renderTimeStep;
    int ticks = 0;
    while (scriptAccumulator_ >= scriptTimeStep && ticks < MAX_SCRIPT_CATCH_UP_TICKS) {
        scriptAccumulator_ -= scriptTimeStep;
        simulationState_.sharedVariables().set("Timegap", scriptTimeStep);
        for (const std::unique_ptr<Vehicle>& vehicle : vehicles_) {
            vehicle->updateScripts(vehicle.get() != playerVehicle_);
        }
        ++ticks;
    }
    if (ticks == MAX_SCRIPT_CATCH_UP_TICKS && scriptAccumulator_ >= scriptTimeStep) {
        scriptAccumulator_ = std::fmod(scriptAccumulator_, scriptTimeStep);
    }
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
    const double baseFieldOfView =
        camera != nullptr && camera->fieldOfView > 0.0 ? camera->fieldOfView : 60.0;
    return std::clamp(baseFieldOfView + fieldOfViewOffset_, 20.0, 120.0);
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
    frameTimeStep_ = timegap;
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
    simulationState_.sharedVariables().updateFrame(timegap, currentTime, cursorX, cursorY);
    for (const std::unique_ptr<Vehicle>& vehicle : vehicles_) {
        const bool isAiVehicle = vehicle.get() != playerVehicle_;
        vehicle->updateFrameVariables(isAiVehicle, timegap);
    }
    const bool rightMouse = glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
    if (rightMouse && !draggingFov_) {
        previousFovCursorY_ = cursorY;
    } else if (rightMouse) {
        const double cursorDeltaY = cursorY - previousFovCursorY_;
        if (cameraView_ == 0) {
            cameraDistance_ = std::clamp(cameraDistance_ + cursorDeltaY * 0.1, 0.0, 80.0);
        } else {
            fieldOfViewOffset_ = std::clamp(fieldOfViewOffset_ + cursorDeltaY * 0.15, -40.0, 60.0);
        }
    }
    draggingFov_ = rightMouse;
    previousFovCursorY_ = cursorY;
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
            const std::array<double, 3> target =
                transformLocalPoint(chassis, simulation.outsideCameraCenter());
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
                playerVehicle_ != nullptr ? playerVehicle_->modelOffsetZ : 0.0;
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
        if (!captureMode_) {
            drawGround(simulation.roadBumps());
        }
    }

    const RenderViewContext context =
        isExteriorView() ? RenderViewContext::PlayerExterior : RenderViewContext::PlayerInterior;

    {
        TraceScope phase("render", "Renderer::draw.model");
        if (playerVehicle_ && playerVehicle_->loaded && !playerVehicle_->displayLists.empty()) {
            pushMatrix();
            applyPose(chassis);
            playerVehicle_->draw(context);
            popMatrix();
        } else {
            pushMatrix();
            applyPose(chassis);
            translate(collision.offsetX, collision.offsetY, collision.offsetZ);
            drawBox(collision.length, collision.width, collision.height, 0.85, 0.70, 0.08);
            popMatrix();
        }
    }

    if (playerVehicle_ && glfwGetTime() - lastStatsTitleTime_ > 0.25) {
        std::ostringstream title;
        title << "OpenBus - " << playerVehicle_->renderedTriangles() << " triangles";
        glfwSetWindowTitle(window_, title.str().c_str());
        lastStatsTitleTime_ = glfwGetTime();
    }

    {
        TraceScope phase("render", "Renderer::draw.overlays");
        // Render center of gravity marker
        const std::array<double, 3> centerOfGravity = simulation.centerOfGravity();
        pushMatrix();
        translate(centerOfGravity[0], centerOfGravity[1], centerOfGravity[2]);
        drawCenterOfGravityMarker(0.35);
        popMatrix();

        // Render axle lines
        const std::array<double, 3> axleColor = {0.20, 0.20, 0.20};
        std::vector<openbus::rendering::PrimitiveVertex> axleLines;
        axleLines.reserve(simulation.axleCount() * 2);
        for (std::size_t axleIndex = 0; axleIndex < simulation.axleCount(); ++axleIndex) {
            const BodyPose leftWheel = simulation.wheelPose(axleIndex * 2);
            const BodyPose rightWheel = simulation.wheelPose(axleIndex * 2 + 1);
            axleLines.push_back({static_cast<float>(rightWheel.position[0]),
                                 static_cast<float>(rightWheel.position[1]),
                                 static_cast<float>(rightWheel.position[2]),
                                 static_cast<float>(axleColor[0]), static_cast<float>(axleColor[1]),
                                 static_cast<float>(axleColor[2])});
            axleLines.push_back({static_cast<float>(leftWheel.position[0]),
                                 static_cast<float>(leftWheel.position[1]),
                                 static_cast<float>(leftWheel.position[2]),
                                 static_cast<float>(axleColor[0]), static_cast<float>(axleColor[1]),
                                 static_cast<float>(axleColor[2])});
        }
        openbus::rendering::drawPrimitives(axleLines, GL_LINES);
    }

    {
        TraceScope phase("render", "Renderer::draw.wheels");
        if (playerVehicle_ && playerVehicle_->hasConfiguredWheels(simulation.wheelCount())) {
            playerVehicle_->drawConfiguredWheels(simulation, chassis, isExteriorView());
        } else {
            for (std::size_t index = 0; index < simulation.wheelCount(); ++index) {
                const BodyPose wheel = simulation.wheelPose(index);
                pushMatrix();
                applyPose(wheel);
                drawWheel(simulation.wheelRadius(index), simulation.wheelHalfWidth());
                popMatrix();
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
        const std::filesystem::path path = directory / (std::string(view.name) + ".png");
        if (!openbus::rendering::saveFramebufferPng(path, width, height)) {
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
    return playerVehicle_ && playerVehicle_->isCaptureReady();
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
