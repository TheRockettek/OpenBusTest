#include "RenderLoop.h"

#include "AssetRequestManager.h"
#include "BusConfiguration.h"
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
#include "RendererReflection.h"
#include "RoadFeatures.h"
#include "ScriptRuntime.h"
#include "ScreenshotWriter.h"
#include "SoundEngine.h"
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
#include <string_view>
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

#ifndef GL_FRAMEBUFFER
#define GL_FRAMEBUFFER 0x8D40
#define GL_COLOR_ATTACHMENT0 0x8CE0
#define GL_DEPTH_ATTACHMENT 0x8D00
#define GL_FRAMEBUFFER_COMPLETE 0x8CD5
#define GL_DEPTH_COMPONENT24 0x81A6
#endif

#ifndef GL_CLAMP_TO_EDGE
#define GL_CLAMP_TO_EDGE 0x812F
#endif

#ifndef GL_CLAMP_TO_BORDER
#define GL_CLAMP_TO_BORDER 0x812D
#endif

#ifndef GL_MIRRORED_REPEAT
#define GL_MIRRORED_REPEAT 0x8370
#endif

#ifndef GL_MIRROR_CLAMP_TO_EDGE
#define GL_MIRROR_CLAMP_TO_EDGE 0x8743
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

using Matrix4 = openbus::rendering::Matrix4;

namespace {

enum class RenderViewContext {
    PlayerExterior = 1,
    PlayerInterior = 2,
    NonPlayer = 4,
};

enum class VehicleRenderPass {
    All,
    Opaque,
    Transparent,
};

constexpr int viewpointMask(RenderViewContext context) {
    return static_cast<int>(context);
}

constexpr double ENVIRONMENT_MAP_OPACITY = 0.1;
constexpr int MAX_SCRIPT_CATCH_UP_TICKS = 8;
constexpr double DEFAULT_FIELD_OF_VIEW = 60.0;
constexpr double MIN_FIELD_OF_VIEW = 20.0;
constexpr double MAX_FIELD_OF_VIEW = 120.0;

GLenum textureAddressModeToGl(TextureAddressMode mode) {
    switch (mode) {
    case TextureAddressMode::Clamp:
        return GL_CLAMP_TO_EDGE;
    case TextureAddressMode::Border:
        return GL_CLAMP_TO_BORDER;
    case TextureAddressMode::Mirror:
        return GL_MIRRORED_REPEAT;
    case TextureAddressMode::MirrorOnce:
        return GL_MIRROR_CLAMP_TO_EDGE;
    case TextureAddressMode::Repeat:
    default:
        return GL_REPEAT;
    }
}

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

std::array<double, 4> transformPoint(const Matrix4& matrix, const std::array<double, 4>& point) {
    std::array<double, 4> result = {};
    for (int row = 0; row < 4; ++row) {
        for (int column = 0; column < 4; ++column) {
            result[row] += matrix[row + column * 4] * point[column];
        }
    }
    return result;
}

using FrustumPlanes = std::array<std::array<double, 4>, 6>;
using FrustumPlaneLengths = std::array<double, 6>;

struct ViewFrustum {
    FrustumPlanes planes = {};
    FrustumPlaneLengths planeLengths = {};
};

ViewFrustum buildViewFrustum(const Matrix4& projection) {
    const std::array<std::array<double, 4>, 6> planeSigns = {{{{1.0, 0.0, 0.0, 1.0}},
                                                              {{-1.0, 0.0, 0.0, 1.0}},
                                                              {{0.0, 1.0, 0.0, 1.0}},
                                                              {{0.0, -1.0, 0.0, 1.0}},
                                                              {{0.0, 0.0, 1.0, 1.0}},
                                                              {{0.0, 0.0, -1.0, 1.0}}}};
    ViewFrustum frustum;
    for (std::size_t plane = 0; plane < planeSigns.size(); ++plane) {
        double lengthSquared = 0.0;
        for (int column = 0; column < 4; ++column) {
            double coefficient = 0.0;
            for (int row = 0; row < 4; ++row) {
                coefficient += planeSigns[plane][row] * projection[row + column * 4];
            }
            frustum.planes[plane][column] = coefficient;
            if (column < 3) {
                lengthSquared += coefficient * coefficient;
            }
        }
        frustum.planeLengths[plane] = std::sqrt(lengthSquared);
    }
    return frustum;
}

bool sphereOutsideFrustum(const ViewFrustum& frustum, const std::array<double, 4>& eye,
                          double radius) {
    for (std::size_t plane = 0; plane < frustum.planes.size(); ++plane) {
        const double planeDistance = frustum.planes[plane][0] * eye[0] +
                                     frustum.planes[plane][1] * eye[1] +
                                     frustum.planes[plane][2] * eye[2] + frustum.planes[plane][3];
        if (planeDistance < -radius * frustum.planeLengths[plane]) {
            return true;
        }
    }
    return false;
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
using openbus::rendering::drawCollisionWireframe;
using openbus::rendering::drawEnvironmentBatch;
using openbus::rendering::drawGround;
using openbus::rendering::drawMaterialBatch;
using openbus::rendering::drawModelBatch;
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

void applyVehiclePlacement(const VehiclePlacement& placement) {
    translate(placement.position[0], placement.position[1], placement.position[2]);
    rotate(placement.yawDegrees, 0.0, 0.0, 1.0);
}

struct Vehicle {
    ModelLoadingPolicy loadingPolicy;
    using TextureRequest = AssetRequestManager::TextureRequest;
    using TextureCacheEntry = AssetRequestManager::TextureCacheEntry;

    struct ObjRequest {
        std::shared_future<std::shared_ptr<openbus::rendering::ParsedObj>> future;
    };

    using MaterialState = openbus::rendering::BusModelMaterialState;

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

    struct MaterialTextureSource {
        GLuint texture = 0;
        bool textureArray = false;
        std::size_t textureArrayLayers = 1;
        std::filesystem::path texturePath;
        std::filesystem::path textureRoot;
        std::string textureName;
        bool textureLoadAttempted = false;
        bool textureLoadStarted = false;
        std::shared_ptr<TextureRequest> textureRequest;
        std::shared_ptr<TextureCacheEntry> textureCacheEntry;
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
        GLenum textureWrapS = GL_REPEAT;
        GLenum textureWrapT = GL_REPEAT;
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
        bool noZcheck = false;
        std::string alphaScaleVariable;
        int baseTextureLayer = 0;
        int textureLayer = 0;
        std::vector<Vertex> vertices;
        std::vector<std::array<float, 4>> materialColors;
        std::vector<MaterialTextureSource> materialTextures;
        std::vector<GLuint> materialTextureIds;
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
        int odeWheelIndex = -1;
        bool backFaceCulling = false;
        std::array<double, 3> center;
        std::array<double, 3> size;
        double radius;
        std::size_t triangleCount;
        std::vector<ModelAnimation> animations;
        std::vector<AnimationState> animationStates;
        std::vector<int> reflectionTextureIndices;
        mutable std::uint64_t animationCacheGeneration = 0;
        mutable Matrix4 cachedAnimationTransform = {};
        mutable std::uint64_t viewDepthCacheGeneration = 0;
        mutable double cachedViewDepth = 0.0;
    };

    std::vector<DisplayPart> displayLists;
    std::vector<bool> variableVisibleParts;
    std::unordered_set<int> visibleReflectionTextureIndices;
    std::unordered_map<int, int> reflectionRequiredSizes;
    openbus::scripting::Vehicle variables;
    std::unique_ptr<ScriptRuntime> scripts;
    std::vector<Part> pendingParts;
    AssetRequestManager* assets;
    VehiclePlacement placement;
    double modelOffsetZ = 0.0;
    double textureScale = 1;
    bool frustumCulling = true;
    std::vector<double> lodThresholds;
    std::size_t opaqueDisplayCount = 0;
    mutable std::size_t lastRenderedTriangles = 0;
    std::uint64_t viewDepthGeneration = 0;
    bool loaded = false;
    bool hasLoadedInitialView = false;
    bool loggedAllObjectsLoaded = false;
    int activeLod = -1;
    double animationTimeStep = 0.0;
    std::uint64_t animationGeneration = 1;
    bool wheelsFromOde = false;
    const BusSimulation* odeSimulation = nullptr;
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
        TraceScope trace("render", "Vehicle::updateAnimationStates");
        ++animationGeneration;
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
                                  ? translationMatrix({0.0, amount, 0.0})
                                  : identityMatrix();
        return multiplyMatrix4(multiplyMatrix4(origin, local), inverseOrigin);
    }

    Matrix4 animationTransformForPart(const DisplayPart& part,
                                      std::vector<const DisplayPart*>& active) const {
        if (part.animationCacheGeneration == animationGeneration) {
            return part.cachedAnimationTransform;
        }
        if (wheelsFromOde && odeSimulation != nullptr && part.odeWheelIndex >= 0) {
            const BodyPose chassis = odeSimulation->chassisPose();
            const BodyPose wheel =
                odeSimulation->wheelPose(static_cast<std::size_t>(part.odeWheelIndex));
            Matrix4 relative = identityMatrix();
            const std::array<double, 3> delta = {wheel.position[0] - chassis.position[0],
                                                 wheel.position[1] - chassis.position[1],
                                                 wheel.position[2] - chassis.position[2]};
            for (int row = 0; row < 3; ++row) {
                const double odeLocalPosition = chassis.rotation[row] * delta[0] +
                                                chassis.rotation[3 + row] * delta[1] +
                                                chassis.rotation[6 + row] * delta[2];
                const double modelLocalPosition =
                    part.center[row] + (row == 2 ? modelOffsetZ : 0.0);
                relative[12 + row] = odeLocalPosition - modelLocalPosition;
                for (int column = 0; column < 3; ++column) {
                    relative[column * 4 + row] =
                        chassis.rotation[row] * wheel.rotation[column] +
                        chassis.rotation[3 + row] * wheel.rotation[3 + column] +
                        chassis.rotation[6 + row] * wheel.rotation[6 + column];
                }
            }
            Matrix4 base = identityMatrix();
            base[5] = 0.0;
            base[6] = -1.0;
            base[9] = 1.0;
            base[10] = 0.0;
            Matrix4 inverseBase = identityMatrix();
            invertAffineMatrix(base, inverseBase);
            relative = multiplyMatrix4(relative, inverseBase);
            const Matrix4 toOrigin =
                translationMatrix({-part.center[0], -part.center[1], -part.center[2]});
            const Matrix4 fromOrigin =
                translationMatrix({part.center[0], part.center[1], part.center[2]});
            part.cachedAnimationTransform =
                multiplyMatrix4(fromOrigin, multiplyMatrix4(relative, toOrigin));
            part.animationCacheGeneration = animationGeneration;
            return part.cachedAnimationTransform;
        }
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
            if (std::find(active.begin(), active.end(), &part) != active.end()) {
                return local;
            }
            part.cachedAnimationTransform = local;
            part.animationCacheGeneration = animationGeneration;
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
            part.cachedAnimationTransform = local;
            part.animationCacheGeneration = animationGeneration;
            return local;
        }
        active.push_back(&part);
        const Matrix4 parentTransform = animationTransformForPart(*parent, active);
        active.pop_back();
        part.cachedAnimationTransform = multiplyMatrix4(parentTransform, local);
        part.animationCacheGeneration = animationGeneration;
        return part.cachedAnimationTransform;
    }

    Matrix4 animationTransformForPart(const DisplayPart& part) const {
        if (part.animationCacheGeneration == animationGeneration) {
            return part.cachedAnimationTransform;
        }
        std::vector<const DisplayPart*> active;
        return animationTransformForPart(part, active);
    }

    void applyAnimations(const DisplayPart& part) const {
        multiplyMatrix(animationTransformForPart(part));
    }

    std::array<double, 3> animatedPartCenter(const DisplayPart& part) const {
        const std::array<double, 4> local = {part.center[0], part.center[1], part.center[2], 1.0};
        const Matrix4 animation = animationTransformForPart(part);
        const std::array<double, 4> transformed = transformPoint(animation, local);
        return {transformed[0], transformed[1], transformed[2] + modelOffsetZ};
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
        TraceScope trace("frame", "Vehicle::updateFrameVariables");
        // Frame-scoped values are refreshed before simulation and rendering run.
        variables.updateFrame();
        variables.set("ai", isAiVehicle ? 1.0 : 0.0);
        animationTimeStep = timeStep;
    }

    void updateScripts(bool isAiVehicle) {
        TraceScope trace("script", "Vehicle::updateScripts");
        if (scripts) {
            const std::size_t scriptErrorCount = scripts->errors().size();
            scripts->update(isAiVehicle);
            for (std::size_t index = scriptErrorCount; index < scripts->errors().size(); ++index) {
                gameLog.Log("Lua frame error: " + scripts->errors()[index]);
            }
        }
    }

    const std::vector<int>& reflectionTextureIndicesForPart(const DisplayPart& part) const {
        return part.reflectionTextureIndices;
    }

    void cacheReflectionTextureIndices(DisplayPart& part) const {
        std::unordered_set<int> uniqueReflectionIndices;
        for (const Batch& batch : part.batches) {
            const auto registerReflection = [&](const std::string& textureName) {
                const int reflectionIndex =
                    openbus::rendering::reflectionTextureIndex(textureName);
                if (reflectionIndex >= 0) {
                    uniqueReflectionIndices.insert(reflectionIndex);
                }
            };
            registerReflection(batch.baseTextureName);
            registerReflection(batch.textureName);
            for (const MaterialTextureSource& source : batch.materialTextures) {
                registerReflection(source.textureName);
            }
            for (const MaterialState::TextureChange& change : batch.textureChanges) {
                registerReflection(change.textureName);
            }
        }
        part.reflectionTextureIndices.assign(uniqueReflectionIndices.begin(),
                                             uniqueReflectionIndices.end());
    }

    void prepareFrameVisibility(RenderViewContext context) {
        TraceScope trace("render", "Vehicle::prepareFrameVisibility");
        variableVisibleParts.resize(displayLists.size());
        visibleReflectionTextureIndices.clear();
        reflectionRequiredSizes.clear();
        GLint viewport[4] = {};
        glGetIntegerv(GL_VIEWPORT, viewport);
        const auto& modelView = openbus::rendering::modelViewMatrix();
        const auto& projection = openbus::rendering::projectionMatrix();
        const double viewportWidth = static_cast<double>(std::max(viewport[2], 1));
        const double viewportHeight = static_cast<double>(std::max(viewport[3], 1));
        ViewFrustum frustum;
        if (frustumCulling) {
            frustum = buildViewFrustum(projection);
        }
        {
            TraceScope phase("render", "Vehicle::prepareFrameVisibility.scanParts");
            for (std::size_t partIndex = 0; partIndex < displayLists.size(); ++partIndex) {
                const DisplayPart& part = displayLists[partIndex];
                const bool visible =
                    part.visibleVariable.empty() ||
                    variables.get(part.visibleVariable) == static_cast<double>(part.visibleValue);
                variableVisibleParts[partIndex] = visible;
                if (!visible ||
                    (part.viewpoint != 0 && (part.viewpoint & viewpointMask(context)) == 0)) {
                    continue;
                }
                const std::vector<int>& partReflectionIndices =
                    reflectionTextureIndicesForPart(part);
                if (partReflectionIndices.empty()) {
                    continue;
                }
                const std::array<double, 3> center = animatedPartCenter(part);
                const std::array<double, 4> local = {center[0], center[1], center[2], 1.0};
                const std::array<double, 4> eye = transformPoint(modelView, local);
                if (frustumCulling && sphereOutsideFrustum(frustum, eye, part.radius)) {
                    continue;
                }
                const Matrix4 animation = animationTransformForPart(part);
                const std::array<double, 3> halfSize = {std::max(part.size[0] * 0.5, 0.0),
                                                        std::max(part.size[1] * 0.5, 0.0),
                                                        std::max(part.size[2] * 0.5, 0.0)};
                double minimumNdcX = std::numeric_limits<double>::max();
                double maximumNdcX = std::numeric_limits<double>::lowest();
                double minimumNdcY = std::numeric_limits<double>::max();
                double maximumNdcY = std::numeric_limits<double>::lowest();
                bool hasProjectedCorner = false;
                bool intersectsNearPlane = false;

                for (int corner = 0; corner < 8; ++corner) {
                    const std::array<double, 4> cornerLocal = {
                        part.center[0] + ((corner & 1) == 0 ? -halfSize[0] : halfSize[0]),
                        part.center[1] + ((corner & 2) == 0 ? -halfSize[1] : halfSize[1]),
                        part.center[2] + ((corner & 4) == 0 ? -halfSize[2] : halfSize[2]), 1.0};
                    std::array<double, 4> animated = transformPoint(animation, cornerLocal);
                    animated[2] += modelOffsetZ;
                    const std::array<double, 4> cornerEye = transformPoint(modelView, animated);
                    const double depth = -cornerEye[2];
                    if (depth <= openbus::rendering::kReflectionNearPlane) {
                        intersectsNearPlane = true;
                        continue;
                    }
                    const double clipX =
                        projection[0] * cornerEye[0] + projection[4] * cornerEye[1] +
                        projection[8] * cornerEye[2] + projection[12] * cornerEye[3];
                    const double clipY =
                        projection[1] * cornerEye[0] + projection[5] * cornerEye[1] +
                        projection[9] * cornerEye[2] + projection[13] * cornerEye[3];
                    minimumNdcX = std::min(minimumNdcX, clipX / depth);
                    maximumNdcX = std::max(maximumNdcX, clipX / depth);
                    minimumNdcY = std::min(minimumNdcY, clipY / depth);
                    maximumNdcY = std::max(maximumNdcY, clipY / depth);
                    hasProjectedCorner = true;
                }
                if (!hasProjectedCorner) {
                    continue;
                }
                const double projectedWidth =
                    intersectsNearPlane
                        ? viewportWidth
                        : std::clamp((maximumNdcX - minimumNdcX) * 0.5 * viewportWidth, 0.0,
                                     viewportWidth);
                const double projectedHeight =
                    intersectsNearPlane
                        ? viewportHeight
                        : std::clamp((maximumNdcY - minimumNdcY) * 0.5 * viewportHeight, 0.0,
                                     viewportHeight);
                const double screenBoundedDiameter = std::max(projectedWidth, projectedHeight);
                const int requiredSize = std::max(
                    openbus::rendering::kMinReflectionTargetSize,
                    static_cast<int>(std::ceil(screenBoundedDiameter)));

                for (const int reflectionIndex : partReflectionIndices) {
                    visibleReflectionTextureIndices.insert(reflectionIndex);
                    reflectionRequiredSizes[reflectionIndex] =
                        std::max(reflectionRequiredSizes[reflectionIndex], requiredSize);
                }
            }
        }
    }

    bool needsReflectionTexture(std::size_t reflectionIndex) const {
        return visibleReflectionTextureIndices.find(static_cast<int>(reflectionIndex)) !=
               visibleReflectionTextureIndices.end();
    }

    int requiredReflectionSize(std::size_t reflectionIndex) const {
        const auto required = reflectionRequiredSizes.find(static_cast<int>(reflectionIndex));
        return required == reflectionRequiredSizes.end()
               ? openbus::rendering::kMinReflectionTargetSize
                                                         : required->second;
    }

    void updateSimulationVariables(const BusSimulation& simulation, double throttle,
                                   double steering, double brake) {
        simulation.updateVariables(variables, throttle, steering, brake);
    }

    void setOdeSimulation(const BusSimulation& simulation) {
        odeSimulation = &simulation;
    }

    void joinTextureWorkers() {
        assets->join();
    }

#ifdef _WIN32
    bool comInitialized = false;
#endif

        explicit Vehicle(const std::filesystem::path& busConfigPath,
                                         const std::filesystem::path& modelConfigPath,
                                         const VehiclePlacement& configuredPlacement, double configuredModelOffsetZ,
                                         ModelLoadingPolicy policy, AssetRequestManager& manager,
                     SimulationState& simulationState, SoundEngine& soundEngine)
                : loadingPolicy(policy), variables(), assets(&manager), placement(configuredPlacement),
          modelOffsetZ(configuredModelOffsetZ),
          wheelsFromOde(parseEnabledFlag(std::getenv("OPENBUS_WHEELS_FROM_ODE"))) {
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
        const std::filesystem::path modelConfigDirectory = modelConfigPath.parent_path();
        const std::filesystem::path modelRoot =
            lower(modelConfigDirectory.filename().string()) == "configuration files"
                ? modelConfigDirectory.parent_path()
                : modelConfigDirectory;
        gameLog.Log("Loading bus model with config: " + modelConfigPath.string() +
                    " and model root: " + modelRoot.string());
        const VehicleConfig vehicleConfiguration = loadBusConfig(busConfigPath);
        soundEngine.load(vehicleConfiguration.soundConfigPath);
        scripts = std::make_unique<ScriptRuntime>(
            vehicleConfiguration, variables, simulationState,
            [&soundEngine](const std::string& name, const std::string& file, double controlValue) {
                soundEngine.trigger(name, file, controlValue);
            });
        for (const std::string& error : scripts->errors()) {
            gameLog.Log("Lua script error: " + error);
        }
        load(modelConfigPath, modelRoot);
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
#ifdef _WIN32
        if (comInitialized) {
            CoUninitialize();
        }
#endif
    }

    void drawBatch(Batch& batch, double alpha, bool forceUntextured = false,
                   const std::array<double, 3>* overrideColor = nullptr,
                   int alphaModeOverride = -1) {
        TraceScope trace("render", "Vehicle::drawBatch");
        if (alpha <= 0.0) {
            return;
        }
        const int alphaMode = alphaModeOverride >= 0 ? alphaModeOverride : batch.alphaMode;
        const bool noZwrite = alphaModeOverride >= 0 ? false : batch.noZwrite;
        if (alphaMode == 2 || noZwrite) {
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            glDepthMask(GL_FALSE);
        } else if (alphaMode == 1) {
            glDisable(GL_BLEND);
            glDepthMask(GL_FALSE);
            glEnable(GL_POLYGON_OFFSET_FILL);
            glPolygonOffset(-1.0f, -1.0f);
        } else {
            glDisable(GL_BLEND);
            glDepthMask(GL_TRUE);
        }
        if (batch.noZcheck) {
            glDisable(GL_DEPTH_TEST);
        }
        const bool materialBatch = !batch.materialTextures.empty() && !forceUntextured;
        const std::vector<GLuint>* materialTextureIds = nullptr;
        std::vector<bool> materialTextureFlips;
        if (materialBatch) {
            batch.materialTextureIds.resize(batch.materialTextures.size());
            materialTextureFlips.resize(batch.materialTextures.size());
            for (MaterialTextureSource& source : batch.materialTextures) {
                ensureMaterialTexture(source);
                if (!openbus::rendering::reflectionPassActive()) {
                    const int reflectionIndex =
                        openbus::rendering::reflectionTextureIndex(source.textureName);
                    const unsigned int reflectionTexture =
                        openbus::rendering::reflectionTextureForIndex(reflectionIndex);
                    if (reflectionTexture != 0) {
                        source.texture = reflectionTexture;
                        source.textureArray = false;
                        materialTextureFlips[&source - batch.materialTextures.data()] = true;
                    }
                }
                batch.materialTextureIds[&source - batch.materialTextures.data()] = source.texture;
            }
            materialTextureIds = &batch.materialTextureIds;
        } else {
            ensureTexture(batch);
        }
        ensureAuxiliaryTexture(batch, batch.lightmap);
        ensureAuxiliaryTexture(batch, batch.nightmap);
        ensureAuxiliaryTexture(batch, batch.transmap);
        updateFreeTexture(batch);
        const std::array<double, 3>& color =
            overrideColor == nullptr ? batch.color : *overrideColor;
        openbus::rendering::ModelMaterial material;
        material.texture = materialBatch ? 0 : batch.texture;
        material.textureArray = materialBatch ? false : batch.textureArray;
        material.textured = materialBatch ? false : batch.textured && !forceUntextured;
        material.textureWrapS = batch.textureWrapS;
        material.textureWrapT = batch.textureWrapT;
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
            std::max(variables.get("nightlighta"), 1.0 - variables.get("envir_brightness"));
        material.nightmapStrength = static_cast<float>(std::clamp(nightlight, 0.0, 1.0));
        material.texcoordOffsetX = static_cast<float>(
            batch.texcoordTransXVariable.empty() ? 0.0
                                                 : variables.get(batch.texcoordTransXVariable));
        material.texcoordOffsetY = static_cast<float>(
            batch.texcoordTransYVariable.empty() ? 0.0
                                                 : variables.get(batch.texcoordTransYVariable));
        if (!openbus::rendering::reflectionPassActive() && !materialBatch) {
            const int reflectionIndex = openbus::rendering::reflectionTextureIndex(
                batch.textureName.empty() ? batch.texturePath.string() : batch.textureName);
            const unsigned int reflectionTexture =
                openbus::rendering::reflectionTextureForIndex(reflectionIndex);
            if (reflectionTexture != 0) {
                material.texture = reflectionTexture;
                material.textureArray = false;
                material.textured = true;
                material.flipTextureY = true;
            }
        }
        if (materialBatch) {
            drawMaterialBatch(batch.buffer, batch.vertexCount, batch.materialColors,
                              *materialTextureIds, materialTextureFlips);
        } else {
            drawModelBatch(batch.buffer, batch.vertexCount, material, color, alpha,
                           forceUntextured ? 0 : alphaMode);
        }
        if (batch.noZcheck) {
            glEnable(GL_DEPTH_TEST);
        }
    }

    void sortTransparentBatch(Batch& batch) {
        if (batch.vertices.size() < 6 || batch.vertices.size() % 3 != 0) {
            return;
        }
        struct TriangleDepth {
            std::size_t index;
            double depth;
        };
        const Matrix4& modelView = openbus::rendering::modelViewMatrix();
        std::vector<TriangleDepth> triangles;
        triangles.reserve(batch.vertices.size() / 3);
        for (std::size_t index = 0; index < batch.vertices.size(); index += 3) {
            const auto& first = batch.vertices[index];
            const auto& second = batch.vertices[index + 1];
            const auto& third = batch.vertices[index + 2];
            const std::array<double, 4> center = {
                (static_cast<double>(first.x) + second.x + third.x) / 3.0,
                (static_cast<double>(first.y) + second.y + third.y) / 3.0,
                (static_cast<double>(first.z) + second.z + third.z) / 3.0, 1.0};
            const std::array<double, 4> viewCenter = transformPoint(modelView, center);
            triangles.push_back({index, -viewCenter[2]});
        }
        std::stable_sort(triangles.begin(), triangles.end(),
                         [](const TriangleDepth& first, const TriangleDepth& second) {
                             return first.depth > second.depth;
                         });
        std::vector<Vertex> sortedVertices;
        sortedVertices.reserve(batch.vertices.size());
        for (const TriangleDepth& triangle : triangles) {
            sortedVertices.insert(sortedVertices.end(), batch.vertices.begin() + triangle.index,
                                  batch.vertices.begin() + triangle.index + 3);
        }
        pglBindBuffer(GL_ARRAY_BUFFER, batch.buffer);
        pglBufferData(GL_ARRAY_BUFFER,
                      static_cast<std::ptrdiff_t>(sortedVertices.size() * sizeof(Vertex)),
                      sortedVertices.data(), GL_STATIC_DRAW);
    }

    void draw(RenderViewContext context, VehicleRenderPass renderPass = VehicleRenderPass::All) {
        TraceScope trace("render", "Vehicle::draw");
        if (!loaded) {
            return;
        }
        ++viewDepthGeneration;
        const bool reflectionPass = openbus::rendering::reflectionPassActive();
        const bool renderOpaque = renderPass != VehicleRenderPass::Transparent;
        const bool renderTransparent =
            renderPass != VehicleRenderPass::Opaque &&
            (!reflectionPass || openbus::rendering::reflectionTransparentEnabled());
        if (!reflectionPass && renderPass != VehicleRenderPass::Transparent) {
            updateAnimationStates();
        }
        // Texture uploads must happen on the OpenGL thread, so decoding and GL
        // upload are deliberately split between the worker and draw paths.
        textureUploadStart = std::chrono::steady_clock::now();

        const auto& modelView = openbus::rendering::modelViewMatrix();
        ViewFrustum frustum;

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
                frustum = buildViewFrustum(openbus::rendering::projectionMatrix());
            }
        }

        const auto visible = [&](const DisplayPart& part, std::size_t partIndex) {
            // TraceScope phase("render", "Vehicle::draw.visible");
            if (partIndex >= variableVisibleParts.size() || !variableVisibleParts[partIndex]) {
                return false;
            }
            const std::array<double, 3> center = animatedPartCenter(part);
            const std::array<double, 4> local = {center[0], center[1], center[2], 1.0};
            const std::array<double, 4> eye = transformPoint(modelView, local);
            part.cachedViewDepth = -eye[2];
            part.viewDepthCacheGeneration = viewDepthGeneration;
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
            if (sphereOutsideFrustum(frustum, eye, part.radius)) {
                return false;
            }
            return true;
        };
        const auto viewDepth = [&](const DisplayPart& part) {
            // TraceScope phase("render", "Vehicle::draw.viewDepth");
            if (part.viewDepthCacheGeneration == viewDepthGeneration) {
                return part.cachedViewDepth;
            }
            const std::array<double, 3> center = animatedPartCenter(part);
            const std::array<double, 4> local = {center[0], center[1], center[2], 1.0};
            const double depth = -transformPoint(modelView, local)[2];
            part.cachedViewDepth = depth;
            part.viewDepthCacheGeneration = viewDepthGeneration;
            return depth;
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
        opaqueParts.reserve(displayLists.size());
        if (renderTransparent) {
            transparentBatches.reserve(displayLists.size());
            noDepthOpaqueBatches.reserve(displayLists.size());
        }
        {
            TraceScope phase("render", "Vehicle::draw.classifyParts");
            for (std::size_t partIndex = 0; partIndex < displayLists.size(); ++partIndex) {
                DisplayPart& part = displayLists[partIndex];
                const bool viewpointMatches =
                    part.viewpoint == 0 || (part.viewpoint & viewpointMask(context)) != 0;
                if (!viewpointMatches || !visible(part, partIndex)) {
                    continue;
                }
                const bool hasOpaqueBatch =
                    std::any_of(part.batches.begin(), part.batches.end(), [](const Batch& batch) {
                        return batch.alphaMode == 0 && !batch.noZwrite;
                    });
                for (Batch& batch : part.batches) {
                    if (renderTransparent && (batch.alphaMode != 0 || batch.noZwrite)) {
                        transparentBatches.push_back(
                            {&batch, &part, viewDepth(part), part.renderType});
                    }
                }
                if (renderOpaque && hasOpaqueBatch) {
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
            if (renderPass != VehicleRenderPass::Transparent) {
                lastRenderedTriangles = 0;
            }
            for (DisplayPart* part : opaqueParts) {
                lastRenderedTriangles += part->triangleCount;
            }
            for (const TransparentBatch& transparent : transparentBatches) {
                lastRenderedTriangles += transparent.batch->vertexCount / 3;
            }
        }
        pushMatrix();
        translate(0.0, 0.0, modelOffsetZ);
        if (renderOpaque) {
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
                    if (!reflectionPass) {
                        drawEnvironmentMap(batch, alpha);
                    }
                }
                popMatrix();
            }
        }
        if (renderTransparent) {
            {
                TraceScope phase("render", "Vehicle::draw.transparentDepthPrepass");
                glEnable(GL_DEPTH_TEST);
                glDepthFunc(GL_LESS);
                glDepthMask(GL_TRUE);
                glDisable(GL_BLEND);
                glDisable(GL_POLYGON_OFFSET_FILL);
                glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
                for (const TransparentBatch& transparent : transparentBatches) {
                    Batch& batch = *transparent.batch;
                    if (batch.alphaMode != 2 || batch.transmap.name.empty() || batch.noZwrite ||
                        batch.noZcheck) {
                        continue;
                    }
                    pushMatrix();
                    applyAnimations(*transparent.part);
                    setBackFaceCulling(transparent.part->backFaceCulling);
                    const double alpha = alphaScale(batch);
                    if (alpha > 0.0) {
                        drawBatch(batch, alpha, false, nullptr, 3);
                    }
                    popMatrix();
                }
                glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
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
                    if (!reflectionPass) {
                        drawEnvironmentMap(batch, alpha);
                    }
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
                        glDepthFunc(GL_LEQUAL);
                        glEnable(GL_POLYGON_OFFSET_FILL);
                        glPolygonOffset(-1.0f, -1.0f);
                    } else if (batch.noZwrite) {
                        glEnable(GL_BLEND);
                        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
                        glDisable(GL_POLYGON_OFFSET_FILL);
                    } else {
                        glDisable(GL_BLEND);
                        glDisable(GL_POLYGON_OFFSET_FILL);
                    }
                    if (batch.alphaMode == 2 && !batch.transmap.name.empty()) {
                        sortTransparentBatch(batch);
                    }
                    drawBatch(batch, alpha);
                    if (!reflectionPass) {
                        drawEnvironmentMap(batch, alpha);
                    }
                    popMatrix();
                }
            }
        }
        {
            TraceScope phase("render", "Vehicle::draw.cleanup");
            glDepthMask(GL_TRUE);
            glDepthFunc(GL_LESS);
            glDisable(GL_BLEND);
            glDisable(GL_POLYGON_OFFSET_FILL);
            setBackFaceCulling(false);
            glDepthMask(GL_TRUE);
        }
        popMatrix();
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
        return true;
    }

    bool isCaptureReady() const {
        return areObjectsLoaded() && areTexturesLoaded();
    }

    void spawn() {
        TraceScope trace("obj", "spawn");
        std::vector<std::pair<Part*, std::shared_future<std::shared_ptr<ParsedObj>>>> requests;
        requests.reserve(pendingParts.size());
        for (Part& part : pendingParts) {
            requests.emplace_back(&part, parsedObjFuture(part.objPath, part.bundleEntry));
        }
        for (const auto& request : requests) {
            loadObj(*request.first, request.second.get());
        }
        pendingParts.clear();
        rebuildDisplayOrder();
        preloadTextures();
        loaded = !displayLists.empty();
        hasLoadedInitialView = true;
        if (!loggedAllObjectsLoaded && loaded) {
            gameLog.Log("All objects loaded. bodyParts=" + std::to_string(displayLists.size()) +
                        " wheelParts=generic");
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

    static double parseDouble(const std::string& value, double fallback) {
        try {
            return std::stod(trim(value));
        } catch (const std::exception&) {
            return fallback;
        }
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
            std::size_t scaledSize = 0;
            if (!openbus::rendering::checkedTextureBufferSize(
                    static_cast<std::size_t>(scaledWidth), static_cast<std::size_t>(scaledHeight), 4,
                    scaledSize)) {
                gameLog.Log("Texture scaling exceeds the decoded image size limit: " +
                            path.generic_string());
                return 0;
            }
            std::vector<std::uint8_t> scaled(scaledSize);
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

    void ensureMaterialTexture(MaterialTextureSource& source, bool visible = true) {
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
                    source.textureCacheEntry->texture = uploadTexture(
                        source.texturePath, *source.textureCacheEntry->request->image);
                }
            }
        }
        source.texture = source.textureCacheEntry->texture;
        source.textureArray = source.textureCacheEntry->textureArray;
        source.textureArrayLayers = source.textureCacheEntry->textureArrayLayers;
        source.textureLoadAttempted = true;
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
                    if (!batch.materialTextures.empty()) {
                        for (MaterialTextureSource& source : batch.materialTextures) {
                            ensureMaterialTexture(source, loadingPolicy.textureMode ==
                                                              AssetLoadingMode::Eager);
                        }
                    } else if (!batch.texturePath.empty() || !batch.textureName.empty()) {
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
    }

    std::shared_future<std::shared_ptr<ParsedObj>>
    parsedObjFuture(const std::filesystem::path& path, const std::string& bundleEntry = {}) {
        return assets->requestObj(path, bundleEntry);
    }

    void loadObj(const Part& part, const std::shared_ptr<ParsedObj>& parsed) {
        TraceScope trace("obj", "loadObj");
        const std::string sourceStem = lower(part.objPath.stem().string());
        if (sourceStem == "shadow") {
            return;
        }
        static const bool verboseObjLoadLogs =
            parseEnabledFlag(std::getenv("OPENBUS_VERBOSE_OBJ_LOAD"));
        static const bool materialBatchingEnabled =
            parseEnabledFlag(std::getenv("OPENBUS_MATERIAL_BATCHING"));
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
            animation.variable = lower(animation.variable);
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

        std::vector<DisplayPart>* destination = &displayLists;
        auto makeBatch = [&](const std::vector<const ObjTriangle*>& source,
                             const std::filesystem::path& texturePath,
                             const std::string& textureName, const std::array<double, 3>& color,
                             const std::string& environmentTextureName, double environmentStrength,
                             int alphaMode, bool noZwrite, bool noZcheck,
                             const std::string& alphaScaleVariable,
                             const std::vector<MaterialState::TextureChange>& textureChanges,
                             const MaterialState& materialState) {
            TraceScope batchTrace("obj", "loadObj.makeBatch");
            std::vector<Vertex> vertices;
            if (source.size() > std::numeric_limits<std::size_t>::max() / 3) {
                throw std::runtime_error("triangle vertex count overflow");
            }
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
            batch.lightmapStrengthVariable = lower(materialState.lightmapStrengthVariable);
            batch.freeTextureVariable = lower(materialState.freeTextureVariable);
            batch.texcoordTransXVariable = lower(materialState.texcoordTransXVariable);
            batch.texcoordTransYVariable = lower(materialState.texcoordTransYVariable);
            batch.color = color;
            batch.hasNormals = true;
            batch.textureWrapS = textureAddressModeToGl(materialState.textureAddressS);
            batch.textureWrapT = textureAddressModeToGl(materialState.textureAddressT);
            batch.alphaMode = alphaMode;
            batch.noZwrite = noZwrite;
            batch.noZcheck = noZcheck;
            batch.alphaScaleVariable = lower(alphaScaleVariable);
            batch.textureChanges = textureChanges;
            for (MaterialState::TextureChange& change : batch.textureChanges) {
                change.activationVariable = lower(change.activationVariable);
            }
            batch.vertexCount = batch.vertices.size();

            const auto canMergeOpaqueBatches = [](const Batch& first, const Batch& second) {
                return first.alphaMode == 0 && second.alphaMode == 0 && !first.noZwrite &&
                       !second.noZwrite && !first.noZcheck && !second.noZcheck &&
                       first.texturePath == second.texturePath &&
                       first.textureRoot == second.textureRoot &&
                       first.textureName == second.textureName &&
                       first.environmentTextureName == second.environmentTextureName &&
                       first.environmentStrength == second.environmentStrength &&
                       first.color == second.color && first.textured == second.textured &&
                       first.hasNormals == second.hasNormals &&
                       first.textureWrapS == second.textureWrapS &&
                       first.textureWrapT == second.textureWrapT &&
                       first.lightmap.name == second.lightmap.name &&
                       first.nightmap.name == second.nightmap.name &&
                       first.transmap.name == second.transmap.name &&
                       first.lightmapStrengthVariable == second.lightmapStrengthVariable &&
                       first.freeTextureVariable == second.freeTextureVariable &&
                       first.texcoordTransXVariable == second.texcoordTransXVariable &&
                       first.texcoordTransYVariable == second.texcoordTransYVariable &&
                       first.alphaScaleVariable == second.alphaScaleVariable &&
                       first.baseTextureLayer == second.baseTextureLayer &&
                       first.textureLayer == second.textureLayer && first.textureChanges.empty() &&
                       second.textureChanges.empty();
            };
            std::vector<Batch>& batches = destination->back().batches;
            const auto compatibleBatch =
                std::find_if(batches.begin(), batches.end(), [&](const Batch& existing) {
                    return canMergeOpaqueBatches(existing, batch);
                });
            if (compatibleBatch != batches.end()) {
                Batch& merged = *compatibleBatch;
                merged.vertices.insert(merged.vertices.end(), batch.vertices.begin(),
                                       batch.vertices.end());
                merged.vertexCount = merged.vertices.size();
                TraceScope uploadTrace("obj", "loadObj.mergeBatch");
                pglBindBuffer(GL_ARRAY_BUFFER, merged.buffer);
                pglBufferData(GL_ARRAY_BUFFER,
                              static_cast<std::ptrdiff_t>(merged.vertices.size() * sizeof(Vertex)),
                              merged.vertices.data(), GL_STATIC_DRAW);
                return;
            }
            {
                TraceScope uploadTrace("obj", "loadObj.uploadVbo");
                pglGenBuffers(1, &batch.buffer);
                pglBindBuffer(GL_ARRAY_BUFFER, batch.buffer);
                pglBufferData(GL_ARRAY_BUFFER,
                              static_cast<std::ptrdiff_t>(batch.vertices.size() * sizeof(Vertex)),
                              batch.vertices.data(), GL_STATIC_DRAW);
            }
            batches.push_back(std::move(batch));
        };

        const auto canUseMaterialBatch = [](const Batch& batch) {
            return batch.alphaMode == 0 && !batch.noZwrite && !batch.noZcheck &&
                   !batch.textureArray && batch.textureChanges.empty() &&
                   batch.textureWrapS == GL_REPEAT && batch.textureWrapT == GL_REPEAT &&
                   batch.lightmap.name.empty() && batch.nightmap.name.empty() &&
                   batch.transmap.name.empty() && batch.freeTextureVariable.empty() &&
                   batch.texcoordTransXVariable.empty() && batch.texcoordTransYVariable.empty() &&
                   batch.alphaScaleVariable.empty();
        };
        const auto makeMaterialTexture = [](const Batch& source) {
            MaterialTextureSource result;
            result.texturePath = source.texturePath;
            result.textureRoot = source.textureRoot;
            result.textureName = source.textureName;
            return result;
        };
        const auto sameMaterialBatch = [&](const Batch& first, const Batch& second) {
            return canUseMaterialBatch(first) && canUseMaterialBatch(second) &&
                   first.environmentTextureName == second.environmentTextureName &&
                   first.environmentStrength == second.environmentStrength &&
                   first.textured == second.textured && first.hasNormals == second.hasNormals &&
                   first.baseTextureLayer == second.baseTextureLayer &&
                   first.textureLayer == second.textureLayer && second.materialColors.empty() &&
                   second.materialTextures.empty();
        };
        const auto consolidateMaterialBatches = [&](std::vector<Batch>& batches) {
            if (!materialBatchingEnabled) {
                return;
            }
            std::vector<Batch> consolidated;
            consolidated.reserve(batches.size());
            for (Batch& source : batches) {
                auto target = std::find_if(consolidated.begin(), consolidated.end(),
                                           [&](const Batch& candidate) {
                                               return candidate.materialTextures.size() < 8 &&
                                                      sameMaterialBatch(candidate, source);
                                           });
                if (target == consolidated.end()) {
                    if (canUseMaterialBatch(source)) {
                        for (Vertex& vertex : source.vertices) {
                            vertex.layer = 0.0f;
                        }
                        source.materialColors.push_back({static_cast<float>(source.color[0]),
                                                         static_cast<float>(source.color[1]),
                                                         static_cast<float>(source.color[2]),
                                                         1.0f});
                        source.materialTextures.push_back(makeMaterialTexture(source));
                    }
                    consolidated.push_back(std::move(source));
                    continue;
                }

                const std::size_t materialIndex = target->materialColors.size();
                for (Vertex& vertex : source.vertices) {
                    vertex.layer = static_cast<float>(materialIndex);
                }
                target->vertices.insert(target->vertices.end(), source.vertices.begin(),
                                        source.vertices.end());
                target->vertexCount = target->vertices.size();
                target->materialColors.push_back({static_cast<float>(source.color[0]),
                                                  static_cast<float>(source.color[1]),
                                                  static_cast<float>(source.color[2]), 1.0f});
                target->materialTextures.push_back(makeMaterialTexture(source));
                if (source.buffer != 0) {
                    pglDeleteBuffers(1, &source.buffer);
                    source.buffer = 0;
                }
                pglBindBuffer(GL_ARRAY_BUFFER, target->buffer);
                pglBufferData(GL_ARRAY_BUFFER,
                              static_cast<std::ptrdiff_t>(target->vertices.size() * sizeof(Vertex)),
                              target->vertices.data(), GL_STATIC_DRAW);
            }
            batches = std::move(consolidated);
            for (Batch& batch : batches) {
                if (batch.materialColors.size() < 2) {
                    batch.materialColors.clear();
                    batch.materialTextures.clear();
                }
            }
            const std::size_t materialBatchCount = static_cast<std::size_t>(
                std::count_if(batches.begin(), batches.end(),
                              [](const Batch& batch) { return batch.materialColors.size() >= 2; }));
            if (verboseObjLoadLogs && materialBatchCount != 0) {
                gameLog.Log("Material batches: " + std::to_string(materialBatchCount) + " from " +
                            std::to_string(batches.size()) + " total batches");
            }
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
            if (state.materialIndex > 10000) {
                gameLog.Log("Ignoring out-of-range material index " +
                            std::to_string(state.materialIndex));
                continue;
            }
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
                hasTransparentMaterial = hasTransparentMaterial || state.alphaMode != 0 ||
                                         state.noZwrite || state.noZcheck;
            }
        }
        DisplayPart displayPart;
        displayPart.viewpoint = part.viewpoint;
        displayPart.renderType = part.renderType;
        displayPart.transparent = hasTransparentMaterial;
        displayPart.lodIndex = part.lodIndex;
        displayPart.visibleVariable = lower(part.visibleVariable);
        displayPart.visibleValue = part.visibleValue;
        displayPart.meshIdentifier = part.meshIdentifier;
        displayPart.animationParent = part.animationParent;
        displayPart.backFaceCulling = parsed->backFaceCulling;
        displayPart.center = boundsCenter;
        displayPart.size = boundsSize;
        displayPart.radius = boundsRadius;
        displayPart.triangleCount = renderedTriangleCount;
        displayPart.animations = std::move(animations);
        const std::string wheelVariable = lower(part.wheelAnimation.rotationVariable);
        if (wheelVariable.rfind("wheel_rotation_", 0) == 0) {
            const std::size_t axleStart = std::string("wheel_rotation_").size();
            const std::size_t sideSeparator = wheelVariable.find('_', axleStart);
            if (sideSeparator != std::string::npos) {
                try {
                    const int axleIndex =
                        std::stoi(wheelVariable.substr(axleStart, sideSeparator - axleStart));
                    const std::string side = wheelVariable.substr(sideSeparator + 1);
                    if (axleIndex >= 0 && (side == "l" || side == "r")) {
                        displayPart.odeWheelIndex = axleIndex * 2 + (side == "r" ? 1 : 0);
                    }
                } catch (const std::exception&) {
                    displayPart.odeWheelIndex = -1;
                }
            }
        }
        destination->push_back(std::move(displayPart));
        for (const std::string& key : groupOrder) {
            const MaterialState& state = groupStates[key];
            makeBatch(groups[key], groupTextures[key], groupTextureNames[key], groupColors[key],
                      groupEnvironmentNames[key], groupEnvironmentStrengths[key], state.alphaMode,
                      state.noZwrite, state.noZcheck, state.alphaScaleVariable,
                      state.textureChanges, state);
        }
        consolidateMaterialBatches(destination->back().batches);
        cacheReflectionTextureIndices(destination->back());
        if (verboseObjLoadLogs) {
            for (const Batch& batch : destination->back().batches) {
                gameLog.Log("OBJ batch: " + part.objPath.filename().string() +
                            " vertices=" + std::to_string(batch.vertexCount) +
                            " alpha=" + std::to_string(batch.alphaMode) +
                            " noZwrite=" + (batch.noZwrite ? "true" : "false") +
                            " texture=" + batch.textureName +
                            " transmap=" + batch.transmap.name);
            }
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
            part.objRequest->future = parsedObjFuture(part.objPath, part.bundleEntry);
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
                parsed = parsedObjFuture(part->objPath, part->bundleEntry).get();
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
            gameLog.Log("All objects loaded. bodyParts=" + std::to_string(displayLists.size()) +
                        " wheelParts=generic");
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

void RenderLoop::scrollCallback(GLFWwindow* window, double, double yOffset) {
    auto* renderer = static_cast<RenderLoop*>(glfwGetWindowUserPointer(window));
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

RenderLoop::RenderLoop(int width, int height, const char* title)
        : window_(nullptr), assetRequestManager_(std::make_unique<AssetRequestManager>()),
            reflectionRenderer_(std::make_unique<openbus::rendering::ReflectionRenderer>()) {
    TraceScope trace("startup", "RenderLoop::RenderLoop");
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
    glfwSetScrollCallback(window_, &RenderLoop::scrollCallback);
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
    gameLog.Log("RenderLoop initialized");
}

RenderLoop::~RenderLoop() {
    gameLog.Log("RenderLoop shutting down");
    reflectionRenderer_->destroy();
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

Vehicle* RenderLoop::AddVehicle(const std::filesystem::path& busConfigPath,
                                const std::filesystem::path& modelConfigPath,
                                const VehiclePlacement& placement,
                                ModelLoadingPolicy loadingPolicy) {
    const std::filesystem::path resolvedBusConfigPath =
        busConfigurationPathFor(busConfigPath);
    const std::filesystem::path resolvedModelConfigPath =
        modelConfigurationPathForBus(resolvedBusConfigPath, modelConfigPath);
    const VehicleConfig vehicleConfiguration = loadBusConfig(resolvedBusConfigPath);
    if (vehicleCameras_.empty()) {
        for (const VehicleCamera& camera : vehicleConfiguration.cameras) {
            if (camera.kind == VehicleCameraKind::Driver ||
                camera.kind == VehicleCameraKind::Passenger ||
                camera.kind == VehicleCameraKind::Reflexion ||
                camera.kind == VehicleCameraKind::Reflexion2) {
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
        reflectionRenderer_->initialize(vehicleCameras_);
    }
    const double modelOffsetZ = -vehicleConfiguration.centerOfGravityHeight;
    auto model = std::make_unique<Vehicle>(resolvedBusConfigPath, resolvedModelConfigPath,
                                           placement, modelOffsetZ, loadingPolicy,
                                           *assetRequestManager_, simulationState_, soundEngine_);
    Vehicle* result = model.get();
    vehicles_.push_back(std::move(model));
    return result;
}

void RenderLoop::SetPlayerVehicle(Vehicle* model) {
    playerVehicle_ = model;
}

void RenderLoop::updatePlayerVariables(const BusSimulation& simulation, double throttle,
                                     double steering, double brake) {
    TraceScope trace("frame", "RenderLoop::updatePlayerVariables");
    soundEngine_.setListenerDistance(cameraView_ == 0 ? cameraDistance_ : 0.0);
    if (playerVehicle_ != nullptr) {
        playerVehicle_->updateSimulationVariables(simulation, throttle, steering, brake);
    }
    updateScripts();
}

void RenderLoop::updateScripts() {
    TraceScope trace("script", "RenderLoop::updateScripts");
    const double renderTimeStep = std::clamp(frameTimeStep_, 0.0, 0.25);
    if (scriptRateHz_ <= 0.0) {
        simulationState_.sharedVariables().set("timegap", renderTimeStep);
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
        simulationState_.sharedVariables().set("timegap", scriptTimeStep);
        for (const std::unique_ptr<Vehicle>& vehicle : vehicles_) {
            vehicle->updateScripts(vehicle.get() != playerVehicle_);
        }
        ++ticks;
    }
    if (ticks == MAX_SCRIPT_CATCH_UP_TICKS && scriptAccumulator_ >= scriptTimeStep) {
        scriptAccumulator_ = std::fmod(scriptAccumulator_, scriptTimeStep);
    }
}

bool RenderLoop::isExteriorView() const {
    if (cameraView_ == 0) {
        return true;
    }
    const VehicleCamera* camera = currentVehicleCamera();
    return camera != nullptr && camera->kind == VehicleCameraKind::Passenger &&
           camera->orbitDistance > 1.0;
}

const VehicleCamera* RenderLoop::currentVehicleCamera() const {
    if (cameraView_ <= 0 || static_cast<std::size_t>(cameraView_) > vehicleCameras_.size()) {
        return nullptr;
    }
    return &vehicleCameras_[static_cast<std::size_t>(cameraView_ - 1)];
}

void RenderLoop::selectVehicleCamera(int direction) {
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

double RenderLoop::currentFieldOfView() const {
    const VehicleCamera* camera = currentVehicleCamera();
    const double baseFieldOfView = camera != nullptr && camera->fieldOfView > 0.0
                                       ? camera->fieldOfView
                                       : DEFAULT_FIELD_OF_VIEW;
    return std::clamp(baseFieldOfView + fieldOfViewOffset_, MIN_FIELD_OF_VIEW, MAX_FIELD_OF_VIEW);
}

void RenderLoop::renderReflectionViews(const BusSimulation& simulation) {
    if (renderingReflection_ || reflectionRenderer_->empty()) {
        return;
    }
    const int previousCameraView = cameraView_;
    const double previousFovOffset = fieldOfViewOffset_;
    const double previousLookYaw = viewLookYaw_;
    const double previousLookPitch = viewLookPitch_;
    renderingReflection_ = true;
    reflectionRenderer_->render(
        simulation, vehicleCameras_,
        [this](std::size_t reflectionIndex) {
            openbus::rendering::ReflectionRequirement result;
            for (const std::unique_ptr<Vehicle>& vehicle : vehicles_) {
                if (vehicle->needsReflectionTexture(reflectionIndex)) {
                    result.needed = true;
                    result.size =
                        std::max(result.size, vehicle->requiredReflectionSize(reflectionIndex));
                }
            }
            return result;
        },
        [this](const BusSimulation& reflectionSimulation, std::size_t cameraIndex, int width,
               int height) {
            cameraView_ = static_cast<int>(cameraIndex + 1);
            fieldOfViewOffset_ = 0.0;
            viewLookYaw_ = 0.0;
            viewLookPitch_ = 0.0;
            setPerspective(static_cast<double>(width), static_cast<double>(height),
                           currentFieldOfView());
            draw(reflectionSimulation);
        },
        [this, previousCameraView, previousFovOffset, previousLookYaw, previousLookPitch](
            int width, int height) {
            cameraView_ = previousCameraView;
            fieldOfViewOffset_ = previousFovOffset;
            viewLookYaw_ = previousLookYaw;
            viewLookPitch_ = previousLookPitch;
            setPerspective(static_cast<double>(width), static_cast<double>(height),
                           currentFieldOfView());
        });
    renderingReflection_ = false;
}

void RenderLoop::renderReflectionDebugOverlay() {
    reflectionRenderer_->renderDebugOverlay(window_, reflectionDebugOverlay_);
}

bool RenderLoop::shouldClose() const {
    return glfwWindowShouldClose(window_) != 0;
}

void RenderLoop::requestClose() {
    if (window_ != nullptr) {
        glfwSetWindowShouldClose(window_, GLFW_TRUE);
    }
}

void RenderLoop::beginFrame() {
    TraceScope trace("frame", "RenderLoop::beginFrame");
    {
        TraceScope phase("frame", "RenderLoop::beginFrame.pollEvents");
        glfwPollEvents();
    }
    const double currentTime = glfwGetTime();
    const double timegap =
        hasPreviousVariableTime_ ? std::max(0.0, currentTime - previousVariableTime_) : 0.0;
    previousVariableTime_ = currentTime;
    hasPreviousVariableTime_ = true;
    frameTimeStep_ = timegap;
    {
        TraceScope phase("frame", "RenderLoop::beginFrame.keyboardInput");
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
    }
    {
        TraceScope phase("frame", "RenderLoop::beginFrame.viewInput");
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
    }
    {
        TraceScope phase("frame", "RenderLoop::beginFrame.cameraNavigation");
        const std::array<int, 2> cameraNavigationKeys = {GLFW_KEY_LEFT, GLFW_KEY_RIGHT};
        for (std::size_t index = 0; index < cameraNavigationKeys.size(); ++index) {
            const bool pressed = glfwGetKey(window_, cameraNavigationKeys[index]) == GLFW_PRESS;
            if (pressed && !previousCameraNavigationStates_[index]) {
                selectVehicleCamera(index == 0 ? -1 : 1);
            }
            previousCameraNavigationStates_[index] = pressed;
        }
    }
    {
        TraceScope phase("frame", "RenderLoop::beginFrame.captureInput");
        const bool captureKeyPressed = glfwGetKey(window_, GLFW_KEY_F12) == GLFW_PRESS;
        if (captureKeyPressed && !previousCaptureKeyState_) {
            captureRequested_ = true;
        }
        previousCaptureKeyState_ = captureKeyPressed;
    }
    {
        TraceScope phase("frame", "RenderLoop::beginFrame.debugInput");
        const bool reflectionDebugKeyPressed = glfwGetKey(window_, GLFW_KEY_R) == GLFW_PRESS;
        if (reflectionDebugKeyPressed && !previousReflectionDebugKeyState_) {
            reflectionDebugOverlay_ = !reflectionDebugOverlay_;
            gameLog.Log(std::string("Reflection texture overlay ") +
                        (reflectionDebugOverlay_ ? "enabled" : "disabled"));
        }
        previousReflectionDebugKeyState_ = reflectionDebugKeyPressed;
        const bool collisionDebugKeyPressed = glfwGetKey(window_, GLFW_KEY_C) == GLFW_PRESS;
        if (collisionDebugKeyPressed && !previousCollisionDebugKeyState_) {
            collisionDebugOverlay_ = !collisionDebugOverlay_;
            gameLog.Log(std::string("Collision wireframe overlay ") +
                        (collisionDebugOverlay_ ? "enabled" : "disabled"));
        }
        previousCollisionDebugKeyState_ = collisionDebugKeyPressed;
    }
    int width = 1;
    int height = 1;
    double cursorX = 0.0;
    double cursorY = 0.0;
    {
        TraceScope phase("frame", "RenderLoop::beginFrame.windowAndVariables");
        glfwGetFramebufferSize(window_, &width, &height);
        glfwGetCursorPos(window_, &cursorX, &cursorY);
        simulationState_.sharedVariables().updateFrame(timegap, currentTime, cursorX, cursorY);
        for (const std::unique_ptr<Vehicle>& vehicle : vehicles_) {
            const bool isAiVehicle = vehicle.get() != playerVehicle_;
            vehicle->updateFrameVariables(isAiVehicle, timegap);
        }
    }
    {
        TraceScope phase("frame", "RenderLoop::beginFrame.mouseInput");
        const bool rightMouse = glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
        if (rightMouse && !draggingFov_) {
            previousFovCursorY_ = cursorY;
        } else if (rightMouse) {
            const double cursorDeltaY = cursorY - previousFovCursorY_;
            if (cameraView_ == 0) {
                cameraDistance_ = std::clamp(cameraDistance_ + cursorDeltaY * 0.1, 0.0, 80.0);
            } else {
                fieldOfViewOffset_ =
                    std::clamp(fieldOfViewOffset_ + cursorDeltaY * 0.15, -40.0, 60.0);
            }
        }
        draggingFov_ = rightMouse;
        previousFovCursorY_ = cursorY;
        const bool middleMouse =
            glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
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
    }
    {
        TraceScope phase("render", "RenderLoop::beginFrame.setupView");
        glViewport(0, 0, width, height);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        setPerspective(static_cast<double>(width), static_cast<double>(height),
                       currentFieldOfView());
    }
}

void RenderLoop::draw(const BusSimulation& simulation) {
    TraceScope trace("frame", "RenderLoop::draw");
    const BodyPose chassis = simulation.chassisPose();
    const ChassisCollisionBox collision = simulation.chassisCollisionBox();
    {
        TraceScope phase("render", "RenderLoop::draw.camera");
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
            const bool reflectionCamera = camera->kind == VehicleCameraKind::Reflexion ||
                                          camera->kind == VehicleCameraKind::Reflexion2;
            const double panSign = reflectionCamera ? 1.0 : -1.0;
            const double pan = panSign * camera->pan * DEGREES_TO_RADIANS + viewLookYaw_;
            const double tilt = camera->tilt * DEGREES_TO_RADIANS + viewLookPitch_;
            const double modelOffsetZ =
                playerVehicle_ != nullptr ? playerVehicle_->modelOffsetZ : 0.0;
            const std::array<double, 3> centerLocal = {camera->position[1], -camera->position[0],
                                                       camera->position[2] + modelOffsetZ};
            const std::array<double, 3> direction = {
                std::cos(tilt) * std::cos(pan), std::cos(tilt) * std::sin(pan), std::sin(tilt)};
            const std::array<double, 3> upLocal = {
                -std::sin(tilt) * std::cos(pan), -std::sin(tilt) * std::sin(pan),
                std::cos(tilt)};
            const std::array<double, 3> eyeLocal = {
                centerLocal[0] - direction[0] * camera->orbitDistance,
                centerLocal[1] - direction[1] * camera->orbitDistance,
                centerLocal[2] - direction[2] * camera->orbitDistance};
            const std::array<double, 3> targetLocal = {eyeLocal[0] + direction[0] * 3.0,
                                                       eyeLocal[1] + direction[1] * 3.0,
                                                       eyeLocal[2] + direction[2] * 3.0};
            const std::array<double, 3> eye = transformLocalPoint(chassis, eyeLocal);
            const std::array<double, 3> target = transformLocalPoint(chassis, targetLocal);
            const std::array<double, 3> up = {
                chassis.rotation[0] * upLocal[0] + chassis.rotation[1] * upLocal[1] +
                    chassis.rotation[2] * upLocal[2],
                chassis.rotation[3] * upLocal[0] + chassis.rotation[4] * upLocal[1] +
                    chassis.rotation[5] * upLocal[2],
                chassis.rotation[6] * upLocal[0] + chassis.rotation[7] * upLocal[1] +
                    chassis.rotation[8] * upLocal[2]};
            lookAt(eye[0], eye[1], eye[2], target[0], target[1], target[2], up[0], up[1], up[2]);
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
        TraceScope phase("render", "RenderLoop::draw.visibility");
        if (!renderingReflection_) {
            const RenderViewContext context = isExteriorView() ? RenderViewContext::PlayerExterior
                                                               : RenderViewContext::PlayerInterior;
            if (playerVehicle_ != nullptr) {
                playerVehicle_->setOdeSimulation(simulation);
            }
            for (const std::unique_ptr<Vehicle>& vehicle : vehicles_) {
                pushMatrix();
                if (vehicle.get() == playerVehicle_) {
                    applyPose(chassis);
                    vehicle->prepareFrameVisibility(context);
                } else {
                    applyVehiclePlacement(vehicle->placement);
                    vehicle->prepareFrameVisibility(RenderViewContext::NonPlayer);
                }
                popMatrix();
            }
        }
    }
    {
        TraceScope phase("render", "RenderLoop::draw.reflections");
        renderReflectionViews(simulation);
    }
    {
        TraceScope phase("render", "RenderLoop::draw.ground");
        if (!captureMode_) {
            glEnable(GL_DEPTH_TEST);
            glDepthFunc(GL_LESS);
            glDepthMask(GL_TRUE);
            glDisable(GL_BLEND);
            glDisable(GL_CULL_FACE);
            glDisable(GL_POLYGON_OFFSET_FILL);
            drawGround(simulation.roadBumps());
        }
    }
    const RenderViewContext context = (renderingReflection_ || isExteriorView())
                                          ? RenderViewContext::PlayerExterior
                                          : RenderViewContext::PlayerInterior;
    {
        TraceScope phase("render", "RenderLoop::draw.model");
        bool playerDrawn = false;
        const auto drawVehicles = [&](VehicleRenderPass renderPass) {
            for (const std::unique_ptr<Vehicle>& vehicle : vehicles_) {
                if (!vehicle->loaded || vehicle->displayLists.empty()) {
                    continue;
                }
                pushMatrix();
                const RenderViewContext vehicleContext = vehicle.get() == playerVehicle_
                                                             ? context
                                                             : RenderViewContext::NonPlayer;
                if (vehicle.get() == playerVehicle_) {
                    applyPose(chassis);
                    if (renderPass == VehicleRenderPass::Opaque) {
                        playerDrawn = true;
                    }
                } else {
                    applyVehiclePlacement(vehicle->placement);
                }
                vehicle->draw(vehicleContext, renderPass);
                popMatrix();
            }
        };
        drawVehicles(VehicleRenderPass::Opaque);
        drawVehicles(VehicleRenderPass::Transparent);
        if (!playerDrawn) {
            pushMatrix();
            applyPose(chassis);
            translate(collision.offsetX, collision.offsetY, collision.offsetZ);
            drawBox(collision.length, collision.width, collision.height, 0.85, 0.70, 0.08);
            popMatrix();
        }
    }
    if (!renderingReflection_ && playerVehicle_ && glfwGetTime() - lastStatsTitleTime_ > 0.25) {
        std::ostringstream title;
        title << "OpenBus - " << playerVehicle_->renderedTriangles() << " triangles";
        glfwSetWindowTitle(window_, title.str().c_str());
        lastStatsTitleTime_ = glfwGetTime();
    }
    {
        TraceScope phase("render", "RenderLoop::draw.overlays");
        if (!renderingReflection_) {
            const std::array<double, 3> centerOfGravity = simulation.centerOfGravity();
            pushMatrix();
            translate(centerOfGravity[0], centerOfGravity[1], centerOfGravity[2]);
            drawCenterOfGravityMarker(0.35);
            popMatrix();
            const std::array<double, 3> axleColor = {0.20, 0.20, 0.20};
            std::vector<openbus::rendering::PrimitiveVertex> axleLines;
            axleLines.reserve(simulation.axleCount() * 2);
            for (std::size_t axleIndex = 0; axleIndex < simulation.axleCount(); ++axleIndex) {
                const BodyPose leftWheel = simulation.wheelPose(axleIndex * 2);
                const BodyPose rightWheel = simulation.wheelPose(axleIndex * 2 + 1);
                axleLines.push_back(
                    {static_cast<float>(rightWheel.position[0]),
                     static_cast<float>(rightWheel.position[1]),
                     static_cast<float>(rightWheel.position[2]), static_cast<float>(axleColor[0]),
                     static_cast<float>(axleColor[1]), static_cast<float>(axleColor[2])});
                axleLines.push_back(
                    {static_cast<float>(leftWheel.position[0]),
                     static_cast<float>(leftWheel.position[1]),
                     static_cast<float>(leftWheel.position[2]), static_cast<float>(axleColor[0]),
                     static_cast<float>(axleColor[1]), static_cast<float>(axleColor[2])});
            }
            openbus::rendering::drawPrimitives(axleLines, GL_LINES);
            if (collisionDebugOverlay_) {
                drawCollisionWireframe(simulation);
            }
        }
    }
    renderReflectionDebugOverlay();
}

void RenderLoop::endFrame() {
    TraceScope trace("frame", "RenderLoop::endFrame");
    glfwSwapBuffers(window_);
}

void RenderLoop::captureViews(const BusSimulation& simulation,
                            const std::filesystem::path& directory) {
    TraceScope trace("capture", "RenderLoop::captureViews");
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
        TraceScope viewTrace("capture", "RenderLoop::captureViews.view");
        cameraYaw_ = view.yaw;
        cameraPitch_ = view.pitch;
        cameraDistance_ = view.distance;
        {
            TraceScope phase("capture", "RenderLoop::captureViews.render");
            glViewport(0, 0, width, height);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            setPerspective(static_cast<double>(width), static_cast<double>(height), 60.0);
            draw(simulation);
            glFinish();
        }
        const std::filesystem::path path = directory / (std::string(view.name) + ".png");
        {
            TraceScope phase("capture", "RenderLoop::captureViews.save");
            if (!openbus::rendering::saveFramebufferPng(path, width, height)) {
                gameLog.Log("Failed to save screenshot: " + path.string());
            } else {
                gameLog.Log("Saved screenshot: " + path.string());
            }
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

bool RenderLoop::consumeCaptureRequest() {
    const bool requested = captureRequested_;
    captureRequested_ = false;
    return requested;
}

bool RenderLoop::isCaptureReady() const {
    return playerVehicle_ && playerVehicle_->isCaptureReady();
}

double RenderLoop::throttle() const {
    return glfwGetKey(window_, GLFW_KEY_W) == GLFW_PRESS ? 1.0 : 0.0;
}

double RenderLoop::steering() const {
    const bool left = glfwGetKey(window_, GLFW_KEY_A) == GLFW_PRESS;
    const bool right = glfwGetKey(window_, GLFW_KEY_D) == GLFW_PRESS;
    return static_cast<double>(right) - static_cast<double>(left);
}

double RenderLoop::brake() const {
    return glfwGetKey(window_, GLFW_KEY_S) == GLFW_PRESS ? 1.0 : 0.0;
}

std::vector<KeyEvent> RenderLoop::consumeKeyEvents() {
    std::vector<KeyEvent> events;
    events.swap(keyEvents_);
    return events;
}
