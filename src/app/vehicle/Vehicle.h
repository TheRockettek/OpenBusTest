#pragma once

// Runtime state for one vehicle instance: model parts, materials, textures,
// animations, scripts and sound. Internal to the app module; implementations
// are split across app/vehicle/Vehicle*.cpp by concern.

#include "app/RenderLoop.h"
#include "app/vehicle/VehicleDefinition.h"
#include "assets/ObjLoader.h"
#include "assets/ScreenshotWriter.h"
#include "assets/TextureAssetLoader.h"
#include "assets/TextureLoader.h"
#include "audio/SoundEngine.h"
#include "audio/VehicleSoundBank.h"
#include "config/BusConfigLoader.h"
#include "core/Environment.h"
#include "core/Logger.h"
#include "core/PerfTrace.h"
#include "core/Variables.h"
#include "core/Viewpoint.h"
#include "input/MouseControlMapping.h"
#include "input/VehicleInputBindings.h"
#include "physics/BusConfiguration.h"
#include "physics/BusSimulation.h"
#include "physics/RoadFeatures.h"
#include "render/AssetRequestManager.h"
#include "render/BusModelLoader.h"
#include "render/CameraMath.h"
#include "render/ComputeCulling.h"
#include "render/CoreRenderer.h"
#include "render/GlCompat.h"
#include "render/InteriorLighting.h"
#include "render/MatrixMath.h"
#include "render/OpenGLFunctions.h"
#include "render/RenderPrimitives.h"
#include "render/RendererReflection.h"
#include "render/map/MapRenderer.h"
#include "scripting/ScriptRuntime.h"

#ifndef GLFW_INCLUDE_NONE
#define GLFW_INCLUDE_NONE
#endif
#include <GLFW/glfw3.h>

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
#include <iomanip>
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

using Matrix4 = openbus::rendering::Matrix4;
using RenderViewContext = openbus::rendering::ViewpointContext;

enum class VehicleRenderPass {
    All,
    Opaque,
    Transparent,
};

inline constexpr double ENVIRONMENT_MAP_OPACITY = 0.1;

// Names used unqualified throughout the vehicle and render-loop implementation.
using openbus::rendering::applyPose;
using AssetRequestManager = openbus::rendering::AssetRequestManager;
using openbus::rendering::drawBox;
using openbus::rendering::drawEnvironmentBatch;
using openbus::rendering::drawGround;
using openbus::rendering::drawMaterialBatch;
using openbus::rendering::drawModelBatch;
using openbus::rendering::lookAt;
using openbus::rendering::makeGroundShadowTransform;
using openbus::rendering::makeModelRootPlacement;
using openbus::rendering::modelViewMatrix;
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

using openbus::rendering::buildViewFrustum;
using openbus::rendering::identityMatrix;
using openbus::rendering::invertAffineMatrix;
using openbus::rendering::makeMeshTransform;
using openbus::rendering::multiplyMatrix4;
using openbus::rendering::poseMatrix;
using openbus::rendering::rotationMatrix;
using openbus::rendering::sphereOutsideFrustum;
using openbus::rendering::textureAddressModeToGl;
using openbus::rendering::transformPoint;
using openbus::rendering::translationMatrix;
using openbus::rendering::ViewFrustum;

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
        float r = 1.0F, g = 1.0F, b = 1.0F, a = 1.0F;
    };

    struct AuxiliaryTexture {
        std::string name;
        GLuint texture = 0;
        bool loadAttempted = false;
        std::shared_ptr<TextureCacheEntry> cacheEntry;
        int width = 0;
        int height = 0;
        std::uint64_t revision = 0;
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

    struct InteriorLightController {
        bool isNumeric = false;
        double numericValue = 0.0;
    };

    struct Batch {
        struct SharedVertices {
            std::shared_ptr<std::vector<Vertex>> values = std::make_shared<std::vector<Vertex>>();

            SharedVertices& operator=(std::vector<Vertex>&& vertices) {
                values = std::make_shared<std::vector<Vertex>>(std::move(vertices));
                return *this;
            }
            std::size_t size() const {
                return values->size();
            }
            bool empty() const {
                return values->empty();
            }
            Vertex* data() {
                return values->data();
            }
            const Vertex* data() const {
                return values->data();
            }
            Vertex& operator[](std::size_t index) {
                return (*values)[index];
            }
            const Vertex& operator[](std::size_t index) const {
                return (*values)[index];
            }
            auto begin() {
                return values->begin();
            }
            auto end() {
                return values->end();
            }
            auto begin() const {
                return values->begin();
            }
            auto end() const {
                return values->end();
            }
            void reserve(std::size_t size) {
                values->reserve(size);
            }
            template <typename Iterator>
            auto insert(typename std::vector<Vertex>::iterator position, Iterator first,
                        Iterator last) {
                return values->insert(position, first, last);
            }
        };

        GLuint buffer = 0;
        bool bufferTracked = false;
        bool sharedGeometryBuffer = false;
        GLuint texture = 0;
        bool textureArray = false;
        std::size_t textureArrayLayers = 1;
        std::array<double, 3> specularColor = {};
        std::array<double, 3> emissionColor = {};
        double specularPower = 8.0;
        std::filesystem::path baseTexturePath;
        std::string baseTextureName;
        std::filesystem::path texturePath;
        std::filesystem::path textureRoot;
        std::filesystem::path sourceMeshPath;
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
        AuxiliaryTexture bumpmap;
        double bumpmapStrength = 0.0;
        std::array<int, 4> interiorLightIndexes = {-1, -1, -1, -1};
        std::string lightmapStrengthVariable;
        std::string freeTextureName;
        std::string freeTextureVariable;
        AuxiliaryTexture freeTextureAsset;
        int scriptTextureIndex = -1;
        int textTextureIndex = -1;
        std::string texcoordTransXVariable;
        std::string texcoordTransYVariable;
        GLuint freeTexture = 0;
        GLuint dynamicTexture = 0;
        int freeTextureIndex = -1;
        int freeTextureWidth = 0;
        int freeTextureHeight = 0;
        std::uint64_t freeTextureRevision = 0;
        int alphaMode = 0;
        bool noZwrite = false;
        bool noZcheck = false;
        std::string alphaScaleVariable;
        int baseTextureLayer = 0;
        int textureLayer = 0;
        SharedVertices vertices;
        std::vector<std::array<float, 4>> materialColors;
        std::vector<MaterialTextureSource> materialTextures;
        std::vector<GLuint> materialTextureIds;
        std::vector<bool> materialTextureFlips;
        std::vector<MaterialState::TextureChange> textureChanges;
        MaterialState baseMaterial;
        bool selectedMaterialItem = false;
        // Owning mesh has a [visible] gate; that gate switches its nightmap (e.g. warning lamps).
        bool visibilitySwitched = false;
        std::uint64_t materialSelectionGeneration = 0;
        std::size_t vertexCount = 0;
        // Lazily built local bounds of consecutive triangle groups; lets mouse picking
        // skip most triangles. Keyed by the vertex storage and its size.
        struct PickChunkBounds {
            std::array<float, 3> minimum;
            std::array<float, 3> maximum;
        };
        mutable std::vector<PickChunkBounds> pickChunkBounds;
        mutable std::vector<PickChunkBounds> pickChunkGroupBounds;
        mutable std::weak_ptr<std::vector<Vertex>> pickChunkSource;
        mutable std::size_t pickChunkVertexCount = 0;
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
        bool fixed = false;
        bool isShadow = false;
        bool transparent;
        int lodIndex;
        std::string meshName;
        std::string visibleVariable;
        int visibleValue = 0;
        std::string meshIdentifier;
        std::string animationParent;
        std::string mouseEvent;
        int odeWheelIndex = -1;
        bool backFaceCulling = false;
        std::array<double, 3> center;
        std::array<double, 3> size;
        double radius;
        std::vector<ModelAnimation> animations;
        std::array<int, 4> interiorLightIndexes = {-1, -1, -1, -1};
        std::vector<AnimationState> animationStates;
        std::vector<int> reflectionTextureIndices;
        bool pickOccluder = false;
        mutable std::uint64_t animationCacheGeneration = 0;
        mutable Matrix4 cachedAnimationTransform = {};
        mutable std::uint64_t viewDepthCacheGeneration = 0;
        mutable double cachedViewDepth = 0.0;
    };

    struct TransparentBatch {
        Batch* batch;
        DisplayPart* part;
        double depth;
        int renderType;
    };

    std::vector<DisplayPart> displayLists;
    std::vector<DisplayPart*> opaquePartsScratch;
    std::vector<TransparentBatch> noDepthOpaqueBatchesScratch;
    std::vector<TransparentBatch> transparentBatchesScratch;
    std::vector<bool> variableVisibleParts;
    std::vector<std::size_t> visibleClickablePartIndices;
    std::vector<std::size_t> visibleOpaquePickPartIndices;
    mutable std::size_t hoveredClickablePartIndex = std::numeric_limits<std::size_t>::max();
    mutable std::size_t hoveredClickableBatchIndex = std::numeric_limits<std::size_t>::max();
    mutable std::size_t hoveredClickableTriangle = 0;
    mutable std::vector<openbus::rendering::PrimitiveVertex> clickableDebugBoxVertices;
    mutable std::vector<openbus::rendering::PrimitiveVertex> clickableDebugWireVertices;
    mutable std::vector<openbus::rendering::PrimitiveVertex> clickableDebugTriangleVertices;
    std::unordered_set<int> visibleReflectionTextureIndices;
    std::unordered_map<int, int> reflectionRequiredSizes;
    openbus::scripting::Vehicle variables;
    std::unique_ptr<ScriptRuntime> scripts;
    SoundEngine* soundPlayback = nullptr;
    VehicleSoundBank soundBank;
    bool soundEventsEnabled = false;
    std::vector<Part> pendingParts;
    std::vector<ModelInteriorLight> interiorLights;
    std::vector<InteriorLightController> interiorLightControllers;
    Matrix4 interiorLightRootModelView = identityMatrix();
    std::string modelCacheKey;
    AssetRequestManager* assets;
    VehiclePlacement placement;
    bool followsMapTrafficRoute = false;
    double modelOffsetZ = 0.0;
    double aiDeltaHeight = 0.0;
    double textureScale = 1;
    double odometerMetres = 0.0;
    std::array<double, 2> odometerPosition = {};
    bool odometerPositionInitialized = false;
    bool frustumCulling = true;
    std::vector<double> lodThresholds;
    std::unordered_map<std::string, std::string> ctcTextureReplacements;
    std::size_t opaqueDisplayCount = 0;
    std::uint64_t viewDepthGeneration = 0;
    // The outer visibility pass advances this once; reflection views and the main
    // view then share unchanged material selections for the rest of that frame.
    std::uint64_t materialSelectionGeneration = 1;
    bool preparedDrawListsValid = false;
    RenderViewContext preparedDrawContext = RenderViewContext::PlayerExterior;
    bool preparedDrawReflection = false;
    bool preparedDrawTransparent = false;
    std::uint64_t preparedDrawAnimationGeneration = 0;
    std::uint64_t preparedDrawMaterialGeneration = 0;
    Matrix4 preparedDrawModelView = {};
    Matrix4 preparedDrawProjection = {};
    bool loaded = false;
    bool hasSharedGeometry = false;
    bool hasLoadedInitialView = false;
    bool loggedAllObjectsLoaded = false;
    int activeLod = -1;
    double animationTimeStep = 0.0;
    std::uint64_t animationGeneration = 1;
    std::uint64_t clickableStateRevision = 1;
    bool wheelsFromOde = false;
    const BusSimulation* odeSimulation = nullptr;
    std::chrono::steady_clock::time_point textureUploadStart;

    std::string resolveCtcTextureName(const std::string& textureName) const;

    void updateMaterialChange(Batch& batch);

    float alphaScale(const Batch& batch) const;

    void updateAnimationStates();

    Matrix4 animationOriginMatrix(const ModelAnimation& animation) const;

    Matrix4 animationTransform(const ModelAnimation& animation, double amount) const;

    Matrix4 animationTransformForPart(const DisplayPart& part,
                                      std::vector<const DisplayPart*>& active) const;

    Matrix4 animationTransformForPart(const DisplayPart& part) const;

    void applyAnimations(const DisplayPart& part) const;

    void applyDrawTransform(const DisplayPart& part, const Matrix4* groundShadowTransform) const;

    std::array<double, 3> animatedPartCenter(const DisplayPart& part) const;

    static void setOneSidedFrontFaceConvention();

    void updateFrameVariables(bool isAiVehicle, double timeStep);

    void updateScripts(bool isAiVehicle);

    void initializePlayerSystems();

    const std::vector<int>& reflectionTextureIndicesForPart(const DisplayPart& part) const;

    void cacheReflectionTextureIndices(DisplayPart& part) const;

    std::vector<std::uint32_t> computePartVisibility(bool selectLod) const;

    void prepareFrameVisibility(RenderViewContext context, int viewportWidth, int viewportHeight);

    bool needsReflectionTexture(std::size_t reflectionIndex) const;

    int requiredReflectionSize(std::size_t reflectionIndex) const;

    void updateSimulationVariables(const BusSimulation& simulation, double throttle,
                                   double steering, double brake);

    void setOdeSimulation(const BusSimulation& simulation);

    bool clickablePartVisible(const DisplayPart& part, std::size_t partIndex,
                              RenderViewContext context) const;

    const DisplayPart* pickClickable(double cursorX, double cursorY, int viewportWidth,
                                     int viewportHeight, RenderViewContext context,
                                     const Batch** hitBatch = nullptr,
                                     std::size_t* hitTriangle = nullptr) const;

    bool hasClickableAt(double cursorX, double cursorY, int viewportWidth, int viewportHeight,
                        RenderViewContext context) const;

    bool hasVisibleClickable(RenderViewContext context) const;

    std::uint64_t clickableRevision() const;

    std::string clickableDebugDescription(const DisplayPart& part, std::size_t partIndex,
                                          RenderViewContext context) const;

    void logClickableDebugInfo(RenderViewContext context) const;

    void drawClickableDebug(RenderViewContext context) const;

    std::string mouseEventAt(double cursorX, double cursorY, int viewportWidth, int viewportHeight,
                             RenderViewContext context) const;

    bool handleMouseClick(const std::string& eventName);

    void joinTextureWorkers();

#ifdef _WIN32
    bool comInitialized = false;
#endif

    explicit Vehicle(const std::shared_ptr<const SharedVehicleDefinition>& definition,
                     const VehiclePlacement& configuredPlacement, double configuredModelOffsetZ,
                     ModelLoadingPolicy policy, AssetRequestManager& manager,
                     SimulationState& simulationState, SoundEngine& soundEngine,
                     openbus::rendering::ViewpointContext& soundViewpoint, bool isAiVehicle)
        : loadingPolicy(policy), variables(), soundPlayback(&soundEngine),
          modelCacheKey(definition->cacheKey), assets(&manager), placement(configuredPlacement),
          modelOffsetZ(configuredModelOffsetZ),
          wheelsFromOde(parseEnabledFlag(openbus::getEnvironment("OPENBUS_WHEELS_FROM_ODE"))) {
        if (const char* scale = openbus::getEnvironment("OPENBUS_TEXTURE_SCALE")) {
            try {
                textureScale = std::clamp(std::stod(scale), 0.25, 1.0);
            } catch (const std::exception&) {
                textureScale = 1.0;
            }
        }
        if (const char* culling = openbus::getEnvironment("OPENBUS_FRUSTUM_CULLING")) {
            frustumCulling = std::string(culling) == "1";
        }
#ifdef _WIN32
        const HRESULT comResult = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
        comInitialized = SUCCEEDED(comResult);
#endif
        VehicleConfig vehicleConfiguration = definition->vehicleConfiguration;
        selectRuntimeRegistration(vehicleConfiguration, configuredPlacement);
        odometerMetres =
            vehicleConfiguration.hasOdometerInitial
                ? std::max(0.0, vehicleConfiguration.odometerInitialKilometres) * 1000.0
                : 0.0;
        soundBank.load(soundConfigPathForVehicle(vehicleConfiguration, isAiVehicle));
        aiDeltaHeight = isAiVehicle && vehicleConfiguration.hasAiDeltaHeight
                            ? vehicleConfiguration.aiDeltaHeight
                            : 0.0;
        scripts = std::make_unique<ScriptRuntime>(
            vehicleConfiguration, variables, simulationState,
            [this, &soundEngine, &soundViewpoint](const std::string& name, const std::string& file,
                                                  double controlValue) {
                if (!soundEventsEnabled) {
                    return;
                }
                if (file.empty() && !soundBank.hasTrigger(name) && name.size() > 4 &&
                    name.compare(name.size() - 4, 4, "_off") == 0) {
                    soundBank.stop(soundEngine, name.substr(0, name.size() - 4));
                } else {
                    soundBank.trigger(soundEngine, name, file, static_cast<float>(controlValue),
                                      variables, soundViewpoint);
                }
            });
        for (const std::string& error : scripts->errors()) {
            gameLog.Log("Lua script error: " + error);
        }
        for (const std::string& name : definition->modelNumericVariables) {
            variables.declare(name);
        }
        for (const std::string& name : definition->modelStringVariables) {
            variables.declareString(name);
        }
        load(*definition);
        scripts->initialize();
        for (const std::string& error : scripts->errors()) {
            gameLog.Log("Lua initialization error: " + error);
        }
        scripts->update(false);
        if (!hasSharedGeometry) {
            spawn();
            cacheSharedDisplayLists();
        } else {
            loggedAllObjectsLoaded = true;
        }
        // Suppress sound callbacks caused by scripts settling initial state.
        soundEventsEnabled = true;
    }

    ~Vehicle();

    void applyInteriorLightMaterial(const Batch& batch,
                                    openbus::rendering::ModelMaterial& material) const;

    void drawBatch(Batch& batch, double alpha, bool forceUntextured = false,
                   const std::array<double, 3>* overrideColor = nullptr, int alphaModeOverride = -1,
                   bool forceDepthTest = false);

    void sortTransparentBatch(Batch& batch);

    void draw(RenderViewContext context, VehicleRenderPass renderPass = VehicleRenderPass::All,
              const Matrix4* groundShadowTransform = nullptr);

    bool areObjectsLoaded() const;

    bool areTexturesLoaded() const;

    bool isCaptureReady() const;

    static std::mutex& sharedDisplayCacheMutex();

    static std::unordered_map<std::string, std::vector<DisplayPart>>& sharedDisplayCache();

    bool loadSharedDisplayLists();

    void cacheSharedDisplayLists();

    static void clearSharedDisplayCache();

    void spawn();

    void ensureLoaded(RenderViewContext context, double viewDistance,
                      std::chrono::steady_clock::time_point frameDeadline);

  private:
    static std::string trim(const std::string& value);

    static std::string lower(std::string value);

    static double parseDouble(const std::string& value, double fallback);

    GLuint uploadTexture(const std::filesystem::path& path, Image image);

    GLuint uploadCompressedTexture(const std::filesystem::path& path, const gli::texture& image,
                                   bool& textureArray, std::size_t& textureArrayLayers);

    GLuint uploadCompressedDds(const std::filesystem::path& path, const CompressedDds& image);

    static std::filesystem::path findTexture(const std::filesystem::path& root,
                                             const std::string& name);

    std::shared_ptr<TextureCacheEntry> textureEntry(Batch& batch);

    void ensureAuxiliaryTexture(Batch& batch, AuxiliaryTexture& auxiliary);

    void updateFreeTexture(Batch& batch);

    void uploadDynamicTexture(const ScriptRuntime::ScriptTextureSnapshot& snapshot, GLuint& texture,
                              int& width, int& height, std::uint64_t& revision, GLenum wrapS,
                              GLenum wrapT);

    void startTextureRequest(const std::shared_ptr<TextureCacheEntry>& entry);

    void ensureTexture(Batch& batch, bool visible = true);

    void ensureMaterialTexture(MaterialTextureSource& source, bool visible = true);

    void ensureEnvironmentTexture(Batch& batch, bool visible = true);

    void drawEnvironmentMap(Batch& batch, double alpha);

    void preloadTextures();

    std::shared_future<std::shared_ptr<ParsedObj>>
    parsedObjFuture(const std::filesystem::path& path, const std::string& bundleEntry = {});

    void loadObj(const Part& part, const std::shared_ptr<ParsedObj>& parsed);

    int lodForDistance(double distance) const;

    void rebuildDisplayOrder();

    void load(const SharedVehicleDefinition& definition);
};
