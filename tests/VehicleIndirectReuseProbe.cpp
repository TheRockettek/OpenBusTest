#include "app/vehicle/Vehicle.h"
#include "ProbeTempDirectory.h"
#include "physics/BusSimulation.h"
#include "physics/World.h"
#include "render/RendererReflection.h"

#include <fstream>
#include <iostream>

struct VehicleTextureLookupProbe {
    static bool run() {
        const auto root = openbus::probe::makeProbeDirectory("openbus_vehicle_texture_lookup");
        const auto model = root / "vehicle" / "model";
        const auto texture = root / "vehicle" / "texture";
        std::filesystem::create_directories(model);
        std::filesystem::create_directories(texture / "nested");
        std::ofstream(texture / "Sprinter.png") << "fixture";
        std::ofstream(texture / "nested" / "Plate.png") << "fixture";
        const bool passed =
            Vehicle::findTexture(model, "Sprinter.png") == texture / "Sprinter.png" &&
            Vehicle::findTexture(model, "plate.PNG") == texture / "nested" / "Plate.png" &&
            Vehicle::findTexture(model, "missing.png").empty();
        openbus::probe::removeProbeDirectory(root);
        if (!passed) {
            std::cerr << "vehicle texture lookup failed for a lowercase texture directory\n";
        }
        return passed;
    }
};

namespace {

DispatchComputeProc dispatchCompute = nullptr;
unsigned int dispatchCount = 0;

void APIENTRY recordDispatch(GLuint x, GLuint y, GLuint z) {
    ++dispatchCount;
    dispatchCompute(x, y, z);
}

} // namespace

int main(int argc, char** argv) {
    if (argc == 2 && std::string(argv[1]) == "--texture-lookup") {
        return VehicleTextureLookupProbe::run() ? 0 : 1;
    }
    using namespace openbus::rendering;
    if (!glfwInit()) return 77;
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    GLFWwindow* window = glfwCreateWindow(64, 64, "Vehicle indirect reuse probe", nullptr, nullptr);
    if (!window) {
        glfwTerminate();
        return 77;
    }
    glfwMakeContextCurrent(window);
    if (!loadOpenGLFunctions() || !initializeCoreRenderer()) return 1;
    initializeComputeCulling(false, true);
    if (!indirectCullingAvailable()) return 1;

    bool passed = true;
    {
        SimulationState simulation;
        AssetRequestManager assets;
        SoundEngine sound;
        ViewpointContext soundView = ViewpointContext::PlayerExterior;
        auto definition = std::make_shared<SharedVehicleDefinition>();
        Vehicle vehicle(definition, {}, 0.0, {}, assets, simulation, sound, soundView, false, true);
        Vehicle::DisplayPart part{};
        part.viewpoint = 0;
        part.lodIndex = -1;
        part.center = {0, 0, -10};
        part.size = {2, 2, 2};
        part.radius = 1;
        Vehicle::Batch opaque;
        opaque.vertexCount = 3;
        opaque.color = {1, 0, 0};
        Vehicle::Batch transparent;
        transparent.vertexCount = 6;
        transparent.alphaMode = 2;
        transparent.baseMaterial.alphaMode = 2;
        transparent.color = {0, 1, 0};
        transparent.transmap.name = "probe_transmap";
        transparent.transmap.loadAttempted = true;
        const std::array<Vehicle::Vertex, 3> vertices = {{
            {-1, -1, -10, 0, 0, 0, 0, 0, 1},
            {0, 1, -10, 0, 0, 0, 0, 0, 1},
            {1, -1, -10, 0, 0, 0, 0, 0, 1},
        }};
        const std::array<Vehicle::Vertex, 6> transparentVertices = {{
            {-1, -1, -10, 0, 0, 0, 0, 0, 1},
            {0, 1, -10, 0, 0, 0, 0, 0, 1},
            {1, -1, -10, 0, 0, 0, 0, 0, 1},
            {-1, -1, -12, 0, 0, 0, 0, 0, 1},
            {0, 1, -12, 0, 0, 0, 0, 0, 1},
            {1, -1, -12, 0, 0, 0, 0, 0, 1},
        }};
        transparent.vertices =
            std::vector<Vehicle::Vertex>(transparentVertices.begin(), transparentVertices.end());
        for (auto* batch : {&opaque, &transparent}) {
            pglGenBuffers(1, &batch->buffer);
            pglBindBuffer(GL_ARRAY_BUFFER, batch->buffer);
            const auto* batchVertices =
                batch == &transparent ? transparentVertices.data() : vertices.data();
            const std::size_t batchVertexBytes =
                batch == &transparent ? sizeof(transparentVertices) : sizeof(vertices);
            pglBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(batchVertexBytes),
                          batchVertices, GL_STATIC_DRAW);
            part.batches.push_back(std::move(*batch));
        }
        vehicle.displayLists.push_back(std::move(part));
        vehicle.variableVisibleParts = {true};
        vehicle.loaded = true;
        auto& sortingBatch = vehicle.displayLists[0].batches[1];
        setModelViewMatrix(identityMatrix());
        const GLuint firstSortedIndexBuffer = vehicle.sortTransparentBatch(sortingBatch);
        std::array<GLuint, 6> identitySortedIndices = {};
        if (firstSortedIndexBuffer == 0 || pglGetBufferSubData == nullptr) {
            std::cerr << "Transparent sorting did not create an inspectable index buffer\n";
            passed = false;
        } else {
            pglBindBuffer(GL_ARRAY_BUFFER, firstSortedIndexBuffer);
            pglGetBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(identitySortedIndices),
                                identitySortedIndices.data());
            if (identitySortedIndices != std::array<GLuint, 6>{3, 4, 5, 0, 1, 2}) {
                std::cerr << "Transparent index buffer did not sort triangles far-to-near\n";
                passed = false;
            }
            if (vehicle.sortTransparentBatch(sortingBatch) != firstSortedIndexBuffer) {
                std::cerr << "Transparent sorting did not reuse the same-view index buffer\n";
                passed = false;
            }
            setModelViewMatrix(rotationMatrix(180, 0, 1, 0));
            const GLuint reversedSortedIndexBuffer = vehicle.sortTransparentBatch(sortingBatch);
            std::array<GLuint, 6> reversedSortedIndices = {};
            if (reversedSortedIndexBuffer == 0 ||
                reversedSortedIndexBuffer == firstSortedIndexBuffer) {
                std::cerr << "Transparent sorting did not cache a second view direction\n";
                passed = false;
            } else {
                pglBindBuffer(GL_ARRAY_BUFFER, reversedSortedIndexBuffer);
                pglGetBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(reversedSortedIndices),
                                    reversedSortedIndices.data());
                if (reversedSortedIndices != std::array<GLuint, 6>{0, 1, 2, 3, 4, 5}) {
                    std::cerr << "Transparent index buffer did not reflect the changed view\n";
                    passed = false;
                }
            }
        }
        setModelViewMatrix(identityMatrix());
        setPerspective(64, 64, 90);
        glViewport(0, 0, 64, 64);
        dispatchCompute = pglDispatchCompute;
        pglDispatchCompute = recordDispatch;
        const auto draw = [&](VehicleRenderPass pass, double threshold = 0.0) {
            vehicle.draw(ViewpointContext::PlayerExterior, pass, nullptr, threshold);
        };
        const auto expect = [&](unsigned int count, const char* message) {
            if (dispatchCount != count) {
                std::cerr << message << ": expected " << count << ", got " << dispatchCount
                          << '\n';
                passed = false;
            }
        };
        draw(VehicleRenderPass::Opaque);
        draw(VehicleRenderPass::Transparent);
        expect(1, "Opaque/transparent passes did not share GPU culling");

        draw(VehicleRenderPass::Opaque);
        setModelViewMatrix(translationMatrix({1, 0, 0}));
        draw(VehicleRenderPass::Transparent);
        expect(3, "Changed view reused stale commands");

        setModelViewMatrix(identityMatrix());
        draw(VehicleRenderPass::Opaque);
        draw(VehicleRenderPass::Transparent, 0.1);
        expect(5, "Changed projected-size threshold reused stale commands");

        draw(VehicleRenderPass::Opaque);
        vehicle.updateAnimationStates();
        draw(VehicleRenderPass::Transparent);
        expect(7, "Changed animation reused stale commands");

        draw(VehicleRenderPass::Opaque);
        vehicle.frustumCulling = false;
        draw(VehicleRenderPass::Transparent);
        expect(9, "Changed frustum setting reused stale commands");

        vehicle.frustumCulling = true;
        draw(VehicleRenderPass::Opaque);
        vehicle.prepareFrameVisibility(ViewpointContext::PlayerExterior, 64, 64, 0.0);
        draw(VehicleRenderPass::Transparent);
        expect(11, "Changed material/visibility generation reused stale commands");

        draw(VehicleRenderPass::Opaque);
        vehicle.draw(ViewpointContext::PlayerInterior, VehicleRenderPass::Transparent);
        expect(13, "Changed viewpoint reused stale commands");
        vehicle.variableVisibleParts[0] = false;
        draw(VehicleRenderPass::Opaque);
        for (const auto& batch : vehicle.displayLists[0].batches) {
            if (batch.indirectSlot != std::numeric_limits<std::size_t>::max()) {
                std::cerr << "Hidden part retained a previous indirect slot\n";
                passed = false;
            }
        }
        vehicle.variableVisibleParts[0] = true;
        draw(VehicleRenderPass::Opaque);
        if (vehicle.displayLists[0].batches[0].indirectSlot != 0 ||
            vehicle.displayLists[0].batches[1].indirectSlot != 1) {
            std::cerr << "Restored part did not receive current indirect slots\n";
            passed = false;
        }
        expect(14, "Restored part did not redispatch culling");
        {
            Vehicle other(definition, {}, 0.0, {}, assets, simulation, sound, soundView, false, true);
            other.displayLists = vehicle.displayLists;
            for (auto& batch : other.displayLists[0].batches) batch.bufferTracked = true;
            other.variableVisibleParts = {true};
            other.loaded = true;
            setModelViewMatrix(identityMatrix());
            draw(VehicleRenderPass::Opaque);
            setModelViewMatrix(translationMatrix({100, 0, 0}));
            other.draw(ViewpointContext::PlayerExterior, VehicleRenderPass::Opaque);
            if (other.displayLists[0].indirectEligible) {
                std::cerr << "Vehicle frustum broad phase retained an off-screen part\n";
                passed = false;
            }
            setModelViewMatrix(identityMatrix());
            draw(VehicleRenderPass::Transparent);
            expect(15, "Another vehicle's preparation invalidated the first vehicle's commands");
        }
        vehicle.lodThresholds = {1.0, 0.5, 0.1};
        vehicle.displayLists[0].lodIndex = 0;
        setModelViewMatrix(translationMatrix({0, 0, -20}));
        draw(VehicleRenderPass::Opaque);
        if (vehicle.displayLists[0].indirectEligible) {
            std::cerr << "Vehicle LOD broad phase retained a non-selected level\n";
            passed = false;
        }
        expect(15, "A rejected vehicle LOD dispatched compute");
        vehicle.lodThresholds.clear();
        vehicle.displayLists[0].lodIndex = -1;
        setModelViewMatrix(identityMatrix());
        resetFrameTriangleCount();
        draw(VehicleRenderPass::Opaque, 0.5);
        draw(VehicleRenderPass::Transparent, 0.5);
        if (vehicle.displayLists[0].indirectEligible ||
            !vehicle.opaquePartsScratch.empty() || !vehicle.transparentBatchesScratch.empty() ||
            frameTriangleCount() != 0) {
            std::cerr << "Undersized vehicle part was classified or submitted\n";
            passed = false;
        }
        for (const auto& batch : vehicle.displayLists[0].batches) {
            if (batch.indirectSlot != std::numeric_limits<std::size_t>::max()) {
                std::cerr << "Undersized vehicle part received an indirect slot\n";
                passed = false;
            }
        }
        expect(15, "Undersized vehicle dispatched compute");
        draw(VehicleRenderPass::Opaque);
        draw(VehicleRenderPass::Transparent);
        if (!vehicle.displayLists[0].indirectEligible || frameTriangleCount() == 0) {
            std::cerr << "Disabled cutoff did not restore vehicle submission\n";
            passed = false;
        }
        expect(16, "Disabled cutoff did not restore GPU culling");
        {
            BusConfiguration configuration{};
            configuration.mass = 12000;
            configuration.length = configuration.collisionLength = 10;
            configuration.width = configuration.collisionWidth = 2.5;
            configuration.bodyHalfLength = 5;
            configuration.bodyHalfWidth = 1.25;
            configuration.bodyHalfHeight = 1.5;
            configuration.collisionHeight = 3;
            configuration.wheelRadius = 0.5;
            configuration.wheelHalfWidth = 0.15;
            configuration.centerOfGravityHeight = 1;
            configuration.axles = {{3, 2, 2, 1.5, 1, 240000, 90000, 20000, true, false},
                                   {-3, 2, 2, 1.5, 1, 240000, 90000, 20000, false, true}};
            World world;
            BusSimulation bus(world, configuration, VehiclePlacement{{0, 0, 0}, 0});
            VehicleCamera camera;
            camera.kind = VehicleCameraKind::Reflexion;
            const std::vector<VehicleCamera> cameras = {camera};
            ReflectionRenderer reflections;
            reflections.initialize(cameras);
            bool rendered = false;
            reflections.render(
                bus, cameras, {0, 0, 64, 64},
                [](std::size_t) { return ReflectionRequirement{true, 64}; },
                [&](const BusSimulation&, std::size_t, int width, int height) {
                    rendered = true;
                    setPerspective(width, height, 90);
                    setModelViewMatrix(identityMatrix());
                    setFrameTriangleCountReflectionPass(true);
                    resetFrameReflectionTriangleCount();
                    draw(VehicleRenderPass::Opaque, 0.5);
                    draw(VehicleRenderPass::Transparent, 0.5);
                    if (!reflectionPassActive() || frameReflectionTriangleCount() != 0 ||
                        vehicle.displayLists[0].indirectEligible ||
                        !vehicle.transparentBatchesScratch.empty()) {
                        std::cerr << "Undersized part was submitted in a reflection pass\n";
                        passed = false;
                    }
                    draw(VehicleRenderPass::Opaque);
                    draw(VehicleRenderPass::Transparent);
                    if (frameReflectionTriangleCount() == 0 ||
                        vehicle.transparentBatchesScratch.empty()) {
                        std::cerr << "Disabled reflection cutoff did not restore vehicle draws\n";
                        passed = false;
                    }
                    setFrameTriangleCountReflectionPass(false);
                },
                [](int width, int height) { setPerspective(width, height, 90); });
            if (!rendered) {
                std::cerr << "Reflection fixture did not render\n";
                passed = false;
            }
        }
        pglDispatchCompute = dispatchCompute;
        glDisable(GL_DITHER);
        const auto renderPair = [&](bool gpu, int scenario) {
            initializeComputeCulling(false, gpu);
            glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
            glDepthMask(GL_TRUE);
            glClearColor(0, 0, 0, 1);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            setModelViewMatrix(identityMatrix());
            if (scenario == 3) setModelViewMatrix(translationMatrix({100, 0, 0}));
            if (scenario == 4 || scenario == 5)
                setModelViewMatrix(translationMatrix({0, 0, -15}));
            draw(VehicleRenderPass::Opaque, scenario == 6 ? 0.5 : 0.0);
            if (scenario == 1) setModelViewMatrix(translationMatrix({1, 0, 0}));
            draw(VehicleRenderPass::Transparent,
                 scenario == 2 ? 0.2 : (scenario == 6 ? 0.5 : 0.0));
            std::vector<std::uint8_t> pixels(64 * 64 * 4);
            glReadPixels(0, 0, 64, 64, GL_RGBA, GL_UNSIGNED_BYTE, pixels.data());
            return pixels;
        };
        for (int scenario : {0, 1, 2, 3, 4, 5, 6}) {
            if (scenario == 4 || scenario == 5) {
                vehicle.lodThresholds = {1.0, 0.5, 0.1};
                vehicle.displayLists[0].lodIndex = scenario == 4 ? 0 : 1;
            } else if (scenario == 6) {
                vehicle.lodThresholds.clear();
                vehicle.displayLists[0].lodIndex = -1;
            }
            const auto cpu = renderPair(false, scenario);
            const auto gpu = renderPair(true, scenario);
            if (cpu != gpu) {
                std::cerr << "Vehicle CPU/indirect pixel mismatch for scenario " << scenario
                          << '\n';
                passed = false;
            }
            bool nonBlack = false;
            for (std::size_t pixel = 0; pixel < cpu.size(); pixel += 4) {
                nonBlack |= cpu[pixel] != 0 || cpu[pixel + 1] != 0 || cpu[pixel + 2] != 0;
            }
            if (nonBlack != (scenario != 3 && scenario != 5 && scenario != 6)) {
                std::cerr << "Vehicle fixture produced unexpected visibility for scenario "
                          << scenario << '\n';
                passed = false;
            }
        }
        vehicle.lodThresholds.clear();
        vehicle.displayLists[0].lodIndex = -1;
        Vehicle::DisplayPart farPart{};
        farPart.viewpoint = 0;
        farPart.lodIndex = -1;
        farPart.center = {0, 0, -18};
        farPart.size = {2, 2, 2};
        farPart.radius = 1;
        farPart.batches.push_back(vehicle.displayLists[0].batches[1]);
        farPart.batches[0].bufferTracked = true;
        vehicle.displayLists.push_back(std::move(farPart));
        auto& changingBatch = vehicle.displayLists[0].batches[0];
        auto transparentItem = std::make_shared<ModelMaterialState>();
        transparentItem->alphaMode = 2;
        ModelMaterialState::TextureChange change;
        change.activationVariable = "probe_material";
        change.items.push_back(transparentItem);
        changingBatch.baseMaterial.textureChanges = {change};
        changingBatch.textureChanges = {change};

        for (bool gpu : {false, true}) {
            initializeComputeCulling(false, gpu);
            setModelViewMatrix(identityMatrix());
            vehicle.variables.set("probe_material", 0.0);
            vehicle.prepareFrameVisibility(ViewpointContext::PlayerExterior, 64, 64, 0.0);
            draw(VehicleRenderPass::Opaque);
            const auto preparedGeneration = vehicle.viewDepthGeneration;
            if (vehicle.opaquePartsScratch.size() != 1 ||
                vehicle.transparentBatchesScratch.size() != 2 ||
                vehicle.transparentBatchesScratch[0].part != &vehicle.displayLists[1] ||
                vehicle.transparentBatchesScratch[1].part != &vehicle.displayLists[0]) {
                std::cerr << "Initial opaque/transparent classification or sorting failed\n";
                passed = false;
            }
            draw(VehicleRenderPass::Transparent);
            if (vehicle.viewDepthGeneration != preparedGeneration) {
                std::cerr << "Same-view transparent pass did not reuse prepared lists\n";
                passed = false;
            }

            vehicle.variables.set("probe_material", 1.0);
            vehicle.prepareFrameVisibility(ViewpointContext::PlayerExterior, 64, 64, 0.0);
            draw(VehicleRenderPass::Transparent);
            if (vehicle.viewDepthGeneration == preparedGeneration || changingBatch.alphaMode != 2 ||
                !changingBatch.selectedMaterialItem ||
                vehicle.transparentBatchesScratch.size() != 3) {
                std::cerr << "Material change did not invalidate and reclassify the draw lists\n";
                passed = false;
            }

            vehicle.variables.set("probe_material", 0.0);
            vehicle.prepareFrameVisibility(ViewpointContext::PlayerExterior, 64, 64, 0.0);
            draw(VehicleRenderPass::Opaque);
            if (changingBatch.alphaMode != 0 || changingBatch.selectedMaterialItem ||
                vehicle.opaquePartsScratch.size() != 1 ||
                vehicle.transparentBatchesScratch.size() != 2) {
                std::cerr << "Restored material did not return to the opaque draw list\n";
                passed = false;
            }
            const auto forwardGeneration = vehicle.viewDepthGeneration;
            setModelViewMatrix(multiplyMatrix4(translationMatrix({0, 0, -30}),
                                               rotationMatrix(180, 0, 1, 0)));
            draw(VehicleRenderPass::Transparent);
            if (vehicle.viewDepthGeneration == forwardGeneration ||
                vehicle.transparentBatchesScratch.size() != 2 ||
                vehicle.transparentBatchesScratch[0].part != &vehicle.displayLists[0] ||
                vehicle.transparentBatchesScratch[1].part != &vehicle.displayLists[1]) {
                std::cerr << "Changed view did not rebuild transparent sorting\n";
                passed = false;
            }
        }
        setModelViewMatrix(identityMatrix());
        {
            IndirectCullingScope modelPhase;
            draw(VehicleRenderPass::Opaque);
            draw(VehicleRenderPass::Transparent);
            if (drawSelectedIndirect(false)) {
                std::cerr << "Vehicle draw leaked its indirect selection into the model phase\n";
                passed = false;
            }
        }
        if (glGetError() != GL_NO_ERROR) {
            std::cerr << "Vehicle draw produced an OpenGL error\n";
            passed = false;
        }
    }
    shutdownCoreRenderer();
    glfwDestroyWindow(window);
    glfwTerminate();
    return passed ? 0 : 1;
}
