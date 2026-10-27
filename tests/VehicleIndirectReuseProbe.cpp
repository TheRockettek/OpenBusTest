#include "app/vehicle/Vehicle.h"

#include <iostream>

namespace {

DispatchComputeProc dispatchCompute = nullptr;
unsigned int dispatchCount = 0;

void APIENTRY recordDispatch(GLuint x, GLuint y, GLuint z) {
    ++dispatchCount;
    dispatchCompute(x, y, z);
}

} // namespace

int main() {
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
        transparent.vertexCount = 3;
        transparent.alphaMode = 2;
        transparent.color = {0, 1, 0};
        const std::array<Vehicle::Vertex, 3> vertices = {{
            {-1, -1, -10, 0, 0, 0, 0, 0, 1},
            {0, 1, -10, 0, 0, 0, 0, 0, 1},
            {1, -1, -10, 0, 0, 0, 0, 0, 1},
        }};
        for (auto* batch : {&opaque, &transparent}) {
            pglGenBuffers(1, &batch->buffer);
            pglBindBuffer(GL_ARRAY_BUFFER, batch->buffer);
            pglBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices.data(), GL_STATIC_DRAW);
            part.batches.push_back(std::move(*batch));
        }
        vehicle.displayLists.push_back(std::move(part));
        vehicle.variableVisibleParts = {true};
        vehicle.loaded = true;
        setPerspective(64, 64, 90);
        setModelViewMatrix(identityMatrix());
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
            if (scenario >= 4) setModelViewMatrix(translationMatrix({0, 0, -15}));
            draw(VehicleRenderPass::Opaque);
            if (scenario == 1) setModelViewMatrix(translationMatrix({1, 0, 0}));
            draw(VehicleRenderPass::Transparent, scenario == 2 ? 0.2 : 0.0);
            std::vector<std::uint8_t> pixels(64 * 64 * 4);
            glReadPixels(0, 0, 64, 64, GL_RGBA, GL_UNSIGNED_BYTE, pixels.data());
            return pixels;
        };
        for (int scenario : {0, 1, 2, 3, 4, 5}) {
            if (scenario >= 4) {
                vehicle.lodThresholds = {1.0, 0.5, 0.1};
                vehicle.displayLists[0].lodIndex = scenario == 4 ? 0 : 1;
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
            if (nonBlack != (scenario != 3 && scenario != 5)) {
                std::cerr << "Vehicle fixture produced unexpected visibility for scenario "
                          << scenario << '\n';
                passed = false;
            }
        }
        vehicle.lodThresholds.clear();
        vehicle.displayLists[0].lodIndex = -1;
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
