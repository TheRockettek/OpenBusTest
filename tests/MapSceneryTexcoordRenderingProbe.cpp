#include "ProbeTempDirectory.h"
#include "core/Variables.h"
#include "render/AssetRequestManager.h"
#include "render/ComputeCulling.h"
#include "render/CoreRenderer.h"
#include "render/MatrixMath.h"
#include "render/OpenGLFunctions.h"
#include "render/map/MapRenderer.h"

#include <GLFW/glfw3.h>

#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <numbers>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

using namespace openbus::rendering;

namespace {

void check(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void writeText(const std::filesystem::path& path, const std::string& text) {
    std::ofstream output(path, std::ios::binary);
    output << text;
    check(static_cast<bool>(output), "Could not write scenery UV fixture");
}

void writeTexture(const std::filesystem::path& path) {
    std::array<std::uint8_t, 30> image = {};
    image[2] = 2;
    image[12] = image[14] = 2;
    image[16] = 24;
    // Four distinct BGR pixels sampled at the centers of the texture quadrants.
    const std::array<std::uint8_t, 12> pixels = {
        0, 0, 255, 0, 255, 0, 255, 0, 0, 0, 255, 255};
    std::copy(pixels.begin(), pixels.end(), image.begin() + 18);
    std::ofstream output(path, std::ios::binary);
    output.write(reinterpret_cast<const char*>(image.data()), image.size());
    check(static_cast<bool>(output), "Could not write scenery UV texture");
}

void writeSolidTexture(const std::filesystem::path& path,
                       const std::array<std::uint8_t, 3>& rgb) {
    std::array<std::uint8_t, 21> image = {};
    image[2] = 2;
    image[12] = image[14] = 1;
    image[16] = 24;
    image[18] = rgb[2];
    image[19] = rgb[1];
    image[20] = rgb[0];
    std::ofstream output(path, std::ios::binary);
    output.write(reinterpret_cast<const char*>(image.data()), image.size());
    check(static_cast<bool>(output), "Could not write scenery lightmap texture");
}

void writeNormalMesh(const std::filesystem::path& path, float normalY) {
    std::vector<std::uint8_t> bytes = {0x84, 0x19, 0x01};
    const auto integer = [&](std::uint32_t value, int width) {
        for (int index = 0; index < width; ++index) {
            bytes.push_back(static_cast<std::uint8_t>(value >> (index * 8)));
        }
    };
    const auto number = [&](float value) { integer(std::bit_cast<std::uint32_t>(value), 4); };
    bytes.push_back(0x17);
    integer(3, 2);
    for (const auto& position : std::array<std::array<float, 3>, 3>{
             {{{-2, 0, -2}}, {{2, 0, -2}}, {{0, 0, 2}}}}) {
        for (const float value : position)
            number(value);
        for (const float value : {0.0F, normalY, 0.0F, 0.5F, 0.5F})
            number(value);
    }
    bytes.push_back(0x49);
    integer(2, 2);
    for (const auto& triangle :
         std::array<std::array<std::uint32_t, 3>, 2>{{{{0, 1, 2}}, {{0, 2, 1}}}}) {
        for (const std::uint32_t index : triangle)
            integer(index, 2);
        integer(0, 2);
    }
    bytes.push_back(0x26);
    integer(1, 2);
    for (const float value : {1.0F, 1.0F, 1.0F, 1.0F, 0.0F, 0.0F,
                              0.0F, 0.0F, 0.0F, 0.0F, 0.0F})
        number(value);
    bytes.push_back(0);
    std::ofstream output(path, std::ios::binary);
    output.write(reinterpret_cast<const char*>(bytes.data()),
                 static_cast<std::streamsize>(bytes.size()));
    check(static_cast<bool>(output), "Could not write O3D normal fixture");
}

void writeHumanMesh(const std::filesystem::path& path) {
    std::vector<std::uint8_t> bytes = {0x84, 0x19, 0x01};
    const auto integer = [&](std::uint32_t value, int width) {
        for (int index = 0; index < width; ++index)
            bytes.push_back(static_cast<std::uint8_t>(value >> (index * 8)));
    };
    const auto number = [&](float value) { integer(std::bit_cast<std::uint32_t>(value), 4); };
    bytes.push_back(0x17);
    integer(3, 2);
    for (const auto& position : std::array<std::array<float, 3>, 3>{
             {{{-0.7F, 0, 0}}, {{0.7F, 0, 0}}, {{0, 1.8F, 0}}}}) {
        for (const float value : position)
            number(value);
        for (const float value : {0.0F, 0.0F, -1.0F, 0.0F, 0.0F})
            number(value);
    }
    bytes.push_back(0x49);
    integer(2, 2);
    for (const auto& triangle :
         std::array<std::array<std::uint32_t, 3>, 2>{{{{0, 1, 2}}, {{0, 2, 1}}}}) {
        for (const auto index : triangle)
            integer(index, 2);
        integer(0, 2);
    }
    bytes.push_back(0x26);
    integer(1, 2);
    for (const float value : {1.0F, 1.0F, 1.0F, 1.0F, 0.0F, 0.0F,
                              0.0F, 0.0F, 0.0F, 0.0F, 0.0F})
        number(value);
    bytes.push_back(0);
    bytes.push_back(0x54);
    integer(1, 2);
    bytes.push_back(4);
    for (const char character : std::string_view("body"))
        bytes.push_back(static_cast<std::uint8_t>(character));
    integer(3, 2);
    for (int index = 0; index < 3; ++index) {
        integer(index, 2);
        number(1.0F);
    }
    std::ofstream output(path, std::ios::binary);
    output.write(reinterpret_cast<const char*>(bytes.data()),
                 static_cast<std::streamsize>(bytes.size()));
    check(static_cast<bool>(output), "Could not write skinned human mesh");
}

void runHumanSkinningProbe(const std::filesystem::path& root) {
    std::filesystem::create_directories(root / "model");
    writeSolidTexture(root / "base.tga", {255, 255, 255});
    writeHumanMesh(root / "model" / "human.o3d");
    writeText(root / "model" / "human.obj",
              "v -0.7 0 0\nv 0.7 0 0\nv 0 1.8 0\n"
              "vn 0 0 -1\nf 1//1 2//1 3//1\nf 3//1 2//1 1//1\n");
    for (const auto& [name, bone] :
         std::array<std::pair<std::string_view, int>, 3>{
             {{"chest", -11}, {"leg", -3}, {"fallback", 0}}}) {
        writeText(root / "model" / (std::string(name) + ".cfg"),
                  "[mesh]\nhuman.o3d\n" +
                      (bone == 0 ? "" : "[setbone]\nbody\n" + std::to_string(bone) + "\n"));
        writeText(root / (std::string(name) + ".hum"),
                  "[model]\nmodel\\" + std::string(name) +
                      ".cfg\n[humangeom]\n0.2\n1.8\n[links]\n"
                      "0.1\n0\n0.9\n0.1\n0\n0.4\n0\n1.0\n"
                      "0.2\n0\n1.4\n0.4\n0\n1.4\n0\n1.6\n"
                      "0.6\n0\n1.4\n0.7\n0\n1.4\n"
                      "[walk_param]\n1.4\n0\n1\n0\n0\n");
    }
    writeText(root / "tile.map", "[version]\n14\n[terrain]\n");
    const std::array<std::uint8_t, 20> terrain = {1};
    std::ofstream terrainFile(root / "tile.terrain", std::ios::binary);
    terrainFile.write(reinterpret_cast<const char*>(terrain.data()), terrain.size());
    check(static_cast<bool>(terrainFile), "Could not write human test terrain");
    terrainFile.close();
    openbus::map::MapDefinition map;
    map.rootPath = root;
    map.groundTextures.push_back({"base.tga", "", {}});
    openbus::map::MapTileReference tile;
    tile.textPath = root / "tile.map";
    tile.terrainPath = root / "tile.terrain";
    tile.hasTerrainFile = true;
    map.tiles.push_back(tile);
    SimulationState simulation;
    AssetRequestManager assets;
    MapRenderer renderer(map, 0, 0, 0, 0, root, simulation, assets);
    glViewport(0, 0, 64, 64);
    glDisable(GL_DITHER);
    setPerspective(64, 64, 60);
    lookAt(0, -4, 2, 0, 0, 0.9, 0, 0, 1);
    setLightingViewMatrix(modelViewMatrix());
    const openbus::map::MapTrafficPose pose;
    const auto draw = [&](const std::filesystem::path& path, double phase, double weight) {
        glClearColor(0, 0, 0, 0);
        glDepthMask(GL_TRUE);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        resetFrameTriangleCount();
        renderer.drawHumanTrafficActor(path, pose, phase, weight);
        std::array<std::uint8_t, 64 * 64 * 4> image = {};
        glReadPixels(0, 0, 64, 64, GL_RGBA, GL_UNSIGNED_BYTE, image.data());
        return image;
    };
    const auto awaitModel = [&](const std::filesystem::path& path) {
        renderer.loadHumanTrafficModel(path);
        for (int attempt = 0; attempt < 500; ++attempt) {
            draw(path, 0, 0);
            if (frameTriangleCount() != 0)
                break;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        check(frameTriangleCount() != 0, "Asynchronous human model did not render");
    };
    for (const std::string name : {"chest", "leg", "fallback"}) {
        const auto path = root / (name + ".hum");
        awaitModel(path);
        const auto standing = draw(path, 0, 0);
        const auto walking = draw(path, std::numbers::pi / 2, 1);
        check(std::any_of(standing.begin(), standing.end(),
                          [](std::uint8_t value) { return value != 0; }),
              "Human model rendered no visible pixels");
        check(name == "chest" ? standing == walking : standing != walking,
              "Human binding did not select the expected skeletal pose");
    }
    if (const char* path = std::getenv("OPENBUS_HUMAN_SKINNING_TEST_PATH")) {
        awaitModel(path);
        const auto standing = draw(path, 0, 0);
        const auto walking = draw(path, std::numbers::pi / 2, 1);
        check(std::any_of(standing.begin(), standing.end(),
                          [](std::uint8_t value) { return value != 0; }) &&
                  std::any_of(walking.begin(), walking.end(),
                              [](std::uint8_t value) { return value != 0; }),
              "Installed human model was not visible in both poses");
    }
}

void runNormalProbe(const std::filesystem::path& root) {
    std::filesystem::create_directories(root / "model");
    writeSolidTexture(root / "base.tga", {255, 255, 255});
    writeNormalMesh(root / "model" / "up.o3d", 1.0F);
    writeNormalMesh(root / "model" / "down.o3d", -1.0F);
    writeText(root / "model" / "up.x",
              "xof 0303txt 0032\n"
              "Mesh { 3; -2;0;-2;, 2;0;-2;, 0;0;2;; 2; 3;0,1,2;, 3;2,1,0;;\n"
              "MeshNormals { 1; 0;1;0;; 2; 3;0,0,0;, 3;0,0,0;; }\n"
              "MeshMaterialList { 1; 2; 0,0;; Material { "
              "1;1;1;1;; 0; 0;0;0;; 0;0;0;; } } }\n");
    writeText(root / "up.sco", "[mesh]\nup.o3d\n");
    writeText(root / "down.sco", "[mesh]\ndown.o3d\n");
    writeText(root / "up-x.sco", "[mesh]\nup.x\n");
    writeText(root / "tile.map",
              "[version]\n14\n[terrain]\n"
              "[object]\n0\nup-x.sco\n100\n-6\n0\n5\n0\n0\n0\n0\n"
              "[object]\n0\nup.sco\n101\n-2\n0\n5\n0\n0\n0\n0\n"
              "[object]\n0\ndown.sco\n102\n2\n0\n5\n0\n0\n0\n0\n");
    const std::array<std::uint8_t, 20> terrain = {1};
    std::ofstream terrainFile(root / "tile.terrain", std::ios::binary);
    terrainFile.write(reinterpret_cast<const char*>(terrain.data()), terrain.size());
    check(static_cast<bool>(terrainFile), "Could not write O3D normal terrain");
    terrainFile.close();
    openbus::map::MapDefinition map;
    map.rootPath = root;
    map.groundTextures.push_back({"base.tga", "", {}});
    openbus::map::MapTileReference tile;
    tile.textPath = root / "tile.map";
    tile.terrainPath = root / "tile.terrain";
    tile.hasTerrainFile = true;
    map.tiles.push_back(tile);
    SimulationState simulation;
    AssetRequestManager assets;
    MapRenderer renderer(map, 0, 0, 0, 0, root, simulation, assets);
    check(renderer.renderStats().activeSceneryObjects == 3, "Normal scenery did not load");
    glViewport(0, 0, 64, 64);
    glDisable(GL_DITHER);
    setPerspective(64, 64, 90);
    setModelViewMatrix(translationMatrix({0, 0, -16}));
    setLightingViewMatrix(identityMatrix());
    glDepthMask(GL_TRUE);
    glClearColor(0, 0, 0, 0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    renderer.draw();
    std::array<std::uint8_t, 4> upX = {}, up = {}, down = {};
    glReadPixels(14, 32, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, upX.data());
    glReadPixels(26, 32, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, up.data());
    glReadPixels(38, 32, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, down.data());
    check(up[0] > 215 && down[0] >= 145 && down[0] <= 150 && up[0] > down[0] + 60,
          "O3D surface normal did not produce the expected directional lighting");
    check(upX[0] == up[0], "X and O3D upward normals did not produce matching lighting");
}

void runLightmapProbe(const std::filesystem::path& root) {
    std::filesystem::create_directories(root / "model");
    std::filesystem::create_directories(root / "script");
    writeSolidTexture(root / "base.tga", {255, 255, 255});
    writeSolidTexture(root / "light.tga", {255, 255, 255});
    writeText(root / "invalid.tga", "invalid image");
    writeText(root / "model" / "surface.x",
              "xof 0303txt 0032\n"
              "Mesh { 3; -2;0;-2;, 2;0;-2;, 0;0;2;; 2; 3;0,1,2;, 3;2,1,0;;\n"
              "MeshTextureCoords { 3; 0.5;0.5;, 0.5;0.5;, 0.5;0.5;; }\n"
              "MeshMaterialList { 1; 2; 0,0;; Material { "
              "1;1;1;1;; 0; 0;0;0;; 0;0;0;; TextureFilename { \"base.tga\"; } } } }\n");
    const std::string mesh = "[mesh]\nsurface.x\n[matl]\nbase.tga\n0\n[matl_alpha]\n2\n";
    writeText(root / "controlled.sco",
              "[script]\n1\nscript\\lamp.osc\n"
              "[varnamelist]\n1\nscript\\lamp_vars.txt\n"
              "[stringvarnamelist]\n1\nscript\\lamp_strings.txt\n" + mesh +
                  "[alphascale]\nlamp_alpha\n[matl_lightmap]\nlight.tga\nlamp_light\n");
    writeText(root / "uncontrolled.sco", mesh + "[matl_lightmap]\nlight.tga\n");
    writeText(root / "missing.sco", mesh + "[matl_lightmap]\nmissing.tga\n");
    writeText(root / "invalid.sco", mesh + "[matl_lightmap]\ninvalid.tga\n");
    writeText(root / "script" / "lamp_vars.txt", "instance_id\nlamp_alpha\nlamp_light\n");
    writeText(root / "script" / "lamp_strings.txt", "placement_id\n");
    writeText(root / "script" / "lamp.osc",
              "{init}\n(L.$.placement_id) $StrToFloat (S.L.instance_id)\n{end}\n"
              "{frame}\n"
              "(L.S.test_alpha) (L.L.instance_id) * (S.L.lamp_alpha)\n"
              "(L.S.test_light) (L.L.instance_id) * (S.L.lamp_light)\n{end}\n");
    writeText(root / "tile.map",
              "[version]\n14\n[terrain]\n"
              "[object]\n0\ncontrolled.sco\n101\n-6\n0\n5\n0\n0\n0\n1\n1\n"
              "[object]\n0\ncontrolled.sco\n102\n-2\n0\n5\n0\n0\n0\n1\n2\n"
              "[object]\n0\nuncontrolled.sco\n103\n2\n0\n5\n0\n0\n0\n0\n"
              "[object]\n0\nmissing.sco\n104\n6\n0\n5\n0\n0\n0\n0\n"
              "[object]\n0\ninvalid.sco\n105\n50\n0\n5\n0\n0\n0\n0\n");
    const std::array<std::uint8_t, 20> terrain = {1};
    std::ofstream terrainFile(root / "tile.terrain", std::ios::binary);
    terrainFile.write(reinterpret_cast<const char*>(terrain.data()), terrain.size());
    check(static_cast<bool>(terrainFile), "Could not write scenery lightmap terrain");
    terrainFile.close();
    openbus::map::MapDefinition map;
    map.rootPath = root;
    map.groundTextures.push_back({"base.tga", "", {}});
    openbus::map::MapTileReference tile;
    tile.textPath = root / "tile.map";
    tile.terrainPath = root / "tile.terrain";
    tile.hasTerrainFile = true;
    map.tiles.push_back(tile);
    SimulationState simulation;
    AssetRequestManager assets;
    MapRenderer renderer(map, 0, 0, 0, 0, root, simulation, assets);
    check(renderer.renderStats().activeSceneryObjects == 5,
          "Lightmap scenery or invalid-lightmap fallback did not load");
    glViewport(0, 0, 64, 64);
    glDisable(GL_DITHER);
    setPerspective(64, 64, 90);
    setModelViewMatrix(translationMatrix({0, 0, -16}));
    using Pixel = std::array<std::uint8_t, 4>;
    const auto draw = [&](float light, float alpha) {
        simulation.sharedVariables().set("test_light", light);
        simulation.sharedVariables().set("test_alpha", alpha);
        renderer.updateScripts();
        glDepthMask(GL_TRUE);
        glClearColor(0, 0, 0, 0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        renderer.draw();
        std::array<Pixel, 4> pixels = {};
        const std::array<int, 4> x = {14, 26, 38, 49};
        for (std::size_t index = 0; index < x.size(); ++index) {
            glReadPixels(x[index], 32, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, pixels[index].data());
        }
        return pixels;
    };
    const auto dark = draw(0, 1);
    const auto half = draw(0.5F, 1);
    const auto bright = draw(1, 1);
    if (!(dark[0][0] > 0 && dark[0][0] == dark[1][0] &&
          bright[0][0] > dark[0][0] + 30 && bright[1][0] == bright[0][0])) {
        for (const auto& pixels : {dark, half, bright}) {
            std::cerr << "Lightmap samples:";
            for (const auto& pixel : pixels) {
                std::cerr << ' ' << static_cast<int>(pixel[0]) << ',' << static_cast<int>(pixel[3]);
            }
            std::cerr << '\n';
        }
    }
    check(dark[0][0] > 0 && dark[0][0] == dark[1][0] &&
              bright[0][0] > dark[0][0] + 30 && bright[1][0] == bright[0][0],
          "Lightmap controller did not change surface illumination");
    check(half[0][0] > dark[0][0] && half[0][0] < bright[0][0] &&
              half[1][0] == bright[1][0],
          "Lightmap strength is not evaluated independently per placement");
    check(bright[2][0] == bright[0][0] && bright[3][0] == dark[0][0],
          "Uncontrolled or missing lightmap did not preserve its expected fallback");
    check(draw(-1, 1)[0] == dark[0] && draw(2, 1)[0] == bright[0],
          "Lightmap strength was not clamped");
    const auto faded = draw(1, 0.5F);
    const auto hidden = draw(1, 0);
    check(faded[0][0] < bright[0][0] && faded[1][0] == bright[1][0] &&
              hidden[0][0] < faded[0][0] && hidden[1][0] < faded[1][0],
          "Alpha scale did not fade and hide placements independently");
    check(draw(1, 2)[0] == bright[0] && draw(1, -1)[0] == hidden[0],
          "Alpha scale was not clamped");
}

void runSplineLabelsProbe(const std::filesystem::path& root) {
    std::filesystem::create_directories(root / "model");
    std::filesystem::create_directories(root / "script");
    std::filesystem::create_directories(root / "Splines");
    writeSolidTexture(root / "base.tga", {255, 255, 255});
    writeText(root / "model" / "surface.x",
              "xof 0303txt 0032\n"
              "Mesh { 3; -2;0;-2;, 2;0;-2;, 0;0;2;; 2; 3;0,1,2;, 3;2,1,0;;\n"
              "MeshTextureCoords { 3; 0.5;0.5;, 0.5;0.5;, 0.5;0.5;; }\n"
              "MeshMaterialList { 1; 2; 0,0;; Material { "
              "1;1;1;1;; 0; 0;0;0;; 0;0;0;; TextureFilename { \"base.tga\"; } } } }\n");
    writeText(root / "sign.sco",
              "[script]\n1\nscript\\sign.osc\n"
              "[varnamelist]\n1\nscript\\vars.txt\n"
              "[stringvarnamelist]\n1\nscript\\strings.txt\n"
              "[mesh]\nsurface.x\n[matl]\nbase.tga\n0\n[matl_alpha]\n2\n"
              "[alphascale]\nlamp_alpha\n");
    writeText(root / "script" / "vars.txt", "instance_id\nlamp_alpha\n");
    writeText(root / "script" / "strings.txt", "placement_id\n");
    writeText(root / "script" / "sign.osc",
              "{init}\n(L.$.placement_id) $StrToFloat (S.L.instance_id)\n{end}\n"
              "{frame}\n(L.S.test_alpha) (L.L.instance_id) * (S.L.lamp_alpha)\n{end}\n");
    writeText(root / "Splines" / "Test.sli",
              "[texture]\nbase.tga\n[profile]\n0\n[profilepnt]\n-2\n0\n0\n0\n"
              "[profilepnt]\n2\n0\n1\n0\n");
    writeText(root / "tile.map",
              "[version]\n14\n[terrain]\n"
              "[spline]\n0\nSplines\\Test.sli\n1\n0\n0\n0\n0\n-5\n0\n20\n"
              "0\n0\n0\n0\n0\n0\n0\n0\n"
              "[splineAttachement]\n0\nsign.sco\n101\n0\n-6\n5\n5\n0\n0\n0\n0\n0\n0\n1\n1\n"
              "[splineAttachement]\n0\nsign.sco\n102\n0\n-2\n5\n5\n0\n0\n0\n0\n0\n0\n1\n2\n"
              "[splineAttachement_repeater]\n0\n0\n0\nsign.sco\n103\n0\n2\n5\n5\n"
              "0\n0\n0\n0\n0\n0\n1\n1\n"
              "[splineAttachement_repeater]\n0\n0\n0\nsign.sco\n104\n0\n6\n5\n5\n"
              "0\n0\n0\n0\n0\n0\n1\n2\n");
    const std::array<std::uint8_t, 20> terrain = {1};
    std::ofstream terrainFile(root / "tile.terrain", std::ios::binary);
    terrainFile.write(reinterpret_cast<const char*>(terrain.data()), terrain.size());
    check(static_cast<bool>(terrainFile), "Could not write spline label terrain");
    terrainFile.close();
    openbus::map::MapDefinition map;
    map.rootPath = root;
    map.groundTextures.push_back({"base.tga", "", {}});
    openbus::map::MapTileReference tile;
    tile.textPath = root / "tile.map";
    tile.terrainPath = root / "tile.terrain";
    tile.hasTerrainFile = true;
    map.tiles.push_back(tile);
    SimulationState simulation;
    AssetRequestManager assets;
    MapRenderer renderer(map, 0, 0, 0, 0, root, simulation, assets);
    check(renderer.renderStats().activeSceneryObjects == 4,
          "Spline label fixture did not load all four attachments");
    glViewport(0, 0, 64, 64);
    glDisable(GL_DITHER);
    setPerspective(64, 64, 90);
    setModelViewMatrix(translationMatrix({0, 0, -16}));
    const auto draw = [&](float alpha) {
        simulation.sharedVariables().set("test_alpha", alpha);
        renderer.updateScripts();
        glDepthMask(GL_TRUE);
        glClearColor(0, 0, 0, 0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        renderer.draw();
        std::array<std::array<std::uint8_t, 4>, 4> pixels = {};
        const std::array<int, 4> positions = {14, 26, 38, 49};
        for (std::size_t index = 0; index < positions.size(); ++index) {
            glReadPixels(positions[index], 32, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE,
                         pixels[index].data());
        }
        return pixels;
    };
    const auto hidden = draw(0.0F);
    const auto half = draw(0.5F);
    const auto full = draw(1.0F);
    bool valid = true;
    for (std::size_t index = 0; index < half.size(); ++index) {
        const int baseline = hidden[index][0];
        const int intermediate = half[index][0];
        const int opaque = full[index][0];
        valid = valid && std::abs(opaque - baseline) > 30;
        if (index % 2 == 0) {
            valid = valid && std::abs(2 * intermediate - baseline - opaque) <= 3;
        } else {
            valid = valid && std::abs(intermediate - opaque) <= 1;
        }
    }
    if (!valid) {
        std::cerr << "Spline label samples:";
        for (std::size_t index = 0; index < half.size(); ++index) {
            std::cerr << ' ' << static_cast<int>(hidden[index][0]) << '/'
                      << static_cast<int>(half[index][0]) << '/' << static_cast<int>(full[index][0]);
        }
        std::cerr << '\n';
    }
    check(valid,
          "Spline attachment labels did not initialize independent regular and repeater scripts");
}

void runSplineTextProbe(const std::filesystem::path& root) {
    std::filesystem::create_directories(root / "model");
    std::filesystem::create_directories(root / "Splines");
    std::filesystem::create_directories(root / "Fonts");
    writeSolidTexture(root / "base.tga", {255, 255, 255});
    writeSolidTexture(root / "Fonts" / "glyph.tga", {255, 255, 255});
    writeText(root / "Fonts" / "spline_label.oft",
              "[newfont]\nSplineLabelFixture\nglyph.tga\nglyph.tga\n1\n0\n"
              "[char]\nX\n0\n1\n0\n");
    writeText(root / "model" / "surface.x",
              "xof 0303txt 0032\n"
              "Mesh { 3; -2;0;-2;, 2;0;-2;, 0;0;2;; 2; 3;0,1,2;, 3;2,1,0;;\n"
              "MeshTextureCoords { 3; 0.5;0.5;, 0.5;0.5;, 0.5;0.5;; }\n"
              "MeshMaterialList { 1; 2; 0,0;; Material { "
              "1;1;1;1;; 0; 0;0;0;; 0;0;0;; TextureFilename { \"base.tga\"; } } } }\n");
    writeText(root / "sign.sco",
              "[texttexture]\n0\nSplineLabelFixture\n1\n1\n0\n255\n0\n0\n"
              "[mesh]\nsurface.x\n[matl]\nbase.tga\n0\n[matl_alpha]\n2\n"
              "[usetexttexture]\n0\n");
    writeText(root / "Splines" / "Test.sli",
              "[texture]\nbase.tga\n[profile]\n0\n[profilepnt]\n-2\n0\n0\n0\n"
              "[profilepnt]\n2\n0\n1\n0\n");
    writeText(root / "tile.map",
              "[version]\n14\n[terrain]\n"
              "[spline]\n0\nSplines\\Test.sli\n1\n0\n0\n0\n0\n-5\n0\n20\n"
              "0\n0\n0\n0\n0\n0\n0\n0\n"
              "[splineAttachement]\n0\nsign.sco\n101\n0\n-6\n5\n5\n0\n0\n0\n0\n0\n0\n1\nX\n"
              "[splineAttachement]\n0\nsign.sco\n102\n0\n-2\n5\n5\n0\n0\n0\n0\n0\n0\n1\n\n"
              "[splineAttachement_repeater]\n0\n0\n0\nsign.sco\n103\n0\n2\n5\n5\n"
              "0\n0\n0\n0\n0\n0\n1\nX\n"
              "[splineAttachement_repeater]\n0\n0\n0\nsign.sco\n104\n0\n6\n5\n5\n"
              "0\n0\n0\n0\n0\n0\n1\n\n");
    const std::array<std::uint8_t, 20> terrain = {1};
    std::ofstream terrainFile(root / "tile.terrain", std::ios::binary);
    terrainFile.write(reinterpret_cast<const char*>(terrain.data()), terrain.size());
    check(static_cast<bool>(terrainFile), "Could not write spline text terrain");
    terrainFile.close();
    openbus::map::MapDefinition map;
    map.rootPath = root;
    map.groundTextures.push_back({"base.tga", "", {}});
    openbus::map::MapTileReference tile;
    tile.textPath = root / "tile.map";
    tile.terrainPath = root / "tile.terrain";
    tile.hasTerrainFile = true;
    map.tiles.push_back(tile);
    SimulationState simulation;
    AssetRequestManager assets;
    MapRenderer renderer(map, 0, 0, 0, 0, root, simulation, assets);
    check(renderer.renderStats().activeSceneryObjects == 4,
          "Spline text fixture did not load all four attachments");
    glViewport(0, 0, 64, 64);
    glDisable(GL_DITHER);
    setPerspective(64, 64, 90);
    setModelViewMatrix(translationMatrix({0, 0, -16}));
    renderer.updateScripts();
    glDepthMask(GL_TRUE);
    glClearColor(0, 0, 0, 0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    renderer.draw();
    std::array<std::array<std::uint8_t, 4>, 4> pixels = {};
    const std::array<int, 4> positions = {14, 26, 38, 49};
    for (std::size_t index = 0; index < positions.size(); ++index) {
        glReadPixels(positions[index], 32, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, pixels[index].data());
    }
    const auto isRed = [](const auto& pixel) {
        return pixel[0] > 80 && pixel[1] < pixel[0] / 3 && pixel[2] < pixel[0] / 3;
    };
    if (!isRed(pixels[0]) || isRed(pixels[1]) || !isRed(pixels[2]) || isRed(pixels[3])) {
        std::cerr << "Spline text samples:";
        for (const auto& pixel : pixels) {
            std::cerr << ' ' << static_cast<int>(pixel[0]) << ',' << static_cast<int>(pixel[1])
                      << ',' << static_cast<int>(pixel[2]) << ',' << static_cast<int>(pixel[3]);
        }
        std::cerr << '\n';
        throw std::runtime_error(
            "Spline text textures did not reflect regular and repeater placement labels");
    }
}

void runSplineOwnershipProbe(const std::filesystem::path& root) {
    std::filesystem::create_directories(root / "model");
    std::filesystem::create_directories(root / "script");
    std::filesystem::create_directories(root / "Splines");
    writeSolidTexture(root / "base.tga", {255, 255, 255});
    writeSolidTexture(root / "ground.tga", {0, 0, 0});
    writeText(root / "model" / "surface.x",
              "xof 0303txt 0032\n"
              "Mesh { 3; -2;0;-2;, 2;0;-2;, 0;0;2;; 2; 3;0,1,2;, 3;2,1,0;;\n"
              "MeshTextureCoords { 3; 0.5;0.5;, 0.5;0.5;, 0.5;0.5;; }\n"
              "MeshMaterialList { 1; 2; 0,0;; Material { "
              "1;1;1;1;; 0; 0;0;0;; 0;0;0;; TextureFilename { \"base.tga\"; } } } }\n");
    writeText(root / "row.sco",
              "[script]\n1\nscript\\row.osc\n"
              "[varnamelist]\n1\nscript\\vars.txt\n"
              "[mesh]\nsurface.x\n[matl]\nbase.tga\n0\n[matl_alpha]\n2\n"
              "[matl_nozwrite]\n[alphascale]\nopacity\n");
    writeText(root / "script" / "vars.txt", "opacity\n");
    writeText(root / "script" / "row.osc", "{init}\n0.5 (S.L.opacity)\n{end}\n");
    writeText(root / "Splines" / "Test.sli",
              "[texture]\nground.tga\n[profile]\n0\n[profilepnt]\n-2\n0\n0\n0\n"
              "[profilepnt]\n2\n0\n1\n0\n");
    writeText(root / "tile0.map",
              "[version]\n14\n[terrain]\n"
              "[spline]\n0\nSplines\\Test.sli\n10\n0\n11\n0\n0\n290\n0\n10\n"
              "0\n0\n0\n0\n0\n0\n0\n0\n"
              "[splineAttachement]\n0\nrow.sco\n101\n0\n0\n5\n5\n0\n0\n0\n"
              "10\n20\n0\n0\n");
    writeText(root / "tile1.map",
              "[version]\n14\n[terrain]\n"
              "[spline]\n0\nSplines\\Test.sli\n11\n10\n0\n0\n0\n0\n0\n20\n"
              "0\n0\n0\n0\n0\n0\n0\n10\n"
              "[splineAttachement_repeater]\n0\n0\n1\nrow.sco\n101\n0\n0\n5\n5\n"
              "0\n0\n0\n10\n20\n0\n0\n");
    const std::array<std::uint8_t, 20> terrain = {1};
    for (const auto& name : {"tile0.terrain", "tile1.terrain"}) {
        std::ofstream terrainFile(root / name, std::ios::binary);
        terrainFile.write(reinterpret_cast<const char*>(terrain.data()), terrain.size());
        check(static_cast<bool>(terrainFile), "Could not write spline ownership terrain");
    }
    openbus::map::MapDefinition map;
    map.rootPath = root;
    map.groundTextures.push_back({"ground.tga", "", {}});
    openbus::map::MapTileReference regularTile;
    regularTile.textPath = root / "tile0.map";
    regularTile.terrainPath = root / "tile0.terrain";
    regularTile.hasTerrainFile = true;
    map.tiles.push_back(regularTile);
    openbus::map::MapTileReference repeaterTile;
    repeaterTile.y = 1;
    repeaterTile.textPath = root / "tile1.map";
    repeaterTile.terrainPath = root / "tile1.terrain";
    repeaterTile.hasTerrainFile = true;
    map.tiles.push_back(repeaterTile);
    SimulationState simulation;
    AssetRequestManager assets;
    MapRenderer renderer(map, 0, 0, -600, 0, root, simulation, assets);
    const std::vector<std::pair<int, int>> regularWindow = {{0, 0}};
    const std::vector<std::pair<int, int>> bothWindow = {{0, 0}, {0, 1}};
    const std::vector<std::pair<int, int>> repeaterWindow = {{0, 1}};
    const auto activate = [&](double y, const auto& expected) {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        do {
            renderer.updateStreaming(0, y);
            if (renderer.activeTileCoordinates() == expected) {
                return;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        } while (std::chrono::steady_clock::now() < deadline);
        throw std::runtime_error("Spline ownership fixture failed to activate requested tiles");
    };
    glViewport(0, 0, 64, 64);
    glDisable(GL_DITHER);
    setPerspective(64, 64, 90);
    setModelViewMatrix(translationMatrix({0, -305, -16}));
    const auto draw = [&]() {
        renderer.updateScripts();
        glDepthMask(GL_TRUE);
        glClearColor(0, 0, 0, 0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        renderer.draw();
        std::array<std::uint8_t, 4> pixel = {};
        glReadPixels(32, 32, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, pixel.data());
        return pixel;
    };
    activate(-600, regularWindow);
    check(renderer.renderStats().activeSceneryObjects == 3,
          "Regular tile should prepare three row placements");
    const auto regular = draw();
    activate(0, bothWindow);
    check(renderer.renderStats().activeSceneryObjects == 5,
          "Both source tiles should select five row placements");
    const auto both = draw();
    activate(900, repeaterWindow);
    check(renderer.renderStats().activeSceneryObjects == 2,
          "Repeater tile should select two row placements");
    const auto repeater = draw();
    activate(0, bothWindow);
    const auto bothAgain = draw();
    activate(-600, regularWindow);
    const auto regularAgain = draw();
    if (!(regular[0] > 30 && both[0] > regular[0] + 20 &&
          std::abs(static_cast<int>(regular[0]) - static_cast<int>(repeater[0])) <= 2 &&
          bothAgain == both && regularAgain == regular)) {
        std::cerr << "Spline ownership samples (regular/both/repeater/both/regular): "
                  << static_cast<int>(regular[0]) << '/' << static_cast<int>(both[0]) << '/'
                  << static_cast<int>(repeater[0]) << '/' << static_cast<int>(bothAgain[0]) << '/'
                  << static_cast<int>(regularAgain[0]) << '\n';
        throw std::runtime_error("Spline source-tile selection did not match duplicate poses");
    }
}

void runSplineDependencyProbe(const std::filesystem::path& root, bool duplicateId) {
    std::filesystem::create_directories(root / "model");
    std::filesystem::create_directories(root / "Splines");
    writeSolidTexture(root / "base.tga", {255, 255, 255});
    writeText(root / "model" / "surface.x",
              "xof 0303txt 0032\n"
              "Mesh { 3; -2;0;-2;, 2;0;-2;, 0;0;2;; 2; 3;0,1,2;, 3;2,1,0;;\n"
              "MeshTextureCoords { 3; 0.5;0.5;, 0.5;0.5;, 0.5;0.5;; }\n"
              "MeshMaterialList { 1; 2; 0,0;; Material { "
              "1;1;1;1;; 0; 0;0;0;; 0;0;0;; TextureFilename { \"base.tga\"; } } } }\n");
    const std::string path = "[path]\n0\n0\n0\n0\n0\n3\n0\n0\n0\n3\n0\n0\n";
    writeText(root / "row.sco",
              "[new_attachment]\nmount\nattach_trans\n0\n0\n0\n" + path +
                  "[mesh]\nsurface.x\n");
    writeText(root / "child.sco", path + "[mesh]\nsurface.x\n");
    writeText(root / "control.sco", "[onlyeditor]\n" + path);
    writeText(root / "Splines" / "Test.sli",
              "[texture]\nbase.tga\n[profile]\n0\n[profilepnt]\n-2\n0\n0\n0\n"
              "[profilepnt]\n2\n0\n1\n0\n");
    writeText(root / "tile0.map",
              "[version]\n14\n[terrain]\n"
              "[spline]\n0\nSplines\\Test.sli\n10\n0\n11\n0\n0\n290\n0\n10\n"
              "0\n0\n0\n0\n0\n0\n0\n0\n"
              "[splineAttachement]\n0\nrow.sco\n101\n0\n0\n5\n5\n0\n0\n0\n"
              "10\n20\n0\n0\n"
              "[attachObj]\n0\nchild.sco\n200\n101\n0\n0\n0\n0\n0\n0\n"
              "[object]\n0\ncontrol.sco\n300\n-1000\n0\n5\n0\n0\n0\n0\n0\n");
    writeText(root / "tile1.map",
              "[version]\n14\n[terrain]\n"
              "[spline]\n0\nSplines\\Test.sli\n11\n10\n0\n0\n0\n0\n0\n20\n"
              "0\n0\n0\n0\n0\n0\n0\n10\n"
              "[splineAttachement_repeater]\n0\n0\n1\nrow.sco\n" +
                  std::string(duplicateId ? "101" : "102") +
                  "\n0\n0\n5\n5\n0\n0\n0\n10\n20\n0\n0\n");
    const std::array<std::uint8_t, 20> terrain = {1};
    for (const auto& name : {"tile0.terrain", "tile1.terrain"}) {
        std::ofstream terrainFile(root / name, std::ios::binary);
        terrainFile.write(reinterpret_cast<const char*>(terrain.data()), terrain.size());
        check(static_cast<bool>(terrainFile), "Could not write spline dependency terrain");
    }
    openbus::map::MapDefinition map;
    map.rootPath = root;
    map.groundTextures.push_back({"base.tga", "", {}});
    for (int y = 0; y < 2; ++y) {
        openbus::map::MapTileReference tile;
        tile.y = y;
        const std::string prefix = "tile" + std::to_string(y);
        tile.textPath = root / (prefix + ".map");
        tile.terrainPath = root / (prefix + ".terrain");
        tile.hasTerrainFile = true;
        map.tiles.push_back(tile);
    }
    SimulationState simulation;
    AssetRequestManager assets;
    MapRenderer renderer(map, 0, 0, -900, 0, root, simulation, assets);
    const auto waitFor = [&](double y, const std::vector<std::pair<int, int>>& window,
                             std::size_t lanes101, std::size_t lanes102) {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        do {
            renderer.updateStreaming(0, y);
            if (renderer.activeTileCoordinates() == window &&
                renderer.lanesForTimetableEntry(300, 0).size() == 1 &&
                renderer.lanesForTimetableEntry(101, 0).size() == lanes101 &&
                renderer.lanesForTimetableEntry(102, 0).size() == lanes102 &&
                renderer.lanesForTimetableEntry(200, 0).size() == (duplicateId ? 0U : 1U)) {
                return;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        } while (std::chrono::steady_clock::now() < deadline);
        throw std::runtime_error(
            "Spline dependency window or traffic lanes did not settle: 101=" +
            std::to_string(renderer.lanesForTimetableEntry(101, 0).size()) +
            " 102=" + std::to_string(renderer.lanesForTimetableEntry(102, 0).size()) +
            " 200=" + std::to_string(renderer.lanesForTimetableEntry(200, 0).size()));
    };
    const auto debugPaths = [&]() {
        const auto& segments = renderer.sceneryTrafficPathSegments();
        return std::count_if(segments.begin(), segments.end(), [](const auto& segment) {
            return segment.mapObjectId == 101 || segment.mapObjectId == 102;
        });
    };
    waitFor(-900, {}, duplicateId ? 0 : 3, 0);
    check(debugPaths() == 0, "Unloaded spline rows must not have prepared debug paths");
    waitFor(-600, {{0, 0}}, 3, 0);
    check(renderer.renderStats().activeSceneryObjects == (duplicateId ? 3U : 4U),
          "Duplicate spline parent IDs should prevent the child placement");
    check(debugPaths() == 3, "The regular rows should prepare three debug paths");
    const auto& segments = renderer.sceneryTrafficPathSegments();
    const auto child = std::find_if(segments.begin(), segments.end(), [](const auto& segment) {
        return segment.mapObjectId == 200;
    });
    check((child == segments.end()) == duplicateId,
          "Child path must be prepared only for a resolvable spline parent");
    if (!duplicateId) {
        check(!child->points.empty() && std::abs(child->points.front()[1] - 295.0) < 0.001,
              "Child must resolve to the regular attachment's first row");
    }
    waitFor(0, {{0, 0}, {0, 1}}, duplicateId ? 5 : 3, duplicateId ? 0 : 2);
    check(renderer.renderStats().activeSceneryObjects == (duplicateId ? 5U : 6U),
          "Both source tiles should retain their rows and any resolved child");
    check(debugPaths() == 5, "Both source tiles should prepare five debug paths");
    std::array<std::vector<double>, 2> starts;
    for (const auto& segment : renderer.sceneryTrafficPathSegments()) {
        if (segment.mapObjectId == 101 || segment.mapObjectId == 102) {
            check(segment.tileX == 0 && segment.tileY >= 0 && segment.tileY < 2 &&
                      !segment.points.empty(),
                  "Spline row debug path must retain its source tile and geometry");
            starts[static_cast<std::size_t>(segment.tileY)].push_back(segment.points.front()[1]);
        }
    }
    for (auto& values : starts) {
        std::sort(values.begin(), values.end());
    }
    check(starts[0].size() == 3 && starts[1].size() == 2 &&
              std::abs(starts[0][0] - 295.0) < 0.001 &&
              std::abs(starts[0][1] - starts[1][0]) < 0.001 &&
              std::abs(starts[0][2] - starts[1][1]) < 0.001,
          "Traffic debug paths must expose the two overlapping cross-tile rows");
    std::array<std::vector<double>, 2> routeStarts;
    for (const int objectId : {101, 102}) {
        for (const auto& lane : renderer.lanesForTimetableEntry(objectId, 0)) {
            const auto route = renderer.buildDrivingRoute(objectId, lane.laneSelector);
            check(route.has_value() && !route->segments.empty() &&
                      !route->segments.front().points.empty(),
                  "Every indexed row lane must produce a driving route");
            const auto& segment = route->segments.front();
            check(segment.spline.tileX == 0 && segment.spline.tileY >= 0 &&
                      segment.spline.tileY < 2,
                  "Driving route must retain the row's source tile");
            routeStarts[static_cast<std::size_t>(segment.spline.tileY)].push_back(
                segment.points.front()[1]);
        }
    }
    for (auto& values : routeStarts) {
        std::sort(values.begin(), values.end());
    }
    for (std::size_t tile = 0; tile < starts.size(); ++tile) {
        check(routeStarts[tile].size() == starts[tile].size(),
              "Driving routes must retain all prepared row paths");
        for (std::size_t index = 0; index < starts[tile].size(); ++index) {
            check(std::abs(routeStarts[tile][index] - starts[tile][index]) < 0.001,
                  "Driving route and debug path starts must coincide");
        }
    }
    waitFor(900, {{0, 1}}, duplicateId ? 5 : 3, duplicateId ? 0 : 2);
    check(renderer.renderStats().activeSceneryObjects == 2,
          "Only the repeater rows should be active in the repeater-only window");
}

void runSplineHumanDependencyProbe(const std::filesystem::path& root, bool duplicateId,
                                   bool referenced, bool repeaterFirst) {
    std::filesystem::create_directories(root / "model");
    std::filesystem::create_directories(root / "Splines");
    writeSolidTexture(root / "base.tga", {255, 255, 255});
    writeText(root / "model" / "surface.x",
              "xof 0303txt 0032\n"
              "Mesh { 3; -2;0;-2;, 2;0;-2;, 0;0;2;; 2; 3;0,1,2;, 3;2,1,0;;\n"
              "MeshMaterialList { 1; 2; 0,0;; Material { "
              "1;1;1;1;; 0; 0;0;0;; 0;0;0;; TextureFilename { \"base.tga\"; } } } }\n");
    writeText(root / "row.sco",
              "[new_attachment]\nmount\nattach_trans\n0\n0\n0\n"
              "[path]\n0\n0\n0\n0\n0\n10\n0\n0\n1\n2\n0\n0\n"
              "[mesh]\nsurface.x\n");
    writeText(root / "child.sco", "[mesh]\nsurface.x\n");
    const std::string profile =
        "[texture]\nbase.tga\n[profile]\n0\n[profilepnt]\n-2\n0\n0\n0\n"
        "[profilepnt]\n2\n0\n1\n0\n";
    writeText(root / "Splines" / "host.sli", profile + "[path]\n1\n0\n0\n0\n0\n");
    writeText(root / "Splines" / "next.sli", profile);
    writeText(root / "tile0.map",
              std::string("[version]\n14\n[terrain]\n"
                          "[spline]\n0\nSplines\\host.sli\n10\n0\n11\n0\n0\n290\n0\n10\n"
                          "0\n0\n0\n0\n0\n0\n0\n0\n"
                          "[splineAttachement]\n0\nrow.sco\n101\n0\n0\n0\n10\n0\n0\n0\n"
                          "0\n0\n0\n0\n") +
                  (referenced ? "[attachObj]\n0\nchild.sco\n200\n101\n0\n0\n0\n0\n0\n0\n" : ""));
    writeText(root / "tile1.map",
              "[version]\n14\n[terrain]\n"
              "[spline]\n0\nSplines\\next.sli\n11\n10\n0\n0\n0\n0\n0\n20\n"
              "0\n0\n0\n0\n0\n0\n0\n10\n"
              "[splineAttachement_repeater]\n0\n0\n0\nrow.sco\n" +
                  std::string(duplicateId ? "101" : "102") +
                  "\n0\n0\n0\n10\n0\n0\n0\n0\n0\n0\n0\n");
    const std::array<std::uint8_t, 20> terrain = {1};
    for (const auto& name : {"tile0.terrain", "tile1.terrain"}) {
        std::ofstream terrainFile(root / name, std::ios::binary);
        terrainFile.write(reinterpret_cast<const char*>(terrain.data()), terrain.size());
        check(static_cast<bool>(terrainFile), "Could not write pedestrian dependency terrain");
    }
    openbus::map::MapDefinition map;
    map.rootPath = root;
    map.groundTextures.push_back({"base.tga", "", {}});
    for (int y = 0; y < 2; ++y) {
        openbus::map::MapTileReference tile;
        tile.y = y;
        const std::string prefix = "tile" + std::to_string(y);
        tile.textPath = root / (prefix + ".map");
        tile.terrainPath = root / (prefix + ".terrain");
        tile.hasTerrainFile = true;
        map.tiles.push_back(tile);
    }
    SimulationState simulation;
    AssetRequestManager assets;
    MapRenderer renderer(map, 0, 0, -900, 0, root, simulation, assets);
    const auto activate = [&](double y, const std::vector<std::pair<int, int>>& window) {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        do {
            renderer.updateStreaming(0, y);
            if (renderer.initialWindowReady() && renderer.activeTileCoordinates() == window &&
                renderer.humanTrafficLaneNetworkReady()) {
                return;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        } while (std::chrono::steady_clock::now() < deadline);
        throw std::runtime_error("Pedestrian dependency window did not settle");
    };
    const auto journey = [&]() {
        return renderer.passengerPedestrianJourneyBetween({0, 298, 0}, {0, 308, 0}, 0, 0.25, 50);
    };
    const auto requireJourney = [&](bool expected, int expectedSegmentId) {
        const auto result = journey();
        check(result.has_value() == expected,
              "Pedestrian journey did not match the available attachment paths");
        if (result) {
            check(result->route && result->route->segments.size() >= 2,
                  "Pedestrian journey must cross from the spline to the attachment path");
            const auto& segment = result->route->segments[1];
            check(segment.spline.placement.splineId == expectedSegmentId &&
                      segment.spline.tileX == 0 && segment.spline.tileY == 0 &&
                      !segment.points.empty() &&
                      std::abs(segment.points.front()[1] - 300.0) < 0.001,
                  "Pedestrian attachment segment has unexpected identity or position");
        }
        check(!renderer.passengerPedestrianJourneyBetween({0, 304, 0}, {0, 308, 0}, 0, 0.25, 50),
              "Attachment-only pedestrian path is not a journey start candidate");
    };
    const auto debugRows = [&]() {
        const auto& segments = renderer.sceneryTrafficPathSegments();
        return std::count_if(segments.begin(), segments.end(), [](const auto& segment) {
            return (segment.mapObjectId == 101 || segment.mapObjectId == 102) &&
                   segment.pathTypeCode == openbus::map::MAP_SPLINE_PATH_TYPE_HUMAN;
        });
    };
    activate(-900, {});
    requireJourney(referenced && !duplicateId, 101);
    check(debugRows() == 0, "Unloaded pedestrian rows must not have prepared debug paths");
    if (repeaterFirst) {
        activate(900, {{0, 1}});
        requireJourney(true, 0);
        check(debugRows() == 1, "Repeater tile must prepare its pedestrian debug path");
        const auto& segments = renderer.sceneryTrafficPathSegments();
        const auto repeater =
            std::find_if(segments.begin(), segments.end(), [](const auto& segment) {
                return segment.mapObjectId == 101 &&
                       segment.pathTypeCode == openbus::map::MAP_SPLINE_PATH_TYPE_HUMAN;
            });
        check(repeater != segments.end() && repeater->tileX == 0 && repeater->tileY == 1,
              "Repeater debug path must retain its actual source tile");
    } else {
        activate(-600, {{0, 0}});
        requireJourney(true, referenced && !duplicateId ? 101 : 0);
        check(debugRows() == 1, "Regular tile must prepare its pedestrian debug path");
    }
    activate(0, {{0, 0}, {0, 1}});
    requireJourney(true, referenced && !duplicateId ? 101 : 0);
    check(debugRows() == 2, "Both source tiles must prepare pedestrian debug paths");
    std::array<bool, 2> sourceTiles = {};
    for (const auto& segment : renderer.sceneryTrafficPathSegments()) {
        if ((segment.mapObjectId == 101 || segment.mapObjectId == 102) &&
            segment.pathTypeCode == openbus::map::MAP_SPLINE_PATH_TYPE_HUMAN) {
            check(segment.tileX == 0 && segment.tileY >= 0 && segment.tileY < 2 &&
                      !segment.points.empty() &&
                      std::abs(segment.points.front()[1] - 300.0) < 0.001,
                  "Pedestrian debug paths must coincide and retain their source tiles");
            sourceTiles[static_cast<std::size_t>(segment.tileY)] = true;
        }
    }
    check(sourceTiles[0] && sourceTiles[1],
          "Each source tile must contribute a pedestrian debug path");
    activate(900, {{0, 1}});
    requireJourney(true, referenced && !duplicateId ? 101 : 0);
}

void runItemLightmapProbe(const std::filesystem::path& root) {
    std::filesystem::create_directories(root / "model");
    std::filesystem::create_directories(root / "script");
    writeSolidTexture(root / "base.tga", {255, 255, 255});
    writeSolidTexture(root / "blue.tga", {0, 0, 255});
    writeSolidTexture(root / "red.tga", {255, 0, 0});
    writeSolidTexture(root / "green.tga", {0, 255, 0});
    writeSolidTexture(root / "white.tga", {255, 255, 255});
    writeText(root / "invalid.tga", "invalid image");
    writeText(root / "model" / "surface.x",
              "xof 0303txt 0032\n"
              "Mesh { 3; -1.7;0;-2;, 1.7;0;-2;, 0;0;2;; 2; 3;0,1,2;, 3;2,1,0;;\n"
              "MeshTextureCoords { 3; 0.5;0.5;, 0.5;0.5;, 0.5;0.5;; }\n"
              "MeshMaterialList { 1; 2; 0,0;; Material { "
              "1;1;1;1;; 0; 0;0;0;; 0;0;0;; TextureFilename { \"base.tga\"; } } } }\n");
    const std::string script =
        "[script]\n1\nscript\\lamp.osc\n[varnamelist]\n1\nscript\\vars.txt\n"
        "[stringvarnamelist]\n1\nscript\\strings.txt\n";
    const std::string mesh = "[mesh]\nsurface.x\n[matl]\nbase.tga\n0\n";
    const std::string change = "[matl_change]\nbase.tga\n0\nselector\n";
    writeText(root / "items.sco", script + mesh + "[matl_lightmap]\nblue.tga\nbase_power\n" +
              change + "[matl_item]\n[matl_lightmap]\nred.tga\nred_power\n"
                       "[matl_item]\n[matl_lightmap]\ngreen.tga\ngreen_power\n");
    writeText(root / "uncontrolled.sco", script + mesh + change +
              "[matl_item]\n[matl_lightmap]\nwhite.tga\n");
    writeText(root / "missing.sco", script + mesh + "[matl_lightmap]\nblue.tga\nbase_power\n" +
              change + "[matl_item]\n[matl_lightmap]\nmissing.tga\nred_power\n");
    writeText(root / "invalid.sco", script + mesh + "[matl_lightmap]\nblue.tga\nbase_power\n" +
              change + "[matl_item]\n[matl_lightmap]\ninvalid.tga\nred_power\n");
    writeText(root / "reference.sco", mesh);
    writeText(root / "script" / "vars.txt",
              "instance_id\nselector\nbase_power\nred_power\ngreen_power\n");
    writeText(root / "script" / "strings.txt", "placement_id\n");
    writeText(root / "script" / "lamp.osc",
              "{init}\n(L.$.placement_id) $StrToFloat (S.L.instance_id)\n{end}\n"
              "{frame}\n"
              "(L.S.test_selector) (L.L.instance_id) * (S.L.selector)\n"
              "1 (S.L.base_power)\n"
              "(L.S.test_red) (S.L.red_power)\n"
              "(L.S.test_green) (S.L.green_power)\n{end}\n");
    writeText(root / "tile.map",
              "[version]\n14\n[terrain]\n"
              "[object]\n0\nitems.sco\n101\n-10\n0\n5\n0\n0\n0\n1\n1\n"
              "[object]\n0\nitems.sco\n102\n-6\n0\n5\n0\n0\n0\n1\n2\n"
              "[object]\n0\nuncontrolled.sco\n103\n-2\n0\n5\n0\n0\n0\n1\n1\n"
              "[object]\n0\nmissing.sco\n104\n2\n0\n5\n0\n0\n0\n1\n1\n"
              "[object]\n0\ninvalid.sco\n105\n6\n0\n5\n0\n0\n0\n1\n1\n"
              "[object]\n0\nreference.sco\n106\n10\n0\n5\n0\n0\n0\n0\n");
    const std::array<std::uint8_t, 20> terrain = {1};
    std::ofstream terrainFile(root / "tile.terrain", std::ios::binary);
    terrainFile.write(reinterpret_cast<const char*>(terrain.data()), terrain.size());
    check(static_cast<bool>(terrainFile), "Could not write scenery item-lightmap terrain");
    terrainFile.close();
    openbus::map::MapDefinition map;
    map.rootPath = root;
    map.groundTextures.push_back({"base.tga", "", {}});
    openbus::map::MapTileReference tile;
    tile.textPath = root / "tile.map";
    tile.terrainPath = root / "tile.terrain";
    tile.hasTerrainFile = true;
    map.tiles.push_back(tile);
    SimulationState simulation;
    AssetRequestManager assets;
    MapRenderer renderer(map, 0, 0, 0, 0, root, simulation, assets);
    check(renderer.renderStats().activeSceneryObjects == 6,
          "Item-lightmap scenery or optional-map fallback did not load");
    glViewport(0, 0, 64, 64);
    glDisable(GL_DITHER);
    setPerspective(64, 64, 90);
    setModelViewMatrix(translationMatrix({0, 0, -16}));
    using Pixel = std::array<std::uint8_t, 4>;
    const auto draw = [&](float selector, float red, float green) {
        simulation.sharedVariables().set("test_selector", selector);
        simulation.sharedVariables().set("test_red", red);
        simulation.sharedVariables().set("test_green", green);
        renderer.updateScripts();
        glDepthMask(GL_TRUE);
        glClearColor(0, 0, 0, 0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        renderer.draw();
        std::array<Pixel, 6> pixels = {};
        const std::array<int, 6> x = {3, 14, 26, 38, 49, 61};
        for (std::size_t index = 0; index < x.size(); ++index) {
            glReadPixels(x[index], 32, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, pixels[index].data());
        }
        return pixels;
    };
    const auto base = draw(0, 1, 1);
    const auto off = draw(1, 0, 0);
    const auto half = draw(1, 0.5F, 0.5F);
    const auto full = draw(1, 1, 1);
    check(base[0] == base[1] && base[0] == base[3] && base[0] == base[4] &&
              base[0][2] > base[0][0] && base[2] == base[5] && base[5][3] > 0,
          "Base and unlit lightmap states did not load");
    check(off[0] == base[5] && off[1] == base[5] &&
              half[0][0] > off[0][0] && half[0][0] < full[0][0] &&
              half[1][1] > off[1][1] && half[1][1] < full[1][1] &&
              full[0][0] > full[0][1] && full[1][1] > full[1][0],
          "Item masks or independent lightmap controllers did not render correctly");
    check(full[2][0] > base[2][0] && full[2][0] == full[2][1] &&
              full[3] == base[5] && full[4] == base[5],
          "Uncontrolled item or unavailable lightmap fallback failed");
    const auto independent = draw(1, 1, 0);
    check(independent[0] == full[0] && independent[1] == off[1],
          "Selected lightmap controllers are not independent");
    check(draw(1, -1, 2)[0] == off[0] && draw(1, -1, 2)[1] == full[1] &&
              draw(0, 1, 1) == base && draw(3, 1, 1)[0] == base[0],
          "Item lightmap clamping or base restoration failed");

    writeText(root / "required.sco", mesh + change +
              "[matl_item]\n[matl_lightmap]\ninvalid.tga\n"
              "[mesh]\nsurface.x\n[matl]\ninvalid.tga\n0\n");
    writeText(root / "required.map",
              "[version]\n14\n[terrain]\n"
              "[object]\n0\nrequired.sco\n201\n0\n0\n5\n0\n0\n0\n0\n");
    openbus::map::MapDefinition requiredMap = map;
    requiredMap.tiles[0].textPath = root / "required.map";
    bool requiredTextureFailed = false;
    try {
        AssetRequestManager requiredAssets;
        MapRenderer requiredRenderer(requiredMap, 0, 0, 0, 0, root, simulation, requiredAssets);
    } catch (const std::runtime_error& error) {
        requiredTextureFailed = std::string_view(error.what()).find(
                                    "Could not decode scenery texture") != std::string_view::npos;
    }
    check(requiredTextureFailed,
          "An item lightmap failure must not hide a later required texture failure");
}

void runNightmapProbe(const std::filesystem::path& root) {
    std::filesystem::create_directories(root / "model");
    std::filesystem::create_directories(root / "script");
    writeSolidTexture(root / "base.tga", {100, 100, 100});
    writeSolidTexture(root / "blue.tga", {0, 0, 255});
    writeSolidTexture(root / "red.tga", {255, 0, 0});
    writeSolidTexture(root / "green.tga", {0, 255, 0});
    writeText(root / "invalid.tga", "invalid image");
    writeText(root / "model" / "surface.x",
              "xof 0303txt 0032\n"
              "Mesh { 3; -1.7;0;-2;, 1.7;0;-2;, 0;0;2;; 2; 3;0,1,2;, 3;2,1,0;;\n"
              "MeshTextureCoords { 3; 0.5;0.5;, 0.5;0.5;, 0.5;0.5;; }\n"
              "MeshMaterialList { 1; 2; 0,0;; Material { "
              "1;1;1;1;; 0; 0;0;0;; 1;1;1;; TextureFilename { \"base.tga\"; } } } }\n");
    const std::string script =
        "[script]\n1\nscript\\lamp.osc\n[varnamelist]\n1\nscript\\vars.txt\n"
        "[stringvarnamelist]\n1\nscript\\strings.txt\n";
    const std::string mesh = "[mesh]\nsurface.x\n[matl]\nbase.tga\n0\n";
    writeText(root / "items.sco", script + mesh + "[matl_nightmap]\nblue.tga\n"
              "[matl_change]\nbase.tga\n0\nselector\n"
              "[matl_item]\n[matl_nightmap]\nred.tga\n"
              "[matl_item]\n[matl_nightmap]\ngreen.tga\n");
    writeText(root / "base.sco", script + mesh + "[matl_nightmap]\nblue.tga\n");
    writeText(root / "visible.sco", script + mesh +
              "[matl_nightmap]\nblue.tga\n[visible]\nvisible_flag\n1\n");
    writeText(root / "missing.sco", script + mesh + "[matl_nightmap]\nblue.tga\n" +
              "[matl_change]\nbase.tga\n0\nselector\n"
              "[matl_item]\n[matl_nightmap]\nmissing.tga\n");
    writeText(root / "invalid.sco", script + mesh + "[matl_nightmap]\nblue.tga\n" +
              "[matl_change]\nbase.tga\n0\nselector\n"
              "[matl_item]\n[matl_nightmap]\ninvalid.tga\n");
    writeText(root / "script" / "vars.txt", "instance_id\nselector\nvisible_flag\n");
    writeText(root / "script" / "strings.txt", "placement_id\n");
    writeText(root / "script" / "lamp.osc",
              "{init}\n(L.$.placement_id) $StrToFloat (S.L.instance_id)\n{end}\n"
              "{frame}\n"
              "(L.S.test_selector) (L.L.instance_id) * (S.L.selector)\n"
              "(L.S.test_night) (S.L.nightlighta)\n"
              "1 (S.L.visible_flag)\n{end}\n");
    writeText(root / "tile.map",
              "[version]\n14\n[terrain]\n"
              "[object]\n0\nitems.sco\n101\n-10\n0\n5\n0\n0\n0\n1\n1\n"
              "[object]\n0\nitems.sco\n102\n-6\n0\n5\n0\n0\n0\n1\n2\n"
              "[object]\n0\nbase.sco\n103\n-2\n0\n5\n0\n0\n0\n0\n"
              "[object]\n0\nvisible.sco\n104\n2\n0\n5\n0\n0\n0\n0\n"
              "[object]\n0\nmissing.sco\n105\n6\n0\n5\n0\n0\n0\n1\n1\n"
              "[object]\n0\ninvalid.sco\n106\n10\n0\n5\n0\n0\n0\n1\n1\n");
    const std::array<std::uint8_t, 20> terrain = {1};
    std::ofstream terrainFile(root / "tile.terrain", std::ios::binary);
    terrainFile.write(reinterpret_cast<const char*>(terrain.data()), terrain.size());
    check(static_cast<bool>(terrainFile), "Could not write scenery nightmap terrain");
    terrainFile.close();
    openbus::map::MapDefinition map;
    map.rootPath = root;
    map.groundTextures.push_back({"base.tga", "", {}});
    openbus::map::MapTileReference tile;
    tile.textPath = root / "tile.map";
    tile.terrainPath = root / "tile.terrain";
    tile.hasTerrainFile = true;
    map.tiles.push_back(tile);
    SimulationState simulation;
    AssetRequestManager assets;
    MapRenderer renderer(map, 0, 0, 0, 0, root, simulation, assets);
    check(renderer.renderStats().activeSceneryObjects == 6,
          "Nightmap scenery or optional-map fallback did not load");
    glViewport(0, 0, 64, 64);
    glDisable(GL_DITHER);
    setPerspective(64, 64, 90);
    setModelViewMatrix(translationMatrix({0, 0, -16}));
    using Pixel = std::array<std::uint8_t, 4>;
    const auto draw = [&](float selector, float night) {
        simulation.sharedVariables().set("test_selector", selector);
        simulation.sharedVariables().set("test_night", night);
        renderer.updateScripts();
        glDepthMask(GL_TRUE);
        glClearColor(0, 0, 0, 0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        renderer.draw();
        std::array<Pixel, 6> pixels = {};
        const std::array<int, 6> x = {3, 14, 26, 38, 49, 61};
        for (std::size_t index = 0; index < x.size(); ++index) {
            glReadPixels(x[index], 32, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, pixels[index].data());
        }
        return pixels;
    };
    const auto day = draw(0, 0);
    const auto half = draw(0, 0.5F);
    const auto night = draw(0, 1);
    const auto selected = draw(1, 0);
    const auto selectedNight = draw(1, 1);
    const auto secondItem = draw(2, 0);
    check(day[0] == day[1] && day[0] == day[2] && day[0] == day[4] &&
              day[0] == day[5] && day[0][3] > 0,
          "Nightmap base materials did not match in daylight");
    check(night[0] == night[1] && night[0] == night[2] &&
              night[0][2] > 250 && night[0][0] < 3 &&
              half[2][2] > day[2][2] && half[2][2] < night[2][2],
          "Base nightmap did not follow its per-placement controller");
    check(day[3] == night[3] && day[3][2] > 250,
          "Visible-gated nightmap did not illuminate in daylight");
    check(selected[0][0] > 250 && selected[0][1] < 3 &&
              selected[1][1] > 250 && selected[1][0] < 3 &&
              selected[4] == day[4] && selected[5] == day[5],
          "Item selection, per-placement texture binding, or optional fallback failed");
    check(night[4] == night[2] && night[5] == night[2] &&
              selectedNight[4] == day[4] && selectedNight[5] == day[5] &&
              secondItem[0][1] > 250 && secondItem[1] == day[1],
          "Unavailable or out-of-range items did not clear or restore the base nightmap");
    check(draw(0, 0) == day && draw(0, -1)[2] == day[2] &&
              draw(0, 2)[2] == night[2],
          "Nightmap state was not restored or controller clamping failed");
}

void runBumpmapProbe(const std::filesystem::path& root) {
    std::filesystem::create_directories(root / "model");
    writeSolidTexture(root / "base.tga", {255, 255, 255});
    writeSolidTexture(root / "bump.tga", {128, 220, 220});
    writeSolidTexture(root / "alternate.tga", {35, 128, 220});
    writeText(root / "invalid.tga", "invalid image");
    writeText(root / "model" / "surface.x",
              "xof 0303txt 0032\n"
              "Mesh { 3; -1.2;0;-2;, 1.2;0;-2;, 0;0;2;; 2; 3;0,1,2;, 3;2,1,0;;\n"
              "MeshNormals { 1; 0;1;0;; 2; 3;0,0,0;, 3;0,0,0;; }\n"
              "MeshTextureCoords { 3; 0;0;, 1;0;, 0.5;1;; }\n"
              "MeshMaterialList { 1; 2; 0,0;; Material { "
              "1;1;1;1;; 0; 0;0;0;; 0;0;0;; TextureFilename { \"base.tga\"; } } } }\n");
    const std::string mesh = "[mesh]\nsurface.x\n[matl]\nbase.tga\n0\n";
    writeText(root / "base.sco", mesh);
    writeText(root / "zero.sco", mesh + "[matl_bumpmap]\nbump.tga\n0\n");
    writeText(root / "half.sco", mesh + "[matl_bumpmap]\nbump.tga\n0.5\n");
    writeText(root / "full.sco", mesh + "[matl_bumpmap]\nbump.tga\n1\n");
    writeText(root / "high.sco", mesh + "[matl_bumpmap]\nbump.tga\n2\n");
    writeText(root / "negative.sco", mesh + "[matl_bumpmap]\nbump.tga\n-1\n");
    writeText(root / "missing.sco", mesh + "[matl_bumpmap]\nmissing.tga\n1\n");
    writeText(root / "invalid.sco", mesh + "[matl_bumpmap]\ninvalid.tga\n1\n");
    writeText(root / "alternate.sco", mesh + "[matl_bumpmap]\nalternate.tga\n1\n");
    writeText(root / "tile.map",
              "[version]\n14\n[terrain]\n"
              "[object]\n0\nbase.sco\n101\n-10\n0\n5\n0\n0\n0\n0\n"
              "[object]\n0\nzero.sco\n102\n-7.5\n0\n5\n0\n0\n0\n0\n"
              "[object]\n0\nhalf.sco\n103\n-5\n0\n5\n0\n0\n0\n0\n"
              "[object]\n0\nfull.sco\n104\n-2.5\n0\n5\n0\n0\n0\n0\n"
              "[object]\n0\nhigh.sco\n105\n0\n0\n5\n0\n0\n0\n0\n"
              "[object]\n0\nnegative.sco\n106\n2.5\n0\n5\n0\n0\n0\n0\n"
              "[object]\n0\nmissing.sco\n107\n5\n0\n5\n0\n0\n0\n0\n"
              "[object]\n0\ninvalid.sco\n108\n7.5\n0\n5\n0\n0\n0\n0\n"
              "[object]\n0\nalternate.sco\n109\n10\n0\n5\n0\n0\n0\n0\n");
    const std::array<std::uint8_t, 20> terrain = {1};
    std::ofstream terrainFile(root / "tile.terrain", std::ios::binary);
    terrainFile.write(reinterpret_cast<const char*>(terrain.data()), terrain.size());
    check(static_cast<bool>(terrainFile), "Could not write scenery bumpmap terrain");
    terrainFile.close();
    openbus::map::MapDefinition map;
    map.rootPath = root;
    map.groundTextures.push_back({"base.tga", "", {}});
    openbus::map::MapTileReference tile;
    tile.textPath = root / "tile.map";
    tile.terrainPath = root / "tile.terrain";
    tile.hasTerrainFile = true;
    map.tiles.push_back(tile);
    SimulationState simulation;
    AssetRequestManager assets;
    MapRenderer renderer(map, 0, 0, 0, 0, root, simulation, assets);
    check(renderer.renderStats().activeSceneryObjects == 9,
          "Bumpmap scenery or invalid-bumpmap fallback did not load");
    glViewport(0, 0, 64, 64);
    glDisable(GL_DITHER);
    setPerspective(64, 64, 90);
    setModelViewMatrix(translationMatrix({0, 0, -16}));
    glDepthMask(GL_TRUE);
    glClearColor(0, 0, 0, 0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    renderer.draw();
    using Pixel = std::array<std::uint8_t, 4>;
    std::array<Pixel, 9> pixels = {};
    const std::array<int, 9> x = {3, 10, 17, 25, 32, 39, 46, 54, 61};
    for (std::size_t index = 0; index < x.size(); ++index) {
        glReadPixels(x[index], 32, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, pixels[index].data());
    }
    if (!(pixels[0][0] > pixels[2][0] && pixels[2][0] > pixels[3][0]) ||
        pixels[3] == pixels[8]) {
        std::cerr << "Bumpmap samples:";
        for (const auto& pixel : pixels) {
            std::cerr << ' ' << static_cast<int>(pixel[0]) << ',' << static_cast<int>(pixel[3]);
        }
        std::cerr << '\n';
    }
    check(pixels[0][0] > pixels[2][0] && pixels[2][0] > pixels[3][0],
          "Bumpmap strength did not change surface shading");
    check(pixels[0] == pixels[1] && pixels[0] == pixels[5] && pixels[0] == pixels[6] &&
              pixels[0] == pixels[7] && pixels[3] == pixels[4],
          "Bumpmap strength clamping or invalid/missing texture fallback failed");
    check(pixels[3] != pixels[8], "Different bumpmap normals produced the same shading");

    writeText(root / "required.sco",
              mesh + "[matl_bumpmap]\ninvalid.tga\n1\n"
                     "[mesh]\nsurface.x\n[matl]\ninvalid.tga\n0\n");
    writeText(root / "required.map",
              "[version]\n14\n[terrain]\n"
              "[object]\n0\nrequired.sco\n201\n0\n0\n5\n0\n0\n0\n0\n");
    openbus::map::MapDefinition requiredMap = map;
    requiredMap.tiles[0].textPath = root / "required.map";
    bool requiredTextureFailed = false;
    try {
        AssetRequestManager requiredAssets;
        MapRenderer requiredRenderer(requiredMap, 0, 0, 0, 0, root, simulation, requiredAssets);
    } catch (const std::runtime_error& error) {
        requiredTextureFailed = std::string_view(error.what()).find(
                                    "Could not decode scenery texture") != std::string_view::npos;
    }
    check(requiredTextureFailed,
          "A failed optional map must not hide a later required base texture failure");
}

void runEnvironmentProbe(const std::filesystem::path& root) {
    std::filesystem::create_directories(root / "model");
    writeSolidTexture(root / "base.tga", {100, 100, 100});
    writeSolidTexture(root / "environment.tga", {255, 0, 0});
    writeText(root / "invalid.tga", "invalid image");
    writeText(root / "model" / "surface.x",
              "xof 0303txt 0032\n"
              "Mesh { 3; -1.2;0;-2;, 1.2;0;-2;, 0;0;2;; 2; 3;0,1,2;, 3;2,1,0;;\n"
              "MeshNormals { 1; 0;0;-1;; 2; 3;0,0,0;, 3;0,0,0;; }\n"
              "MeshTextureCoords { 3; 0;0;, 1;0;, 0.5;1;; }\n"
              "MeshMaterialList { 1; 2; 0,0;; Material { "
              "1;1;1;1;; 0; 0;0;0;; 1;1;1;; TextureFilename { \"base.tga\"; } } } }\n");
    const std::string mesh = "[mesh]\nsurface.x\n[matl]\nbase.tga\n0\n";
    writeText(root / "base.sco", mesh);
    writeText(root / "zero.sco", mesh + "[matl_envmap]\nenvironment.tga\n0\n");
    writeText(root / "half.sco", mesh + "[matl_envmap]\nenvironment.tga\n0.5\n");
    writeText(root / "one.sco", mesh + "[matl_envmap]\nenvironment.tga\n1\n");
    writeText(root / "full.sco", mesh + "[matl_envmap]\nenvironment.tga\n10\n");
    writeText(root / "high.sco", mesh + "[matl_envmap]\nenvironment.tga\n20\n");
    writeText(root / "negative.sco", mesh + "[matl_envmap]\nenvironment.tga\n-1\n");
    writeText(root / "missing.sco", mesh + "[matl_envmap]\nmissing.tga\n1\n");
    writeText(root / "invalid.sco", mesh + "[matl_envmap]\ninvalid.tga\n1\n");
    writeText(root / "tile.map",
              "[version]\n14\n[terrain]\n"
              "[object]\n0\nbase.sco\n101\n-10\n0\n5\n0\n0\n0\n0\n"
              "[object]\n0\nzero.sco\n102\n-7.5\n0\n5\n0\n0\n0\n0\n"
              "[object]\n0\nhalf.sco\n103\n-5\n0\n5\n0\n0\n0\n0\n"
              "[object]\n0\none.sco\n104\n-2.5\n0\n5\n0\n0\n0\n0\n"
              "[object]\n0\nfull.sco\n105\n0\n0\n5\n0\n0\n0\n0\n"
              "[object]\n0\nhigh.sco\n106\n2.5\n0\n5\n0\n0\n0\n0\n"
              "[object]\n0\nnegative.sco\n107\n5\n0\n5\n0\n0\n0\n0\n"
              "[object]\n0\nmissing.sco\n108\n7.5\n0\n5\n0\n0\n0\n0\n"
              "[object]\n0\ninvalid.sco\n109\n10\n0\n5\n0\n0\n0\n0\n");
    const std::array<std::uint8_t, 20> terrain = {1};
    std::ofstream terrainFile(root / "tile.terrain", std::ios::binary);
    terrainFile.write(reinterpret_cast<const char*>(terrain.data()), terrain.size());
    check(static_cast<bool>(terrainFile), "Could not write scenery environment terrain");
    terrainFile.close();
    openbus::map::MapDefinition map;
    map.rootPath = root;
    map.groundTextures.push_back({"base.tga", "", {}});
    openbus::map::MapTileReference tile;
    tile.textPath = root / "tile.map";
    tile.terrainPath = root / "tile.terrain";
    tile.hasTerrainFile = true;
    map.tiles.push_back(tile);
    SimulationState simulation;
    AssetRequestManager assets;
    MapRenderer renderer(map, 0, 0, 0, 0, root, simulation, assets);
    check(renderer.renderStats().activeSceneryObjects == 9,
          "Environment scenery or optional-map fallback did not load");
    glViewport(0, 0, 64, 64);
    glDisable(GL_DITHER);
    setPerspective(64, 64, 90);
    setModelViewMatrix(translationMatrix({0, 0, -16}));
    glDepthMask(GL_TRUE);
    glClearColor(0, 0, 0, 0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    renderer.draw();
    using Pixel = std::array<std::uint8_t, 4>;
    std::array<Pixel, 9> pixels = {};
    const std::array<int, 9> x = {3, 10, 17, 25, 32, 39, 46, 54, 61};
    for (std::size_t index = 0; index < x.size(); ++index) {
        glReadPixels(x[index], 32, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, pixels[index].data());
    }
    const auto close = [](int actual, int expected) { return std::abs(actual - expected) <= 2; };
    if (!(close(pixels[2][0], 108) && close(pixels[3][0], 116) &&
          pixels[4][0] > 250 && pixels[4][1] < 3 &&
          pixels[0] == pixels[1] && pixels[0] == pixels[6] &&
          pixels[0] == pixels[7] && pixels[0] == pixels[8] && pixels[4] == pixels[5])) {
        std::cerr << "Environment samples:";
        for (const auto& pixel : pixels) {
            std::cerr << ' ' << static_cast<int>(pixel[0]) << ',' << static_cast<int>(pixel[1])
                      << ',' << static_cast<int>(pixel[3]);
        }
        std::cerr << '\n';
        throw std::runtime_error("Environment strength, clamping, or optional fallback failed");
    }
}

void runProbe(const std::filesystem::path& root) {
    std::filesystem::create_directories(root / "model");
    std::filesystem::create_directories(root / "script");
    writeTexture(root / "uv.tga");
    writeText(root / "model" / "surface.x",
              "xof 0303txt 0032\n"
              "Mesh { 3; -2;0;-2;, 2;0;-2;, 0;0;2;; 2; 3;0,1,2;, 3;2,1,0;;\n"
              "MeshTextureCoords { 3; 0.25;0.25;, 0.25;0.25;, 0.25;0.25;; }\n"
              "MeshMaterialList { 1; 2; 0,0;; Material { "
              "1;1;1;1;; 0; 0;0;0;; 1;1;1;; TextureFilename { \"uv.tga\"; } } } }\n");
    writeText(root / "scroll.sco",
              "[script]\n1\nscript\\scroll.osc\n"
              "[varnamelist]\n1\nscript\\scroll_varlist.txt\n"
              "[stringvarnamelist]\n1\nscript\\scroll_strings.txt\n"
              "[mesh]\nsurface.x\n[matl]\nuv.tga\n0\n"
              "[texcoordtransX]\nscroll_x\n[texcoordtransY]\nscroll_y\n");
    writeText(root / "script" / "scroll_varlist.txt", "instance_id\nscroll_x\nscroll_y\n");
    writeText(root / "script" / "scroll_strings.txt", "placement_id\n");
    writeText(root / "script" / "scroll.osc",
              "{init}\n(L.$.placement_id) $StrToFloat (S.L.instance_id)\n{end}\n"
              "{frame}\n"
              "(L.S.uv_test_x) (L.L.instance_id) * (S.L.scroll_x)\n"
              "(L.S.uv_test_y) (L.L.instance_id) * (S.L.scroll_y)\n{end}\n");
    writeText(root / "tile.map",
              "[version]\n14\n[terrain]\n"
              "[object]\n0\nscroll.sco\n101\n-3\n0\n5\n0\n0\n0\n1\n1\n"
              "[object]\n0\nscroll.sco\n102\n3\n0\n5\n0\n0\n0\n1\n2\n");
    const std::array<std::uint8_t, 20> terrain = {1};
    std::ofstream terrainFile(root / "tile.terrain", std::ios::binary);
    terrainFile.write(reinterpret_cast<const char*>(terrain.data()), terrain.size());
    check(static_cast<bool>(terrainFile), "Could not write scenery UV terrain");
    terrainFile.close();

    openbus::map::MapDefinition map;
    map.rootPath = root;
    map.groundTextures.push_back({"uv.tga", "", {}});
    openbus::map::MapTileReference tile;
    tile.textPath = root / "tile.map";
    tile.terrainPath = root / "tile.terrain";
    tile.hasTerrainFile = true;
    map.tiles.push_back(tile);
    SimulationState simulation;
    AssetRequestManager assets;
    MapRenderer renderer(map, 0, 0, 0, 0, root, simulation, assets);
    check(renderer.renderStats().activeSceneryObjects == 2, "UV scenery did not load");
    glViewport(0, 0, 32, 32);
    glDisable(GL_DITHER);
    setPerspective(32, 32, 90);
    setModelViewMatrix(translationMatrix({0, 0, -12}));

    using Pixel = std::array<std::uint8_t, 4>;
    const auto draw = [&](float x, float y) {
        simulation.sharedVariables().set("uv_test_x", x);
        simulation.sharedVariables().set("uv_test_y", y);
        renderer.updateScripts();
        glDepthMask(GL_TRUE);
        glClearColor(0, 0, 0, 0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        renderer.draw();
        std::array<Pixel, 2> result = {};
        glReadPixels(9, 16, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, result[0].data());
        glReadPixels(22, 16, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, result[1].data());
        return result;
    };
    const auto base = draw(0, 0);
    const auto horizontal = draw(0.5F, 0);
    const auto vertical = draw(0, 0.5F);
    if (base[0] != base[1] || base[0][3] == 0) {
        std::cerr << "Base samples: ";
        for (const auto& pixel : base) {
            std::cerr << static_cast<int>(pixel[0]) << ',' << static_cast<int>(pixel[1])
                      << ',' << static_cast<int>(pixel[2]) << ',' << static_cast<int>(pixel[3])
                      << ' ';
        }
        std::cerr << '\n';
    }
    check(base[0] == base[1] && base[0][3] > 0, "Base UV samples do not match");
    check((horizontal[0] == base[0]) != (horizontal[1] == base[1]),
          "X translation did not vary independently per placement");
    check((vertical[0] == base[0]) != (vertical[1] == base[1]),
          "Y translation did not vary independently per placement");
    check((horizontal[0] == base[0]) == (vertical[0] == base[0]),
          "Different placements responded to X and Y translation");
    const Pixel xChanged = horizontal[0] == base[0] ? horizontal[1] : horizontal[0];
    const Pixel yChanged = vertical[0] == base[0] ? vertical[1] : vertical[0];
    check(base[0][0] < 15 && base[0][1] < 15 && base[0][2] > 240 &&
              xChanged[0] > 240 && xChanged[1] > 240 && xChanged[2] < 15 &&
              yChanged[0] > 240 && yChanged[1] < 15 && yChanged[2] < 15,
          "UV translation sampled the wrong texture quadrant");
    check(draw(0, 0) == base, "UV translation did not reset");
}

} // namespace

int main(int argc, char** argv) {
    const bool lightmap = argc == 2 && std::string_view(argv[1]) == "--lightmap";
    const bool bumpmap = argc == 2 && std::string_view(argv[1]) == "--bumpmap";
    const bool environment = argc == 2 && std::string_view(argv[1]) == "--envmap";
    const bool nightmap = argc == 2 && std::string_view(argv[1]) == "--nightmap";
    const bool itemLightmap = argc == 2 && std::string_view(argv[1]) == "--item-lightmap";
    const bool normals = argc == 2 && std::string_view(argv[1]) == "--normals";
    const bool human = argc == 2 && std::string_view(argv[1]) == "--human-skinning";
    const bool splineLabels = argc == 2 && std::string_view(argv[1]) == "--spline-labels";
    const bool splineText = argc == 2 && std::string_view(argv[1]) == "--spline-text";
    const bool splineOwnership = argc == 2 && std::string_view(argv[1]) == "--spline-ownership";
    const bool splineDependencies =
        argc == 2 && std::string_view(argv[1]) == "--spline-dependencies";
    const bool splineHumanDependencies =
        argc == 2 && std::string_view(argv[1]) == "--spline-human-dependencies";
    if (argc != 1 && !lightmap && !bumpmap && !environment && !nightmap && !itemLightmap &&
        !normals && !human && !splineLabels && !splineText && !splineOwnership &&
        !splineDependencies && !splineHumanDependencies) {
        return 2;
    }
    if (!glfwInit()) {
        return 77;
    }
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    const char* setting = std::getenv("OPENBUS_CULLING_BACKEND");
    const bool indirect = setting != nullptr && std::string_view(setting) == "compute-indirect";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, indirect ? 4 : 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    GLFWwindow* window = glfwCreateWindow(
        lightmap || bumpmap || environment || nightmap || itemLightmap || normals || human ||
                splineLabels || splineText || splineOwnership ? 64 : 32,
        lightmap || bumpmap || environment || nightmap || itemLightmap || normals || human ||
                splineLabels || splineText || splineOwnership ? 64 : 32,
                                          "Scenery material probe", nullptr, nullptr);
    if (window == nullptr) {
        glfwTerminate();
        return 77;
    }
    glfwMakeContextCurrent(window);
    const auto root = openbus::probe::makeProbeDirectory("openbus-scenery-uv-rendering");
    bool passed = loadOpenGLFunctions() && initializeCoreRenderer();
    const bool unavailable = passed && indirect && !indirectCullingAvailable();
    if (passed && !unavailable) {
        try {
            if (human) {
                runHumanSkinningProbe(root);
            } else if (normals) {
                runNormalProbe(root);
            } else if (lightmap) {
                runLightmapProbe(root);
            } else if (bumpmap) {
                runBumpmapProbe(root);
            } else if (environment) {
                runEnvironmentProbe(root);
            } else if (nightmap) {
                runNightmapProbe(root);
            } else if (itemLightmap) {
                runItemLightmapProbe(root);
            } else if (splineLabels) {
                runSplineLabelsProbe(root);
            } else if (splineText) {
                runSplineTextProbe(root);
            } else if (splineOwnership) {
                runSplineOwnershipProbe(root);
            } else if (splineDependencies) {
                runSplineDependencyProbe(root / "unique", false);
                runSplineDependencyProbe(root / "duplicate", true);
            } else if (splineHumanDependencies) {
                runSplineHumanDependencyProbe(root / "unique", false, true, false);
                runSplineHumanDependencyProbe(root / "unreferenced", false, false, false);
                runSplineHumanDependencyProbe(root / "duplicate", true, true, false);
                runSplineHumanDependencyProbe(root / "repeater-first", true, true, true);
            } else {
                runProbe(root);
            }
        } catch (const std::exception& error) {
            std::cerr << error.what() << '\n';
            passed = false;
        }
    }
    shutdownCoreRenderer();
    glfwDestroyWindow(window);
    glfwTerminate();
    openbus::probe::removeProbeDirectory(root);
    return unavailable ? 77 : passed ? 0 : 1;
}
