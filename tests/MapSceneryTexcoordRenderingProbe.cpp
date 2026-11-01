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
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>

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
              "MeshNormals { 1; 0;0;-1;; 2; 3;0,0,0;, 3;0,0,0;; }\n"
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
    if (argc != 1 && !lightmap && !bumpmap && !environment && !nightmap) {
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
    GLFWwindow* window = glfwCreateWindow(lightmap || bumpmap || environment || nightmap ? 64 : 32,
                                          lightmap || bumpmap || environment || nightmap ? 64 : 32,
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
            if (lightmap) {
                runLightmapProbe(root);
            } else if (bumpmap) {
                runBumpmapProbe(root);
            } else if (environment) {
                runEnvironmentProbe(root);
            } else if (nightmap) {
                runNightmapProbe(root);
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
