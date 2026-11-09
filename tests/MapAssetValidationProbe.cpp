#include "map/MapAssetValidation.h"
#include "map/MapBakeManifest.h"
#include "map/MapCarparkList.h"
#include "map/MapConfigLoader.h"
#include "assets/ObjLoader.h"
#include "core/Utf8Path.h"
#include "ProbeTempDirectory.h"

#include <algorithm>
#include <array>
#include <bit>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

void write(const std::filesystem::path& path, const std::string& content) {
    std::filesystem::create_directories(path.parent_path());
    std::ofstream output(path, std::ios::binary);
    if (!(output << content)) {
        throw std::runtime_error("could not write fixture");
    }
}

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

bool hasFinding(const openbus::map::MapAssetValidationReport& report, const std::string& category,
                const std::string& reference) {
    return std::any_of(report.findings.begin(), report.findings.end(), [&](const auto& finding) {
        return finding.category == category &&
               finding.reference.find(reference) != std::string::npos;
    });
}

std::string xMesh(const std::string& texture) {
    return "xof 0303txt 0032\nMaterial Surface { 1;1;1;1;; 8; 0;0;0;; 0;0;0;; "
           "TextureFilename { \"" +
           texture +
           "\"; } }\nMesh { 3; 0;0;0;, 1;0;0;, 0;1;0;; 1; 3;0,1,2;; "
           "MeshMaterialList { 1; 1; 0;; { Surface } } }\n";
}

std::string pcmWav() {
    std::string bytes = "RIFF";
    const auto append16 = [&](std::uint16_t value) {
        bytes.push_back(static_cast<char>(value & 0xffU));
        bytes.push_back(static_cast<char>(value >> 8));
    };
    const auto append32 = [&](std::uint32_t value) {
        for (unsigned shift = 0; shift < 32; shift += 8) {
            bytes.push_back(static_cast<char>((value >> shift) & 0xffU));
        }
    };
    append32(38);
    bytes.append("WAVEfmt ", 8);
    append32(16);
    append16(1);
    append16(1);
    append32(44100);
    append32(88200);
    append16(2);
    append16(16);
    bytes.append("data", 4);
    append32(2);
    append16(0);
    return bytes;
}

std::string onePixelBmp() {
    std::string bytes = "BM";
    const auto append16 = [&](std::uint16_t value) {
        bytes.push_back(static_cast<char>(value & 0xffU));
        bytes.push_back(static_cast<char>(value >> 8));
    };
    const auto append32 = [&](std::uint32_t value) {
        for (unsigned shift = 0; shift < 32; shift += 8) {
            bytes.push_back(static_cast<char>((value >> shift) & 0xffU));
        }
    };
    append32(58);
    append16(0);
    append16(0);
    append32(54);
    append32(40);
    append32(1);
    append32(1);
    append16(1);
    append16(24);
    append32(0);
    append32(4);
    append32(0);
    append32(0);
    append32(0);
    append32(0);
    bytes.append("\0\0\0\0", 4);
    return bytes;
}

std::string o3dMesh(std::uint16_t lastIndex = 2) {
    std::string bytes("\x84\x19\x03\x17\x03\x00", 6);
    for (const auto& vertex :
         std::array<std::array<float, 3>, 3>{std::array<float, 3>{0, 0, 0}, {1, 0, 0}, {0, 1, 0}}) {
        for (std::size_t channel = 0; channel < 8; ++channel) {
            const float value = channel < vertex.size() ? vertex[channel] : 0.0F;
            const auto bits = std::bit_cast<std::uint32_t>(value);
            for (unsigned shift = 0; shift < 32; shift += 8) {
                bytes.push_back(static_cast<char>((bits >> shift) & 0xffU));
            }
        }
    }
    bytes.append("\x49\x01\x00\x00\x00\x01\x00", 7);
    bytes.push_back(static_cast<char>(lastIndex & 0xffU));
    bytes.push_back(static_cast<char>(lastIndex >> 8));
    bytes.append(2, '\0');
    return bytes;
}

std::string texturedO3dMesh(const std::string& texture) {
    std::string bytes = o3dMesh();
    bytes.append("\x26\x01\x00", 3);
    bytes.append(44, '\0');
    bytes.push_back(static_cast<char>(texture.size()));
    bytes += texture;
    return bytes;
}

void run() {
    const auto root = openbus::probe::makeProbeDirectory("openbus-map-asset-validation", false);
    try {
        const auto map = root / "Maps" / "Fixture";
        write(map / "global.cfg",
              "[name]\nFixture\n[map]\n0\n0\ntile_0_0.map\n"
              "[map]\n1\n0\ntile_1_0.map\n"
              "[map]\n2\n0\ntile_2_0.map\n"
              "[ticketpack]\nTicketPacks\\missing.otp\n"
              "[moneysystem]\nMoney\\missing.cfg\n"
              "[groundtex]\nTexture\\grass.bmp\nTexture\\missing.bmp\n0\n1\n60\n");
        write(map / "timeline.prt.cfg", "opaque timeline companion\n");
        write(map / "laststn.osn", "opaque station companion\n");
        write(map / "laststn.osn.owt", "opaque station weather companion\n");
        write(map / "laststn.osn_event.dds", "opaque station image companion\n");
        std::filesystem::create_directories(map / "laststn.osn_directory.dds");
        std::filesystem::create_directories(map / "Holidays_directory.txt");
        write(map / "signalroutes.cfg", "opaque signal route companion\n");
        write(map / "registrations.txt", "opaque registration companion\n");
        write(map / "unclassified-local.dat", "unclassified map-root fixture\n");
        write(map / "custom" / "opaque" / "nested-resource.dat",
              "nested unclassified map-root fixture\n");
        write(map / "Holidays_2025.txt", "opaque holiday companion\n");
        write(map / "environment" / "sky.cfg",
              "[sky_textures]\nsky\\day.bmp\nsky\\missing-evening.bmp\n");
        write(map / "environment" / "clouds.cfg",
              "[cloudtype]\nFixture Cloud\nclouds\\fixture.bmp\n1200\n1\n"
              "[cloudtype]\nMissing Cloud\nclouds\\missing-cloud.bmp\n900\n2\n"
              "[unsupported_environment]\nvalue\n");
        write(root / "Texture" / "sky" / "day.bmp", onePixelBmp());
        write(root / "Texture" / "clouds" / "fixture.bmp", onePixelBmp());
        write(map / "TicketPacks" / "missing.otp", "[ticketpack]\n0.5\n");
        write(map / "TTData" / "SplineCheck.ttp",
              "[trip]\nSplineCheck\nDestination\nLine\n"
              "[station]\n19003\n0\nStop\n0\n0\n0\n0\n0\n"
              "[future_timetable_section]\nFuture payload\n");
        write(map / "TTData" / "SplineCheck.ttr",
              "[track_entry]\n1\n0\n1\n1\n10\n0\n"
              "[track_entry]\n5\n0\n1\n2\n10\n0\n"
              "[track_entry]\n999\n0\n1\n2\n10\n0\n"
              "[future_track_section]\nFuture payload\n");
        write(map / "TTData" / "Busstops.cfg",
              "[future_bus_stop_section]\nFuture payload\n");
        write(map / "TTData" / "StnLinks.cfg",
              "[future_station_link_section]\nFuture payload\n");
        write(map / "tile_0_0.map",
              "Fixture tile\n[version]\n14\n[terrain]\n"
              "[spline]\n0\nSplines\\road.sli\n1\n0\n0\n0\n0\n0\n0\n"
              "10\n0\n0\n0\n0\n0\n0\n0\n0\n\n"
              "[spline]\n0\nSplines\\missing.sli\n2\n0\n0\n0\n0\n0\n0\n"
              "10\n0\n0\n0\n0\n0\n0\n0\n0\n\n"
              "[object]\n0\nScenery\\missing.sco\n5\n1\n2\n0\n0\n0\n0\n0\n"
              "[object]\n0\nScenery\\directory.sco\n23\n1\n2\n0\n0\n0\n0\n0\n"
              "[object]\n0\nscenery\\PRESENT.sco\n6\n1\n2\n0\n0\n0\n0\n0\n"
              "[object]\n0\nScenery\\present.sco\n7\n1\n2\n0\n0\n0\n0\n0\n"
              "[object]\n0\nScenery\\external-missing.sco\n8\n1\n2\n0\n0\n0\n0\n0\n"
              "[object]\n0\nScenery\\editor.sco\n9\n1\n2\n0\n0\n0\n0\n0\n"
              "[object]\n0\nScenery\\second.sco\n10\n1\n2\n0\n0\n0\n0\n0\n"
              "[object]\n0\nScenery\\external-directory.sco\n25\n1\n2\n0\n0\n0\n0\n0\n"
              "[object]\n0\nScenery\\valid-collision.sco\n11\n1\n2\n0\n0\n0\n0\n0\n"
              "[object]\n0\nScenery\\broken-collision.sco\n12\n1\n2\n0\n0\n0\n0\n0\n"
              "[object]\n0\nScenery\\collision-external.sco\n13\n1\n2\n0\n0\n0\n0\n0\n"
              "[object]\n0\nScenery\\invalid-cabin.sco\n24\n1\n2\n0\n0\n0\n0\n0\n"
              "[object]\n0\nScenery\\park.sco\n14\n1\n2\n0\n0\n0\n0\n0\n"
              "[object]\n0\nScenery\\park.sco\n15\n1\n2\n0\n0\n0\n0\n1\n2\n"
              "[object]\n0\nScenery\\park.sco\n16\n1\n2\n0\n0\n0\n0\n1\n3\n"
              "[object]\n0\nScenery\\park.sco\n17\n1\n2\n0\n0\n0\n0\n1\nbad\n"
              "[object]\n0\nScenery\\park.sco\n18\n1\n2\n0\n0\n0\n0\n1\n4\n"
              "[object]\n0\nScenery\\park.sco\n19\n1\n2\n0\n0\n0\n0\n0\n"
              "[object]\n0\nScenery\\park.sco\n27\n1\n2\n0\n0\n0\n0\n1\n7\n"
              "[attachObj]\n0\nScenery\\park.sco\n20\n14\n0\n0\n0\n0\n0\n1\n5\n"
              "[splineAttachement]\n0\nScenery\\park.sco\n21\n0\n0\n0\n0\n0\n0\n0\n"
              "10\n2\n0\n1\n6\n"
              "[object]\n0\nScenery\\caf\xc3\xa9.sco\n22\n1\n2\n0\n0\n0\n0\n0\n");
        write(map / "tile_0_0.map.prt", "opaque tile companion\n");
        write(map / "tile_0_0.map.terrain_0.rdy", "opaque readiness companion\n");
        write(map / "tile_0_0.map.LM.bmp", onePixelBmp());
        write(map / "tile_2_0.map", "[version]\ninvalid\n[terrain]\n[object]\n");
        write(root / "Texture" / "grass.bmp", "invalid texture image");
        write(root / "Scenery" / "present.sco",
              "[model]\nmodel\\scene.cfg\n[script]\n3\nscript\\good.osc\nscript\\missing.osc\n"
              "script\\directory.osc\n"
              "[varnamelist]\n3\nscript\\GOOD-vars.txt\nscript\\missing-vars.txt\n"
              "script\\directory-vars.txt\n"
              "[stringvarnamelist]\n1\nscript\\missing-strings.txt\n"
              "[constfile]\n3\nscript\\good-const.txt\nscript\\missing-const.txt\n"
              "script\\bad-const.txt\n[passengercabin]\ncabin.cfg\n");
        write(root / "Scenery" / "cabin.cfg",
              "[passpos]\n-0.2\n-1.5\n0\n0\n185\n");
        std::filesystem::create_directories(root / "Scenery" / "directory.sco");
        write(root / "Scenery" / "script" / "good.osc", "{macro:init}\n{end}\n");
        std::filesystem::create_directories(root / "Scenery" / "script" / "directory.osc");
        write(root / "Scenery" / "script" / "good-vars.txt", "foo\n");
        write(root / "Scenery" / "script" / "good-const.txt", "[const]\nfoo\n1\n");
        write(root / "Scenery" / "script" / "bad-const.txt", "[const]\nfoo\ninvalid\n");
        std::filesystem::create_directories(root / "Scenery" / "script" / "directory-vars.txt");
        write(root / "Scenery" / "external-missing.sco", "[model]\nmodel\\absent.cfg\n");
        write(root / "Scenery" / "external-directory.sco", "[model]\nmodel\\directory.cfg\n");
        std::filesystem::create_directories(root / "Scenery" / "model" / "directory.cfg");
        write(root / "Scenery" / "editor.sco", "[onlyeditor]\n[mesh]\nmissing-editor.x\n");
        write(root / "Scenery" / openbus::pathFromUtf8("caf\xc3\xa9.sco"), "[onlyeditor]\n");
        write(root / "Scenery" / "second.sco",
              "[model]\nmodel\\scene.cfg\n[passengercabin]\nmissing-cabin.cfg\n");
        write(root / "Scenery" / "invalid-cabin.sco",
              "[model]\nmodel\\scene.cfg\n[passengercabin]\ninvalid-cabin.cfg\n");
        write(root / "Scenery" / "invalid-cabin.cfg",
              "[passpos]\nnot-a-number\n-1.5\n0\n0\n185\n");
        write(root / "Scenery" / "valid-collision.sco", "[collision_mesh]\nvalid-collision.o3d\n");
        write(root / "Scenery" / "broken-collision.sco", "[collision_mesh]\nbroken-inline.o3d\n");
        write(root / "Scenery" / "collision-external.sco", "[model]\nmodel\\collision.cfg\n");
        write(root / "Scenery" / "park.sco", "[onlyeditor]\n[carpark_p]\n");
        std::filesystem::create_directories(root / "Scenery" / "directory-parked.sco");
        write(root / "Scenery" / "parked.sco", "[mesh]\nmissing-parked-mesh.o3d\n"
                                               "[script]\n1\nscript\\missing-parked.osc\n");
        write(map / "Local" / "parked.sco", "[mesh]\nmissing-local-parked.o3d\n");
        write(root / "Scenery" / "numbered-parked.sco", "[mesh]\nmissing-numbered-parked.o3d\n");
        write(map / "ParkList_P.txt", "\xEF\xBB\xBF# comment\n; comment\n\n  Scenery\\parked.sco \n"
                                      "Scenery\\parked.sco\nLocal\\parked.sco\n"
                                      "Scenery\\missing-parked.sco\nScenery\\missing-parked.sco\n");
        write(map / "parklist_p_2.txt", "Scenery\\numbered-parked.sco\n");
        write(map / "parklist_p_4.txt", "# no candidates\n");
        std::filesystem::create_directories(map / "parklist_p_6.txt");
        write(map / "parklist_p_7.txt", "Scenery\\directory-parked.sco\n");
        write(map / "parklist_p_99.txt", "Scenery\\unreferenced-parked.sco\n");
        const auto list =
            openbus::map::readCarparkList(map / "ParkList_P.txt", [](const std::string& entry) {
                return entry != "Scenery\\missing-parked.sco";
            });
        require(list.opened && list.complete &&
                    list.entries == std::vector<std::string>{"Scenery\\parked.sco",
                                                             "Scenery\\parked.sco",
                                                             "Local\\parked.sco"},
                "shared car-park reader changed comment, BOM, filtering or weighting behavior");
        require(openbus::map::carparkListIndex({}).value() == 0 &&
                    openbus::map::carparkListIndex({"2"}).value() == 2 &&
                    !openbus::map::carparkListIndex({"-1"}) &&
                    !openbus::map::carparkListIndex({"2x"}),
                "car-park list selector parsing changed");
        write(root / "Scenery" / "model" / "collision.cfg",
              "[collision_mesh]\nShared\\globalonly.o3d\n"
              "[collision_mesh]\nMapAssets\\localonly.o3d\n"
              "[collision_mesh]\nbad-index.o3d\n"
              "[collision_mesh]\nmissing-external.o3d\n"
              "[collision_mesh]\ndirectory-collision.o3d\n");
        write(root / "Scenery" / "model" / "valid-collision.o3d", o3dMesh());
        std::filesystem::create_directories(root / "Scenery" / "model" / "directory-mesh.o3d");
        std::filesystem::create_directories(root / "Scenery" / "model" /
                                            "directory-texture.bmp");
        write(root / "Scenery" / "model" / "broken-inline.o3d", "invalid");
        write(root / "Scenery" / "model" / "bad-index.o3d", o3dMesh(9));
        std::filesystem::create_directories(root / "Scenery" / "model" /
                                            "directory-collision.o3d");
        write(root / "Shared" / "globalonly.o3d", o3dMesh());
        write(map / "MapAssets" / "localonly.o3d", o3dMesh());
        write(root / "Scenery" / "model" / "scene.cfg",
              "[mesh]\nmissing.o3d\n[mesh]\ndirectory-mesh.o3d\n"
              "[matl]\npresent.bmp\n0\n"
              "[matl_transmap]\nmissing-alpha.bmp\n"
              "[mesh]\npresent.o3d\n[matl]\nmissing-texture.bmp\n0\n"
              "[matl]\ndirectory-texture.bmp\n0\n"
              "[collision_mesh]\nmissing-collision.o3d\n"
              "[matl]\ntext-placeholder.bmp\n0\n[usetexttexture]\n0\n"
              "[matl]\nscript-placeholder.bmp\n0\n[usescripttexture]\n0\n"
              "[mesh]\nembedded.x\n"
              "[mesh]\noverride.x\n[matl]\nvirtual.bmp\n0\n[usetexttexture]\n0\n"
              "[mesh]\nmissing.o3d\n[matl]\nbroken.bmp\n0\n"
              "[mesh]\nmesh.o3d\n[mesh]\nmissingmat.o3d\n[mesh]\ncorrupt.o3d\n");
        write(root / "Scenery" / "model" / "present.o3d", std::string("\x84\x19\x03", 3));
        write(root / "Scenery" / "model" / "embedded.x", xMesh("embedded-missing.bmp"));
        write(root / "Scenery" / "model" / "override.x", xMesh("overridden-source.bmp"));
        write(root / "Scenery" / "model" / "mesh.obj",
              "mtllib mesh.mtl\nv 0 0 0\nv 1 0 0\nv 0 1 0\nusemtl matl_0\nf 1 2 3\n");
        write(root / "Scenery" / "model" / "mesh.mtl", "newmtl matl_0\nmap_Kd obj-missing.bmp\n");
        write(root / "Scenery" / "model" / "missingmat.obj", "mtllib missing-lib.mtl\n");
        write(root / "Scenery" / "model" / "corrupt.o3d", "invalid");
        write(root / "Scenery" / "texture" / "present.dds", onePixelBmp());
        write(root / "Scenery" / "texture" / "Summer" / "present.dds", onePixelBmp());
        write(root / "Scenery" / "texture" / "WinterSnow" / "present.dds", "invalid image");
        std::filesystem::create_directories(root / "Scenery" / "texture" / "Winter" /
                                            "present.dds");
        write(root / "Scenery" / "texture" / "broken.bmp", "invalid image");
        write(root / "Splines" / "road.sli",
              "[texture]\nmissing-road.bmp\n[texture]\nsub\\present.bmp\n"
              "[texture]\nsub\\broken.bmp\n"
              "[future_spline_directive]\nopaque value\n");
        write(root / "Splines" / "sub" / "texture" / "present.bmp", onePixelBmp());
        write(root / "Splines" / "sub" / "texture" / "broken.bmp", "invalid image");
        write(root / "Texture" / "grass.dds", "");
        write(map / "ailists.cfg", "[aigroup_2]\nNormalCars\nVehicles\\missing.bus\n"
                                   "Vehicles\\car\\car.ovh\nVehicles\\car\\car.ovh\n"
                                   "Vehicles\\car\\car2.ovh\nVehicles\\bundle\\bundle.ovh\n"
                                   "Vehicles\\embedded\\embedded.ovh\n"
                                   "Vehicles\\ctc\\ctc.ovh\n"
                                   "Trains\\base-fixture.zug\n"
                                   "base-fixture-bare.zug\n"
                                   "Trains\\missing-base-train.zug\n"
                                   "Vehicles\\car\\directory-model.ovh\n[end]\n"
                                   "[aigroup_depot]\nDepot\nHof\n[aigroup_depot_typgroup_2]\n"
                                   "Vehicles\\missing-depot.bus\n[end]\n");
        write(root / "Trains" / "base-fixture.zug",
              "Vehicles\\missing-base-train-car.ovh\n0\n"
              "[future_train_section]\nFuture payload\n");
        write(root / "Trains" / "base-fixture-bare.zug",
              "Vehicles\\missing-bare-train-car.ovh\n0\n");
        write(root / "Vehicles" / "car" / "car.ovh",
              "[model]\nmodel\\car.cfg\n[sound]\nsound\\sound.cfg\n"
              "[sound_ai]\nsound\\missing.cfg\n"
              "[paths]\nmodel\\paths.cfg\n"
              "[passengercabin]\nmodel\\invalid-cabin.cfg\n"
              "[number]\nmodel\\missing-numbers.org\n"
              "[registration_list]\nmodel\\missing-registrations.org\n"
              "[script]\n1\nscript\\missing.osc\n"
              "[varnamelist]\n1\nscript\\missing-vehicle-vars.txt\n"
              "[stringvarnamelist]\n1\nscript\\missing-vehicle-strings.txt\n"
              "[constfile]\n1\nscript\\missing-vehicle-const.txt\n"
              "[couple_back]\ntrailer.ovh\nfalse\n");
        write(root / "Vehicles" / "car" / "depot.hof", "[name]\nFixture Depot\n");
        write(root / "Vehicles" / "car" / "model" / "paths.cfg",
              "[stepsoundpack]\n3\npresent-step.wav\nmissing-step.wav\nbroken-step.wav\n"
              "[next_stepsound]\ninvalid-index\n[stepsoundpack]\n1\ndirectory-step.wav\n");
        write(root / "Sounds" / "Passengers" / "present-step.wav", pcmWav());
        write(root / "Sounds" / "Passengers" / "broken-step.wav", "not a WAV file");
        std::filesystem::create_directories(root / "Sounds" / "Passengers" /
                                            "directory-step.wav");
        write(root / "Vehicles" / "car" / "car2.ovh",
              "[model]\nmodel\\car.cfg\n[sound]\nsound\\sound.cfg\n"
              "[sound_ai]\nsound\\directory.cfg\n"
              "[passengercabin]\nmodel\\missing-cabin.cfg\n"
              "[paths]\nmodel\\missing-paths.cfg\n"
              "[couple_back]\nabsent-trailer.ovh\nfalse\n");
        std::filesystem::create_directories(root / "Vehicles" / "car" / "sound" /
                                            "directory.cfg");
        write(root / "Vehicles" / "car" / "directory-model.ovh",
              "[model]\nmodel\\directory.cfg\n");
        std::filesystem::create_directories(root / "Vehicles" / "car" / "model" /
                                            "directory.cfg");
        write(root / "Vehicles" / "car" / "model" / "invalid-cabin.cfg",
              "[passpos]\nnot-a-number\n-1.5\n0\n0\n185\n");
        write(root / "Vehicles" / "car" / "trailer.ovh",
              "[model]\nmodel\\trailer.cfg\n[couple_back]\ncar.ovh\nfalse\n"
              "[script]\n1\nscript\\missing-trailer.osc\n");
        write(root / "Vehicles" / "car" / "model" / "trailer.cfg", "[mesh]\nmissing-trailer.o3d\n");
        write(root / "Vehicles" / "car" / "model" / "car.cfg",
              "[mesh]\nmissing-car.o3d\n"
              "[mesh]\nvehicle.obj\n[matl]\nbase.bmp\n0\n"
              "[matl_transmap]\nmissing-alpha-vehicle.bmp\n"
              "[matl_nightmap]\nEXISTING-night.bmp\n"
              "[matl_bumpmap]\n\\S:0\n1\n"
              "[matl_lightmap]\n\\S:1\n--\n"
              "[matl_freetex]\n\\S:2\nfreevar\n"
              "[matl_envmap]\nmissing-environment.bmp\n1\n"
              "[matl_change]\nbase.bmp\n0\nswitch\n"
              "[matl_item]\n[matl_nightmap]\nmissing-item-night.bmp\n"
              "[matl_item]\n[matl_freetex]\ngenerated-placeholder.bmp\nfreevar\n"
              "[usescripttexture]\n0\n"
              "[matl]\nother.bmp\n0\n[matl_envmap]\nunused-environment.bmp\n0\n"
              "[matl]\nbroken-vehicle.bmp\n0\n"
              "[collision_mesh]\ngood.obj\n[collision_mesh]\nempty.obj\n"
              "[collision_mesh]\ninvalid-index.obj\n[collision_mesh]\nnonfinite.obj\n"
              "[collision_mesh]\nmixed.obj\n[collision_mesh]\ncorrupt-collision.o3d\n"
              "[collision_mesh]\ndirectory-collision.o3d\n");
        write(root / "Vehicles" / "car" / "model" / "vehicle.obj",
              "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n");
        write(root / "Vehicles" / "car" / "texture" / "nested" / "existing-NIGHT.dds",
              onePixelBmp());
        write(root / "Vehicles" / "car" / "texture" / "broken-vehicle.bmp", "invalid image");
        write(root / "Vehicles" / "car" / "model" / "good.obj",
              "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n");
        write(root / "Vehicles" / "car" / "model" / "empty.obj", "v 0 0 0\n");
        write(root / "Vehicles" / "car" / "model" / "invalid-index.obj",
              "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 4\n");
        write(root / "Vehicles" / "car" / "model" / "nonfinite.obj",
              "v nan 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n");
        write(root / "Vehicles" / "car" / "model" / "mixed.obj",
              "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 4\nf 1 2 3\n");
        write(root / "Vehicles" / "car" / "model" / "corrupt-collision.o3d", "invalid");
        std::filesystem::create_directories(root / "Vehicles" / "car" / "model" /
                                            "directory-collision.o3d");
        write(root / "Vehicles" / "embedded" / "embedded.ovh", "[model]\nmodel\\embedded.cfg\n");
        write(root / "Vehicles" / "embedded" / "model" / "embedded.cfg",
              "[mesh]\nembedded.x\n[mesh]\nembedded.obj\n[mesh]\nsource-o3d.o3d\n"
              "[mesh]\noverridden.x\n[matl]\nplaceholder.bmp\n0\n[usescripttexture]\n0\n");
        write(root / "Vehicles" / "embedded" / "model" / "embedded.x",
              xMesh("missing-vehicle-x.bmp"));
        write(root / "Vehicles" / "embedded" / "model" / "overridden.x",
              xMesh("overridden-vehicle-source.bmp"));
        write(root / "Vehicles" / "embedded" / "model" / "source-o3d.o3d",
              texturedO3dMesh("missing-vehicle-o3d.bmp"));
        write(root / "Vehicles" / "embedded" / "model" / "embedded.obj",
              "mtllib embedded.mtl\nv 0 0 0\nv 1 0 0\nv 0 1 0\n"
              "usemtl matl_0\nf 1 2 3\nusemtl matl_1\nf 1 2 3\n");
        write(root / "Vehicles" / "embedded" / "model" / "embedded.mtl",
              "newmtl matl_0\nmap_Kd missing-vehicle-obj.bmp\n"
              "newmtl matl_1\nmap_Kd EXISTING-embedded.bmp\n"
              "newmtl matl_2\nmap_Kd unused-vehicle-obj.bmp\n");
        write(root / "Vehicles" / "embedded" / "texture" / "nested" / "existing-embedded.dds",
              onePixelBmp());
        write(root / "Vehicles" / "ctc" / "ctc.ovh", "[model]\nmodel\\ctc.cfg\n");
        write(root / "Vehicles" / "ctc" / "model" / "ctc.cfg",
              "[ctctexture]\npaint\nrepaint-source.bmp\n[mesh]\nrepaint.x\n");
        write(root / "Vehicles" / "ctc" / "model" / "repaint.x", xMesh("repaint-source.bmp"));
        write(root / "Vehicles" / "ctc" / openbus::pathFromUtf8("p\xc3\xa4int.cti"),
              "[item]\nMissing Paint\npaint\nmissing-ctc-texture.bmp\n"
              "[item]\nPresent Paint\npaint\ntexture\\pr\xc3\xa4sent-repaint.bmp\n"
              "[item]\nDirectory Paint\npaint\ntexture\\directory-repaint.bmp\n"
              "[item]\nCompressed Paint\npaint\ntexture\\compressed-repaint.bmp\n"
              "[item]\nBroken Paint\npaint\ntexture\\broken-repaint.bmp\n"
              "[item]\nUnused Slot\nunmodeled\nmissing-unused-ctc-texture.bmp\n");
        write(root / "Vehicles" / "ctc" / "empty.cti", "[unknown]\n");
        std::filesystem::create_directories(root / "Vehicles" / "ctc" / "directory.cti");
        write(root / "Vehicles" / "ctc" / "texture" /
                  openbus::pathFromUtf8("pr\xc3\xa4sent-repaint.bmp"),
              onePixelBmp());
        write(root / "Vehicles" / "ctc" / "texture" / "compressed-repaint.dds", onePixelBmp());
        write(root / "Vehicles" / "ctc" / "texture" / "broken-repaint.bmp", "invalid image");
        std::filesystem::create_directories(root / "Vehicles" / "ctc" / "texture" /
                                            "directory-repaint.bmp");
        write(root / "Vehicles" / "bundle" / "bundle.ovh", "[model]\nmodel\\bundle.cfg\n");
        write(root / "Vehicles" / "bundle" / "model" / "bundle.cfg",
              "[mesh]\nembedded.o3d\n"
              "[collision_mesh]\npresent.o3d\n[collision_mesh]\nmissing-bundle.o3d\n");
        const auto bundleInput = root / "bundle-input";
        write(bundleInput / "present.obj", "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n");
        write(bundleInput / "embedded.obj", "mtllib embedded.mtl\nv 0 0 0\nv 1 0 0\nv 0 1 0\n"
                                            "usemtl matl_0\nf 1 2 3\n");
        write(bundleInput / "embedded.mtl", "newmtl matl_0\nmap_Kd missing-vehicle-bundle.bmp\n");
        require(openbus::rendering::ObjLoader::writeBundle(
                    root / "Vehicles" / "bundle" / "model" / "openbus.obx", bundleInput,
                    {bundleInput / "present.obj", bundleInput / "embedded.obj"}),
                "could not build collision bundle fixture");
        write(root / "Vehicles" / "car" / "sound" / "sound.cfg",
              "[sound]\ngood.WAV\n1\n[sound]\ndirectory.wav\n1\n"
              "[loopsound]\nmissing-loop.wav\n1\n"
              "[sound]\n[trigger]\ninvalid\n[sound]\nmissing-later.wav\n1\n"
              "[sound]\nbroken.wav\n0.7\n"
              "[sound]\n\n0.5\n[3d]\n0\n0\n0\n5\n"
              "[future_sound_keyword]\nopaque payload\n"
              "-<DISABLED>-\n[sound]\nignored.wav\n-<ENABLED>-\n");
        write(root / "Vehicles" / "car" / "sound" / "good.wav", pcmWav());
        write(root / "Vehicles" / "car" / "sound" / "broken.wav", "not a WAV file");
        std::filesystem::create_directories(root / "Vehicles" / "car" / "sound" /
                                            "directory.wav");
        write(map / "humans.txt",
              "Humans\\missing.hum\nHumans\\person\\person.hum\nnot-a-human-reference\n"
              "Humans\\person\\directory-model.hum\n");
        write(root / "Humans" / "person" / "person.hum",
              "[model]\nmodel\\person.cfg\n[future_human_keyword]\nopaque value\n");
        write(root / "Humans" / "person" / "directory-model.hum",
              "[model]\nmodel\\directory.cfg\n");
        std::filesystem::create_directories(root / "Humans" / "person" / "model" /
                                            "directory.cfg");
        write(root / "Humans" / "person" / "model" / "person.cfg",
              "[mesh]\nmissing-person.o3d\n[matl]\nmissing-person.bmp\n0\n");
        write(map / "drivers.txt",
              "Humans\\missing-driver.hum\nHumans\\person\\missing-model.hum\n"
              "not-a-driver-reference\n");
        write(root / "Humans" / "person" / "missing-model.hum", "[model]\nmodel\\absent.cfg\n");

        std::vector<openbus::map::MapAssetValidationProgress> progressUpdates;
        openbus::map::MapAssetValidationOptions validationOptions;
        validationOptions.onProgress = [&](const auto& progress) {
            progressUpdates.push_back(progress);
        };
        const auto report = openbus::map::validateMapAssets(map, root, validationOptions);
        require(!progressUpdates.empty() &&
                    progressUpdates.back().phase ==
                        "Complete (dependency coverage remains incomplete)" &&
                    std::any_of(progressUpdates.begin(), progressUpdates.end(), [](const auto& p) {
                        return p.phase == "Checking base tiles" && p.completed == p.total;
                    }),
                "validation progress did not report tile completion and final status");
        require(!report.complete, "partial scan must not certify the map");
        require(report.tilesScanned == 1, "expected one readable tile");
        const auto containsSource = [&](const std::filesystem::path& path) {
            return std::any_of(report.sourceFiles.begin(), report.sourceFiles.end(),
                               [&](const auto& source) {
                                   return std::filesystem::equivalent(source, path);
                               });
        };
        require(containsSource(map / "global.cfg"), "source inventory omitted global.cfg");
        require(containsSource(map / "unclassified-local.dat") &&
                    std::any_of(report.findings.begin(), report.findings.end(),
                                [&](const auto& finding) {
                                    return finding.unsupported &&
                                           finding.category == "unclassified map file" &&
                                           finding.source == map / "unclassified-local.dat";
                                }),
                "unclassified map-root file was omitted or treated as understood");
        require(containsSource(map / "custom" / "opaque" / "nested-resource.dat") &&
                    std::any_of(report.findings.begin(), report.findings.end(),
                                [&](const auto& finding) {
                                    return finding.unsupported &&
                                           finding.category == "unclassified map file" &&
                                           finding.source ==
                                               map / "custom" / "opaque" / "nested-resource.dat";
                                }),
                "nested unclassified map-root file was not inventoried as unsupported");
        require(containsSource(map / "environment" / "sky.cfg") &&
                    containsSource(map / "environment" / "clouds.cfg"),
                "source inventory omitted environment configuration inputs");
        require(containsSource(map / "tile_0_0.map"), "source inventory omitted a tile");
        require(containsSource(map / "tile_0_0.map.LM.bmp") &&
                    !hasFinding(report, "lightmap", "tile_0_0.map.LM.bmp"),
                "valid tile lightmap was not inventoried or failed image decoding");
        require(std::any_of(report.findings.begin(), report.findings.end(),
                            [&](const auto& finding) {
                                return finding.category == "ground texture" &&
                                       finding.reference == "Texture\\grass.bmp" &&
                                       finding.detail.find("could not be decoded") !=
                                           std::string::npos;
                            }),
                "existing but undecodable ground texture was not reported");
        for (const auto* filename : {"timeline.prt.cfg", "laststn.osn", "laststn.osn.owt",
                                     "laststn.osn_event.dds", "signalroutes.cfg",
                                     "registrations.txt", "Holidays_2025.txt"}) {
            const auto path = map / filename;
            require(containsSource(path) &&
                        std::any_of(report.findings.begin(), report.findings.end(),
                                    [&](const auto& finding) {
                                        return finding.unsupported &&
                                               finding.category == "unsupported map companion" &&
                                               finding.source == path;
                                    }),
                    "opaque map-root companion was not inventoried and reported");
        }
        require(std::count_if(report.findings.begin(), report.findings.end(),
                             [](const auto& finding) {
                                 return finding.category == "map companion" &&
                                        (finding.reference == "laststn.osn_directory.dds" ||
                                         finding.reference == "Holidays_directory.txt") &&
                                        finding.detail.find("not a regular file") !=
                                            std::string::npos;
                             }) == 2,
                "wildcard-discovered companion directories were silently skipped");
        require(containsSource(map / "tile_0_0.map.prt") &&
                    std::any_of(report.findings.begin(), report.findings.end(),
                                [&](const auto& finding) {
                                    return finding.unsupported &&
                                           finding.category == "unsupported tile sidecar" &&
                                           finding.source == map / "tile_0_0.map.prt";
                                }),
                "uninterpreted tile .prt sidecars must be inventoried and reported");
        require(containsSource(map / "tile_0_0.map.terrain_0.rdy") &&
                    std::any_of(report.findings.begin(), report.findings.end(),
                                [&](const auto& finding) {
                                    return finding.unsupported &&
                                           finding.category == "unsupported tile sidecar" &&
                                           finding.source ==
                                               map / "tile_0_0.map.terrain_0.rdy" &&
                                           finding.detail.find("payload is not interpreted") !=
                                               std::string::npos;
                                }),
                "uninterpreted terrain-readiness payload must be reported");
        require(containsSource(map / "ailists.cfg") &&
                    containsSource(map / "TTData" / "SplineCheck.ttp") &&
                    containsSource(map / "TTData" / "SplineCheck.ttr"),
                "source inventory omitted AI or timetable inputs");
        require(containsSource(root / "Scenery" / "present.sco"),
                "source inventory omitted a placed scenery object");
        require(containsSource(root / "Scenery" / "cabin.cfg"),
                "source inventory omitted a scenery-object passenger-cabin config");
        require(hasFinding(report, "passenger cabin", "missing-cabin.cfg"),
                "missing scenery-object passenger-cabin config was not reported");
        require(std::count_if(report.findings.begin(), report.findings.end(),
                              [&](const auto& finding) {
                                  return finding.category == "passenger cabin" &&
                                         finding.source ==
                                             root / "Scenery" / "second.sco" &&
                                         finding.reference.find("missing-cabin.cfg") !=
                                             std::string::npos;
                              }) == 1,
                "missing scenery-object passenger-cabin config finding was duplicated");
        require(std::any_of(report.findings.begin(), report.findings.end(),
                            [&](const auto& finding) {
                                return finding.category == "passenger cabin" &&
                                       finding.source == root / "Scenery" / "invalid-cabin.cfg" &&
                                       finding.detail.find("(line 1)") != std::string::npos;
                            }),
                "invalid passenger-cabin record was not reported with its source line");
        require(!std::any_of(report.findings.begin(), report.findings.end(),
                             [&](const auto& finding) {
                                 return finding.category == "object" &&
                                        finding.source ==
                                            root / "Scenery" / "invalid-cabin.sco" &&
                                        finding.detail == "expected numeric values";
                             }),
                "passenger-cabin parser error was duplicated as a scenery-object error");
        require(containsSource(root / "Scenery" / "model" / "scene.cfg"),
                "source inventory omitted a scenery model configuration");
        require(containsSource(root / "Scenery" / "texture" / "Summer" / "present.dds") &&
                    hasFinding(report, "seasonal scenery texture", "WinterSnow") &&
                    std::any_of(report.findings.begin(), report.findings.end(),
                                [](const auto& finding) {
                                    return finding.category == "seasonal scenery texture" &&
                                           finding.reference.find("Winter") !=
                                               std::string::npos &&
                                           finding.reference.find("present.dds") !=
                                               std::string::npos &&
                                           finding.detail.find("not a regular file") !=
                                               std::string::npos;
                                }),
                "seasonal scenery textures were not inventoried or decoded");
        require(containsSource(root / "Splines" / "road.sli"),
                "source inventory omitted a placed spline profile");
        require(std::any_of(report.findings.begin(), report.findings.end(),
                            [&](const auto& finding) {
                                return finding.unsupported &&
                                       finding.category == "unsupported section" &&
                                       finding.source == root / "Splines" / "road.sli" &&
                                       finding.reference == "future_spline_directive";
                            }),
                "unsupported spline-profile directives were not reported");
        require(std::any_of(report.findings.begin(), report.findings.end(),
                            [&](const auto& finding) {
                                return finding.unsupported &&
                                       finding.category == "unsupported section" &&
                                       finding.source == map / "TTData" &&
                                       finding.reference == "future_timetable_section";
                            }),
                "unsupported timetable sections were not reported");
        require(std::any_of(report.findings.begin(), report.findings.end(),
                                            [&](const auto& finding) {
                                                return finding.unsupported &&
                                                       finding.category == "unsupported section" &&
                                                       finding.source == map / "TTData" &&
                                                       finding.reference == "future_track_section";
                                            }),
                                "unsupported timetable track sections were not reported");
        require(std::any_of(report.findings.begin(), report.findings.end(),
                                            [&](const auto& finding) {
                                                return finding.unsupported &&
                                                       finding.category == "unsupported section" &&
                                                       finding.source == map / "TTData" &&
                                                       finding.reference == "future_bus_stop_section";
                                            }) &&
                                    std::any_of(report.findings.begin(), report.findings.end(),
                                                [&](const auto& finding) {
                                                    return finding.unsupported &&
                                                           finding.category == "unsupported section" &&
                                                           finding.source == map / "TTData" &&
                                                           finding.reference ==
                                                               "future_station_link_section";
                                                }),
                                "unsupported bus-stop or station-link sections were not reported");
        require(containsSource(root / "Splines" / "sub" / "texture" / "present.bmp"),
                "source inventory omitted a spline texture");
        require(containsSource(root / "Vehicles" / "ctc" /
                               openbus::pathFromUtf8("p\xc3\xa4int.cti")),
                "source inventory omitted a CTI appearance file");
        require(containsSource(root / "Vehicles" / "ctc" / "empty.cti"),
                "source inventory omitted an empty but scanned CTI file");
        require(containsSource(root / "Vehicles" / "ctc" / "texture" /
                               openbus::pathFromUtf8("pr\xc3\xa4sent-repaint.bmp")),
                "source inventory omitted a resolved CTC replacement texture");
        require(containsSource(root / "Vehicles" / "car" / "depot.hof"),
                "source inventory omitted a scanned vehicle depot file");
        require(containsSource(root / "Vehicles" / "car" / "car.ovh") &&
                    containsSource(root / "Vehicles" / "car" / "model" / "car.cfg") &&
                    containsSource(root / "Vehicles" / "car" / "sound" / "sound.cfg") &&
                    containsSource(root / "Vehicles" / "car" / "sound" / "good.wav"),
                "source inventory omitted a transitive AI vehicle dependency");
        const auto manifestSources =
            openbus::map::makeMapBakeSourceFiles(report.sourceFiles, map, root);
        const auto hasManifestKey = [&](std::string expected) {
            std::transform(expected.begin(), expected.end(), expected.begin(),
                           [](unsigned char ch) {
                               return static_cast<char>(ch >= 'A' && ch <= 'Z'
                                                            ? ch + ('a' - 'A')
                                                            : ch);
                           });
            return std::any_of(manifestSources.begin(), manifestSources.end(),
                               [&](const auto& source) {
                                   std::string key = source.key;
                                   std::transform(key.begin(), key.end(), key.begin(),
                                                  [](unsigned char ch) {
                                                      return static_cast<char>(
                                                          ch >= 'A' && ch <= 'Z'
                                                              ? ch + ('a' - 'A')
                                                              : ch);
                                                  });
                                   return key == expected;
                               });
        };
        require(hasManifestKey("map/global.cfg") && hasManifestKey("map/tile_0_0.map") &&
                    hasManifestKey("omsi/scenery/present.sco") &&
                    hasManifestKey("omsi/scenery/texture/summer/present.dds"),
                "validation inventory was not assigned rooted logical manifest keys");
        require(std::adjacent_find(report.sourceFiles.begin(), report.sourceFiles.end(),
                                   [](const auto& left, const auto& right) {
                                       return std::filesystem::equivalent(left, right);
                                   }) == report.sourceFiles.end(),
                "source inventory contains duplicate paths");
        require(hasFinding(report, "tile", "tile_1_0.map"), "missing tile not reported");
        require(std::any_of(report.findings.begin(), report.findings.end(),
                            [&](const auto& finding) {
                                return finding.category == "tile" &&
                                       finding.source == map / "tile_2_0.map";
                            }),
                "malformed tile failure not reported");
        require(hasFinding(report, "terrain", "tile_0_0.map.terrain"),
                "missing terrain not reported");
        require(std::any_of(report.findings.begin(), report.findings.end(),
                            [&](const auto& finding) {
                                return finding.category == "timetable track" &&
                                       finding.reference == "999" &&
                                       finding.detail.find("line 16") != std::string::npos;
                            }) &&
                    std::none_of(report.findings.begin(), report.findings.end(),
                                 [](const auto& finding) {
                                     return finding.category == "timetable track" &&
                                            (finding.reference == "1" ||
                                             finding.reference == "5");
                                 }),
                "missing timetable track ID was not reported or a present placement ID was rejected");
        require(hasFinding(report, "object", "missing.sco"), "missing object not reported");
        require(std::any_of(report.findings.begin(), report.findings.end(),
                            [&](const auto& finding) {
                                return finding.category == "object" &&
                                       finding.reference == "Scenery\\directory.sco" &&
                                       finding.detail.find("not a regular file") !=
                                           std::string::npos;
                            }),
                "referenced directories must be distinguished from missing object files");
        require(!hasFinding(report, "object", "caf\xc3\xa9.sco"),
                "UTF-8 scenery filename was not resolved");
        require(hasFinding(report, "parked object", "missing-parked.sco") &&
                    hasFinding(report, "mesh", "missing-parked-mesh.o3d") &&
                    hasFinding(report, "mesh", "missing-local-parked.o3d") &&
                    hasFinding(report, "mesh", "missing-numbered-parked.o3d") &&
                    hasFinding(report, "script", "missing-parked.osc"),
                "car-park list entries or their dependencies were not checked");
        require(std::any_of(report.findings.begin(), report.findings.end(),
                            [](const auto& finding) {
                                return finding.category == "parked object" &&
                                       finding.reference == "Scenery\\directory-parked.sco" &&
                                       finding.detail.find("not a regular file") !=
                                           std::string::npos;
                            }),
                "car-park entries pointing to directories were reported as missing scenery");
        require(hasFinding(report, "park list", "parklist_p_3.txt") &&
                    hasFinding(report, "park list", "bad") &&
                    hasFinding(report, "park list", "parklist_p_4.txt") &&
                    hasFinding(report, "park list", "parklist_p_5.txt") &&
                    hasFinding(report, "park list", "parklist_p_6.txt") &&
                    std::any_of(report.findings.begin(), report.findings.end(),
                                [&](const auto& finding) {
                                    return finding.category == "park list" &&
                                           finding.reference == "parklist_p_6.txt" &&
                                           finding.detail.find("not a regular file") !=
                                               std::string::npos;
                                }),
                "missing, invalid or empty car-park list was not reported");
        require(!hasFinding(report, "parked object", "unreferenced-parked.sco") &&
                    std::count_if(report.findings.begin(), report.findings.end(),
                                  [](const auto& finding) {
                                      return finding.category == "parked object" &&
                                             finding.reference == "Scenery\\missing-parked.sco";
                                  }) == 1,
                "unreferenced car-park lists or duplicate entries were checked incorrectly");
        require(!hasFinding(report, "object", "PRESENT.sco"), "case-insensitive lookup failed");
        require(hasFinding(report, "model", "absent.cfg"), "missing model not reported");
        const auto hasDirectoryModelFinding = [&](const std::filesystem::path& source) {
            return std::any_of(report.findings.begin(), report.findings.end(),
                               [&](const auto& finding) {
                                   return finding.category == "model" && finding.source == source &&
                                          finding.detail.find("not a regular file") !=
                                              std::string::npos;
                               });
        };
        require(hasDirectoryModelFinding(root / "Scenery" / "external-directory.sco"),
                "scenery model directory was not distinguished from a missing file");
        require(hasDirectoryModelFinding(root / "Humans" / "person" / "directory-model.hum"),
                "human model directory was not distinguished from a missing file");
        require(hasDirectoryModelFinding(root / "Vehicles" / "car" / "directory-model.ovh"),
                "vehicle model directory was not distinguished from a missing file");
        require(hasFinding(report, "mesh", "missing.o3d"), "missing scenery mesh not reported");
        require(std::any_of(report.findings.begin(), report.findings.end(),
                            [](const auto& finding) {
                                return finding.category == "mesh" &&
                                       finding.reference == "directory-mesh.o3d" &&
                                       finding.detail.find("not a regular file") !=
                                           std::string::npos;
                            }),
                "model mesh directory was reported as missing");
        require(hasFinding(report, "mesh", "missing-editor.x"),
                "editor-only object mesh not checked");
        require(hasFinding(report, "collision mesh", "missing-collision.o3d"),
                "missing collision mesh not reported");
        require(!hasFinding(report, "collision mesh", "valid-collision.o3d") &&
                    !hasFinding(report, "collision mesh", "globalonly.o3d") &&
                    !hasFinding(report, "collision mesh", "localonly.o3d"),
                "valid scenery collision mesh or root fallback was not resolved");
        require(hasFinding(report, "collision mesh", "broken-inline.o3d") &&
                    hasFinding(report, "collision mesh", "bad-index.o3d") &&
                    hasFinding(report, "collision mesh", "missing-external.o3d"),
                "invalid or missing scenery collision mesh was not reported");
        require(std::count_if(report.findings.begin(), report.findings.end(),
                              [](const auto& finding) {
                                  return finding.category == "collision mesh" &&
                                         finding.reference == "directory-collision.o3d" &&
                                         finding.detail.find("not a regular file") !=
                                             std::string::npos;
                              }) == 2,
                "scenery and vehicle collision-mesh directories were not identified explicitly");
        require(std::count_if(report.findings.begin(), report.findings.end(),
                              [](const auto& finding) {
                                  return finding.category == "collision mesh" &&
                                         finding.reference == "broken-inline.o3d";
                              }) == 1,
                "inline scenery collision mesh finding was duplicated");
        require(hasFinding(report, "model texture", "missing-alpha.bmp"),
                "missing transmap not reported");
        require(hasFinding(report, "model texture", "missing-texture.bmp"),
                "missing scenery texture not reported");
        require(std::any_of(report.findings.begin(), report.findings.end(),
                            [](const auto& finding) {
                                return finding.category == "model texture" &&
                                       finding.reference == "directory-texture.bmp" &&
                                       finding.detail.find("not a regular file") !=
                                           std::string::npos;
                            }),
                "scenery texture directory was reported as missing");
        require(!hasFinding(report, "model texture", "present.bmp"),
                "scenery texture fallback not honored");
        require(hasFinding(report, "model texture", "broken.bmp"),
                "undecodable scenery material texture was not reported");
        require(!hasFinding(report, "model texture", "text-placeholder.bmp"),
                "generated text texture was treated as a file");
        require(!hasFinding(report, "model texture", "script-placeholder.bmp"),
                "generated script texture was treated as a file");
        require(hasFinding(report, "model texture", "embedded-missing.bmp"),
                "missing X mesh texture not reported");
        require(std::count_if(report.findings.begin(), report.findings.end(),
                              [](const auto& finding) {
                                  return finding.category == "model texture" &&
                                         finding.reference == "embedded-missing.bmp";
                              }) == 1,
                "shared mesh texture finding was duplicated");
        require(hasFinding(report, "model texture", "obj-missing.bmp"),
                "missing OBJ material texture not reported");
        require(hasFinding(report, "material library", "missing-lib.mtl"),
                "missing OBJ material library not reported");
        require(hasFinding(report, "mesh", "corrupt.o3d"), "unreadable mesh not reported");
        require(!hasFinding(report, "model texture", "overridden-source.bmp"),
                "generated texture override did not suppress source texture");
        require(hasFinding(report, "script", "missing.osc"), "missing script not reported");
        require(!hasFinding(report, "script", "good.osc"), "existing script reported missing");
        require(std::any_of(report.findings.begin(), report.findings.end(),
                            [](const auto& finding) {
                                return finding.category == "script" &&
                                       finding.reference == "script\\directory.osc" &&
                                       finding.detail.find("not a regular file") !=
                                           std::string::npos;
                            }),
                "script directory was not distinguished from a missing file");
        require(hasFinding(report, "variable list", "missing-vars.txt") &&
                    hasFinding(report, "variable list", "directory-vars.txt") &&
                    hasFinding(report, "string variable list", "missing-strings.txt") &&
                    hasFinding(report, "constant file", "missing-const.txt") &&
                    hasFinding(report, "variable list", "missing-vehicle-vars.txt") &&
                    hasFinding(report, "string variable list", "missing-vehicle-strings.txt") &&
                    hasFinding(report, "constant file", "missing-vehicle-const.txt"),
                "missing scenery or vehicle script definition files were not reported");
        require(std::any_of(report.findings.begin(), report.findings.end(),
                            [](const auto& finding) {
                                return finding.category == "variable list" &&
                                       finding.reference == "script\\directory-vars.txt" &&
                                       finding.detail.find("not a regular file") !=
                                           std::string::npos;
                            }),
                "variable-list directory was not distinguished from a missing file");
        require(!hasFinding(report, "variable list", "GOOD-vars.txt") &&
                    !hasFinding(report, "constant file", "good-const.txt") &&
                    std::none_of(report.findings.begin(), report.findings.end(),
                                 [](const auto& finding) {
                                     return (finding.category == "object" ||
                                             finding.category == "AI vehicle") &&
                                            (finding.reference == "varnamelist" ||
                                             finding.reference == "stringvarnamelist" ||
                                             finding.reference == "constfile") &&
                                            finding.detail.find("unable to open ") == 0;
                                 }),
                "existing script definitions or duplicate open diagnostics were reported");
        require(std::any_of(report.findings.begin(), report.findings.end(),
                            [](const auto& finding) {
                                return finding.category == "object" &&
                                       finding.reference == "const" &&
                                       finding.detail.find("numeric value") != std::string::npos;
                            }),
                "constant-file parse error was suppressed");
        require(std::any_of(report.findings.begin(), report.findings.end(),
                            [&](const auto& finding) {
                                return finding.category == "variable list" &&
                                       std::filesystem::equivalent(
                                           finding.source, root / "Scenery" / "present.sco") &&
                                       finding.reference == "script\\missing-vars.txt";
                            }) &&
                    std::any_of(report.findings.begin(), report.findings.end(),
                                [&](const auto& finding) {
                                    return finding.category == "constant file" &&
                                           std::filesystem::equivalent(finding.source,
                                                                       root / "Vehicles" / "car" /
                                                                           "car.ovh") &&
                                           finding.reference == "script\\missing-vehicle-const.txt";
                                }),
                "script definition finding lost its source or authored path");
        for (const auto& entry : std::filesystem::directory_iterator(root / "Scenery" / "script")) {
            require(entry.path().extension() != ".lua", "validator generated a Lua script");
        }
        require(std::count_if(report.findings.begin(), report.findings.end(),
                              [&](const auto& finding) {
                                  return finding.category == "mesh" &&
                                         finding.reference == "missing.o3d";
                              }) == 1,
                "duplicate object was scanned more than once");
        require(hasFinding(report, "spline", "missing.sli"), "missing spline not reported");
        require(hasFinding(report, "spline texture", "missing-road.bmp"),
                "missing spline texture not reported");
        require(!hasFinding(report, "spline texture", "present.bmp"),
                "spline subdirectory texture fallback not honored");
        require(hasFinding(report, "spline texture", "broken.bmp"),
                "undecodable spline texture was not reported");
        require(hasFinding(report, "ground texture", "missing.bmp"),
                "missing ground texture not reported");
        require(hasFinding(report, "sky texture", "sky\\missing-evening.bmp") &&
                    !hasFinding(report, "sky texture", "sky\\day.bmp") &&
                    hasFinding(report, "cloud texture", "clouds\\missing-cloud.bmp") &&
                    !hasFinding(report, "cloud texture", "clouds\\fixture.bmp"),
                "environment sky or cloud textures were not checked with OMSI texture roots");
        require(hasFinding(report, "ticket pack", "missing.otp"),
                "missing OMSI-root ticket pack not reported");
        require(hasFinding(report, "currency", "missing.cfg"),
                "missing currency configuration not reported");
        const auto currencyDirectoryMap = root / "Maps" / "CurrencyDirectory";
        write(currencyDirectoryMap / "global.cfg",
              "[map]\n0\n0\ntile_0_0.map\n[ticketpack]\n"
              "TicketPacks\\directory.otp\n[moneysystem]\nMoney\\currency.cfg\n");
        write(currencyDirectoryMap / "tile_0_0.map", "[version]\n14\n[terrain]\n");
        std::filesystem::create_directories(root / "TicketPacks" / "directory.otp");
        write(root / "Money" / "currency.cfg",
              "[currency]\nCredits\n2\n[coin]\ncoin-directory.o3d\n1\n");
        std::filesystem::create_directories(root / "Money" / "coin-directory.o3d");
        const auto currencyDirectoryReport =
            openbus::map::validateMapAssets(currencyDirectoryMap, root);
        require(std::any_of(currencyDirectoryReport.findings.begin(),
                            currencyDirectoryReport.findings.end(), [&](const auto& finding) {
                                return finding.category == "currency model" &&
                                       finding.source == root / "Money" / "currency.cfg" &&
                                       finding.reference == "coin-directory.o3d" &&
                                       finding.detail.find("not a regular file") !=
                                           std::string::npos;
                            }) &&
                    std::any_of(currencyDirectoryReport.findings.begin(),
                                currencyDirectoryReport.findings.end(), [&](const auto& finding) {
                                    return finding.category == "ticket pack" &&
                                           finding.reference.find("directory.otp") !=
                                               std::string::npos &&
                                           finding.detail.find("not a regular file") !=
                                               std::string::npos;
                                }),
                "currency model and ticket-pack directories were not distinguished from missing files");
        require(std::none_of(report.findings.begin(), report.findings.end(),
                             [](const auto& finding) {
                                 return finding.category == "ground texture" &&
                                        finding.reference == "Texture\\grass.bmp" &&
                                        finding.detail == "Referenced file was not found";
                             }),
                "ground texture format fallback was not honored");
        require(hasFinding(report, "AI vehicle", "missing.bus"), "missing AI vehicle not reported");
        require(hasFinding(report, "AI vehicle", "missing-depot.bus"),
                "missing depot bus not reported");
        require(hasFinding(report, "AI train", "missing-base-train.zug") &&
                    hasFinding(report, "AI train vehicle", "missing-base-train-car.ovh") &&
                    hasFinding(report, "AI train vehicle", "missing-bare-train-car.ovh"),
                "base AI train consists and their vehicle entries were not checked");
        require(std::any_of(report.findings.begin(), report.findings.end(),
                            [&](const auto& finding) {
                                return finding.unsupported &&
                                       finding.category == "unsupported section" &&
                                       finding.source == root / "Trains" / "base-fixture.zug" &&
                                       finding.reference == "future_train_section";
                            }),
                "unsupported train-consist sections were not reported");
        require(hasFinding(report, "mesh", "missing-car.o3d"), "missing AI mesh not reported");
        require(hasFinding(report, "vehicle texture", "missing-alpha-vehicle.bmp") &&
                    hasFinding(report, "vehicle texture", "missing-environment.bmp") &&
                    hasFinding(report, "vehicle texture", "missing-item-night.bmp") &&
                    hasFinding(report, "vehicle texture", "base.bmp") &&
                    hasFinding(report, "vehicle texture", "other.bmp") &&
                    hasFinding(report, "vehicle texture", "broken-vehicle.bmp"),
                "missing or undecodable configured vehicle textures were not reported");
        require(!hasFinding(report, "vehicle texture", "EXISTING-night.bmp") &&
                    !hasFinding(report, "vehicle texture", "\\S:") &&
                    !hasFinding(report, "vehicle texture", "generated-placeholder.bmp") &&
                    !hasFinding(report, "vehicle texture", "unused-environment.bmp"),
                "vehicle texture fallback or generated texture handling failed");
        require(
            hasFinding(report, "vehicle repaint texture", "missing-ctc-texture.bmp") &&
            std::any_of(report.findings.begin(), report.findings.end(),
                        [](const auto& finding) {
                            return finding.category == "vehicle repaint texture" &&
                                   finding.reference.find("directory-repaint.bmp") !=
                                       std::string::npos &&
                                   finding.detail.find("not a regular file") !=
                                       std::string::npos;
                        }) &&
            !hasFinding(report, "vehicle repaint texture", "pr\xc3\xa4sent-repaint.bmp") &&
                !hasFinding(report, "vehicle repaint texture", "compressed-repaint.bmp") &&
                !hasFinding(report, "vehicle repaint texture", "compressed-repaint.dds") &&
                hasFinding(report, "vehicle repaint texture", "broken-repaint.bmp") &&
                !hasFinding(report, "vehicle repaint texture", "missing-unused-ctc-texture.bmp"),
            "CTI repaint textures were not checked against declared model slots");
         require(std::any_of(report.findings.begin(), report.findings.end(),
                            [&](const auto& finding) {
                                return finding.category == "vehicle appearance" &&
                                       finding.source == root / "Vehicles" / "ctc" / "ctc.ovh" &&
                                       finding.reference.find("directory.cti") !=
                                           std::string::npos &&
                                       finding.detail.find("not a regular file") !=
                                           std::string::npos;
                            }),
                "non-file CTI appearance path was silently skipped");
        require(hasFinding(report, "vehicle texture", "missing-vehicle-x.bmp") &&
                    hasFinding(report, "vehicle texture", "missing-vehicle-obj.bmp") &&
                    hasFinding(report, "vehicle texture", "missing-vehicle-o3d.bmp") &&
                    hasFinding(report, "vehicle texture", "missing-vehicle-bundle.bmp"),
                "missing unambiguous mesh-embedded vehicle textures were not reported");
        require(!hasFinding(report, "vehicle texture", "EXISTING-embedded.bmp") &&
                    !hasFinding(report, "vehicle texture", "unused-vehicle-obj.bmp") &&
                    !hasFinding(report, "vehicle texture", "overridden-vehicle-source.bmp") &&
                    !hasFinding(report, "vehicle texture", "repaint-source.bmp"),
                "unused, generated, repaintable or existing mesh textures were misreported");
        require(std::any_of(report.findings.begin(), report.findings.end(),
                            [](const auto& finding) {
                                return finding.category == "vehicle texture" &&
                                       finding.reference == "missing-vehicle-bundle.bmp" &&
                                       finding.detail.find("embedded.obj") != std::string::npos;
                            }),
                "bundle texture finding did not identify its entry");
        require(std::count_if(report.findings.begin(), report.findings.end(),
                              [](const auto& finding) {
                                  return finding.category == "vehicle texture" &&
                                         finding.reference == "missing-alpha-vehicle.bmp";
                              }) == 1,
                "shared vehicle model texture finding was duplicated");
        write(root / "Vehicles" / "car" / "texture" / "missing-alpha-vehicle.dds",
              onePixelBmp());
        write(root / "Vehicles" / "embedded" / "texture" / "missing-vehicle-x.dds",
              onePixelBmp());
        write(root / "Scenery" / "script" / "missing-vars.txt", "installed\n");
        const auto refreshedReport = openbus::map::validateMapAssets(map, root);
        require(!hasFinding(refreshedReport, "vehicle texture", "missing-alpha-vehicle.bmp") &&
                    !hasFinding(refreshedReport, "vehicle texture", "missing-vehicle-x.bmp") &&
                    !hasFinding(refreshedReport, "variable list", "missing-vars.txt"),
                "new validation scan did not see an installed vehicle texture");
        require(hasFinding(report, "mesh", "missing-trailer.o3d") &&
                    hasFinding(report, "script", "missing-trailer.osc"),
                "coupled vehicle dependencies were not checked");
        require(hasFinding(report, "coupled vehicle", "absent-trailer.ovh") &&
                    std::any_of(report.findings.begin(), report.findings.end(),
                                [&](const auto& finding) {
                                    return finding.category == "coupled vehicle" &&
                                           finding.source == root / "Vehicles" / "car" / "car2.ovh";
                                }),
                "missing coupled vehicle was not reported with its referring configuration");
        const auto relativeRoot = std::filesystem::relative(root, std::filesystem::current_path());
        const auto relativeReport = openbus::map::validateMapAssets(map, relativeRoot);
        require(hasFinding(relativeReport, "mesh", "missing-trailer.o3d") &&
                    hasFinding(relativeReport, "coupled vehicle", "absent-trailer.ovh"),
                "coupled vehicle traversal failed with a relative OMSI root");
        require(!hasFinding(report, "collision mesh", "good.obj"),
                "valid vehicle collision mesh was reported");
        require(hasFinding(report, "collision mesh", "empty.obj") &&
                    hasFinding(report, "collision mesh", "invalid-index.obj") &&
                    hasFinding(report, "collision mesh", "nonfinite.obj"),
                "unusable vehicle collision triangles were not reported");
        require(!hasFinding(report, "collision mesh", "mixed.obj"),
                "vehicle collision mesh with a usable triangle was reported");
        require(hasFinding(report, "collision mesh", "corrupt-collision.o3d"),
                "corrupt vehicle collision mesh was not reported");
        require(!hasFinding(report, "collision mesh", "present.o3d") &&
                    hasFinding(report, "collision mesh", "missing-bundle.o3d"),
                "vehicle collision bundle entries were not checked");
        require(hasFinding(report, "vehicle resource", "missing.cfg"),
                "missing AI sound configuration not reported");
        require(std::any_of(report.findings.begin(), report.findings.end(),
                            [&](const auto& finding) {
                                return finding.category == "vehicle resource" &&
                                       finding.source == root / "Vehicles" / "car" / "car2.ovh" &&
                                       finding.reference.find("directory.cfg") !=
                                           std::string::npos &&
                                       finding.detail.find("not a regular file") !=
                                           std::string::npos;
                            }),
                "vehicle sound configuration directory was not distinguished from a missing file");
        for (const std::string reference : {"missing-numbers.org", "missing-registrations.org"}) {
            require(std::count_if(report.findings.begin(), report.findings.end(),
                                  [&](const auto& finding) {
                                      return finding.category == "vehicle resource" &&
                                             finding.source == root / "Vehicles" / "car" / "car.ovh" &&
                                             finding.reference.find(reference) !=
                                                 std::string::npos;
                                  }) == 1 &&
                        !std::any_of(report.findings.begin(), report.findings.end(),
                                     [&](const auto& finding) {
                                         return finding.category != "vehicle resource" &&
                                                finding.source ==
                                                    root / "Vehicles" / "car" / "car.ovh" &&
                                                finding.detail.find(reference) !=
                                                    std::string::npos;
                                     }),
                    "missing vehicle registration resource was not reported once");
        }
        require(containsSource(root / "Vehicles" / "car" / "model" / "invalid-cabin.cfg") &&
                    std::any_of(report.findings.begin(), report.findings.end(),
                                [&](const auto& finding) {
                                    return finding.category == "passenger cabin" &&
                                           finding.source ==
                                               root / "Vehicles" / "car" / "model" /
                                                   "invalid-cabin.cfg" &&
                                           finding.detail.find("(line 1)") != std::string::npos;
                                }),
                "vehicle passenger-cabin config or its parser error was not inventoried");
        require(hasFinding(report, "passenger cabin", "missing-cabin.cfg") &&
                    std::count_if(report.findings.begin(), report.findings.end(),
                                  [&](const auto& finding) {
                                      return finding.category == "passenger cabin" &&
                                             finding.source ==
                                                 root / "Vehicles" / "car" / "car2.ovh" &&
                                             finding.reference.find("missing-cabin.cfg") !=
                                                 std::string::npos;
                                  }) == 1,
                "missing vehicle passenger-cabin config was not reported exactly once");
        require(!std::any_of(report.findings.begin(), report.findings.end(),
                             [&](const auto& finding) {
                                 return finding.category != "passenger cabin" &&
                                        finding.source == root / "Vehicles" / "car" / "car.ovh" &&
                                        finding.detail == "expected numeric values";
                             }),
                "vehicle passenger-cabin parser error was duplicated as a vehicle error");
        require(std::any_of(report.findings.begin(), report.findings.end(),
                            [&](const auto& finding) {
                                return finding.category == "passenger paths" &&
                                       finding.source ==
                                           root / "Vehicles" / "car" / "model" / "paths.cfg" &&
                                       finding.detail.find("(line 6)") != std::string::npos;
                            }) &&
                    !std::any_of(report.findings.begin(), report.findings.end(),
                                 [&](const auto& finding) {
                                     return finding.category != "passenger paths" &&
                                            finding.source ==
                                                root / "Vehicles" / "car" / "car.ovh" &&
                                            finding.detail.find("sound-pack index") !=
                                                std::string::npos;
                                 }),
                "passenger-path parser diagnostics were not attributed to paths.cfg");
        require(std::count_if(report.findings.begin(), report.findings.end(),
                              [&](const auto& finding) {
                                  return finding.category == "passenger paths" &&
                                         finding.source ==
                                             root / "Vehicles" / "car" / "car2.ovh" &&
                                         finding.reference.find("missing-paths.cfg") !=
                                             std::string::npos;
                              }) == 1,
                "missing passenger paths config was not reported exactly once");
        require(hasFinding(report, "sound", "missing-loop.wav"), "missing loop sound not reported");
        require(hasFinding(report, "sound", "missing-later.wav"),
                "sound scan did not continue after a malformed entry");
        require(std::any_of(report.findings.begin(), report.findings.end(),
                            [](const auto& finding) {
                                return finding.category == "sound" &&
                                       finding.reference == "directory.wav" &&
                                       finding.detail.find("not a regular file") !=
                                           std::string::npos;
                            }),
                "sound configuration directory reference was reported as missing");
        require(hasFinding(report, "sound", "broken.wav"),
                "undecodable sound file was not reported");
        require(!hasFinding(report, "step sound", "present-step.wav"),
                "present passenger step sound was reported missing");
        require(hasFinding(report, "step sound", "missing-step.wav"),
                "missing passenger step sound was not reported");
        require(std::any_of(report.findings.begin(), report.findings.end(),
                            [](const auto& finding) {
                                return finding.category == "step sound" &&
                                       finding.reference == "directory-step.wav" &&
                                       finding.detail.find("not a regular file") !=
                                           std::string::npos;
                            }),
                "passenger step-sound directory reference was reported as missing");
        require(hasFinding(report, "step sound", "broken-step.wav"),
                "undecodable passenger step sound was not reported");
        require(!hasFinding(report, "sound", "good.WAV"), "existing sound was reported missing");
        require(!hasFinding(report, "sound", "ignored.wav"), "disabled sound was checked");
        require(!hasFinding(report, "sound", "0.5"),
                "blank dynamic sound filename was treated as a path");
        require(std::any_of(report.findings.begin(), report.findings.end(),
                            [](const auto& finding) {
                                return finding.category == "sound configuration" &&
                                       finding.detail.find("dynamic or missing") !=
                                           std::string::npos;
                            }),
                "blank dynamic sound entry was not reported as uncertain");
        require(std::count_if(report.findings.begin(), report.findings.end(),
                              [](const auto& finding) {
                                  return finding.category == "sound" &&
                                         finding.reference == "missing-loop.wav";
                              }) == 1,
                "shared sound configuration was scanned more than once");
        require(hasFinding(report, "sound configuration", "sound"),
                "malformed sound entry not reported");
        require(std::any_of(report.findings.begin(), report.findings.end(),
                            [&](const auto& finding) {
                                return finding.unsupported &&
                                       finding.category == "unsupported section" &&
                                       finding.source ==
                                           root / "Vehicles" / "car" / "sound" / "sound.cfg" &&
                                       finding.reference == "future_sound_keyword" &&
                                       finding.detail.find("asset references in this directive "
                                                           "are not checked") !=
                                           std::string::npos;
                            }),
                "unsupported sound-config keywords were not reported");
        require(hasFinding(report, "human", "missing.hum"), "missing human not reported");
        require(hasFinding(report, "driver", "missing-driver.hum"), "missing driver not reported");
        require(std::any_of(report.findings.begin(), report.findings.end(),
                            [&](const auto& finding) {
                                return finding.unsupported &&
                                       finding.category == "unsupported list entry" &&
                                       finding.source == map / "humans.txt" &&
                                       finding.reference == "not-a-human-reference" &&
                                       finding.detail.find("line 3") != std::string::npos;
                            }) &&
                    std::any_of(report.findings.begin(), report.findings.end(),
                                [&](const auto& finding) {
                                    return finding.unsupported &&
                                           finding.category == "unsupported list entry" &&
                                           finding.source == map / "drivers.txt" &&
                                           finding.reference == "not-a-driver-reference";
                                }),
                "non-path entries in human and driver lists were silently ignored");
        require(hasFinding(report, "mesh", "missing-person.o3d"),
                "missing human mesh not reported");
        require(hasFinding(report, "model texture", "missing-person.bmp"),
                "missing human material texture not reported");
        require(std::any_of(report.findings.begin(), report.findings.end(),
                            [&](const auto& finding) {
                                return finding.unsupported &&
                                       finding.category == "unsupported section" &&
                                       finding.source == root / "Humans" / "person" / "person.hum" &&
                                       finding.reference == "future_human_keyword";
                            }),
                "unsupported human-config keywords were not reported");

        const auto aiDepotMap = root / "Maps" / "AiDepotHof";
        write(aiDepotMap / "global.cfg", "[map]\n0\n0\ntile_0_0.map\n");
        write(aiDepotMap / "tile_0_0.map",
              "[version]\n14\n[terrain]\n[water]\n"
              "[object]\n0\nScenery\\unsupported.sco\n101\n1\n2\n3\n0\n0\n0\n0\n0\n");
        write(aiDepotMap / "ailists.cfg",
              "[aigroup_2]\nPresent Group\nExpectedHof\n"
              "Vehicles\\hofcheck\\bus.bus\n[end]\n"
              "[aigroup_2]\nMissing Group\nAbsentGroupHof\n"
              "Vehicles\\hofcheck\\bus.bus\n[end]\n"
              "[aigroup_depot]\nPresent Depot\nExpectedHof\n"
              "[aigroup_depot_typgroup_2]\nVehicles\\hofcheck\\bus.bus\n[end]\n"
              "[aigroup_depot]\nMissing Depot\nAbsentHof\n"
              "[aigroup_depot_typgroup_2]\nVehicles\\hofcheck\\bus.bus\n[end]\n");
        {
            std::ofstream aiList(aiDepotMap / "ailists.cfg", std::ios::binary | std::ios::app);
            aiList << "[unsupported_ai_option]\nopaque payload\n";
        }
        write(root / "Scenery" / "unsupported.sco",
              "[model]\nmodel\\unsupported.cfg\n[future_sco_setting]\nopaque value\n");
        write(root / "Scenery" / "model" / "unsupported.cfg",
              "[mesh]\nunsupported.o3d\n[future_model_keyword]\nopaque value\n");
        write(root / "Scenery" / "model" / "unsupported.o3d", o3dMesh());
        write(root / "Vehicles" / "hofcheck" / "bus.bus",
              "[model]\nmodel\\check.cfg\n[future_vehicle_keyword]\nopaque value\n");
        write(root / "Vehicles" / "hofcheck" / "model" / "check.cfg",
              "[mesh]\ncheck.o3d\n");
        write(root / "Vehicles" / "hofcheck" / "model" / "check.o3d", o3dMesh());
        write(root / "Vehicles" / "hofcheck" / "expected.hof",
              "[name]\nExpectedHof\n[future_hof_section]\nFuture payload\n");
        std::filesystem::create_directories(root / "Vehicles" / "hofcheck" / "directory.hof");
        const auto aiDepotReport = openbus::map::validateMapAssets(aiDepotMap, root);
        require(hasFinding(aiDepotReport, "AI depot HOF", "AbsentHof") &&
                    hasFinding(aiDepotReport, "AI group HOF", "AbsentGroupHof") &&
                    !hasFinding(aiDepotReport, "AI group HOF", "ExpectedHof"),
                "AI group/depot HOF associations were not checked against the configured bus files");
        require(std::any_of(aiDepotReport.findings.begin(), aiDepotReport.findings.end(),
                            [&](const auto& finding) {
                                return finding.category == "vehicle depot" &&
                                       finding.source == root / "Vehicles" / "hofcheck" /
                                                            "bus.bus" &&
                                       finding.reference.ends_with("directory.hof") &&
                                       finding.detail.find("not a regular file") !=
                                           std::string::npos;
                            }),
                "non-file HOF path was silently skipped during AI depot validation");
        require(std::any_of(aiDepotReport.findings.begin(), aiDepotReport.findings.end(),
                            [&](const auto& finding) {
                                return finding.unsupported &&
                                       finding.category == "unsupported section" &&
                                       finding.source ==
                                           root / "Vehicles" / "hofcheck" / "expected.hof" &&
                                       finding.reference == "future_hof_section";
                            }),
                "unsupported HOF sections were not reported");
        require(std::any_of(aiDepotReport.findings.begin(), aiDepotReport.findings.end(),
                            [&](const auto& finding) {
                                return finding.unsupported &&
                                       finding.category == "unsupported section" &&
                                       finding.source == aiDepotMap / "ailists.cfg" &&
                                       finding.reference == "unsupported_ai_option";
                            }),
                "unsupported base AI-list sections were not reported");
        require(std::any_of(aiDepotReport.findings.begin(), aiDepotReport.findings.end(),
                            [&](const auto& finding) {
                                return finding.unsupported &&
                                       finding.category == "unsupported section" &&
                                       finding.source == root / "Scenery" / "unsupported.sco" &&
                                       finding.reference == "future_sco_setting";
                            }),
                "unsupported scenery-object keywords were not reported");
        require(std::any_of(aiDepotReport.findings.begin(), aiDepotReport.findings.end(),
                            [&](const auto& finding) {
                                return finding.unsupported &&
                                       finding.category == "unsupported section" &&
                                       finding.source ==
                                           root / "Scenery" / "model" / "unsupported.cfg" &&
                                       finding.reference == "future_model_keyword";
                            }),
                "unsupported model-config keywords were not reported");
        require(std::any_of(aiDepotReport.findings.begin(), aiDepotReport.findings.end(),
                            [&](const auto& finding) {
                                return finding.unsupported &&
                                       finding.category == "unsupported section" &&
                                       finding.source == root / "Vehicles" / "hofcheck" / "bus.bus" &&
                                       finding.reference == "future_vehicle_keyword";
                            }),
                "unsupported vehicle-config keywords were not reported");

        const auto cleanMap = root / "Maps" / "Clean";
        write(cleanMap / "global.cfg",
              "[map]\n0\n0\ntile_0_0.map\n"
              "[ticketpack]\nticketpacks\\GOOD.otp\n[moneysystem]\nmoney\\GOOD.cfg\n");
        write(root / "TicketPacks" / "good.otp", "[ticketpack]\n0.5\n");
        write(root / "Money" / "good.cfg", "[currency]\nCredits\n2\n[coin]\ncoins\\GOOD.o3d\n0.1\n"
                                           "[bill]\nnotes\\GOOD.o3d\n5\n");
        write(root / "Money" / "coins" / "good.o3d", texturedO3dMesh("good-coin.bmp"));
        write(root / "Money" / "coins" / "good-coin.bmp", onePixelBmp());
        write(root / "Money" / "notes" / "good.o3d", texturedO3dMesh("good-bill.bmp"));
        write(root / "Money" / "notes" / "good-bill.bmp", onePixelBmp());
        write(cleanMap / "tile_0_0.map", "[version]\n14\n[terrain]\n");
        const std::string validTerrain = std::string("\x01\x00\x00\x00", 4) + std::string(16, '\0');
        const std::string validWater(4, '\0');
        write(cleanMap / "tile_0_0.map.terrain", validTerrain);
        const auto cleanReport = openbus::map::validateMapAssets(cleanMap, root);
        require(cleanReport.findings.empty(), "complete direct fixture has unexpected findings");
        require(!cleanReport.complete, "clean partial report must remain incomplete");

        const auto invalidInputTypeMap = root / "Maps" / "InvalidInputType";
        write(invalidInputTypeMap / "global.cfg", "[map]\n0\n0\ntile_0_0.map\n");
        write(invalidInputTypeMap / "tile_0_0.map", "[version]\n14\n[terrain]\n");
        std::filesystem::create_directories(invalidInputTypeMap / "drivers.txt");
        std::filesystem::create_directories(invalidInputTypeMap / "tile_0_0.map.terrain");
        std::filesystem::create_directories(invalidInputTypeMap / "tile_0_0.map.terrain_0.rdy");
        const auto invalidInputTypeReport =
            openbus::map::validateMapAssets(invalidInputTypeMap, root);
        require(invalidInputTypeReport.tilesScanned == 1 &&
                    std::any_of(invalidInputTypeReport.findings.begin(),
                                invalidInputTypeReport.findings.end(),
                                [&](const auto& finding) {
                                    return finding.category == "map input" &&
                                           finding.source == invalidInputTypeMap / "drivers.txt" &&
                                           finding.detail.find("not a regular file") !=
                                               std::string::npos;
                                }),
                "non-file optional map inputs must be reported without aborting tile scanning");
        require(std::any_of(invalidInputTypeReport.findings.begin(),
                            invalidInputTypeReport.findings.end(),
                            [&](const auto& finding) {
                                return finding.category == "tile sidecar" &&
                                       finding.source == invalidInputTypeMap / "tile_0_0.map" &&
                                       finding.reference.ends_with(".terrain_0.rdy") &&
                                       finding.detail == "Tile sidecar is not a regular file";
                            }),
                "non-file terrain-readiness sidecar was reported as a different sidecar type");
        require(std::any_of(invalidInputTypeReport.findings.begin(),
                                        invalidInputTypeReport.findings.end(),
                                        [&](const auto& finding) {
                                            return finding.category == "terrain" &&
                                                   finding.reference.ends_with(".map.terrain") &&
                                                   finding.detail ==
                                                       "Sidecar path exists but is not a regular file";
                                        }) &&
                                std::none_of(invalidInputTypeReport.sourceFiles.begin(),
                                             invalidInputTypeReport.sourceFiles.end(),
                                             [&](const auto& source) {
                                                 return source.lexically_normal() ==
                                                        (invalidInputTypeMap / "tile_0_0.map.terrain")
                                                            .lexically_normal();
                                             }),
                            "non-file terrain sidecar was treated as missing or inventoried as a source file");

        const auto groundTextureSidecarMap = root / "Maps" / "GroundTextureSidecar";
        write(groundTextureSidecarMap / "global.cfg",
              "[map]\n0\n0\ntile_0_0.map\n"
              "[groundtex]\nTexture\\sky\\day.bmp\nTexture\\sky\\day.bmp\n0\n1\n60\n"
              "[groundtex]\nTexture\\sky\\day.bmp\nTexture\\sky\\day.bmp\n0\n1\n60\n");
        write(groundTextureSidecarMap / "tile_0_0.map", "[version]\n14\n");
        write(groundTextureSidecarMap / "texture" / "map" / "tile_0_0.map.1.dds",
              onePixelBmp());
        write(groundTextureSidecarMap / "texture" / "map" / "tile_0_0.map.2.dds",
              "invalid image");
        write(groundTextureSidecarMap / "texture" / "map" / "tile_0_0.map.3.dds",
              onePixelBmp());
        write(groundTextureSidecarMap / "texture" / "map" / "tile_0_0.map.bad.dds",
              onePixelBmp());
        write(groundTextureSidecarMap / "texture" / "map" / "tile_0_0.map..dds",
              onePixelBmp());
        const auto groundTextureDefinition =
            openbus::map::loadMapDefinition(groundTextureSidecarMap);
        require(groundTextureDefinition.groundTextures.size() == 2,
                "ground-texture sidecar fixture did not load both layers");
        require(groundTextureDefinition.tiles.front().groundTextureSidecars.size() == 3,
                "ground-texture sidecar fixture did not discover all files");
        const auto groundTextureSidecarReport =
            openbus::map::validateMapAssets(groundTextureSidecarMap, root);
        require(std::any_of(groundTextureSidecarReport.sourceFiles.begin(),
                            groundTextureSidecarReport.sourceFiles.end(),
                            [&](const auto& source) {
                                return std::filesystem::equivalent(
                                    source, groundTextureSidecarMap / "texture" / "map" /
                                                "tile_0_0.map.1.dds");
                            }) &&
                    !hasFinding(groundTextureSidecarReport, "ground texture sidecar",
                                "tile_0_0.map.1.dds") &&
                    hasFinding(groundTextureSidecarReport, "ground texture sidecar",
                               "tile_0_0.map.2.dds") &&
                    hasFinding(groundTextureSidecarReport, "ground texture sidecar",
                               "tile_0_0.map.3.dds"),
                "ground-texture sidecar decoding or layer-suffix validation failed");
        require(hasFinding(groundTextureSidecarReport, "ground texture sidecar",
                           "tile_0_0.map.bad.dds") &&
                    std::none_of(groundTextureSidecarReport.sourceFiles.begin(),
                                 groundTextureSidecarReport.sourceFiles.end(),
                                 [&](const auto& source) {
                                     return source.lexically_normal() ==
                                            (groundTextureSidecarMap / "texture" / "map" /
                                             "tile_0_0.map.bad.dds")
                                                .lexically_normal();
                                 }),
                "non-numeric ground-texture sidecar suffix was ignored or inventoried as an asset");
        require(std::any_of(groundTextureSidecarReport.findings.begin(),
                            groundTextureSidecarReport.findings.end(),
                            [&](const auto& finding) {
                                return finding.category == "ground texture sidecar" &&
                                       finding.reference.ends_with("tile_0_0.map..dds") &&
                                       finding.detail == "Ground-texture sidecar suffix is empty";
                            }),
                "empty ground-texture sidecar suffix was ignored");

        const auto unsupportedSupportMap = root / "Maps" / "UnsupportedSupport";
        write(unsupportedSupportMap / "global.cfg",
              "[map]\n0\n0\ntile_0_0.map\n"
              "[backgroundimage]\n0\n0\n0\n0\n0\n0\n"
              "[ticketpack]\nTicketPacks\\unsupported.otp\n"
              "[moneysystem]\nMoney\\unsupported.cfg\n");
        write(unsupportedSupportMap / "tile_0_0.map", "[version]\n14\n[terrain]\n");
        write(unsupportedSupportMap / "tile_0_0.map.terrain", validTerrain);
        write(root / "TicketPacks" / "unsupported.otp",
              "[ticketpack]\n0.5\n[future_ticket_section]\nFuture payload\n");
        write(root / "Money" / "unsupported.cfg",
              "[currency]\nCredits\n2\n[future_currency_section]\nFuture payload\n");
        const auto unsupportedSupportReport =
            openbus::map::validateMapAssets(unsupportedSupportMap, root);
        require(std::any_of(unsupportedSupportReport.findings.begin(),
                            unsupportedSupportReport.findings.end(), [&](const auto& finding) {
                                return finding.unsupported &&
                                       finding.category == "unsupported section" &&
                                       finding.source == root / "TicketPacks" / "unsupported.otp" &&
                                       finding.reference == "future_ticket_section";
                            }) &&
                    std::any_of(unsupportedSupportReport.findings.begin(),
                                unsupportedSupportReport.findings.end(),
                                [&](const auto& finding) {
                                    return finding.unsupported &&
                                           finding.category == "unsupported section" &&
                                           finding.source == root / "Money" / "unsupported.cfg" &&
                                           finding.reference == "future_currency_section";
                                }) &&
                    std::any_of(unsupportedSupportReport.findings.begin(),
                                unsupportedSupportReport.findings.end(),
                                [&](const auto& finding) {
                                    return finding.unsupported &&
                                           finding.category == "unsupported section" &&
                                           finding.source ==
                                               unsupportedSupportMap / "global.cfg" &&
                                           finding.reference == "backgroundimage" &&
                                           finding.detail.find("asset references are not interpreted") !=
                                               std::string::npos;
                                }),
                "unsupported support-config or unchecked background-image references were not reported");

        const auto malformedSourceMap = root / "Maps" / "MalformedSourceInventory";
        write(malformedSourceMap / "global.cfg",
              "[map]\n0\n0\ntile_0_0.map\n[unsupported_global_section]\nvalue\n");
        write(malformedSourceMap / "tile_0_0.map",
              "[version]\n14\n[terrain]\n[water]\n[unsupported_tile_section]\nvalue\n");
        write(malformedSourceMap / "tile_0_0.map.terrain", "bad terrain");
        write(malformedSourceMap / "tile_0_0.map.water", "bad water");
        write(malformedSourceMap / "tile_0_0.map.LM.bmp", "bad bitmap");
        const auto malformedSourceReport =
            openbus::map::validateMapAssets(malformedSourceMap, root);
        const auto hasSourcePath = [&](const std::filesystem::path& path) {
            return std::any_of(malformedSourceReport.sourceFiles.begin(),
                               malformedSourceReport.sourceFiles.end(), [&](const auto& source) {
                                   return std::filesystem::equivalent(source, path);
                               });
        };
        require(hasSourcePath(malformedSourceMap / "tile_0_0.map") &&
                    hasSourcePath(malformedSourceMap / "tile_0_0.map.terrain") &&
                    hasSourcePath(malformedSourceMap / "tile_0_0.map.water") &&
                    hasSourcePath(malformedSourceMap / "tile_0_0.map.LM.bmp"),
                "source inventory omitted malformed tile or sidecar inputs");
        require(hasFinding(malformedSourceReport, "terrain", "tile_0_0.map.terrain") &&
                    hasFinding(malformedSourceReport, "water", "tile_0_0.map.water") &&
                    hasFinding(malformedSourceReport, "lightmap",
                               "tile_0_0.map.LM.bmp"),
                "malformed tile sidecars or lightmaps were not reported");
        const auto hasUnsupportedSection = [&](const std::filesystem::path& source,
                                                const std::string& reference) {
            return std::any_of(malformedSourceReport.findings.begin(),
                               malformedSourceReport.findings.end(), [&](const auto& finding) {
                                   return finding.unsupported && finding.source == source &&
                                          finding.reference == reference;
                               });
        };
        require(hasUnsupportedSection(malformedSourceMap / "global.cfg",
                                      "unsupported_global_section") &&
                    hasUnsupportedSection(malformedSourceMap / "tile_0_0.map",
                                          "unsupported_tile_section"),
                "unsupported map sections were not explicitly classified");
        require(std::any_of(report.findings.begin(), report.findings.end(),
                            [&](const auto& finding) {
                                return finding.unsupported &&
                                       finding.category == "unsupported section" &&
                                       finding.source == map / "environment" / "clouds.cfg" &&
                                       finding.reference == "unsupported_environment";
                            }),
                "unsupported environment configuration section was not reported");

        const auto supportMap = root / "Maps" / "Support";
        write(supportMap / "global.cfg",
              "[map]\n0\n0\ntile_0_0.map\n"
              "[ticketpack]\nTicketPacks\\broken.otp\n[moneysystem]\nMoney\\broken.cfg\n");
        write(supportMap / "tile_0_0.map", "[version]\n14\n[terrain]\n");
        write(supportMap / "tile_0_0.map.terrain", validTerrain);
        write(root / "TicketPacks" / "broken.otp", std::string("\xff\xfe\x41", 3));
        write(root / "Money" / "broken.cfg",
              "[coin]\ncoin.o3d\ninvalid\n[bill]\ninvalid-bill.o3d\ninvalid\n"
              "[coin]\nmissing-coin.o3d\n1\n[coin]\nmissing-coin.o3d\n1\n"
              "[coin]\ncoins\\broken-texture.o3d\n2\n"
              "[bill]\nmissing-bill.o3d\n5\n[bill]\nnotes\\GOOD.o3d\n10\n");
        write(root / "missing-coin.o3d", "not next to currency");
        write(root / "Money" / "missing-coin.obj", "not the authored model");
        write(root / "Money" / "coins" / "broken-texture.o3d",
              texturedO3dMesh("missing-currency.bmp"));
        const auto supportReport = openbus::map::validateMapAssets(supportMap, root);
        require(supportReport.tilesScanned == 1, "support-config failure prevented tile scan");
        require(std::any_of(supportReport.findings.begin(), supportReport.findings.end(),
                            [&](const auto& finding) {
                                return finding.category == "ticket pack" &&
                                       finding.source == root / "TicketPacks" / "broken.otp" &&
                                       finding.detail.find("Odd-length UTF-16LE") !=
                                           std::string::npos;
                            }),
                "unreadable ticket pack was not reported");
        require(std::any_of(supportReport.findings.begin(), supportReport.findings.end(),
                            [&](const auto& finding) {
                                return finding.category == "currency" &&
                                       finding.source == root / "Money" / "broken.cfg" &&
                                       finding.reference == "coin" &&
                                       finding.detail.find("Parser warning:") != std::string::npos;
                            }),
                "invalid currency entry was not reported with parser warning");
        require(std::any_of(supportReport.findings.begin(), supportReport.findings.end(),
                            [&](const auto& finding) {
                                return finding.category == "currency" &&
                                       finding.reference == "bill" &&
                                       finding.detail.find("Parser warning:") != std::string::npos;
                            }),
                "invalid bill entry was not reported with parser warning");
        require(hasFinding(supportReport, "currency model", "missing-coin.o3d") &&
                    hasFinding(supportReport, "currency model", "missing-bill.o3d") &&
                    !hasFinding(supportReport, "currency model", "GOOD.o3d"),
                "currency coin and bill model references were not resolved correctly");
        require(hasFinding(supportReport, "model texture", "missing-currency.bmp") &&
                    !hasFinding(supportReport, "model texture", "good-coin.bmp") &&
                    !hasFinding(supportReport, "model texture", "good-bill.bmp"),
                "currency model material textures were not validated");
        require(std::count_if(supportReport.findings.begin(), supportReport.findings.end(),
                              [&](const auto& finding) {
                                  return finding.category == "currency model" &&
                                         finding.source == root / "Money" / "broken.cfg" &&
                                         finding.reference == "missing-coin.o3d";
                              }) == 1,
                "duplicate currency model finding was not deduplicated");

        const auto sidecars = root / "Maps" / "Sidecars";
        write(sidecars / "global.cfg", "[map]\n0\n0\ntile_0_0.map\n[map]\n1\n0\ntile_1_0.map\n"
                                       "[map]\n2\n0\ntile_2_0.map\n[map]\n3\n0\ntile_3_0.map\n"
                                       "[map]\n4\n0\ntile_4_0.map\n[map]\n5\n0\ntile_5_0.map\n"
                                       "[map]\n6\n0\ntile_6_0.map\n");
        write(sidecars / "tile_0_0.map", "[version]\n14\n[terrain]\n[water]\n");
        write(sidecars / "tile_0_0.map.terrain", validTerrain);
        write(sidecars / "tile_0_0.map.water", validWater);
        write(sidecars / "tile_1_0.map", "[version]\n14\n[terrain]\n");
        write(sidecars / "tile_1_0.map.terrain", "bad");
        write(sidecars / "tile_2_0.map", "[version]\n14\n[terrain]\n[water]\n");
        write(sidecars / "tile_2_0.map.terrain", validTerrain);
        write(sidecars / "tile_2_0.map.water", std::string("\x01\x00\x00\x00", 4));
        write(sidecars / "tile_3_0.map", "[version]\n14\n[terrain]\n[water]\n");
        write(sidecars / "tile_3_0.map.terrain", validTerrain);
        write(sidecars / "tile_4_0.map", "[version]\n14\n[terrain]\n[water]\n");
        const std::string nonFinite("\x00\x00\xc0\x7f", 4);
        write(sidecars / "tile_4_0.map.terrain",
              std::string("\x01\x00\x00\x00", 4) + nonFinite + std::string(12, '\0'));
        write(sidecars / "tile_4_0.map.water",
              std::string("\x01\x00\x00\x00", 4) + nonFinite + std::string(12, '\0'));
        write(sidecars / "tile_5_0.map", "[version]\n14\n[terrain]\n");
        write(sidecars / "tile_5_0.map.terrain", validTerrain);
        write(sidecars / "tile_6_0.map", "[version]\ninvalid\n");
        write(sidecars / "tile_6_0.map.terrain", "bad");
        write(sidecars / "tile_6_0.map.water", "bad");
        std::filesystem::create_directories(
            sidecars / "texture" / "map" / "tile_0_0.map.4.dds");
        write(sidecars / "texture" / "water.tga", onePixelBmp());
        write(sidecars / "texture" / "water_bump.bmp", onePixelBmp());
        const auto sidecarReport = openbus::map::validateMapAssets(sidecars, root);
        const auto sidecarFinding = [&](const std::string& category, const std::string& tile,
                                        const std::string& detail) {
            return std::any_of(sidecarReport.findings.begin(), sidecarReport.findings.end(),
                               [&](const auto& finding) {
                                   return finding.category == category &&
                                          finding.source == sidecars / tile &&
                                          finding.detail.find(detail) != std::string::npos;
                               });
        };
        require(sidecarReport.tilesScanned == 6, "sidecar failures prevented later tile scans");
        require(!hasFinding(sidecarReport, "terrain", "tile_0_0.map.terrain") &&
                    !hasFinding(sidecarReport, "water", "tile_0_0.map.water"),
                "valid sidecars were reported as invalid");
        require(hasFinding(sidecarReport, "terrain", "tile_1_0.map.terrain") &&
                    sidecarFinding("terrain", "tile_1_0.map", "shorter than its header"),
                "truncated terrain was not reported");
        require(hasFinding(sidecarReport, "water", "tile_2_0.map.water") &&
                    sidecarFinding("water", "tile_2_0.map", "size does not match"),
                "truncated water was not reported");
        require(hasFinding(sidecarReport, "water", "tile_3_0.map.water") &&
                    sidecarFinding("water", "tile_3_0.map", "Water file was not found"),
                "missing water sidecar was not reported");
        require(hasFinding(sidecarReport, "terrain", "tile_4_0.map.terrain") &&
                    hasFinding(sidecarReport, "water", "tile_4_0.map.water") &&
                    sidecarFinding("terrain", "tile_4_0.map", "Non-finite terrain height") &&
                    sidecarFinding("water", "tile_4_0.map", "Non-finite water height"),
                "non-finite sidecar heights were not reported");
        require(!hasFinding(sidecarReport, "water", "tile_5_0.map.water"),
                "optional water sidecar was incorrectly required");
        require(hasFinding(sidecarReport, "terrain", "tile_6_0.map.terrain") &&
                    hasFinding(sidecarReport, "water", "tile_6_0.map.water"),
                "sidecars were not checked when their tile could not be parsed");
        require(std::any_of(sidecarReport.findings.begin(), sidecarReport.findings.end(),
                            [&](const auto& finding) {
                                return finding.category == "ground texture sidecar" &&
                                       finding.source == sidecars / "tile_0_0.map" &&
                                       finding.reference.ends_with("tile_0_0.map.4.dds") &&
                                       finding.detail.find("not a regular file") !=
                                           std::string::npos;
                            }) &&
                    std::none_of(sidecarReport.sourceFiles.begin(), sidecarReport.sourceFiles.end(),
                                 [&](const auto& source) {
                                     return source.lexically_normal() ==
                                            (sidecars / "texture" / "map" /
                                             "tile_0_0.map.4.dds")
                                                .lexically_normal();
                                 }),
                "non-file base ground-texture sidecar was ignored or inventoried as an asset");
        const auto containsSidecarSource = [&](const std::filesystem::path& path) {
            return std::any_of(sidecarReport.sourceFiles.begin(), sidecarReport.sourceFiles.end(),
                               [&](const auto& source) {
                                   return source.lexically_normal() == path.lexically_normal();
                               });
        };
        require(containsSidecarSource(sidecars / "texture" / "water.tga") &&
                    containsSidecarSource(sidecars / "texture" / "water_bump.bmp") &&
                    !containsSidecarSource(sidecars / "texture" / "water_envmap.bmp") &&
                    !hasFinding(sidecarReport, "water texture", "water.tga") &&
                    !hasFinding(sidecarReport, "water texture", "water_bump.bmp") &&
                    !hasFinding(sidecarReport, "water texture", "water_envmap.bmp"),
                "map water textures were not inventoried or absent optional textures were reported");
        const auto sidecarManifestSources = openbus::map::makeMapBakeSourceFiles(
            sidecarReport.sourceFiles, sidecars, root);
        require(std::any_of(sidecarManifestSources.begin(), sidecarManifestSources.end(),
                            [](const auto& source) {
                                return source.key == "map/texture/water.tga";
                            }) &&
                    std::any_of(sidecarManifestSources.begin(), sidecarManifestSources.end(),
                                [](const auto& source) {
                                    return source.key == "map/texture/water_bump.bmp";
                                }),
                "map water textures were omitted from bake manifest inputs");
        write(sidecars / "texture" / "water_bump.bmp", "invalid image");
        const auto invalidWaterTextureReport =
            openbus::map::validateMapAssets(sidecars, root);
        require(hasFinding(invalidWaterTextureReport, "water texture", "water_bump.bmp"),
                "invalid map water texture was not reported");

        const auto invalidTimetableDirectory = root / "Maps" / "InvalidTimetableDirectory";
        write(invalidTimetableDirectory / "global.cfg", "[map]\n0\n0\ntile_0_0.map\n");
        write(invalidTimetableDirectory / "tile_0_0.map", "[version]\n14\n[terrain]\n");
        write(invalidTimetableDirectory / "TTData", "not a directory");
        const auto invalidTimetableReport =
            openbus::map::validateMapAssets(invalidTimetableDirectory, root);
        require(invalidTimetableReport.tilesScanned == 1,
                "invalid timetable source path stopped the tile scan");
        require(std::any_of(invalidTimetableReport.findings.begin(),
                            invalidTimetableReport.findings.end(),
                            [&](const auto& finding) {
                                return finding.category == "timetable source" &&
                                       finding.source == invalidTimetableDirectory / "TTData" &&
                                       finding.detail.find("not a directory") != std::string::npos;
                            }),
                "invalid timetable source path was silently ignored");

        const auto invalidGroundTextureDirectory = root / "Maps" / "InvalidGroundTextureDirectory";
        write(invalidGroundTextureDirectory / "global.cfg", "[map]\n0\n0\ntile_0_0.map\n");
        write(invalidGroundTextureDirectory / "tile_0_0.map", "[version]\n14\n[terrain]\n");
        write(invalidGroundTextureDirectory / "texture" / "map", "not a directory");
        const auto invalidGroundTextureReport =
            openbus::map::validateMapAssets(invalidGroundTextureDirectory, root);
        require(invalidGroundTextureReport.tilesScanned == 1 &&
                    std::any_of(invalidGroundTextureReport.findings.begin(),
                                invalidGroundTextureReport.findings.end(),
                                [&](const auto& finding) {
                                    return finding.category == "ground texture sidecar" &&
                                           finding.source ==
                                               invalidGroundTextureDirectory / "texture" / "map" &&
                                           finding.detail.find(
                                               "Could not enumerate base ground-texture sidecars") ==
                                               0;
                                }),
                "invalid base ground-texture directory was silently ignored or stopped the scan");

        const auto chronoMap = root / "Maps" / "ChronoFixture";
        const auto scenario = chronoMap / "Chrono" / "Event";
        write(chronoMap / "global.cfg",
              "[map]\n0\n0\ntile_0_0.map\n"
              "[groundtex]\nTexture\\sky\\day.bmp\nTexture\\sky\\day.bmp\n0\n1\n60\n"
              "[groundtex]\nTexture\\sky\\day.bmp\nTexture\\sky\\day.bmp\n0\n1\n60\n");
        write(chronoMap / "ailists.cfg",
              "[aigroup_2]\nBase invalid HOF group\nAbsentChronoGroupHof\n"
              "Vehicles\\hofcheck\\bus.bus\n[end]\n");
        write(chronoMap / "tile_0_0.map",
              "[version]\n14\n[terrain]\n[object]\n0\nScenery\\present.sco\n101\n"
              "1\n2\n3\n0\n0\n0\n0\n0\n"
              "[object]\n0\nScenery\\park.sco\n102\n"
              "1\n2\n3\n0\n0\n0\n0\n0\n"
              "[spline]\n0\nSplines\\road.sli\n200\n0\n0\n0\n0\n0\n0\n"
              "10\n0\n0\n0\n0\n0\n0\n0\n0\n");
        write(chronoMap / "tile_0_0.map.terrain", validTerrain);
        write(chronoMap / "parklist_p.txt", "Scenery\\parked.sco\n");
        write(chronoMap / "TTData" / "TrackRefs.ttp",
              "[trip]\nTrackRefs\nDestination\nLine\n"
              "[station]\n19003\n0\nStop\n0\n0\n0\n0\n0\n");
        write(chronoMap / "TTData" / "TrackRefs.ttr",
              "[track_entry]\n101\n0\n1\n1\n10\n0\n"
              "[track_entry]\n200\n0\n1\n2\n10\n0\n"
              "[track_entry]\n103\n0\n1\n3\n10\n0\n"
              "[track_entry]\n201\n0\n1\n4\n10\n0\n"
              "[track_entry]\n9999\n0\n1\n5\n10\n0\n");
        write(scenario / "Chrono.cfg",
              "[startdate]\n20990101\n[ticketpack]\nTicketPacks\\missing-chrono.otp\n"
              "[moneysystem]\nMoney\\missing-chrono.cfg\n"
              "[unsupported_chrono_setting]\nopaque value\n");
        write(scenario / "parklist_p_7.txt", "Scenery\\missing-chrono-parked.sco\n");
        write(scenario / "TTData" / "event.ttl", "[addtrip]\nChronoMissingTrip\n0\n400\n");
        write(scenario / "TTData" / "track.ttl",
              "[addtrip]\nChronoTrackTrip\n0\n410\n");
        write(scenario / "TTData" / "ChronoTrackTrip.ttp",
              "[trip]\nChronoTrackTrip\nDestination\nLine\n"
              "[station]\n19004\n0\nStop\n0\n0\n0\n0\n0\n");
        write(scenario / "TTData" / "ChronoTrackTrip.ttr",
              "[track_entry]\n103\n0\n1\n1\n10\n0\n"
              "[track_entry]\n201\n0\n1\n2\n10\n0\n"
              "[track_entry]\n9998\n0\n1\n3\n10\n0\n");
        write(scenario / "ailists_#upd.cfg", "[aigroup_2]\nChrono cars\n"
                                             "Vehicles\\missing-chrono-group.bus 2\n"
                                             "Vehicles\\missing-chrono-group.ovh 1\n[end]\n"
                                             "[ailist]\n"
                                             "Vehicles\\missing-chrono-ailist.bus\t3\n[end]\n"
                                             "[aigroup_2]\nChrono valid group\nExpectedHof\n"
                                             "Vehicles\\hofcheck\\bus.bus\n[end]\n"
                                             "[aigroup_2]\nChrono missing HOF group\n"
                                             "AbsentChronoGroupHof\n"
                                             "Vehicles\\hofcheck\\bus.bus\n[end]\n"
                                             "[aigroup_depot]\nChrono depot\n"
                                             "AbsentChronoDepotHof\n"
                                             "[aigroup_depot_typgroup_2]\n"
                                             "Vehicles\\hofcheck\\bus.bus\n[end]\n"
                                             "[aigroup_depot_typgroup_2]\n"
                                             "Vehicles\\missing-chrono-depot.bus\n"
                                             "123\tadvertisement\t20200101\n[end]\n"
                                             "[unsupported_chrono_ai]\nopaque payload\n"
                                             "[aigroup_2]\nChrono trains\n"
                                             "Trains\\fixture.zug\n"
                                             "Trains\\missing-chrono-train.zug\n[end]\n"
                                             "[ailist]\nTrains\\missing-chrono-ailist.zug\n[end]\n");
        write(scenario / "chrono_#upd.cfg", "[traffic_update]\nopaque payload\n");
        write(scenario / "humans_#upd.cfg", "Humans\\missing-chrono-human.hum\n");
        write(scenario / "drivers_#upd.cfg", "Humans\\missing-chrono-driver.hum\n");
        write(scenario / "humans.txt",
              "Humans\\missing-chrono-scenario-human.hum\nnot-a-scenario-human-reference\n");
        write(scenario / "drivers.txt", "Humans\\missing-chrono-scenario-driver.hum\n");
        write(root / "Trains" / "fixture.zug", "Vehicles\\missing-chrono-train-car.ovh\n0\n");
        write(scenario / "tile_0_0.map",
              "[version]\n14\n[selobject]\n101\n[typ]\nScenery\\chrono-replacement.sco\n"
              "[selobject]\n102\n[relabel]\n1\n7\n"
              "[selobject]\nbad-id\n[typ]\nScenery\\unreachable.sco\n"
              "[selspline]\n200\n[typ]\nSplines\\chrono-replacement.sli\n"
              "[object]\n0\nScenery\\chrono-added.sco\n103\n"
              "1\n2\n3\n0\n0\n0\n0\n0\n"
              "[object]\n0\nScenery\\present.sco\n999\n"
              "1\n2\n3\n0\n0\n0\n0\n0\n"
              "[object]\n0\nScenery\\park.sco\n998\n"
              "1\n2\n3\n0\n0\n0\n0\n0\n"
              "[spline]\n0\nSplines\\chrono-added.sli\n201\n0\n0\n0\n0\n0\n0\n"
              "10\n0\n0\n0\n0\n0\n0\n0\n0\n"
              "[unsupported_chrono_tile]\nvalue\n");
        write(scenario / "tile_0_0.map.terrain", "bad");
        write(scenario / "tile_0_0.map.water", "bad");
        write(scenario / "tile_0_0.map.LM.bmp", "bad bitmap");
        write(chronoMap / "texture" / "water.tga", onePixelBmp());
        write(scenario / "tile_0_0.map.prt", "opaque Chrono tile companion\n");
        write(scenario / "tile_0_0.map.terrain_0.rdy", "opaque Chrono readiness companion\n");
        write(scenario / "texture" / "map" / "tile_0_0.map.1.dds", onePixelBmp());
        write(scenario / "texture" / "map" / "tile_0_0.map.2.dds", "invalid image");
        write(scenario / "texture" / "map" / "tile_0_0.map.3.dds", onePixelBmp());
        std::filesystem::create_directories(
            scenario / "texture" / "map" / "tile_0_0.map.4.dds");
        write(scenario / "unlisted.map", "[version]\n14\n[object]\n0\nScenery\\unlisted.sco\n103\n"
                                         "1\n2\n3\n0\n0\n0\n0\n0\n");
        write(chronoMap / "Chrono" / "NoConfig" / "tile_0_0.map",
              "[version]\n14\n[object]\n0\nScenery\\no-config.sco\n103\n"
              "1\n2\n3\n0\n0\n0\n0\n0\n");
        const auto brokenScenario = chronoMap / "Chrono" / "Broken";
        write(brokenScenario / "Chrono.cfg", "[startdate]\n20200101\n[enddate]\n20200101\n");
        write(brokenScenario / "tile_0_0.map", "[version]\n14\n[object]\n");
        write(brokenScenario / "tile_0_0.map.water", "bad");
        const auto dependentScenario = chronoMap / "Chrono" / "ZDependent";
        write(dependentScenario / "Chrono.cfg", "[startdate]\n20990101\n");
        write(dependentScenario / "parklist_p_8.txt",
              "Scenery\\missing-combined-chrono-parked.sco\n");
        write(dependentScenario / "tile_0_0.map",
              "[version]\n14\n[selobject]\n101\n[typ]\n"
              "[selobject]\n999\n[typ]\nScenery\\depends-on-other.sco\n"
              "[selobject]\n998\n[relabel]\n1\n8\n"
              "[selspline]\n201\n[typ]\nSplines\\depends-on-other.sli\n");
        const auto labelScenario = chronoMap / "Chrono" / "ZZLabels";
        write(labelScenario / "Chrono.cfg", "[startdate]\n20990101\n");
        write(labelScenario / "tile_0_0.map",
              "[version]\n14\n[selobject]\n999\n[relabel]\n1\nNew label\n");
        const auto transientScenario = chronoMap / "Chrono" / "Mid";
        write(transientScenario / "Chrono.cfg", "[startdate]\n20990101\n[enddate]\n20990101\n");
        write(transientScenario / "tile_0_0.map",
              "[version]\n14\n[selobject]\n999\n[delete]\n"
              "[selobject]\n998\n[delete]\n[selspline]\n201\n[delete]\n");
        const auto disjointScenario = chronoMap / "Chrono" / "AOld";
        write(disjointScenario / "Chrono.cfg", "[startdate]\n20900101\n[enddate]\n20900101\n");
        write(disjointScenario / "tile_0_0.map",
              "[version]\n14\n[selobject]\n999\n[typ]\nScenery\\disjoint.sco\n");
        const auto invalidDateScenario = chronoMap / "Chrono" / "InvalidDate";
        write(invalidDateScenario / "Chrono.cfg", "[startdate]\ninvalid\n");
        write(invalidDateScenario / "tile_0_0.map",
              "[version]\n14\n[object]\n0\nScenery\\invalid-date.sco\n104\n"
              "1\n2\n3\n0\n0\n0\n0\n0\n");
        const auto noOverlayScenario = chronoMap / "Chrono" / "NoOverlay";
        write(noOverlayScenario / "Chrono.cfg", "[startdate]\n20990101\n");
        write(noOverlayScenario / "parklist_p.txt",
              "Scenery\\missing-no-overlay-chrono-parked.sco\n");
        const auto chronoReport = openbus::map::validateMapAssets(chronoMap, root);
        const auto chronoDirectoryTileMap = root / "Maps" / "ChronoDirectoryTile";
        const auto chronoDirectoryTileScenario = chronoDirectoryTileMap / "Chrono" / "Scenario";
        write(chronoDirectoryTileMap / "global.cfg", "[map]\n0\n0\ntile_0_0.map\n");
        write(chronoDirectoryTileMap / "tile_0_0.map", "[version]\n14\n[terrain]\n");
        write(chronoDirectoryTileScenario / "Chrono.cfg", "[startdate]\n20990101\n");
        std::filesystem::create_directories(chronoDirectoryTileScenario / "tile_0_0.map");
        std::filesystem::create_directories(chronoDirectoryTileScenario / "humans.txt");
        std::filesystem::create_directories(chronoDirectoryTileScenario / "TTData" / "invalid.ttp");
        const auto chronoDirectoryConfig =
            chronoDirectoryTileMap / "Chrono" / "DirectoryConfig" / "Chrono.cfg";
        std::filesystem::create_directories(chronoDirectoryConfig);
        const auto chronoDirectoryTileReport =
            openbus::map::validateMapAssets(chronoDirectoryTileMap, root);
        require(std::any_of(chronoDirectoryTileReport.findings.begin(),
                            chronoDirectoryTileReport.findings.end(),
                            [&](const auto& finding) {
                                return finding.category == "chrono tile" &&
                                       finding.source ==
                                           chronoDirectoryTileScenario / "tile_0_0.map" &&
                                       finding.detail.find("not a regular file") !=
                                           std::string::npos;
                            }),
                "Chrono tile directory was silently treated as an absent overlay");
        require(std::any_of(chronoDirectoryTileReport.findings.begin(),
                            chronoDirectoryTileReport.findings.end(),
                            [&](const auto& finding) {
                                return finding.category == "chrono" &&
                                       finding.source == chronoDirectoryConfig &&
                                       finding.detail.find("not a regular file") !=
                                           std::string::npos;
                            }),
                "Chrono.cfg directory was silently skipped during scenario discovery");
        require(std::any_of(chronoDirectoryTileReport.findings.begin(),
                                        chronoDirectoryTileReport.findings.end(),
                                        [&](const auto& finding) {
                                            return finding.category == "human list" &&
                                                   finding.source == chronoDirectoryTileScenario / "humans.txt" &&
                                                   finding.detail.find("not a regular file") !=
                                                       std::string::npos;
                                        }) &&
                                std::any_of(chronoDirectoryTileReport.findings.begin(),
                                            chronoDirectoryTileReport.findings.end(),
                                            [&](const auto& finding) {
                                                return finding.category == "timetable source" &&
                                                       finding.source ==
                                                           chronoDirectoryTileScenario / "TTData" /
                                                               "invalid.ttp" &&
                                                       finding.detail.find("not a regular file") !=
                                                           std::string::npos;
                                            }),
                            "Chrono human-list or timetable directories were silently skipped");
        const auto invalidChronoGroundTextureMap =
            root / "Maps" / "InvalidChronoGroundTextureDirectory";
        const auto invalidChronoGroundTextureScenario =
            invalidChronoGroundTextureMap / "Chrono" / "Scenario";
        write(invalidChronoGroundTextureMap / "global.cfg",
              "[map]\n0\n0\ntile_0_0.map\n");
        write(invalidChronoGroundTextureMap / "tile_0_0.map",
              "[version]\n14\n[terrain]\n");
        write(invalidChronoGroundTextureScenario / "Chrono.cfg", "[startdate]\n20990101\n");
        write(invalidChronoGroundTextureScenario / "texture" / "map", "not a directory");
        const auto invalidChronoGroundTextureReport =
            openbus::map::validateMapAssets(invalidChronoGroundTextureMap, root);
        require(std::any_of(invalidChronoGroundTextureReport.findings.begin(),
                            invalidChronoGroundTextureReport.findings.end(),
                            [&](const auto& finding) {
                                return finding.category == "ground texture sidecar" &&
                                       finding.source == invalidChronoGroundTextureScenario &&
                                       finding.detail.find(
                                           "Could not enumerate Chrono ground-texture sidecars") ==
                                           0;
                            }),
                "invalid Chrono ground-texture directory was silently ignored");
        const auto chronoSourceFinding = [&](const std::string& category,
                                             const std::string& reference) {
            return std::any_of(chronoReport.findings.begin(), chronoReport.findings.end(),
                               [&](const auto& finding) {
                                   return finding.category == category &&
                                          finding.source == scenario / "tile_0_0.map" &&
                                          finding.reference.find(reference) != std::string::npos;
                               });
        };
        require(chronoReport.tilesScanned == 1 && chronoReport.chronoTilesScanned == 6 &&
                    chronoReport.chronoCombinedTilesScanned == 2 &&
                    chronoReport.chronoTimetableDatesScanned >= 1,
                ("Chrono variants must not change the base tile count (tiles=" +
                    std::to_string(chronoReport.tilesScanned) +
                    ", chrono=" + std::to_string(chronoReport.chronoTilesScanned) +
                    ", combined=" + std::to_string(chronoReport.chronoCombinedTilesScanned) +
                    ", dates=" + std::to_string(chronoReport.chronoTimetableDatesScanned) + ")")
                    .c_str());
        require(std::any_of(chronoReport.sourceFiles.begin(), chronoReport.sourceFiles.end(),
                            [&](const auto& source) {
                                return source.lexically_normal() ==
                                       (chronoMap / "texture" / "water.tga").lexically_normal();
                            }) &&
                    !hasFinding(chronoReport, "water texture", "water.tga"),
                "Chrono water sidecars must trigger validation of present map water textures");
        const auto chronoManifestSources = openbus::map::makeMapBakeSourceFiles(
            chronoReport.sourceFiles, chronoMap, root);
        require(std::any_of(chronoManifestSources.begin(), chronoManifestSources.end(),
                            [](const auto& source) {
                                return source.key == "map/texture/water.tga";
                            }),
                "Chrono-triggered map water texture was omitted from bake manifest inputs");
        require(std::any_of(chronoReport.findings.begin(), chronoReport.findings.end(),
                            [&](const auto& finding) {
                                return finding.unsupported &&
                                       finding.category == "unsupported tile sidecar" &&
                                       finding.source == scenario / "tile_0_0.map.prt";
                            }),
                "uninterpreted Chrono tile .prt sidecars must be reported");
        require(std::any_of(chronoReport.findings.begin(), chronoReport.findings.end(),
                            [&](const auto& finding) {
                                return finding.unsupported &&
                                       finding.category == "unsupported tile sidecar" &&
                                       finding.source ==
                                           scenario / "tile_0_0.map.terrain_0.rdy";
                            }),
                "uninterpreted Chrono terrain-readiness sidecars must be reported");
        require(std::any_of(chronoReport.findings.begin(), chronoReport.findings.end(),
                            [&](const auto& finding) {
                                return finding.category == "lightmap" &&
                                       finding.source == scenario / "tile_0_0.map" &&
                                       finding.reference.find("tile_0_0.map.LM.bmp") !=
                                           std::string::npos;
                            }),
                "undecodable Chrono lightmap was not reported");
        require(std::any_of(chronoReport.sourceFiles.begin(), chronoReport.sourceFiles.end(),
                            [&](const auto& source) {
                                return std::filesystem::equivalent(
                                    source, scenario / "texture" / "map" /
                                                "tile_0_0.map.1.dds");
                            }) &&
                    !hasFinding(chronoReport, "ground texture sidecar", "tile_0_0.map.1.dds") &&
                    hasFinding(chronoReport, "ground texture sidecar", "tile_0_0.map.2.dds") &&
                    hasFinding(chronoReport, "ground texture sidecar", "tile_0_0.map.3.dds"),
                "Chrono ground-texture sidecars were not decoded and validated by layer suffix");
        require(std::any_of(chronoReport.findings.begin(), chronoReport.findings.end(),
                            [&](const auto& finding) {
                                return finding.category == "ground texture sidecar" &&
                                       finding.source == scenario / "tile_0_0.map" &&
                                       finding.reference.ends_with("tile_0_0.map.4.dds") &&
                                       finding.detail.find("not a regular file") !=
                                           std::string::npos;
                            }) &&
                    std::none_of(chronoReport.sourceFiles.begin(), chronoReport.sourceFiles.end(),
                                 [&](const auto& source) {
                                     return source.lexically_normal() ==
                                            (scenario / "texture" / "map" /
                                             "tile_0_0.map.4.dds")
                                                .lexically_normal();
                                 }),
                "non-file Chrono ground-texture sidecar was ignored or inventoried as an asset");
        require(hasFinding(chronoReport, "timetable track", "9999") &&
                    std::none_of(chronoReport.findings.begin(), chronoReport.findings.end(),
                                 [](const auto& finding) {
                                     return finding.category == "timetable track" &&
                                            (finding.reference == "101" ||
                                             finding.reference == "200" ||
                                             finding.reference == "103" ||
                                             finding.reference == "201");
                                 }),
                "timetable track IDs in base and Chrono placements were not resolved correctly");
        require(std::any_of(chronoReport.findings.begin(), chronoReport.findings.end(),
                            [&](const auto& finding) {
                                return finding.category == "timetable track" &&
                                       finding.source ==
                                           scenario / "TTData" / "ChronoTrackTrip.ttr" &&
                                       finding.reference == "9998" &&
                                       finding.detail.find("line 16") != std::string::npos;
                            }) &&
                    std::none_of(chronoReport.findings.begin(), chronoReport.findings.end(),
                                 [&](const auto& finding) {
                                     return finding.category == "timetable track" &&
                                            finding.source ==
                                                scenario / "TTData" / "ChronoTrackTrip.ttr" &&
                                            (finding.reference == "103" ||
                                             finding.reference == "201");
                                 }),
                "Chrono timetable track IDs were not checked against overlay placements");
        require(std::any_of(chronoReport.findings.begin(), chronoReport.findings.end(),
                            [&](const auto& finding) {
                                return finding.category == "chrono timetable" &&
                                       finding.reference == "addtrip" &&
                                       finding.detail.find("ChronoMissingTrip") !=
                                           std::string::npos;
                            }),
                "date-selected Chrono timetable dependencies were not checked");
        require(
            hasFinding(chronoReport, "Chrono AI vehicle", "missing-chrono-group.bus") &&
                hasFinding(chronoReport, "Chrono AI vehicle", "missing-chrono-group.ovh") &&
            hasFinding(chronoReport, "Chrono AI vehicle", "missing-chrono-ailist.bus") &&
            hasFinding(chronoReport, "Chrono AI vehicle", "missing-chrono-depot.bus") &&
                hasFinding(chronoReport, "Chrono AI train", "missing-chrono-train.zug") &&
            hasFinding(chronoReport, "Chrono AI train", "missing-chrono-ailist.zug") &&
            hasFinding(chronoReport, "Chrono AI train vehicle", "missing-chrono-train-car.ovh") &&
            hasFinding(chronoReport, "human", "missing-chrono-scenario-human.hum") &&
            hasFinding(chronoReport, "driver", "missing-chrono-scenario-driver.hum") &&
            hasFinding(chronoReport, "human", "missing-chrono-human.hum") &&
            hasFinding(chronoReport, "driver", "missing-chrono-driver.hum"),
            "Chrono AI vehicle update references were not checked");
        require(std::any_of(chronoReport.findings.begin(), chronoReport.findings.end(),
                            [&](const auto& finding) {
                                return finding.unsupported &&
                                       finding.category == "unsupported list entry" &&
                                       finding.source == scenario / "humans.txt" &&
                                       finding.reference == "not-a-scenario-human-reference";
                            }),
                "non-path entries in scenario-local human lists were silently ignored");
        require(std::any_of(chronoReport.findings.begin(), chronoReport.findings.end(),
                        [&](const auto& finding) {
                            return finding.category == "human" &&
                                   finding.source == scenario / "humans_#upd.cfg" &&
                                   finding.reference == "Humans\\missing-chrono-human.hum";
                        }) &&
                std::any_of(chronoReport.findings.begin(), chronoReport.findings.end(),
                            [&](const auto& finding) {
                                return finding.category == "driver" &&
                                       finding.source == scenario / "drivers_#upd.cfg" &&
                                       finding.reference ==
                                           "Humans\\missing-chrono-driver.hum";
                            }),
            "Chrono human/driver update paths lost their source attribution");
        require(hasFinding(chronoReport, "AI group HOF", "AbsentChronoGroupHof") &&
                    hasFinding(chronoReport, "AI depot HOF", "AbsentChronoDepotHof") &&
                    std::any_of(chronoReport.findings.begin(), chronoReport.findings.end(),
                                [&](const auto& finding) {
                                    return finding.category == "AI group HOF" &&
                                           finding.source == scenario / "ailists_#upd.cfg" &&
                                           finding.reference == "AbsentChronoGroupHof";
                                }) &&
                    std::any_of(chronoReport.findings.begin(), chronoReport.findings.end(),
                                [&](const auto& finding) {
                                    return finding.category == "AI group HOF" &&
                                           finding.source == chronoMap / "ailists.cfg" &&
                                           finding.reference == "AbsentChronoGroupHof";
                                }) &&
                    std::any_of(chronoReport.findings.begin(), chronoReport.findings.end(),
                                [&](const auto& finding) {
                                    return finding.category == "AI depot HOF" &&
                                           finding.source == scenario / "ailists_#upd.cfg" &&
                                           finding.reference == "AbsentChronoDepotHof";
                                }) &&
                    std::none_of(chronoReport.findings.begin(), chronoReport.findings.end(),
                                 [&](const auto& finding) {
                                     return (finding.category == "AI group HOF" ||
                                             finding.category == "AI depot HOF") &&
                                            finding.source == scenario / "ailists_#upd.cfg" &&
                                            finding.reference == "ExpectedHof";
                                 }),
                "Chrono AI group HOF associations were not checked against the update source");
        require(hasFinding(chronoReport, "Chrono ticket pack", "missing-chrono.otp") &&
                    std::any_of(chronoReport.findings.begin(), chronoReport.findings.end(),
                                [&](const auto& finding) {
                                    return finding.category == "currency" &&
                                           finding.source == scenario / "Chrono.cfg" &&
                                           finding.reference == "Money\\missing-chrono.cfg";
                                }),
                "Chrono ticket-pack and currency overrides were not checked");
        require(std::any_of(chronoReport.findings.begin(), chronoReport.findings.end(),
                            [&](const auto& finding) {
                                return finding.unsupported &&
                                       finding.category == "unsupported section" &&
                                       finding.source == scenario / "Chrono.cfg" &&
                                       finding.reference == "unsupported_chrono_setting";
                            }),
                "unsupported Chrono.cfg section was silently ignored");
        require(std::any_of(chronoReport.findings.begin(), chronoReport.findings.end(),
                            [&](const auto& finding) {
                                return finding.unsupported &&
                                       finding.category == "unsupported section" &&
                                       finding.source == scenario / "ailists_#upd.cfg" &&
                                       finding.reference == "unsupported_chrono_ai";
                            }),
                "unsupported Chrono AI-list sections were not reported");
        require(std::any_of(chronoReport.findings.begin(), chronoReport.findings.end(),
                            [&](const auto& finding) {
                                return finding.unsupported &&
                                       finding.category == "Chrono traffic update" &&
                                       finding.source == scenario / "chrono_#upd.cfg" &&
                                       finding.detail.find("references in the file are not checked") !=
                                           std::string::npos;
                            }),
                "unsupported Chrono traffic update was not identified");
        require(std::any_of(chronoReport.findings.begin(), chronoReport.findings.end(),
                            [&](const auto& finding) {
                                return finding.unsupported &&
                                       finding.category == "Chrono update" &&
                                       finding.source == scenario / "humans_#upd.cfg" &&
                                       finding.detail.find("full update semantics are not validated") !=
                                           std::string::npos;
                            }),
                "partially interpreted Chrono human update was not marked unsupported");
        require(hasFinding(chronoReport, "human", "missing-chrono-human.hum"),
                "Chrono human update references were not checked");
        require(chronoSourceFinding("object", "chrono-replacement.sco") &&
                    chronoSourceFinding("object", "chrono-added.sco") &&
                    chronoSourceFinding("spline", "chrono-replacement.sli") &&
                    chronoSourceFinding("spline", "chrono-added.sli"),
                "Chrono additive and selector dependencies were not checked");
        require(hasFinding(chronoReport, "parked object", "missing-chrono-parked.sco") &&
                    !std::any_of(chronoReport.findings.begin(), chronoReport.findings.end(),
                                 [&](const auto& finding) {
                                     return finding.category == "park list" &&
                                            finding.source == scenario / "tile_0_0.map" &&
                                            finding.reference == "parklist_p_7.txt" &&
                                            finding.detail.find("not found") != std::string::npos;
                                 }),
                "Chrono-specific car-park list was not preferred over the base list");
        require(hasFinding(chronoReport, "parked object", "missing-no-overlay-chrono-parked.sco"),
                "Chrono car-park lists without a matching tile overlay were not checked");
        require(chronoSourceFinding("chrono tile", "selobject") &&
                    chronoSourceFinding("terrain", "tile_0_0.map.terrain") &&
                    chronoSourceFinding("water", "tile_0_0.map.water"),
                "Chrono selector and sidecar errors were not reported");
        require(std::any_of(chronoReport.findings.begin(), chronoReport.findings.end(),
                            [&](const auto& finding) {
                                return finding.unsupported &&
                                       finding.category == "unsupported section" &&
                                       finding.source == scenario / "tile_0_0.map" &&
                                       finding.reference == "unsupported_chrono_tile";
                            }),
                "unsupported section in a Chrono tile was not reported");
        const auto containsChronoSource = [&](const std::filesystem::path& path) {
            return std::any_of(chronoReport.sourceFiles.begin(), chronoReport.sourceFiles.end(),
                              [&](const auto& source) {
                                  return std::filesystem::equivalent(source, path);
                              });
        };
        require(containsChronoSource(scenario / "tile_0_0.map") &&
                   containsChronoSource(scenario / "tile_0_0.map.terrain") &&
                   containsChronoSource(scenario / "tile_0_0.map.water"),
                "source inventory omitted malformed Chrono source files");
        require(std::any_of(chronoReport.findings.begin(), chronoReport.findings.end(),
                            [&](const auto& finding) {
                                return finding.category == "chrono tile" &&
                                       finding.source == brokenScenario / "tile_0_0.map";
                            }),
                "malformed Chrono tile was not reported");
        require(std::any_of(chronoReport.findings.begin(), chronoReport.findings.end(),
                            [&](const auto& finding) {
                                return finding.source == brokenScenario / "tile_0_0.map" &&
                                       finding.category == "water";
                            }),
                "Chrono sidecars must be checked even when the overlay is malformed");
        const auto independentWarning = [&](const std::string& detail) {
            return std::any_of(chronoReport.findings.begin(), chronoReport.findings.end(),
                               [&](const auto& finding) {
                                   return finding.source == dependentScenario / "tile_0_0.map" &&
                                          finding.category == "chrono tile" &&
                                          finding.detail.find("Independent scenario check") !=
                                              std::string::npos &&
                                          finding.detail.find(detail) != std::string::npos;
                               });
        };
        require(independentWarning("missing its value") && independentWarning("did not match"),
                "malformed and cross-scenario selectors must be reported as independent checks");
        require(std::count_if(chronoReport.findings.begin(), chronoReport.findings.end(),
                              [&](const auto& finding) {
                                  return finding.source == dependentScenario / "tile_0_0.map" &&
                                         finding.category == "chrono tile" &&
                                         finding.detail.find("Active Chrono combination") !=
                                             std::string::npos &&
                                         finding.detail.find("missing its value") !=
                                             std::string::npos;
                              }) == 1,
                "repeated combination diagnostics should retain one example date");
        require(std::any_of(chronoReport.findings.begin(), chronoReport.findings.end(),
                            [&](const auto& finding) {
                                return finding.source == invalidDateScenario / "tile_0_0.map" &&
                                       finding.category == "object" &&
                                       finding.reference.find("invalid-date.sco") !=
                                           std::string::npos;
                            }),
                "independent validation should include invalid-date scenarios");
        const auto combinedSourceFinding = [&](const std::string& category,
                                               const std::string& reference) {
            return std::any_of(chronoReport.findings.begin(), chronoReport.findings.end(),
                               [&](const auto& finding) {
                                   return finding.source == dependentScenario / "tile_0_0.map" &&
                                          finding.category == category &&
                                          finding.reference.find(reference) != std::string::npos;
                               });
        };
        require(combinedSourceFinding("object", "depends-on-other.sco") &&
                    combinedSourceFinding("spline", "depends-on-other.sli") &&
                    hasFinding(chronoReport, "parked object", "missing-combined-chrono-parked.sco"),
                "overlap after a scenario ends did not validate dependent selector assets");
        require(std::any_of(chronoReport.findings.begin(), chronoReport.findings.end(),
                            [&](const auto& finding) {
                                return finding.source == invalidDateScenario / "Chrono.cfg" &&
                                       finding.category == "chrono" &&
                                       finding.detail.find("scenario is inactive") !=
                                           std::string::npos;
                            }),
                "invalid Chrono dates must produce configuration diagnostics");
        require(!hasFinding(chronoReport, "object", "unlisted.sco") &&
                    !hasFinding(chronoReport, "object", "no-config.sco") &&
                    !hasFinding(chronoReport, "object", "unreachable.sco") &&
                    !hasFinding(chronoReport, "object", "disjoint.sco"),
                "unmatched, unconfigured or nonoverlapping Chrono records were treated as active");
        require(std::any_of(chronoReport.sourceFiles.begin(), chronoReport.sourceFiles.end(),
                            [&](const auto& source) {
                                return source.lexically_normal() ==
                                       (scenario / "unlisted.map").lexically_normal();
                            }) &&
                    std::any_of(chronoReport.findings.begin(), chronoReport.findings.end(),
                                [&](const auto& finding) {
                                    return finding.unsupported &&
                                           finding.category == "unclassified Chrono file" &&
                                           finding.source == scenario / "unlisted.map";
                                }),
                "unlisted Chrono scenario file was not inventoried as unsupported");
        const auto badConfigMap = root / "Maps" / "BadChronoConfig";
        write(badConfigMap / "global.cfg", "[map]\n0\n0\ntile_0_0.map\n");
        write(badConfigMap / "tile_0_0.map", "[version]\n14\n[terrain]\n");
        write(badConfigMap / "tile_0_0.map.terrain", validTerrain);
        const auto badConfig = badConfigMap / "Chrono" / "Broken" / "Chrono.cfg";
        write(badConfig, std::string("\xff\xfe\x41", 3));
        write(badConfig.parent_path() / "tile_0_0.map",
              "[version]\n14\n[object]\n0\nScenery\\bad-config-added.sco\n101\n"
              "1\n2\n3\n0\n0\n0\n0\n0\n");
        const auto badConfigReport = openbus::map::validateMapAssets(badConfigMap, root);
        require(badConfigReport.chronoTilesScanned == 1 &&
                    badConfigReport.chronoCombinedTilesScanned == 0 &&
                    hasFinding(badConfigReport, "object", "bad-config-added.sco") &&
                    std::any_of(badConfigReport.findings.begin(), badConfigReport.findings.end(),
                                [&](const auto& finding) {
                                    return finding.source == badConfig &&
                                           finding.category == "chrono" &&
                                           finding.detail.find("UTF-16") != std::string::npos;
                                }) &&
                    std::any_of(badConfigReport.findings.begin(), badConfigReport.findings.end(),
                                [&](const auto& finding) {
                                    return finding.source == badConfigMap / "Chrono" &&
                                           finding.category == "chrono" &&
                                           finding.detail.find("combinations were not checked") !=
                                               std::string::npos;
                                }),
                "unreadable Chrono dates must not stop independent validation or imply coverage");
        const auto invalidChronoRoot = root / "Maps" / "InvalidChronoRoot";
        write(invalidChronoRoot / "global.cfg", "[map]\n0\n0\ntile_0_0.map\n");
        write(invalidChronoRoot / "tile_0_0.map", "[version]\n14\n[terrain]\n");
        write(invalidChronoRoot / "tile_0_0.map.terrain", validTerrain);
        write(invalidChronoRoot / "Chrono", "not a directory");
        const auto invalidChronoReport = openbus::map::validateMapAssets(invalidChronoRoot, root);
        require(std::any_of(invalidChronoReport.findings.begin(),
                            invalidChronoReport.findings.end(),
                            [&](const auto& finding) {
                                return finding.category == "chrono" &&
                                       finding.source == invalidChronoRoot / "Chrono" &&
                                       finding.detail.find("not a directory") != std::string::npos;
                            }) &&
                    std::any_of(invalidChronoReport.findings.begin(),
                                invalidChronoReport.findings.end(),
                                [&](const auto& finding) {
                                    return finding.category == "chrono" &&
                                           finding.source == invalidChronoRoot / "Chrono" &&
                                           finding.detail.find("combinations were not checked") !=
                                               std::string::npos;
                                }),
                "invalid Chrono directory was not reported");

        const auto absentMap = root / "Maps" / "Absent";
        write(absentMap / "global.cfg", "[worldcoordinates]\n[map]\n0\n0\ntile_0_0.map\n"
                                        "[map]\n1\n0\ntile_1_0.map\n");
        write(absentMap / "humans.txt", "Humans\\absent.hum\n");
        std::filesystem::create_directories(absentMap / "tile_1_0.map");
        const auto absentReport = openbus::map::validateMapAssets(absentMap, root);
        require(absentReport.tilesScanned == 0 &&
                    hasFinding(absentReport, "tile", "tile_0_0.map") &&
                    hasFinding(absentReport, "tile", "tile_1_0.map") &&
                    std::any_of(absentReport.findings.begin(), absentReport.findings.end(),
                                [](const auto& finding) {
                                    return finding.category == "tile" &&
                                           finding.reference.find("tile_1_0.map") !=
                                               std::string::npos &&
                                           finding.detail.find("not a regular file") !=
                                               std::string::npos;
                                }) &&
                    hasFinding(absentReport, "human", "absent.hum"),
                "missing or non-regular tiles should not prevent dependency reporting");
        bool defaultLoaderRejectedMissingTiles = false;
        try {
            (void)openbus::map::loadMapDefinition(absentMap);
        } catch (const std::runtime_error&) {
            defaultLoaderRejectedMissingTiles = true;
        }
        require(defaultLoaderRejectedMissingTiles,
                "normal map loader must still reject missing tiles");

        const auto missingGlobalMap = root / "Maps" / "MissingGlobal";
        std::filesystem::create_directories(missingGlobalMap);
        const auto missingGlobalReport =
            openbus::map::validateMapAssets(missingGlobalMap, root);
        require(missingGlobalReport.findings.size() == 1 &&
                    missingGlobalReport.findings.front().category == "map configuration" &&
                    missingGlobalReport.findings.front().source ==
                        missingGlobalMap / "global.cfg" &&
                    missingGlobalReport.findings.front().detail.find("not found") !=
                        std::string::npos &&
                    !missingGlobalReport.complete,
                "missing global.cfg did not produce an explicit incomplete validation report");

        const auto directoryGlobalMap = root / "Maps" / "DirectoryGlobal";
        std::filesystem::create_directories(directoryGlobalMap / "global.cfg");
        const auto directoryGlobalReport =
            openbus::map::validateMapAssets(directoryGlobalMap, root);
        require(directoryGlobalReport.findings.size() == 1 &&
                    directoryGlobalReport.findings.front().detail.find("not a regular file") !=
                        std::string::npos &&
                    !directoryGlobalReport.complete,
                "non-file global.cfg did not produce an explicit incomplete report");

        const auto malformedGlobalMap = root / "Maps" / "MalformedGlobal";
        write(malformedGlobalMap / "global.cfg", "[map]\ninvalid-x\n0\ntile_0_0.map\n");
        const auto malformedGlobalReport =
            openbus::map::validateMapAssets(malformedGlobalMap, root);
        require(malformedGlobalReport.findings.size() == 1 &&
                    malformedGlobalReport.findings.front().category == "map configuration" &&
                    malformedGlobalReport.findings.front().detail.find("Could not parse") == 0 &&
                    malformedGlobalReport.sourceFiles.size() == 1 &&
                    malformedGlobalReport.sourceFiles.front() ==
                        std::filesystem::absolute(malformedGlobalMap / "global.cfg")
                            .lexically_normal() &&
                    !malformedGlobalReport.complete,
                "malformed global.cfg did not produce an incomplete report with its source inventoried");

        const auto nonDirectoryMap = root / "Maps" / "NotAMapDirectory";
        write(nonDirectoryMap, "not a map directory");
        const auto nonDirectoryMapReport = openbus::map::validateMapAssets(nonDirectoryMap, root);
        require(nonDirectoryMapReport.findings.size() == 1 &&
                    nonDirectoryMapReport.findings.front().category == "map input" &&
                    nonDirectoryMapReport.findings.front().detail == "Map path is not a directory" &&
                    !nonDirectoryMapReport.complete,
                "non-directory map input did not produce an explicit incomplete report");

        const auto invalidOmsiRoot = root / "NotAnOmsiRoot";
        write(invalidOmsiRoot, "not a directory");
        const auto invalidOmsiRootReport =
            openbus::map::validateMapAssets(map, invalidOmsiRoot);
        require(invalidOmsiRootReport.findings.size() == 1 &&
                    invalidOmsiRootReport.findings.front().category == "OMSI root" &&
                    invalidOmsiRootReport.findings.front().detail == "OMSI root is not a directory" &&
                    !invalidOmsiRootReport.complete,
                "invalid OMSI root did not produce an explicit incomplete report");

        openbus::probe::removeProbeDirectory(root);
    } catch (...) {
        openbus::probe::removeProbeDirectory(root);
        throw;
    }
}

} // namespace

int main() {
    try {
        run();
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
