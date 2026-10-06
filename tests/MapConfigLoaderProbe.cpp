#include "MapConfigLoader.h"

#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

void require(bool condition, const char* description) {
    if (!condition) {
        throw std::runtime_error(description);
    }
}

void writeUtf16Le(const std::filesystem::path& path, const std::string& text) {
    std::ofstream output(path, std::ios::binary);
    if (!output) {
        throw std::runtime_error("could not create UTF-16 fixture");
    }
    output.put(static_cast<char>(0xff));
    output.put(static_cast<char>(0xfe));
    for (const unsigned char character : text) {
        output.put(static_cast<char>(character));
        output.put('\0');
    }
}

void writeU32(std::ofstream& output, std::uint32_t value) {
    for (int byte = 0; byte < 4; ++byte) {
        output.put(static_cast<char>((value >> (byte * 8)) & 0xffU));
    }
}

void runFixtureProbe() {
    const std::filesystem::path root =
        std::filesystem::temp_directory_path() / "openbus-map-config-probe";
    std::filesystem::remove_all(root);
    std::filesystem::create_directories(root);
    try {
        const std::filesystem::path groundTextureDirectory = root / "texture" / "map";
        std::filesystem::create_directories(groundTextureDirectory);
        for (const std::string& sidecarName : {"tile_0_0.map.1.dds", "tile_0_0.map.09.dds",
                                               "tile_0_0.map.preview.dds"}) {
            std::ofstream(groundTextureDirectory / sidecarName, std::ios::binary);
        }
        std::ofstream(root / "tile_0_0.map.LM.bmp", std::ios::binary);
        std::ofstream(root / "tile_0_0.map.terrain_0.rdy", std::ios::binary);
        writeUtf16Le(root / "global.cfg",
                     "[name]\nFixture map\n[friendlyname]\nFriendly fixture\n"
                     "[description]\nFirst line\n\nSecond line\n[end]\n"
                     "[version]\n14\n[NextIDCode]\n81\n[worldcoordinates]\n"
                     "[dynhelperactive]\n[realrail]\n[backgroundimage]\n1\n2\n3\n4\n5\n6\n"
                     "[mapcam]\n0\n1\n2\n3\n4\n5\n6\n7\n[moneysystem]\nMoney.cfg\n"
                     "[ticketpack]\nTickets.otp\n[repair_time_min]\n10\n[years]\n2022\n2023\n"
                     "[realyearoffset]\n0\n[standarddepot]\nDepot\n"
                     "Winter:\n[addseason]\n4\n0\n38\n[trafficdensity_road]\n12\n1\n"
                     "[trafficdensity_passenger]\n0\n0.2\n[groundtex]\nTexture\\grass.bmp\n"
                     "Texture\\detail.bmp\n0\n1\n60\n[groundtex]\n"
                     "Texture\\asphalt.bmp\nTexture\\detail2.bmp\nraw-a\nraw-b\nraw-c\n"
                     "[unknown_global]\n opaque global payload \n\n"
                     "[entrypoints]\n2\n"
                     "2\n11\n0\n10\n20\n0\n0\n0\n0\n1\n0\nDepot\n"
                     "3\n12\n0\n30\n5\n40\n2\n5\n3\n7\n1\nDepot East\n"
                     "[map]\n0\n0\ntile_0_0.map\n[map]\n1\n0\ntile_1_0.map\n");
        writeUtf16Le(root / "tile_0_0.map",
                 "Fixture tile\n[version]\n14\n[terrain]\n[water]\n"
                 "[unknown_future_section]\n opaque tile payload \n\n"
                     "[variable_terrain]\n[variable_terrainlightmap]\n\n"
                     "[spline]\n0\nSplines\\road.sli\n30273\n30272\n0\n10\n20\n30\n"
                     "0\n50\n0\n0\n0\n0\n0\n0\n0\n0\n\n"
                     "[spline_h]\n0\nSplines\\bridge.sli\n30274\n0\n0\n11\n21\n31\n"
                     "0\n70\n0\n12\n12\n3.5\n0\n0\n0\n0\n0\n\n"
                     "[spline_terrain_align_2]\nraw spline alignment\n[rule]\n"
                     "spline rule 1\nspline rule 2\nspline rule 3\nspline rule 4\n"
                     "Object Nr. 4\n[object]\n0\n"
                     "Scenery\\tree.sco\n5\n1\n2\n0\n0\n0\n0\n3\n"
                     "sign text\n[object]\n\n[varparent]\n3\n"
                     "[spline_terrain_align]\n[rule]\nobject rule 1\nobject rule 2\n"
                     "object rule 3\nobject rule 4\n[kill_rule]\nkill rule 1\n"
                     "kill rule 2\nkill rule 3\nkill rule 4\n"
                     "Object Nr. 5\n[splineAttachement]\n0\nScenery\\lamp.sco\n7\n1\n"
                     "3\n-0.2\n4\n90\n0\n0\n10\n2\n0\n0\n[varparent]\n23\n"
                     "[spline_terrain_align]\n[kill_rule]\nattachment rule 1\n"
                     "attachment rule 2\nattachment rule 3\nattachment rule 4\n\n"
                     "Object Nr. 6\n[splineAttachement_repeater]\n0\n0\n1\n"
                     "Scenery\\lamp.sco\n8\n0\n1\n0.2\n4\n0\n0\n0\n10\n20\n0\n0\n\n"
                     "Object Nr. 7\n[attachObj]\n0\nScenery\\attached.sco\n17\n5\n0\n"
                     "2\n30\n4\n0\n1\ncustom tail\n\n[varparent]\n41\n"
                     "[spline_terrain_align]\n[rule]\nattached rule 1\nattached rule 2\n"
                     "attached rule 3\nattached rule 4\n");
                writeUtf16Le(root / "tile_1_0.map", "Second fixture tile\n[version]\n14\n[terrain]\n");
        {
            std::ofstream terrain(root / "tile_0_0.map.terrain", std::ios::binary);
            writeU32(terrain, 2);
            for (std::uint32_t value = 1; value <= 9; ++value) {
                writeU32(terrain, std::bit_cast<std::uint32_t>(static_cast<float>(value)));
            }
        }
        {
            std::ofstream water(root / "tile_0_0.map.water", std::ios::binary);
            writeU32(water, 2);
            for (const float height : {1.25F, 2.5F, 3.75F, 5.0F, -1.0F, 0.0F, 12.5F, 40.0F}) {
                writeU32(water, std::bit_cast<std::uint32_t>(height));
            }
        }

        const openbus::map::MapDefinition definition = openbus::map::loadMapDefinition(root);
        require(definition.metadata.name == "Fixture map" &&
                    definition.metadata.friendlyName == "Friendly fixture" &&
                    definition.metadata.description == "First line\n\nSecond line" &&
                    definition.metadata.version == "14" &&
                    definition.metadata.nextIdCode == "81" &&
                    definition.metadata.hasWorldCoordinates &&
                    definition.metadata.hasDynamicHelperActive && definition.metadata.hasRealRail,
                "global map identity, description, version, and marker metadata captured");
        require(definition.metadata.backgroundImage ==
                    std::array<std::string, 6>{"1", "2", "3", "4", "5", "6"} &&
                    definition.metadata.hasBackgroundImage &&
                    definition.metadata.mapCamera ==
                        std::array<std::string, 8>{"0", "1", "2", "3", "4", "5", "6", "7"} &&
                    definition.metadata.hasMapCamera &&
                    definition.metadata.moneySystem == "Money.cfg" &&
                    definition.metadata.ticketPack == "Tickets.otp" &&
                    definition.metadata.repairTimeMin == "10" &&
                    definition.metadata.years == std::array<std::string, 2>{"2022", "2023"} &&
                    definition.metadata.realYearOffset == "0" &&
                    definition.metadata.standardDepot == "Depot",
                "fixed-width and single-value global metadata captured as source text");
        require(definition.metadata.seasons.size() == 1 &&
                    definition.metadata.seasons[0].label == "Winter:" &&
                    definition.metadata.seasons[0].fields ==
                        std::array<std::string, 3>{"4", "0", "38"} &&
                    definition.metadata.roadTrafficDensity.size() == 1 &&
                    definition.metadata.roadTrafficDensity[0] ==
                        std::array<std::string, 2>{"12", "1"} &&
                    definition.metadata.passengerTrafficDensity.size() == 1 &&
                    definition.metadata.passengerTrafficDensity[0] ==
                        std::array<std::string, 2>{"0", "0.2"},
                "repeated global season and traffic-density records captured");
        require(definition.diagnostics.entries.size() == 1 &&
                    definition.diagnostics.entries[0].keyword == "unknown_global" &&
                    definition.diagnostics.entries[0].line > 0,
                "unsupported global.cfg sections produce line-numbered warnings");
        require(definition.unsupportedSections.size() == 1 &&
                    definition.unsupportedSections[0].keyword == "unknown_global" &&
                    definition.unsupportedSections[0].sectionLine ==
                        definition.diagnostics.entries[0].line &&
                    definition.unsupportedSections[0].payloadLines ==
                        std::vector<std::string>{" opaque global payload ", ""},
                "unsupported global section payload is preserved verbatim");
        require(definition.groundTextures.size() == 2, "multiple groundtex records parsed");
        require(definition.groundTextures[0].texturePath == "Texture\\grass.bmp",
                "ground texture path preserved");
        require(definition.groundTextures[1].texturePath == "Texture\\asphalt.bmp" &&
                    definition.groundTextures[1].detailTexturePath == "Texture\\detail2.bmp" &&
                    definition.groundTextures[1].parameters ==
                        std::array<std::string, 3>{"raw-a", "raw-b", "raw-c"},
                "additional ground texture paths and opaque parameters preserved");
        require(openbus::map::selectMapGroundTextureIndex(definition, "") == 0 &&
                    openbus::map::selectMapGroundTextureIndex(definition, "1") == 1,
                "ground texture preview selector defaults to first record and accepts index");
        for (const std::string invalidSelector : {"-1", "2", "not-an-index"}) {
            bool invalidGroundTextureRejected = false;
            try {
                static_cast<void>(openbus::map::selectMapGroundTextureIndex(
                    definition, invalidSelector));
            } catch (const std::runtime_error&) {
                invalidGroundTextureRejected = true;
            }
            require(invalidGroundTextureRejected,
                    "invalid ground texture preview selector is rejected");
        }
        require(definition.entryPoints.size() == 2, "entrypoint list parsed");
        require(definition.entryPoints[0].tileIndex == 0 &&
                    definition.entryPoints[0].name == "Depot",
                "entrypoint fields mapped");
        require(definition.entryPoints[0].placement.position[0] == 10.0 &&
                    definition.entryPoints[0].placement.position[1] == 0.0 &&
                    definition.entryPoints[0].placement.position[2] == 20.0,
                "entrypoint placement maps tile-local coordinates into map coordinates");
        require(definition.entryPoints[1].placement.position[0] == 330.0 &&
                    definition.entryPoints[1].placement.position[1] == 40.0 &&
                    definition.entryPoints[1].placement.position[2] == 5.0 &&
                    std::abs(definition.entryPoints[1].placement.yawDegrees -
                             std::atan2(82.0, 19.0) * 180.0 / 3.14159265358979323846) < 1.0e-9,
                "indexed entrypoint placement includes tile offset, elevation, and quaternion yaw");
        require(openbus::map::selectMapSpawnPoint(definition, "") == 0 &&
                    openbus::map::selectMapSpawnPoint(definition, "1") == 1,
                "map spawn selection defaults to index zero and accepts zero-based indices");
        for (const std::string invalidSpawn : {"-1", "2", "depot east", "not-an-index"}) {
            bool invalidSpawnRejected = false;
            try {
                static_cast<void>(openbus::map::selectMapSpawnPoint(definition, invalidSpawn));
            } catch (const std::runtime_error&) {
                invalidSpawnRejected = true;
            }
            require(invalidSpawnRejected, "invalid map spawn index is rejected");
        }
        require(openbus::map::selectMapEntryPoint(definition, "") == 0 &&
                    openbus::map::selectMapEntryPoint(definition, "1") == 1 &&
                    openbus::map::selectMapEntryPoint(definition, "depot east") == 1,
                "entrypoint selection supports the default, zero-based index, and name");
        bool invalidEntryPointRejected = false;
        try {
            static_cast<void>(openbus::map::selectMapEntryPoint(definition, "99"));
        } catch (const std::runtime_error&) {
            invalidEntryPointRejected = true;
        }
        require(invalidEntryPointRejected, "invalid entrypoint index is rejected");
        require(definition.tiles.size() == 2 && definition.tiles[0].hasTerrainFile &&
                    definition.tiles[0].hasWaterFile &&
                    definition.tiles[0].hasLightmapFile &&
                    definition.tiles[0].lightmapPath.filename() == "tile_0_0.map.LM.bmp" &&
                    definition.tiles[0].hasTerrainReadinessFile &&
                    definition.tiles[0].terrainReadinessPath.filename() ==
                        "tile_0_0.map.terrain_0.rdy" &&
                    definition.tiles[0].groundTextureSidecars.size() == 2 &&
                    definition.tiles[0].groundTextureSidecars[0].ordinalSuffix == "09" &&
                    definition.tiles[0].groundTextureSidecars[1].ordinalSuffix == "1" &&
                    definition.tiles[0].groundTextureSidecars[0].path.filename() ==
                        "tile_0_0.map.09.dds" &&
                    !definition.tiles[1].hasLightmapFile &&
                    !definition.tiles[1].hasTerrainReadinessFile &&
                    definition.tiles[1].groundTextureSidecars.empty(),
                "tile manifest inventories known sidecars and discovers numeric textures only");

        const openbus::map::MapTileData tile = openbus::map::loadMapTile(definition.tiles[0]);
        require(tile.version == "14" && tile.hasTerrainMarker && tile.hasWaterMarker &&
                    tile.hasVariableTerrainMarker && tile.hasVariableTerrainLightmapMarker,
                "tile header and terrain/water marker families parsed");
        require(tile.diagnostics.entries.size() == 1 &&
                    tile.diagnostics.entries[0].keyword == "unknown_future_section" &&
                    tile.diagnostics.entries[0].line > 0 &&
                    tile.diagnostics.entries[0].message == "map tile section is not implemented",
                "unimplemented tile sections produce line-numbered diagnostics");
        require(tile.unsupportedSections.size() == 1 &&
                tile.unsupportedSections[0].keyword == "unknown_future_section" &&
                tile.unsupportedSections[0].sectionLine == tile.diagnostics.entries[0].line &&
                tile.unsupportedSections[0].payloadLines ==
                        std::vector<std::string>{" opaque tile payload ", ""},
            "unsupported tile section payload is preserved verbatim");
        require(tile.splineCount == 1 && tile.elevatedSplineCount == 1 && tile.objectCount == 1 &&
                    tile.splineAttachmentCount == 1 && tile.splineRepeaterCount == 1 &&
                    tile.attachedObjects.size() == 1,
                "tile record families counted");
        require(tile.attachedObjects[0].label == "Object Nr. 7" &&
                tile.attachedObjects[0].editorObjectNumber == 7 &&
                    tile.attachedObjects[0].line1 == "0" &&
                    tile.attachedObjects[0].assetPath == "Scenery\\attached.sco" &&
                    tile.attachedObjects[0].id == 17 &&
                    tile.attachedObjects[0].attachedToObjectId == 5 &&
                    tile.attachedObjects[0].line5 == "0" &&
                    tile.attachedObjects[0].attachPointIndex == "2" &&
                    tile.attachedObjects[0].attachPointIndexValue == 2 &&
                    tile.attachedObjects[0].rotationFields ==
                        std::array<std::string, 3>{"30", "4", "0"} &&
                    tile.attachedObjects[0].rotationDegrees ==
                        std::array<double, 3>{30.0, 4.0, 0.0} &&
                    tile.attachedObjects[0].labelCount == "1" &&
                    tile.attachedObjects[0].hasExplicitId &&
                    tile.attachedObjects[0].transformValid &&
                    tile.attachedObjects[0].variableParentId == 41 &&
                    tile.attachedObjects[0].splineTerrainAlign &&
                    tile.attachedObjects[0].rules.size() == 1 &&
                    tile.attachedObjects[0].optionalLines ==
                        std::vector<std::string>{"custom tail", ""},
                "modern attachObj IDs and opaque fields parsed and preserved");
        require(tile.splines.size() == 2 && tile.splines[0].geometryValid &&
                    tile.splines[0].assetPath == "Splines\\road.sli" &&
                    tile.splines[0].localX == 10.0 && tile.splines[0].elevation == 20.0 &&
                    tile.splines[0].localY == 30.0 && tile.splines[0].length == 50.0,
                "fixed-width spline geometry parsed while raw fields remain available");
        require(tile.splines[1].geometryValid && tile.splines[1].elevated &&
                    tile.splines[1].assetPath == "Splines\\bridge.sli" &&
                    tile.splines[1].length == 70.0 && tile.splines[1].heightDelta == 3.5 &&
                    tile.splines[1].splineTerrainAlign2 == "raw spline alignment" &&
                    tile.splines[1].rules.size() == 1 &&
                    tile.splines[1].rules[0].fields ==
                        std::array<std::string, 4>{"spline rule 1", "spline rule 2",
                                                   "spline rule 3", "spline rule 4"},
                "elevated spline geometry, delta height, and raw spline modifiers parsed");
        require(tile.sceneryObjects.size() == 1 && tile.sceneryObjects[0].id == 5 &&
                tile.sceneryObjects[0].editorObjectNumber == 4 &&
                    tile.sceneryObjects[0].label == "Object Nr. 4" &&
                    tile.sceneryObjects[0].line1 == "0" &&
                    tile.sceneryObjects[0].assetPath == "Scenery\\tree.sco" &&
                    tile.sceneryObjects[0].transformValid &&
                    tile.sceneryObjects[0].variableParentId == 3 &&
                    tile.sceneryObjects[0].splineTerrainAlign &&
                    tile.sceneryObjects[0].rules.size() == 2 &&
                    !tile.sceneryObjects[0].rules[0].kill &&
                    tile.sceneryObjects[0].rules[1].kill &&
                    tile.sceneryObjects[0].localPosition[0] == 1.0 &&
                    tile.sceneryObjects[0].localPosition[1] == 2.0 &&
                    tile.sceneryObjects[0].rotationDegrees == std::array<double, 3>{0.0, 0.0, 0.0} &&
                    tile.sceneryObjects[0].labels.size() == 3 &&
                    tile.sceneryObjects[0].labels[1] == "[object]" &&
                    tile.sceneryObjects[0].labels[2].empty(),
                "scenery object placement fields preserved without treating label text as a section");
        require(tile.splineAttachments.size() == 2 &&
                    tile.splineAttachments[0].assetPath == "Scenery\\lamp.sco" &&
                    tile.splineAttachments[0].editorObjectNumber == 5 &&
                    tile.splineAttachments[0].transformValid &&
                    tile.splineAttachments[0].splineIndex == 1 &&
                    tile.splineAttachments[0].offset[0] == 3.0 &&
                    tile.splineAttachments[0].offset[1] == -0.2 &&
                    tile.splineAttachments[0].offset[2] == 4.0 &&
                    tile.splineAttachments[0].rotationDegrees[0] == 90.0 &&
                    tile.splineAttachments[0].interval == 10.0 &&
                    tile.splineAttachments[0].range == 2.0 &&
                    tile.splineAttachments[0].variableParentId == 23 &&
                    tile.splineAttachments[0].splineTerrainAlign &&
                    tile.splineAttachments[0].rules.size() == 1 &&
                    tile.splineAttachments[0].rules[0].kill &&
                    !tile.splineAttachments[0].tilt,
                "spline attachment fields decoded as typed spline-relative values");
        require(tile.splineAttachments[1].repeater &&
                    tile.splineAttachments[1].repeaterMasterTileIndex == 0 &&
                    tile.splineAttachments[1].repeaterFirstObjectIndex == 1 &&
                    tile.splineAttachments[1].splineIndex == 0 &&
                    tile.splineAttachments[1].interval == 10.0 &&
                    tile.splineAttachments[1].range == 20.0,
                "spline repeater master and row fields decoded");

        const auto loadVersionedObject = [&](const std::string& version,
                                             const std::string& record) {
            const std::string filename = "tile_version_" + version + ".map";
            const auto path = root / filename;
            writeUtf16Le(path, "[version]\n" + version + "\n[terrain]\n" + record);
            openbus::map::MapTileReference reference;
            reference.textPath = path;
            return openbus::map::loadMapTile(reference);
        };
        const auto version5 = loadVersionedObject(
            "5", "[object]\nScenery\\legacy5.sco\n10\n20\n3\n1\nfirst label\n");
        require(version5.sceneryObjects.size() == 1 && version5.sceneryObjects[0].id == -1 &&
                    version5.sceneryObjects[0].localPosition == std::array<double, 3>{10.0, 20.0, 3.0} &&
                    version5.sceneryObjects[0].rotationDegrees ==
                        std::array<double, 3>{0.0, 0.0, 0.0} &&
                    version5.sceneryObjects[0].labels == std::vector<std::string>{"first label"},
                "version 5 object omits ID and rotations but retains counted labels");
        const auto version7 = loadVersionedObject(
            "7", "[object]\nScenery\\legacy7.sco\n77\n1\n2\n3\n0\n");
        require(version7.sceneryObjects.size() == 1 && version7.sceneryObjects[0].id == 77 &&
                    version7.sceneryObjects[0].rotationDegrees ==
                        std::array<double, 3>{0.0, 0.0, 0.0} &&
                    version7.sceneryObjects[0].labels.empty(),
                "version 7 object has an ID but no heading or pitch/bank fields");
        const auto version11 = loadVersionedObject(
            "11", "[object]\n0\nScenery\\legacy11.sco\n111\n1\n2\n3\n90\n0\n");
        require(version11.sceneryObjects.size() == 1 &&
                    version11.sceneryObjects[0].detailLevel == 0 &&
                    version11.sceneryObjects[0].rotationDegrees ==
                        std::array<double, 3>{90.0, 0.0, 0.0},
                "version 11 object reads detail and heading only");
        const auto version12 = loadVersionedObject(
            "12", "[object]\n0\nScenery\\legacy12.sco\n112\n1\n2\n3\n90\n4\n5\n0\n");
        require(version12.sceneryObjects.size() == 1 &&
                    version12.sceneryObjects[0].localPosition ==
                        std::array<double, 3>{1.0, 2.0, 3.0} &&
                    version12.sceneryObjects[0].rotationDegrees ==
                        std::array<double, 3>{90.0, 4.0, 5.0},
                "version 12 object preserves x/y/z and reads heading, pitch, and bank");

        const auto loadVersionedSpline = [&](const std::string& version,
                                             const std::string& record) {
            const std::string filename = "tile_spline_version_" + version + ".map";
            const auto path = root / filename;
            writeUtf16Le(path, "[version]\n" + version + "\n[terrain]\n" + record);
            openbus::map::MapTileReference reference;
            reference.textPath = path;
            return openbus::map::loadMapTile(reference);
        };
        const auto version5Spline = loadVersionedSpline(
            "5", "[spline]\nSplines\\legacy.sli\n-1\n10\n20\n0.5\n90\n40\n-20\n1\n2\n3\n4\n");
        require(version5Spline.splines.size() == 1 &&
                    version5Spline.splines[0].splineId == -1 &&
                    version5Spline.splines[0].assetPath == "Splines\\legacy.sli" &&
                    version5Spline.splines[0].geometryValid &&
                    version5Spline.splines[0].localX == 10.0 &&
                    version5Spline.splines[0].elevation == 20.0 &&
                    version5Spline.splines[0].localY == 0.5 &&
                    version5Spline.splines[0].cantStart == 3.0,
                "version 5 spline omits detail, ID, and neighbor IDs but retains cant fields");
        const auto version5MixedRecords = loadVersionedSpline(
            "5", "[spline]\nSplines\\legacy.sli\n-1\n10\n20\n0.5\n90\n40\n-20\n1\n2\n3\n4\n"
                  "[object]\nScenery\\legacy.sco\n1\n2\n0\n0\n"
                  "Legacy attachment\n[attachObj]\n0\nScenery\\legacy-attached.sco\n44\n"
                  "0\n1\n0\n0\n0\n0\n");
        require(version5MixedRecords.splines.size() == 1 &&
                    version5MixedRecords.splines[0].splineId == -1 &&
                    version5MixedRecords.sceneryObjects.size() == 1 &&
                    version5MixedRecords.sceneryObjects[0].id == -2 &&
                    version5MixedRecords.attachedObjects.size() == 1 &&
                    version5MixedRecords.attachedObjects[0].id == -3 &&
                    version5MixedRecords.attachedObjects[0].attachedToObjectId == 44 &&
                    version5MixedRecords.attachedObjects[0].attachPointIndexValue == 1 &&
                    version5MixedRecords.attachedObjects[0].rotationDegrees ==
                        std::array<double, 3>{0.0, 0.0, 0.0} &&
                    version5MixedRecords.attachedObjects[0].transformValid &&
                    !version5MixedRecords.attachedObjects[0].hasExplicitId,
                "pre-v6 automatic IDs are shared across spline, object, and attached-object records");
        const auto malformedAttachedObject = loadVersionedSpline(
            "14", "Attached object\n[attachObj]\n0\nScenery\\opaque.sco\n99\n5\n0\n"
                  "bad-index\n0\n2.5\nNaN\nnot-a-count\nopaque tail\n");
        require(malformedAttachedObject.attachedObjects.size() == 1 &&
                    !malformedAttachedObject.attachedObjects[0].transformValid &&
                    malformedAttachedObject.attachedObjects[0].attachPointIndex == "bad-index" &&
                    malformedAttachedObject.attachedObjects[0].rotationFields ==
                        std::array<std::string, 3>{"0", "2.5", "NaN"} &&
                    malformedAttachedObject.attachedObjects[0].labelCount == "not-a-count" &&
                    malformedAttachedObject.attachedObjects[0].optionalLines ==
                        std::vector<std::string>{"opaque tail", ""},
                "attached-object numeric conversions are non-destructive and count stays opaque");
        const auto version7Splines = loadVersionedSpline(
            "7", "[spline]\nSplines\\legacy.sli\n71\n-1\n10\n20\n0.5\n90\n40\n-20\n1\n2\n"
                  "3\n4\n[spline]\nSplines\\legacy.sli\n72\n1\n11\n21\n0.5\n90\n40\n-20\n1\n2\n3\n4\n");
        require(version7Splines.splines.size() == 2 &&
                    version7Splines.splines[0].nextSplineId == 72 &&
                    version7Splines.splines[1].previousSplineId == 71 &&
                    version7Splines.splines[1].cantStart == 3.0 &&
                    version7Splines.splines[1].geometryValid,
                "version 7 legacy link flag is expanded to linked spline IDs");
        const auto version11Spline = loadVersionedSpline(
            "11", "[spline]\n0\nSplines\\modern.sli\n111\n110\n112\n1\n2\n0.5\n45\n50\n-30\n0\n0\n1.5\n2.5\n18\n");
        require(version11Spline.splines.size() == 1 &&
                    version11Spline.splines[0].splineId == 111 &&
                    version11Spline.splines[0].previousSplineId == 110 &&
                    version11Spline.splines[0].nextSplineId == 112 &&
                    version11Spline.splines[0].chainOffsetValid &&
                    version11Spline.splines[0].chainOffset == 18.0,
                "version 11 spline reads detail, neighbor IDs, cant, and chain texture offset");
        const auto version14Spline = loadVersionedSpline(
            "14", "[spline]\n0\nSplines\\v14.sli\n141\n140\n142\n10\n20\n0.5\n45\n50\n-30\n0\n0\n1.5\n2.5\n11.25\n22.5\n77\nmirror\n");
        require(version14Spline.splines.size() == 1 &&
                    version14Spline.splines[0].geometryValid &&
                    version14Spline.splines[0].cantStart == 1.5 &&
                    version14Spline.splines[0].cantEnd == 2.5 &&
                    version14Spline.splines[0].skewStart == 11.25 &&
                    version14Spline.splines[0].skewEnd == 22.5 &&
                    version14Spline.splines[0].mirrored &&
                    version14Spline.splines[0].chainOffsetValid &&
                    version14Spline.splines[0].chainOffset == 77.0,
                "version 14 spline skips skew fields before reading chain texture offset");

        const openbus::map::TerrainGrid terrain =
            openbus::map::loadTerrainGrid(definition.tiles[0].terrainPath);
        require(terrain.intervals == 2 && terrain.heights.size() == 9 &&
                    terrain.heights.front() == 1.0F && terrain.heights.back() == 9.0F,
                "little-endian terrain grid decoded");
        const openbus::map::WaterData water =
            openbus::map::loadWaterData(definition.tiles[0].waterPath);
        require(water.surfaces.size() == 2 &&
                    water.surfaces[0].heights ==
                        std::array<float, 4>{1.25F, 2.5F, 3.75F, 5.0F} &&
                    water.surfaces[1].heights ==
                        std::array<float, 4>{-1.0F, 0.0F, 12.5F, 40.0F},
                "water sidecar count and four little-endian heights per surface decoded");
        const std::filesystem::path malformedWaterPath = root / "malformed.map.water";
        {
            std::ofstream malformedWater(malformedWaterPath, std::ios::binary);
            writeU32(malformedWater, 1);
            writeU32(malformedWater, std::bit_cast<std::uint32_t>(1.0F));
        }
        bool malformedWaterRejected = false;
        try {
            static_cast<void>(openbus::map::loadWaterData(malformedWaterPath));
        } catch (const std::runtime_error&) {
            malformedWaterRejected = true;
        }
        require(malformedWaterRejected, "truncated water sidecar is rejected");
    } catch (...) {
        std::filesystem::remove_all(root);
        throw;
    }
    std::filesystem::remove_all(root);
}

void runInstalledMapProbe(const std::filesystem::path& root) {
    const openbus::map::MapDefinition definition = openbus::map::loadMapDefinition(root);
    std::size_t terrainCount = 0;
    std::size_t groundTextureSidecarCount = 0;
    std::size_t lightmapSidecarCount = 0;
    std::size_t terrainReadinessSidecarCount = 0;
    std::size_t splineCount = 0;
    std::size_t objectCount = 0;
    std::size_t sceneryPlacementCount = 0;
    for (const openbus::map::MapTileReference& reference : definition.tiles) {
        groundTextureSidecarCount += reference.groundTextureSidecars.size();
        lightmapSidecarCount += reference.hasLightmapFile ? 1 : 0;
        terrainReadinessSidecarCount += reference.hasTerrainReadinessFile ? 1 : 0;
        const openbus::map::MapTileData tile = openbus::map::loadMapTile(reference);
        splineCount += tile.splineCount + tile.elevatedSplineCount;
        sceneryPlacementCount += tile.sceneryObjects.size() + tile.splineAttachments.size();
        objectCount += tile.objectCount + tile.attachedObjects.size() +
                       tile.splineAttachmentCount + tile.splineRepeaterCount;
        if (reference.hasTerrainFile) {
            const openbus::map::TerrainGrid terrain =
                openbus::map::loadTerrainGrid(reference.terrainPath);
            require(terrain.heights.size() == (terrain.intervals + 1) * (terrain.intervals + 1),
                    "terrain vertex count matches interval header");
            ++terrainCount;
        }
    }
    std::cout << "tiles=" << definition.tiles.size()
              << " groundtex=" << definition.groundTextures.size()
              << " entrypoints=" << definition.entryPoints.size()
              << " global_name=" << definition.metadata.name
              << " seasons=" << definition.metadata.seasons.size()
              << " road_density=" << definition.metadata.roadTrafficDensity.size()
              << " passenger_density=" << definition.metadata.passengerTrafficDensity.size()
              << " groundtex_sidecars=" << groundTextureSidecarCount
              << " lightmaps=" << lightmapSidecarCount
              << " terrain_readiness=" << terrainReadinessSidecarCount
              << " terrain=" << terrainCount << " spline_records=" << splineCount
              << " object_records=" << objectCount
              << " scenery_placements=" << sceneryPlacementCount << '\n';
    if (root.filename() == "Grande Porto 2022") {
        require(definition.metadata.name == "Grande Porto 2022" &&
                    definition.metadata.friendlyName == "Grande Porto 2022" &&
                    definition.metadata.version == "14" &&
                    definition.metadata.seasons.size() == 5 &&
                    definition.metadata.roadTrafficDensity.size() == 1 &&
                    definition.metadata.passengerTrafficDensity.size() == 11 &&
                    groundTextureSidecarCount == 710,
                "Grande Porto metadata and numeric tile DDS inventory match the installed map");
        const VehiclePlacement& spawn = definition.entryPoints.front().placement;
        require(definition.entryPoints.front().name == "Porto (Boavista-Bom Sucesso)",
                "Grande Porto first entrypoint selected as the default spawn");
        require(std::abs(spawn.position[0] - -2009.882) < 1.0e-6 &&
                    std::abs(spawn.position[1] - 3586.663) < 1.0e-6 &&
                    std::abs(spawn.position[2] - 64.540) < 1.0e-6,
                "Grande Porto default spawn uses tile-local x/z/y fields");
        std::cout << "default_spawn=" << definition.entryPoints.front().name
                  << " position=" << spawn.position[0] << ',' << spawn.position[1] << ','
                  << spawn.position[2] << " yaw=" << spawn.yawDegrees << '\n';
    }
}

} // namespace

int main(int argc, char** argv) {
    try {
        if (argc == 1) {
            runFixtureProbe();
        } else if (argc == 2) {
            runInstalledMapProbe(argv[1]);
        } else {
            std::cerr << "usage: OpenBusMapConfigProbe [map-directory]\n";
            return 2;
        }
    } catch (const std::exception& error) {
        std::cerr << "map config probe failed: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
