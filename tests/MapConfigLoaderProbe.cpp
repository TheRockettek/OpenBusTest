#include "map/MapConfigLoader.h"
#include "ProbeTempDirectory.h"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
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
        openbus::probe::makeProbeDirectory("openbus-map-config-probe");
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
                     "[dynhelperactive]\n[realrail]\n[LHT]\n[backgroundimage]\n1\n2\n3\n4\n5\n6\n"
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
                     "3\n12\n0\n30\n5\n40\n2\n5\n3\n7\n2\nDepot East\n"
                     "[map]\n0\n0\ntile_0_0.map\n[map]\n0\n0\ntile_0_0.map\n"
                     "[map]\n1\n0\ntile_1_0.map\n");
        writeUtf16Le(root / "Tickets.otp",
                     "[ticketpack]\n0.1\n0.75\n0.2\n0.3\n"
                     "[ticket]\n\"Single Ride\"\n\"Single Ride\"\n0\n0\n120\n1.5\n\"1.50\"\n"
                     "[ticket]\nDay Pass\nDay Pass\n0\n16\n120\n6\n6.00\n");
        writeUtf16Le(root / "LegacyTickets.otp",
                     "[ticketpack]\n0.5\n[ticket]\n\"Legacy Ride\"\n\"2.00\"\n2\n0\n");
        writeUtf16Le(root / "Money.cfg", "[currency]\nCredits\n2\n[coin]\ncoins\\small.o3d\n0.05\n"
                                         "[coin]\ncoins\\large.o3d\n1.00\n");
        writeUtf16Le(root / "tile_0_0.map",
                     "Fixture tile\n[version]\n14\n[terrain]\n[water]\n"
                     "[unknown_future_section]\n opaque tile payload \n\n"
                     "[variable_terrain]\n[variable_terrainlightmap]\n\n"
                     "[spline]\n0\nSplines\\road.sli\n30273\n30272\n0\n10\n20\n30\n"
                     "0\n50\n0\n0\n0\n0\n0\n0\n0\n0\n\n"
                     "[spline_h]\n0\nSplines\\bridge.sli\n30274\n0\n0\n11\n21\n31\n"
                     "0\n70\n0\n12\n12\n3.5\n3.5\n-2.5\n0\n0\n0\n\n"
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
                     "2\n30\n4\n0\n1\nCar park B\ncustom tail\n\n[varparent]\n41\n"
                     "[spline_terrain_align]\n[rule]\nattached rule 1\nattached rule 2\n"
                     "attached rule 3\nattached rule 4\n");
        writeUtf16Le(root / "tile_1_0.map", "Second fixture tile\n[version]\n14\n[terrain]\n");
        writeUtf16Le(root / "tile_invalid_version.map",
                     "Invalid version fixture\n[version]\n12junk\n[terrain]\n"
                     "[splineAttachement]\n");
        writeUtf16Le(root / "ailists.cfg",
                     "[aigroup_2]\nNormalCars\nvehicles\\sedan.bus\t7\n"
                     "vehicles\\delivery van.ovh\n[end]\n"
                     "[aigroup_2]\nTrucks\nvehicles\\truck.bus\t3\n"
                     "vehicles\\truck.ovh\n[end]\n"
                     "[aigroup_2]\nCommercial\nvehicles\\commercial.bus\n[end]\n"
                     "[aigroup_2]\nCommercials\nvehicles\\commercial_van.bus\n"
                     "vehicles\\delivery van.ovh\n[end]\n"
                     "[aigroup_2]\nOther\nvehicles\\other.bus\n[end]\n"
                     "[aigroup_2]\nNormalCar\nvehicles\\normal_car.ovh\n[end]\n"
                     "[aigroup_2]\nAmbulance\nvehicles\\ambulance.ovh\n[end]\n"
                     "[aigroup_2]\nSudden Stop Vehicles\nvehicles\\sudden_stop.bus\n[end]\n"
                     "[aigroup_depot]\nCentral Depot\nFixture HOF\n"
                     "[aigroup_depot_typgroup_2]\nvehicles\\city_bus.bus\n"
                     "1234\tBlue livery\n[end]\n");
        writeUtf16Le(root / "unsched_vehgroups.txt",
                     "[group]\n<aigroup-name>\n0\n[group]\nTrucks\n0\n"
                     "[group]\nNormalCars\n0\n[group]\nCommercials\n0\n"
                     "[group]\nAmbulance\n0\n[group]\nSudden Stop Vehicles\n0\n");
        writeUtf16Le(root / "unsched_trafficdens.txt",
                     "[group]\nNormalCars\n2\n[set_day_of_week]\n0\n"
                     "[trafficdensity]\n0\n0.3\n[trafficdensity]\n12\n0.3\n"
                     "[set_day_of_week]\n8\n"
                     "[set_day_of_week]\n1\n"
                     "[trafficdensity]\n0\n0.2\n[trafficdensity]\n12\n0.8\n"
                     "[set_day_of_week]\n4\n[trafficdensity]\n0\n0.1\n"
                     "[trafficdensity]\n12\n0.1\n[group]\nTrucks\n1\n"
                     "[set_day_of_week]\n0\n[trafficdensity]\n0\n0.4\n"
                     "[trafficdensity]\n12\n0.2\n");
        writeUtf16Le(root / "drivers.txt", "humans\\drivers\\driver_one.hum\n"
                                           "humans\\custom map\\driver_two.hum\n");
        writeUtf16Le(root / "humans.txt", "humans\\pedestrians\\walker_one.hum\n"
                                          "; ignored comment\n"
                                          "humans\\pedestrians\\walker_two.hum\n");
        const std::filesystem::path timetableDirectory = root / "TTData";
        std::filesystem::create_directories(timetableDirectory);
        writeUtf16Le(timetableDirectory / "FixtureTrip.ttp",
                     "[trip]\nFixtureLine\nTown End\n12\n"
                     "[station]\n15479\n1\nMain Stop\n0\n0\n0\n3.5\n5\n"
                     "[station]\n15480\n1\nInvalid Tile\n-1\n0\n0\n0\n5\n"
                     "[profile]\nStandard\n13\n"
                     "[profile_man_arr_time]\n0\n0.250\n"
                     "[profile_man_dep_time]\n0\n0.500\n"
                     "[profile]\nExpress\n8\n"
                     "[profile_man_arr_time]\n0\n1.500\n"
                     "[profile_man_dep_time]\n0\n2.000\n");
        writeUtf16Le(timetableDirectory / "BusOnly.ttp",
                     "[trip]\n\nTown End\n12\n"
                     "[station_typ2]\n15479\n[station_typ2]\n15480\n"
                     "[profile]\nStandard\n12\n"
                     "[profile_man_arr_time]\n0\n1.000\n"
                     "[profile_man_dep_time]\n0\n1.250\n");
        writeUtf16Le(timetableDirectory / "TrackFixture.ttp",
                     "[trip]\nTrackFixture\nTown End\n12\n"
                     "[station_typ2]\n15479\n[station_typ2]\n15480\n"
                     "[profile]\nStandard\n12\n");
        writeUtf16Le(timetableDirectory / "Broken.ttp",
                     "[trip]\nBroken\nTown End\n12\n"
                     "[station]\n19001\n0\nFirst Stop\n0\n0\n0\n0\n0\n"
                     "[station]\n19002\n0\nLast Stop\n0\n0\n0\n0\n0\n");
        writeUtf16Le(timetableDirectory / "Broken.ttr", "[track_entry]\ninvalid\n");
        writeUtf16Le(timetableDirectory / "BusOnly.ttl", "[newtour]\nBus\n1\nDep.: 8:15:0\n"
                                                         "[addtrip]\nBusOnly\n0\n495.000\n");
        writeUtf16Le(timetableDirectory / "Busstops.cfg",
                     "[busstop]\nMain Stop\n0\n15479\n0\n0\n0\n"
                     "[busstop]\nIncomplete Stop\n"
                     "[busstop]\nTown End\n0\n15480\n0\n0\n0\n"
                     "[busstop]\nInvalid Stop\n0\n15481\ninvalid\n0\n0\n");
        writeUtf16Le(timetableDirectory / "StnLinks.cfg",
                     "[StnLink]\n"
                     "[StnLink]\n12.5\n15479\n15480\n0\n0\n0\n0\n0\n0\n"
                     "[StnLink_entry]\n1\n2\n3\n4.5\n5\n6\n7\n"
                     "[StnLink_entry]\n2\n3\n4\n5.5\n6\n7\n8\n"
                     "[StnLink]\ninvalid-length\n15479\n15480\n0\n0\n0\n0\n0\n0\n");
        writeUtf16Le(timetableDirectory / "FixtureLine.ttr",
                     "[track_entry]\n30273\n0\n1\n1\n10\n0\n"
                     "[track_entry]\n30273\n1\n1\n2\n10\n0\n"
                     "[track_entry]\n30274\n0\n0\n3\n10\n0\n");
        writeUtf16Le(timetableDirectory / "TrackFixture.ttr",
                     "[track_entry]\n1\n2\n3\n0\n4.5\n0\n"
                     "[track_entry]\n2\n3\n4\n0\n5.5\n0\n");
        writeUtf16Le(timetableDirectory / "Fixture.ttl",
                     "[newtour]\nDriver\n1\n"
                     "Dep.: 8:15:0\n[addtrip]\nFixtureTrip\n1\n495.000\n"
                     "Dep.: 6:45:0\n[addtrip]\nFixtureTrip\n0\n405.000\n"
                     "[addtrip]\nBroken\n0\n515.000\n");
        writeUtf16Le(timetableDirectory / "TrackFixture.ttl",
                     "[newtour]\nDriver\n1\nDep.: 8:15:0\n"
                     "[addtrip]\nTrackFixture\n0\n495.000\n");
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
        const std::filesystem::path standardAiListDirectory = root / "standard-ai-list";
        std::filesystem::create_directories(standardAiListDirectory);
        std::filesystem::copy_file(root / "global.cfg", standardAiListDirectory / "global.cfg");
        std::filesystem::copy_file(root / "tile_0_0.map",
                                   standardAiListDirectory / "tile_0_0.map");
        std::filesystem::copy_file(root / "tile_1_0.map",
                                   standardAiListDirectory / "tile_1_0.map");
        writeUtf16Le(standardAiListDirectory / "ailists.cfg",
                     "[ailist]\n0\n-1\n4\n"
                     "vehicles\\car.ovh\n"
                     "vehicles\\city_bus.bus\n"
                     "vehicles\\van.ovh\n"
                     "vehicles\\articulated.bus\n"
                     "[aigroup_depot]\nCentral Depot\nFixture HOF\n"
                     "[aigroup_depot_typgroup]\n1\n1234\tBlue livery\nFixture HOF\n[end]\n"
                     "[aigroup_depot_typgroup]\n3\n5678\tRed livery\nFixture HOF\n[end]\n");
        const openbus::map::MapDefinition standardAiListDefinition =
            openbus::map::loadMapDefinition(standardAiListDirectory);
        require(standardAiListDefinition.aiTraffic.vehicleGroups.size() == 1 &&
                    standardAiListDefinition.aiTraffic.vehicleGroups[0].name == "NormalCars" &&
                    openbus::map::mapAiRoadVehicleAssetPaths(
                        standardAiListDefinition.aiTraffic) ==
                        std::vector<std::string>{"vehicles\\car.ovh", "vehicles\\city_bus.bus",
                                                 "vehicles\\van.ovh",
                                                 "vehicles\\articulated.bus"} &&
                    openbus::map::mapAiBusAssetPaths(standardAiListDefinition.aiTraffic) ==
                        std::vector<std::string>{"vehicles\\city_bus.bus",
                                                 "vehicles\\articulated.bus"},
                "standard [ailist] assets provide road traffic and depot typgroups resolve bus indices");
        const std::filesystem::path unicodeMapDirectory =
            root / std::filesystem::path(std::u8string(u8"map-\u00e9"));
        std::filesystem::create_directories(unicodeMapDirectory);
        writeUtf16Le(unicodeMapDirectory / "global.cfg",
                     "[entrypoints]\n1\n2\n11\n0\n10\n20\n0\n0\n0\n0\n1\n1\nMissing tile spawn\n"
                     "[map]\n-10\n-15\ntile_-10_-15.map\n"
                     "[map]\n2413\n11273\ntile_100_100.map\n");
        writeUtf16Le(unicodeMapDirectory / "tile_-10_-15.map", "");
        openbus::map::MapDefinition edgeCaseDefinition =
            openbus::map::loadMapDefinition(unicodeMapDirectory);
        const auto hasWarning = [&](const std::string& text) {
            return std::any_of(
                edgeCaseDefinition.diagnostics.entries.begin(),
                edgeCaseDefinition.diagnostics.entries.end(),
                [&](const auto& diagnostic) {
                    return diagnostic.message.find(text) != std::string::npos;
                });
        };
        require(edgeCaseDefinition.tiles.size() == 1 &&
                    edgeCaseDefinition.tiles[0].x == -10 &&
                    edgeCaseDefinition.tiles[0].y == -15 &&
                    edgeCaseDefinition.entryPoints.empty() &&
                    !edgeCaseDefinition.diagnostics.hasErrors() &&
                    hasWarning("coordinates do not match tile filename") &&
                    hasWarning("listed map tile does not exist") &&
                    hasWarning("entry point references a missing tile"),
                "missing and mismatched tile records warn while UTF-8 map paths still load");
        const std::filesystem::path chronoScenarioDirectory =
            unicodeMapDirectory / "Chrono" / "scenario";
        std::filesystem::create_directories(chronoScenarioDirectory);
        writeUtf16Le(chronoScenarioDirectory / "chrono.cfg", "[startdate]\n20200101\n");
        writeUtf16Le(chronoScenarioDirectory / "tile_-10_-15.map", "");
        require(openbus::map::applyMapChronoTiles(edgeCaseDefinition, 20200101) == 1,
                "Chrono tile application handles Unicode map paths");
        require(definition.metadata.name == "Fixture map" &&
                    definition.metadata.friendlyName == "Friendly fixture" &&
                    definition.metadata.description == "First line\n\nSecond line" &&
                    definition.metadata.version == "14" &&
                    definition.metadata.nextIdCode == "81" &&
                    definition.metadata.hasWorldCoordinates &&
                    definition.metadata.hasDynamicHelperActive && definition.metadata.hasRealRail &&
                    definition.metadata.hasLeftHandTraffic,
                "global map identity, description, version, and marker metadata captured");
        const auto ticketPack = openbus::map::loadMapTicketPackDefinition(root / "Tickets.otp");
        const auto legacyTicketPack =
            openbus::map::loadMapTicketPackDefinition(root / "LegacyTickets.otp");
        const auto currency = openbus::map::loadMapCurrencyDefinition(root / "Money.cfg");
        require(ticketPack.tickets.size() == 2 &&
                    ticketPack.stamperProbability == 0.1 &&
                    ticketPack.ticketBuyProbability == 0.75 &&
                    ticketPack.chattiness == 0.2 && ticketPack.whingeProbability == 0.3 &&
                    ticketPack.tickets[0].name == "Single Ride" &&
                    ticketPack.tickets[0].englishName == "Single Ride" &&
                    ticketPack.tickets[0].displayString == "1.50" &&
                    ticketPack.tickets[0].value == 1.5 &&
                    ticketPack.tickets[0].ageMax == 120.0 &&
                    ticketPack.tickets[1].name == "Day Pass" &&
                    ticketPack.tickets[1].ageMin == 16.0 &&
                    ticketPack.tickets[1].value == 6.0 &&
                    legacyTicketPack.ticketBuyProbability == 0.5 &&
                    legacyTicketPack.tickets.size() == 1 &&
                    legacyTicketPack.tickets[0].name == "Legacy Ride" &&
                    legacyTicketPack.tickets[0].value == 2.0 && currency.name == "Credits" &&
                    currency.mode == "2" && currency.coins.size() == 2 &&
                    currency.coins[0].modelPath == "coins\\small.o3d" &&
                    currency.coins[0].value == 0.05 && currency.coins[1].value == 1.0,
                "ticket-pack prices and currency coin denominations are loaded in authored order");
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
                        std::array<std::string, 2>{"0", "0.2"} &&
                    definition.aiTraffic.roadDensity.size() == 1 &&
                    definition.aiTraffic.roadDensity[0].hour == 12.0 &&
                    definition.aiTraffic.roadDensity[0].density == 1.0 &&
                    definition.aiTraffic.passengerDensity.size() == 1 &&
                    definition.aiTraffic.passengerDensity[0].hour == 0.0 &&
                    definition.aiTraffic.passengerDensity[0].density == 0.2,
                "repeated global season and traffic-density records captured");
        const std::vector<openbus::map::MapTrafficDensityPoint> densitySchedule{
            {0.0, 0.1}, {4.0, 0.0}, {6.0, 1.0}};
        require(std::abs(openbus::map::mapTrafficDensityAtHour(densitySchedule, 2.0).value() -
                         0.05) < 1.0e-9 &&
                    std::abs(openbus::map::mapTrafficDensityAtHour(densitySchedule, 5.0).value() -
                             0.5) < 1.0e-9 &&
                    openbus::map::mapTrafficDensityAtHour(densitySchedule, 23.0) == 1.0,
                "density lookup interpolates schedule points and clamps outside their range");
        require(!openbus::map::mapTrafficDensityAtHour({}, 12.0).has_value() &&
                    !openbus::map::mapTrafficDensityAtHour(
                         {{4.0, 0.5}, {4.0, 0.8}}, 4.0)
                         .has_value(),
                "empty and ambiguous density schedules do not return a value");
        require(openbus::map::mapTrafficPopulationForDensity(8, 0.25) == 2 &&
                    openbus::map::mapTrafficPopulationForDensity(8, 0.0) == 0 &&
                    openbus::map::mapTrafficPopulationForDensity(8, 2.0) == 16 &&
                    openbus::map::mapTrafficPopulationForDensity(8, std::nullopt) == 8 &&
                    openbus::map::mapTrafficPopulationForDensity(0, 1.0) == 0 &&
                    openbus::map::mapTrafficPopulationForDensity(200, 2.0) == 400,
                "road density scales a base population without a fixed active cap");
        bool populationOverflowRejected = false;
        try {
            static_cast<void>(openbus::map::mapTrafficPopulationForDensity(
                (std::numeric_limits<std::size_t>::max)(), 2.0));
        } catch (const std::overflow_error&) {
            populationOverflowRejected = true;
        }
        require(populationOverflowRejected, "unrepresentable traffic population is rejected");
        bool invalidDensityHourRejected = false;
        try {
            static_cast<void>(openbus::map::mapTrafficDensityAtHour(densitySchedule, 24.1));
        } catch (const std::invalid_argument&) {
            invalidDensityHourRejected = true;
        }
        require(invalidDensityHourRejected, "density lookup rejects hours outside [0, 24]");
        require(definition.aiTraffic.vehicleGroups.size() == 8 &&
                    definition.aiTraffic.vehicleGroups[0].name == "NormalCars" &&
                    definition.aiTraffic.vehicleGroups[0].vehicles.size() == 2 &&
                    definition.aiTraffic.vehicleGroups[0].vehicles[0].assetPath ==
                        "vehicles\\sedan.bus" &&
                    definition.aiTraffic.vehicleGroups[0].vehicles[0].rawLine ==
                        "vehicles\\sedan.bus\t7" &&
                    definition.aiTraffic.vehicleGroups[0].vehicles[1].assetPath ==
                        "vehicles\\delivery van.ovh" &&
                    definition.aiTraffic.vehicleGroups[0].vehicles[1].rawLine ==
                        "vehicles\\delivery van.ovh" &&
                    definition.aiTraffic.vehicleGroups[0].rawLines ==
                        std::vector<std::string>{"NormalCars", "vehicles\\sedan.bus\t7",
                                                 "vehicles\\delivery van.ovh"},
                "road vehicle references are extracted while full group lines are preserved");
        require(openbus::map::mapAiRoadVehicleAssetPaths(definition.aiTraffic) ==
                    std::vector<std::string>{"vehicles\\sedan.bus",
                                             "vehicles\\delivery van.ovh",
                                             "vehicles\\truck.bus",
                                             "vehicles\\truck.ovh",
                                             "vehicles\\commercial.bus",
                                             "vehicles\\commercial_van.bus",
                                             "vehicles\\normal_car.ovh",
                                             "vehicles\\ambulance.ovh",
                                             "vehicles\\sudden_stop.bus"} &&
                    openbus::map::mapAiBusAssetPaths(definition.aiTraffic) ==
                        std::vector<std::string>{"vehicles\\city_bus.bus"},
                "random road traffic selects .ovh and .bus assets from supported groups while "
                "scheduled buses remain sourced from depot definitions");
        const std::vector<openbus::map::MapAiRoadVehicle> roadVehicles =
            openbus::map::mapAiRoadVehicles(definition.aiTraffic);
        openbus::map::MapAiTrafficData legacyGroupTraffic;
        legacyGroupTraffic.vehicleGroups = {
            {"Normal", {}, {{"vehicles\\normal.bus", "vehicles\\normal.bus\t1"}}},
            {"Emergency", {}, {{"vehicles\\emergency.bus", "vehicles\\emergency.bus\t1"}}},
            {"Taxi", {}, {{"vehicles\\taxi.bus", "vehicles\\taxi.bus\t1"}}}};
        const std::vector<openbus::map::MapAiRoadVehicle> legacyGroupVehicles =
            openbus::map::mapAiRoadVehicles(legacyGroupTraffic);
        openbus::map::MapAiTrafficData explicitGroupTraffic;
        explicitGroupTraffic.vehicleGroups = {
            {"Special Traffic", {}, {{"vehicles\\special.ovh", "vehicles\\special.ovh"}}},
            {"Taxi", {}, {{"vehicles\\taxi.bus", "vehicles\\taxi.bus\t1"}}},
            {"Emergency", {}, {{"vehicles\\emergency.bus", "vehicles\\emergency.bus\t1"}}}};
        explicitGroupTraffic.unscheduledVehicleGroupNames = {"Taxi", "Special Traffic",
                                                             "Emergency"};
        const std::vector<openbus::map::MapAiRoadVehicle> explicitGroupVehicles =
            openbus::map::mapAiRoadVehicles(explicitGroupTraffic);
        require(definition.aiTraffic.unscheduledVehicleGroupNames ==
                    std::vector<std::string>{"Trucks", "NormalCars", "Commercials", "Ambulance",
                                             "Sudden Stop Vehicles"} &&
                    legacyGroupVehicles.size() == 1 &&
                    legacyGroupVehicles[0].trafficRuleGroupIndex == 0 &&
                    legacyGroupVehicles[0].groupName == "normalcars" &&
                    explicitGroupVehicles.size() == 3 &&
                    explicitGroupVehicles[0].groupIndex == 1 &&
                    explicitGroupVehicles[0].trafficRuleGroupIndex == 1 &&
                    explicitGroupVehicles[1].groupIndex == 0 &&
                    explicitGroupVehicles[1].trafficRuleGroupIndex == 0 &&
                    explicitGroupVehicles[2].groupIndex == 2 &&
                    explicitGroupVehicles[2].trafficRuleGroupIndex == 2 &&
                    definition.aiTraffic.hasUnscheduledTrafficDensityFile &&
                    definition.aiTraffic.unscheduledTrafficDensityGroups.size() == 2 &&
                    definition.aiTraffic.unscheduledTrafficDensityGroups[0].name == "NormalCars" &&
                    definition.aiTraffic.unscheduledTrafficDensityGroups[0].factor == 2.0 &&
                    definition.aiTraffic.unscheduledTrafficDensityGroups[0].schedules.size() == 3 &&
                    definition.aiTraffic.unscheduledTrafficDensityGroups[0].schedules[0]
                            .dayOfWeekMask == 0 &&
                    definition.aiTraffic.unscheduledTrafficDensityGroups[0].schedules[1]
                            .dayOfWeekMask == 1 &&
                    definition.aiTraffic.unscheduledTrafficDensityGroups[0].schedules[1]
                            .points.size() == 2 &&
                    definition.aiTraffic.unscheduledTrafficDensityGroups[0].schedules[1]
                            .points[1]
                            .hour == 12.0 &&
                    definition.aiTraffic.unscheduledTrafficDensityGroups[0].schedules[1]
                            .points[1]
                            .density == 0.8 &&
                    roadVehicles.size() == 10 &&
                    roadVehicles[0].assetPath == "vehicles\\sedan.bus" &&
                    roadVehicles[0].groupIndex == 1 &&
                    roadVehicles[0].trafficRuleGroupIndex == 1 &&
                    roadVehicles[1].assetPath == "vehicles\\delivery van.ovh" &&
                    roadVehicles[1].groupIndex == 1 &&
                    roadVehicles[1].trafficRuleGroupIndex == 1 &&
                    roadVehicles[2].assetPath == "vehicles\\truck.bus" &&
                    roadVehicles[2].groupIndex == 0 &&
                    roadVehicles[2].trafficRuleGroupIndex == 0 &&
                    roadVehicles[2].defaultVehicleType == 3 &&
                    roadVehicles[3].assetPath == "vehicles\\truck.ovh" &&
                    roadVehicles[3].groupIndex == 0 &&
                    roadVehicles[3].trafficRuleGroupIndex == 0 &&
                    roadVehicles[3].defaultVehicleType == 3 &&
                    roadVehicles[4].assetPath == "vehicles\\commercial.bus" &&
                    roadVehicles[4].groupIndex == 2 &&
                    roadVehicles[4].trafficRuleGroupIndex == 2 &&
                    roadVehicles[5].assetPath == "vehicles\\commercial_van.bus" &&
                    roadVehicles[5].groupIndex == 2 &&
                    roadVehicles[5].trafficRuleGroupIndex == 2 &&
                    roadVehicles[6].assetPath == "vehicles\\delivery van.ovh" &&
                    roadVehicles[6].groupIndex == 2 &&
                    roadVehicles[6].trafficRuleGroupIndex == 2 &&
                    roadVehicles[7].assetPath == "vehicles\\normal_car.ovh" &&
                    roadVehicles[7].groupIndex == 1 &&
                    roadVehicles[7].trafficRuleGroupIndex == 1 &&
                    roadVehicles[8].assetPath == "vehicles\\ambulance.ovh" &&
                    roadVehicles[8].groupIndex == 3 &&
                    roadVehicles[8].trafficRuleGroupIndex == 3 &&
                    roadVehicles[9].assetPath == "vehicles\\sudden_stop.bus" &&
                    roadVehicles[9].groupIndex == 4 &&
                    roadVehicles[9].trafficRuleGroupIndex == 4,
                "both population scheduling and path rules use unsched_vehgroups.txt group "
                "indices, regardless of AI-list group names");
        const std::vector<openbus::map::MapAiRoadVehicle> weekdayRoadVehicles =
            openbus::map::mapAiRoadVehicles(definition.aiTraffic, 12.0, 1);
        const std::optional<double> weekdayGroupDensity =
            openbus::map::mapAiRoadTrafficDensityAtHour(
                definition.aiTraffic, roadVehicles, 12.0, 1);
        const std::optional<double> weekendGroupDensity =
            openbus::map::mapAiRoadTrafficDensityAtHour(
                definition.aiTraffic, roadVehicles, 12.0, 4);
        const std::optional<double> saturdayGroupDensity =
            openbus::map::mapAiRoadTrafficDensityAtHour(
                definition.aiTraffic, roadVehicles, 12.0, 2);
        const std::optional<double> maximumGroupDensity =
            openbus::map::mapAiMaximumRoadTrafficDensity(definition.aiTraffic, roadVehicles);
        openbus::map::MapAiTrafficData invalidSpecificTraffic = definition.aiTraffic;
        for (openbus::map::MapAiTrafficDensityGroup& group :
             invalidSpecificTraffic.unscheduledTrafficDensityGroups) {
            if (group.name == "NormalCars") {
                group.factor = 2.0;
                group.schedules = {{0, {{0.0, 0.3}, {12.0, 0.3}}},
                                   {1, {{0.0, 0.8}, {0.0, 0.9}}}};
            } else if (group.name == "Trucks") {
                group.factor = 1.0;
                group.schedules = {{0, {{0.0, 0.4}, {12.0, 0.4}}}};
            }
        }
        const auto invalidSpecificFallbackDensity =
            openbus::map::mapAiRoadTrafficDensityAtHour(
                invalidSpecificTraffic, roadVehicles, 12.0, 1);
        require(weekdayGroupDensity.has_value() &&
                    std::abs(*weekdayGroupDensity - 0.6) < 1.0e-9 &&
                    weekendGroupDensity.has_value() &&
                    std::abs(*weekendGroupDensity - (0.4 / 3.0)) < 1.0e-9 &&
                    saturdayGroupDensity.has_value() &&
                    std::abs(*saturdayGroupDensity - (0.8 / 3.0)) < 1.0e-9 &&
                    maximumGroupDensity.has_value() &&
                    std::abs(*maximumGroupDensity - 0.6) < 1.0e-9 &&
                    invalidSpecificFallbackDensity.has_value() &&
                    std::abs(*invalidSpecificFallbackDensity - (1.0 / 3.0)) < 1.0e-9,
                "group density curves select weekday/weekend masks and average road groups");
        openbus::map::MapAiTrafficData splitPeakTraffic = definition.aiTraffic;
        for (openbus::map::MapAiTrafficDensityGroup& group :
             splitPeakTraffic.unscheduledTrafficDensityGroups) {
            group.factor = 1.0;
            if (group.name == "NormalCars") {
                group.schedules = {{0, {{0.0, 1.0}, {12.0, 0.0}}}};
            } else if (group.name == "Trucks") {
                group.schedules = {{0, {{0.0, 0.0}, {12.0, 1.0}}}};
            }
        }
        const auto splitPeakPool = openbus::map::mapAiMaximumRoadTrafficPoolByGroup(
            splitPeakTraffic, roadVehicles, 10, 1.0);
        openbus::map::MapAiTrafficData laterPeakTraffic = definition.aiTraffic;
        for (openbus::map::MapAiTrafficDensityGroup& group :
             laterPeakTraffic.unscheduledTrafficDensityGroups) {
            group.factor = 1.0;
            group.schedules = {{0, {{0.0, 0.0}, {12.0, 1.0}}}};
        }
        const auto noDensityAtStart =
            openbus::map::mapAiRoadVehicles(laterPeakTraffic, 0.0, 1);
        const auto laterPeakPool = openbus::map::mapAiMaximumRoadTrafficPoolByGroup(
            laterPeakTraffic, roadVehicles, 10, 1.0);
        require(splitPeakPool.size() == 2, "split peak pool contains both groups");
        require(splitPeakPool[0].groupIndex == 0 && splitPeakPool[0].populationCount >= 5 &&
                    splitPeakPool[1].groupIndex == 1 && splitPeakPool[1].populationCount >= 5,
                "split peak pool reserves each group's maximum");
        require(!openbus::map::selectMapAiRoadVehicle(noDensityAtStart, 1).has_value(),
                "future-peak groups have no selectable vehicles at zero density");
        require(laterPeakPool.size() == 2 &&
                    laterPeakPool[0].populationCount + laterPeakPool[1].populationCount >= 10,
                "future group peaks are reserved at startup");
        for (const int dayMask : {1, 2, 4}) {
            for (int hourQuarter = 0; hourQuarter <= 96; ++hourQuarter) {
                const double hour = static_cast<double>(hourQuarter) * 0.25;
                const auto activeGroups =
                    openbus::map::mapAiRoadTrafficPopulationByGroup(
                        splitPeakTraffic,
                        openbus::map::mapTrafficPopulationForDensity(
                            10, openbus::map::mapAiRoadTrafficDensityAtHour(
                                    splitPeakTraffic, roadVehicles, hour, dayMask)),
                        hour, dayMask);
                for (const auto& activeGroup : activeGroups) {
                    const auto reserved = std::find_if(
                        splitPeakPool.begin(), splitPeakPool.end(),
                        [&](const auto& poolGroup) {
                            return poolGroup.groupIndex == activeGroup.groupIndex;
                        });
                    require(reserved != splitPeakPool.end() &&
                                activeGroup.populationCount <= reserved->populationCount,
                            "per-group reservations cover sampled active targets");
                }
            }
        }
        const auto uncappedMorningTargets =
            openbus::map::mapAiRoadTrafficPopulationByGroup(
                splitPeakTraffic,
                openbus::map::mapTrafficPopulationForDensity(
                    10, openbus::map::mapAiRoadTrafficDensityAtHour(
                            splitPeakTraffic, roadVehicles, 0.0, 1)),
                0.0, 1);
        const auto morningPeakReservation =
            uncappedMorningTargets.empty()
                ? splitPeakPool.end()
                : std::find_if(splitPeakPool.begin(),
                               splitPeakPool.end(),
                               [&](const auto& group) {
                                   return group.groupIndex ==
                                          uncappedMorningTargets[0].groupIndex;
                               });
        require(uncappedMorningTargets.size() == 1 &&
                    uncappedMorningTargets[0].populationCount == 5 &&
                    morningPeakReservation != splitPeakPool.end() &&
                    morningPeakReservation->populationCount >= 5 &&
                    splitPeakPool[0].populationCount +
                            splitPeakPool[1].populationCount > 6,
                "group reservations can exceed any one simultaneous target");
        openbus::map::MapAiTrafficData roundingPeakTraffic = definition.aiTraffic;
        for (openbus::map::MapAiTrafficDensityGroup& group :
             roundingPeakTraffic.unscheduledTrafficDensityGroups) {
            group.factor = 0.0;
            group.schedules = {{0, {{0.0, 0.0}, {12.0, 0.0}}}};
            if (group.name == "NormalCars") {
                group.factor = 1.0;
                group.schedules = {{0, {{0.0, 0.4}, {12.0, 0.8}}}};
            } else if (group.name == "Trucks") {
                group.factor = 1.0;
                group.schedules = {{0, {{0.0, 0.05}, {12.0, 1.25}}}};
            }
        }
        const auto roundingPeakPool =
            openbus::map::mapAiMaximumRoadTrafficPoolByGroup(
                roundingPeakTraffic, roadVehicles, 10, 1.0);
        const auto roundingPeakTargets =
            openbus::map::mapAiRoadTrafficPopulationByGroup(
                roundingPeakTraffic,
                openbus::map::mapTrafficPopulationForDensity(
                    10, openbus::map::mapAiRoadTrafficDensityAtHour(
                            roundingPeakTraffic, roadVehicles, 3.375, 1)),
                3.375, 1);
        const auto roundingPeakTargetsAtStart =
            openbus::map::mapAiRoadTrafficPopulationByGroup(
                roundingPeakTraffic,
                openbus::map::mapTrafficPopulationForDensity(
                    10, openbus::map::mapAiRoadTrafficDensityAtHour(
                            roundingPeakTraffic, roadVehicles, 0.0, 1)),
                0.0, 1);
        const auto roundingPeakTargetsAtEnd =
            openbus::map::mapAiRoadTrafficPopulationByGroup(
                roundingPeakTraffic,
                openbus::map::mapTrafficPopulationForDensity(
                    10, openbus::map::mapAiRoadTrafficDensityAtHour(
                            roundingPeakTraffic, roadVehicles, 12.0, 1)),
                12.0, 1);
        const auto normalCarRoundingReservation =
            std::find_if(roundingPeakPool.begin(), roundingPeakPool.end(),
                         [](const auto& group) { return group.groupIndex == 1; });
        require(roundingPeakTargets.size() == 2 &&
                    roundingPeakTargets[1].groupIndex == 1 &&
                    roundingPeakTargets[1].populationCount == 3 &&
                    roundingPeakTargetsAtStart[1].populationCount == 2 &&
                    roundingPeakTargetsAtEnd[1].populationCount == 4 &&
                    normalCarRoundingReservation != roundingPeakPool.end() &&
                    normalCarRoundingReservation->populationCount >=
                        roundingPeakTargets[1].populationCount,
                "group startup reservations cover rounded targets between curve knots");
        openbus::map::MapAiTrafficData uncappedRoundingTraffic = definition.aiTraffic;
        for (auto& group : uncappedRoundingTraffic.unscheduledTrafficDensityGroups) {
            group.factor = 1.0;
            if (group.name == "NormalCars") {
                group.schedules = {{0, {{0.0, 0.6005}, {12.0, 0.6005}}}};
            } else if (group.name == "Trucks") {
                group.schedules = {{0, {{0.0, 0.0002}, {12.0, 0.0038}}}};
            }
        }
        const auto uncappedPool = openbus::map::mapAiMaximumRoadTrafficPoolByGroup(
            uncappedRoundingTraffic, roadVehicles, 1000, 1.0);
        const auto uncappedTargets = openbus::map::mapAiRoadTrafficPopulationByGroup(
            uncappedRoundingTraffic,
            openbus::map::mapTrafficPopulationForDensity(
                1000, openbus::map::mapAiRoadTrafficDensityAtHour(
                          uncappedRoundingTraffic, roadVehicles, 1.1, 1)),
            1.1, 1);
        const auto normalPool = std::find_if(
            uncappedPool.begin(), uncappedPool.end(),
            [](const auto& group) { return group.groupIndex == 1; });
        const auto normalTarget = std::find_if(
            uncappedTargets.begin(), uncappedTargets.end(),
            [](const auto& group) { return group.groupIndex == 1; });
        for (const double hour : {0.0, 12.0}) {
            const auto knotTargets = openbus::map::mapAiRoadTrafficPopulationByGroup(
                uncappedRoundingTraffic,
                openbus::map::mapTrafficPopulationForDensity(
                    1000, openbus::map::mapAiRoadTrafficDensityAtHour(
                              uncappedRoundingTraffic, roadVehicles, hour, 1)),
                hour, 1);
            const auto normalKnot = std::find_if(
                knotTargets.begin(), knotTargets.end(),
                [](const auto& group) { return group.groupIndex == 1; });
            require(normalKnot != knotTargets.end() && normalKnot->populationCount == 300,
                    "uncapped rounding fixture has lower group counts at both knots");
        }
        require(normalTarget != uncappedTargets.end() && normalTarget->populationCount == 301 &&
                    normalPool != uncappedPool.end() &&
                    normalPool->populationCount >= normalTarget->populationCount,
                "uncapped group pool covers an interior rounding peak above 200");
        double normalCarSelectionWeight = 0.0;
        double truckSelectionWeight = 0.0;
        for (const openbus::map::MapAiRoadVehicle& vehicle : weekdayRoadVehicles) {
            if (vehicle.groupName == "normalcars") {
                normalCarSelectionWeight += vehicle.selectionWeight;
            } else if (vehicle.groupName == "trucks") {
                truckSelectionWeight += vehicle.selectionWeight;
            } else {
                require(vehicle.selectionWeight == 0.0,
                        "groups without a matching density schedule are not selected");
            }
        }
        require(std::abs(normalCarSelectionWeight - 1.6) < 1.0e-9 &&
                    std::abs(truckSelectionWeight - 0.2) < 1.0e-9,
                "schedule factors weight group selection independent of group size");
        const std::vector<openbus::map::MapAiRoadVehicle> poolRoadVehicles =
            openbus::map::mapAiRoadVehiclesForPool(definition.aiTraffic);
        double poolNormalCarWeight = 0.0;
        double poolTruckWeight = 0.0;
        for (const openbus::map::MapAiRoadVehicle& vehicle : poolRoadVehicles) {
            if (vehicle.groupName == "normalcars") {
                poolNormalCarWeight += vehicle.selectionWeight;
            } else if (vehicle.groupName == "trucks") {
                poolTruckWeight += vehicle.selectionWeight;
            } else {
                require(vehicle.selectionWeight == 0.0,
                        "pool composition excludes groups without density curves");
            }
        }
        const auto weekdayGroupPopulation =
            openbus::map::mapAiRoadTrafficPopulationByGroup(
                definition.aiTraffic, 9, 12.0, 1);
        const auto weekendGroupPopulation =
            openbus::map::mapAiRoadTrafficPopulationByGroup(
                definition.aiTraffic, 10, 12.0, 4);
        require(std::abs(poolNormalCarWeight - 1.6) < 1.0e-9 &&
                    std::abs(poolTruckWeight - 0.4) < 1.0e-9 &&
                    weekdayGroupPopulation.size() == 2 &&
                    weekdayGroupPopulation[0].groupIndex == 0 &&
                    weekdayGroupPopulation[0].populationCount == 1 &&
                    weekdayGroupPopulation[1].groupIndex == 1 &&
                    weekdayGroupPopulation[1].populationCount == 8 &&
                    weekendGroupPopulation.size() == 2 &&
                    weekendGroupPopulation[0].groupIndex == 0 &&
                    weekendGroupPopulation[0].populationCount == 5 &&
                    weekendGroupPopulation[1].groupIndex == 1 &&
                    weekendGroupPopulation[1].populationCount == 5,
                "the pool covers future groups and scheduled population targets rebalance");
        std::size_t selectedNormalCars = 0;
        std::size_t selectedTrucks = 0;
        for (std::size_t seed = 0; seed < 500; ++seed) {
            const auto selected =
                openbus::map::selectMapAiRoadVehicle(weekdayRoadVehicles, seed);
            require(selected.has_value(),
                    "positive scheduled group density selects a road vehicle");
            const std::string& selectedGroup = weekdayRoadVehicles[*selected].groupName;
            selectedNormalCars += selectedGroup == "normalcars" ? 1U : 0U;
            selectedTrucks += selectedGroup == "trucks" ? 1U : 0U;
            require(selectedGroup == "normalcars" || selectedGroup == "trucks",
                    "weighted selection excludes zero-density groups");
        }
        require(selectedNormalCars > selectedTrucks && selectedTrucks > 0,
                "weighted startup selection respects each group's scheduled density");
        writeUtf16Le(root / "unsched_trafficdens.txt", "");
        const openbus::map::MapDefinition emptyDensityDefinition =
            openbus::map::loadMapDefinition(root);
        const std::vector<openbus::map::MapAiRoadVehicle> emptyDensityRoadVehicles =
            openbus::map::mapAiRoadVehicles(emptyDensityDefinition.aiTraffic);
        require(!emptyDensityDefinition.aiTraffic.hasUnscheduledTrafficDensityFile &&
                    !openbus::map::mapAiRoadTrafficDensityAtHour(emptyDensityDefinition.aiTraffic,
                                                                 emptyDensityRoadVehicles, 12.0, 1)
                         .has_value(),
                "an empty unscheduled-density file preserves the global-density fallback");
        require(definition.aiTraffic.timetableRoutes.size() == 3 &&
                    definition.aiTraffic.timetableRoutes[0].timetableKey == "fixturetrip" &&
                    definition.aiTraffic.timetableRoutes[0].tripName == "FixtureLine" &&
                    definition.aiTraffic.timetableRoutes[0].destinationName == "Town End" &&
                    definition.aiTraffic.timetableRoutes[0].lineName == "12" &&
                    definition.aiTraffic.timetableRoutes[0].stationCount == 1 &&
                    definition.aiTraffic.timetableRoutes[0].stations ==
                        std::vector<openbus::map::MapAiTimetableRoute::Station>{
                            {15479, 1, "Main Stop", 0, {"0", "0", "3.5", "5"}, 3.5}} &&
                    definition.aiTraffic.timetableRoutes[0].splineIds ==
                        std::vector<int>{30273, 30274} &&
                    definition.aiTraffic.timetableRoutes[0].trackEntries ==
                        std::vector<openbus::map::MapAiTimetableRoute::TrackEntry>{
                            {30273, 0, 1, 10.0}, {30273, 1, 1, 10.0},
                            {30274, 0, 0, 10.0}} &&
                    definition.aiTraffic.timetableRoutes[0].scheduledDepartureMinutes ==
                        std::vector<double>{405.0, 495.0} &&
                    definition.aiTraffic.timetableRoutes[0]
                            .scheduledDepartureProfileIndices == std::vector<int>{0, 1} &&
                    definition.aiTraffic.timetableRoutes[0].profiles.size() == 2 &&
                    definition.aiTraffic.timetableRoutes[0].profiles[0].name == "Standard" &&
                    definition.aiTraffic.timetableRoutes[0]
                            .profiles[0]
                            .arrivalOffsetSeconds == std::vector<double>{15.0} &&
                    definition.aiTraffic.timetableRoutes[0]
                            .profiles[0]
                            .departureOffsetSeconds == std::vector<double>{30.0} &&
                    definition.aiTraffic.timetableRoutes[0]
                            .profiles[1]
                            .arrivalOffsetSeconds == std::vector<double>{90.0} &&
                    definition.aiTraffic.timetableRoutes[0]
                            .profiles[1]
                            .departureOffsetSeconds == std::vector<double>{120.0},
                "timetable routes retain stations, aligned profile selections, and stop times");
        const auto trackFixtureRoute = std::find_if(
            definition.aiTraffic.timetableRoutes.begin(),
            definition.aiTraffic.timetableRoutes.end(),
            [](const openbus::map::MapAiTimetableRoute& route) {
                return route.timetableKey == "trackfixture";
            });
        const auto trackFixtureLastStop =
            trackFixtureRoute == definition.aiTraffic.timetableRoutes.end()
                ? std::nullopt
                : openbus::map::mapAiTimetableStationPosition(
                      *trackFixtureRoute, trackFixtureRoute->stations.back());
        require(trackFixtureRoute != definition.aiTraffic.timetableRoutes.end() &&
                    trackFixtureRoute->stations ==
                        std::vector<openbus::map::MapAiTimetableRoute::Station>{
                            {15479, 0, "Main Stop", 0, {}, std::nullopt},
                            {15480, 2, "Town End", 0, {}, 5.5}} &&
                    trackFixtureRoute->scheduledDepartureMinutes == std::vector<double>{495.0} &&
                    trackFixtureLastStop && trackFixtureLastStop->trackEntryIndex == 1 &&
                    trackFixtureLastStop->offsetMeters == 5.5,
                "driving routes with station-ID-only trips resolve linked passenger stops");
        require(definition.aiTraffic.timetableCourses.size() == 3 &&
                    definition.aiTraffic.timetableCourses[0].name == "BusOnly" &&
                    definition.aiTraffic.timetableCourses[0].trips ==
                        std::vector<openbus::map::MapAiTimetableCourseTrip>{
                            {"busonly", 0, 495.0}} &&
                    definition.aiTraffic.timetableCourses[1].name == "Fixture" &&
                    definition.aiTraffic.timetableCourses[1].trips ==
                        std::vector<openbus::map::MapAiTimetableCourseTrip>{
                            {"fixturetrip", 1, 495.0}, {"fixturetrip", 0, 405.0},
                            {"broken", 0, 515.0}} &&
                    definition.aiTraffic.timetableCourses[2].name == "TrackFixture" &&
                    definition.aiTraffic.timetableCourses[2].trips ==
                        std::vector<openbus::map::MapAiTimetableCourseTrip>{
                            {"trackfixture", 0, 495.0}},
                "timetable courses retain source order, trip keys, profiles, and departure times");
        const auto brokenTimetable = std::ranges::find(
            definition.aiTraffic.passengerTimetables, std::string("broken"),
            &openbus::map::MapAiPassengerTimetable::timetableKey);
        require(brokenTimetable != definition.aiTraffic.passengerTimetables.end() &&
                    brokenTimetable->stationIds == std::vector<int>{19001, 19002} &&
                    brokenTimetable->stationRecords.size() == 2 &&
                    brokenTimetable->stationRecords[0] &&
                    brokenTimetable->stationRecords[0]->name == "First Stop" &&
                    brokenTimetable->stationRecords[1] &&
                    brokenTimetable->stationRecords[1]->name == "Last Stop" &&
                    brokenTimetable->scheduledDepartureMinutes == std::vector<double>{515.0} &&
                    std::ranges::none_of(definition.aiTraffic.timetableRoutes, [](const auto& route) {
                        return route.timetableKey == "broken";
                    }) &&
                    std::ranges::any_of(definition.diagnostics.entries, [](const auto& diagnostic) {
                        return diagnostic.keyword == "track_entry" &&
                               diagnostic.message.find("Broken.ttr") != std::string::npos;
                    }),
                "broken driving tracks preserve scheduled authored stops and retain diagnostics");
        const openbus::map::MapAiTimetableCourse progressionCourse{
            "Progression",
            {{"unavailable", 0, 100.0}, {"fixturetrip", 0, 110.0}, {"otherline", 1, 120.0}}};
        const std::vector<std::string> availableProgressionRoutes{"fixturetrip", "otherline"};
        require(openbus::map::mapAiTimetableNextCourseTripIndex(
                    progressionCourse, 0, availableProgressionRoutes) == 1 &&
                    openbus::map::mapAiTimetableNextCourseTripIndex(
                        progressionCourse, 1, availableProgressionRoutes) == 2 &&
                    !openbus::map::mapAiTimetableNextCourseTripIndex(
                        progressionCourse, 2, availableProgressionRoutes) &&
                    !openbus::map::mapAiTimetableNextCourseTripIndex(
                        progressionCourse, 3, availableProgressionRoutes),
                "player course progression skips unavailable routes and preserves trip order");
        const auto preciseStationPosition = openbus::map::mapAiTimetableStationPosition(
            definition.aiTraffic.timetableRoutes[0],
            definition.aiTraffic.timetableRoutes[0].stations[0]);
        require(preciseStationPosition && preciseStationPosition->trackEntryIndex == 0 &&
                    preciseStationPosition->offsetMeters == 3.5,
                "timetable stop positions map to the offset on their preceding track entry");
        auto firstStationPosition = definition.aiTraffic.timetableRoutes[0].stations[0];
        firstStationPosition.trackEntryIndex = 0;
        firstStationPosition.positionOnPreviousTrackEntryMeters = 2.0;
        const auto firstTrackPosition = openbus::map::mapAiTimetableStationPosition(
            definition.aiTraffic.timetableRoutes[0], firstStationPosition);
        require(firstTrackPosition && firstTrackPosition->trackEntryIndex == 0 &&
                    firstTrackPosition->offsetMeters == 2.0,
                "the first timetable station can be positioned within the first track entry");
        auto invalidStationPosition = definition.aiTraffic.timetableRoutes[0].stations[0];
        invalidStationPosition.positionOnPreviousTrackEntryMeters = 10.5;
        require(!openbus::map::mapAiTimetableStationPosition(
                    definition.aiTraffic.timetableRoutes[0], invalidStationPosition),
                "timetable stop offsets outside their track entry are rejected");
        const auto busOnlyTimetable =
            std::find_if(definition.aiTraffic.passengerTimetables.begin(),
                         definition.aiTraffic.passengerTimetables.end(),
                         [](const openbus::map::MapAiPassengerTimetable& timetable) {
                             return timetable.timetableKey == "busonly";
                         });
        const auto busOnlyRoute =
            std::find_if(definition.aiTraffic.timetableRoutes.begin(),
                         definition.aiTraffic.timetableRoutes.end(),
                         [](const openbus::map::MapAiTimetableRoute& route) {
                             return route.timetableKey == "busonly";
                         });
        require(definition.aiTraffic.busStops.size() == 2 &&
                    definition.aiTraffic.busStops[0].name == "Main Stop" &&
        definition.aiTraffic.busStops[0].tileIndex == 0 &&
                    definition.aiTraffic.busStops[0].id == 15479 &&
                    definition.aiTraffic.stationLinks.size() == 1 &&
                    definition.aiTraffic.stationLinks[0].fromStopId == 15479 &&
                    definition.aiTraffic.stationLinks[0].toStopId == 15480 &&
                    definition.aiTraffic.stationLinks[0].fields[0] == "12.5" &&
                    definition.aiTraffic.stationLinks[0].fields[3] == "0" &&
                    definition.aiTraffic.stationLinks[0].entries ==
                        std::vector<openbus::map::MapAiStationLink::Entry>{
                            {1, 2, 3, 4.5, {"5", "6", "7"}},
                            {2, 3, 4, 5.5, {"6", "7", "8"}}} &&
                    busOnlyTimetable != definition.aiTraffic.passengerTimetables.end() &&
                    busOnlyTimetable->stationIds == std::vector<int>{15479, 15480} &&
                    busOnlyTimetable->destinationName == "Town End" &&
                    busOnlyTimetable->lineName == "12" &&
                    busOnlyTimetable->scheduledDepartureMinutes == std::vector<double>{495.0} &&
                    busOnlyRoute != definition.aiTraffic.timetableRoutes.end() &&
                    busOnlyRoute->lineName == "12" &&
                    busOnlyRoute->destinationName == "Town End" &&
                    busOnlyRoute->stations.size() == 2 &&
                    busOnlyRoute->stations[0].trackEntryIndex == 0 &&
                    busOnlyRoute->stations[1].trackEntryIndex == 2 &&
                    busOnlyRoute->scheduledDepartureProfileIndices == std::vector<int>{0} &&
                    busOnlyRoute->profiles.size() == 1 &&
                    busOnlyRoute->profiles[0].arrivalOffsetSeconds ==
                        std::vector<double>{60.0, 720.0} &&
                    busOnlyRoute->profiles[0].departureOffsetSeconds ==
                        std::vector<double>{75.0, 720.0} &&
                    busOnlyRoute->splineIds == std::vector<int>{1, 2} &&
                    openbus::map::mapAiPassengerTimetableHasResolvedLinks(definition.aiTraffic,
                                                                          *busOnlyTimetable),
                "station-only trips build scheduled route checkpoints from ordered station links");
        const auto stationOnlyFinalPosition = openbus::map::mapAiTimetableStationPosition(
            *busOnlyRoute, busOnlyRoute->stations[1]);
        require(stationOnlyFinalPosition && stationOnlyFinalPosition->trackEntryIndex == 1 &&
                    stationOnlyFinalPosition->offsetMeters == 5.5,
                "station-only final calls are positioned at the end of their incoming link");
        openbus::map::MapAiTrafficData ambiguousPassengerStops = definition.aiTraffic;
        ambiguousPassengerStops.busStops.push_back(ambiguousPassengerStops.busStops.front());
        require(
            !openbus::map::mapAiPassengerTimetableHasResolvedLinks(ambiguousPassengerStops,
                                                                   *busOnlyTimetable),
            "duplicate stop IDs fail closed rather than binding a passenger station arbitrarily");
        require(definition.aiTraffic.busTypes.size() == 1 &&
                    definition.aiTraffic.busTypes[0].type == 2 &&
                    definition.aiTraffic.busTypes[0].assetPath == "vehicles\\city_bus.bus" &&
                    definition.aiTraffic.busTypes[0].rawLines ==
                        std::vector<std::string>{"vehicles\\city_bus.bus", "1234\tBlue livery"} &&
                    definition.aiTraffic.busTypes[0].depotIndex == 0 &&
                    definition.aiTraffic.depots.size() == 1 &&
                    definition.aiTraffic.depots[0].name == "Central Depot" &&
                    definition.aiTraffic.depots[0].hofName == "Fixture HOF" &&
                    definition.aiTraffic.depots[0].busTypeDefinitionIndices ==
                        std::vector<std::size_t>{0} &&
                    definition.aiTraffic.driverModelPaths ==
                        std::vector<std::string>{"humans\\drivers\\driver_one.hum",
                                                 "humans\\custom map\\driver_two.hum"},
                "depot/HOF association, bus models, raw assignments, and drivers are parsed");
        require(definition.aiTraffic.humanModelPaths ==
                    std::vector<std::string>{"humans\\pedestrians\\walker_one.hum",
                                             "humans\\pedestrians\\walker_two.hum"},
                "humans.txt pedestrian models are parsed separately from drivers.txt");
        bool invalidWeekdayMaskWarning = false;
        bool unknownGlobalWarning = false;
        bool duplicateTileWarning = false;
        for (const auto& diagnostic : definition.diagnostics.entries) {
            invalidWeekdayMaskWarning |= diagnostic.keyword == "set_day_of_week" &&
                                         diagnostic.line > 0;
            unknownGlobalWarning |= diagnostic.keyword == "unknown_global" &&
                                    diagnostic.line > 0;
            duplicateTileWarning |= diagnostic.keyword == "map" && diagnostic.line > 0 &&
                                    diagnostic.message.find("duplicate tile record") !=
                                        std::string::npos;
        }
        require(invalidWeekdayMaskWarning && unknownGlobalWarning && duplicateTileWarning,
                "invalid weekday masks, unsupported sections, and duplicate tiles produce line-numbered warnings");
        require(definition.unsupportedSections.size() == 1 &&
                    definition.unsupportedSections[0].keyword == "unknown_global" &&
                    definition.unsupportedSections[0].sectionLine > 0 &&
                    definition.unsupportedSections[0].payloadLines ==
                        std::vector<std::string>{" opaque global payload ", ""},
                "unsupported global section payload is preserved verbatim");
        require(definition.groundTextures.size() == 2, "multiple groundtex records parsed");
        require(openbus::map::resolveMapGroundTextureSidecarIndex("1", 8) == 1 &&
                openbus::map::resolveMapGroundTextureSidecarIndex("7", 8) == 7 &&
                openbus::map::resolveMapGroundTextureSidecarIndex("8", 8) == 7 &&
                openbus::map::resolveMapGroundTextureSidecarIndex("08", 8) == 7 &&
                !openbus::map::resolveMapGroundTextureSidecarIndex("0", 8) &&
                !openbus::map::resolveMapGroundTextureSidecarIndex("9", 8) &&
                !openbus::map::resolveMapGroundTextureSidecarIndex("8", 1) &&
                !openbus::map::resolveMapGroundTextureSidecarIndex("x", 8),
            "ground-texture sidecar suffixes resolve direct indices and terminal layer ordinal");
        require(definition.groundTextures[0].texturePath == "Texture\\grass.bmp",
                "ground texture path preserved");
        require(definition.groundTextures[1].texturePath == "Texture\\asphalt.bmp" &&
                    definition.groundTextures[1].detailTexturePath == "Texture\\detail2.bmp" &&
                    definition.groundTextures[1].parameters ==
                        std::array<std::string, 3>{"raw-a", "raw-b", "raw-c"},
                "additional ground texture paths and opaque parameters preserved");
        const openbus::map::MapGroundTextureTiling grassTiling =
            openbus::map::mapGroundTextureTiling(definition.groundTextures[0]);
        const openbus::map::MapGroundTextureTiling fallbackTiling =
            openbus::map::mapGroundTextureTiling(definition.groundTextures[1]);
        require(grassTiling.textureRepeatsPerTile == 1.0 &&
                    grassTiling.detailTextureRepeatsPerTile == 60.0 &&
                    fallbackTiling.textureRepeatsPerTile == 1.0 &&
                    fallbackTiling.detailTextureRepeatsPerTile == 1.0,
                "groundtex repeat fields are parsed independently with safe defaults");
        openbus::map::GroundTextureDefinition oversizedTiling = definition.groundTextures[1];
        oversizedTiling.parameters[1] = "1e100";
        oversizedTiling.parameters[2] = "1e100";
        const openbus::map::MapGroundTextureTiling boundedTiling =
            openbus::map::mapGroundTextureTiling(oversizedTiling);
        require(boundedTiling.textureRepeatsPerTile == 1.0 &&
                    boundedTiling.detailTextureRepeatsPerTile == 1.0,
                "unrepresentable ground-texture repeat counts use safe defaults");
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
        const double equatorWidth = 40075016.69 / 65536.0;
        const double equatorUpperWidth =
            equatorWidth / std::cosh(2.0 * 3.14159265358979323846 / 65536.0);
        require(definition.coordinates.worldCoordinates &&
                    std::abs(definition.coordinates.tileSize - equatorWidth) < 1.0e-5,
                "world-coordinate maps derive a metric grid from their tile rows");
        require(definition.entryPoints[0].tileIndex == 0 &&
                    definition.entryPoints[0].name == "Depot",
                "entrypoint fields mapped");
        require(definition.entryPoints[0].placement.position[0] == 10.0 &&
                    definition.entryPoints[0].placement.position[1] == 0.0 &&
                    definition.entryPoints[0].placement.position[2] == 20.0,
                "entrypoint placement maps tile-local coordinates into map coordinates");
        require(definition.entryPoints[1].tileIndex == 1 &&
                    std::abs(definition.entryPoints[1].placement.position[0] -
                             (definition.coordinates.tileSize + 30.0)) < 1.0e-8 &&
                    std::abs(definition.entryPoints[1].placement.position[1] -
                             40.0 * definition.coordinates.tileSize / equatorUpperWidth) < 1.0e-8 &&
                    definition.entryPoints[1].placement.position[2] == 5.0 &&
                    std::abs(definition.entryPoints[1].placement.yawDegrees -
                             std::atan2(29.0 * definition.coordinates.tileSize / equatorUpperWidth,
                                        82.0) *
                                 180.0 / 3.14159265358979323846) < 1.0e-9,
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
        openbus::map::MapTileReference invalidVersionTile;
        invalidVersionTile.textPath = root / "tile_invalid_version.map";
        bool invalidVersionRejected = false;
        try {
            static_cast<void>(openbus::map::loadMapTile(invalidVersionTile));
        } catch (const std::runtime_error& error) {
            invalidVersionRejected =
                std::string(error.what()).find("Invalid [version]") != std::string::npos;
        }
        require(invalidVersionRejected,
                "spline-attachment parsing rejects tile versions with trailing characters");
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
                    tile.attachedObjects[0].labels == std::vector<std::string>{"Car park B"} &&
                    tile.attachedObjects[0].hasExplicitId &&
                    tile.attachedObjects[0].transformValid &&
                    tile.attachedObjects[0].variableParentId == 41 &&
                    tile.attachedObjects[0].splineTerrainAlign &&
                    tile.attachedObjects[0].rules.size() == 1 &&
                    tile.attachedObjects[0].optionalLines ==
                        std::vector<std::string>{"custom tail", ""},
                "modern attachObj IDs, counted labels, and trailing opaque fields parsed");
        require(tile.splines.size() == 2 && tile.splines[0].geometryValid &&
                    tile.splines[0].assetPath == "Splines\\road.sli" &&
                    tile.splines[0].localX == 10.0 && tile.splines[0].elevation == 20.0 &&
                    tile.splines[0].localY == 30.0 && tile.splines[0].length == 50.0,
                "fixed-width spline geometry parsed while raw fields remain available");
        require(tile.splines[1].geometryValid && tile.splines[1].elevated &&
                    tile.splines[1].assetPath == "Splines\\bridge.sli" &&
                    tile.splines[1].length == 70.0 && tile.splines[1].heightDelta == 3.5 &&
                    tile.splines[1].cantStart == 3.5 && tile.splines[1].cantEnd == -2.5 &&
                    tile.splines[1].splineTerrainAlign2 == "raw spline alignment" &&
                    tile.splines[1].rules.size() == 1 &&
                    tile.splines[1].rules[0].fields ==
                        std::array<std::string, 4>{"spline rule 1", "spline rule 2",
                                                   "spline rule 3", "spline rule 4"},
                "elevated spline reads serialized cant values directly as percentage points");
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
                    version5Spline.splines[0].cantStart == 3.0 &&
                    version5Spline.splines[0].cantEnd == 4.0,
                "version 5 spline reads cant directly as map-editor percentage points");
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
                    version7Splines.splines[1].cantEnd == 4.0 &&
                    version7Splines.splines[1].geometryValid,
                "version 7 links and direct percentage-point cant fields are decoded");
        const auto version11Spline = loadVersionedSpline(
            "11", "[spline]\n0\nSplines\\modern.sli\n111\n110\n112\n1\n2\n0.5\n45\n50\n-30\n0\n0\n1.5\n2.5\n18\n");
        require(version11Spline.splines.size() == 1 &&
                    version11Spline.splines[0].splineId == 111 &&
                    version11Spline.splines[0].previousSplineId == 110 &&
                    version11Spline.splines[0].nextSplineId == 112 &&
                    version11Spline.splines[0].cantStart == 1.5 &&
                    version11Spline.splines[0].cantEnd == 2.5 &&
                    version11Spline.splines[0].chainOffsetValid &&
                    version11Spline.splines[0].chainOffset == 18.0,
                "version 11 spline reads direct cant percentages and chain texture offset");
        const auto version14Spline = loadVersionedSpline(
            "14", "[spline]\n0\nSplines\\v14.sli\n141\n140\n142\n10\n20\n0.5\n45\n50\n-30\n0\n0\n-50\n50\n11.25\n22.5\n77\nmirror\n");
        require(version14Spline.splines.size() == 1 &&
                    version14Spline.splines[0].geometryValid &&
                    version14Spline.splines[0].cantStart == -50.0 &&
                    version14Spline.splines[0].cantEnd == 50.0 &&
                    version14Spline.splines[0].skewStart == 11.25 &&
                    version14Spline.splines[0].skewEnd == 22.5 &&
                    version14Spline.splines[0].mirrored &&
                    version14Spline.splines[0].chainOffsetValid &&
                    version14Spline.splines[0].chainOffset == 77.0,
                "version 14 reads raw cant as percentage points before skew and chain fields");

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
        openbus::probe::removeProbeDirectory(root);
        throw;
    }
    openbus::probe::removeProbeDirectory(root);
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
    std::size_t timetableDepartureCount = 0;
    for (const openbus::map::MapAiTimetableRoute& route : definition.aiTraffic.timetableRoutes) {
        timetableDepartureCount += route.scheduledDepartureMinutes.size();
    }
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
              << " scenery_placements=" << sceneryPlacementCount
              << " timetable_routes=" << definition.aiTraffic.timetableRoutes.size()
              << " timetable_departures=" << timetableDepartureCount << '\n';
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
