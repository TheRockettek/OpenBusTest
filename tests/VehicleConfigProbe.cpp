#include "config/VehicleConfigLoader.h"
#include "core/Variables.h"
#include "osc/OscConverter.h"
#include "physics/BusConfiguration.h"
#include "ProbeTempDirectory.h"

#include <algorithm>
#include <array>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <vector>

int main(int argc, char** argv) {
    if (argc >= 3 && std::string(argv[1]) == "--inspect-view-markers") {
        bool passed = true;
        for (int index = 2; index < argc; ++index) {
            const VehicleConfig parsed =
                loadVehicleConfig(argv[index], VehicleFileKind::Bus, false);
            for (const ConfigurationDiagnostic& diagnostic : parsed.diagnostics.entries) {
                if (diagnostic.severity == ConfigurationDiagnostic::Severity::Error &&
                    (diagnostic.keyword == "view_schedule" ||
                     diagnostic.keyword == "view_ticketselling" ||
                     diagnostic.keyword == "ticket_sale" ||
                     diagnostic.keyword == "ticket_sale_money_point_2" ||
                     diagnostic.keyword == "ticket_sale_change_point_2")) {
                    std::cerr << argv[index] << ':' << diagnostic.line << " ["
                              << diagnostic.keyword << "] " << diagnostic.message << '\n';
                    passed = false;
                }
            }
            std::cout << argv[index] << ": schedule=" << parsed.scheduleViews.size()
                      << " ticket=" << parsed.ticketSellingViews.size()
                      << " ticket_machines=" << parsed.passengerTicketSaleMachines.size()
                      << " stampers=" << parsed.passengerCabinStampers.size()
                      << " money_point=" << parsed.passengerTicketMoneyPoint.has_value()
                      << " change_point=" << parsed.passengerTicketChangePoint.has_value()
                      << " schedule_indices=";
            for (const VehicleViewEntry& view : parsed.scheduleViews) {
                std::cout << view.viewIndex << ',';
            }
            std::cout << " ticket_indices=";
            for (const VehicleViewEntry& view : parsed.ticketSellingViews) {
                std::cout << view.viewIndex << ',';
            }
            std::cout << '\n';
            passed &= !parsed.scheduleViews.empty() && !parsed.ticketSellingViews.empty();
        }
        return passed ? 0 : 1;
    }
    const std::filesystem::path root =
        openbus::probe::makeProbeDirectory("openbus_vehicle_config_probe");
    std::filesystem::create_directories(root);
    std::filesystem::create_directories(root / "model");
    std::filesystem::create_directories(root / "Script");
    const std::filesystem::path configPath = root / "bus.bus";
    const std::filesystem::path scriptPath = root / "Script" / "Engine.OSC";
    std::ofstream script(scriptPath);
    script << "{init}\n{end}\n";
    script.close();
    std::ofstream cabin(root / "model/cabin.cfg");
    cabin << "[entry]\n0\n"
             "[exit]\n4\n"
             "[drivpos]\n0.7\n4.8\n1.5\n0.4\n0\n"
             "[illumination_interior]\n1\n2\n3\n4\n"
             "[passpos]\n0.9\n3.9\n1.8\n0.3\n0\n"
             "[linkToPrevVeh]\n16\n"
             "[linkToNextVeh]\n0\n"
             "[noticketsale]\n"
             "[stamper]\n7\n0.57833\n3.90257\n1.757\n"
             "[ticket_sale]\n2\n0.14959\n6.12376\n1.615\n"
             "[ticket_sale_money_point_2]\n-0.126\n5.747\n1.49\n0.05\n0.08\nfahrertuer\n"
             "[ticket_sale_change_point_2]\n-0.216\n5.46375\n1.319\n0.03\n0.03\nfahrertuer\n";
    cabin.close();
    std::ofstream paths(root / "model/paths.cfg");
    paths << "[stepsoundpack]\n2\nstep_a.wav\nstep_b.wav\n"
             "0\n[pathpnt]\n0\n0\n0\n"
             "1\n[pathpnt]\n1\n0\n0\n"
             "[pathlink]\n0\n1\n"
             "[pathlink_oneway]\n1\n0\n"
             "[next_stepsound]\n0\n"
             "[next_roomheight]\n1.85\n";
    paths.close();
    std::ofstream numbers(root / "registrations.org");
    numbers << "BUS-001\nBUS-002\n";
    numbers.close();
    std::ofstream registrationList(root / "registration_list.org");
    registrationList << "BUS-002\nBUS-003\n";
    registrationList.close();
    std::ofstream lightsConstants(root / "lights_constfile.txt", std::ios::binary);
    lightsConstants.write("\xEF\xBB\xBF", 3);
    lightsConstants << "[const]\nlights_blinkertime_on\n0.6\n"
                       "[const]\nlights_blinkertime_off\n0.4\n";
    lightsConstants.close();
    std::ofstream config(configPath);
    config << "[friendlyname]\nMercedes-Benz\nCitaro E400\nNight blue\n"
              "[description]\nUrban service bus.\n\n Electric drivetrain. \n[end]\n"
              "[type]\n0\n"
              "[model]\nmodel.cfg\n"
              "[rollwiderstand]\n1250\n"
              "[script]\n1\nscript/engine.osc\n"
              "[boundingbox]\n2.4\n12.0\n3.2\n0.1\n-0.2\n1.4\n"
              "[sound]\nsound/player.cfg\n"
              "[sound_ai]\nsound/ai.cfg\n"
              "[ai_deltaheight]\n0.35\n"
              "[paths]\nmodel/paths.cfg\n"
              "[passengercabin]\nmodel/cabin.cfg\n"
              "[number]\nregistrations.org\n"
              "[registration_list]\nregistration_list.org\n"
              "[constfile]\n1\nlights_constfile.txt\n"
              "[registration_automatic]\nB-TEST \n"
              "[registration_free]\n"
              "[kmcounter_init]\n2026\n1234.5\n";
    config.close();

    const std::filesystem::path viewConfigPath = root / "views.bus";
    {
        std::ofstream viewConfig(viewConfigPath);
        viewConfig << "[model]\nmodel.cfg\n"
                      "[add_camera_pax]\n0\n0\n0\n0\n60\n0\n0\n"
                      "[add_camera_driver]\n0\n0\n0\n0\n60\n0\n0\n"
                      "[view_schedule]\n"
                      "[view_ticketselling]\n";
    }
    const VehicleConfig viewConfig = loadVehicleConfig(viewConfigPath, VehicleFileKind::Bus, false);
    if (viewConfig.diagnostics.hasErrors() || viewConfig.scheduleViews.size() != 1 ||
        viewConfig.ticketSellingViews.size() != 1 || viewConfig.scheduleViews[0].viewIndex != 0 ||
        viewConfig.ticketSellingViews[0].viewIndex != 0 ||
        std::any_of(viewConfig.diagnostics.entries.begin(), viewConfig.diagnostics.entries.end(),
                    [](const ConfigurationDiagnostic& diagnostic) {
                        return diagnostic.keyword == "view_schedule" ||
                               diagnostic.keyword == "view_ticketselling";
                    })) {
        std::cerr << "Configured schedule/ticket views did not retain their driver camera\n";
        return 1;
    }

    const VehicleConfig result = loadVehicleConfig(configPath, VehicleFileKind::Bus);
    VehicleConfig aiSoundFallback = result;
    aiSoundFallback.soundAiConfigPath.clear();
    const VehiclePlacement testPlacement{{1.0, 2.0, 3.0}, 0.0};
    const bool caseMismatchedScriptConverted =
        result.scripts.size() == 1 && std::filesystem::exists(generatedLuaPath(scriptPath));
    VehicleConfig secondRegistration = result;
    selectRegistrationAtIndex(secondRegistration, 1);
    VehicleConfig manualRegistration = result;
    manualRegistration.registrationAutomatic = false;
    manualRegistration.registrationFree = false;
    const VehicleRegistrationSelection manualSelection =
        vehicleRegistrationSelectionAtIndex(manualRegistration, 1, "typed-number", "typed-plate");
    VehicleConfig automaticRegistration = manualRegistration;
    automaticRegistration.registrationAutomatic = true;
    const VehicleRegistrationSelection automaticSelection =
        vehicleRegistrationSelectionAtIndex(automaticRegistration, 1, "typed-number",
                                            "typed-plate");
    const VehicleRegistrationSelection freeSelection =
        vehicleRegistrationSelectionAtIndex(result, 1, "CUSTOM-7", "CUSTOM-PLATE");
    const std::vector<VehicleCamera> cameraLayout = {
        {VehicleCameraKind::Passenger},
        {VehicleCameraKind::Driver},
        {VehicleCameraKind::Reflexion},
        {VehicleCameraKind::Driver},
    };

    const auto writeResistanceConfig = [&](const std::filesystem::path& path,
                                           const std::string& directive) {
        std::ofstream output(path);
        output << "[model]\nmodel.cfg\n" << directive;
    };
    const std::vector<std::string> unsupportedKeywords = {
        "fixed",          "scriptshare",             "rowdy_factor",
        "boogies",        "ai_brakeperformance",    "ai_veh_type",
        "sinus",
        "couple_front_open_for_sound",               "control_cable_front",
        "control_cable_back",                        "new_attachment",
        "contact_shoe",   "rail_body_osc",
    };
    const std::filesystem::path unsupportedPath = root / "unsupported.bus";
    {
        std::ofstream output(unsupportedPath);
        output << "[model]\nmodel.cfg\n";
        for (const std::string& keyword : unsupportedKeywords) {
            output << '[' << keyword << "]\npayload\n";
        }
        output << "[unknown_probe_keyword]\npayload\n[rollwiderstand]\n1250\n";
    }
    for (VehicleFileKind kind : {VehicleFileKind::Bus, VehicleFileKind::Vehicle}) {
        const VehicleConfig unsupported = loadVehicleConfig(unsupportedPath, kind, false);
        for (const std::string& keyword : unsupportedKeywords) {
            const auto diagnostic = std::find_if(
                unsupported.diagnostics.entries.begin(), unsupported.diagnostics.entries.end(),
                [&](const ConfigurationDiagnostic& entry) {
                    return entry.keyword == keyword &&
                           entry.severity == ConfigurationDiagnostic::Severity::Warning &&
                           entry.message.find("recognized but unsupported:") == 0;
                });
            if (diagnostic == unsupported.diagnostics.entries.end() || diagnostic->line == 0) {
                std::cerr << "Missing unsupported vehicle keyword diagnostic: " << keyword << '\n';
                return 1;
            }
        }
        const auto unknown = std::find_if(
            unsupported.diagnostics.entries.begin(), unsupported.diagnostics.entries.end(),
            [](const ConfigurationDiagnostic& entry) {
                return entry.keyword == "unknown_probe_keyword" &&
                       entry.message == "unknown vehicle keyword";
            });
        if (unsupported.diagnostics.hasErrors() || unknown == unsupported.diagnostics.entries.end() ||
            unsupported.rollingResistanceForceNewtons != 1250.0) {
            std::cerr << "Unsupported vehicle records disrupted later supported directives\n";
            return 1;
        }
    }
    const std::filesystem::path couplingPointsPath = root / "coupling_points.bus";
    {
        std::ofstream output(couplingPointsPath);
        output << "[model]\nmodel.cfg\n"
                  "[coupling_front]\n0\n4.788\n0.501\n"
                  "[coupling_back]\n0\n-4.331\n0.315\n"
                  "[coupling_front_character]\n52.5\n-20\n20\n1\n"
                  "[couple_back]\ntrailer.bus\nfalse\n";
    }
    const VehicleConfig couplingPoints =
        loadVehicleConfig(couplingPointsPath, VehicleFileKind::Bus, false);
    if (couplingPoints.diagnostics.hasErrors() ||
        couplingPoints.frontCouplingPoint != std::array<double, 3>{0.0, 4.788, 0.501} ||
    couplingPoints.rearCouplingPoint != std::array<double, 3>{0.0, -4.331, 0.315} ||
    !couplingPoints.frontCouplingCharacter ||
    couplingPoints.frontCouplingCharacter->alphaDegrees != 52.5 ||
    couplingPoints.frontCouplingCharacter->minimumBetaDegrees != -20.0 ||
    couplingPoints.frontCouplingCharacter->maximumBetaDegrees != 20.0 ||
    couplingPoints.frontCouplingCharacter->type != 1 ||
    !couplingPoints.rearCouplingReference ||
    couplingPoints.rearCouplingReference->vehicleConfigPath != root / "trailer.bus" ||
    couplingPoints.rearCouplingReference->flag) {
        std::cerr << "Vehicle coupling point records were not retained: errors="
                  << couplingPoints.diagnostics.hasErrors() << " front="
                  << couplingPoints.frontCouplingPoint.has_value() << " rear="
                  << couplingPoints.rearCouplingPoint.has_value() << '\n';
        for (const ConfigurationDiagnostic& diagnostic : couplingPoints.diagnostics.entries) {
            std::cerr << diagnostic.keyword << ": " << diagnostic.message << '\n';
        }
        return 1;
    }
    const std::filesystem::path invalidCouplingReferencePath =
        root / "invalid_coupling_reference.bus";
    {
        std::ofstream output(invalidCouplingReferencePath);
        output << "[model]\nmodel.cfg\n[couple_back]\ntrailer.bus\nunknown\n";
    }
    const VehicleConfig invalidCouplingReference =
        loadVehicleConfig(invalidCouplingReferencePath, VehicleFileKind::Bus, false);
    if (!invalidCouplingReference.diagnostics.hasErrors() ||
        invalidCouplingReference.rearCouplingReference) {
        std::cerr << "Invalid vehicle coupling reference was accepted\n";
        return 1;
    }
    const std::filesystem::path invalidCouplingCharacterPath =
        root / "invalid_coupling_character.bus";
    {
        std::ofstream output(invalidCouplingCharacterPath);
        output << "[model]\nmodel.cfg\n"
                  "[coupling_front_character]\n52.5\n20\n-20\n2\n";
    }
    const VehicleConfig invalidCouplingCharacter =
        loadVehicleConfig(invalidCouplingCharacterPath, VehicleFileKind::Bus, false);
    if (!invalidCouplingCharacter.diagnostics.hasErrors() ||
        invalidCouplingCharacter.frontCouplingCharacter) {
        std::cerr << "Invalid vehicle coupling character was accepted\n";
        return 1;
    }
    const std::filesystem::path zeroResistancePath = root / "zero_roll.bus";
    writeResistanceConfig(zeroResistancePath, "[rollwiderstand]\n0\n");
    const VehicleConfig zeroResistance =
        loadVehicleConfig(zeroResistancePath, VehicleFileKind::Bus, false);
    const std::filesystem::path absentResistancePath = root / "absent_roll.bus";
    writeResistanceConfig(absentResistancePath, "");
    const VehicleConfig absentResistance =
        loadVehicleConfig(absentResistancePath, VehicleFileKind::Bus, false);
    const std::filesystem::path negativeResistancePath = root / "negative_roll.bus";
    writeResistanceConfig(negativeResistancePath, "[rollwiderstand]\n-1\n");
    const VehicleConfig negativeResistance =
        loadVehicleConfig(negativeResistancePath, VehicleFileKind::Bus, false);
    const std::filesystem::path reflectionCameraPath = root / "reflection_cameras.bus";
    writeResistanceConfig(reflectionCameraPath,
                          "[add_camera_reflexion]\n1\n2\n3\n0\n40\n10\n-5\n"
                          "[add_camera_reflexion_static]\n1\n2\n3\n0\n40\n10\n-5\n"
                          "[add_camera_reflexion_2]\n1\n2\n3\n0\n40\n10\n-5\n12\n"
                          "[add_camera_reflexion_2_static]\n1\n2\n3\n0\n40\n10\n-5\n12\n");
    const VehicleConfig reflectionCameras =
        loadVehicleConfig(reflectionCameraPath, VehicleFileKind::Bus, false);
    const std::filesystem::path inactiveRecordsPath = root / "inactive_records.bus";
    writeResistanceConfig(inactiveRecordsPath,
                          "[add_camera_driver]\n1\n2\n3\n0\n40\n10\n-5\n"
                          "[view_schedule]\n0\nSchedule view\n"
                          "[view_schedule]\n2\nSchedule route\n"
                          "[view_ticketselling]\n1\nTicket view\n"
                          "[rot_pnt_long]\n-2.5\n"
                          "[ai_deltaheight]\n0.4\n");
    const VehicleConfig inactiveRecords =
        loadVehicleConfig(inactiveRecordsPath, VehicleFileKind::Bus, false);
    const auto hasIgnoredWarning = [&](const std::string& keyword) {
        return std::any_of(inactiveRecords.diagnostics.entries.begin(),
                           inactiveRecords.diagnostics.entries.end(),
                           [&](const ConfigurationDiagnostic& diagnostic) {
                               return diagnostic.severity == ConfigurationDiagnostic::Severity::Warning &&
                                      diagnostic.keyword == keyword && diagnostic.line > 0 &&
                                      diagnostic.message.find("not ") != std::string::npos;
                           });
    };
    const bool inactiveRecordsReported =
        !inactiveRecords.diagnostics.hasErrors() &&
        inactiveRecords.longitudinalRotationPoint == -2.5 &&
        inactiveRecords.hasAiDeltaHeight && inactiveRecords.aiDeltaHeight == 0.4 &&
        inactiveRecords.scheduleViews.size() == 2 &&
        inactiveRecords.scheduleViews[0].viewIndex == 0 &&
        inactiveRecords.scheduleViews[0].label == "Schedule view" &&
        inactiveRecords.scheduleViews[1].viewIndex == 2 &&
        inactiveRecords.scheduleViews[1].label == "Schedule route" &&
        inactiveRecords.ticketSellingViews.size() == 1 &&
        inactiveRecords.ticketSellingViews[0].viewIndex == 1 &&
        inactiveRecords.ticketSellingViews[0].label == "Ticket view" &&
        !hasIgnoredWarning("view_schedule") && !hasIgnoredWarning("view_ticketselling") &&
        hasIgnoredWarning("rot_pnt_long");
    const std::filesystem::path invalidViewPath = root / "invalid_view.bus";
    writeResistanceConfig(invalidViewPath, "[view_schedule]\n-1\nInvalid view\n");
    const VehicleConfig invalidView =
        loadVehicleConfig(invalidViewPath, VehicleFileKind::Bus, false);
    const bool invalidViewRejected =
        invalidView.diagnostics.hasErrors() && invalidView.scheduleViews.empty();
    const std::filesystem::path markerPath = root / "view_markers.bus";
    writeResistanceConfig(markerPath,
                          "[add_camera_driver]\n1\n2\n3\n0\n40\n10\n-5\n"
                          "[view_schedule]\n\n8: next camera\n"
                          "[add_camera_pax]\n1\n2\n3\n0\n40\n10\n-5\n"
                          "[add_camera_driver]\n4\n5\n6\n0\n40\n10\n-5\n"
                          "[view_ticketselling]\n");
    const VehicleConfig markers = loadVehicleConfig(markerPath, VehicleFileKind::Bus, false);
    const std::filesystem::path invalidMarkerPath = root / "invalid_view_marker.bus";
    writeResistanceConfig(invalidMarkerPath,
                          "[add_camera_pax]\n1\n2\n3\n0\n40\n10\n-5\n"
                          "[view_schedule]\n"
                          "[add_camera_driver]\n1\n2\n3\n0\n40\n10\n-5\n");
    const VehicleConfig invalidMarker =
        loadVehicleConfig(invalidMarkerPath, VehicleFileKind::Bus, false);
    const bool markersParsed =
        !markers.diagnostics.hasErrors() && markers.cameras.size() == 3 &&
        markers.scheduleViews.size() == 1 && markers.scheduleViews[0].viewIndex == 0 &&
        markers.scheduleViews[0].label.empty() && markers.ticketSellingViews.size() == 1 &&
        markers.ticketSellingViews[0].viewIndex == 1 &&
        markers.ticketSellingViews[0].label.empty() && invalidMarker.diagnostics.hasErrors() &&
        invalidMarker.scheduleViews.empty() && invalidMarker.cameras.size() == 2;
    const std::filesystem::path axleLabelsPath = root / "axle_labels.bus";
    std::ofstream axleLabels(axleLabelsPath);
    axleLabels << "[model]\nmodel.cfg\n"
                  "[newachse]\n"
                  "Vorderachse:\n"
                  "Koordinate:\n"
                  "achse_long\n1.5\n"
                  "Breite der Auflagefläche beider Reifen\n"
                  "achse_maxwidth\n2.4\n"
                  "achse_minwidth\n1.8\n"
                  "Raddurchmesser der Räder dieser Achse\n"
                  "achse_raddurchmesser\n1.0\n"
                  "Federkonstante der Federung PRO SEITE in kN/m\n"
                  "achse_feder\n300\n"
                  "Höchstbelastung in kN\n"
                  "achse_maxforce\n80\n"
                  "Dämpferkonstante in kNs/m\n"
                  "achse_daempfer\n20\n"
                  "Ist diese Achse angetrieben?\n"
                  "achse_antrieb\n1\n";
    axleLabels.close();
    const VehicleConfig axleLabelsConfig =
        loadVehicleConfig(axleLabelsPath, VehicleFileKind::Bus, false);
    const std::filesystem::path ovhConfigPath = root / "ai_car.ovh";
    std::ofstream ovhConfig(ovhConfigPath);
    ovhConfig << "[friendlyname]\nTest\nAI car\nDefault\n"
                 "[model]\nmodel/model.cfg\n";
    ovhConfig.close();
    std::ofstream vehicleModelConfig(root / "model/model.cfg");
    vehicleModelConfig << "[mesh]\ncar.o3d\n";
    vehicleModelConfig.close();
    std::ofstream vehicleMesh(root / "model/car.o3d", std::ios::binary);
    vehicleMesh.close();
    const VehicleConfig ovhVehicle =
        loadVehicleConfig(ovhConfigPath, VehicleFileKind::Vehicle, false);
    const std::filesystem::path ovhModelConfigPath =
        modelConfigurationPathForVehicle(ovhConfigPath);
    openbus::scripting::Vehicle ovhModelVariables;
    const ModelConfig ovhModel = loadVehicleModelConfig(
        ovhModelConfigPath, root / "model", ovhModelVariables);
    const bool ovhModelLoaded =
        !ovhVehicle.diagnostics.hasErrors() && !ovhModel.diagnostics.hasErrors() &&
        ovhModelConfigPath == root / "model/model.cfg" &&
        ovhModel.parts.size() == 1 && ovhModel.parts[0].objPath == root / "model/car.o3d";
    const auto& reflections = reflectionCameras.cameras;
    const bool reflectionCameraKeywordsParsed =
        !reflectionCameras.diagnostics.hasErrors() && reflections.size() == 4 &&
        reflections[0].kind == VehicleCameraKind::Reflexion && reflections[0].followsViewer &&
        reflections[1].kind == VehicleCameraKind::Reflexion && !reflections[1].followsViewer &&
        reflections[2].kind == VehicleCameraKind::Reflexion2 && reflections[2].followsViewer &&
        reflections[3].kind == VehicleCameraKind::Reflexion2 && !reflections[3].followsViewer &&
        reflections[3].pan == 10.0 && reflections[3].tilt == -5.0;
    openbus::probe::removeProbeDirectory(root);

    if (!reflectionCameraKeywordsParsed || !ovhModelLoaded || !inactiveRecordsReported ||
        !invalidViewRejected || !markersParsed) {
        std::cerr << "Reflection camera parsing or OVH vehicle model loading failed\n";
        return 1;
    }
    if (axleLabelsConfig.diagnostics.hasErrors() || axleLabelsConfig.axles.size() != 1 ||
        axleLabelsConfig.axles.front().position != 1.5 ||
        axleLabelsConfig.axles.front().maxWidth != 2.4 ||
        axleLabelsConfig.axles.front().driven != true) {
        std::cerr << "Descriptive axle labels interfered with numeric axle records\n";
        return 1;
    }
    if (result.diagnostics.hasErrors() || !caseMismatchedScriptConverted ||
        result.friendlyManufacturer != "Mercedes-Benz" ||
        result.friendlyVehicleName != "Citaro E400" ||
        result.friendlyDefaultPaint != "Night blue" ||
        !result.rollingResistanceForceNewtons.has_value() ||
        *result.rollingResistanceForceNewtons != 1250.0 ||
        !zeroResistance.rollingResistanceForceNewtons.has_value() ||
        *zeroResistance.rollingResistanceForceNewtons != 0.0 ||
        absentResistance.rollingResistanceForceNewtons.has_value() ||
        !negativeResistance.diagnostics.hasErrors() ||
        result.description != "Urban service bus.\n\n Electric drivetrain. " ||
        !result.vehicleType.has_value() || *result.vehicleType != 0 ||
        !result.hasBoundingBox ||
        result.boundingBox != std::array<double, 6>{2.4, 12.0, 3.2, 0.1, -0.2, 1.4} ||
        result.soundConfigPath.generic_string() != (root / "sound/player.cfg").generic_string() ||
        result.soundAiConfigPath.generic_string() != (root / "sound/ai.cfg").generic_string() ||
        soundConfigPathForVehicle(result, false) != result.soundConfigPath ||
        soundConfigPathForVehicle(result, true) != result.soundAiConfigPath ||
        soundConfigPathForVehicle(aiSoundFallback, true) != aiSoundFallback.soundConfigPath ||
        !result.hasAiDeltaHeight || result.aiDeltaHeight != 0.35 ||
        placementForVehicle(result, testPlacement, false).position[2] != 3.0 ||
        placementForVehicle(result, testPlacement, true).position[2] != 3.35 ||
        result.pathsConfigPath.generic_string() != (root / "model/paths.cfg").generic_string() ||
        result.passengerCabinConfigPath.generic_string() != (root / "model/cabin.cfg").generic_string() ||
        result.numberConfigPath.generic_string() != (root / "registrations.org").generic_string() ||
        result.registrationListConfigPath.generic_string() !=
            (root / "registration_list.org").generic_string() ||
        !result.registrationAutomatic || result.registrationPrefix != "B-TEST" ||
        !result.registrationFree || !result.hasOdometerInitial ||
        result.odometerInitialYear != 2026 || result.odometerInitialKilometres != 1234.5 ||
        !result.passengerCabinLoaded || !result.driverPosition.has_value() ||
        result.passengerPositions.size() != 1 || result.passengerEntryPathPoints.size() != 1 ||
        result.passengerExitPathPoints.size() != 1 ||
        !result.passengerCabinNoTicketSale || result.passengerTicketSaleMachines.size() != 1 ||
        result.passengerCabinStampers.size() != 1 ||
        result.passengerCabinStampers[0].index != 7 ||
        result.passengerCabinStampers[0].position !=
            std::array<double, 3>{0.57833, 3.90257, 1.757} ||
        result.passengerPathPointToPreviousVehicle != 16 ||
        result.passengerPathPointToNextVehicle != 0 ||
        result.passengerTicketSaleMachines[0].index != 2 ||
        result.passengerTicketSaleMachines[0].position !=
            std::array<double, 3>{0.14959, 6.12376, 1.615} ||
        !result.passengerTicketMoneyPoint.has_value() ||
        result.passengerTicketMoneyPoint->position !=
            std::array<double, 3>{-0.126, 5.747, 1.49} ||
        result.passengerTicketMoneyPoint->size != std::array<double, 2>{0.05, 0.08} ||
        result.passengerTicketMoneyPoint->doorName != "fahrertuer" ||
        !result.passengerTicketChangePoint.has_value() ||
        result.passengerTicketChangePoint->position !=
            std::array<double, 3>{-0.216, 5.46375, 1.319} ||
        result.passengerTicketChangePoint->size != std::array<double, 2>{0.03, 0.03} ||
        result.passengerTicketChangePoint->doorName != "fahrertuer" ||
        result.passengerPositions[0].interiorLightIndexes != std::array<int, 4>{1, 2, 3, 4} ||
        !result.passengerPathsLoaded || result.passengerPathPoints.size() != 2 ||
        result.passengerPathLinks.size() != 2 ||
        result.passengerPathLinks[0].from != 0 || result.passengerPathLinks[0].to != 1 ||
        result.passengerPathLinks[0].oneWay ||
        result.passengerPathLinks[1].from != 1 || result.passengerPathLinks[1].to != 0 ||
        !result.passengerPathLinks[1].oneWay ||
        result.passengerStepSoundPacks.size() != 1 ||
        result.passengerStepSoundPacks[0].size() != 2 ||
        result.passengerPathNextStepSound != 0 ||
        result.passengerPathNextRoomHeights.size() != 1 ||
        result.passengerPathNextRoomHeights[0] != 1.85 ||
        !result.registrationListsLoaded || result.vehicleNumbers.size() != 2 ||
        result.registrationNumbers.size() != 2 || result.selectedRegistrationIndex != 0 ||
        result.selectedVehicleNumber != "BUS-001" ||
        result.selectedRegistration != "BUS-002" ||
        secondRegistration.selectedRegistrationIndex != 1 ||
        secondRegistration.selectedVehicleNumber != "BUS-002" ||
        secondRegistration.selectedRegistration != "BUS-003" ||
        manualSelection.vehicleNumber != "BUS-002" ||
        manualSelection.registration != "BUS-003" ||
        automaticSelection.vehicleNumber != "BUS-002" ||
        automaticSelection.registration.has_value() ||
        freeSelection.vehicleNumber != "CUSTOM-7" ||
        freeSelection.registration != "CUSTOM-PLATE" ||
        vehicleCameraArrayIndexForDriverView(cameraLayout, 0) != 1 ||
        vehicleCameraArrayIndexForDriverView(cameraLayout, 1) != 3 ||
        vehicleCameraArrayIndexForDriverView(cameraLayout, 2).has_value() ||
        vehicleCameraArrayIndexForDriverView(cameraLayout, -1).has_value()) {
        std::cerr << "BUS metadata records were not retained\n";
        return 1;
    }
    const auto blinkOn = result.constants.find("lights_blinkertime_on");
    const auto blinkOff = result.constants.find("lights_blinkertime_off");
    if (blinkOn == result.constants.end() || blinkOn->second != 0.6F ||
        blinkOff == result.constants.end() || blinkOff->second != 0.4F) {
        std::cerr << "BOM-prefixed constant file did not retain its first constant\n";
        return 1;
    }
    const std::vector<PassengerPathPoint> cabinPoints = {
        {10, {0.0, 0.0, 0.0}}, {20, {1.0, 0.0, 0.0}}, {30, {2.0, 0.0, 0.0}}};
    const std::vector<PassengerPathLink> cabinLinks = {
        {10, 20, true}, {20, 30, false}};
    if (passengerPathRoute(cabinPoints, cabinLinks, 10, 30) !=
            std::vector<int>{10, 20, 30} ||
        !passengerPathRoute(cabinPoints, cabinLinks, 30, 10).empty() ||
        passengerPathRoute(cabinPoints, cabinLinks, 20, 20) != std::vector<int>{20}) {
        std::cerr << "Passenger cabin path routing did not preserve link direction\n";
        return 1;
    }
    VehicleConfig multiDoorCabin;
    multiDoorCabin.passengerPositions = {
        {{1.0, 0.0, 0.0}}, {{0.0, 0.0, 0.0}}};
    multiDoorCabin.passengerPathPoints = cabinPoints;
    multiDoorCabin.passengerPathLinks = cabinLinks;
    multiDoorCabin.passengerExitPathPoints = {10, 30};
    if (passengerExitDoorForSeat(multiDoorCabin, 0, 8) != 1 ||
        passengerExitDoorForSeat(multiDoorCabin, 1, 8) != 0 ||
        passengerExitDoorForSeat(multiDoorCabin, 0, 1) != 0 ||
        passengerExitDoorForSeat(multiDoorCabin, 0, 0)) {
        std::cerr << "Passenger exit selection ignored directed paths or supported door limits\n";
        return 1;
    }
    multiDoorCabin.passengerPathLinks.clear();
    multiDoorCabin.passengerPositions[0].position[0] = 1.8;
    if (passengerExitDoorForSeat(multiDoorCabin, 0, 8) != 1) {
        std::cerr << "Passenger exit selection did not fall back to the nearest door\n";
        return 1;
    }
    multiDoorCabin.passengerPathPoints.clear();
    if (passengerExitDoorForSeat(multiDoorCabin, 0, 8) != 0) {
        std::cerr << "Passenger exit selection did not use the first unresolved door\n";
        return 1;
    }
    const std::vector<std::array<double, 3>> travelPath = {
        {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {3.0, 0.0, 0.0}};
    const std::vector<std::optional<std::size_t>> travelPathLinks = {std::nullopt, 4};
    const auto travelProfile = passengerPathLinkProgressProfile(travelPath, travelPathLinks);
    if (travelProfile.size() != 2 || std::abs(travelProfile[0].first - (1.0 / 3.0)) > 1.0e-9 ||
        travelProfile[0].second || travelProfile[1].first != 1.0 ||
        travelProfile[1].second != 4 ||
        passengerPathLinkAtProgress(travelPath, travelPathLinks, 0.2) ||
        passengerPathLinkAtProgress(travelPath, travelPathLinks, 0.5) != 4 ||
        passengerPathLinkAtProgress(travelPath, travelPathLinks, 1.5) != 4 ||
        passengerPathLinkAtProgress(travelPath, {std::nullopt}, 0.5)) {
        std::cerr << "Passenger path progress did not identify the active cabin pathlink\n";
        return 1;
    }
    return 0;
}
