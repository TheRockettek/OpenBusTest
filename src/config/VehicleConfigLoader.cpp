#include "config/VehicleConfigLoader.h"

#include "core/AssetPath.h"
#include "config/ConfigurationParser.h"
#include "config/ModelConfigLoader.h"
#include "core/PerfTrace.h"
#include "core/Variables.h"
#include "osc/OscConverter.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cctype>
#include <filesystem>
#include <limits>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace {

constexpr std::size_t MAX_CONFIG_RECORDS = 1'000'000;

using openbus::config::Line;
using openbus::config::lower;
using openbus::config::parseDouble;
using openbus::config::parseFloat;
using openbus::config::parseInt;
using openbus::config::Reader;
using openbus::config::trim;

bool readValues(Reader& reader, const std::string& keyword, std::size_t count,
                std::vector<std::string>& values, ConfigurationDiagnostics& diagnostics) {
    return reader.readPayloads(count, values, diagnostics, keyword);
}

bool readDoubleRecord(Reader& reader, const std::string& keyword, std::size_t count,
                      std::vector<double>& values, ConfigurationDiagnostics& diagnostics,
                      std::size_t lineNumber = 0) {
    std::vector<std::string> raw;
    if (!readValues(reader, keyword, count, raw, diagnostics)) {
        return false;
    }
    values.clear();
    values.reserve(raw.size());
    for (const std::string& value : raw) {
        double parsed = 0.0;
        if (!parseDouble(value, parsed)) {
            diagnostics.error(lineNumber, keyword, "expected numeric values");
            return false;
        }
        values.push_back(parsed);
    }
    return true;
}

bool readOneString(Reader& reader, const std::string& keyword, std::string& value,
                   ConfigurationDiagnostics& diagnostics) {
    Line line;
    if (!reader.readPayload(line, diagnostics, keyword)) {
        return false;
    }
    value = trim(line.text);
    return true;
}

bool allowsBusOnly(const std::string& keyword, VehicleFileKind kind) {
    if (kind == VehicleFileKind::Bus) {
        return true;
    }
    return keyword != "add_camera_reflexion" && keyword != "add_camera_reflexion_2" &&
           keyword != "add_camera_reflexion_static" && keyword != "add_camera_reflexion_2_static" &&
           keyword != "view_schedule" && keyword != "view_ticketselling";
}

const std::unordered_set<std::string>& knownVehicleKeywords() {
    static const std::unordered_set<std::string> keywords = {
        "end",
        "description",
        "friendlyname",
        "type",
        "model",
        "sound",
        "sound_ai",
        "paths",
        "passengercabin",
        "number",
        "registration_list",
        "registration_automatic",
        "registration_free",
        "script",
        "varnamelist",
        "stringvarnamelist",
        "constfile",
        "add_camera_driver",
        "add_camera_pax",
        "add_camera_reflexion",
        "add_camera_reflexion_2",
        "add_camera_reflexion_static",
        "add_camera_reflexion_2_static",
        "set_camera_std",
        "set_camera_outside_center",
        "view_schedule",
        "view_ticketselling",
        "newachse",
        "mass",
        "momentofintertia",
        "boundingbox",
        "openbus_wheel_half_width",
        "openbus_articulated",
        "cog",
        "schwerpunkt",
        "kmcounter_init",
        "rollwiderstand",
        "rot_pnt_long",
        "inv_min_turnradius",
        "ai_deltaheight",
        "coupling_front",
        "coupling_back",
        "coupling_front_character",
        "couple_back",
        "fixed",
        "scriptshare",
        "rowdy_factor",
        "boogies",
        "ai_brakeperformance",
        "ai_veh_type",
        "sinus",
        "couple_front_open_for_sound",
        "control_cable_front",
        "control_cable_back",
        "new_attachment",
        "contact_shoe",
        "rail_body_osc",
    };
    return keywords;
}

const std::unordered_map<std::string, std::string>& unsupportedVehicleKeywords() {
    static const std::unordered_map<std::string, std::string> keywords = {
        {"fixed", "vehicle-level fixed behavior is not implemented"},
        {"scriptshare", "shared vehicle script state is not implemented"},
        {"rowdy_factor", "family-specific ride behavior is not implemented"},
        {"boogies", "bogie behavior is not implemented"},
        {"ai_brakeperformance", "family-specific AI braking behavior is not implemented"},
        {"ai_veh_type", "family-specific AI vehicle categories are not implemented"},
        {"sinus", "family-specific body oscillation is not implemented"},
        {"couple_front_open_for_sound", "vehicle coupling sound is not implemented"},
        {"control_cable_front", "coupled-vehicle control transfer is not implemented"},
        {"control_cable_back", "coupled-vehicle control transfer is not implemented"},
        {"new_attachment", "vehicle attachment behavior is not implemented"},
        {"contact_shoe", "rail and trolley contact behavior is not implemented"},
        {"rail_body_osc", "rail body oscillation is not implemented"},
    };
    return keywords;
}

std::string ignoredVehicleKeywordReason(const std::string& keyword) {
    const auto unsupported = unsupportedVehicleKeywords().find(keyword);
    if (unsupported != unsupportedVehicleKeywords().end()) {
        return unsupported->second;
    }
    if (keyword == "openbus_articulated") {
        return "articulated vehicle runtime behavior is not implemented";
    }
    if (keyword == "rot_pnt_long") {
        return "rotation point is retained but not applied to vehicle physics";
    }
    if (keyword == "coupling_front" || keyword == "coupling_back" ||
        keyword == "coupling_front_character" || keyword == "couple_back") {
        return "articulated vehicle behavior is not implemented";
    }
    return {};
}

const std::unordered_set<std::string>& passengerCabinKeywords() {
    static const std::unordered_set<std::string> keywords = {
        "illumination_interior",
        "passpos",
        "drivpos",
        "entry",
        "exit",
        "linktonextveh",
        "linktoprevveh",
        "noticketsale",
        "stamper",
        "ticket_sale",
        "ticket_sale_money_point_2",
        "ticket_sale_change_point_2",
        "end",
    };
    return keywords;
}

std::string ignoredPassengerCabinKeywordReason(const std::string& keyword) {
    if (keyword == "linktonextveh" || keyword == "linktoprevveh") {
        return "cross-vehicle cabin traversal is not implemented";
    }
    return {};
}

const std::unordered_set<std::string>& passengerPathKeywords() {
    static const std::unordered_set<std::string> keywords = {
        "pathpnt",          "pathlink",        "pathlink_oneway",
        "stepsoundpack",    "next_stepsound",  "next_roomheight",
    };
    return keywords;
}

std::string ignoredPassengerPathKeywordReason(const std::string& keyword) {
    if (keyword == "stepsoundpack" || keyword == "next_stepsound") {
        return "passenger step-sound playback is not implemented";
    }
    if (keyword == "next_roomheight") {
        return "passenger room-height transitions are not applied";
    }
    return {};
}

std::filesystem::path resolveReferencedPath(const std::filesystem::path& configPath,
                                            const std::string& referencedPath) {
    return openbus::assets::resolveReferencedPath(configPath, referencedPath);
}

void loadVariableFile(const std::filesystem::path& configPath, const std::string& referencedPath,
                      bool stringVariables, VehicleConfig& result) {
    const std::filesystem::path path = resolveReferencedPath(configPath, referencedPath);
    Reader reader(path);
    if (!reader.isOpen()) {
        result.diagnostics.error(0, stringVariables ? "stringvarnamelist" : "varnamelist",
                                 "unable to open " + path.string());
        return;
    }
    Line line;
    while (reader.next(line)) {
        const std::string name = lower(trim(line.text));
        if (name.empty() || name.front() == ';' || name.front() == '/') {
            continue;
        }
        (stringVariables ? result.stringVariables : result.floatVariables).push_back(name);
    }
}

void loadConstantFile(const std::filesystem::path& configPath, const std::string& referencedPath,
                      VehicleConfig& result) {
    const std::filesystem::path path = resolveReferencedPath(configPath, referencedPath);
    Reader reader(path);
    if (!reader.isOpen()) {
        result.diagnostics.error(0, "constfile", "unable to open " + path.string());
        return;
    }
    Line line;
    while (reader.next(line)) {
        if (!line.isKeyword()) {
            continue;
        }
        const std::string keyword = lower(line.keyword());
        if (keyword == "const") {
            Line nameLine;
            Line valueLine;
            float value = 0.0F;
            if (!reader.readPayload(nameLine, result.diagnostics, "const") ||
                !reader.readPayload(valueLine, result.diagnostics, "const") ||
                !parseFloat(valueLine.text, value)) {
                result.diagnostics.error(line.number, "const", "expected a name and numeric value");
                continue;
            }
            result.constants[lower(trim(nameLine.text))] = value;
            continue;
        }
        if (keyword == "newcurve") {
            Line nameLine;
            if (!reader.readPayload(nameLine, result.diagnostics, "newcurve")) {
                continue;
            }
            ConstantCurve curve;
            curve.name = lower(trim(nameLine.text));
            while (reader.next(line)) {
                if (!line.isKeyword()) {
                    continue;
                }
                if (lower(line.keyword()) != "pnt") {
                    reader.pushBack(std::move(line));
                    break;
                }
                std::vector<std::string> values;
                if (!reader.readPayloads(2, values, result.diagnostics, "pnt")) {
                    break;
                }
                ConstantCurvePoint point;
                if (!parseFloat(values[0], point.x) || !parseFloat(values[1], point.y)) {
                    result.diagnostics.error(line.number, "pnt", "expected numeric x and y values");
                    continue;
                }
                curve.points.push_back(point);
            }
            if (curve.name.empty() || curve.points.empty()) {
                result.diagnostics.error(nameLine.number, "newcurve",
                                         "curve requires a name and at least one point");
            } else {
                result.curves.try_emplace(curve.name, std::move(curve));
            }
            continue;
        }
        openbus::config::logUnhandledConfigurationTag(
            "constfile", path, line.number, line.keyword(), "unknown constant-file keyword");
    }
}

void loadReferencedDefinitions(const std::filesystem::path& configPath, VehicleConfig& result) {
    for (const std::string& path : result.variableLists) {
        loadVariableFile(configPath, path, false, result);
    }
    for (const std::string& path : result.stringVariableLists) {
        loadVariableFile(configPath, path, true, result);
    }
    for (const std::string& path : result.constantFiles) {
        loadConstantFile(configPath, path, result);
    }
}

void loadPassengerCabin(const VehicleConfig& source, VehicleConfig& result) {
    if (source.passengerCabinConfigPath.empty()) {
        return;
    }
    Reader reader(source.passengerCabinConfigPath);
    if (!reader.isOpen()) {
        result.diagnostics.error(0, "passengercabin",
                                 "unable to open " + source.passengerCabinConfigPath.string());
        return;
    }

    std::array<int, 4> pendingIllumination = {-1, -1, -1, -1};
    bool hasPendingIllumination = false;
    Line line;
    while (reader.next(line)) {
        if (!line.isKeyword()) {
            continue;
        }
        const std::string keyword = lower(line.keyword());
        if (passengerCabinKeywords().find(keyword) != passengerCabinKeywords().end()) {
            const std::string ignoredReason = ignoredPassengerCabinKeywordReason(keyword);
            openbus::config::logConfigurationTagStatus(
                "passengercabin.cfg", source.passengerCabinConfigPath, line.number,
                line.keyword(),
                ignoredReason.empty() ? openbus::config::ConfigurationTagStatus::Active
                                      : openbus::config::ConfigurationTagStatus::Ignored,
                ignoredReason.empty() ? "passenger-cabin record has runtime handling"
                                      : ignoredReason);
        }
        if (keyword == "illumination_interior") {
            std::vector<std::string> values;
            if (!readValues(reader, keyword, 4, values, result.diagnostics)) {
                continue;
            }
            std::array<int, 4> indexes = {-1, -1, -1, -1};
            bool valid = true;
            for (std::size_t index = 0; index < values.size(); ++index) {
                if (!parseInt(values[index], indexes[index]) || indexes[index] < -1) {
                    valid = false;
                    break;
                }
            }
            if (!valid) {
                result.diagnostics.error(line.number, keyword,
                                         "expected four light indexes of -1 or greater");
            } else {
                pendingIllumination = indexes;
                hasPendingIllumination = true;
            }
            continue;
        }
        if (keyword == "passpos" || keyword == "drivpos") {
            std::vector<double> values;
            if (!readDoubleRecord(reader, keyword, 5, values, result.diagnostics)) {
                continue;
            }
            PassengerCabinPosition position;
            position.position = {values[0], values[1], values[2]};
            position.seatHeight = values[3];
            position.rotationDegrees = values[4];
            position.driver = keyword == "drivpos";
            if (hasPendingIllumination) {
                position.interiorLightIndexes = pendingIllumination;
                hasPendingIllumination = false;
            }
            if (position.driver) {
                result.driverPosition = position;
            } else {
                result.passengerPositions.push_back(position);
            }
            result.passengerCabinLoaded = true;
            continue;
        }
        if (keyword == "entry" || keyword == "exit") {
            int pathPoint = -1;
            Line value;
            if (!reader.readPayload(value, result.diagnostics, keyword) ||
                !parseInt(value.text, pathPoint) || pathPoint < 0) {
                result.diagnostics.error(line.number, keyword,
                                         "expected a non-negative path-point index");
                continue;
            }
            (keyword == "entry" ? result.passengerEntryPathPoints : result.passengerExitPathPoints)
                .push_back(pathPoint);
            result.passengerCabinLoaded = true;
            continue;
        }
        if (keyword == "linktonextveh" || keyword == "linktoprevveh") {
            int pathPoint = -1;
            Line value;
            if (!reader.readPayload(value, result.diagnostics, keyword) ||
                !parseInt(value.text, pathPoint) || pathPoint < 0) {
                result.diagnostics.error(line.number, keyword,
                                         "expected a non-negative passenger path-point index");
                continue;
            }
            const bool linksToNextVehicle = keyword == "linktonextveh";
            if (linksToNextVehicle) {
                result.passengerPathPointToNextVehicle = pathPoint;
            } else {
                result.passengerPathPointToPreviousVehicle = pathPoint;
            }
            result.diagnostics.warning(line.number, keyword,
                                       "linked-cabin path-point index retained; traversal across "
                                       "coupled vehicles is not implemented");
            result.passengerCabinLoaded = true;
            continue;
        }
        if (keyword == "noticketsale") {
            result.passengerCabinNoTicketSale = true;
            result.passengerCabinLoaded = true;
            result.diagnostics.warning(
                line.number, keyword,
                "eligibility marker retained; ticket-sale behavior is not implemented");
            continue;
        }
        if (keyword == "stamper") {
            std::vector<std::string> values;
            int index = -1;
            PassengerCabinStamper stamper;
            if (!readValues(reader, keyword, 4, values, result.diagnostics)) {
                continue;
            }
            if (parseInt(values[0], index) && index >= 0 &&
                parseDouble(values[1], stamper.position[0]) &&
                parseDouble(values[2], stamper.position[1]) &&
                parseDouble(values[3], stamper.position[2])) {
                stamper.index = index;
                result.passengerCabinStampers.push_back(stamper);
                result.diagnostics.warning(
                    line.number, keyword,
                    "stamper location retained; eligible passenger tickets can walk to the first "
                    "configured stamper, but hand and ticket animations are not implemented");
            } else {
                result.diagnostics.error(line.number, keyword,
                                         "expected a non-negative stamper index and xyz position");
            }
            result.passengerCabinLoaded = true;
            continue;
        }
        if (keyword == "ticket_sale") {
            std::vector<std::string> values;
            int index = -1;
            PassengerTicketSaleMachine machine;
            if (!readValues(reader, keyword, 4, values, result.diagnostics)) {
                continue;
            }
            if (parseInt(values[0], index) && index >= 0 &&
                parseDouble(values[1], machine.position[0]) &&
                parseDouble(values[2], machine.position[1]) &&
                parseDouble(values[3], machine.position[2])) {
                machine.index = index;
                result.passengerTicketSaleMachines.push_back(machine);
                result.diagnostics.warning(
                    line.number, keyword,
                    "machine location retained; ticket-sale interaction is not implemented");
            } else {
                result.diagnostics.error(line.number, keyword,
                                         "expected a non-negative machine index and xyz position");
            }
            result.passengerCabinLoaded = true;
            continue;
        }
        if (keyword == "ticket_sale_money_point_2" || keyword == "ticket_sale_change_point_2") {
            std::vector<std::string> values;
            PassengerTicketSalePoint point;
            if (!readValues(reader, keyword, 6, values, result.diagnostics)) {
                continue;
            }
            bool valid = true;
            for (std::size_t index = 0; index < point.position.size(); ++index) {
                valid &= parseDouble(values[index], point.position[index]);
            }
            for (std::size_t index = 0; index < point.size.size(); ++index) {
                valid &= parseDouble(values[index + point.position.size()], point.size[index]) &&
                         point.size[index] > 0.0;
            }
            point.doorName = trim(values.back());
            if (valid) {
                if (keyword == "ticket_sale_money_point_2") {
                    result.passengerTicketMoneyPoint = std::move(point);
                } else {
                    result.passengerTicketChangePoint = std::move(point);
                }
                result.diagnostics.warning(
                    line.number, keyword,
                    "interaction-point location retained; ticket buyers visit configured money and "
                    "change points, but physical payment and coin output are not implemented");
            } else {
                result.diagnostics.error(
                    line.number, keyword,
                    "expected xyz, positive width/height, and optional door name");
            }
            result.passengerCabinLoaded = true;
            continue;
        }
        if (keyword == "end") {
            continue;
        }
        openbus::config::logUnhandledConfigurationTag(
            "passengercabin.cfg", source.passengerCabinConfigPath, line.number, line.keyword(),
            "passenger-cabin keyword is not implemented");
        openbus::config::logConfigurationTagStatus(
            "passengercabin.cfg", source.passengerCabinConfigPath, line.number, line.keyword(),
            openbus::config::ConfigurationTagStatus::Unknown,
            "passenger-cabin keyword is not recognized");
    }
}

void loadPassengerPaths(const VehicleConfig& source, VehicleConfig& result) {
    if (source.pathsConfigPath.empty()) {
        return;
    }
    Reader reader(source.pathsConfigPath);
    if (!reader.isOpen()) {
        result.diagnostics.error(0, "paths", "unable to open " + source.pathsConfigPath.string());
        return;
    }

    int pendingPointIndex = -1;
    Line line;
    while (reader.next(line)) {
        if (!line.isKeyword()) {
            int parsedIndex = -1;
            if (parseInt(line.text, parsedIndex)) {
                pendingPointIndex = parsedIndex;
            }
            continue;
        }
        const std::string keyword = lower(line.keyword());
        if (passengerPathKeywords().find(keyword) != passengerPathKeywords().end()) {
            const std::string ignoredReason = ignoredPassengerPathKeywordReason(keyword);
            openbus::config::logConfigurationTagStatus(
                "paths.cfg", source.pathsConfigPath, line.number, line.keyword(),
                ignoredReason.empty() ? openbus::config::ConfigurationTagStatus::Active
                                      : openbus::config::ConfigurationTagStatus::Ignored,
                ignoredReason.empty() ? "passenger-path record has runtime handling"
                                      : ignoredReason);
        }
        if (keyword == "pathpnt") {
            std::vector<double> values;
            if (pendingPointIndex < 0 ||
                !readDoubleRecord(reader, keyword, 3, values, result.diagnostics)) {
                result.diagnostics.error(line.number, keyword,
                                         "expected an index followed by three coordinates");
                pendingPointIndex = -1;
                continue;
            }
            result.passengerPathPoints.push_back(
                {pendingPointIndex, {values[0], values[1], values[2]}});
            result.passengerPathsLoaded = true;
            pendingPointIndex = -1;
            continue;
        }
        if (keyword == "pathlink" || keyword == "pathlink_oneway") {
            std::vector<std::string> values;
            if (!readValues(reader, keyword, 2, values, result.diagnostics)) {
                continue;
            }
            PassengerPathLink link;
            if (!parseInt(values[0], link.from) || !parseInt(values[1], link.to) || link.from < 0 ||
                link.to < 0) {
                result.diagnostics.error(line.number, keyword,
                                         "expected two non-negative path-point indexes");
                continue;
            }
            link.oneWay = keyword == "pathlink_oneway";
            result.passengerPathLinks.push_back(link);
            result.passengerPathsLoaded = true;
            continue;
        }
        if (keyword == "stepsoundpack") {
            Line countLine;
            int count = 0;
            if (!reader.readPayload(countLine, result.diagnostics, keyword) ||
                !parseInt(countLine.text, count) || count < 0) {
                result.diagnostics.error(line.number, keyword,
                                         "expected a non-negative sound count");
                continue;
            }
            std::vector<std::string> sounds;
            if (readValues(reader, keyword, static_cast<std::size_t>(count), sounds,
                           result.diagnostics)) {
                result.passengerStepSoundPacks.push_back(std::move(sounds));
            }
            continue;
        }
        if (keyword == "next_stepsound") {
            Line value;
            if (reader.readPayload(value, result.diagnostics, keyword) &&
                parseInt(value.text, result.passengerPathNextStepSound) &&
                result.passengerPathNextStepSound >= 0) {
                continue;
            }
            result.diagnostics.error(line.number, keyword,
                                     "expected a non-negative sound-pack index");
            continue;
        }
        if (keyword == "next_roomheight") {
            std::vector<double> values;
            if (readDoubleRecord(reader, keyword, 1, values, result.diagnostics)) {
                // The path format permits repeated room-height transitions.
                result.passengerPathNextRoomHeights.push_back(values[0]);
            }
            continue;
        }
        openbus::config::logUnhandledConfigurationTag("paths.cfg", source.pathsConfigPath,
                                                      line.number, line.keyword(),
                                                      "passenger-path keyword is not implemented");
        openbus::config::logConfigurationTagStatus(
            "paths.cfg", source.pathsConfigPath, line.number, line.keyword(),
            openbus::config::ConfigurationTagStatus::Unknown,
            "passenger-path keyword is not recognized");
    }
}

void loadRegistrationEntries(const std::filesystem::path& path, std::vector<std::string>& entries,
                             VehicleConfig& result) {
    if (path.empty()) {
        return;
    }
    std::ifstream input(path);
    if (!input) {
        result.diagnostics.error(0, "registration", "unable to open " + path.string());
        return;
    }
    std::string value;
    while (std::getline(input, value)) {
        value = trim(value);
        if (value.empty() || value.front() == ';' || value.front() == '/') {
            continue;
        }
        entries.push_back(std::move(value));
    }
    result.registrationListsLoaded = true;
}

void convertVehicleScripts(const std::filesystem::path& configPath, VehicleConfig& result) {
    for (const std::string& referencedPath : result.scripts) {
        const std::filesystem::path sourcePath = resolveReferencedPath(configPath, referencedPath);
        if (sourcePath.extension() != ".osc" && sourcePath.extension() != ".OSC") {
            continue;
        }

        std::filesystem::path generatedPath;
        std::error_code sourceError;
        const bool sourceExists = std::filesystem::exists(sourcePath, sourceError);
        if (!sourceExists || sourceError) {
            result.diagnostics.error(0, "script", "unable to open " + sourcePath.string());
            continue;
        }

        try {
            generatedPath = generatedLuaPath(sourcePath);
        } catch (const std::exception& exception) {
            result.diagnostics.error(
                0, "script", "unable to hash " + sourcePath.string() + ": " + exception.what());
            continue;
        }
        if (std::filesystem::exists(generatedPath)) {
            continue;
        }

        std::string error;
        if (!convertOscToLua(sourcePath, generatedPath, error)) {
            result.diagnostics.error(0, "script",
                                     "unable to convert " + sourcePath.string() + ": " + error);
        }
    }
}

bool parseCamera(Reader& reader, const std::string& keyword, VehicleCameraKind kind,
                 VehicleCamera& camera, ConfigurationDiagnostics& diagnostics) {
    const std::size_t count = kind == VehicleCameraKind::Reflexion2 ? 8 : 7;
    std::vector<double> values;
    if (!readDoubleRecord(reader, keyword, count, values, diagnostics)) {
        return false;
    }
    camera.kind = kind;
    camera.position = {values[0], values[1], values[2]};
    camera.orbitDistance = values[3];
    camera.fieldOfView = values[4];
    camera.pan = values[5];
    camera.tilt = values[6];
    return true;
}

bool parseAxle(Reader& reader, VehicleConfig& result, const Line& keywordLine) {
    double position = 0.0;
    double maxWidth = 0.0;
    double minWidth = 0.0;
    double wheelDiameter = 0.0;
    double spring = 0.0;
    double maxForce = 0.0;
    double damper = 0.0;
    double driven = 0.0;
    bool hasPosition = false;
    bool hasMaxWidth = false;
    bool hasMinWidth = false;
    bool hasWheelDiameter = false;
    bool hasSpring = false;
    bool hasMaxForce = false;
    bool hasDamper = false;
    bool hasDriven = false;

    Line field;
    while (reader.next(field)) {
        if (field.isKeyword()) {
            reader.pushBack(std::move(field));
            break;
        }
        const std::string name = lower(field.text);
        const bool recognized = name == "achse_long" || name == "achse_maxwidth" ||
                                name == "achse_minwidth" || name == "achse_raddurchmesser" ||
                                name == "achse_feder" || name == "achse_maxforce" ||
                                name == "achse_daempfer" || name == "achse_antrieb";
        if (!recognized) {
            if (openbus::config::isConfigurationIdentifier(name)) {
                openbus::config::logUnhandledConfigurationTag(
                    "vehicle axle block", result.sourcePath, field.number, field.text,
                    "axle subdirective is not implemented");
            }
            continue;
        }
        Line valueLine;
        if (!reader.readPayload(valueLine, result.diagnostics, name)) {
            return false;
        }
        double value = 0.0;
        if (!parseDouble(valueLine.text, value)) {
            result.diagnostics.error(valueLine.number, name, "expected a numeric value");
            continue;
        }
        if (name == "achse_long") {
            position = value;
            hasPosition = true;
        } else if (name == "achse_maxwidth") {
            maxWidth = value;
            hasMaxWidth = true;
        } else if (name == "achse_minwidth") {
            minWidth = value;
            hasMinWidth = true;
        } else if (name == "achse_raddurchmesser") {
            wheelDiameter = value;
            hasWheelDiameter = true;
        } else if (name == "achse_feder") {
            spring = value;
            hasSpring = true;
        } else if (name == "achse_maxforce") {
            maxForce = value;
            hasMaxForce = true;
        } else if (name == "achse_daempfer") {
            damper = value;
            hasDamper = true;
        } else if (name == "achse_antrieb") {
            driven = value;
            hasDriven = true;
        }
    }

    const bool validFiniteValues = std::isfinite(position) && std::isfinite(maxWidth) &&
                                   std::isfinite(minWidth) && std::isfinite(wheelDiameter) &&
                                   std::isfinite(spring) && std::isfinite(maxForce) &&
                                   std::isfinite(damper);
    if (!hasPosition || !hasMaxWidth || !hasMinWidth || !hasWheelDiameter || !hasSpring ||
        !hasMaxForce || !hasDamper || !hasDriven || !validFiniteValues || maxWidth <= 0.0 ||
        minWidth < 0.0 || minWidth > maxWidth || wheelDiameter <= 0.0 || spring <= 0.0 ||
        maxForce <= 0.0 || damper < 0.0) {
        result.diagnostics.error(
            keywordLine.number, "newachse",
            "axle requires finite values, positive width/wheel/spring/force, min width no "
            "greater than max width, and non-negative damping");
        return false;
    }
    result.axles.push_back({position, maxWidth, maxWidth, minWidth, wheelDiameter, spring * 1000.0,
                            maxForce * 1000.0, damper * 1000.0, false, driven != 0.0});
    return true;
}

} // namespace

void loadPassengerCabinConfiguration(VehicleConfig& configuration) {
    loadPassengerCabin(configuration, configuration);
}

void prepareScriptConfiguration(VehicleConfig& configuration, bool convertScripts) {
    loadReferencedDefinitions(configuration.sourcePath, configuration);
    if (convertScripts) {
        convertVehicleScripts(configuration.sourcePath, configuration);
    }
}

VehicleConfig loadVehicleConfig(const std::filesystem::path& configPath, VehicleFileKind kind,
                                bool convertScripts) {
    openbus::rendering::TraceScope trace("config", "loadVehicleConfig");
    VehicleConfig result;
    result.kind = kind;
    result.sourcePath = configPath;
    Reader reader(configPath);
    if (!reader.isOpen()) {
        result.diagnostics.error(0, "file", "unable to open " + configPath.string());
        return result;
    }

    std::size_t scheduleLine = 0;
    std::size_t ticketSellingLine = 0;
    std::size_t cogLine = 0;
    const std::string configFormat =
        kind == VehicleFileKind::Bus ? "bus.cfg" : "vehicle.cfg";
    Line line;
    while (reader.next(line)) {
        if (!line.isKeyword()) {
            continue;
        }
        const std::string keyword = lower(line.keyword());
        if (knownVehicleKeywords().find(keyword) != knownVehicleKeywords().end()) {
            const std::string ignoredReason = ignoredVehicleKeywordReason(keyword);
            openbus::config::logConfigurationTagStatus(
                configFormat, configPath, line.number, line.keyword(),
                ignoredReason.empty() ? openbus::config::ConfigurationTagStatus::Active
                                      : openbus::config::ConfigurationTagStatus::Ignored,
                ignoredReason.empty() ? "vehicle configuration record has runtime handling"
                                      : ignoredReason);
        }
        if (keyword == "end") {
            continue;
        }
        if (keyword == "description") {
            bool terminated = false;
            std::string description;
            bool firstLine = true;
            while (reader.nextIncludingBlank(line)) {
                if (line.isKeyword() && lower(line.keyword()) == "end") {
                    terminated = true;
                    break;
                }
                if (!firstLine) {
                    description.push_back('\n');
                }
                description += line.raw;
                firstLine = false;
            }
            result.description = std::move(description);
            if (!terminated) {
                result.diagnostics.error(line.number, "description", "missing [end]");
            }
            continue;
        }
        if (keyword == "friendlyname") {
            std::vector<std::string> values;
            if (readValues(reader, keyword, 3, values, result.diagnostics)) {
                result.friendlyManufacturer = std::move(values[0]);
                result.friendlyVehicleName = std::move(values[1]);
                result.friendlyDefaultPaint = std::move(values[2]);
            }
            continue;
        }
        if (keyword == "type") {
            Line value;
            int vehicleType = 0;
            if (reader.readPayload(value, result.diagnostics, keyword)) {
                if (parseInt(value.text, vehicleType)) {
                    result.vehicleType = vehicleType;
                } else {
                    result.diagnostics.error(value.number, keyword, "expected an integer value");
                }
            }
            continue;
        }
        if (keyword == "model") {
            std::string modelPath;
            if (readOneString(reader, keyword, modelPath, result.diagnostics)) {
                result.modelPath = modelPath;
            }
            continue;
        }
        if (keyword == "sound" || keyword == "sound_ai" || keyword == "paths" ||
            keyword == "passengercabin" || keyword == "number" || keyword == "registration_list") {
            std::string referencedPath;
            if (readOneString(reader, keyword, referencedPath, result.diagnostics)) {
                const std::filesystem::path resolved =
                    resolveReferencedPath(result.sourcePath, referencedPath);
                if (keyword == "sound") {
                    result.soundConfigPath = resolved;
                } else if (keyword == "sound_ai") {
                    result.soundAiConfigPath = resolved;
                } else if (keyword == "paths") {
                    result.pathsConfigPath = resolved;
                } else if (keyword == "passengercabin") {
                    result.passengerCabinConfigPath = resolved;
                } else if (keyword == "number") {
                    result.numberConfigPath = resolved;
                } else {
                    result.registrationListConfigPath = resolved;
                }
            }
            continue;
        }
        if (keyword == "registration_automatic") {
            std::string prefix;
            if (readOneString(reader, keyword, prefix, result.diagnostics)) {
                result.registrationAutomatic = true;
                result.registrationPrefix = prefix;
            }
            continue;
        }
        if (keyword == "registration_free") {
            result.registrationFree = true;
            continue;
        }
        if (keyword == "script" || keyword == "varnamelist" || keyword == "stringvarnamelist" ||
            keyword == "constfile") {
            Line countLine;
            int count = 0;
            if (!reader.readPayload(countLine, result.diagnostics, keyword) ||
                !parseInt(countLine.text, count) || count < 0) {
                result.diagnostics.error(line.number, keyword,
                                         "expected a non-negative entry count");
                continue;
            }
            if (static_cast<std::size_t>(count) > MAX_CONFIG_RECORDS) {
                result.diagnostics.error(line.number, keyword, "entry count is too large");
                continue;
            }
            std::vector<std::string> values;
            if (!readValues(reader, keyword, static_cast<std::size_t>(count), values,
                            result.diagnostics)) {
                continue;
            }
            if (keyword == "script") {
                result.scripts = std::move(values);
            } else if (keyword == "varnamelist") {
                result.variableLists = std::move(values);
            } else if (keyword == "stringvarnamelist") {
                result.stringVariableLists = std::move(values);
            } else {
                result.constantFiles = std::move(values);
            }
            continue;
        }
        if (keyword == "add_camera_driver" || keyword == "add_camera_pax" ||
            keyword == "add_camera_reflexion" || keyword == "add_camera_reflexion_2" ||
            keyword == "add_camera_reflexion_static" ||
            keyword == "add_camera_reflexion_2_static") {
            if (!allowsBusOnly(keyword, kind)) {
                result.diagnostics.error(line.number, keyword,
                                         "keyword is only valid in a Bus configuration");
            }
            VehicleCamera camera;
            const bool staticReflection = keyword == "add_camera_reflexion_static" ||
                                          keyword == "add_camera_reflexion_2_static";
            const VehicleCameraKind cameraKind =
                keyword == "add_camera_driver" ? VehicleCameraKind::Driver
                : keyword == "add_camera_pax"  ? VehicleCameraKind::Passenger
                : keyword == "add_camera_reflexion" || keyword == "add_camera_reflexion_static"
                    ? VehicleCameraKind::Reflexion
                    : VehicleCameraKind::Reflexion2;
            camera.followsViewer = !staticReflection;
            if (parseCamera(reader, keyword, cameraKind, camera, result.diagnostics)) {
                result.cameras.push_back(camera);
            }
            continue;
        }
        if (keyword == "set_camera_std") {
            int index = -1;
            Line value;
            if (reader.readPayload(value, result.diagnostics, keyword) &&
                parseInt(value.text, index)) {
                result.standardDriverCamera = index;
            } else {
                result.diagnostics.error(line.number, keyword, "expected a camera index");
            }
            continue;
        }
        if (keyword == "set_camera_outside_center") {
            std::vector<double> values;
            if (readDoubleRecord(reader, keyword, 3, values, result.diagnostics)) {
                result.outsideCameraCenter = {values[0], values[1], values[2]};
                result.hasOutsideCameraCenter = true;
            }
            continue;
        }
        if (keyword == "view_schedule" || keyword == "view_ticketselling") {
            if (!allowsBusOnly(keyword, kind)) {
                result.diagnostics.error(line.number, keyword,
                                         "keyword is only valid in a Bus configuration");
            }
            VehicleViewEntry entry;
            Line next;
            const bool hasNext = reader.next(next);
            int explicitIndex = -1;
            if (hasNext && !next.isKeyword() && parseInt(next.text, explicitIndex)) {
                Line label;
                if (!reader.readPayload(label, result.diagnostics, keyword)) {
                    continue;
                }
                entry.viewIndex = explicitIndex;
                entry.label = trim(label.text);
                if (entry.viewIndex < 0 || entry.label.empty()) {
                    result.diagnostics.error(
                        line.number, keyword,
                        "expected a non-negative view index and a non-empty label");
                    continue;
                }
            } else {
                if (hasNext) {
                    reader.pushBack(std::move(next));
                }
                if (result.cameras.empty() ||
                    result.cameras.back().kind != VehicleCameraKind::Driver) {
                    result.diagnostics.error(line.number, keyword,
                                             "marker must follow a driver camera");
                    continue;
                }
                entry.viewIndex =
                    static_cast<int>(std::count_if(result.cameras.begin(), result.cameras.end(),
                                                   [](const VehicleCamera& camera) {
                                                       return camera.kind ==
                                                              VehicleCameraKind::Driver;
                                                   })) -
                    1;
            }
            if (keyword == "view_schedule") {
                scheduleLine = line.number;
                result.scheduleViews.push_back(std::move(entry));
            } else {
                ticketSellingLine = line.number;
                result.ticketSellingViews.push_back(std::move(entry));
            }
            continue;
        }
        if (keyword == "newachse") {
            parseAxle(reader, result, line);
            continue;
        }
        if (keyword == "mass") {
            std::vector<double> values;
            if (readDoubleRecord(reader, keyword, 1, values, result.diagnostics)) {
                result.massTonnes = values[0];
            }
            continue;
        }
        if (keyword == "momentofintertia") {
            std::vector<double> values;
            if (readDoubleRecord(reader, keyword, 3, values, result.diagnostics)) {
                result.momentOfInertia = {values[0], values[1], values[2]};
            }
            continue;
        }
        if (keyword == "boundingbox") {
            std::vector<double> values;
            if (readDoubleRecord(reader, keyword, 6, values, result.diagnostics)) {
                for (std::size_t index = 0; index < values.size(); ++index) {
                    result.boundingBox[index] = values[index];
                }
                result.hasBoundingBox = true;
            }
            continue;
        }
        if (keyword == "openbus_wheel_half_width") {
            std::vector<double> values;
            if (readDoubleRecord(reader, keyword, 1, values, result.diagnostics) &&
                values[0] > 0.0) {
                result.wheelHalfWidth = values[0];
                result.hasWheelHalfWidth = true;
            } else {
                result.diagnostics.error(line.number, keyword, "expected a positive width");
            }
            continue;
        }
        if (keyword == "openbus_articulated") {
            int value = 0;
            Line valueLine;
            if (reader.readPayload(valueLine, result.diagnostics, keyword) &&
                parseInt(valueLine.text, value) && (value == 0 || value == 1)) {
                result.articulated = value != 0;
                result.hasArticulated = true;
            } else {
                result.diagnostics.error(line.number, keyword, "expected 0 or 1");
            }
            continue;
        }
        if (keyword == "cog") {
            std::vector<double> values;
            if (readDoubleRecord(reader, keyword, 3, values, result.diagnostics, line.number)) {
                if (values[2] <= 0.0) {
                    result.diagnostics.error(line.number, keyword,
                                             "center-of-gravity height must be positive");
                } else {
                    result.centerOfGravity = std::array<double, 3>{values[0], values[1], values[2]};
                    cogLine = line.number;
                    if (values[0] != 0.0 || values[1] != 0.0) {
                        result.diagnostics.warning(
                            line.number, keyword,
                            "horizontal center-of-gravity offsets are retained but not applied");
                    }
                }
            } else {
                result.diagnostics.error(line.number, keyword, "expected three finite coordinates");
            }
            continue;
        }
        if (keyword == "schwerpunkt") {
            std::vector<double> values;
            if (readDoubleRecord(reader, keyword, 1, values, result.diagnostics, line.number)) {
                result.centerOfGravityHeight = values[0];
                result.hasCenterOfGravityHeight = true;
            } else {
                result.diagnostics.error(line.number, keyword, "expected a finite height");
            }
            continue;
        }
        if (keyword == "kmcounter_init") {
            std::vector<std::string> values;
            if (readValues(reader, keyword, 2, values, result.diagnostics) &&
                parseInt(values[0], result.odometerInitialYear) &&
                parseDouble(values[1], result.odometerInitialKilometres) &&
                result.odometerInitialYear >= 0 && result.odometerInitialKilometres >= 0.0) {
                result.hasOdometerInitial = true;
            } else {
                result.diagnostics.error(line.number, keyword,
                                         "expected a non-negative year and odometer value");
            }
            continue;
        }
        if (keyword == "rollwiderstand") {
            std::vector<double> values;
            if (readDoubleRecord(reader, keyword, 1, values, result.diagnostics)) {
                if (values[0] < 0.0) {
                    result.diagnostics.error(line.number, keyword,
                                             "rolling resistance must be non-negative");
                } else {
                    result.rollingResistanceForceNewtons = values[0];
                }
            }
            continue;
        }
        if (keyword == "rot_pnt_long" || keyword == "inv_min_turnradius" ||
            keyword == "ai_deltaheight") {
            std::vector<double> values;
            if (readDoubleRecord(reader, keyword, 1, values, result.diagnostics)) {
                if (keyword == "inv_min_turnradius") {
                    result.inverseMinimumTurnRadius = values[0];
                    result.hasInverseMinimumTurnRadius = true;
                } else if (keyword == "ai_deltaheight") {
                    result.aiDeltaHeight = values[0];
                    result.hasAiDeltaHeight = true;
                } else {
                    result.longitudinalRotationPoint = values[0];
                    result.diagnostics.warning(
                        line.number, keyword,
                        "recognized and retained but not applied to vehicle physics");
                }
            }
            continue;
        }
        if (keyword == "coupling_front" || keyword == "coupling_back") {
            std::vector<double> values;
            if (readDoubleRecord(reader, keyword, 3, values, result.diagnostics, line.number)) {
                const std::array<double, 3> point = {values[0], values[1], values[2]};
                if (keyword == "coupling_front") {
                    result.frontCouplingPoint = point;
                } else {
                    result.rearCouplingPoint = point;
                }
                result.diagnostics.warning(
                    line.number, keyword,
                    "coupling point retained; articulated vehicle behavior is not implemented");
            }
            continue;
        }
        if (keyword == "coupling_front_character") {
            std::vector<std::string> values;
            VehicleCouplingCharacter character;
            if (!readValues(reader, keyword, 4, values, result.diagnostics)) {
                continue;
            }
            if (parseDouble(values[0], character.alphaDegrees) &&
                parseDouble(values[1], character.minimumBetaDegrees) &&
                parseDouble(values[2], character.maximumBetaDegrees) &&
                parseInt(values[3], character.type) &&
                character.minimumBetaDegrees <= character.maximumBetaDegrees &&
                (character.type == 0 || character.type == 1)) {
                result.frontCouplingCharacter = character;
                result.diagnostics.warning(
                    line.number, keyword,
                    "coupling character retained; articulated vehicle behavior is not implemented");
            } else {
                result.diagnostics.error(
                    line.number, keyword,
                    "expected alpha, ordered beta limits, and type 0 (truck) or 1 (bus)");
            }
            continue;
        }
        if (keyword == "couple_back") {
            std::vector<std::string> values;
            if (!readValues(reader, keyword, 2, values, result.diagnostics)) {
                continue;
            }
            const std::string flagValue = lower(trim(values[1]));
            if (trim(values[0]).empty() || (flagValue != "true" && flagValue != "false")) {
                result.diagnostics.error(line.number, keyword,
                                         "expected a vehicle config path and a true/false flag");
            } else {
                result.rearCouplingReference = VehicleCouplingReference{
                    resolveReferencedPath(configPath, trim(values[0])), flagValue == "true"};
                result.diagnostics.warning(line.number, keyword,
                                           "rear vehicle reference retained; articulated vehicle "
                                           "behavior is not implemented");
            }
            continue;
        }

        const auto unsupported = unsupportedVehicleKeywords().find(keyword);
        if (unsupported != unsupportedVehicleKeywords().end()) {
            const std::string message = "recognized but unsupported: " + unsupported->second;
            result.diagnostics.warning(line.number, line.keyword(), message);
            openbus::config::logUnhandledConfigurationTag(
                configFormat, configPath, line.number, line.keyword(), message);
            continue;
        }
        result.diagnostics.warning(line.number, line.keyword(), "unknown vehicle keyword");
        openbus::config::logConfigurationTagStatus(
            configFormat, configPath, line.number, line.keyword(),
            openbus::config::ConfigurationTagStatus::Unknown,
            "vehicle configuration keyword is not recognized");
    }

    if (result.centerOfGravity) {
        if (!result.hasCenterOfGravityHeight) {
            result.centerOfGravityHeight = (*result.centerOfGravity)[2];
            result.hasCenterOfGravityHeight = true;
        } else if (result.centerOfGravityHeight != (*result.centerOfGravity)[2]) {
            result.diagnostics.warning(
                cogLine, "cog", "[schwerpunkt] takes precedence over the different [cog] height");
        }
    }
    if (result.modelPath.empty()) {
        result.diagnostics.error(0, "model", "vehicle configuration has no [model] entry");
    }
    if (result.standardDriverCamera >= 0) {
        int driverCount = 0;
        for (const VehicleCamera& camera : result.cameras) {
            if (camera.kind == VehicleCameraKind::Driver) {
                ++driverCount;
            }
        }
        if (result.standardDriverCamera >= driverCount) {
            result.diagnostics.error(
                0, "set_camera_std",
                "default driver camera index is outside the driver camera list");
        }
    }
    if (scheduleLine != 0 &&
        std::none_of(result.cameras.begin(), result.cameras.end(), [](const VehicleCamera& camera) {
            return camera.kind == VehicleCameraKind::Driver;
        })) {
        result.diagnostics.error(scheduleLine, "view_schedule",
                                 "must follow at least one [add_camera_driver]");
    }
    if (ticketSellingLine != 0 &&
        std::none_of(result.cameras.begin(), result.cameras.end(), [](const VehicleCamera& camera) {
            return camera.kind == VehicleCameraKind::Driver;
        })) {
        result.diagnostics.error(ticketSellingLine, "view_ticketselling",
                                 "must follow at least one [add_camera_driver]");
    }
    // Shared script resources are also consumed by scenery-object configurations.
    prepareScriptConfiguration(result, convertScripts);
    loadPassengerCabin(result, result);
    loadPassengerPaths(result, result);
    loadRegistrationEntries(result.numberConfigPath, result.vehicleNumbers, result);
    loadRegistrationEntries(result.registrationListConfigPath, result.registrationNumbers, result);
    if (!result.vehicleNumbers.empty() && !result.registrationNumbers.empty() &&
        result.vehicleNumbers.size() != result.registrationNumbers.size()) {
        result.diagnostics.warning(
            0, "registration_list",
            "number and registration lists have different lengths; unmatched trailing entries "
            "are ignored");
    }
    selectRegistrationAtIndex(result, 0);
    return result;
}

std::size_t registrationOptionCount(const VehicleConfig& configuration) {
    if (!configuration.vehicleNumbers.empty() && !configuration.registrationNumbers.empty()) {
        return std::min(configuration.vehicleNumbers.size(),
                        configuration.registrationNumbers.size());
    }
    return (std::max)(configuration.vehicleNumbers.size(),
                      configuration.registrationNumbers.size());
}

void selectRegistrationAtIndex(VehicleConfig& configuration, std::size_t index) {
    const std::size_t optionCount = registrationOptionCount(configuration);
    if (optionCount == 0) {
        return;
    }

    index %= optionCount;
    configuration.selectedRegistrationIndex = index;
    configuration.selectedVehicleNumber =
        index < configuration.vehicleNumbers.size() ? configuration.vehicleNumbers[index] : "";
    configuration.selectedRegistration = index < configuration.registrationNumbers.size()
                                             ? configuration.registrationNumbers[index]
                                             : configuration.selectedVehicleNumber;
}

VehicleRegistrationSelection
vehicleRegistrationSelectionAtIndex(const VehicleConfig& configuration, std::size_t index,
                                    std::string_view freeVehicleNumber,
                                    std::string_view freeRegistration) {
    VehicleRegistrationSelection selection;
    const std::size_t optionCount = registrationOptionCount(configuration);
    if (optionCount > 0) {
        VehicleConfig selected = configuration;
        selectRegistrationAtIndex(selected, index);
        selection.vehicleNumber = configuration.registrationFree ? std::string(freeVehicleNumber)
                                                                 : selected.selectedVehicleNumber;
        if (configuration.registrationFree) {
            selection.registration = std::string(freeRegistration);
        } else if (!configuration.registrationAutomatic) {
            selection.registration = selected.selectedRegistration;
        }
    } else if (configuration.registrationFree) {
        selection.vehicleNumber = std::string(freeVehicleNumber);
        selection.registration = std::string(freeRegistration);
    }
    return selection;
}

std::optional<std::size_t>
vehicleCameraArrayIndexForDriverView(const std::vector<VehicleCamera>& cameras, int viewIndex) {
    if (viewIndex < 0) {
        return std::nullopt;
    }

    int driverViewIndex = 0;
    for (std::size_t cameraIndex = 0; cameraIndex < cameras.size(); ++cameraIndex) {
        if (cameras[cameraIndex].kind != VehicleCameraKind::Driver) {
            continue;
        }
        if (driverViewIndex == viewIndex) {
            return cameraIndex;
        }
        ++driverViewIndex;
    }
    return std::nullopt;
}

std::vector<int> passengerPathRoute(const std::vector<PassengerPathPoint>& points,
                                    const std::vector<PassengerPathLink>& links, int fromPoint,
                                    int toPoint) {
    const auto pointIndex = [&](int pointId) -> std::optional<std::size_t> {
        const auto point =
            std::find_if(points.begin(), points.end(), [&](const PassengerPathPoint& candidate) {
                return candidate.index == pointId;
            });
        if (point == points.end()) {
            return std::nullopt;
        }
        return static_cast<std::size_t>(std::distance(points.begin(), point));
    };
    const auto start = pointIndex(fromPoint);
    const auto finish = pointIndex(toPoint);
    if (!start || !finish) {
        return {};
    }
    if (fromPoint == toPoint) {
        return {fromPoint};
    }

    struct Edge {
        std::size_t destination;
        double length;
    };
    std::vector<std::vector<Edge>> graph(points.size());
    const auto addEdge = [&](std::size_t from, std::size_t to) {
        const auto& first = points[from].position;
        const auto& second = points[to].position;
        const double length = std::hypot(std::hypot(second[0] - first[0], second[1] - first[1]),
                                         second[2] - first[2]);
        graph[from].push_back({to, length});
    };
    for (const PassengerPathLink& link : links) {
        const auto from = pointIndex(link.from);
        const auto to = pointIndex(link.to);
        if (!from || !to || *from == *to) {
            continue;
        }
        addEdge(*from, *to);
        if (!link.oneWay) {
            addEdge(*to, *from);
        }
    }

    std::vector<double> distances(points.size(), std::numeric_limits<double>::infinity());
    std::vector<std::size_t> previous(points.size(), points.size());
    std::vector<bool> visited(points.size(), false);
    distances[*start] = 0.0;
    for (std::size_t iteration = 0; iteration < points.size(); ++iteration) {
        std::size_t current = points.size();
        for (std::size_t index = 0; index < points.size(); ++index) {
            if (!visited[index] &&
                (current == points.size() || distances[index] < distances[current])) {
                current = index;
            }
        }
        if (current == points.size() || !std::isfinite(distances[current])) {
            break;
        }
        if (current == *finish) {
            break;
        }
        visited[current] = true;
        for (const Edge& edge : graph[current]) {
            const double candidate = distances[current] + edge.length;
            if (candidate < distances[edge.destination]) {
                distances[edge.destination] = candidate;
                previous[edge.destination] = current;
            }
        }
    }
    if (!std::isfinite(distances[*finish])) {
        return {};
    }
    std::vector<int> route;
    for (std::size_t current = *finish;; current = previous[current]) {
        route.push_back(points[current].index);
        if (current == *start) {
            break;
        }
        if (previous[current] >= points.size()) {
            return {};
        }
    }
    std::reverse(route.begin(), route.end());
    return route;
}

std::optional<std::size_t> passengerExitDoorForSeat(const VehicleConfig& configuration,
                                                    std::size_t seatIndex,
                                                    std::size_t maximumDoorCount) {
    const auto& seat = configuration.passengerPositions.at(seatIndex);
    const auto& points = configuration.passengerPathPoints;
    const auto& exits = configuration.passengerExitPathPoints;
    const std::size_t count = std::min(exits.size(), maximumDoorCount);
    if (count == 0) {
        return std::nullopt;
    }
    const std::array<double, 3> seatFloor = {seat.position[0], seat.position[1],
                                             seat.position[2] - seat.seatHeight};
    const auto distance = [](const auto& a, const auto& b) {
        return std::hypot(std::hypot(a[0] - b[0], a[1] - b[1]), a[2] - b[2]);
    };
    const auto findPoint = [&](int id) -> const PassengerPathPoint* {
        const auto found = std::find_if(points.begin(), points.end(),
                                        [id](const auto& point) { return point.index == id; });
        return found == points.end() ? nullptr : &*found;
    };
    const PassengerPathPoint* nearest = nullptr;
    double nearestDistance = std::numeric_limits<double>::infinity();
    for (const auto& point : points) {
        const double candidate = distance(seatFloor, point.position);
        if (candidate < nearestDistance) {
            nearest = &point;
            nearestDistance = candidate;
        }
    }
    std::optional<std::size_t> bestConnected;
    std::optional<std::size_t> bestDirect;
    double bestRouteLength = std::numeric_limits<double>::infinity();
    double bestDirectDistance = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0; index < count; ++index) {
        const auto* exit = findPoint(exits[index]);
        if (!exit) {
            continue;
        }
        const double directDistance = distance(seatFloor, exit->position);
        if (directDistance < bestDirectDistance) {
            bestDirect = index;
            bestDirectDistance = directDistance;
        }
        if (!nearest) {
            continue;
        }
        const auto route = passengerPathRoute(points, configuration.passengerPathLinks,
                                              nearest->index, exit->index);
        if (route.empty()) {
            continue;
        }
        double length = nearestDistance;
        for (std::size_t routeIndex = 1; routeIndex < route.size(); ++routeIndex) {
            const auto* from = findPoint(route[routeIndex - 1]);
            const auto* to = findPoint(route[routeIndex]);
            if (!from || !to) {
                throw std::logic_error("Passenger path route contains an unknown point");
            }
            length += distance(from->position, to->position);
        }
        if (length < bestRouteLength) {
            bestConnected = index;
            bestRouteLength = length;
        }
    }
    return bestConnected ? bestConnected : bestDirect ? bestDirect : std::optional<std::size_t>{0};
}

std::vector<std::pair<double, std::optional<std::size_t>>>
passengerPathLinkProgressProfile(const std::vector<std::array<double, 3>>& path,
                                 const std::vector<std::optional<std::size_t>>& segmentLinks) {
    std::vector<std::pair<double, std::optional<std::size_t>>> profile;
    if (path.size() < 2 || segmentLinks.size() != path.size() - 1) {
        return profile;
    }
    double totalLength = 0.0;
    for (std::size_t index = 1; index < path.size(); ++index) {
        const auto& from = path[index - 1];
        const auto& to = path[index];
        const double length = std::hypot(to[0] - from[0], to[1] - from[1], to[2] - from[2]);
        if (!std::isfinite(length) || !std::isfinite(totalLength + length)) {
            return {};
        }
        totalLength += length;
    }
    if (totalLength <= 1.0e-9) {
        return profile;
    }

    profile.reserve(segmentLinks.size());
    double cumulativeLength = 0.0;
    for (std::size_t index = 1; index < path.size(); ++index) {
        const auto& from = path[index - 1];
        const auto& to = path[index];
        const double length = std::hypot(to[0] - from[0], to[1] - from[1], to[2] - from[2]);
        if (length <= 1.0e-9) {
            continue;
        }
        cumulativeLength += length;
        const double endProgress = std::min(1.0, cumulativeLength / totalLength);
        if (profile.empty() || endProgress > profile.back().first) {
            profile.emplace_back(endProgress, segmentLinks[index - 1]);
        }
    }
    if (!profile.empty()) {
        profile.back().first = 1.0;
    }
    return profile;
}

std::optional<std::size_t>
passengerPathLinkAtProgress(const std::vector<std::array<double, 3>>& path,
                            const std::vector<std::optional<std::size_t>>& segmentLinks,
                            double progress) {
    if (!std::isfinite(progress)) {
        return std::nullopt;
    }
    const auto profile = passengerPathLinkProgressProfile(path, segmentLinks);
    const double clampedProgress = std::clamp(progress, 0.0, 1.0);
    const auto segment =
        std::find_if(profile.begin(), profile.end(), [clampedProgress](const auto& candidate) {
            return clampedProgress <= candidate.first;
        });
    return segment == profile.end() ? std::nullopt : segment->second;
}

std::filesystem::path soundConfigPathForVehicle(const VehicleConfig& configuration,
                                                bool isAiVehicle) {
    if (isAiVehicle && !configuration.soundAiConfigPath.empty()) {
        return configuration.soundAiConfigPath;
    }
    return configuration.soundConfigPath;
}

VehiclePlacement placementForVehicle(const VehicleConfig& configuration,
                                     const VehiclePlacement& placement, bool isAiVehicle) {
    VehiclePlacement result = placement;
    if (isAiVehicle && configuration.hasAiDeltaHeight) {
        result.position[2] += configuration.aiDeltaHeight;
    }
    return result;
}

ModelConfig loadVehicleModelConfig(const std::filesystem::path& configPath,
                                   const std::filesystem::path& modelRoot,
                                   openbus::scripting::Vehicle& variables) {
    return loadModelConfig(configPath, modelRoot, ModelConfigKind::Vehicle, variables);
}
