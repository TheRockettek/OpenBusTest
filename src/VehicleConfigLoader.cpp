#include "VehicleConfigLoader.h"

#include "ConfigurationParser.h"
#include "ModelConfigLoader.h"

#include <array>
#include <algorithm>
#include <filesystem>
#include <string>
#include <utility>
#include <vector>

namespace {

using openbus::config::Line;
using openbus::config::Reader;
using openbus::config::lower;
using openbus::config::parseDouble;
using openbus::config::parseInt;
using openbus::config::trim;

bool readValues(Reader& reader, const std::string& keyword, std::size_t count,
                std::vector<std::string>& values, ConfigurationDiagnostics& diagnostics) {
    return reader.readPayloads(count, values, diagnostics, keyword);
}

bool readDoubleRecord(Reader& reader, const std::string& keyword, std::size_t count,
                      std::vector<double>& values, ConfigurationDiagnostics& diagnostics) {
    std::vector<std::string> raw;
    if (!readValues(reader, keyword, count, raw, diagnostics)) {
        return false;
    }
    values.clear();
    values.reserve(raw.size());
    for (const std::string& value : raw) {
        double parsed = 0.0;
        if (!parseDouble(value, parsed)) {
            diagnostics.error(0, keyword, "expected numeric values");
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
           keyword != "view_schedule" && keyword != "view_ticketselling";
}

std::filesystem::path resolveReferencedPath(const std::filesystem::path& configPath,
                                            const std::string& referencedPath) {
    std::string normalized = referencedPath;
    std::replace(normalized.begin(), normalized.end(), '\\', '/');
    return configPath.parent_path() / std::filesystem::path(normalized);
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
        const std::string name = trim(line.text);
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
            double value = 0.0;
            if (!reader.readPayload(nameLine, result.diagnostics, "const") ||
                !reader.readPayload(valueLine, result.diagnostics, "const") ||
                !parseDouble(valueLine.text, value)) {
                result.diagnostics.error(line.number, "const", "expected a name and numeric value");
                continue;
            }
            result.constants[trim(nameLine.text)] = value;
            continue;
        }
        if (keyword == "newcurve") {
            Line nameLine;
            if (!reader.readPayload(nameLine, result.diagnostics, "newcurve")) {
                continue;
            }
            ConstantCurve curve;
            curve.name = trim(nameLine.text);
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
                if (!parseDouble(values[0], point.x) || !parseDouble(values[1], point.y)) {
                    result.diagnostics.error(line.number, "pnt", "expected numeric x and y values");
                    continue;
                }
                curve.points.push_back(point);
            }
            if (curve.name.empty() || curve.points.empty()) {
                result.diagnostics.error(nameLine.number, "newcurve",
                                         "curve requires a name and at least one point");
            } else {
                result.curves.push_back(std::move(curve));
            }
        }
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
    if (kind == VehicleCameraKind::Reflexion2) {
        camera.culling = values[7];
    }
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

    // TODO: Validate all axle dimensions and suspension values before accepting the record.
    if (!hasPosition || !hasMaxWidth || !hasMinWidth || !hasWheelDiameter || !hasSpring ||
        !hasMaxForce || !hasDamper || !hasDriven || maxWidth <= 0.0 || wheelDiameter <= 0.0) {
        result.diagnostics.error(keywordLine.number, "newachse",
                                 "axle requires position, widths, wheel, suspension, and drive values");
        return false;
    }
    result.axles.push_back({position,
                            maxWidth,
                            maxWidth,
                            minWidth,
                            wheelDiameter,
                            spring * 1000.0,
                            maxForce * 1000.0,
                            damper * 1000.0,
                            false,
                            driven != 0.0});
    return true;
}

}  // namespace

VehicleConfig loadVehicleConfig(const std::filesystem::path& configPath, VehicleFileKind kind) {
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
    Line line;
    while (reader.next(line)) {
        if (!line.isKeyword()) {
            continue;
        }
        const std::string keyword = lower(line.keyword());
        if (keyword == "end") {
            continue;
        }
        if (keyword == "description") {
            bool terminated = false;
            while (reader.next(line)) {
                if (line.isKeyword() && lower(line.keyword()) == "end") {
                    terminated = true;
                    break;
                }
            }
            if (!terminated) {
                result.diagnostics.error(line.number, "description", "missing [end]");
            }
            continue;
        }
        if (keyword == "friendlyname") {
            std::vector<std::string> values;
            readValues(reader, keyword, 3, values, result.diagnostics);
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
            std::string ignored;
            readOneString(reader, keyword, ignored, result.diagnostics);
            continue;
        }
        if (keyword == "script" || keyword == "varnamelist" || keyword == "stringvarnamelist" ||
            keyword == "constfile") {
            Line countLine;
            int count = 0;
            if (!reader.readPayload(countLine, result.diagnostics, keyword) ||
                !parseInt(countLine.text, count) || count < 0) {
                result.diagnostics.error(line.number, keyword, "expected a non-negative entry count");
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
            keyword == "add_camera_reflexion" || keyword == "add_camera_reflexion_2") {
            if (!allowsBusOnly(keyword, kind)) {
                result.diagnostics.error(line.number, keyword,
                                         "keyword is only valid in a Bus configuration");
            }
            VehicleCamera camera;
            const VehicleCameraKind cameraKind =
                keyword == "add_camera_driver"
                    ? VehicleCameraKind::Driver
                    : keyword == "add_camera_pax"
                          ? VehicleCameraKind::Passenger
                          : keyword == "add_camera_reflexion"
                                ? VehicleCameraKind::Reflexion
                                : VehicleCameraKind::Reflexion2;
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
            if (keyword == "view_schedule") {
                scheduleLine = line.number;
            } else {
                ticketSellingLine = line.number;
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
        if (keyword == "schwerpunkt") {
            std::vector<double> values;
            if (readDoubleRecord(reader, keyword, 1, values, result.diagnostics)) {
                result.centerOfGravityHeight = values[0];
                result.hasCenterOfGravityHeight = true;
            }
            continue;
        }
        if (keyword == "kmcounter_init") {
            std::vector<std::string> values;
            readValues(reader, keyword, 2, values, result.diagnostics);
            continue;
        }
        if (keyword == "rollwiderstand" || keyword == "rot_pnt_long" ||
            keyword == "inv_min_turnradius" || keyword == "ai_deltaheight") {
            std::vector<double> values;
            readDoubleRecord(reader, keyword, 1, values, result.diagnostics);
            continue;
        }

        result.diagnostics.warning(line.number, line.keyword(), "unknown vehicle keyword");
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
            result.diagnostics.error(0, "set_camera_std",
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
    loadReferencedDefinitions(configPath, result);
    return result;
}

ModelConfig loadVehicleModelConfig(const std::filesystem::path& configPath,
                                   const std::filesystem::path& modelRoot, Variables& variables) {
    return loadModelConfig(configPath, modelRoot, ModelConfigKind::Vehicle, variables);
}
