#include "Variables.h"

#include <algorithm>
#include <cctype>
#include <ctime>
#include <unordered_set>

namespace {

std::string keyFor(const std::string& name) {
    std::string key = name;
    std::transform(key.begin(), key.end(), key.begin(), [](unsigned char character) {
        return static_cast<char>(std::tolower(character));
    });
    return key;
}

} // namespace

Variables::Variables(ScriptObjectKind objectKind) : objectKind_(objectKind) {}

void Variables::configure(std::initializer_list<VariableDefinition> definitions) {
    for (const VariableDefinition& definition : definitions) {
        const std::string key = keyFor(definition.name);
        if (!key.empty()) {
            values_.try_emplace(key, definition.value);
        }
    }
}

void Variables::declare(const std::string& name) {
    const std::string key = keyFor(name);
    if (key.empty()) {
        return;
    }
    values_.try_emplace(key, 0.0);
}

void Variables::declareString(const std::string& name) {
    const std::string key = keyFor(name);
    if (key.empty()) {
        return;
    }
    strings_.try_emplace(key);
}

bool Variables::has(const std::string& name) const {
    return values_.find(keyFor(name)) != values_.end();
}

bool Variables::hasString(const std::string& name) const {
    return strings_.find(keyFor(name)) != strings_.end();
}

double Variables::get(const std::string& name) const {
    const auto found = values_.find(keyFor(name));
    return found == values_.end() ? 0.0 : found->second;
}

double Variables::getNormalized(const std::string& name) const {
    const auto found = values_.find(name);
    return found == values_.end() ? 0.0 : found->second;
}

std::string Variables::getString(const std::string& name) const {
    const auto found = strings_.find(keyFor(name));
    return found == strings_.end() ? std::string() : found->second;
}

std::string Variables::getStringNormalized(const std::string& name) const {
    const auto found = strings_.find(name);
    return found == strings_.end() ? std::string() : found->second;
}

void Variables::set(const std::string& name, double value) {
    const std::string key = keyFor(name);
    if (key.empty()) {
        return;
    }
    values_[key] = value;
}

void Variables::setNormalized(const std::string& name, double value) {
    values_[name] = value;
}

void Variables::setValues(std::initializer_list<VariableDefinition> values) {
    for (const VariableDefinition& value : values) {
        const std::string key = keyFor(value.name);
        if (!key.empty()) {
            values_[key] = value.value;
        }
    }
}

void Variables::setString(const std::string& name, const std::string& value) {
    const std::string key = keyFor(name);
    if (key.empty()) {
        return;
    }
    strings_[key] = value;
}

void Variables::setStringNormalized(const std::string& name, const std::string& value) {
    strings_[name] = value;
}

void Variables::clearString(const std::string& name) {
    const std::string key = keyFor(name);
    if (key.empty()) {
        return;
    }
    strings_[key].clear();
}

ScriptObjectKind Variables::objectKind() const {
    return objectKind_;
}

bool Variables::supportsSystemMacro(const std::string& name) const {
    static const std::unordered_set<std::string> vehicleMacros = {
        "getterminusindex",      "getterminuscode",     "getterminusstring",
        "getbusstopindex",       "getbusstopstring",    "getrouteindex",
        "getrouteterminusindex", "getbusstopcount",     "getroutebusstopident",
        "getttlinestring",       "getttterminusindex",  "getttbusstopcount",
        "getttbusstopindex",     "getttdelay",          "getttbusstopname",
        "getttbusstoparr",       "getttbusstopdep",     "givechangecoin",
        "getticketname",         "getticketvalue",      "gethumancountonpathlink",
        "nrspecrandom",          "getheightabovepoint", "getdepotstringglobal",
        "gethumancountonseat",
    };
    static const std::unordered_set<std::string> sceneryMacros = {
        "getarrbusline",
        "getarrbusterminus",
        "getarrbustimediff",
    };
    static const std::unordered_set<std::string> scriptTextureMacros = {
        "stnewtex",   "stlock",    "stunlock",    "stfilter",     "stsetcolor", "stdrawpixel",
        "stdrawrect", "sttextout", "streadpixel", "stcopycolor",  "stloadtex",  "stgetr",
        "stgetg",     "stgetb",    "stgeta",      "getfontindex", "textlength",
    };

    const std::string key = keyFor(name);
    if (objectKind_ == ScriptObjectKind::Vehicle) {
        return vehicleMacros.count(key) != 0 || scriptTextureMacros.count(key) != 0;
    }
    if (objectKind_ == ScriptObjectKind::SceneryObject) {
        return sceneryMacros.count(key) != 0 || scriptTextureMacros.count(key) != 0;
    }
    return false;
}

bool Variables::supportsSystemTrigger(const std::string& name) const {
    static const std::unordered_set<std::string> vehicleTriggers = {
        "collision",
        "int_haltewunsch",
        "ai_scheduled_settarget",
        "ai_scheduled_setbusstop",
        "malfunction_gettime",
        "malfunction_reset",
        "veh_tank",
    };
    const std::string key = keyFor(name);
    if (objectKind_ == ScriptObjectKind::Vehicle) {
        return vehicleTriggers.count(key) != 0 || key.rfind("railbond_", 0) == 0;
    }
    if (objectKind_ == ScriptObjectKind::Human) {
        return key == "int_haltewunsch" || key.rfind("railbond_", 0) == 0;
    }
    return false;
}

SystemVariables::SystemVariables() : Variables(ScriptObjectKind::System) {
    const std::time_t now = std::time(nullptr);
    std::tm localDate = {};
    if (const std::tm* currentDate = std::localtime(&now)) {
        localDate = *currentDate;
    }
    configure({
        {"Timegap", 0.0},
        {"GetTime", 0.0},
        {"NoSound", 1.0},
        {"Pause", 0.0},
        {"Time", 43200.0},
        {"Day", static_cast<double>(localDate.tm_mday)},
        {"Month", static_cast<double>(localDate.tm_mon + 1)},
        {"Year", static_cast<double>(localDate.tm_year + 1900)},
        {"DayOfYear", static_cast<double>(localDate.tm_yday)},
        {"mouse_x", 0.0},
        {"mouse_y", 0.0},
        {"PrecipType", 0.0},
        {"PrecipRate", 0.0},
        {"coll_pos_x", 0.0},
        {"coll_pos_y", 0.0},
        {"coll_pos_z", 0.0},
        {"coll_energy", 0.0},
        {"Weather_Temperature", 20.0},
        {"Weather_AbsHum", 10.0},
        {"AutoClutch", 0.0},
        {"wearlifespan", 1.0},
        {"SunAlt", 60.0},
    });
}

void SystemVariables::updateFrame(double timegap, double getTime, double mouseX, double mouseY) {
    setValues({
        {"Timegap", timegap},
        {"GetTime", getTime},
        {"mouse_x", mouseX},
        {"mouse_y", mouseY},
        {"PrecipRate", 0.0},
        {"PrecipType", 0.0},
    });
}

namespace openbus::scripting {

namespace {

void configureVehicle(Variables& variables) {
    variables.configure({
        {"Envir_Brightness", 1.0},
        {"StreetCond", 0.0},
        {"Spot_Select", -1.0},
        {"Colorscheme", 0.0},
        {"M_Wheel", 0.0},
        {"n_Wheel", 0.0},
        {"Throttle", 0.0},
        {"Brake", 0.0},
        {"Clutch", 0.0},
        {"Brakeforce", 0.0},
        {"Velocity", 0.0},
        {"Velocity_Ground", 0.0},
        {"tank_percent", 1.0},
        {"kmcounter_km", 0.0},
        {"kmcounter_m", 0.0},
        {"relrange", 0.0},
        {"Driver_Seat_VertTransl", 0.0},
        {"AI", 0.0},
        {"AI_Blinker_L", 0.0},
        {"AI_Blinker_R", 0.0},
        {"AI_Light", 0.0},
        {"AI_Interiorlight", 0.0},
        {"AI_Brakelight", 0.0},
        {"AI_Engine", 0.0},
        {"AI_target_index", 0.0},
        {"target_index_int", 0.0},
        {"AI_Scheduled_AtStation", 0.0},
        {"AI_Scheduled_AtStation_Side", 0.0},
        {"GivenTicket", -1.0},
        {"humans_count", 0.0},
        {"FF_Vib_Period", 0.0},
        {"FF_Vib_Amp", 0.0},
        {"Snd_OutsideVol", 0.0},
        {"Snd_Microphone", 0.0},
        {"Snd_Radio", 0.0},
        {"Cabinair_Temp", 20.0},
        {"Cabinair_absHum", 10.0},
        {"Cabinair_relHum", 0.0},
        {"Dirt_Norm", 0.0},
        {"DirtRate", 0.0},
        {"schedule_active", 0.0},
        {"train_frontcoupling", 0.0},
        {"train_backcoupling", 0.0},
        {"train_me_reverse", 0.0},
        {"TrafficPriority", 0.0},
        {"TrafficPriorityWarningNeeded", 0.0},
        {"wearlifespan", 1.0},
        {"Steering", 0.0},
        {"SteeringAngle", 0.0},
    });
    for (int index = 0; index < 6; ++index) {
        variables.configure({{"Debug_" + std::to_string(index), 0.0}});
    }
    for (int index = 0; index < 4; ++index) {
        for (const char* side : {"L", "R"}) {
            variables.configure({
                {"Wheel_Rotation_" + std::to_string(index) + "_" + side, 0.0},
                {"Wheel_RotationSpeed_" + std::to_string(index) + "_" + side, 0.0},
                {"Axle_Suspension_" + std::to_string(index) + "_" + side, 0.0},
                {"Axle_Steering_" + std::to_string(index) + "_" + side, 0.0},
                {"Axle_Springfactor_" + std::to_string(index) + "_" + side, 1.0},
                {"Axle_Brakeforce_" + std::to_string(index) + "_" + side, 0.0},
                {"Axle_SurfaceID_" + std::to_string(index) + "_" + side, 0.0},
            });
        }
        variables.configure({
            {"articulation_" + std::to_string(index) + "_alpha", 0.0},
            {"articulation_" + std::to_string(index) + "_beta", 0.0},
            {"boogie_" + std::to_string(index) + "_wheel_at_limit", 0.0},
            {"boogie_" + std::to_string(index) + "_invradius", 0.0},
            {"contactshoe_" + std::to_string(index) + "_rail_pos_x", 0.0},
            {"contactshoe_" + std::to_string(index) + "_rail_pos_y", 0.0},
            {"contactshoe_" + std::to_string(index) + "_rail_index", 0.0},
            {"contactshoe_" + std::to_string(index) + "_volt_rail", 0.0},
            {"contactshoe_" + std::to_string(index) + "_volt_veh", 0.0},
            {"contactshoe_" + std::to_string(index) + "_freq", 0.0},
        });
    }
    for (int index = 0; index < 8; ++index) {
        variables.configure({
            {"door_" + std::to_string(index), 0.0},
            {"PAX_Entry" + std::to_string(index) + "_Open", 0.0},
            {"PAX_Exit" + std::to_string(index) + "_Open", 0.0},
            {"PAX_Entry" + std::to_string(index) + "_Req", 0.0},
            {"PAX_Exit" + std::to_string(index) + "_Req", 0.0},
        });
    }
    for (const char* name :
         {"ident", "number", "act_route", "act_busstop", "SetLineTo", "yard", "file_schedule"}) {
        variables.declareString(name);
    }
}

} // namespace

Vehicle::Vehicle() : Variables(ScriptObjectKind::Vehicle) {
    configureVehicle(*this);
}

void Vehicle::updateFrame() {
    // Local variables persist; host-owned values are overwritten by their
    // simulation or system update paths before scripts run.
}

Human::Human() : Variables(ScriptObjectKind::Human) {
    configure({
        {"LastMovedDist", 0.0},
        {"PAX_State", 0.0},
        {"HeightOfSeat", 0.0},
        {"Colorscheme", 0.0},
    });
}

SceneryObject::SceneryObject() : Variables(ScriptObjectKind::SceneryObject) {
    configure({
        {"NightlightA", 0.0},
        {"InUse", 1.0},
        {"TrafficLightPhase", 0.0},
        {"TrafficLightApproach", 0.0},
        {"Colorscheme", 0.0},
        {"Signal", 0.0},
        {"NextSignal", 0.0},
        {"Refresh_Strings", 0.0},
        {"Switch", 0.0},
    });
}

} // namespace openbus::scripting