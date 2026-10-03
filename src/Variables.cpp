#include "Variables.h"

#include <ctime>
#include <unordered_set>

Variables::Variables(ScriptObjectKind objectKind) : objectKind_(objectKind) {}

// void Variables::configure(std::initializer_list<VariableDefinition> definitions) {
//     for (const VariableDefinition& definition : definitions) {
//         if (!definition.name.empty()) {
//             values_.try_emplace(definition.name, definition.value);
//         }
//     }
// }

void Variables::declare(const std::string& name) {
    if (name.empty()) {
        return;
    }
    values_.try_emplace(name, 0.0);
}

void Variables::declareString(const std::string& name) {
    if (name.empty()) {
        return;
    }
    strings_.try_emplace(name);
}

bool Variables::has(const std::string& name) const {
    return values_.find(name) != values_.end();
}

bool Variables::hasString(const std::string& name) const {
    return strings_.find(name) != strings_.end();
}

double Variables::get(const std::string& name) const {
    const auto found = values_.find(name);
    return found == values_.end() ? 0.0 : found->second;
}

std::string Variables::getString(const std::string& name) const {
    const auto found = strings_.find(name);
    return found == strings_.end() ? std::string() : found->second;
}

void Variables::set(const std::string& name, double value) {
    values_[name] = value;
}

const std::unordered_map<std::string, double>& Variables::numericValues() const {
    return values_;
}

void Variables::setValues(std::initializer_list<VariableDefinition> values) {
    for (const VariableDefinition& value : values) {
        if (!value.name.empty()) {
            values_[value.name] = value.value;
        }
    }
}

void Variables::setString(const std::string& name, const std::string& value) {
    strings_[name] = value;
}

const std::unordered_map<std::string, std::string>& Variables::stringValues() const {
    return strings_;
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

    if (objectKind_ == ScriptObjectKind::Vehicle) {
        return vehicleMacros.count(name) != 0 || scriptTextureMacros.count(name) != 0;
    }
    if (objectKind_ == ScriptObjectKind::SceneryObject) {
        return sceneryMacros.count(name) != 0 || scriptTextureMacros.count(name) != 0;
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
    if (objectKind_ == ScriptObjectKind::Vehicle) {
        return vehicleTriggers.count(name) != 0 || name.rfind("railbond_", 0) == 0;
    }
    if (objectKind_ == ScriptObjectKind::Human) {
        return name == "int_haltewunsch" || name.rfind("railbond_", 0) == 0;
    }
    return false;
}

SystemVariables::SystemVariables() : Variables(ScriptObjectKind::System) {
    const std::time_t now = std::time(nullptr);
    std::tm localDate = {};
#ifdef _WIN32
    localtime_s(&localDate, &now);
#else
    localtime_r(&now, &localDate);
#endif
    setValues({
        {"timegap", 0.0},
        {"gettime", 0.0},
        {"nosound", 1.0},
        {"pause", 0.0},
        {"time", 43200.0},
        {"day", static_cast<double>(localDate.tm_mday)},
        {"month", static_cast<double>(localDate.tm_mon + 1)},
        {"year", static_cast<double>(localDate.tm_year + 1900)},
        {"dayofyear", static_cast<double>(localDate.tm_yday)},
        // These are initialized for the system-variable contract. updateFrame()
        // supplies the live cursor position, and ScriptRuntime temporarily
        // overwrites them with OMSI drag deltas while invoking a drag handler.
        {"mouse_x", 0.0},
        {"mouse_y", 0.0},
        {"preciptype", 0.0},
        {"preciprate", 0.0},
        {"coll_pos_x", 0.0},
        {"coll_pos_y", 0.0},
        {"coll_pos_z", 0.0},
        {"coll_energy", 0.0},
        {"weather_temperature", 20.0},
        {"weather_abshum", 10.0},
        {"autoclutch", 0.0},
        {"wearlifespan", 1.0},
        {"sunalt", 60.0},
    });
}

void SystemVariables::updateFrame(double timegap, double getTime, double mouseX, double mouseY) {
    setValues({
        {"timegap", timegap},
        {"gettime", getTime},
        {"mouse_x", mouseX},
        {"mouse_y", mouseY},
        {"preciprate", 0.0},
        {"preciptype", 0.0},
    });
}

namespace openbus::scripting {

namespace {

void configureVehicle(Variables& variables) {
    variables.setValues({
        {"envir_brightness", 1.0},
        {"streetcond", 0.0},
        {"spot_select", -1.0},
        {"colorscheme", 0.0},
        {"m_wheel", 0.0},
        {"n_wheel", 0.0},
        {"throttle", 0.0},
        {"brake", 0.0},
        {"clutch", 0.0},
        {"brakeforce", 0.0},
        {"velocity", 0.0},
        {"velocity_ground", 0.0},
        {"tank_percent", 1.0},
        {"kmcounter_km", 0.0},
        {"kmcounter_m", 0.0},
        {"relrange", 0.0},
        {"driver_seat_verttransl", 0.0},
        {"ai", 0.0},
        {"ai_blinker_l", 0.0},
        {"ai_blinker_r", 0.0},
        {"ai_light", 0.0},
        {"ai_interiorlight", 0.0},
        {"ai_brakelight", 0.0},
        {"ai_engine", 0.0},
        {"ai_target_index", 0.0},
        {"target_index_int", 0.0},
        {"ai_scheduled_atstation", 0.0},
        {"ai_scheduled_atstation_side", 0.0},
        {"giventicket", -1.0},
        {"humans_count", 0.0},
        {"ff_vib_period", 0.0},
        {"ff_vib_amp", 0.0},
        {"snd_outsidevol", 0.0},
        {"snd_microphone", 0.0},
        {"snd_radio", 0.0},
        {"cabinair_temp", 20.0},
        {"cabinair_abshum", 10.0},
        {"cabinair_relhum", 0.0},
        {"dirt_norm", 0.0},
        {"dirtrate", 0.0},
        {"schedule_active", 0.0},
        {"train_frontcoupling", 0.0},
        {"train_backcoupling", 0.0},
        {"train_me_reverse", 0.0},
        {"trafficpriority", 0.0},
        {"trafficprioritywarningneeded", 0.0},
        {"wearlifespan", 1.0},
        {"steering", 0.0},
        {"steeringangle", 0.0},
    });
    for (int index = 0; index < 6; ++index) {
        variables.setValues({{"debug_" + std::to_string(index), 0.0}});
    }
    for (int index = 0; index < 4; ++index) {
        for (const char* side : {"l", "r"}) {
            variables.setValues({
                {"wheel_rotation_" + std::to_string(index) + "_" + side, 0.0},
                {"wheel_rotationspeed_" + std::to_string(index) + "_" + side, 0.0},
                {"axle_suspension_" + std::to_string(index) + "_" + side, 0.0},
                {"axle_steering_" + std::to_string(index) + "_" + side, 0.0},
                {"axle_springfactor_" + std::to_string(index) + "_" + side, 1.0},
                {"axle_brakeforce_" + std::to_string(index) + "_" + side, 0.0},
                {"axle_surfaceid_" + std::to_string(index) + "_" + side, 0.0},
            });
        }
        variables.setValues({
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
        variables.setValues({
            {"door_" + std::to_string(index), 0.0},
            {"pax_entry" + std::to_string(index) + "_open", 0.0},
            {"pax_exit" + std::to_string(index) + "_open", 0.0},
            {"pax_entry" + std::to_string(index) + "_req", 0.0},
            {"pax_exit" + std::to_string(index) + "_req", 0.0},
        });
    }
    for (const char* name :
         {"ident", "number", "act_route", "act_busstop", "setlineto", "yard", "file_schedule"}) {
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
    setValues({
        {"lastmoveddist", 0.0},
        {"pax_state", 0.0},
        {"heightofseat", 0.0},
        {"colorscheme", 0.0},
    });
}

SceneryObject::SceneryObject() : Variables(ScriptObjectKind::SceneryObject) {
    setValues({
        {"nightlighta", 0.0},
        {"inuse", 1.0},
        {"trafficlightphase", 0.0},
        {"trafficlightapproach", 0.0},
        {"colorscheme", 0.0},
        {"signal", 0.0},
        {"nextsignal", 0.0},
        {"refresh_strings", 0.0},
        {"switch", 0.0},
    });
}

} // namespace openbus::scripting