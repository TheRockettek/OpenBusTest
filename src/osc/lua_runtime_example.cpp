// lua_runtime_example.cpp
// Minimal host runtime that executes generated engine.osc.lua and provides
// get/set APIs for local, system, and constants.
//
// Build (example, Windows):
//   clang-cl /EHsc /std:c++23preview lua_runtime_example.cpp /I"C:\path\to\lua\include"
//   /I"C:\path\to\sol2\include" /link /LIBPATH:"C:\path\to\lua\lib" lua54.lib
//
// sol2 is header-only; download from https://github.com/ThePhD/sol2

#include <sol/sol.hpp>

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <windows.h>
#include <psapi.h>
#include <stack>

struct ScriptContext {
    std::unordered_map<std::string, float> local_num;
    std::unordered_map<std::string, std::string> local_str;
    std::unordered_map<std::string, float> sys_num;
    std::unordered_map<std::string, float> constants;

    float get_num(const std::unordered_map<std::string, float>& map,
                  const std::string& key) const {
        auto it = map.find(key);
        return (it == map.end()) ? 0.0F : it->second;
    }
};

int main(int argc, char** argv) {
    const std::string script_path = (argc > 1) ? argv[1] : "engine.osc.lua";

    ScriptContext ctx;

    // System variables (example)
    ctx.sys_num["Timegap"] = 1.0 / 60.0;
    ctx.sys_num["Weather_Temperature"] = 18.0;

    // Constants (add what your script expects)
    ctx.constants["engine_RPM_ign_cold"] = 700.0;
    ctx.constants["engine_RPM_ign_warm"] = 400.0;
    ctx.constants["engine_RPM_ign_time2warm"] = 35.0;
    ctx.constants["elec_busbar_minV"] = 20.0;
    ctx.constants["engine_startering_sound_n_max"] = 700.0;
    ctx.constants["engine_speedcontrol"] = 1.0;
    ctx.constants["engine_ASR"] = 1.0;
    ctx.constants["engine_e-gas"] = 1.0;
    ctx.constants["engine_J"] = 8.0;
    ctx.constants["engine_power_idle"] = 2500.0;
    ctx.constants["engine_fuel_value"] = 9.8;

    // Initial local vars
    ctx.local_num["engine_n"] = 0.0;
    ctx.local_num["engine_on"] = 0.0;
    ctx.local_num["cp_schluessel_trans_mode"] = 1.0;
    ctx.local_num["cp_schluessel_rot_mode"] = 2.0;
    ctx.local_num["cp_schluessel_rot"] = 0.0;
    ctx.local_num["engine_tank_content"] = 120.0;
    ctx.local_num["elec_busbar_main"] = 24.0;
    ctx.local_num["antrieb_getr_gangwahl"] = 1.0;

    sol::state lua;
    lua.open_libraries(sol::lib::base, sol::lib::math);

    try {
        // Data API - wrap functions that take ctx as parameter
        auto get_local_var = [&ctx](const std::string& key) {
            auto value = ctx.get_num(ctx.local_num, key);
            // std::cout << "[get_local_var] " << key << " = " << value << "\n";
            return value;
        };
        auto set_local_var = [&ctx](const std::string& key, float value) {
            // std::cout << "[set_local_var] " << key << " = " << value << "\n";
            ctx.local_num[key] = value;
        };
        auto get_local_string_var = [&ctx](const std::string& key) {
            auto it = ctx.local_str.find(key);
            auto value = (it == ctx.local_str.end()) ? std::string("") : it->second;
            // std::cout << "[get_local_string_var] " << key << " = " << value << "\n";
            return value;
        };
        auto set_local_string_var = [&ctx](const std::string& key, const std::string& value) {
            // std::cout << "[set_local_string_var] " << key << " = " << value << "\n";
            ctx.local_str[key] = value;
        };
        auto get_sys_var = [&ctx](const std::string& key) {
            auto value = ctx.get_num(ctx.sys_num, key);
            // std::cout << "[get_sys_var] " << key << " = " << value << "\n";
            return value;
        };
        auto set_sys_var = [&ctx](const std::string& key, float value) {
            // std::cout << "[set_sys_var] " << key << " = " << value << "\n";
            ctx.sys_num[key] = value;
        };
        auto get_const = [&ctx](const std::string& key) {
            auto value = ctx.get_num(ctx.constants, key);
            // std::cout << "[get_const] " << key << " = " << value << "\n";
            return value;
        };
        auto set_const_var = [&ctx](const std::string& key, float value) {
            // std::cout << "[set_const_var] " << key << " = " << value << "\n";
            ctx.constants[key] = value;
        };

        std::stack<std::string> string_stack;
        std::stack<float> float_stack;

        auto string_stack_push = [&string_stack](const std::string& value) {
            string_stack.push(value);
        };

        auto string_stack_pop = [&string_stack]() {
            if (string_stack.empty()) {
                return std::string("");
            }
            std::string value = string_stack.top();
            string_stack.pop();
            return value;
        };

        auto string_stack_peek = [&string_stack]() {
            if (string_stack.empty()) {
                return std::string("");
            }
            return string_stack.top();
        };

        auto float_stack_push = [&float_stack](float value) { float_stack.push(value); };

        auto float_stack_pop = [&float_stack]() {
            if (float_stack.empty()) {
                return 0.0;
            }
            float value = float_stack.top();
            float_stack.pop();
            return value;
        };

        auto float_stack_peek = [&float_stack]() {
            if (float_stack.empty()) {
                return 0.0;
            }
            return float_stack.top();
        };

        // Register data API

        lua.set_function("osc_f32", [](float value) { return value; });
        lua.set_function("_pushf", float_stack_push);
        lua.set_function("_popf", float_stack_pop);
        lua.set_function("_peekf", float_stack_peek);
        lua.set_function("_pushs", string_stack_push);
        lua.set_function("_pops", string_stack_pop);
        lua.set_function("_peeks", string_stack_peek);

        lua.set_function("get_local_var", get_local_var);
        lua.set_function("set_local_var", set_local_var);
        lua.set_function("get_local_string_var", get_local_string_var);
        lua.set_function("set_local_string_var", set_local_string_var);
        lua.set_function("get_sys_var", get_sys_var);
        lua.set_function("set_sys_var", set_sys_var);
        lua.set_function("get_const", get_const);
        lua.set_function("set_const_var", set_const_var);
        lua.set_function("call_func", [](const std::string& name, float x) {
            // std::cout << "[call_func] " << name << "\n";
            return x; // identity for testing
        });
        lua.set_function("omsi_debug", [](const std::string& msg) {
            std::cout << "[omsi_debug] " << msg << "\n";
        });

        // Register misc hooks
        lua.set_function("sound_trigger", [](const std::string& name, sol::object arg) {
            // if (arg != sol::nil) {
            //     std::cout << "[sound_trigger] " << name << " arg=" << arg.as<std::string>() <<
            //     "\n";
            // } else {
            //     std::cout << "[sound_trigger] " << name << "\n";
            // }
        });

        lua.set_function("call_sys_macro", [](const std::string& name) {
            // std::cout << "[call_sys_macro] " << name << "\n";
        });

        // Register external function stubs
        lua.set_function("radiator_fan_RPM", [](float rpm) { return rpm * 0.01F; });
        lua.set_function("radiator_fan_RPM2", [](float rpm) { return rpm * 0.015F; });
        lua.set_function("radiator_fan_RPM3", [](float rpm) { return rpm * 0.02F; });
        lua.set_function("engine_turbo_RPM_factor", [](float rpm) { return rpm / 3000.0F; });
        lua.set_function("engine_turbo_throttle_factor", [](float x) { return x; });
        lua.set_function("engine_M_maxThrottle", [](float x) {
            (void)x;
            return 2200.0;
        });
        lua.set_function("engine_M_minThrottle", [](float x) {
            (void)x;
            return 700.0;
        });
        lua.set_function("engine_n_coldM", [](float x) {
            (void)x;
            return 0.0;
        });
        lua.set_function("engine_efficiency_rpm", [](float x) {
            (void)x;
            return 0.38;
        });
        lua.set_function("engine_efficiency_throttle", [](float x) {
            (void)x;
            return 0.40;
        });
        lua.set_function("kuehlwassersmoke", [](float x) {
            (void)x;
            return 0.0;
        });

        // Load the script
        auto result = lua.safe_script_file(script_path, sol::script_pass_on_error);
        if (!result.valid()) {
            sol::error err = result;
            throw std::runtime_error("Failed to load script: " + std::string(err.what()));
        }

        sol::safe_function init = lua["init"];
        sol::safe_function frame = lua["frame"];

        if (!init.valid() || !frame.valid()) {
            throw std::runtime_error("Required Lua functions not found in script");
        }

        string_stack = std::stack<std::string>();
        float_stack = std::stack<float>();
        init();

        auto execs = 0;

        auto t_start = std::chrono::high_resolution_clock::now();

        for (;;) {
            string_stack = std::stack<std::string>();
            float_stack = std::stack<float>();
            frame();

            execs++;

            auto t_end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = t_end - t_start;

            if (elapsed.count() >= 1) {
                std::cerr << "Executed " << execs << " frames in " << elapsed.count()
                          << " seconds\n";
                std::cerr << "Runs per second: " << execs / elapsed.count() << "\n";

                PROCESS_MEMORY_COUNTERS pmc;
                if (GetProcessMemoryInfo(GetCurrentProcess(), &pmc, sizeof(pmc)))
                    std::cerr << "Memory used: " << pmc.PeakWorkingSetSize / 1024 << " KB\n";

                t_start = t_end;
                execs = 0;
            }
        }

        // // Run a small scenario
        // sol::safe_function macro_engine_init = lua["macro_engine_init"];
        // sol::safe_function trigger_start = lua["trigger_kw_m_enginestart"];
        // sol::safe_function macro_engine_frame = lua["macro_engine_frame"];

        // if (!macro_engine_init.valid() || !trigger_start.valid() || !macro_engine_frame.valid())
        // {
        //     throw std::runtime_error("Required Lua functions not found in script");
        // }

        // std::cout << "Starting engine simulation...\n";

        // auto res1 = macro_engine_init();
        // if (!res1.valid()) {
        //     sol::error err = res1;
        //     throw std::runtime_error("macro_engine_init failed: " + std::string(err.what()));
        // }

        // std::cout << "Engine initialized. Starting engine...\n";

        // auto res2 = trigger_start();
        // if (!res2.valid()) {
        //     sol::error err = res2;
        //     throw std::runtime_error("trigger_kw_m_enginestart failed: " +
        //     std::string(err.what()));
        // }

        // std::cout << "Engine started. Running frames...\n";

        // for (int i = 0; i < 60 * 10; ++i) {
        //     ctx.sys_num["Timegap"] = 1.0 / 60.0;
        //     auto res = macro_engine_frame();
        //     if (!res.valid()) {
        //         sol::error err = res;
        //         throw std::runtime_error("macro_engine_frame failed: " +
        //         std::string(err.what()));
        //     }
        // }

        // std::cout << "engine_on=" << ctx.get_num(ctx.local_num, "engine_on") << "\n";
        // std::cout << "engine_n=" << ctx.get_num(ctx.local_num, "engine_n") << "\n";
        // std::cout << "engine_turbo_RPM=" << ctx.get_num(ctx.local_num, "engine_turbo_RPM") <<
        // "\n"; std::cout << "engine_tank_content=" << ctx.get_num(ctx.local_num,
        // "engine_tank_content") << "\n";

        // auto t_end = std::chrono::high_resolution_clock::now();
        // std::chrono::duration<double> elapsed = t_end - t_start;
        // std::cerr << "Elapsed time: " << elapsed.count() << " seconds\n";

        // PROCESS_MEMORY_COUNTERS pmc;
        // if (GetProcessMemoryInfo(GetCurrentProcess(), &pmc, sizeof(pmc)))
        //     std::cerr << "Memory used: " << pmc.PeakWorkingSetSize / 1024 << " KB\n";

    } catch (const std::exception& ex) {
        std::cerr << ex.what() << "\n";
        return 1;
    }

    return 0;
}
