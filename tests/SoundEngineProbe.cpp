#include "SoundEngine.h"
#include "Variables.h"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <vector>

struct SoundEngineProbeAccess {
    static const std::vector<SoundTriggerDefinition>& untriggeredLoops(const SoundEngine& engine) {
        return engine.untriggeredLoopSounds_;
    }

    static bool conditionsAllow(const SoundTriggerDefinition& definition,
                                const Variables& variables) {
        return SoundEngine::conditionsAllow(definition, variables);
    }
};

int main() {
    const std::filesystem::path root =
        std::filesystem::temp_directory_path() / "openbus_sound_engine_probe";
    std::filesystem::remove_all(root);
    std::filesystem::create_directories(root);
    const std::filesystem::path configPath = root / "sound.cfg";
    std::ofstream config(configPath);
    config << "[sound]\nWind.wav\n0.3\n"
              "[viewpoint]\n2\n"
              "[volcurve]\nVelocity\n[pnt]\n0\n0\n[pnt]\n80\n1\n"
              "[volcurve]\nCabWindowOpen\n[pnt]\n0\n0\n[pnt]\n1\n1\n"
              "[sound]\nExplicitOneShot.wav\n0.4\n[noloop]\n"
              "[volcurve]\nWindowOpen\n[pnt]\n0\n0\n[pnt]\n1\n1\n"
              "[sound]\nAssaultAlarm.wav\n1\n"
              "[volcurve]\nbatteryMasterSwitchState\n[pnt]\n0\n0\n[pnt]\n1\n1\n"
              "[conditionSingle]\nAssaultAlarm\n1\n1\n"
              "[conditionSingle]\nConditionNotEqual\n2\n0\n"
              "[conditionSingle]\nConditionLess\n2\n2\n"
              "[conditionSingle]\nConditionGreater\n0\n3\n"
              "[conditionSingle]\nConditionLessEqual\n1\n4\n"
              "[conditionSingle]\nConditionGreaterEqual\n1\n5\n";
    config.close();

    SoundEngine engine;
    engine.load(configPath);
    const std::vector<SoundTriggerDefinition>& loops =
        SoundEngineProbeAccess::untriggeredLoops(engine);
    Variables conditionVariables;
    conditionVariables.set("assaultalarm", 1.0);
    conditionVariables.set("conditionnotequal", 3.0);
    conditionVariables.set("conditionless", 1.0);
    conditionVariables.set("conditiongreater", 1.0);
    conditionVariables.set("conditionlessequal", 1.0);
    conditionVariables.set("conditiongreaterequal", 1.0);
    const bool alarmGateWorks = loops.size() == 2 && loops[1].file.filename() == "AssaultAlarm.wav" &&
                                loops[1].conditions.size() == 6 &&
                                SoundEngineProbeAccess::conditionsAllow(loops[1], conditionVariables);
    conditionVariables.set("assaultalarm", 0.0);
    const bool alarmGateInitiallyClosed =
        !SoundEngineProbeAccess::conditionsAllow(loops[1], conditionVariables);
    const bool valid = loops.size() == 2 && loops[0].loop &&
                       loops[0].file.filename() == "Wind.wav" &&
                       loops[0].viewpoint == 2 && loops[0].baseGain == 0.3 &&
                       loops[0].volumeCurves.size() == 2 &&
                       loops[0].volumeCurves[0].variable == "velocity" &&
                       loops[0].volumeCurves[1].variable == "cabwindowopen" &&
                       loops[0].volumeCurves[1].points.size() == 2 &&
                       loops[0].volumeCurves[1].points[0].x == 0.0 &&
                       loops[0].volumeCurves[1].points[0].y == 0.0 &&
                       loops[0].volumeCurves[1].points[1].x == 1.0 &&
                       loops[0].volumeCurves[1].points[1].y == 1.0 && alarmGateWorks &&
                       alarmGateInitiallyClosed;
    std::filesystem::remove_all(root);

    if (!valid) {
        std::cerr << "curve-driven ambient sounds were not retained as configured loops\n";
        return 1;
    }
    return 0;
}
