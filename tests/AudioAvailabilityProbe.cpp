#include "audio/SoundEngine.h"
#include "core/Variables.h"

#include <iostream>
#include <string_view>

int main(int argc, char** argv) {
    if (argc != 2 || (std::string_view(argv[1]) != "available" &&
                      std::string_view(argv[1]) != "unavailable")) {
        std::cerr << "Expected available or unavailable\n";
        return 1;
    }

    SoundEngine sound;
    const bool expected = std::string_view(argv[1]) == "available";
    SystemVariables variables;
    variables.setSoundAvailable(sound.isAvailable());
    if (sound.isAvailable() != expected ||
        variables.get("nosound") != (expected ? 0.0F : 1.0F)) {
        std::cerr << "Audio availability and NoSound do not match the expected backend state\n";
        return 1;
    }
    return 0;
}
