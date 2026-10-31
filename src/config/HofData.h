#pragma once

#include <string>
#include <vector>

namespace openbus::config {

struct HofTerminus {
    int code = 0;
    std::vector<std::string> strings;
};

struct HofBusstop {
    std::string ident;
    std::vector<std::string> strings;
};

struct HofRoute {
    std::string code;
    int terminusIndex = 0;
    bool hasTerminusIndex = false;
    std::vector<std::string> busstopIdents;
    std::string name;
    std::string line;
};

struct HofData {
    std::vector<HofTerminus> termini;
    std::vector<HofBusstop> busstops;
    std::vector<HofRoute> routes;
    std::vector<std::string> globalStrings;
};

} // namespace openbus::config
