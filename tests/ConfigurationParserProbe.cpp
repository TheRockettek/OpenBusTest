#include "ConfigurationParser.h"

#include <iostream>

namespace {

bool require(bool condition, const char* description) {
    if (!condition) {
        std::cerr << "failed: " << description << '\n';
        return false;
    }
    return true;
}

} // namespace

int main() {
    bool valid = true;
    int integer = 0;
    double real = 0.0;

    valid &= require(openbus::config::parseInt(" +42 ", integer) && integer == 42,
                     "integer parsing");
    valid &= require(!openbus::config::parseInt("42px", integer), "integer trailing text");
    valid &= require(openbus::config::parseDouble(" +1.25e2 ", real) && real == 125.0,
                     "floating-point parsing");
    valid &= require(!openbus::config::parseDouble("nan", real), "non-finite parsing");
    return valid ? 0 : 1;
}
