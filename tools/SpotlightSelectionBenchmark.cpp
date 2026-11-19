#include "app/vehicle/VehicleSpotlight.h"

#include <algorithm>
#include <charconv>
#include <chrono>
#include <iostream>
#include <numeric>
#include <string_view>
#include <vector>

int main(int argc, char** argv) {
    using namespace openbus::app::vehicle;
    int count = 3000;
    if (argc > 2) {
        std::cerr << "Usage: OpenBusSpotlightSelectionBenchmark [count]\n";
        return 1;
    }
    if (argc == 2) {
        const std::string_view input = argv[1];
        const auto result = std::from_chars(input.data(), input.data() + input.size(), count);
        if (result.ec != std::errc{} || result.ptr != input.data() + input.size() || count < 1 ||
            count > 100000) {
            std::cerr << "Spotlight count must be an integer from 1 to 100000\n";
            return 1;
        }
    }
    std::vector<SpotlightCandidate> source;
    source.reserve(static_cast<std::size_t>(count));
    for (int i = 0; i < count; ++i) {
        openbus::rendering::VehicleSpotlight light;
        light.position = {static_cast<double>((i * 37) % 1001 - 500),
                          static_cast<double>((i * 97) % 997 - 498),
                          static_cast<double>((i * 73) % 199 - 99)};
        source.push_back({light, i == count / 2});
    }
    double checksum = 0.0;
    std::vector<double> samples;
    samples.reserve(200);
    for (int i = 0; i < 220; ++i) {
        auto input = source;
        const auto start = std::chrono::steady_clock::now();
        const auto result = selectSpotlights(input, input.size());
        const auto end = std::chrono::steady_clock::now();
        checksum += result.front().position[0] + result.back().position[0];
        if (i >= 20)
            samples.push_back(std::chrono::duration<double, std::milli>(end - start).count());
    }
    std::sort(samples.begin(), samples.end());
    std::cout << "count=" << count
              << " mean_ms=" << std::accumulate(samples.begin(), samples.end(), 0.0) /
                                    static_cast<double>(samples.size())
              << " p95_ms=" << samples[189] << " checksum=" << checksum << '\n';
}
