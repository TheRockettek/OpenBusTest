#pragma once

#include "BusTypes.h"

#include <filesystem>
#include <ostream>

struct ModelConfig;

std::filesystem::path busConfigurationPathFor(BusVehicle vehicle);
BusConfiguration loadBusConfiguration(const std::filesystem::path& configPath);
std::filesystem::path modelConfigurationPathForBus(
	const std::filesystem::path& busConfigPath);
ModelConfig loadBusModelConfiguration(const std::filesystem::path& busConfigPath);
void writeBusConfigurationJson(std::ostream& output, const BusConfiguration& configuration);
BusVehicle busVehicleFromEnvironment();
BusConfiguration busConfigurationFor(BusVehicle vehicle);