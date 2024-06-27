#pragma once

#include <cstdint>
#include <array>
#include <vector>

struct GlobalTrackerResult {
	unsigned int id;
	std::uint8_t object_class;
	bool matched;
	std::array<double, 2> position;
	std::array<double, 2> velocity;
	double heading;
};

struct GlobalTrackerResults {
	std::vector<GlobalTrackerResult> objects;
};