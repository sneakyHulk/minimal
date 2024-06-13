#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <filesystem>

#include "tracker.h"

int main() {
	nlohmann::json j;
	std::ifstream f(std::filesystem::path(CMAKE_SOURCE_DIR) / std::filesystem::path("test/tracking/detections_file.json"));

	f >> j;
	j["data"];


	return 0;
}