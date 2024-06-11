#include <fstream>
#include <nlohmann/json.hpp>

#include "common_output.h"

int main(int argc, char** argv) {
	std::ifstream in(std::filesystem::path(CMAKE_SOURCE_DIR) / std::filesystem::path("test/tracking/detections_file.json"));

	nlohmann::json j;
	in >> j;

	std::vector<std::int64_t> time_yolo;
	for (auto const& e : j["data"]) {
		time_yolo.emplace_back(e["timestamp"].template get<std::int64_t>());
	}

	std::vector files(std::filesystem::recursive_directory_iterator(std::filesystem::path(CMAKE_SOURCE_DIR) / std::filesystem::path("data/camera_simulator")), {});
	files.erase(std::remove_if(files.begin(), files.end(), [](auto const& e) { return e.path().extension() != ".jpg"; }), files.end());
	files.erase(std::remove_if(files.begin(), files.end(), [](auto const& e) { return e.path().generic_string().find("_distorted") == std::string::npos; }), files.end());
	std::sort(files.begin(), files.end(), [](auto const& e1, auto const& e2) { return e1.path().stem() < e2.path().stem(); });

	std::vector<std::int64_t> time_files;
	for (auto const& entry : files) {
		time_files.emplace_back(std::stoll(entry.path().stem()));
	}

	std::vector<std::int64_t> diff;
	std::set_difference(time_files.begin(), time_files.end(), time_yolo.begin(), time_yolo.end(), std::inserter(diff, diff.begin()));
	common::println(diff);

	/*std::int64_t before = 0;
	for (auto const& e : j["data"]) {
	    auto step = e["timestamp"].template get<std::int64_t>();
	    common::println((step - before) / 1000000);
	    before = step;
	}



	common::println("------------------------------------------------");

	std::chrono::time_point<std::chrono::system_clock> before_(std::chrono::milliseconds(0));
	for (auto const& entry : files) {
	    std::chrono::time_point<std::chrono::system_clock> step_(std::chrono::milliseconds());

	    common::println((step_ - before_).count() / 1000000);

	    before_ = step_;
	}*/
}