#pragma once

#include <oneapi/tbb/concurrent_queue.h>

#include <chrono>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <queue>
#include <thread>
#include <vector>

#include "common_output.h"
#include "node/node.h"

using namespace std::chrono_literals;

struct

class Camera : public InputNode<int> {
	std::vector<std::filesystem::directory_entry> files;
	std::vector<std::filesystem::directory_entry> files_copy;

	Camera(std::string const& cam_name) : files(std::filesystem::recursive_directory_iterator(std::filesystem::path(CMAKE_SOURCE_DIR) / std::filesystem::path("data/camera_simulator") / std::filesystem::path(cam_name)), {}) {
		files.erase(std::remove_if(files.begin(), files.end(), [](auto const& e) { return e.path().extension() != ".jpg"; }), files.end());
		files.erase(std::remove_if(files.begin(), files.end(), [](auto const& e) { return e.path().generic_string().find("_distorted") == std::string::npos; }), files.end());

		std::sort(files.begin(), files.end(), [](auto const& e1, auto const& e2) { return e1.path().stem() > e2.path().stem(); });
		files_copy = files;
		common::println("Dealing with ", files.size(), " files!");
	}
	int input_function() final {
		while (files.empty()) {
			common::println("No files for simulation found! Starting again.");
			files = files_copy;
			std::this_thread::sleep_for(1s);
		}

		auto current_file = files.back();
		files.pop_back();

		cv::Mat image_read = cv::imread(current_file.path(), cv::IMREAD_COLOR);
		std::chrono::time_point<std::chrono::system_clock> next(std::chrono::milliseconds(std::stoll(current_file.path().stem())));  // image name in microseconds
		static auto images_time = next;
		static auto current_time = std::chrono::system_clock::now();

		std::this_thread::sleep_until(current_time + (next - images_time));


	}
};