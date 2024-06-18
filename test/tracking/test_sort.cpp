#include <cstdint>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "tracking/Sort.h"

int main() {
	nlohmann::json j;
	std::ifstream f(std::filesystem::path(CMAKE_SOURCE_DIR) / std::filesystem::path("test/tracking/detections_file.json"));
	f >> j;

	tracking::Sort<> tracker;

	std::int64_t timestamp_old = -1;
	for (auto frame : j["data"]) {
		if (frame["src"] != "s110_w_cam_8") continue;

		std::vector<tracking::Detection2D> detections;
		for (auto const& detection : frame["detections"]) {
			detections.emplace_back(detection["left"], detection["top"], detection["right"], detection["bottom"], detection["conf"], 1);
		}

		auto const [matched, unmatched] = tracker.update(timestamp_old > 0 ? static_cast<double>(frame["timestamp"].get<std::int64_t>() - timestamp_old) / 1000. : 0, detections);

		auto img = cv::imread(std::filesystem::path(CMAKE_SOURCE_DIR) / std::filesystem::path("data/camera_simulator/s110_w_cam_8/s110_w_cam_8_images") / std::filesystem::path(to_string(frame["timestamp"]) + ".jpg"));

		for (auto const& detection : detections) cv::rectangle(img, cv::Point2d(detection.bbox.left, detection.bbox.top), cv::Point2d(detection.bbox.right, detection.bbox.bottom), cv::Scalar_<int>(0, 0, 0), 1);
		for (auto const& bbox : matched) cv::rectangle(img, cv::Point2d(bbox.left, bbox.top), cv::Point2d(bbox.right, bbox.bottom), cv::Scalar_<int>(0, 0, 255), 5);
		for (auto const& bbox : unmatched) cv::rectangle(img, cv::Point2d(bbox.left, bbox.top), cv::Point2d(bbox.right, bbox.bottom), cv::Scalar_<int>(0, 0, 255), 1);

		cv::imshow("img", img);
		cv::waitKey(0);

		timestamp_old = frame["timestamp"];
	}

	return 0;
}