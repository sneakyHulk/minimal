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
		if (frame["src"] != "s110_s_cam_8") continue;

		std::vector<tracking::Detection2D<tracking::BoundingBoxXYXY>> detections;
		for (auto const& detection : frame["detections"]) {
			detections.emplace_back(tracking::BoundingBoxXYXY(detection["left"], detection["top"], detection["right"], detection["bottom"]), detection["conf"], 1);
		}

		common::println("dt: ", static_cast<double>(frame["timestamp"].get<std::int64_t>() - timestamp_old) / 1000.);
		// first dt is undefined but also never used;
		auto matched = tracker.update(static_cast<double>(frame["timestamp"].get<std::int64_t>() - timestamp_old) / 1000., detections);

		auto img = cv::imread(std::filesystem::path(CMAKE_SOURCE_DIR) / std::filesystem::path("data/camera_simulator/s110_s_cam_8/s110_s_cam_8_images") / std::filesystem::path(to_string(frame["timestamp"]) + ".jpg"));

		for (auto const& detection : detections) cv::rectangle(img, cv::Point2d(detection.bbox().left(), detection.bbox().top()), cv::Point2d(detection.bbox().right(), detection.bbox().bottom()), cv::Scalar_<int>(0, 0, 0), 1);
		for (auto const& e : matched) {
			e.matched() ? cv::rectangle(img, cv::Point2d(e.bbox().left(), e.bbox().top()), cv::Point2d(e.bbox().right(), e.bbox().bottom()), cv::Scalar_<int>(0, 0, 255), 5)
			        : cv::rectangle(img, cv::Point2d(e.bbox().left(), e.bbox().top()), cv::Point2d(e.bbox().right(), e.bbox().bottom()), cv::Scalar_<int>(0, 0, 255), 1);

			cv::putText(img, std::to_string(e.id()), cv::Point2d(e.bbox().left(), e.bbox().top()), cv::FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar_<int>(0, 0, 0), 1);
		}

		cv::imshow("img", img);
		cv::waitKey(0);

		timestamp_old = frame["timestamp"];
	}

	return 0;
}