#include <ecal/ecal.h>
#include <ecal/msg/flatbuffers/publisher.h>
#include <ecal/msg/publisher.h>
#include <flatbuffers/flatbuffers.h>

#include <algorithm>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>

#include "Image_generated.h"
#include "common_output.h"
#include "eCAL_RAII.h"
#include "signal_handler.h"

int main(int argc, char** argv) {
	eCAL_RAII ecal_raii(argc, argv);
	signal_handler();

	eCAL::flatbuffers::CPublisher<flatbuffers::FlatBufferBuilder> image_publisher("image");
	image_publisher.ShmEnableZeroCopy(true);

	std::vector files(std::filesystem::recursive_directory_iterator(std::filesystem::path(CMAKE_SOURCE_DIR) / std::filesystem::path("data/camera_simulator")), {});
	files.erase(std::remove_if(files.begin(), files.end(), [](auto const& e) { return e.path().extension() != ".jpg"; }), files.end());
	files.erase(std::remove_if(files.begin(), files.end(), [](auto const& e) { return e.path().generic_string().find("_distorted") == std::string::npos; }), files.end());
	common::println("Dealing with ", files.size(), " files!");

	std::sort(files.begin(), files.end(), [](auto const& e1, auto const& e2) { return e1.path().stem() < e2.path().stem(); });
	// while (!signal_handler::gSignalStatus) {
	for (auto const& entry : files) {
		cv::Mat image_read = cv::imread(entry.path(), cv::IMREAD_COLOR);
		std::chrono::time_point<std::chrono::system_clock> next(std::chrono::milliseconds(std::stoll(entry.path().stem())));  // image name in microseconds
		static auto images_time = next;
		static auto current_time = std::chrono::system_clock::now();

		std::this_thread::sleep_until(current_time + (next - images_time));

		ImageT image;
		image.timestamp = std::stoll(entry.path().stem());  // std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count();
		image.width = image_read.cols;
		image.height = image_read.rows;
		image.source = entry.path().parent_path().parent_path().filename();
		image.mat = {image_read.data, image_read.data + (image_read.total() * image_read.elemSize())};

		flatbuffers::FlatBufferBuilder builder;
		builder.Finish(Image::Pack(builder, &image));

		image_publisher.Send(builder, -1);
		if (signal_handler::gSignalStatus) break;
	}
	//}

	return 0;
}