#include <ecal/ecal.h>
#include <ecal/msg/publisher.h>

#include <argparse/argparse.hpp>
#include <opencv2/opencv.hpp>
#include <ranges>
#include <string>
#include <thread>

#include "common_output.h"
#include "eCAL_RAII.h"
#include "signal_handler.h"

int main(int argc, char** argv) {
	argparse::ArgumentParser program;
	program.add_argument("--height").help("set height of simulated images").default_value(1200);
	program.add_argument("--width").help("set width of simulated images").default_value(1920);

	try {
		program.parse_args(argc, argv);
	} catch (const std::exception& err) {
		std::cerr << err.what() << std::endl;
		std::cerr << program;
		std::exit(1);
	}

	eCAL_RAII ecal_raii(argc, argv);
	signal_handler();

	eCAL::CPublisher image_publisher("image");
	image_publisher.ShmEnableZeroCopy(true);

	std::vector files(std::filesystem::directory_iterator("/home/lukas/src/minimal/s110_w_cam_8_images"), {});
	std::sort(files.begin(), files.end());
	for (auto const& entry : files) {
		cv::Mat image_read = cv::imread(entry.path(), cv::IMREAD_COLOR);
		std::chrono::time_point<std::chrono::system_clock> next(std::chrono::milliseconds(std::stoll(entry.path().stem())));  // image name in microseconds

		static auto images_time = next;
		static auto current_time = std::chrono::system_clock::now();

		std::this_thread::sleep_until(current_time + (next - images_time));

		cv::Mat image(program.get<int>("--height"), program.get<int>("--width"), CV_8UC3, image_read.data);

		image_publisher.Send(image.data, image.total() * image.elemSize());
		if (signal_handler::gSignalStatus) break;
	}

	return 0;
}