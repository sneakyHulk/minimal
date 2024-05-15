#include <ecal/ecal.h>

#include <argparse/argparse.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "common_output.h"
#include "eCAL_RAII.h"
#include "signal_handler.h"

argparse::ArgumentParser program;

void OnImage(const struct eCAL::SReceiveCallbackData* data_) {
	cv::Mat image(program.get<int>("--height"), program.get<int>("--width"), CV_8UC3, data_->buf);

	common::println("Time taken = ", eCAL::Time::GetMicroSeconds() - data_->time, " μs");
	cv::imshow("Display window", image);
	cv::waitKey(1);
}

int main(int argc, char** argv) {
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

	eCAL::CSubscriber image_subscriber("display");
	image_subscriber.AddReceiveCallback(std::bind(OnImage, std::placeholders::_2));

	while (!signal_handler::gSignalStatus) {
		// std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}

	return 0;
}