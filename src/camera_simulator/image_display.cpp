#include <ecal/ecal.h>
#include <ecal/msg/flatbuffers/publisher.h>
#include <ecal/msg/flatbuffers/subscriber.h>
#include <flatbuffers/flatbuffers.h>

#include <opencv2/opencv.hpp>
#include <thread>

#include "Image_generated.h"
#include "common_output.h"
#include "eCAL_RAII.h"
#include "signal_handler.h"

int main(int argc, char** argv) {
	eCAL_RAII ecal_raii(argc, argv);
	signal_handler();

	eCAL::flatbuffers::CSubscriber<flatbuffers::FlatBufferBuilder> image_subscriber("image");

	while (!signal_handler::gSignalStatus) {
		flatbuffers::FlatBufferBuilder msg;

		if (image_subscriber.Receive(msg)) {
			auto image(GetImage(msg.GetBufferPointer()));

			cv::Mat cv_image(image->height(), image->width(), CV_8UC3, (void*)image->mat()->data());

			common::println("Time taken = ",
			    std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::nanoseconds(std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count() - image->timestamp())));
			cv::imshow("Display window", cv_image);
			cv::waitKey(1);
		}
	}

	return 0;
}