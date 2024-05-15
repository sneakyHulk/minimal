#include <opencv2/opencv.hpp>

#include "UserMatDebugAllocator.h"

int main() {
	UserMatDebugAllocator userAllocator;
	cv::Mat::setDefaultAllocator(&userAllocator);

	cv::Mat image = cv::imread("/home/lukas/src/minimal/s110_w_cam_8_images/1690366190021.jpg", cv::IMREAD_GRAYSCALE);

	common::println(&image.data);
	common::println(&image);
}