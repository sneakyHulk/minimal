#include <ecal/ecal.h>
#include <ecal/msg/flatbuffers/subscriber.h>
#include <flatbuffers/flatbuffers.h>

#include <fstream>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/viz/types.hpp>
#include <ranges>
// #include <rapidxml/rapidxml.hpp>

#include <Lane.h>
#include <LaneSection.h>
#include <OpenDriveMap.h>

#include <Eigen/Eigen>

#include "CompactObject_generated.h"
#include "Image_generated.h"
#include "common_output.h"
#include "eCAL_RAII.h"
#include "signal_handler.h"

int main() {
	auto display_width = 1920;
	auto display_height = 1200;

	Eigen::Matrix<double, 4, 4> affine_transformation_map_origin_to_utm;
	affine_transformation_map_origin_to_utm << 1., 0., 0., 695942.4856864865, 0., 1., 0., 5346521.128436302, 0., 0., 1., 485.0095881917835, 0., 0., 0., 1.;
	Eigen::Matrix<double, 4, 4> affine_transformation_utm_to_map_origin = affine_transformation_map_origin_to_utm.inverse();

	Eigen::Matrix<double, 4, 4> affine_transformation_map_origin_to_s110_base;
	affine_transformation_map_origin_to_s110_base << 0., 1., 0., -854.96568588, -1., 0., 0., -631.98486299, 0., 0., 1., 0., 0., 0., 0., 1.;
	Eigen::Matrix<double, 4, 4> affine_transformation_s110_base_to_map_origin = affine_transformation_map_origin_to_s110_base.inverse();

	Eigen::Matrix<double, 4, 4> affine_transformation_s110_base_to_image_center;
	affine_transformation_s110_base_to_image_center << 1., 0., 0., display_width / 2., 0., 1., 0., display_height / 2., 0., 0., 1., 0., 0., 0., 0., 1.;

	Eigen::Vector4d camera_east_s110_base;
	camera_east_s110_base << -15.7626, 1.45602, 7.81478, 1.;

	Eigen::Vector4d s110_base;
	s110_base << 0., 0., 0., 1.;

	Eigen::Vector4d intersection_center;
	intersection_center << 695288.632, 5347362.337, 0., 1.;

	auto test = affine_transformation_map_origin_to_utm * affine_transformation_map_origin_to_s110_base.inverse() * camera_east_s110_base;
	auto test2 = affine_transformation_map_origin_to_utm * affine_transformation_map_origin_to_s110_base.inverse() * s110_base;
	auto test3 = affine_transformation_s110_base_to_image_center * affine_transformation_map_origin_to_s110_base * affine_transformation_utm_to_map_origin * intersection_center;

	Eigen::Matrix<double, 4, 4> const affine_transformation_utm_to_image_center = affine_transformation_s110_base_to_image_center * affine_transformation_map_origin_to_s110_base * affine_transformation_utm_to_map_origin;

	std::cout << test << std::endl;
	std::cout << test2 << std::endl;
	std::cout << test3 << std::endl;

	auto filename = std::filesystem::path(CMAKE_SOURCE_DIR) / std::filesystem::path("2021-07-07_1490_Providentia_Plus_Plus_1_6.xodr");

	odr::OpenDriveMap garching(filename);
	std::vector<odr::Vec3D> lane_pts;
	std::vector<odr::Vec3D> roadmark_pts;
	std::vector<odr::Vec3D> road_object_pts;
	std::vector<odr::Vec3D> road_signal_pts;

	cv::Mat view(display_height, display_width, CV_8UC3, cv::Scalar(255, 255, 255));  // Declaring a white matrix

	for (odr::Road const& road : garching.get_roads()) {
		for (odr::LaneSection const& lanesection : road.get_lanesections()) {
			for (odr::Lane const& lane : lanesection.get_lanes()) {
				auto lane_border = road.get_lane_border_line(lane, 0.1);

				cv::Scalar black_color(0, 0, 0);

				for (auto const [start, end] : lane_border | std::ranges::views::adjacent<2>) {
					Eigen::Vector4d start_;
					start_ << start[0], start[1], start[2], 1.;
					Eigen::Vector4d end_;
					end_ << end[0], end[1], end[2], 1.;

					auto start_transformed = affine_transformation_utm_to_image_center * start_;
					auto end_transformed = affine_transformation_utm_to_image_center * end_;

					cv::line(view, cv::Point(start_transformed(0), start_transformed(1)), cv::Point(end_transformed(0), end_transformed(1)), black_color, 1);
				}
			}
		}
	}

	auto tmp = affine_transformation_s110_base_to_image_center * s110_base;
	cv::Scalar other_color(0, 0, 255);
	cv::circle(view, cv::Point(tmp(0), tmp(1)), 1, other_color, 5);

	cv::imshow("display", view);
	cv::waitKey(0);

	return 0;
}