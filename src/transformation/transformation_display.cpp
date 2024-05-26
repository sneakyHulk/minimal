#include <Lane.h>
#include <LaneSection.h>
#include <OpenDriveMap.h>
#include <ecal/ecal.h>
#include <ecal/msg/flatbuffers/subscriber.h>
#include <flatbuffers/flatbuffers.h>
#include <transformation/Config.h>

#include <Eigen/Eigen>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <ranges>

#include "CompactObject_generated.h"
#include "Image_generated.h"
#include "common_output.h"
#include "eCAL_RAII.h"
#include "signal_handler.h"
#include "transformation/Config.h"
#include "transformation/EigenUtils.h"

void print_camera_view_field(cv::Mat& view, Config const& config, std::string const& camera_name, Eigen::Matrix<double, 4, 4> const& affine_transformation_base_to_image_center) {
	Eigen::Matrix<double, 4, 1> const left_bottom = affine_transformation_base_to_image_center * config.camera_config.at(camera_name).image_to_world(100., config.camera_config.at(camera_name).image_height - 100., 0.);
	Eigen::Matrix<double, 4, 1> const left_top = affine_transformation_base_to_image_center * config.camera_config.at(camera_name).image_to_world(100., 200., 0.);
	Eigen::Matrix<double, 4, 1> const right_bottom =
	    affine_transformation_base_to_image_center * config.camera_config.at(camera_name).image_to_world(config.camera_config.at(camera_name).image_width - 100., config.camera_config.at(camera_name).image_height - 100., 0.);
	Eigen::Matrix<double, 4, 1> const right_top = affine_transformation_base_to_image_center * config.camera_config.at(camera_name).image_to_world(config.camera_config.at(camera_name).image_width - 100., 200., 0.);

	std::cout << cv::Point(static_cast<int>(left_bottom(0, 0)), static_cast<int>(left_bottom(1, 0))) << std::endl;
	std::cout << cv::Point(static_cast<int>(left_top(0, 0)), static_cast<int>(left_top(1, 0))) << std::endl;
	std::cout << cv::Point(static_cast<int>(right_bottom(0, 0)), static_cast<int>(right_bottom(1, 0))) << std::endl;
	std::cout << cv::Point(static_cast<int>(right_top(0, 0)), static_cast<int>(right_top(1, 0))) << std::endl;

	cv::Scalar other_color(0, 255, 0);

	for (auto i = 0.; i <= 300.; ++i) {
		Eigen::Matrix<double, 4, 1> const right_top = affine_transformation_base_to_image_center * config.camera_config.at(camera_name).image_to_world(config.camera_config.at(camera_name).image_width, i, 0.);
		cv::circle(view, cv::Point(static_cast<int>(right_top(0, 0)), static_cast<int>(right_top(1, 0))), 1, other_color, 10);
	}

	for (auto i = 0.; i <= 300.; ++i) {
		Eigen::Matrix<double, 4, 1> const left_top = affine_transformation_base_to_image_center * config.camera_config.at(camera_name).image_to_world(0., i, 0.);
		cv::circle(view, cv::Point(static_cast<int>(left_top(0, 0)), static_cast<int>(left_top(1, 0))), 1, other_color, 10);
	}

	cv::circle(view, cv::Point(static_cast<int>(left_bottom(0, 0)), static_cast<int>(left_bottom(1, 0))), 1, other_color, 10);
	cv::circle(view, cv::Point(static_cast<int>(left_top(0, 0)), static_cast<int>(left_top(1, 0))), 1, other_color, 10);
	cv::circle(view, cv::Point(static_cast<int>(right_bottom(0, 0)), static_cast<int>(right_bottom(1, 0))), 1, other_color, 10);
	cv::circle(view, cv::Point(static_cast<int>(right_top(0, 0)), static_cast<int>(right_top(1, 0))), 1, other_color, 10);

	cv::imshow("display", view);
	cv::waitKey(0);
}

int main() {
	auto display_width = 1920;
	auto display_height = 1200;
	auto scaling = 4.;
	cv::Mat view(display_height, display_width, CV_8UC3, cv::Scalar(255, 255, 255));  // Declaring a white matrix

	Config config = make_config();

	Eigen::Matrix<double, 4, 4> affine_transformation_s110_base_to_image_center = make_matrix<4, 4>(1. * scaling, 0., 0., display_width / 2., 0., 1. * scaling, 0., display_height / 2., 0., 0., 1. * scaling, 0., 0., 0., 0., 1.);
	Eigen::Matrix<double, 4, 4> const affine_transformation_utm_to_image_center =
	    affine_transformation_s110_base_to_image_center * config.affine_transformation_map_origin_to_bases.at("s110_base") * config.affine_transformation_utm_to_map_origin;

	auto filename = std::filesystem::path(CMAKE_SOURCE_DIR) / std::filesystem::path("2021-07-07_1490_Providentia_Plus_Plus_1_6.xodr");
	odr::OpenDriveMap garching(filename);

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

					cv::line(view, cv::Point(static_cast<int>(start_transformed(0)), static_cast<int>(start_transformed(1))), cv::Point(static_cast<int>(end_transformed(0)), static_cast<int>(end_transformed(1))), black_color, 1);
				}
			}
		}
	}

	auto tmp = affine_transformation_s110_base_to_image_center * make_matrix<4, 1>(0., 0., 0., 1.);
	cv::Scalar other_color(0, 0, 255);
	cv::circle(view, cv::Point(static_cast<int>(tmp(0)), static_cast<int>(tmp(1))), 1, other_color, 5);

	print_camera_view_field(view, config, "s110_s_cam_8", affine_transformation_s110_base_to_image_center);
	// print_camera_view_field(view, config, "s110_n_cam_8", affine_transformation_s110_base_to_image_center);
	// print_camera_view_field(view, config, "s110_o_cam_8", affine_transformation_s110_base_to_image_center);
	// print_camera_view_field(view, config, "s110_w_cam_8", affine_transformation_s110_base_to_image_center);

	cv::imshow("display", view);
	cv::waitKey(0);

	return 0;
}

/*
Eigen::Vector3d getCameraOrigin() const
{
    Eigen::Matrix3d _R_cam2world = _R_world2cam;
    Eigen::Vector3d _t_cam2world_world = _t_world2cam_cam;
    utils::invert_transform(_R_cam2world, _t_cam2world_world);

Eigen::Vector3d origin = Eigen::Vector3d::Zero();
origin = _R_cam2world * origin + _t_cam2world_world;
return origin;

Eigen::Vector3d getCameraOrigin() const
       {
           Eigen::Vector3d origin = Eigen::Vector3d::Zero();
           if (_projections.empty()) return origin;

for (auto &projection : _projections) {
   origin += projection.second.getCameraOrigin() * 1/_projections.size();
}
return origin;
}
}
 */