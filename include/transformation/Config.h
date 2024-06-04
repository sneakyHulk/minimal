#pragma once
#include <Eigen/Eigen>
#include <concepts>
#include <filesystem>
#include <fstream>
#include <map>
#include <nlohmann/json.hpp>
#include <regex>
#include <span>
#include <utility>

#include "CameraConfig.h"
#include "EigenUtils.h"
#include "LensConfig.h"

struct Config {
	Eigen::Matrix<double, 4, 4> const _affine_transformation_map_origin_to_utm;
	Eigen::Matrix<double, 4, 4> const _affine_transformation_utm_to_map_origin;
	std::map<std::string, Eigen::Matrix<double, 4, 4>> const _affine_transformation_map_origin_to_bases;
	std::map<std::string, Eigen::Matrix<double, 4, 4>> const _affine_transformation_bases_to_map_origin;

	std::map<std::string, LensConfig> const _lens_config;
	std::map<std::tuple<std::string, int, int>, cv::Mat> const _new_camera_matrix;

	std::map<std::string, CameraConfig> const _camera_config;

	Config(Eigen::Matrix<double, 4, 4>&& affine_transformation_map_origin_to_utm, Eigen::Matrix<double, 4, 4>&& affine_transformation_utm_to_map_origin,
	    std::map<std::string, Eigen::Matrix<double, 4, 4>>&& affine_transformation_map_origin_to_bases, std::map<std::string, Eigen::Matrix<double, 4, 4>>&& affine_transformation_bases_to_map_origin,
	    std::map<std::string, CameraConfig>&& camera_config, std::map<std::string, LensConfig>&& lens_config, std::map<std::tuple<std::string, int, int>, cv::Mat>&& new_camera_matrix)
	    : _affine_transformation_map_origin_to_utm(std::forward<decltype(affine_transformation_map_origin_to_utm)>(affine_transformation_map_origin_to_utm)),
	      _affine_transformation_utm_to_map_origin(std::forward<decltype(affine_transformation_utm_to_map_origin)>(affine_transformation_utm_to_map_origin)),
	      _affine_transformation_map_origin_to_bases(std::forward<decltype(affine_transformation_map_origin_to_bases)>(affine_transformation_map_origin_to_bases)),
	      _affine_transformation_bases_to_map_origin(std::forward<decltype(affine_transformation_bases_to_map_origin)>(affine_transformation_bases_to_map_origin)),
	      _camera_config(std::forward<decltype(camera_config)>(camera_config)),
	      _lens_config(std::forward<decltype(lens_config)>(lens_config)),
	      _new_camera_matrix(std::forward<decltype(new_camera_matrix)>(new_camera_matrix)) {}

	template <typename scalar, int... other>
	[[nodiscard]] Eigen::Matrix<scalar, 4, 1, other...> map_image_to_world_coordinate(std::string const& camera_name, scalar x, scalar y, scalar height) const {
		Eigen::Matrix<scalar, 4, 1, other...> image_coordinates = Eigen::Matrix<scalar, 4, 1, other...>::Ones();
		image_coordinates(0) = x;
		image_coordinates(1) = y;

		image_coordinates.template head<3>() = camera_config(camera_name).KR_inv * image_coordinates.template head<3>();
		image_coordinates.template head<3>() = image_coordinates.template head<3>() * (height - camera_config(camera_name).translation_camera(2)) / image_coordinates(2);
		image_coordinates.template head<3>() = image_coordinates.template head<3>() + camera_config(camera_name).translation_camera;

		return image_coordinates;
	}

	[[nodiscard]] Eigen::Matrix<double, 4, 4> const& affine_transformation_map_origin_to_utm() const { return _affine_transformation_map_origin_to_utm; }
	[[nodiscard]] Eigen::Matrix<double, 4, 4> const& affine_transformation_utm_to_map_origin() const { return _affine_transformation_utm_to_map_origin; }
	[[nodiscard]] Eigen::Matrix<double, 4, 4> const& affine_transformation_map_origin_to_bases(std::string const& camera_name) const { return _affine_transformation_map_origin_to_bases.at(camera_name); }
	[[nodiscard]] Eigen::Matrix<double, 4, 4> const& affine_transformation_bases_to_map_origin(std::string const& camera_name) const { return _affine_transformation_bases_to_map_origin.at(camera_name); }

	[[nodiscard]] LensConfig const& lens_config(std::string const& camera_name) const { return _lens_config.at(camera_config(camera_name).lens_name()); }
	[[nodiscard]] CameraConfig const& camera_config(std::string const& camera_name) const { return _camera_config.at(camera_name); }
	[[nodiscard]] cv::Mat const& new_camera_matrix(std::string const& camera_name) const {
		return _new_camera_matrix.at(std::tie(camera_config(camera_name).lens_name(), camera_config(camera_name).image_width(), camera_config(camera_name).image_height()));
	}
	[[nodiscard]] std::vector<cv::Point2d> undistort_points(std::string const& camera_name, std::vector<cv::Point2d> const& distorted_points) const {
		std::vector<cv::Point2d> out;

		cv::undistortPoints(distorted_points, out, lens_config(camera_name).camera_matrix(), lens_config(camera_name).distortion_values(), cv::Mat_<double>::eye(3, 3), new_camera_matrix(camera_name));

		return out;
	}
};

Config make_config(std::filesystem::path const config_directory = std::filesystem::path(CMAKE_SOURCE_DIR) / std::filesystem::path("thirdparty/projection_library/config"), double const map_origin_x = 695942.4856864865,
    double const map_origin_y = 5346521.128436302, double const map_origin_z = 485.0095881917835) {
	Eigen::Matrix<double, 4, 4> affine_transformation_map_origin_to_utm = make_matrix<4, 4>(1., 0., 0., map_origin_x, 0., 1., 0., map_origin_y, 0., 0., 1., map_origin_z, 0., 0., 0., 1.);
	Eigen::Matrix<double, 4, 4> affine_transformation_utm_to_map_origin = affine_transformation_map_origin_to_utm.inverse();

	std::map<std::string, Eigen::Matrix<double, 4, 4>> affine_transformation_map_origin_to_bases;
	std::map<std::string, Eigen::Matrix<double, 4, 4>> affine_transformation_bases_to_map_origin;

	{
		Eigen::Matrix<double, 4, 4> affine_transformation_map_origin_to_s110_base = make_matrix<4, 4>(0., 1., 0., -854.96568588, -1., 0., 0., -631.98486299, 0., 0., 1., 0., 0., 0., 0., 1.);
		affine_transformation_map_origin_to_bases["s110_base"] = affine_transformation_map_origin_to_s110_base;
		Eigen::Matrix<double, 4, 4> affine_transformation_s110_base_to_map_origin = affine_transformation_map_origin_to_s110_base.inverse();
		affine_transformation_bases_to_map_origin["s110_base"] = affine_transformation_s110_base_to_map_origin;

		Eigen::Matrix<double, 4, 4> affine_transformation_map_origin_to_road_s50_base = make_matrix<4, 4>(-0.28401534, -0.95881973, 0., 1.69503701, 0.95881973, -0.28401534, 0., -27.2150203, 0., 0., 1., 0., 0., 0., 0., 1.);
		affine_transformation_map_origin_to_bases["road_s50_base"] = affine_transformation_map_origin_to_road_s50_base;
		Eigen::Matrix<double, 4, 4> affine_transformation_road_s50_base_to_map_origin = affine_transformation_map_origin_to_road_s50_base.inverse();
		affine_transformation_bases_to_map_origin["road_s50_base"] = affine_transformation_road_s50_base_to_map_origin;

		Eigen::Matrix<double, 4, 4> affine_transformation_map_origin_to_road = make_matrix<4, 4>(-0.28401534, -0.95881973, 0., -7.67, 0.95881973, -0.28401534, 0., -25.89, 0., 0., 1., 0., 0., 0., 0., 1.);
		affine_transformation_map_origin_to_bases["road"] = affine_transformation_map_origin_to_road;
		Eigen::Matrix<double, 4, 4> affine_transformation_road_to_map_origin = affine_transformation_map_origin_to_road.inverse();
		affine_transformation_bases_to_map_origin["road"] = affine_transformation_road_to_map_origin;
	}

	std::map<std::string, LensConfig> lens_configs;

	for (std::regex const lens_config_regex("intrinsic_camera_parameters_([0-9]+)_mm_lens.txt"); const auto& e : std::filesystem::directory_iterator(config_directory / std::filesystem::path("distortion"))) {
		std::string const filename = e.path().filename().generic_string();  // no temporary string allowed for regex_match

		if (std::smatch lens_name_match; std::regex_match(filename, lens_name_match, lens_config_regex)) {
			std::ifstream f(e.path().string());

			lens_configs.emplace(lens_name_match[1].str(), make_lens_config(f));
		}
	}

	std::map<std::string, CameraConfig> camera_configs;
	std::map<std::string, std::tuple<std::string, int, int>> lens_mappings;

	for (std::regex const camera_config_regex("projection_([a-zA-Z0-9_]+?([0-9]+))\\.json"); const auto& e : std::filesystem::directory_iterator(config_directory)) {
		std::string const filename = e.path().filename().generic_string();  // no temporary string allowed for regex_match

		if (std::smatch camera_name_match; std::regex_match(filename, camera_name_match, camera_config_regex)) {
			std::ifstream f(e.path().string());
			nlohmann::json const config = nlohmann::json::parse(f);

			camera_configs.emplace(camera_name_match[1].str(), make_camera_config(config, camera_name_match[2].str()));
		}
	}

	std::map<std::tuple<std::string, int, int>, cv::Mat> new_camera_matrix;

	for (auto const& [camera_name, camera_config] : camera_configs) {
		auto const lens_mapping = std::tie(camera_config.lens_name(), camera_config.image_width(), camera_config.image_height());
		if (new_camera_matrix.contains(lens_mapping)) continue;
		if (!lens_configs.contains(camera_config.lens_name())) continue;

		cv::Size image_size(camera_config.image_width(), camera_config.image_height());
		cv::Mat intrinsic_camera_matrix_optimized = cv::getOptimalNewCameraMatrix(lens_configs.at(camera_config.lens_name()).camera_matrix(), lens_configs.at(camera_config.lens_name()).distortion_values(), image_size, 0., image_size);

		new_camera_matrix.emplace(lens_mapping, intrinsic_camera_matrix_optimized);
	}

	return Config{std::forward<decltype(affine_transformation_map_origin_to_utm)>(affine_transformation_map_origin_to_utm), std::forward<decltype(affine_transformation_utm_to_map_origin)>(affine_transformation_utm_to_map_origin),
	    std::forward<decltype(affine_transformation_map_origin_to_bases)>(affine_transformation_map_origin_to_bases), std::forward<decltype(affine_transformation_bases_to_map_origin)>(affine_transformation_bases_to_map_origin),
	    std::forward<decltype(camera_configs)>(camera_configs), std::forward<decltype(lens_configs)>(lens_configs), std::forward<decltype(new_camera_matrix)>(new_camera_matrix)};
}
