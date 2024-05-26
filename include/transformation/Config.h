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

struct Config {
	Eigen::Matrix<double, 4, 4> const affine_transformation_map_origin_to_utm;
	Eigen::Matrix<double, 4, 4> const affine_transformation_utm_to_map_origin;
	std::map<std::string, Eigen::Matrix<double, 4, 4>> const affine_transformation_map_origin_to_bases;
	std::map<std::string, Eigen::Matrix<double, 4, 4>> const affine_transformation_bases_to_map_origin;

	std::map<std::string, CameraConfig> const camera_config;

	Config(Eigen::Matrix<double, 4, 4>&& affine_transformation_map_origin_to_utm, Eigen::Matrix<double, 4, 4>&& affine_transformation_utm_to_map_origin,
	    std::map<std::string, Eigen::Matrix<double, 4, 4>>&& affine_transformation_map_origin_to_bases, std::map<std::string, Eigen::Matrix<double, 4, 4>>&& affine_transformation_bases_to_map_origin,
	    std::map<std::string, CameraConfig>&& camera_config)
	    : affine_transformation_map_origin_to_utm(std::forward<decltype(affine_transformation_map_origin_to_utm)>(affine_transformation_map_origin_to_utm)),
	      affine_transformation_utm_to_map_origin(std::forward<decltype(affine_transformation_utm_to_map_origin)>(affine_transformation_utm_to_map_origin)),
	      affine_transformation_map_origin_to_bases(std::forward<decltype(affine_transformation_map_origin_to_bases)>(affine_transformation_map_origin_to_bases)),
	      affine_transformation_bases_to_map_origin(std::forward<decltype(affine_transformation_bases_to_map_origin)>(affine_transformation_bases_to_map_origin)),
	      camera_config(std::forward<decltype(camera_config)>(camera_config)) {}
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

	std::map<std::string, CameraConfig> camera_config;

	for (std::regex const camera_config_regex("projection_([a-zA-Z0-9_]+)\\.json");
	     const auto& e : std::filesystem::directory_iterator(std::filesystem::path(CMAKE_SOURCE_DIR) / std::filesystem::path("thirdparty/projection_library/config"))) {
		std::string const filename = e.path().filename().generic_string();  // no temporary string allowed for regex_match

		if (std::smatch camera_name_match; std::regex_match(filename, camera_name_match, camera_config_regex)) {
			std::ifstream f(e.path().string());
			nlohmann::json const config = nlohmann::json::parse(f);

			camera_config.emplace(camera_name_match[1].str(), make_camera_config(config));
		}
	}

	return Config{std::forward<decltype(affine_transformation_map_origin_to_utm)>(affine_transformation_map_origin_to_utm), std::forward<decltype(affine_transformation_utm_to_map_origin)>(affine_transformation_utm_to_map_origin),
	    std::forward<decltype(affine_transformation_map_origin_to_bases)>(affine_transformation_map_origin_to_bases), std::forward<decltype(affine_transformation_bases_to_map_origin)>(affine_transformation_bases_to_map_origin),
	    std::forward<decltype(camera_config)>(camera_config)};
}
