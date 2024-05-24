#include <Eigen/Eigen>
#include <concepts>
#include <filesystem>
#include <nlohmann/json.hpp>
#include <map>
#include <regex>
#include <span>

#include "CameraConfig.h"

struct Config {
	Eigen::Matrix<double, 4, 4> affine_transformation_map_origin_to_utm;
	std::map<std::string, CameraConfig> camera_config;

	Config(std::filesystem::path const config_directory, std::span<double, 3> const map_origin) : Config(config_directory, map_origin[0], map_origin[1], map_origin[2]) {}
	Config(std::filesystem::path const config_directory, double const map_origin_x = 695942.4856864865, double const map_origin_y = 5346521.128436302, double const map_origin_z = 485.0095881917835)
	    : affine_transformation_map_origin_to_utm(make_matrix<4, 4>(1., 0., 0., map_origin_x, 0., 1., 0., map_origin_y, 0., 0., 1., map_origin_z, 0., 0., 0., 1.)) {
		for (std::regex const camera_config_regex("projection_([a-zA-Z0-9_]+)\\.json");
		     const auto& e : std::filesystem::directory_iterator(std::filesystem::path(CMAKE_SOURCE_DIR) / std::filesystem::path("thirdparty/projection_library/config"))) {
			std::string const filename = e.path().filename().generic_string();  // no tempory string allowed for regex_match
			if (std::smatch camera_name_match; std::regex_match(filename, camera_name_match, camera_config_regex)) {
				camera_config.emplace(camera_name_match[1].str(), e.path());
			}
		}
	}
	template <std::size_t rows, std::size_t cols, std::convertible_to<double>... Values>
	static Eigen::Matrix<double, rows, cols> make_matrix(Values... args) requires(sizeof...(args) == rows * cols) {
		Eigen::Matrix<double, rows, cols> m;

		m << (args, ...);
		return m;
	}
};