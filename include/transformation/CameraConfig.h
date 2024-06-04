#pragma once

#include <Eigen/Eigen>
#include <filesystem>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "EigenUtils.h"

struct CameraConfig {
	Eigen::Matrix<double, 3, 4> const projection_matrix;
	Eigen::Matrix<double, 3, 3> const KR;
	Eigen::Matrix<double, 3, 3> const KR_inv;
	Eigen::Matrix<double, 3, 1> const C;
	Eigen::Matrix<double, 3, 1> const translation_camera;
	Eigen::Matrix<double, 3, 3> const intrinsic;
	int const _image_width;
	int const _image_height;
	std::string const _base_name;
	std::string const _lens_name;

	CameraConfig(std::string&& base_name, std::string&& lens_name, int&& image_width, int&& image_height, Eigen::Matrix<double, 3, 4>&& projection_matrix, Eigen::Matrix<double, 3, 3>&& KR, Eigen::Matrix<double, 3, 3>&& KR_inv,
	    Eigen::Matrix<double, 3, 1>&& C, Eigen::Matrix<double, 3, 1>&& translation_camera)
	    : _base_name(std::forward<decltype(base_name)>(base_name)),
	      _lens_name(std::forward<decltype(lens_name)>(lens_name)),
	      _image_width(std::forward<decltype(image_width)>(image_width)),
	      _image_height(std::forward<decltype(image_height)>(image_height)),
	      projection_matrix(std::forward<decltype(projection_matrix)>(projection_matrix)),
	      KR(std::forward<decltype(KR)>(KR)),
	      KR_inv(std::forward<decltype(KR_inv)>(KR_inv)),
	      C(std::forward<decltype(C)>(C)),
	      translation_camera(std::forward<decltype(translation_camera)>(translation_camera)) {}

	[[nodiscard]] std::string const& base_name() const { return _base_name; }
	[[nodiscard]] std::string const& lens_name() const { return _lens_name; }
	[[nodiscard]] int const& image_width() const { return _image_width; }
	[[nodiscard]] int const& image_height() const { return _image_height; }

};

CameraConfig make_camera_config(nlohmann::json const& config, std::string lens_name) {
	std::string base_name = config["projections"][0]["extrinsic_transform"]["src"].template get<std::string>();

	int image_width = config["image_width"];
	int image_height = config["image_height"];

	// std::vector<std::vector<double>> const projection_matrix_stl = config["projections"][0]["projection_matrix"].template get<std::vector<std::vector<double>>>();
	Eigen::Matrix<double, 3, 4> projection_matrix = config["projections"][0]["projection_matrix"].template get<Eigen::Matrix<double, 3, 4>>();
	Eigen::Matrix<double, 3, 3> KR = projection_matrix(Eigen::all, Eigen::seq(0, Eigen::last - 1));
	Eigen::Matrix<double, 3, 3> KR_inv = KR.inverse();
	Eigen::Matrix<double, 3, 1> C = projection_matrix(Eigen::all, Eigen::last);
	Eigen::Matrix<double, 3, 1> translation_camera = -KR_inv * C;

	return CameraConfig{std::forward<decltype(base_name)>(base_name), std::forward<decltype(lens_name)>(lens_name), std::forward<decltype(image_width)>(image_width), std::forward<decltype(image_height)>(image_height),
	    std::forward<decltype(projection_matrix)>(projection_matrix), std::forward<decltype(KR)>(KR), std::forward<decltype(KR_inv)>(KR_inv), std::forward<decltype(C)>(C), std::forward<decltype(translation_camera)>(translation_camera)};
}
