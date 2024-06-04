#include <common_output.h>

#include <Eigen/Eigen>
#include <array>
#include <filesystem>
#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/viz/types.hpp>
#include <vector>

#include "transformation/Config.h"

int main() {
	Config const tmp = make_config();

	Eigen::Matrix<double, 3, 3> const intrinsic = make_matrix<3, 3>(1400.3096617691212, 0., 967.7899705163408, 0., 1403.041082755918, 581.7195041357244, 0., 0., 1.);
	Eigen::Matrix<double, 5, 1> const distortion = make_matrix<5, 1>(-0.17018194636847647, 0.12138270789030073, -0.00011663550730431874, -0.0023506235533554587, -0.030445936493878178);

	std::vector<std::vector<double>> intrinsic__ = {{1400.3096617691212, 0., 967.7899705163408}, {0., 1403.041082755918, 581.7195041357244}, {0., 0., 1.}};
	std::vector<double> intrinsic_ = {1400.3096617691212, 0., 967.7899705163408, 0., 1403.041082755918, 581.7195041357244, 0., 0., 1.};
	std::vector<double> distortion_ = {-0.17018194636847647, 0.12138270789030073, -0.00011663550730431874, -0.0023506235533554587, -0.030445936493878178};

	cv::Point2d p1(699.6090, 476.3735);
	cv::Point2d p2(1364.6891, 890.7142);

	std::vector<cv::Point2d> dis = {p1, p2};
	std::vector<cv::Point2d> out;

	cv::Point2d p3(708.5668, 477.6735);
	cv::Point2d p4(1342.6271, 883.6778);

	// [[1306.1984292822583, 0.0, 963.6965415722059], [0.0, 1300.0, 556.5225367034017], [0.0, 0.0, 1.0]]
	cv::Mat M1 = (cv::Mat_<double>(3, 3) << 1400.3096617691212, 0., 967.7899705163408, 0., 1403.041082755918, 581.7195041357244, 0., 0., 1.);

	cv::Size image_size(tmp.camera_config("s110_n_cam_8").image_width(), tmp.camera_config("s110_n_cam_8").image_height());
	cv::Mat M2 = (cv::Mat_<double>(3, 3) << 1306.1984292822583, 0.0, 963.6965415722059, 0.0, 1300.0, 556.5225367034017, 0., 0., 1.);
	cv::Mat D1 = (cv::Mat_<double>(5, 1) << -0.17018194636847647, 0.12138270789030073, -0.00011663550730431874, -0.0023506235533554587, -0.030445936493878178);
	auto alpha = 0.;

	cv::Mat intrinsic_camera_matrix_optimized = cv::getOptimalNewCameraMatrix(M1, D1, image_size, alpha, image_size);

	// cv::remap(gpu_image_mat_rgb, gpu_i mage_mat_rgb, _undistortion_map1_gpu, _undistortion_map2_gpu, cv::INTER_LINEAR);

	auto mat = cv::imread(std::filesystem::path(CMAKE_SOURCE_DIR) / std::filesystem::path("test/transformation/1690366193546_undis.jpg"));

	cv::rectangle(mat, p1, p2, cv::viz::Color::yellow());
	cv::rectangle(mat, p3, p4, cv::viz::Color::red());
	cv::undistortPoints(dis, out, M1, D1, cv::Mat_<double>::eye(3, 3), intrinsic_camera_matrix_optimized);
	cv::rectangle(mat, out[0], out[1], cv::viz::Color::blue());

	cv::Mat intrinsic_camera_matrix_optimized2 = cv::getOptimalNewCameraMatrix(M2, D1, image_size, alpha, image_size);
	cv::undistortPoints(dis, out, M2, D1, cv::Mat_<double>::eye(3, 3), intrinsic_camera_matrix_optimized2);
	cv::rectangle(mat, out[0], out[1], cv::viz::Color::bluberry());

	out = tmp.undistort_points("s110_n_cam_8", dis);
	std::cout << tmp.new_camera_matrix("s110_n_cam_8") << std::endl;
	std::cout << intrinsic_camera_matrix_optimized << std::endl;

	std::cout << "-----" << std::endl;

	std::cout << tmp.lens_config("s110_n_cam_8").camera_matrix() << std::endl;
	std::cout << M1 << std::endl;
	std::cout << tmp.lens_config("s110_n_cam_8").distortion_values() << std::endl;
	std::cout << D1 << std::endl;

	cv::rectangle(mat, out[0], out[1], cv::viz::Color::orange());

	cv::imshow("Display undis", mat);
	cv::waitKey(0);

	// common::println(tmp.camera_config.at("s110_n_cam_8").projection_matrix(0, 0), ", ", tmp.camera_config.at("s110_n_cam_8").projection_matrix(0, 2));
	// common::println(tmp.camera_config.at("s110_n_cam_8").projection_matrix(1, 0), ", ", tmp.camera_config.at("s110_n_cam_8").projection_matrix(1, 2));

	// out_[0].x = out_[0].x * tmp.camera_config.at("s110_n_cam_8").projection_matrix(0, 0) + tmp.camera_config.at("s110_n_cam_8").projection_matrix(0, 2);
	// out_[0].y = out_[0].y * tmp.camera_config.at("s110_n_cam_8").projection_matrix(1, 0) + tmp.camera_config.at("s110_n_cam_8").projection_matrix(1, 2);

	// common::println(cv::undistortImagePoints());

	return 0;
}