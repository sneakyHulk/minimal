#include

int main() {
	Config config = make_config();

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
	auto test4 = config.affine_transformation_map_origin_to_utm * config.affine_transformation_bases_to_map_origin.at("s110_base") * camera_east_s110_base;
	auto test5 = config.affine_transformation_map_origin_to_utm * config.affine_transformation_bases_to_map_origin.at("s110_base") * s110_base;
	auto test6 = affine_transformation_s110_base_to_image_center * config.affine_transformation_map_origin_to_bases.at("s110_base") * config.affine_transformation_utm_to_map_origin * intersection_center;
}