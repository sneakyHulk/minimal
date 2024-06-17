#pragma once
#include <tuple>

constexpr auto iou = [](std::tuple<double, double, double, double> const& bbox1, std::tuple<double, double, double, double> const& bbox2) {
	auto const xx1 = std::max(std::get<0>(bbox1), std::get<0>(bbox2));
	auto const yy1 = std::max(std::get<1>(bbox1), std::get<1>(bbox2));
	auto const xx2 = std::min(std::get<2>(bbox1), std::get<2>(bbox2));
	auto const yy2 = std::min(std::get<3>(bbox1), std::get<3>(bbox2));

	auto const w = std::max(0., xx2 - xx1);
	auto const h = std::max(0., yy2 - yy1);
	auto const wh = w * h;

	auto const A1 = (std::get<2>(bbox1) - std::get<0>(bbox1)) * (std::get<3>(bbox1) - std::get<1>(bbox1));
	auto const A2 = (std::get<2>(bbox2) - std::get<0>(bbox2)) * (std::get<3>(bbox2) - std::get<1>(bbox2));

	auto const iou = wh / (A1 + A2 - wh);
	return iou;
};

constexpr auto diou = [](std::tuple<double, double, double, double> const& bbox1, std::tuple<double, double, double, double> const& bbox2) {
	auto const iou_ = iou(bbox1, bbox2);

	auto const x_center_bbox1 = (std::get<0>(bbox1) + std::get<2>(bbox1)) / 2.;
	auto const y_center_bbox1 = (std::get<1>(bbox1) + std::get<3>(bbox1)) / 2.;
	auto const x_center_bbox2 = (std::get<0>(bbox2) + std::get<2>(bbox2)) / 2.;
	auto const y_center_bbox2 = (std::get<1>(bbox2) + std::get<3>(bbox2)) / 2.;

	auto const inner_diag_squared = (x_center_bbox1 - x_center_bbox2) * (x_center_bbox1 - x_center_bbox2) + (y_center_bbox1 - y_center_bbox2) * (y_center_bbox1 - y_center_bbox2);

	auto const xxc1 = std::min(std::get<0>(bbox1), std::get<0>(bbox2));
	auto const yyc1 = std::min(std::get<1>(bbox1), std::get<1>(bbox2));
	auto const xxc2 = std::max(std::get<2>(bbox1), std::get<2>(bbox2));
	auto const yyc2 = std::max(std::get<3>(bbox1), std::get<3>(bbox2));

	auto const outer_diag_squared = (xxc2 - xxc1) * (xxc2 - xxc1) + (yyc2 - yyc1) * (yyc2 - yyc1);

	auto const diou = iou_ - inner_diag_squared / outer_diag_squared;
	return (diou + 1) / 2.;  // resize from (-1,1) to (0,1)
};
