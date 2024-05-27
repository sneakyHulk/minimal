#include <ecal/ecal.h>
#include <ecal/msg/flatbuffers/subscriber.h>
#include <flatbuffers/flatbuffers.h>
#include <transformation/Config.h>

#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>

#include "CompactObject_generated.h"
#include "Image_generated.h"
#include "common_output.h"
#include "eCAL_RAII.h"
#include "signal_handler.h"
#include "transformation/Config.h"
#include "transformation/DrawingUtils.h"
#include "transformation/EigenUtils.h"

int main(int argc, char** argv) {
	eCAL_RAII ecal_raii(argc, argv);
	signal_handler();

	Config config = make_config();
	auto const [map, utm_to_image] = draw_map(std::filesystem::path(CMAKE_SOURCE_DIR) / std::filesystem::path("2021-07-07_1490_Providentia_Plus_Plus_1_6.xodr"), config);

	cv::imshow("display", map);
	cv::waitKey(0);

	eCAL::flatbuffers::CSubscriber<flatbuffers::FlatBufferBuilder> object_list_subscriber("object_list");

	while (!signal_handler::gSignalStatus) {
		flatbuffers::FlatBufferBuilder msg;

		if (object_list_subscriber.Receive(msg)) {
			auto tmp = map.clone();

			auto object_list(GetCompactObjectList(msg.GetBufferPointer()));

			for (auto const e : *object_list->object()) {
				Eigen::Vector4d const position = utm_to_image * make_matrix<4, 1>(e->position()->operator[](0), e->position()->operator[](1), e->position()->operator[](2), 1.);

				cv::circle(tmp, cv::Point(static_cast<int>(position(0)), static_cast<int>(position(1))), 1, cv::Scalar(255, 0, 0), 5);
			}

			cv::imshow("display", tmp);
			cv::waitKey(10);
		}
	}

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