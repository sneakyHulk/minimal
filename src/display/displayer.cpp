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

#include "CompactObject_generated.h"
#include "Image_generated.h"
#include "common_output.h"
#include "eCAL_RAII.h"
#include "signal_handler.h"

int main() {
	auto filename = std::filesystem::path(CMAKE_SOURCE_DIR) / std::filesystem::path("2021-07-07_1490_Providentia_Plus_Plus_1_6.xodr");

	odr::OpenDriveMap garching(filename);
	std::vector<odr::Vec3D> lane_pts;
	std::vector<odr::Vec3D> roadmark_pts;
	std::vector<odr::Vec3D> road_object_pts;
	std::vector<odr::Vec3D> road_signal_pts;

	const double eps = 0.1;

	cv::Mat view(1200, 1200, CV_8UC3, cv::Scalar(255, 255, 255));  // Declaring a white matrix

	for (odr::Road const& road : garching.get_roads()) {
		printf("road: %s, length: %.2f\n", road.id.c_str(), road.length);
		for (odr::LaneSection lanesection : road.get_lanesections()) {
			const double s_start = lanesection.s0;
			const double s_end = road.get_lanesection_end(lanesection);

			for (odr::Lane const& lane : lanesection.get_lanes()) {
				auto const lane_mesh = road.get_lane_mesh(lane, eps);

				for (auto [start, end] : lane_mesh.vertices | std::ranges::views::adjacent<2>) {
					cv::line(view, cv::Point(start[0] - 695150, start[1] - 5347300), cv::Point(end[0] - 695150, end[1] - 5347300), cv::Scalar(0, 0, 0), 1);
				}

			}
		}
	}
	cv::imshow("display", view);
	cv::waitKey(0);

	printf("Finished, got %llu lane points, %llu roadmark points, %llu road object points, %llu road signal points\n", lane_pts.size(), roadmark_pts.size(), road_object_pts.size(), road_signal_pts.size());

	odr::RoadNetworkMesh road_network_mesh = garching.get_road_network_mesh(eps);
	printf("Got road network mesh\n");

	// std::string str((std::istreambuf_iterator<char>(f)),
	//				 std::istreambuf_iterator<char>());
	//
	// rapidxml::xml_document<> xodr;
	// xodr.parse<rapidxml::parse_non_destructive>(str.data());
	//
	// rapidxml::xml_node<> *node = xodr.first_node("foobar");
}