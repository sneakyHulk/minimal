#include <ecal/ecal.h>
#include <ecal/msg/flatbuffers/publisher.h>
#include <ecal/msg/flatbuffers/subscriber.h>
#include <flatbuffers/flatbuffers.h>

#include <eigen3/Eigen/Eigen>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <regex>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include "Detection2D_generated.h"
#include "common_output.h"
#include "eCAL_RAII.h"
#include "signal_handler.h"
#include "transformation/Config.h"

int main(int argc, char** argv) {
	eCAL_RAII ecal_raii(argc, argv);
	signal_handler();

	Config config = make_config();

	eCAL::flatbuffers::CSubscriber<flatbuffers::FlatBufferBuilder> detection2d_list_subscriber("detection2d_list");
	eCAL::flatbuffers::CPublisher<flatbuffers::FlatBufferBuilder> object_list_publisher("object_list");

	std::vector<std::tuple<std::pair<std::string, std::int64_t>, std::pair<std::string, std::string>,
	    std::pair<std::string, std::vector<std::tuple<std::pair<std::string, double>, std::pair<std::string, double>, std::pair<std::string, double>, std::pair<std::string, double>, std::pair<std::string, double>,
	                               std::pair<std::string, double>, std::pair<std::string, double>, std::pair<std::string, double>, std::pair<std::string, double>>>>>>
	    data;

	std::int64_t timestamp = 0;
	while (!signal_handler::gSignalStatus) {
		flatbuffers::FlatBufferBuilder msg;

		if (detection2d_list_subscriber.Receive(msg)) {
			auto detection2d_list = GetDetection2DList(msg.GetBufferPointer());

			if (!detection2d_list->object()) {
				common::println("No Detections found!");
				continue;
			}

			std::vector<std::tuple<std::pair<std::string, double>, std::pair<std::string, double>, std::pair<std::string, double>, std::pair<std::string, double>, std::pair<std::string, double>, std::pair<std::string, double>,
			    std::pair<std::string, double>, std::pair<std::string, double>, std::pair<std::string, double>>>
			    detections;

			for (auto e : *detection2d_list->object()) {
				std::array<double, 3> not_implemented{std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};

				auto const [x, y] = config.undistort_point(detection2d_list->source()->str(), (e->bbox().left() + e->bbox().right()) / 2., (e->bbox().top() + e->bbox().bottom()) / 2.);
				auto const [left, top] = config.undistort_point(detection2d_list->source()->str(), e->bbox().left(), e->bbox().top());
				auto const [right, bottom] = config.undistort_point(detection2d_list->source()->str(), e->bbox().right(), e->bbox().bottom());

				Eigen::Vector4d const center_position_eigen = config.affine_transformation_map_origin_to_utm() * config.affine_transformation_bases_to_map_origin(config.camera_config(detection2d_list->source()->str()).base_name()) *
				                                              config.map_image_to_world_coordinate<double>(detection2d_list->source()->str(), x, y, 0.f);
				auto E = center_position_eigen(0);
				auto N = center_position_eigen(1);

				detections.emplace_back(std::make_pair("conf", e->conf()), std::make_pair("x", x), std::make_pair("y", y), std::make_pair("left", left), std::make_pair("top", top), std::make_pair("right", right),
				    std::make_pair("bottom", bottom), std::make_pair("E", E), std::make_pair("N", N));
			}

			if (detection2d_list->timestamp() < timestamp) common::println(detection2d_list->timestamp(), " - ", timestamp, " = ", detection2d_list->timestamp() - timestamp);
			timestamp = detection2d_list->timestamp();

			data.emplace_back(std::make_pair("timestamp", detection2d_list->timestamp()), std::make_pair("src", detection2d_list->source()->str()), std::make_pair("detections", detections));

			common::println("source: ", std::setw(15), detection2d_list->source()->str(), " took ",
			    std::chrono::duration_cast<std::chrono::milliseconds>(
			        std::chrono::nanoseconds(std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count() - detection2d_list->timestamp())));
		}
	}

	nlohmann::json out;
	out["data"] = data;

	std::ofstream o(std::filesystem::path(CMAKE_SOURCE_DIR) / std::filesystem::path("test/tracking/detections_file.json"));
	o << std::setw(4) << out << std::endl;

	return 0;
}