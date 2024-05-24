#include <ecal/ecal.h>
#include <ecal/msg/flatbuffers/publisher.h>
#include <ecal/msg/flatbuffers/subscriber.h>
#include <flatbuffers/flatbuffers.h>

#include <eigen3/Eigen/Eigen>
#include <filesystem>
#include <fstream>
#include <map>
#include <nlohmann/json.hpp>
#include <regex>
#include <string>
#include <thread>

#include "CompactObject_generated.h"
#include "Detection2D_generated.h"
[[maybe_unused]] std::ostream& operator<<(std::ostream& stream, BoundingBox2D_XYXY const& bbox) {
	stream << '[' << bbox.left() << ", " << bbox.top() << ", " << bbox.right() << ", " << bbox.bottom() << ']';

	return stream;
}

#include "common_output.h"
#include "eCAL_RAII.h"
#include "signal_handler.h"

int main(int argc, char** argv) {
	eCAL_RAII ecal_raii(argc, argv);
	signal_handler();

	std::map<std::string, CameraConfig> camera_config;

	for (std::regex const camera_config_regex("projection_([a-zA-Z0-9_]+)\\.json");
	     const auto& e : std::filesystem::directory_iterator(std::filesystem::path(CMAKE_SOURCE_DIR) / std::filesystem::path("thirdparty/projection_library/config"))) {
		std::string const filename = e.path().filename().generic_string();  // no tempory string allowed for regex_match
		if (std::smatch camera_name_match; std::regex_match(filename, camera_name_match, camera_config_regex)) {
			camera_config.emplace(camera_name_match[1].str(), e.path());
		}
	}

	eCAL::flatbuffers::CSubscriber<flatbuffers::FlatBufferBuilder> detection2d_list_subscriber("detection2d_list");
	eCAL::flatbuffers::CPublisher<flatbuffers::FlatBufferBuilder> object_list_publisher("object_list");

	while (!signal_handler::gSignalStatus) {
		flatbuffers::FlatBufferBuilder msg;
		auto const n_bytes = detection2d_list_subscriber.Receive(msg);
		if (n_bytes) {
			auto detection2d_list(GetDetection2DList(msg.GetBufferPointer()));

			for (auto e : *detection2d_list->object()) {
				std::array not_implemented{std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity()};

				auto center_position = camera_config.at(detection2d_list->source()->str()).image_to_world((e->bbox().left() + e->bbox().right()) / 2.f, (e->bbox().top() + e->bbox().bottom()) / 2.f, 0.f);

				CompactObject object(0, e->object_class(), center_position, not_implemented, not_implemented, 0., 0., not_implemented, not_implemented);
				common::print(center_position, ", ");
			}

			common::println("Time taken = ", (std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count() - detection2d_list->timestamp()) / 1000000, " ms");
		}
	}

	return 0;
}