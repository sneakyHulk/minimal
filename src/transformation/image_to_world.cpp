#include <ecal/ecal.h>
#include <ecal/msg/flatbuffers/publisher.h>
#include <ecal/msg/flatbuffers/subscriber.h>
#include <flatbuffers/flatbuffers.h>

#include <eigen3/Eigen/Eigen>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <nlohmann/json.hpp>
#include <regex>
#include <string>
#include <thread>

#include "CompactObject_generated.h"
#include "Detection2D_generated.h"
#include "transformation/Config.h"
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

	Config config = make_config();

	eCAL::flatbuffers::CSubscriber<flatbuffers::FlatBufferBuilder> detection2d_list_subscriber("detection2d_list");
	eCAL::flatbuffers::CPublisher<flatbuffers::FlatBufferBuilder> object_list_publisher("object_list");

	while (!signal_handler::gSignalStatus) {
		flatbuffers::FlatBufferBuilder msg;

		if (detection2d_list_subscriber.Receive(msg)) {
			auto detection2d_list = GetDetection2DList(msg.GetBufferPointer());

			if (!detection2d_list->object()) {
				common::println("No Detections found!");
				continue;
			}

			CompactObjectListT object_list;
			for (auto e : *detection2d_list->object()) {
				std::array<double, 3> not_implemented{std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};

				Eigen::Vector4d const center_position_eigen = config.affine_transformation_map_origin_to_utm * config.affine_transformation_bases_to_map_origin.at(config.camera_config.at(detection2d_list->source()->str()).base_name) *
				                                              config.camera_config.at(detection2d_list->source()->str()).image_to_world((e->bbox().left() + e->bbox().right()) / 2.f, (e->bbox().top() + e->bbox().bottom()) / 2.f, 0.f);
				std::array<double, 3> center_position = {center_position_eigen(0), center_position_eigen(1), center_position_eigen(2)};

				object_list.object.emplace_back(0, e->object_class(), center_position, not_implemented, not_implemented, 0., 0., not_implemented, not_implemented);
			}

			object_list.timestamp = detection2d_list->timestamp();
			object_list.num_objects = detection2d_list->num_objects();

			flatbuffers::FlatBufferBuilder builder;
			builder.Finish(CompactObjectList::Pack(builder, &object_list));

			object_list_publisher.Send(builder, -1);

			common::println("source: ", std::setw(15), detection2d_list->source()->str(), " took ",
			    std::chrono::duration_cast<std::chrono::milliseconds>(
			        std::chrono::nanoseconds(std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count() - detection2d_list->timestamp())));
		}
	}

	return 0;
}