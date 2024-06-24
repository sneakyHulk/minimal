#include <ecal/ecal.h>
#include <ecal/msg/flatbuffers/publisher.h>
#include <ecal/msg/flatbuffers/subscriber.h>
#include <flatbuffers/flatbuffers.h>

#include <chrono>
#include <eigen3/Eigen/Eigen>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <ranges>
#include <regex>
#include <string>
#include <thread>

#include "CompactObject_generated.h"
#include "Detection2D_generated.h"
#include "ImageTrackerResult_generated.h"
#include "tracking/Sort.h"
#include "transformation/Config.h"
[[maybe_unused]] std::ostream& operator<<(std::ostream& stream, BoundingBoxXYXY const& bbox) {
	stream << '[' << bbox.left() << ", " << bbox.top() << ", " << bbox.right() << ", " << bbox.bottom() << ']';

	return stream;
}

#include <range/v3/view/transform.hpp>

#include "common_output.h"
#include "eCAL_RAII.h"
#include "signal_handler.h"

int main(int argc, char** argv) {
	eCAL_RAII ecal_raii(argc, argv);
	signal_handler();

	Config config = make_config();

	eCAL::flatbuffers::CSubscriber<flatbuffers::FlatBufferBuilder> detection2d_list_subscriber("detection2d_list");
	eCAL::flatbuffers::CPublisher<flatbuffers::FlatBufferBuilder> object_list_publisher("object_list");

	tracking::Sort<Detection2D, ImageTrackerResult> tracker;

	std::invoke_result_t<decltype(&Detection2DList::timestamp), Detection2DList> old_timestamp;
	while (!signal_handler::gSignalStatus) {
		flatbuffers::FlatBufferBuilder msg;

		if (detection2d_list_subscriber.Receive(msg)) {
			auto detection2d_list = GetDetection2DList(msg.GetBufferPointer());

			if (!detection2d_list->object()) {
				common::println("No Detections found!");
				continue;
			}

			auto detections_range = ranges::view::transform(*detection2d_list->object(), [&detection2d_list, &config](Detection2D const* e) -> Detection2D {
				auto const [left, top] = config.undistort_point(detection2d_list->source()->str(), e->bbox().left(), e->bbox().top());
				auto const [right, bottom] = config.undistort_point(detection2d_list->source()->str(), e->bbox().right(), e->bbox().bottom());

				return Detection2D(BoundingBoxXYXY(left, top, right, bottom), e->conf(), e->object_class());
			});

			auto const& matched =
			    tracker.update(std::chrono::duration<double, std::ratio<1>>(std::chrono::nanoseconds(old_timestamp) - std::chrono::nanoseconds(detection2d_list->timestamp())).count(), detections_range | ranges::to<std::vector>);

			old_timestamp = detection2d_list->timestamp();

			CompactObjectListT object_list;
			object_list.timestamp = detection2d_list->timestamp();
			object_list.num_objects = matched.size();

			for (auto const& result : matched) {
				std::array<double, 3> not_implemented{std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};

				Eigen::Vector4d const center_position_eigen = config.affine_transformation_map_origin_to_utm() * config.affine_transformation_bases_to_map_origin(config.camera_config(detection2d_list->source()->str()).base_name()) *
				                                              config.map_image_to_world_coordinate<double>(detection2d_list->source()->str(), result.position()->Get(0), result.position()->Get(1), 0.);

				std::array<double, 3> pos{center_position_eigen(0), center_position_eigen(1), center_position_eigen(2)};
				std::array<double, 3> vel{result.velocity()->Get(0), result.velocity()->Get(1), 0.};
				object_list.object.emplace_back(result.id(), result.object_class(), pos, not_implemented, vel, 0, 0, not_implemented, not_implemented);
			}

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