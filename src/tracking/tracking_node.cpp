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
#include <opencv2/opencv.hpp>
#include <ranges>
#include <regex>
#include <string>
#include <thread>

#include "CompactObject_generated.h"
#include "Detection2D_generated.h"
#include "tracking/Sort.h"
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

	tracking::Sort<Detection2D> tracker;

	std::invoke_result_t<decltype(&Detection2DList::timestamp), Detection2DList> old_timestamp;
	while (!signal_handler::gSignalStatus) {
		flatbuffers::FlatBufferBuilder msg;

		if (detection2d_list_subscriber.Receive(msg)) {
			auto detection2d_list = GetDetection2DList(msg.GetBufferPointer());

			if (!detection2d_list->object()) {
				common::println("No Detections found!");
				continue;
			}

			auto detections_range = std::ranges::views::transform(*detection2d_list->object(), [](Detection2D const* e) -> Detection2D { return *e; });
			tracker.update(old_timestamp - detection2d_list->timestamp(), detections_range);

			old_timestamp = detection2d_list->timestamp();

			CompactObjectListT object_list;
			// std::vector<Detection2D> undistorted_detections_list;
			// for (Detection2D const* e : *detection2d_list->object()) {
			//	std::array<double, 3> not_implemented{std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
			//
			//	auto const [left, top] = config.undistort_point(detection2d_list->source()->str(), e->bbox().left(), e->bbox().top());
			//	auto const [right, bottom] = config.undistort_point(detection2d_list->source()->str(), e->bbox().right(), e->bbox().bottom());
			//
			//	Detection2D undistorted_detection(BoundingBox2D_XYXY(left, top, right, bottom), e->conf(), e->object_class());
			//}

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