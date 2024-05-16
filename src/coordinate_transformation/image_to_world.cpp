#include <ecal/ecal.h>
#include <ecal/msg/flatbuffers/publisher.h>
#include <ecal/msg/flatbuffers/subscriber.h>
#include <flatbuffers/flatbuffers.h>

#include <filesystem>
#include <string>
#include <thread>

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

	eCAL::flatbuffers::CSubscriber<flatbuffers::FlatBufferBuilder> detection2d_list_subscriber("detection2d_list");

	while (!signal_handler::gSignalStatus) {
		flatbuffers::FlatBufferBuilder msg;
		auto const n_bytes = detection2d_list_subscriber.Receive(msg);
		if (n_bytes) {
			auto detection2d_list(GetDetection2DList(msg.GetBufferPointer()));

			for (auto e : *detection2d_list->object()) common::println(e->bbox());



			common::println("Time taken = ", (std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count() - detection2d_list->timestamp()) / 1000000, " ms");
		}
	}

	return 0;
}