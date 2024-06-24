#include <common_literals.h>
#include <ecal/ecal.h>
#include <ecal/msg/flatbuffers/subscriber.h>
#include <flatbuffers/flatbuffers.h>
#include <transformation/Config.h>

#include <Eigen/Eigen>
#include <boost/circular_buffer.hpp>
#include <cstdint>
#include <map>
#include <opencv2/opencv.hpp>
#include <ranges>
#include <tuple>
#include <vector>

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

	boost::circular_buffer<std::pair<std::int64_t, std::vector<std::tuple<int, int, cv::Scalar>>>> drawing_fifo(20);

	Config config = make_config();
	auto const [map, utm_to_image] = draw_map(std::filesystem::path(CMAKE_SOURCE_DIR) / std::filesystem::path("data/visualization/2021-07-07_1490_Providentia_Plus_Plus_1_6.xodr"), config);

	cv::imshow("display", map);
	cv::waitKey(100);

	eCAL::flatbuffers::CSubscriber<flatbuffers::FlatBufferBuilder> object_list_subscriber("object_list");

	while (!signal_handler::gSignalStatus) {
		flatbuffers::FlatBufferBuilder msg;

		if (object_list_subscriber.Receive(msg)) {
			auto object_list = GetCompactObjectList(msg.GetBufferPointer());
			if (!object_list->num_objects()) continue;

			std::vector<std::tuple<int, int, cv::Scalar>> new_points;

			for (auto const e : *object_list->object()) {
				// common::println("x: ", e->position()->Get(0), ", y: ", e->position()->Get(1));
				Eigen::Vector4d const position = utm_to_image * make_matrix<4, 1>(e->position()->Get(0), e->position()->Get(1), e->position()->Get(2), 1.);

				cv::Scalar color;
				switch (e->object_class()) {
					case 0_u8: color = cv::Scalar(0, 0, 255); break;
					case 1_u8: color = cv::Scalar(0, 0, 150); break;
					case 2_u8: color = cv::Scalar(255, 0, 0); break;
					case 3_u8: color = cv::Scalar(150, 0, 0); break;
					case 5_u8: color = cv::Scalar(0, 150, 0); break;
					case 7_u8: color = cv::Scalar(0, 255, 0); break;
					default: throw;
				}

				new_points.emplace_back(static_cast<int>(position(0)), static_cast<int>(position(1)), color);
			}
			drawing_fifo.push_back(std::make_pair(object_list->timestamp(), new_points));

			auto tmp = map.clone();
			for (auto const& e : drawing_fifo | std::ranges::views::values) {
				for (auto const& [x, y, color] : e) {
					cv::circle(tmp, cv::Point(x, y), 1, color, 10);
				}
			}

			cv::imshow("display", tmp);
			cv::waitKey(10);
		}
	}

	return 0;
}