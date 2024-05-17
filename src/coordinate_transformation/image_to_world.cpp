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

template <typename Scalar>
inline static Eigen::Matrix<Scalar, -1, -1> toEigenMatrix(const std::vector<std::vector<Scalar>>& vectors) {
	Eigen::Matrix<Scalar, -1, -1> M(vectors.size(), vectors.front().size());
	for (size_t i = 0; i < vectors.size(); i++)
		for (size_t j = 0; j < vectors.front().size(); j++) M(i, j) = vectors[i][j];
	return M;
}

template <typename Scalar, typename Matrix>
inline static std::vector<std::vector<Scalar>> fromEigenMatrix(const Matrix& M) {
	std::vector<std::vector<Scalar>> m;
	m.resize(M.rows(), std::vector<Scalar>(M.cols(), 0));
	for (size_t i = 0; i < m.size(); i++)
		for (size_t j = 0; j < m.front().size(); j++) m[i][j] = M(i, j);
	return m;
}

void image_to_world(std::map<std::string, nlohmann::json>& camera_config, std::string const& camera_name) {
	std::vector<std::vector<float>> const projection_matrix_stl = camera_config[camera_name]["projections"][0]["projection_matrix"].template get<std::vector<std::vector<float>>>();
	auto const projection_matrix = toEigenMatrix(projection_matrix_stl);

	auto KR = projection_matrix(Eigen::all, Eigen::last - 1);
	auto KR_inv = KR.inverse();

	auto tmp =

}

int main(int argc, char** argv) {
	eCAL_RAII ecal_raii(argc, argv);
	signal_handler();

	std::map<std::string, nlohmann::json> camera_config;

	for (std::regex const camera_config_regex("projection_([a-zA-Z0-9_]+)\\.json");
	     const auto& e : std::filesystem::directory_iterator(std::filesystem::path(CMAKE_SOURCE_DIR) / std::filesystem::path("thirdparty/projection_library/config"))) {
		std::string const filename = e.path().filename().generic_string();  // no tempory string allowed for regex_match
		if (std::smatch camera_name_match; std::regex_match(filename, camera_name_match, camera_config_regex)) {
			std::ifstream f(e.path());
			camera_config.emplace(camera_name_match[1].str(), nlohmann::json::parse(f));
		}
	}

	image_to_world(camera_config, "s110_w_cam_8");

	eCAL::flatbuffers::CSubscriber<flatbuffers::FlatBufferBuilder> detection2d_list_subscriber("detection2d_list");
	eCAL::flatbuffers::CPublisher<flatbuffers::FlatBufferBuilder> object_list_publisher("object_list");

	while (!signal_handler::gSignalStatus) {
		flatbuffers::FlatBufferBuilder msg;
		auto const n_bytes = detection2d_list_subscriber.Receive(msg);
		if (n_bytes) {
			auto detection2d_list(GetDetection2DList(msg.GetBufferPointer()));

			for (auto e : *detection2d_list->object()) {
				std::array not_implemented{std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};

				 //(e->bbox().right() + e->bbox().left() / 2)

				CompactObject object(0, e->object_class(), not_implemented, not_implemented, not_implemented, 0., 0., not_implemented, not_implemented);
				common::println(e->bbox());
			}

			common::println("Time taken = ", (std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count() - detection2d_list->timestamp()) / 1000000, " ms");
		}
	}

	return 0;
}