#include "tracking/tracking.h"

#include <Eigen/Eigen>

#include "common_literals.h"
#include "tracking/Sort.h"

NoTracking::NoTracking(Config const& config) : config(config) {}
CompactObjects NoTracking::function(Detections2D const& data) {
	CompactObjects object_list;
	object_list.timestamp = data.timestamp;

	for (auto const& e : data.objects) {
		double x_image_position = (e.bbox.left + e.bbox.right) / 2.;
		double y_image_position = e.bbox.top + (e.bbox.bottom - e.bbox.top) * (3. / 4.);

		Eigen::Vector4d const position = config.affine_transformation_map_origin_to_utm() * config.affine_transformation_bases_to_map_origin(config.camera_config(data.source).base_name()) *
		                                 config.map_image_to_world_coordinate<double>(data.source, x_image_position, y_image_position, 0.);

		object_list.objects.emplace_back(0_u8, e.object_class, std::array{position[0], position[1], position[2]}, std::array{0., 0., 0.});
	}

	return object_list;
}
SortTracking::SortTracking(const Config& config) : config(config) {}
ImageTrackerResults SortTracking::function(Detections2D const& data) {
	static thread_local std::map<std::string, Sort<>> trackers;

	if (!trackers.contains(data.source)) {
	} else {
		trackers.insert({data.source, Sort<>()});
	}

	ImageTrackerResults results;

	results.objects = trackers.at(data.source).update(data.timestamp, data.objects);
	results.source = data.source;
	results.timestamp = data.timestamp;

	return results;
}
