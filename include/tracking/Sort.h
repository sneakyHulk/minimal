#pragma once
#include <common.h>

#include <Eigen/Eigen>
#include <ranges>
#include <vector>

#include "KalmanFilterTracker.h"
#include "association_functions.h"
#include "common_output.h"
#include "linear_assignment.h"

#if !__cpp_lib_ranges_enumerate
#include <range/v3/view/enumerate.hpp>
#endif

namespace tracking {
	struct Detection2D {
		BoundingBoxXYXY bbox;
		double conf;
		std::uint8_t object_class;
		constexpr Detection2D(double left, double top, double right, double bottom, double conf, std::uint8_t object_class) : bbox(left, top, right, bottom), conf(conf), object_class(object_class) {}
		constexpr Detection2D(BoundingBoxXYXY&& bbox, double conf, std::uint8_t object_class) : bbox(std::forward<decltype(bbox)>(bbox)), conf(conf), object_class(object_class) {}
	};

	template <std::size_t max_age = 4, std::size_t min_consecutive_hits = 3, double association_threshold = 0.3, auto association_function = iou>
	class Sort {
		std::vector<KalmanBoxTracker<max_age, min_consecutive_hits>> trackers;

	   public:
		Sort() = default;
		void update(double dt, std::vector<Detection2D>&& detections) {
			Eigen::MatrixXd association_matrix = Eigen::MatrixXd::Zero(detections.size(), trackers.size());
#if __cpp_lib_ranges_enumerate
			auto trackers_enumerate = std::views::enumerate(trackers);
#else
			auto trackers_enumerate = ranges::view::enumerate(trackers);
#endif
			for (auto const& [i, tracker] : trackers_enumerate) {
				auto predict_bbox = tracker.predict(dt);

#if __cpp_lib_ranges_enumerate
				auto detections_enumerate = std::views::enumerate(detections);
#else
				auto detections_enumerate = ranges::view::enumerate(detections);
#endif
				for (auto const& [j, detection] : detections_enumerate) {
					association_matrix(i, j) = association_function(predict_bbox, detection.bbox);
				}
			}

			auto const [matches, unmatched_detections, unmatched_trackers] = linear_assignment(association_matrix, association_threshold);

			common::println(", matches: ", matches, ", unmatched_detections: ", unmatched_detections, ", unmatched_trackers: ", unmatched_trackers);
		}
	};
}  // namespace tracking