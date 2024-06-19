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
		constexpr Detection2D(double const left, double const top, double const right, double const bottom, double const conf, std::uint8_t const object_class) : bbox(left, top, right, bottom), conf(conf), object_class(object_class) {}
		constexpr Detection2D(BoundingBoxXYXY const& bbox, double const conf, std::uint8_t const object_class) : bbox(bbox), conf(conf), object_class(object_class) {}
	};

	template <std::size_t max_age = 4, std::size_t min_consecutive_hits = 3, double association_threshold = 0.9, auto association_function = iou>
	class Sort {
		std::vector<KalmanBoxTracker<max_age, min_consecutive_hits>> trackers;

	   public:
		Sort() = default;
		std::pair<std::vector<std::tuple<BoundingBoxXYXY, int>>, std::vector<std::tuple<BoundingBoxXYXY, int>>> update(double dt, std::vector<Detection2D> const& detections) {
			Eigen::MatrixXd association_matrix = Eigen::MatrixXd::Ones(trackers.size(), detections.size());
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
					association_matrix(i, j) -= association_function(predict_bbox, detection.bbox);
				}
			}

			//common::println(association_matrix);

			auto const [matches, unmatched_trackers, unmatched_detections] = linear_assignment(association_matrix, association_threshold);

			common::println("matches: ", matches, ", unmatched_detections: ", unmatched_detections, ", unmatched_trackers: ", unmatched_trackers);

			for (auto const [tracker_index, detection_index] : matches) {
				trackers.at(tracker_index).update(detections.at(detection_index).bbox);
			}

			for (auto const detection_index : unmatched_detections) {
				trackers.emplace_back(detections.at(detection_index).bbox);
			}

			std::vector<std::tuple<BoundingBoxXYXY, int>> matched;
			std::vector<std::tuple<BoundingBoxXYXY, int>> unmatched;
			for (auto tracker = trackers.rbegin(); tracker != trackers.rend(); ++tracker) {
				if (tracker->consecutive_fails() < max_age) {
					if (tracker->consecutive_hits() >= min_consecutive_hits) {
						matched.emplace_back(tracker->state(), tracker->id());
					} else if (tracker->displayed()) {
						unmatched.emplace_back(tracker->state(), tracker->id());
					}
				}
				if (tracker->consecutive_fails() > max_age) {
					trackers.erase(std::next(tracker).base());
				}
			}

			return {matched, unmatched};
		}
	};
}  // namespace tracking