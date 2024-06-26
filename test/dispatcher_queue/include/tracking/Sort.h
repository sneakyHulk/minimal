#pragma once

#include <map>
#include <ratio>
#include <vector>

#include "linear_assignment.h"
#include "msg/ImageTrackerResult.h"
#include "tracking/KalmanBoxTracker.h"
#include "tracking/association_functions.h"

#if !__cpp_lib_ranges_enumerate
#include <range/v3/view/enumerate.hpp>
#endif

template <std::size_t max_age = 4, std::size_t min_consecutive_hits = 3, double association_threshold = 0.1, auto association_function = iou>
class Sort {
	static_assert(association_threshold < 1. && association_threshold > 0.);
	std::vector<KalmanBoxTracker<max_age, min_consecutive_hits>> trackers{};
	std::map<unsigned int, std::uint8_t> _cls;
	std::uint64_t old_timestamp = 0;

   public:
	Sort() = default;
	std::vector<ImageTrackerResult> update(std::uint64_t timestamp, std::vector<Detection2D> const& detections) {
		if (timestamp < old_timestamp) {
			trackers.clear();
		}

		double dt = std::chrono::duration<double>(std::chrono::nanoseconds(timestamp) - std::chrono::nanoseconds(old_timestamp)).count();

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

		auto const [matches, unmatched_trackers, unmatched_detections] = linear_assignment(association_matrix, 1. - association_threshold);

		// create new tracker from new not matched detections:
		for (auto const detection_index : unmatched_detections) {
			auto const& tracker = trackers.emplace_back(detections.at(detection_index).bbox);
			_cls[tracker.id()] = detections.at(detection_index).object_class;
		}

		// update old tracker with matched detections:
		for (auto const [tracker_index, detection_index] : matches) {
			trackers.at(tracker_index).update(detections.at(detection_index).bbox);
		}

		std::vector<ImageTrackerResult> matched;
		for (auto tracker = trackers.rbegin(); tracker != trackers.rend(); ++tracker) {
			if (tracker->consecutive_fails() < max_age) {
				if (tracker->consecutive_hits() >= min_consecutive_hits) {
					matched.emplace_back(tracker->state(), tracker->position(), tracker->velocity(), tracker->id(), _cls.at(tracker->id()), true);
				} else if (tracker->displayed()) {
					matched.emplace_back(tracker->state(), tracker->position(), tracker->velocity(), tracker->id(), _cls.at(tracker->id()), false);
				}
			}
			if (tracker->consecutive_fails() > max_age) {
				trackers.erase(std::next(tracker).base());
			}
		}

		old_timestamp = timestamp;

		return matched;
	}
};