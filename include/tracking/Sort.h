#pragma once
#include <common.h>

#include <Eigen/Eigen>
#include <concepts>
#include <cstdint>
#include <ranges>
#include <type_traits>
#include <vector>

#include "KalmanFilterTracker.h"
#include "association_functions.h"
#include "common_output.h"
#include "linear_assignment.h"

#if !__cpp_lib_ranges_enumerate
#include <range/v3/view/enumerate.hpp>
#endif

template <std::ranges::forward_range Range>
std::ranges::range_reference_t<Range> at(Range& range, std::ranges::range_difference_t<Range> index) {
	return *std::ranges::next(range.begin(), index);
}

namespace tracking {
	template <BoundingBoxType BoundingBox>
	class Detection2D {
		BoundingBox _bbox;
		double _conf;
		std::uint8_t _object_class;

	   public:
		constexpr Detection2D(BoundingBox const& bbox, double const conf, std::uint8_t const object_class) : _bbox(bbox), _conf(conf), _object_class(object_class) {}
		BoundingBox const& bbox() const { return _bbox; }
		[[nodiscard]] double conf() const { return _conf; }
		[[nodiscard]] std::uint8_t object_class() const { return _object_class; }
	};

	template <typename T>
	concept Detection2DType = requires(T detection) {
		{ detection.bbox() } -> BoundingBoxType;
		{ detection.conf() } -> std::convertible_to<double>;
		{ detection.object_class() } -> std::convertible_to<std::uint8_t>;
	};

	template <Detection2DType Detection2DT = Detection2D<BoundingBoxXYXY>, std::size_t max_age = 4, std::size_t min_consecutive_hits = 3, double association_threshold = 0.1, auto association_function = iou>
	class Sort {
		using BoundingBox = std::remove_cvref_t<std::invoke_result_t<decltype(&Detection2DT::bbox), Detection2DT>>;
		std::vector<KalmanBoxTracker<max_age, min_consecutive_hits, BoundingBox>> trackers;

	   public:
		Sort() = default;
		std::pair<std::vector<BoundingBox>, std::vector<BoundingBox>> update(double dt, std::ranges::viewable_range auto const& detections)
		    requires std::is_same_v<std::iter_value_t<std::ranges::iterator_t<decltype(detections)>>, Detection2DT> {
			Eigen::MatrixXd association_matrix = Eigen::MatrixXd::Zero(trackers.size(), detections.size());
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
					association_matrix(i, j) = association_function(predict_bbox, detection.bbox());
				}
			}

			common::println(association_matrix);

			auto const [matches, unmatched_trackers, unmatched_detections] = linear_assignment(association_matrix, association_threshold);

			common::println("matches: ", matches, ", unmatched_detections: ", unmatched_detections, ", unmatched_trackers: ", unmatched_trackers);

			for (auto const [tracker_index, detection_index] : matches) {
				trackers.at(tracker_index).update(at(detections, detection_index).bbox());
			}

			for (auto const detection_index : unmatched_detections) {
				trackers.emplace_back(at(detections, detection_index).bbox());
			}

			std::vector<BoundingBox> matched;
			std::vector<BoundingBox> unmatched;
			for (auto tracker = trackers.rbegin(); tracker != trackers.rend(); ++tracker) {
				if (tracker->consecutive_fails() < max_age) {
					if (tracker->consecutive_hits() >= min_consecutive_hits) {
						matched.push_back(tracker->state());
					}
					if (tracker->displayed()) {
						unmatched.push_back(tracker->state());
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