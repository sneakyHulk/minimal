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
	class [[maybe_unused]] Detection2D {
		BoundingBox _bbox;
		double _conf;
		std::uint8_t _object_class;

	   public:
		constexpr Detection2D(BoundingBox&& bbox, double&& conf, std::uint8_t&& object_class)
		    : _bbox(std::forward<decltype(bbox)>(bbox)), _conf(std::forward<decltype(conf)>(conf)), _object_class(std::forward<decltype(object_class)>(object_class)) {}
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

	template <BoundingBoxType BoundingBox>
	class [[maybe_unused]] ImageTrackerResult {
		BoundingBox _bbox;
		std::array<double, 2> _position;
		std::array<double, 2> _velocity;
		unsigned int _id;
		std::uint8_t _object_class;
		bool _matched;

	   public:
		ImageTrackerResult(BoundingBox&& bbox, std::array<double, 2>&& position, std::array<double, 2>&& velocity, unsigned int id, std::uint8_t object_class, bool matched)
		    : _bbox(std::forward<decltype(bbox)>(bbox)),
		      _position(std::forward<decltype(position)>(position)),
		      _velocity(std::forward<decltype(velocity)>(velocity)),
		      _id(std::forward<decltype(id)>(id)),
		      _object_class(std::forward<decltype(object_class)>(object_class)),
		      _matched(std::forward<decltype(matched)>(matched)) {}
		BoundingBox const& bbox() const { return _bbox; }
		[[nodiscard]] std::array<double, 2> const& position() const { return _position; }
		[[nodiscard]] std::array<double, 2> const& velocity() const { return _velocity; }
		[[nodiscard]] unsigned int id() const { return _id; }
		[[nodiscard]] std::uint8_t object_class() const { return _object_class; }
		[[nodiscard]] bool matched() const { return _matched; }
	};

	template <typename T>
	concept ImageTrackerResultType = requires(T result) {
		{ result.bbox() } -> BoundingBoxType;
		{ result.position() };
		{ result.velocity() };
		{ result.id() } -> std::convertible_to<unsigned int>;
		{ result.object_class() } -> std::convertible_to<std::uint8_t>;
		{ result.matched() } -> std::convertible_to<bool>;
	};

	template <Detection2DType Detection = Detection2D<BoundingBoxXYXY>, ImageTrackerResultType TrackerResult = ImageTrackerResult<BoundingBoxXYXY>, std::size_t max_age = 4, std::size_t min_consecutive_hits = 3,
	    double association_threshold = 0.9, auto association_function = iou>
	class Sort {
		using BoundingBox = std::remove_cvref_t<std::invoke_result_t<decltype(&Detection::bbox), Detection>>;
		std::vector<KalmanBoxTracker<max_age, min_consecutive_hits, BoundingBox>> trackers{};
		std::map<unsigned int, std::uint8_t> _cls;

	   public:
		Sort() = default;
		std::vector<TrackerResult> update(double dt, std::vector<Detection> const& detections) {
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
					association_matrix(i, j) -= association_function(predict_bbox, detection.bbox());
				}
			}

			auto const [matches, unmatched_trackers, unmatched_detections] = linear_assignment(association_matrix, association_threshold);

			// create new tracker from new not matched detections:
			for (auto const detection_index : unmatched_detections) {
				auto const& tracker = trackers.emplace_back(detections.at(detection_index).bbox());
				_cls[tracker.id()] = detections.at(detection_index).object_class();
			}

			// update old tracker with matched detections:
			for (auto const [tracker_index, detection_index] : matches) {
				trackers.at(tracker_index).update(detections.at(detection_index).bbox());
			}

			std::vector<TrackerResult> matched;
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

			return matched;
		}
	};
}  // namespace tracking