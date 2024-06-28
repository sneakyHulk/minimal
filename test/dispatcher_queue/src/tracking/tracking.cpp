#include "tracking/tracking.h"

#include <Eigen/Eigen>

SortTracking::SortTracking(const Config& config) : config(config) {}
ImageTrackerResults SortTracking::function(Detections2D const& data) {
	// static thread_local std::map<std::string, Sort<>> trackers;  // will work for now because all Nodes are running on a different thread

	if (!trackers.contains(data.source)) {
		trackers.insert({data.source, Sort<>()});
	}

	ImageTrackerResults results;

	results.objects = trackers.at(data.source).update(data.timestamp, data.objects);
	results.source = data.source;
	results.timestamp = data.timestamp;

	return results;
}
GlobalTracking::GlobalTracking(Config const& config) : config(config) {}
GlobalTrackerResults GlobalTracking::function(Detections2D const& data) {
	if (!image_trackers.contains(data.source)) {
		image_trackers.insert({data.source, {}});
	}

	double const dt = std::chrono::duration<double>(std::chrono::nanoseconds(data.timestamp) - std::chrono::nanoseconds(old_timestamp)).count();

	for (auto& [source, image_tracker] : image_trackers) {
		for (auto& track : image_tracker) {
			track.predict(dt);
		}
	}

	auto& image_tracker = image_trackers[data.source];

	Eigen::MatrixXd association_matrix = Eigen::MatrixXd::Ones(image_tracker.size(), data.objects.size());
	for (auto i = 0; i < image_tracker.size(); ++i) {
		for (auto j = 0; j < data.objects.size(); ++j) {
			association_matrix(i, j) -= association_function(image_tracker[i].state(), data.objects[j].bbox);
		}
	}
	auto const [matches, unmatched_tracks, unmatched_detections] = linear_assignment(association_matrix, 1. - association_threshold);

	// create new tracker from new not matched detections:
	for (auto const detection_index : unmatched_detections) {
		image_tracker.emplace_back(data.objects[detection_index].bbox, data.objects[detection_index].object_class);
	}

	// update old tracker with matched detections:
	for (auto const [tracker_index, detection_index] : matches) {
		image_tracker[tracker_index].update(data.objects[detection_index].bbox);
	}

	std::erase_if(image_tracker, [this](KalmanBoxTracker const& track) { return track.consecutive_fails() > max_age; });



	return GlobalTrackerResults();
}
