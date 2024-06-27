#include "tracking/tracking.h"

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

	double dt = std::chrono::duration<double>(std::chrono::nanoseconds(data.timestamp) - std::chrono::nanoseconds(old_timestamp)).count();

	for (auto& [source, image_tracker] : image_trackers) {
		for (auto& e : image_tracker) {
			e.predict(dt);
		}
	}

	return GlobalTrackerResults();
}
