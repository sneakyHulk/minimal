#pragma once

#include <map>
#include <string>

#include "msg/CompactObject.h"
#include "msg/Detection2D.h"
#include "msg/ImageTrackerResult.h"
#include "msg/GlobalTrackerResult.h"
#include "node/node.h"
#include "tracking/Sort.h"
#include "transformation/Config.h"

class SortTracking : public InputOutputNode<Detections2D, ImageTrackerResults> {
	Config const& config;
	std::map<std::string, Sort<>> trackers;

   public:
	explicit SortTracking(Config const& config);

   private:
	ImageTrackerResults function(Detections2D const& data) final;
};

class GlobalTracking : public InputOutputNode<Detections2D, GlobalTrackerResults> {
	Config const& config;
	std::map<std::string, std::vector<KalmanBoxTracker>> image_trackers;
	std::uint64_t old_timestamp = 0;

   public:
	explicit GlobalTracking(Config const& config);

   private:
	GlobalTrackerResults function(Detections2D const& data) final;
};