#pragma once

#include <map>
#include <string>

#include "msg/CompactObject.h"
#include "msg/Detection2D.h"
#include "msg/ImageTrackerResult.h"
#include "node/node.h"
#include "transformation/Config.h"

class SortTracking : public InputOutputNode<Detections2D, ImageTrackerResults> {
	Config const& config;

   public:
	explicit SortTracking(Config const& config);

   private:
	ImageTrackerResults function(Detections2D const& data) final;
};