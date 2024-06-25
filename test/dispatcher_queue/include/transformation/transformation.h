#pragma once

#include "msg/CompactObject.h"
#include "msg/Detection2D.h"
#include "msg/ImageTrackerResult.h"
#include "node/node.h"
#include "transformation/Config.h"

class DetectionTransformation : public InputOutputNode<Detections2D, CompactObjects> {
	Config const& config;

   public:
	explicit DetectionTransformation(Config const& config);

   private:
	CompactObjects function(Detections2D const& data) final;
};

class ImageTrackingTransformation : public InputOutputNode<ImageTrackerResults, CompactObjects> {
	Config const& config;

   public:
	explicit ImageTrackingTransformation(Config const& config);

   private:
	CompactObjects function(ImageTrackerResults const& data) final;
};