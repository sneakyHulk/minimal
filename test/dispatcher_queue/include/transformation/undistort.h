#pragma once

#include "msg/Detection2D.h"
#include "node/node.h"
#include "transformation/Config.h"

class UndistortDetections : public InputOutputNode<Detections2D, Detections2D> {
	Config const& config;

   public:
	UndistortDetections(Config const& config);

   private:
	Detections2D function(Detections2D const& data) final;
};