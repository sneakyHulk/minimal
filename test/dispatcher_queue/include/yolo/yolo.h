#pragma once

#include <torch/script.h>
#include <torch/torch.h>

#include <filesystem>

#include "msg/Detection2D.h"
#include "msg/ImageData.h"
#include "node/node.h"

class Yolo : public InputOutputNode<ImageData, Detections2D> {
	std::array<std::string, 80> classes{"person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep",
	    "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
	    "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch", "potted plant", "bed",
	    "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"};

	torch::Device device;
	torch::jit::script::Module yolo_model;

   public:
	Yolo(std::filesystem::path const& model_path = std::filesystem::path(CMAKE_SOURCE_DIR) / std::filesystem::path("data/yolo") / std::filesystem::path("yolov9c.torchscript"));

	Detections2D function(ImageData const& data) final;
};