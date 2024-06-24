#pragma once

#include <filesystem>
#include <opencv2/opencv.hpp>
#include <vector>

#include "common_output.h"
#include "node/node.h"
#include "msg/ImageData.h"


class Camera : public InputNode<ImageData> {
	std::vector<std::filesystem::directory_entry> files;
	std::vector<std::filesystem::directory_entry> files_copy;

   public:
	explicit Camera(std::string const& cam_name);

   private:
	ImageData input_function() final;
};