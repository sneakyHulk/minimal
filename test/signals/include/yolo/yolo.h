#pragma once

#include <oneapi/tbb/concurrent_queue.h>

#include <memory>

#include "common_output.h"
#include "node/node.h"

class Yolo : public OutputNode<int> {
	void output_function(int const& input) final { common::println(input); }
};