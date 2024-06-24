#pragma once

#include <oneapi/tbb/concurrent_queue.h>

#include <boost/signals2.hpp>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <thread>

#include "node/node.h"

using namespace std::chrono_literals;

// class Camera {
//	using OnImage = boost::signals2::signal<void(std::shared_ptr<int> const)>;
//	OnImage _image_signal;
//
//    public:
//	Camera() {}
//	boost::signals2::connection on_image(OnImage::slot_type const& subscriber) { return _image_signal.connect(subscriber); }
//	[[noreturn]] void operator()() {
//		auto i = 0;
//		while (true) {
//			_image_signal(std::make_shared<int>(i++));
//			std::this_thread::sleep_for(1s);
//		}
//	}
// };

class Camera : public InputNode<int> {
	int i = 0;

	int input_function() final {
		std::this_thread::sleep_for(1s);
		return i++;
	}
};