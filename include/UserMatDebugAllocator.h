#include <iostream>
#include <opencv2/opencv.hpp>

#include "common_output.h"

class UserMatDebugAllocator : public cv::MatAllocator {
   public:
	UserMatDebugAllocator() {
		m_defaultAllocator = cv::Mat::getDefaultAllocator();
		m_stdAllocator = cv::Mat::getStdAllocator();
		std::cout << "UserMatDebugAllocator: constructor" << std::endl;
	}

	~UserMatDebugAllocator() override { std::cout << "UserMatDebugAllocator: destructor" << std::endl; }

	cv::UMatData* allocate(int dims, const int* sizes, int type, void* data, size_t* step, cv::AccessFlag flags, cv::UMatUsageFlags usageFlags) const override {
		std::cout << "UserMatDebugAllocator: allocate" << std::endl;

		cv::UMatData* ret = m_defaultAllocator->allocate(dims, sizes, type, data, step, flags, usageFlags);
		common::println(ret);
		common::println(&ret->originalUMatData);


		if (nullptr != ret) {
			// the following line is what causes the unmap() to reach this class as well
			ret->currAllocator = this;
		}
		return ret;
	}

	bool allocate(cv::UMatData* data, cv::AccessFlag accessflags, cv::UMatUsageFlags usageFlags) const override {
		std::cout << "UserMatDebugAllocator: allocate2" << std::endl;
		return m_defaultAllocator->allocate(data, accessflags, usageFlags);
	}

	void deallocate(cv::UMatData* data) const override {
		std::cout << "UserMatDebugAllocator: deallocate" << std::endl;
		return m_defaultAllocator->deallocate(data);
	}

	void map(cv::UMatData* data, cv::AccessFlag accessflags) const override {
		std::cout << "UserMatDebugAllocator: map" << std::endl;
		return m_defaultAllocator->map(data, accessflags);
	}

	void unmap(cv::UMatData* data) const override {
		std::cout << "UserMatDebugAllocator: unmap" << std::endl;
		if ((data->urefcount == 0) && (data->refcount == 0)) {
			deallocate(data);
		}
	}

	void download(cv::UMatData* data, void* dst, int dims, const size_t sz[], const size_t srcofs[], const size_t srcstep[], const size_t dststep[]) const override {
		std::cout << "UserMatDebugAllocator: download" << std::endl;
		return m_defaultAllocator->download(data, dst, dims, sz, srcofs, srcstep, dststep);
	}

	void upload(cv::UMatData* data, const void* src, int dims, const size_t sz[], const size_t dstofs[], const size_t dststep[], const size_t srcstep[]) const override {
		std::cout << "UserMatDebugAllocator: upload" << std::endl;
		return m_defaultAllocator->upload(data, src, dims, sz, dstofs, dststep, srcstep);
	}

	void copy(cv::UMatData* srcdata, cv::UMatData* dstdata, int dims, const size_t sz[], const size_t srcofs[], const size_t srcstep[], const size_t dstofs[], const size_t dststep[], bool sync) const override {
		std::cout << "UserMatDebugAllocator: copy" << std::endl;
		return m_defaultAllocator->copy(srcdata, dstdata, dims, sz, srcofs, srcstep, dstofs, dststep, sync);
	}

	cv::BufferPoolController* getBufferPoolController(const char* id = NULL) const override {
		std::cout << "UserMatDebugAllocator: getBufferPoolController" << std::endl;
		return m_defaultAllocator->getBufferPoolController(id);
	}

	cv::MatAllocator* m_defaultAllocator;
	cv::MatAllocator* m_stdAllocator;
};