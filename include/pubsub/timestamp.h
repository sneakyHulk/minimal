#pragma once

#include <cstdint>
#include <msgpack.hpp>

class Timestamp {
   public:
	Timestamp() : ns(0){};
	std::int64_t ns;
	std::vector<int> data;

	MSGPACK_DEFINE(ns, data);
};