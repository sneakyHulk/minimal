#pragma once
#include "KalmanFilterTracker.h"
#include "association_functions.h"
#include <vector>

template <std::size_t max_age = 4, std::size_t min_consecutive_hits = 3, double association_threshold = 0.3>
class Sort {
	std::vector<KalmanBoxTracker<max_age, min_consecutive_hits>> trackers;
   public:
	Sort() = default;

};