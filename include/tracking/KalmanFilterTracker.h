#pragma once
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <utility>
#include <vector>

#include "KalmanFilter.h"

template <std::size_t max_age = 4, std::size_t min_consecutive_hits = 3>
class KalmanBoxTracker : private KalmanFilter<7, 4, 3, {0, 1, 2}, {4, 5, 6}> {
	// reuse random generator
	static thread_local boost::uuids::random_generator generator;

	boost::uuids::uuid id = generator();
	int consecutive_hits = 0;
	int consecutive_fails = 0;
	bool displayed = false;

	// per frame history
	std::vector<std::tuple<double, double, double, double>> history;

   public:
	KalmanBoxTracker(double left, double top, double right, double bottom) : KalmanFilter<7, 4, 3, {0, 1, 2}, {4, 5, 6}>(make_constant_box_velocity_model_kalman_filter()) {}

	static decltype(x) convert_bbox_to_z(double left, double top, double right, double bottom) {
		decltype(x) x_;

		auto const w = right - left;
		auto const h = bottom - top;

		auto const x_center = left + w / 2.;
		auto const y_center = top + h / 2.;

		auto const s = w * h;
		auto const r = w / h;

		x_ << x_center, y_center, s, r, 0., 0., 0.;
		return x_;
	}

	static std::tuple<double, double, double, double> convert_x_to_bbox(decltype(x) const& x_) {
		auto const w = std::sqrt(x_(2) * x_(3));
		auto const h = x_(2) / w;

		return {x_(0) - w / 2., x_(1) - h / 2., x_(0) + w / 2., x_(1) + h / 2.};
	}

	std::tuple<double, double, double, double> predict(double dt) {
		if (dt * x(6) + x(2) <= 0) x(2) *= 0.;  // area must be >= 0;
		KalmanFilter<7, 4, 3, {0, 1, 2}, {4, 5, 6}>::predict(dt);
		if (consecutive_fails > 0) consecutive_hits = 0;
		consecutive_fails += 1;

		return history.emplace_back(convert_x_to_bbox(x));
	}

	void update(double left, double top, double right, double bottom) {
		consecutive_fails = 0;
		consecutive_hits += 1;

		KalmanFilter<7, 4, 3, {0, 1, 2}, {4, 5, 6}>::update(convert_bbox_to_z(left, top, right, bottom));

		history.back() = convert_x_to_bbox(x);
	}

   private:
	static KalmanFilter<7, 4, 3, {0, 1, 2}, {4, 5, 6}> make_constant_box_velocity_model_kalman_filter(double left, double top, double right, double bottom) {
		decltype(x) x_ = convert_bbox_to_z(left, top, right, bottom);
		decltype(F) F_;
		F_ << 1., 0., 0., 0., 1., 0., 0.,  //
		    0., 1., 0., 0., 0., 1., 0.,    //
		    0., 0., 1., 0., 0., 0., 1.,    //
		    0., 0., 0., 1., 0., 0., 0.,    //
		    0., 0., 0., 0., 1., 0., 0.,    //
		    0., 0., 0., 0., 0., 1., 0.,    //
		    0., 0., 0., 0., 0., 0., 1.;    //
		decltype(H) H_;
		H_ << 1., 0., 0., 0., 0., 0., 0.,  //
		    0., 1., 0., 0., 0., 0., 0.,    //
		    0., 0., 1., 0., 0., 0., 0.,    //
		    0., 0., 0., 1., 0., 0., 0.;    //
		decltype(P) P_ = decltype(P)::Identity();
		// P_(0, 0) *= 0.01;   // give low uncertainty to the initial position values (because newest yolo ist pretty accurate)
		// P_(1, 1) *= 0.01;   // give low uncertainty to the initial position values (because newest yolo ist pretty accurate)
		P_(2, 2) *= 10.;
		P_(3, 3) *= 10.;
		P_(4, 4) *= 10000.;  // give high uncertainty to the unobservable initial velocities
		P_(5, 5) *= 10000.;  // give high uncertainty to the unobservable initial velocities
		P_(6, 6) *= 10000.;  // give high uncertainty to the unobservable initial velocities
		decltype(R) R_ = decltype(R)::Identity();
		R_(2, 2) *= 10.;
		R_(3, 3) *= 10.;
		decltype(Q) Q_ = decltype(Q)::Identity();
		Q_(6, 6) *= 0.01;

		return {std::forward<decltype(x)>(x_), std::forward<decltype(F)>(F_), std::forward<decltype(H)>(H_), std::forward<decltype(P)>(P_), std::forward<decltype(R)>(R_), std::forward<decltype(Q)>(Q_)};
	}
};