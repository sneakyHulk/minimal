#pragma once
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <tuple>
#include <utility>
#include <vector>

#include "KalmanFilter.h"

namespace tracking {
	class BoundingBoxXYXY {
		double _left;
		double _top;
		double _right;
		double _bottom;

	   public:
		constexpr BoundingBoxXYXY(double const left, double const top, double const right, double const bottom) : _left(left), _top(top), _right(right), _bottom(bottom) {}
		[[nodiscard]] double left() const { return _left; }
		[[nodiscard]] double top() const { return _top; }
		[[nodiscard]] double right() const { return _right; }
		[[nodiscard]] double bottom() const { return _bottom; }
	};

	template <typename T>
	concept BoundingBoxType = requires(T bbox) {
		bbox.left();
		bbox.top();
		bbox.right();
		bbox.bottom();
	};

	template <std::size_t max_age, std::size_t min_consecutive_hits, BoundingBoxType BoundingBox = BoundingBoxXYXY>
	class KalmanBoxTracker : private KalmanFilter<7, 4, 3, {0, 1, 2}, {4, 5, 6}> {
		// reuse random generator
		static int _id_max;

		int _id = ++_id_max;
		int _consecutive_hits = 0;
		int _consecutive_fails = 0;
		bool _displayed = false;

		// per frame history
		std::vector<BoundingBox> _history{};

	   public:
		explicit KalmanBoxTracker(BoundingBox const& bbox) : KalmanFilter<7, 4, 3, {0, 1, 2}, {4, 5, 6}>(make_constant_box_velocity_model_kalman_filter(bbox)) {}

		[[nodiscard]] static decltype(z) convert_bbox_to_z(BoundingBox const& bbox) {
			decltype(z) z_;

			auto const w = bbox.right() - bbox.left();
			auto const h = bbox.bottom() - bbox.top();

			auto const x_center = bbox.left() + w / 2.;
			auto const y_center = bbox.top() + h / 2.;

			auto const s = w * h;
			auto const r = w / h;

			z_ << x_center, y_center, s, r;
			return z_;
		}

		[[nodiscard]] static constexpr BoundingBox convert_x_to_bbox(decltype(x) const& x_) {
			auto const w = std::sqrt(x_(2) * x_(3));
			auto const h = x_(2) / w;

			return {x_(0) - w / 2., x_(1) - h / 2., x_(0) + w / 2., x_(1) + h / 2.};
		}

		BoundingBox const& predict(double const dt) {
			if (dt * x(6) + x(2) <= 0) x(2) *= 0.;  // area must be >= 0;
			KalmanFilter<7, 4, 3, {0, 1, 2}, {4, 5, 6}>::predict(dt);
			if (_consecutive_fails > 0) _consecutive_hits = 0;
			_consecutive_fails += 1;

			return _history.emplace_back(convert_x_to_bbox(x));
		}

		void update(BoundingBox const& bbox) {
			_consecutive_fails = 0;
			_consecutive_hits += 1;
			if (_consecutive_hits >= min_consecutive_hits) _displayed = true;

			KalmanFilter<7, 4, 3, {0, 1, 2}, {4, 5, 6}>::update(convert_bbox_to_z(bbox));

			_history.back() = convert_x_to_bbox(x);
		}

		[[nodiscard]] BoundingBox state() const { return _history.back(); }
		[[nodiscard]] auto id() const { return _id; }
		[[nodiscard]] auto consecutive_hits() const { return _consecutive_hits; }
		[[nodiscard]] auto consecutive_fails() const { return _consecutive_fails; }
		[[nodiscard]] auto displayed() const { return _displayed; }

	   private:
		static KalmanFilter<7, 4, 3, {0, 1, 2}, {4, 5, 6}> make_constant_box_velocity_model_kalman_filter(BoundingBox const& bbox) {
			decltype(x) x_;
			x_ << convert_bbox_to_z(bbox), 0., 0., 0.;
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
			P_(0, 0) *= 10.;  // give low uncertainty to the initial position values (because newest yolo ist pretty accurate)
			P_(1, 1) *= 10.;  // give low uncertainty to the initial position values (because newest yolo ist pretty accurate)
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

	template <std::size_t max_age, std::size_t min_consecutive_hits, BoundingBoxType BoundingBox>
	int KalmanBoxTracker<max_age, min_consecutive_hits, BoundingBox>::_id_max = 0;
}  // namespace tracking