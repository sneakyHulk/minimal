// #include <cmath>
//
// struct adouble {
//	double v;
//	double t;
//	adouble() : v(0.0), t(0.0) {}
//	explicit adouble(double const& x) : v(x), t(0.0) {}
//	adouble(adouble const& x) = default;
//	adouble& operator=(double x) {
//		v = x;
//		return *this;
//	}
// };
//
// inline adouble operator+(adouble const& x1, adouble const& x2) {
//	adouble y;
//	y.v = x1.v + x2.v;
//	y.t = x1.t + x2.t;
//	return y;
// }
//
// inline adouble operator-(adouble const& x1, adouble const& x2) {
//	adouble y;
//	y.v = x1.v - x2.v;
//	y.t = x1.t - x2.t;
//	return y;
// }
//
// inline adouble operator*(adouble const& x1, adouble const& x2) {
//	adouble y;
//	y.v = x1.v * x2.v;
//	y.t = x2.v * x1.t + x1.v * x2.t;
//	return y;
// }
//
// inline adouble operator/(adouble const& x1, adouble const& x2) {
//	adouble y;
//	y.v = x1.v / x2.v;
//	y.t = (x2.v * x1.t - x1.v * x2.t) / (x2.v * x2.v);
//	return y;
// }
//
// inline adouble sin(adouble const& x) {
//	adouble y;
//	y.v = std::sin(x.v);
//	y.t = std::cos(x.v) * x.t;
//	return y;
// }
//
// inline adouble pow(const adouble& x, int n) {
//	adouble y;
//	y.v = std::pow(x.v, n);
//	y.t = n * std::pow(x.v, n - 1) * x.t;
//	return y;
// }
// inline adouble const& conj(adouble const& x) { return x; }
// inline adouble const& real(adouble const& x) { return x; }
// inline adouble imag(const adouble&) { return {}; }
// inline adouble abs(const adouble& x) {
//	adouble y;
//	y.v = std::fabs(x.v);
//	y.t = std::fabs(x.t);
//
//	return y;
// }
// inline adouble abs2(const adouble& x) { return x * x; }

#include <Eigen/Eigen>
#include <autodiff/reverse/var.hpp>
#include <autodiff/reverse/var/eigen.hpp>
#include <iostream>

template <typename T>
T func(T x) {
	return x * x * 2.0;
}

int main() {
	// Eigen::Matrix<double, 4, 4> test;
	Eigen::Matrix4d test;
	test << 0., -4., 0., 960., -4., 0., 0., 600., 0., 0., 4., 0., 0., 0., 0., 1.;
	autodiff::Vector4var e;
	e << 1., 3000., 333., 4.;

	autodiff::Vector4var erg = test * e;

	auto [ux1] = autodiff::derivatives(erg(0), autodiff::wrt(e(1)));
	e(1).update(200.);
	erg(0).update();
	auto [ux2] = autodiff::derivatives(erg(0), autodiff::wrt(e(1)));



	// auto dudx = autodiff::derivative(erg, autodiff::wrt(e(1)), autodiff::at(e(1)));  // evaluate the derivative du/dx
	//  auto dudx2 = autodiff::derivative(erg, autodiff::wrt(e(0)), autodiff::at(e));  // evaluate the derivative du/dx

	std::cout << ux1 << std::endl;
	std::cout << ux2 << std::endl;
	// std::cout << dudx2 << std::endl;

	return 0;
	// std::cout << "du/dx = " << dudx << std::endl;  // print the evaluated derivative du/dx
}