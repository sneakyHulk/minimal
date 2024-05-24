#include <Eigen/Eigen>

#include <nlohmann/json.hpp>

struct CameraConfig {
	nlohmann::json const config;

	Eigen::Matrix<float, 3, 4> projection_matrix;
	Eigen::Matrix<float, 3, 3> KR;
	Eigen::Matrix<float, 3, 3> KR_inv;
	Eigen::Matrix<float, 3, 1> C;
	Eigen::Matrix<float, 3, 1> translation_camera;

	explicit CameraConfig(std::string const& filename) : config(nlohmann::json::parse(std::ifstream(filename))) {
		std::vector<std::vector<float>> const projection_matrix_stl = config["projections"][0]["projection_matrix"].template get<std::vector<std::vector<float>>>();
		projection_matrix = toEigenMatrix<3, 4>(projection_matrix_stl);
		KR = projection_matrix(Eigen::all, Eigen::seq(0, Eigen::last - 1));
		KR_inv = KR.inverse();
		C = projection_matrix(Eigen::all, Eigen::last);
		translation_camera = -KR_inv * C;
	}

	std::array<float, 3> image_to_world(float x, float y, float height) {
		// auto KR = projection_matrix(Eigen::all, Eigen::last - 1);
		auto image_coordinates = Eigen::Vector3f();
		image_coordinates << x, y, 1.0;

		auto tmp1 = KR_inv * image_coordinates;
		auto tmp2 = tmp1 * (height - translation_camera(2)) / tmp1(2);
		auto tmp3 = tmp2 + translation_camera;

		return {tmp3(0), tmp3(1), height};
	}

	template <std::size_t rows, std::size_t cols, typename Scalar>
	inline static Eigen::Matrix<Scalar, rows, cols> toEigenMatrix(const std::vector<std::vector<Scalar>>& vectors) {
		Eigen::Matrix<Scalar, -1, -1> M(vectors.size(), vectors.front().size());
		for (size_t i = 0; i < vectors.size(); i++)
			for (size_t j = 0; j < vectors.front().size(); j++) M(i, j) = vectors[i][j];
		return M;
	}

	template <typename Scalar, typename Matrix>
	inline static std::vector<std::vector<Scalar>> fromEigenMatrix(const Matrix& M) {
		std::vector<std::vector<Scalar>> m;
		m.resize(M.rows(), std::vector<Scalar>(M.cols(), 0));
		for (size_t i = 0; i < m.size(); i++)
			for (size_t j = 0; j < m.front().size(); j++) m[i][j] = M(i, j);
		return m;
	}
};