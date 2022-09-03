#pragma once
#include <cassert>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <algorithm>
#include <sstream>
#include <random>
#include <array>
#include <type_traits>


// TODO: get_submatrix; inverse, determinant; 
namespace SR
{
	/* Type declarations */

	template <typename ScalarType, size_t row_, size_t col_>
	class Matrix;

	// Define common matrix alias
	template <size_t row_, size_t col_>
	using MatrixXf = Matrix<float, row_, col_>;

	template <size_t row_, size_t col_>
	using MatrixXd = Matrix<double, row_, col_>;

	template <size_t row_, size_t col_>
	using MatrixXi = Matrix<int, row_, col_>;

	using Matrix2f = MatrixXf<2, 2>;
	using Matrix2d = MatrixXd<2, 2>;
	using Matrix2i = MatrixXi<2, 2>;
	using Matrix3f = MatrixXf<3, 3>;
	using Matrix3d = MatrixXd<3, 3>;
	using Matrix3i = MatrixXi<3, 3>;
	using Matrix4f = MatrixXf<4, 4>;
	using Matrix4d = MatrixXd<4, 4>;
	using Matrix4i = MatrixXi<4, 4>;

	// Define vector alias; vector is a column vector (matrix with col_ = 1)
	template <typename ScalarType, size_t row_>
	using Vector = Matrix<ScalarType, row_, 1>;

	template <size_t row_>
	using VectorXf = Vector<float, row_>;

	template <size_t row_>
	using VectorXd = Vector<double, row_>;

	template <size_t row_>
	using VectorXi = Vector<int, row_>;

	using Vector2f = VectorXf<2>;
	using Vector2d = VectorXd<2>;
	using Vector2i = VectorXi<2>;
	using Vector3f = VectorXf<3>;
	using Vector3d = VectorXd<3>;
	using Vector3i = VectorXi<3>;
	using Vector4f = VectorXf<4>;
	using Vector4d = VectorXd<4>;
	using Vector4i = VectorXi<4>;


	/* Type definitions */

	template <typename ScalarType, size_t row_, size_t col_>
	class Matrix {
	public:
		// Construct to be a zero matrix by default. Other members have an initial value.
		// std::array will manage the memory for us, so just use the synthesized copy control functions.
		Matrix() : data{ 0 } {}

		// Init the matrix with initializer list like {{1,2,3}, {4,5,6}}
		// If the initializer is ill-formed to be a matrix, an assertion is triggered.
		// Refer to http://eigen.tuxfamily.org/dox/classEigen_1_1Matrix.html#a1b552edeee11ced34c3e471ad00b820a
		Matrix(const std::initializer_list<std::initializer_list<ScalarType>>& mat_list)
		{
			assert(mat_list.size() == row);

			size_t i = 0;
			for (const auto& row_list : mat_list) {
				assert(row_list.size() == col);

				size_t j = 0;
				for (const auto& n : row_list) {
					this->at(i, j) = n;
					j++;
				}
				i++;
			}
		}

		// Init the matrix if it's either a row vector or column vector.
		Matrix(const std::initializer_list<ScalarType>& vec_list)
		{
			assert((vec_list.size() == row && col == 1) || (vec_list.size() == col && row == 1));

			if (col == 1) {
				size_t i = 0;
				for (const auto& n : vec_list) {
					this->at(i, 0) = n;
					i++;
				}
			}
			else {
				size_t j = 0;
				for (const auto& n : vec_list) {
					this->at(0, j) = n;
					j++;
				}
			}
		}

		/* Element access */
		ScalarType& at(size_t i, size_t j)
		{
			assert(i < row);
			assert(j < col);
			return data[i * col + j];
		}

		const ScalarType& at(size_t i, size_t j) const
		{
			assert(i < row);
			assert(j < col);
			return data[i * col + j];
		}

		Matrix<ScalarType, 1, col_> get_row(int i) const
		{
			Matrix<ScalarType, 1, col_> row;
			for (size_t j = 0; j < col_; j++) {
				row.at(0, j) = this->at(i, j);
			}
			return row;
		}

		void set_row(int i, const Matrix<ScalarType, 1, col_>& row)
		{
			for (size_t j = 0; j < col_; j++) {
				this->at(i, j) = row.at(0, j);
			}
		}

		Vector<ScalarType, row_> get_col(int j) const
		{
			Vector<ScalarType, row_> col;
			for (size_t i = 0; i < row_; i++) {
				col.at(i, 0) = this->at(i, j);
			}
			return col;
		}

		void set_col(int j, const Vector<ScalarType, row_>& col)
		{
			for (size_t i = 0; i < row_; i++) {
				this->at(i, j) = col.at(i, 0);
			}
		}

		// For vectors
		ScalarType& at(size_t i)
		{
			assert(col == 1 && i < row);
			return this->data[i];
		}

		const ScalarType& at(size_t i) const
		{
			assert(col == 1 && i < row);
			return this->data[i];
		}

		const ScalarType x() const
		{
			assert(col_ == 1);
			return this->at(0);
		}

		ScalarType& x()
		{
			assert(col_ == 1);
			return this->at(0);
		}

		const ScalarType y() const
		{
			assert(col_ == 1);
			return this->at(1);
		}

		ScalarType& y()
		{
			assert(col_ == 1);
			return this->at(1);
		}

		const ScalarType z() const
		{
			assert(col_ == 1);
			return this->at(2);
		}

		ScalarType& z()
		{
			assert(col_ == 1);
			return this->at(2);
		}

		/* Unary operators */

		// Get the negative matrix.
		Matrix<ScalarType, row_, col_> operator-() const
		{
			Matrix<ScalarType, row_, col_> result(*this);
			return result *= -1;
		}

		void transpose()
		{
			assert(row_ == col_);
			for (size_t i = 0; i < row_; i++) {
				for (size_t j = 0; j < i; j++) {
					std::swap(this->at(i, j), this->at(j, i));
				}
			}
		}

		Matrix<ScalarType, col_, row_> transposed() const
		{
			Matrix<ScalarType, col_, row_> transposed_mat;
			for (size_t i = 0; i < row_; i++) {
				for (size_t j = 0; j < col_; j++) {
					transposed_mat.at(i, j) = this->at(j, i);
				}
			}
			return transposed_mat;
		}

		// Compute the L2-norm squared.
		ScalarType norm_squared() const
		{
			ScalarType res = 0;
			for (size_t i = 0; i < data.size(); i++) {
				res += data[i] * data[i];
			}
			return res;
		}

		double norm() const
		{
			return std::sqrt(static_cast<double>(this->norm_squared()));
		}

		// Normalize this matrix / vector with L2-norm.
		void normalize()
		{
			// Not for non-floating-point type.
			assert(std::is_floating_point<ScalarType>::value);

			ScalarType length = static_cast<ScalarType>(this->norm());  // must be float or double
			for (size_t i = 0; i < data.size(); i++) {
				data[i] /= length;
			}
		}

		Matrix<ScalarType, row_, col_> normalized() const
		{
			Matrix<ScalarType, row_, col_> res(*this);
			res.normalize();

			return res;
		}


		//double determinant() const
		//{
		//	assert(col_ == row_);

		//	// 2x2
		//	if (row_ == 2) {
		//		return this->at(0, 0) * this->at(1, 1) - this->at(0, 1) * this->at(1, 0);
		//	}

		//	if (row_ == 3) {

		//	}
		//}

		//MatrixBase<ScalarType, row_, col_> inverse() const
		//{
		//	// TODO: deal with the case when ScalarType is integer
		//}

		/* Binary operators */

		// Elementwise addition
		Matrix<ScalarType, row_, col_>& operator+=(const Matrix<ScalarType, row_, col_>& rhs)
		{
			for (size_t i = 0; i < row_; i++) {
				for (size_t j = 0; j < col_; j++) {
					this->at(i, j) += rhs.at(i, j);
				}
			}
			return *this;
		}

		Matrix<ScalarType, row_, col_> operator+(const Matrix<ScalarType, row_, col_>& rhs) const
		{
			Matrix<ScalarType, row_, col_> result(*this);
			return result += rhs;
		}

		// Elementwise subtraction
		Matrix<ScalarType, row_, col_>& operator-=(const Matrix<ScalarType, row_, col_>& rhs)
		{
			for (size_t i = 0; i < row_; i++) {
				for (size_t j = 0; j < col_; j++) {
					this->at(i, j) -= rhs.at(i, j);
				}
			}
			return *this;
		}

		Matrix<ScalarType, row_, col_> operator-(const Matrix<ScalarType, row_, col_>& rhs) const
		{
			Matrix<ScalarType, row_, col_> result(*this);
			return result -= rhs;
		}


		// Scalar multiplication; *= version is defined for convenience.
		Matrix<ScalarType, row_, col_>& operator*=(ScalarType scalar)
		{
			for (size_t i = 0; i < row_; i++) {
				for (size_t j = 0; j < col_; j++) {
					this->at(i, j) *= scalar;
				}
			}
			return *this;
		}

		// The scalar must be in lhs.
		friend Matrix<ScalarType, row_, col_> operator*(ScalarType scalar, const Matrix<ScalarType, row_, col_>& mat)
		{
			// Note that for integer matrix, only integer arithmetic is performed.
			Matrix<ScalarType, row_, col_> result(mat);
			return result *= scalar;
		}

		// Scalar division, just for convenience when doing scalar multiplication.
		Matrix<ScalarType, row_, col_>& operator/=(ScalarType scalar)
		{
			for (size_t i = 0; i < row_; i++) {
				for (size_t j = 0; j < col_; j++) {
					this->at(i, j) /= scalar;
				}
			}
			return *this;
		}

		// The scalar must be in rhs.
		Matrix<ScalarType, row_, col_> operator/(ScalarType scalar) const
		{
			Matrix<ScalarType, row_, col_> result(*this);
			return result /= scalar;
		}

		// Matrix multiplication
		template <size_t rhs_col_>
		Matrix<ScalarType, row_, rhs_col_> operator*(const Matrix<ScalarType, col_, rhs_col_>& rhs) const
		{
			Matrix<ScalarType, row_, rhs_col_> result;
			for (size_t i = 0; i < row_; i++) {
				for (size_t j = 0; j < rhs_col_; j++) {
					for (size_t k = 0; k < col_; k++) {
						result.at(i, j) += this->at(i, k) * rhs.at(k, j);
					}
				}
			}
			return result;
		}

		// Dot product for vectors only.
		ScalarType dot(const Vector<ScalarType, row_>& rhs) const
		{
			assert(col_ == 1);

			ScalarType res = 0;
			for (size_t i = 0; i < data.size(); i++) {
				res += this->at(i) * rhs.at(i);
			}
			return res;
		}

		// Cross Product for 3-dim column vectors only.
		Vector<ScalarType, 3> cross(const Vector<ScalarType, 3>& rhs) const
		{
			Vector<ScalarType, 3> lhs(*this);
			return Vector<ScalarType, 3>{ 
				lhs.data[1] * rhs.data[2] - lhs.data[2] * rhs.data[1],
				lhs.data[2] * rhs.data[0] - lhs.data[0] * rhs.data[2],
				lhs.data[0] * rhs.data[1] - lhs.data[1] * rhs.data[0] };
		}

		/* Static functions */
		static Matrix<ScalarType, row_, col_> identity()
		{
			Matrix<ScalarType, row_, col_> mat;
			for (size_t i = 0; i < std::min(row_, col_); i++) {
				mat.at(i, i) = 1;
			}
			return mat;
		}

	protected:
		size_t row = row_;
		size_t col = col_;
		std::array<ScalarType, row_* col_> data;
	};
	

	template <typename ScalarType, size_t row_>
	inline ScalarType dot(const Vector<ScalarType, row_>& lhs, const Vector<ScalarType, row_>& rhs)
	{
		return lhs.dot(rhs);
	}

	template <typename ScalarType>
	inline Vector<ScalarType, 3> cross(const Vector<ScalarType, 3>& lhs, const Vector<ScalarType, 3>& rhs)
	{
		return lhs.cross(rhs);
	}

	template <typename ScalarType, size_t row_, size_t col_>
	inline Matrix<ScalarType, row_, col_> elementwise_multiply(
		const Matrix<ScalarType, row_, col_>& lhs, const Matrix<ScalarType, row_, col_>& rhs)
	{
		Matrix<ScalarType, row_, col_> res;
		for (size_t i = 0; i < row_; i++) {
			for (size_t j = 0; j < col_; j++) {
				res.at(i, j) = lhs.at(i, j) * rhs.at(i, j);
			}
		}
		return res;
	}

	// Print the matrix or vector.
	template <typename ScalarType, size_t row_, size_t col_>
	std::ostream& operator<<(std::ostream& os, const Matrix<ScalarType, row_, col_>& mat)
	{
		if (col_ != 1) {
			// Print a matrix
			std::array<size_t, col_> col_max_length = { 0 };

			for (size_t j = 0; j < col_; j++) {
				for (size_t i = 0; i < row_; i++) {
					std::ostringstream num_str;
					num_str << mat.at(i, j);
					col_max_length[j] = std::max(col_max_length[j], num_str.str().size());
				}
			}

			for (size_t i = 0; i < row_; i++) {
				for (size_t j = 0; j < col_; j++) {
					os << std::setw(col_max_length[j]) << mat.at(i, j) << " ";
				}
				if (i != row_ - 1) {
					std::cout << std::endl;
				}
			}
		}
		else {
			// Print a vector as a row vector, enclosed by "()"
			os << "(";
			for (size_t i = 0; i < row_; i++) {
				os << mat.at(i);
				if (i != row_ - 1) {
					os << ", ";
				}
			}
			os << ")";
			return os;
		}

		return os;
	}
	
	/* Math utils */
	const double DOUBLE_POS_INFINITY = std::numeric_limits<double>::infinity();
	const double DOUBLE_NEG_INFINITY = -std::numeric_limits<double>::infinity();
	const double EPS = 1e-4;

	/* Random number generation */
	struct Random {
		static std::uniform_real_distribution<double> uniform_distribution;
		static std::normal_distribution<double> normal_distribution;
		static std::mt19937 gen;

		static double rand_uniform(double min = -1.0, double max = 1.0)
		{
			return uniform_distribution(gen) * (max - min) + min;
		}

		static double rand_normal(double mean = 0.0, double stddev = 1.0)
		{
			return normal_distribution(gen) * stddev + mean;
		}
	};

	std::uniform_real_distribution<double> Random::uniform_distribution{ 0.0, 1.0 };
	std::normal_distribution<double> Random::normal_distribution{ 0.0, 1.0 };
	std::mt19937 Random::gen{ std::random_device{}() };

	Vector3d rand_vec3_on_unit_sphere()
	{
		// Using random normal samples
		Vector3d vec;
		
		do {
			vec = { Random::rand_normal(), Random::rand_normal(), Random::rand_normal() };
		} while (vec.norm() < 0.0001);

		// Project onto unit sphere
		vec.normalize();

		return vec;
	}

	Vector2d rand_vec2_in_unit_circle()
	{
		Vector2d vec;

		do {
			vec = { Random::rand_normal(), Random::rand_normal() };
		} while (vec.norm() > 1);

		return vec;
	}

	//Vector3d random_in_hemisphere(const Vector3d& normal)
	//{
	//	Vector3d in_unit_sphere = random_in_unit_sphere();
	//	if (dot(in_unit_sphere, normal) > 0.0) // In the same hemisphere as the normal
	//		return in_unit_sphere;
	//	else
	//		return -in_unit_sphere;
	//}


	//Vector3d random_uniform_vec3()
	//{
	//	while (true) {
	//		Vector3d vec { random_double(), random_double(), random_double() };
	//		if (p.norm_squared() >= 1) continue;
	//		return p;
	//	}
	//}
}
