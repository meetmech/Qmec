#include "QMEC/Math/Mat4.h"
#include "QMEC/Math/Vec3.h"
#include <cmath>
#include <cassert>
#include <utility>
namespace qmec::math
{

	
		Mat4 Mat4::Identity() noexcept
		{
			Mat4 result{};
			result.values[0][0] = 1.0f;
			result.values[1][1] = 1.0f;
			result.values[2][2] = 1.0f;
			result.values[3][3] = 1.0f;

			return result;
		}

		Mat4 Mat4::Translation(float x, float y,float z) noexcept
		{
			Mat4 result = Mat4::Identity();

			result.values[3][0] = x;
			result.values[3][1] = y;
			result.values[3][2] = z;

			return result;
		}

		Mat4 Mat4::RotationZ(float rad) noexcept
		{
			const float cosine = std::cos(rad);
			const float sine = std::sin(rad);

			Mat4 result = Mat4::Identity();
			result.values[0][0] = cosine;
			result.values[0][1] = sine;
			result.values[1][0] = -sine;
			result.values[1][1] = cosine;

			return result;
		}

		Mat4 Mat4::RotationY(float rad) noexcept
		{
			const float cosine = std::cos(rad);
			const float sine = std::sin(rad);

			Mat4 result = Mat4::Identity();
			result.values[0][0] = cosine;
			result.values[0][2] = -sine;
			result.values[2][0] = sine;
			result.values[2][2] = cosine;

			return result;
		}

		Mat4 Mat4::RotationX(float rad) noexcept
		{
			const float cosine = std::cos(rad);
			const float sine = std::sin(rad);

			Mat4 result = Mat4::Identity();
			result.values[1][1] = cosine;
			result.values[1][2] = sine;
			result.values[2][1] = -sine;
			result.values[2][2] = cosine;

			return result;
		}



		Mat4 Mat4::Transpose(const Mat4& mat) noexcept
		{
			Mat4 result{};

			for (size_t i = 0; i < 4; i++)
			{
				for (size_t j = 0; j < 4; j++)
				{
					result.values[j][i] = mat.values[i][j];
				}
			}
			return result;
		}


		Mat4 Mat4::InverseTransform(const Mat4& transform) noexcept
		{
			Mat4 result = Mat4::Identity();

			Mat4 inverseRotation = Mat4::Transpose(transform);

		
			for (size_t i = 0; i < 3; ++i)
			{
				for (size_t j = 0; j < 3; ++j)
				{
					result.values[i][j] = inverseRotation.values[i][j];
				}
			}

			Vec3 position{transform.values[3][0],transform.values[3][1],transform.values[3][2]};


			result.values[3][0] = -(position.x * result.values[0][0] + position.y * result.values[1][0] +
					                position.z * result.values[2][0]);

			result.values[3][1] = -(position.x * result.values[0][1] + position.y * result.values[1][1] +
					                 position.z * result.values[2][1]);

			result.values[3][2] = -(position.x * result.values[0][2] + position.y * result.values[1][2] +
					                 position.z * result.values[2][2]);

			return result;
		}

		Mat4 Mat4::operator*(const Mat4& other) const noexcept
		{
			Mat4 result{};
			for (int row = 0; row < 4; row++)
			{
				for (int coloumn = 0; coloumn < 4; coloumn++)
				{
					for (int k = 0; k < 4; k++)
					{
						result.values[row][coloumn] += values[row][k] * other.values[k][coloumn];
					}
				}
			}

			return result;
		}

		Mat4& Mat4::operator*=(const Mat4& other) noexcept
		{
			*this = *this * other;
			return *this;
		}

		


		Mat4 Mat4::Perspective(float verticalFovRadians,float aspectRatio,float nearPlane,float farPlane) noexcept
		{
			const float f = 1.0f / std::tan(verticalFovRadians * 0.5f);

			const float A = farPlane / (farPlane - nearPlane);

			const float B = -(nearPlane * farPlane) / (farPlane - nearPlane);

			return Mat4{
				f / aspectRatio, 0.0f, 0.0f, 0.0f,
				0.0f,f,0.0f, 0.0f,
				0.0f,0.0f, A,1.0f,
				0.0f,0.0f, B,0.0f
			};
		}

		Mat4 Mat4::Orthographic(float width, float height, float nearPlane, float farPlane) noexcept
		{
			assert(std::isfinite(width) && width > 0.0f);
			assert(std::isfinite(height) && height > 0.0f);
			assert(std::isfinite(nearPlane) && std::isfinite(farPlane));
			const float depthRange = farPlane - nearPlane;
			assert(std::isfinite(depthRange) && depthRange > 0.0f);

			Mat4 result = Identity();
			result.values[0][0] = 2.0f / width;
			result.values[1][1] = 2.0f / height;
			result.values[2][2] = 1.0f / depthRange;
			result.values[3][2] = -nearPlane / depthRange;
			return result;
		}

		bool Mat4::TryInverse(Mat4& result, float tolerance) const noexcept
		{
			if (!std::isfinite(tolerance) || tolerance <= 0.0f)
				return false;

			float augmented[4][8]{};
			for (size_t row = 0; row < 4; ++row)
			{
				for (size_t column = 0; column < 4; ++column)
					augmented[row][column] = values[row][column];
				augmented[row][row + 4] = 1.0f;
			}

			for (size_t column = 0; column < 4; ++column)
			{
				size_t pivotRow = column;
				for (size_t row = column + 1; row < 4; ++row)
				{
					if (std::fabs(augmented[row][column]) >
						std::fabs(augmented[pivotRow][column]))
						pivotRow = row;
				}

				const float pivot = augmented[pivotRow][column];
				if (!std::isfinite(pivot) || std::fabs(pivot) <= tolerance)
					return false;

				if (pivotRow != column)
				{
					for (size_t i = 0; i < 8; ++i)
						std::swap(augmented[pivotRow][i], augmented[column][i]);
				}

				const float divisor = augmented[column][column];
				for (size_t i = 0; i < 8; ++i)
					augmented[column][i] /= divisor;

				for (size_t row = 0; row < 4; ++row)
				{
					if (row == column)
						continue;

					const float factor = augmented[row][column];
					for (size_t i = 0; i < 8; ++i)
						augmented[row][i] -= factor * augmented[column][i];
				}
			}

			Mat4 inverse{};
			for (size_t row = 0; row < 4; ++row)
			{
				for (size_t column = 0; column < 4; ++column)
				{
					const float value = augmented[row][column + 4];
					if (!std::isfinite(value))
						return false;
					inverse.values[row][column] = value;
				}
			}

			result = inverse;
			return true;
		}

		Mat4 Mat4::LookAt(const Vec3& position,const Vec3& target,const Vec3& up) noexcept
		{
			const Vec3 forward = (target - position).Normalized();
			if (forward.Length() <= 0.000001f)
			{
				return Mat4::Identity();
			}

			Vec3 cameraUp = up.Normalized();
			if (cameraUp.Length() <= 0.000001f)
			{
				cameraUp = { 0.0f, 1.0f, 0.0f };
			}

			if (std::abs(Dot(forward, cameraUp)) >= 0.999f)
			{
				cameraUp = std::abs(forward.y) < 0.999f? Vec3{ 0.0f, 1.0f, 0.0f }: Vec3{ 0.0f, 0.0f, 1.0f };
			}

			const Vec3 right = Cross(cameraUp, forward).Normalized();
			cameraUp = Cross(forward, right);
			Mat4 result = Mat4::Identity();

			result.values[0][0] = right.x;
			result.values[0][1] = cameraUp.x;
			result.values[0][2] = forward.x;

			result.values[1][0] = right.y;
			result.values[1][1] = cameraUp.y;
			result.values[1][2] = forward.y;

			result.values[2][0] = right.z;
			result.values[2][1] = cameraUp.z;
			result.values[2][2] = forward.z;

			result.values[3][0] = -Dot(right, position);
			result.values[3][1] = -Dot(cameraUp, position);
			result.values[3][2] = -Dot(forward, position);

			return result;
		}

		Vec3 Mat4::TransformDirection(const Vec3& vector) const noexcept
		{
			return
			{
				vector.x * values[0][0] + vector.y * values[1][0] + vector.z * values[2][0],
				vector.x * values[0][1] + vector.y * values[1][1] + vector.z * values[2][1],
				vector.x * values[0][2] + vector.y * values[1][2] + vector.z * values[2][2]
			};
		}
}
