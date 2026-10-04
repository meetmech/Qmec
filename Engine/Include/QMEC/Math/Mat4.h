#pragma once
#include "QMEC/Math/Vec3.h"

namespace qmec
{

	struct alignas(16) Mat4
	{
	public :
		float values[4][4]{};

		static Mat4 Identity() noexcept;
		static Mat4 Translation(float x, float y, float z) noexcept;
		static Mat4 RotationZ(float radians) noexcept;
		static Mat4 RotationY(float radians) noexcept;
		static Mat4 RotationX(float radians) noexcept;
		static Mat4 Transpose(const Mat4&) noexcept;
		static Mat4 InverseTransform(const Mat4&) noexcept;
		[[nodiscard]] bool TryInverse(Mat4& result, float tolerance = 1.0e-8f) const noexcept;
		[[nodiscard]] Vec3 TransformDirection(const Vec3& vector) const noexcept;
		static Mat4 LookAt(const Vec3& eye,const Vec3& target,const Vec3& up) noexcept;
		Mat4 operator*(const Mat4& other) const noexcept;
		Mat4& operator*=(const Mat4& other)  noexcept;
		static Mat4 Perspective(float verticalFovRadians,float aspectRatio,float nearPlane,float farPlane) noexcept;

		[[nodiscard]] static Mat4 Orthographic(float width, float height, float nearPlane, float farPlane) noexcept;
		
	};

	


}
