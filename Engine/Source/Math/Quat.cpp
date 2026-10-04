#include "QMEC/Math/Quat.h"

#include "QMEC/Math/Mat4.h"

#include <cassert>
#include <cmath>

namespace qmec
{
    float Quat::LengthSquared() const noexcept
    {
        return Dot(*this, *this);
    }

    float Quat::Length() const noexcept
    {
        return std::sqrt(LengthSquared());
    }

    Quat Quat::Normalized() const noexcept
    {
        const float length = Length();

        if(length <= 0.000001f)
        {
            return Quat{};
        }

        return *this / length;
    }

    Quat Quat::Conjugated() const noexcept
    {
        return Quat{-x, -y, -z, w};
    }

    bool Quat::TryInverse(Quat& result, float tolerance) const noexcept
    {
        const float lengthSquared = LengthSquared();
        const float toleranceSquared = tolerance * tolerance;

        if(lengthSquared <= toleranceSquared)
        {
            return false;
        }

        result = Conjugated() / lengthSquared;
        return true;
    }

    Vec3 Quat::Rotate(const Vec3& vector) const noexcept
    {
        assert(std::fabs(LengthSquared() - 1.0f) <= 0.001f && "Quaternion must be normalized before rotating a vector.");

        const Vec3 quaternionVector{x, y, z};
        const Vec3 doubledCross = 2.0f * Cross(quaternionVector, vector);

        return vector + w * doubledCross + Cross(quaternionVector, doubledCross);
    }

    Mat4 Quat::ToMat4() const noexcept
    {
        assert(std::fabs(LengthSquared() - 1.0f) <= 0.001f && "Quaternion must be normalized before creating a rotation matrix.");

        const float xx = x * x;
        const float yy = y * y;
        const float zz = z * z;
        const float xy = x * y;
        const float xz = x * z;
        const float yz = y * z;
        const float wx = w * x;
        const float wy = w * y;
        const float wz = w * z;

        
        return Mat4{ 1.0f - 2.0f * (yy + zz),
            2.0f * (xy + wz),
            2.0f * (xz - wy),
            0.0f,

            2.0f * (xy - wz),
            1.0f - 2.0f * (xx + zz),
            2.0f * (yz + wx),
            0.0f,

            2.0f * (xz + wy),
            2.0f * (yz - wx),
            1.0f - 2.0f * (xx + yy),
            0.0f,

            0.0f,
            0.0f,
            0.0f,
            1.0f
        };
    }

    Quat Quat::FromAxisAngle(const Vec3& axis,float angleRadians) noexcept
    {
        constexpr float AxisTolerance = 0.000001f;

        const float lengthSquared = Dot(axis, axis);
        if(lengthSquared <= AxisTolerance * AxisTolerance)
        {
            return Quat{};
        }

        Vec3 normalizedAxis = axis;
        if(std::fabs(lengthSquared - 1.0f) > AxisTolerance)
        {
            normalizedAxis = axis * (1.0f / std::sqrt(lengthSquared));
        }

        const float halfAngle = angleRadians * 0.5f;
        const float sine = std::sin(halfAngle);

        return Quat{normalizedAxis.x * sine, normalizedAxis.y * sine, normalizedAxis.z * sine,std::cos(halfAngle)};
    }

    Quat Quat::FromYawPitch( float yawRadians,float pitchRadians) noexcept
    {
        const Quat yaw = FromAxisAngle(Vec3{0.0f, 1.0f, 0.0f}, yawRadians);
        const Quat pitch = FromAxisAngle(Vec3{1.0f, 0.0f, 0.0f}, -pitchRadians);

        return (yaw * pitch).Normalized();
    }

    Quat& Quat::operator+=(const Quat& other) noexcept
    {
        x += other.x;
        y += other.y;
        z += other.z;
        w += other.w;
        return *this;
    }

    Quat& Quat::operator-=(const Quat& other) noexcept
    {
        x -= other.x;
        y -= other.y;
        z -= other.z;
        w -= other.w;
        return *this;
    }

    Quat& Quat::operator*=(float scalar) noexcept
    {
        x *= scalar;
        y *= scalar;
        z *= scalar;
        w *= scalar;
        return *this;
    }

    Quat& Quat::operator/=(float scalar) noexcept
    {
        assert(scalar != 0.0f && "Cannot divide a Quat by zero.");

        x /= scalar;
        y /= scalar;
        z /= scalar;
        w /= scalar;
        return *this;
    }

    Quat& Quat::operator*=(const Quat& other) noexcept
    {
        *this = *this * other;
        return *this;
    }

    Quat operator+(Quat left, const Quat& right) noexcept
    {
        left += right;
        return left;
    }

    Quat operator-(Quat left, const Quat& right) noexcept
    {
        left -= right;
        return left;
    }

    Quat operator-(const Quat& quaternion) noexcept
    {
        return Quat{-quaternion.x, -quaternion.y, -quaternion.z, -quaternion.w};
    }

    Quat operator*(Quat quaternion, float scalar) noexcept
    {
        quaternion *= scalar;
        return quaternion;
    }

    Quat operator*(float scalar, Quat quaternion) noexcept
    {
        quaternion *= scalar;
        return quaternion;
    }

    Quat operator/(Quat quaternion, float scalar) noexcept
    {
        quaternion /= scalar;
        return quaternion;
    }

    Quat operator*(Quat left, const Quat& right) noexcept
    {
        return Quat{
            left.w * right.x
                + left.x * right.w
                + left.y * right.z
                - left.z * right.y,

            left.w * right.y
                - left.x * right.z
                + left.y * right.w
                + left.z * right.x,

            left.w * right.z
                + left.x * right.y
                - left.y * right.x
                + left.z * right.w,

            left.w * right.w
                - left.x * right.x
                - left.y * right.y
                - left.z * right.z
        };
    }

    float Dot(const Quat& left, const Quat& right) noexcept
    {
        return left.x * right.x+ left.y * right.y+ left.z * right.z + left.w * right.w;
    }
}
