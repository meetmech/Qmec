#include "QMEC/Math/Vec3.h"
#include <cmath>

namespace qmec
{

    float Dot(const Vec3& a, const Vec3& b) noexcept
    {
        return a.x * b.x+ a.y * b.y+ a.z * b.z;
    }

    Vec3 Cross(const Vec3& a, const Vec3& b) noexcept
    {
        return
        {
            a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x
        };
    }

    float Vec3::Length() const noexcept
    {
        return std::sqrt(x * x +y * y +z * z);
    }

    float Vec3::LengthSquared() const noexcept
    {
        return x * x + y * y + z * z;
    }

    Vec3 Vec3::Normalized() const noexcept
    {
      
        const float length = Length();

        if (length <= 0.000001f)
        {
            return {};
        }
        return
        {
            x / length,
            y / length,
            z / length
        };
    }

  
    Vec3 operator-(const Vec3& left, const Vec3& right) noexcept
    {
        return {
            left.x - right.x,
            left.y - right.y,
            left.z - right.z
        };
    }

    Vec3& Vec3::operator+=(const Vec3& other) noexcept
    {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }

    Vec3& Vec3::operator-=(const Vec3& other) noexcept
    {
        x -= other.x;
        y -= other.y;
        z -= other.z;
        return *this;
    }

    Vec3 Vec3::operator-() const noexcept
    {
        return Vec3{-x,-y,-z};
    }

    Vec3 operator+(Vec3 left, const Vec3& right) noexcept
    {
        left += right;
        return left;
    }

    Vec3 operator*(Vec3 vector, float scalar) noexcept
    {
        vector.x *= scalar;
        vector.y *= scalar;
        vector.z *= scalar;
        return vector;
    }

    Vec3 operator/(float scalar, Vec3 vector) noexcept
    {
        return
        {
            scalar / vector.x,
            scalar / vector.y,
            scalar / vector.z
        };
    }

    Vec3 operator/(Vec3 vector, float scalar) noexcept
    {
        return
        {
            vector.x / scalar,
            vector.y / scalar,
            vector.z / scalar
        };
    }

    Vec3 operator*(float scalar, Vec3 vector) noexcept
    {
        vector.x *= scalar;
        vector.y *= scalar;
        vector.z *= scalar;
        return vector;
    }
}
