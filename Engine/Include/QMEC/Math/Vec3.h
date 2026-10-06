#pragma once
namespace qmec::math
{
    struct Vec3
    {
        float x{};
        float y{};
        float z{};

        float Length() const noexcept;
        float LengthSquared() const noexcept;
        Vec3 Normalized() const noexcept;
        Vec3& operator+=(const Vec3& other) noexcept;
        Vec3& operator-=(const Vec3& other) noexcept;
        Vec3 operator-() const noexcept;


    };
    Vec3 operator+(Vec3 left, const Vec3& right) noexcept;
    Vec3 operator*(Vec3 vector, float scalar) noexcept;
    Vec3 operator/(Vec3 vector, float scalar) noexcept;
    Vec3 operator/(float scalar ,Vec3 vector) noexcept;
    Vec3 operator*(float scalar, Vec3 vector) noexcept;
    Vec3 operator-(const Vec3& left, const Vec3& right) noexcept;
    float Dot(const Vec3& a, const Vec3& b) noexcept;
    Vec3 Cross(const Vec3& a, const Vec3& b) noexcept;
    

}

namespace qmec
{
    using math::Vec3;
    using math::Dot;
    using math::Cross;
}
