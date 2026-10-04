#pragma once

#include "QMEC/Math/Vec3.h"

namespace qmec
{
    struct Mat4;

    struct Quat
    {
        float x{0.0f};
        float y{0.0f};
        float z{0.0f};
        float w{1.0f};

        [[nodiscard]] float LengthSquared() const noexcept;
        [[nodiscard]] float Length() const noexcept;
        [[nodiscard]] Quat Normalized() const noexcept;
        [[nodiscard]] Quat Conjugated() const noexcept;

        [[nodiscard]] bool TryInverse(
            Quat& result,
            float tolerance = 0.000001f) const noexcept;

        [[nodiscard]] Vec3 Rotate(const Vec3& vector) const noexcept;
        [[nodiscard]] Mat4 ToMat4() const noexcept;

        [[nodiscard]] static Quat FromAxisAngle(
            const Vec3& axis,
            float angleRadians) noexcept;

        [[nodiscard]] static Quat FromYawPitch(
            float yawRadians,
            float pitchRadians) noexcept;

        Quat& operator+=(const Quat& other) noexcept;
        Quat& operator-=(const Quat& other) noexcept;
        Quat& operator*=(float scalar) noexcept;
        Quat& operator/=(float scalar) noexcept;
        Quat& operator*=(const Quat& other) noexcept;
    };

    [[nodiscard]] Quat operator+(Quat left, const Quat& right) noexcept;
    [[nodiscard]] Quat operator-(Quat left, const Quat& right) noexcept;
    [[nodiscard]] Quat operator-(const Quat& quaternion) noexcept;

    [[nodiscard]] Quat operator*(Quat quaternion, float scalar) noexcept;
    [[nodiscard]] Quat operator*(float scalar, Quat quaternion) noexcept;
    [[nodiscard]] Quat operator/(Quat quaternion, float scalar) noexcept;

    [[nodiscard]] Quat operator*(Quat left, const Quat& right) noexcept;

    [[nodiscard]] float Dot(const Quat& left, const Quat& right) noexcept;
}
