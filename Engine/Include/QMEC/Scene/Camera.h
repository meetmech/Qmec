#pragma once
#include "QMEC/Math/Mat4.h"
#include "QMEC/Math/Vec3.h"
#include "QMEC/Math/Quat.h"

namespace qmec
{
    class Camera final
    {
    public:
        Camera() noexcept;

        void SetPosition(const Vec3& position) noexcept;
        void SetOrientation(const Quat& rotation) noexcept;
        void SetNearPlane(float nearPlane) noexcept;
        void SetFarPlane(float farPlane) noexcept;
        void SetFov(float verticalFovRadians) noexcept;
        void SetAspectRatio(float aspectRatio) noexcept;
        void SetYawPitch(float yawRadians, float pitchRadians) noexcept;
        void AddYawPitch(float yawDeltaRadians, float pitchDeltaRadians) noexcept;
        void LookAt(const Vec3& target) noexcept;

        [[nodiscard]] const Vec3& Position() const noexcept;
        [[nodiscard]] const Quat& Orientation() const noexcept;
        [[nodiscard]] const Mat4& ViewMatrix() const noexcept;
        [[nodiscard]] const Mat4& ProjectionMatrix() const noexcept;
        [[nodiscard]] float Yaw() const noexcept;
        [[nodiscard]] float Pitch() const noexcept;
        [[nodiscard]] Vec3 Forward() const noexcept;
        [[nodiscard]] Vec3 Right() const noexcept;
        [[nodiscard]] Vec3 Up() const noexcept;

    private:
        void UpdateView() noexcept;
        void UpdateProjection() noexcept;
        void UpdateAnglesFromDirection(const Vec3& direction) noexcept;

        Vec3 position_{};
        Quat orientation_{};
        float nearPlane_{ 0.1f };
        float farPlane_{ 100.0f };
        float fieldOfViewRadians_{ 1.0471975512f };
        float aspectRatio_{ 16.0f / 9.0f };
        float yawRadians_{};
        float pitchRadians_{};
        Mat4 view_{ Mat4::Identity() };
        Mat4 projection_{ Mat4::Identity() };
    };
}
