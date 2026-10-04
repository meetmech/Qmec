#include "QMEC/Scene/Camera.h"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace qmec
{
    Camera::Camera() noexcept
    {
        UpdateView();
        UpdateProjection();
    }

    void Camera::SetPosition(const Vec3& position) noexcept
    {
        position_ = position;
        UpdateView();
    }

    void Camera::SetOrientation(const Quat& rotation) noexcept
    {
        orientation_ = rotation.Normalized();
        UpdateAnglesFromDirection(Forward());
        UpdateView();
    }

    void Camera::SetNearPlane(float value) noexcept
    {
        if(value <= 0.0f || value >= farPlane_)
        {
            return;
        }

        nearPlane_ = value;
        UpdateProjection();
    }

    void Camera::SetFarPlane(float value) noexcept
    {
        if(value <= nearPlane_)
        {
            return;
        }

        farPlane_ = value;
        UpdateProjection();
    }

    void Camera::SetFov(float value) noexcept
    {
        if(value <= 0.0f || value >= std::numbers::pi_v<float>)
        {
            return;
        }

        fieldOfViewRadians_ = value;
        UpdateProjection();
    }

    void Camera::SetAspectRatio(float value) noexcept
    {
        if(value <= 0.0f)
        {
            return;
        }

        aspectRatio_ = value;
        UpdateProjection();
    }

    void Camera::SetYawPitch(float yawRadians, float pitchRadians) noexcept
    {
        constexpr float MaxPitch = std::numbers::pi_v<float> * (89.0f / 180.0f);

        yawRadians_ = std::remainder(yawRadians,2.0f * std::numbers::pi_v<float>);
        pitchRadians_ = std::clamp(pitchRadians, -MaxPitch, MaxPitch);
        orientation_ = Quat::FromYawPitch(yawRadians_, pitchRadians_);
        UpdateView();
    }

    void Camera::AddYawPitch( float yawDeltaRadians,float pitchDeltaRadians) noexcept
    {
        SetYawPitch(yawRadians_ + yawDeltaRadians,pitchRadians_ + pitchDeltaRadians);
    }

    void Camera::LookAt(const Vec3& target) noexcept
    {
        const Vec3 direction = (target - position_).Normalized();
        if(direction.Length() <= 0.000001f)
        {
            return;
        }

        UpdateAnglesFromDirection(direction);
        orientation_ = Quat::FromYawPitch(yawRadians_, pitchRadians_);
        UpdateView();
    }

    const Vec3& Camera::Position() const noexcept
    {
        return position_;
    }

    const Quat& Camera::Orientation() const noexcept
    {
        return orientation_;
    }

    const Mat4& Camera::ViewMatrix() const noexcept
    {
        return view_;
    }

    const Mat4& Camera::ProjectionMatrix() const noexcept
    {
        return projection_;
    }

    float Camera::Yaw() const noexcept
    {
        return yawRadians_;
    }

    float Camera::Pitch() const noexcept
    {
        return pitchRadians_;
    }

   

    void Camera::UpdateView() noexcept
    {
        view_ = Mat4::LookAt(position_,position_ + Forward(),Up());
    }

    void Camera::UpdateProjection() noexcept
    {
        projection_ = Mat4::Perspective(fieldOfViewRadians_,aspectRatio_,nearPlane_,farPlane_);
    }

    void Camera::UpdateAnglesFromDirection(const Vec3& direction) noexcept
    {
        const Vec3 normalizedDirection = direction.Normalized();
        if(normalizedDirection.Length() <= 0.000001f)
        {
            return;
        }

        yawRadians_ = std::atan2(normalizedDirection.x, normalizedDirection.z);
        pitchRadians_ = std::asin(std::clamp(normalizedDirection.y, -1.0f, 1.0f));
    }

    Vec3 Camera::Forward() const noexcept
    {
        return orientation_.Rotate(Vec3{0.0f, 0.0f, 1.0f});
    }

    Vec3 Camera::Right() const noexcept
    {
        return orientation_.Rotate(Vec3{1.0f, 0.0f, 0.0f});
    }

    Vec3 Camera::Up() const noexcept
    {
        return orientation_.Rotate(Vec3{0.0f, 1.0f, 0.0f});
    }
}
