#pragma once

#include "QMEC/Physics/AABB.h"
#include "QMEC/Physics/Collider/ColliderShapes.h"
#include "QMEC/Scene/Components/TransformComponent.h"

namespace qmec::physics
{
    struct WorldBox
    {
        Vec3 centre{};
        Vec3 halfExtents{};
        Vec3 axisX{1.0f, 0.0f, 0.0f};
        Vec3 axisY{0.0f, 1.0f, 0.0f};
        Vec3 axisZ{0.0f, 0.0f, 1.0f};
    };

    struct WorldPlane
    {
        Vec3 centre{};
        float halfWidthX{};
        float halfLengthZ{};
        Vec3 axisX{1.0f, 0.0f, 0.0f};
        Vec3 normal{0.0f, 1.0f, 0.0f};
        Vec3 axisZ{0.0f, 0.0f, 1.0f};
    };

    struct WorldSphere
    {
        Vec3 centre{};
        float radius{};
        Vec3 axisX{ 1.0f, 0.0f, 0.0f };
        Vec3 axisY{ 0.0f, 1.0f, 0.0f };
        Vec3 axisZ{ 0.0f, 0.0f, 1.0f };
    };

    struct WorldCylinder
    {
        Vec3 centre{};
        float radius{};
        float halfHeight{};
        Vec3 axisX{ 1.0f, 0.0f, 0.0f };
        Vec3 axisY{ 0.0f, 1.0f, 0.0f };
        Vec3 axisZ{ 0.0f, 0.0f, 1.0f };
    };


    [[nodiscard]] WorldBox ToWorldShape(const BoxShape& shape,const TransformComponent& transform) noexcept;
    [[nodiscard]] WorldPlane ToWorldShape(const PlaneShape& shape,const TransformComponent& transform) noexcept;
    [[nodiscard]] WorldSphere ToWorldShape(const SphereShape& shape, const TransformComponent& transform) noexcept;
    [[nodiscard]] WorldCylinder ToWorldShape(const CylinderShape& shape, const TransformComponent& transform) noexcept;
    [[nodiscard]] AABB CalculateBounds(const WorldBox& box) noexcept;
    [[nodiscard]] AABB CalculateBounds(const WorldPlane& plane) noexcept;
    [[nodiscard]] AABB CalculateBounds(const WorldSphere& sphere) noexcept;
    [[nodiscard]] AABB CalculateBounds(const WorldCylinder& cylinder) noexcept;
}

namespace qmec
{
    using physics::WorldBox;
    using physics::WorldPlane;
    using physics::WorldSphere;
    using physics::WorldCylinder;
    using physics::ToWorldShape;
    using physics::CalculateBounds;
}
