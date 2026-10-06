#include "QMEC/Physics/Collider/WorldShapes.h"

#include <algorithm>
#include <cmath>

namespace qmec::physics
{
    WorldBox ToWorldShape(const BoxShape& shape,const TransformComponent& transform) noexcept
    {
        const Quat rotation = transform.rotation.Normalized();
        const Vec3 scaledCentre{shape.centre.x * transform.scale.x,shape.centre.y * transform.scale.y, shape.centre.z * transform.scale.z};

        WorldBox box;
        box.centre = transform.position + rotation.Rotate(scaledCentre);
        box.halfExtents = {std::abs(shape.halfExtents.x * transform.scale.x),std::abs(shape.halfExtents.y * transform.scale.y),std::abs(shape.halfExtents.z * transform.scale.z)};
        box.axisX = rotation.Rotate({1.0f, 0.0f, 0.0f});
        box.axisY = rotation.Rotate({0.0f, 1.0f, 0.0f});
        box.axisZ = rotation.Rotate({0.0f, 0.0f, 1.0f});
        return box;
    }

    WorldPlane ToWorldShape(const PlaneShape& shape, const TransformComponent& transform) noexcept
    {
        const WorldBox box = ToWorldShape(BoxShape{shape.centre, {shape.halfWidthX, 0.0f, shape.halfLengthZ}}, transform);
        return {box.centre, box.halfExtents.x, box.halfExtents.z,box.axisX, box.axisY, box.axisZ};
    }

    WorldSphere ToWorldShape(const SphereShape& shape,const TransformComponent& transform) noexcept
    {
         WorldSphere sphere{};
       
        const Vec3 scaledCentre{ shape.centre.x * transform.scale.x,shape.centre.y * transform.scale.y, shape.centre.z * transform.scale.z };
        const float radius = std::abs(shape.radius) * (std::max)({
            std::abs(transform.scale.x), std::abs(transform.scale.y), std::abs(transform.scale.z)});

        const Quat rotation = transform.rotation.Normalized();

        sphere.axisX = rotation.Rotate({ 1.0f, 0.0f, 0.0f });
        sphere.axisY = rotation.Rotate({ 0.0f, 1.0f, 0.0f });
        sphere.axisZ = rotation.Rotate({ 0.0f, 0.0f, 1.0f });
        sphere.centre = transform.position + rotation.Rotate(scaledCentre);
        sphere.radius = radius;

        return sphere;
    }

    WorldCylinder ToWorldShape(const CylinderShape& shape, const TransformComponent& transform) noexcept
    {
        WorldCylinder Cylinder{};

        const Vec3 scaledCentre{ shape.centre.x * transform.scale.x,shape.centre.y * transform.scale.y, shape.centre.z * transform.scale.z };
        const float radius = std::abs(shape.radius) * (std::max)({
            std::abs(transform.scale.x), std::abs(transform.scale.z) });

        const float halfHeight = std::abs(shape.halfHeight) * std::abs(transform.scale.y);

        const Quat rotation = transform.rotation.Normalized();

        Cylinder.axisX = rotation.Rotate({ 1.0f, 0.0f, 0.0f });
        Cylinder.axisY = rotation.Rotate({ 0.0f, 1.0f, 0.0f });
        Cylinder.axisZ = rotation.Rotate({ 0.0f, 0.0f, 1.0f });
        Cylinder.centre = transform.position + rotation.Rotate(scaledCentre);
        Cylinder.radius = radius;
        Cylinder.halfHeight = halfHeight;

        return Cylinder;
    }



    AABB CalculateBounds(const WorldBox& box) noexcept
    {
        const Vec3 x = box.axisX * box.halfExtents.x;
        const Vec3 y = box.axisY * box.halfExtents.y;
        const Vec3 z = box.axisZ * box.halfExtents.z;
        const Vec3 halfExtents{std::abs(x.x) + std::abs(y.x) + std::abs(z.x),
            std::abs(x.y) + std::abs(y.y) + std::abs(z.y),
            std::abs(x.z) + std::abs(y.z) + std::abs(z.z)};
        return {box.centre - halfExtents, box.centre + halfExtents};
    }

    AABB CalculateBounds(const WorldPlane& plane) noexcept
    {
        return CalculateBounds(WorldBox{plane.centre, {plane.halfWidthX, 0.0f, plane.halfLengthZ}, plane.axisX, plane.normal, plane.axisZ});
    }

    AABB CalculateBounds(const WorldSphere& sphere) noexcept
    {
        const Vec3 halfExtents{sphere.radius, sphere.radius, sphere.radius};
        return {sphere.centre - halfExtents, sphere.centre + halfExtents};
    }

    AABB CalculateBounds(const WorldCylinder& cylinder) noexcept
    {
        // Axial height plus projected circular radius along each world axis.
        const Vec3 axis = cylinder.axisY;
        const Vec3 halfExtents{
            cylinder.halfHeight * std::abs(axis.x) + cylinder.radius * std::sqrt((std::max)(0.0f, 1.0f - axis.x * axis.x)),
            cylinder.halfHeight * std::abs(axis.y) + cylinder.radius * std::sqrt((std::max)(0.0f, 1.0f - axis.y * axis.y)),
            cylinder.halfHeight * std::abs(axis.z) + cylinder.radius * std::sqrt((std::max)(0.0f, 1.0f - axis.z * axis.z))};
        return { cylinder.centre - halfExtents, cylinder.centre + halfExtents };
    }
}
