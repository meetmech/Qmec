#include "Game/WheelController.h"

#include "QMEC/ECS/Registry.h"
#include "QMEC/Math/Mat4.h"
#include "QMEC/Scene/Components/ColliderComponent.h"
#include "QMEC/Scene/Components/RigidBodyComponent.h"
#include "QMEC/Scene/Scene.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <variant>

namespace qmec::game
{
    bool WheelController::RefreshColliderState(const Scene& scene, Entity wheelEntity) noexcept
    {
        entity = wheelEntity;
        worldPosition = {};
        axleAxis = { 0.0f, 1.0f, 0.0f };
        radius = 0.0f;
        halfWidth = 0.0f;
        geometryValid = false;

        const auto& registry = scene.GetRegistry();
        if (!registry.IsAlive(wheelEntity))
        {
            grounded = false;
            return false;
        }

        const auto* collider = registry.GetComponent<ColliderComponent>(wheelEntity);
        if (collider == nullptr)
        {
            grounded = false;
            return false;
        }

        const auto* cylinder = std::get_if<CylinderShape>(&collider->shape);
        if (cylinder == nullptr)
        {
            grounded = false;
            return false;
        }

        const Mat4 world = scene.GetWorldMatrix(wheelEntity);
        const Vec3 localX = world.TransformDirection({ 1.0f, 0.0f, 0.0f });
        const Vec3 localY = world.TransformDirection({ 0.0f, 1.0f, 0.0f });
        const Vec3 localZ = world.TransformDirection({ 0.0f, 0.0f, 1.0f });

        worldPosition = world.TransformDirection(cylinder->centre) + scene.GetWorldPosition(wheelEntity);
        axleAxis = localY.Normalized();
        radius = std::abs(cylinder->radius) * (std::max)(localX.Length(), localZ.Length());
        halfWidth = std::abs(cylinder->halfHeight) * localY.Length();

        geometryValid = std::isfinite(worldPosition.x) && std::isfinite(worldPosition.y) && std::isfinite(worldPosition.z) &&
            std::isfinite(radius) && std::isfinite(halfWidth) &&
            radius > 0.0f && halfWidth > 0.0f && axleAxis.LengthSquared() > 0.0f;

        if (!geometryValid)
            grounded = false;

        return geometryValid;
    }

    bool WheelController::UpdateGroundContact( Scene& scene,Entity wheelEntity, Entity chassisEntity,RigidBodyComponent& chassisBody) noexcept
    {
        grounded = false;
        normalForce = 0.0f;
        suspensionCompression = 0.0f;
        groundContactPoint = {};
        groundContactNormal = { 0.0f, 1.0f, 0.0f };

        if (!RefreshColliderState(scene, wheelEntity))
            return false;

        Registry& registry = scene.GetRegistry();
        auto* wheelCollider = registry.GetComponent<ColliderComponent>(wheelEntity);
        if (wheelCollider == nullptr)
            return false;

   
        wheelCollider->isTrigger = true;

        constexpr float directionTolerance = 1.0e-6f;
        float bestGap = (std::numeric_limits<float>::max)();
        Vec3 bestPoint{};
        Vec3 bestNormal{};
        bool foundGround = false;

        const float restLength = (std::max)(0.0f, suspensionRestLength);
        for (Entity groundEntity : registry.GetEntitiesWith<ColliderComponent>())
        {
            if (groundEntity == wheelEntity || groundEntity == chassisEntity)
                continue;

            const auto* groundCollider = registry.GetComponent<ColliderComponent>(groundEntity);
            if (groundCollider == nullptr)
                continue;

            const auto* plane = std::get_if<PlaneShape>(&groundCollider->shape);
            if (plane == nullptr)
                continue;

            const Mat4 planeWorld = scene.GetWorldMatrix(groundEntity);
            const Vec3 rawAxisX = planeWorld.TransformDirection({ 1.0f, 0.0f, 0.0f });
            const Vec3 rawNormal = planeWorld.TransformDirection({ 0.0f, 1.0f, 0.0f });
            const Vec3 rawAxisZ = planeWorld.TransformDirection({ 0.0f, 0.0f, 1.0f });
            const float axisXLength = rawAxisX.Length();
            const float normalLength = rawNormal.Length();
            const float axisZLength = rawAxisZ.Length();
            if (axisXLength <= directionTolerance || normalLength <= directionTolerance || axisZLength <= directionTolerance)
                continue;

            const Vec3 planeCentre = planeWorld.TransformDirection(plane->centre) + scene.GetWorldPosition(groundEntity);
            const Vec3 planeAxisX = rawAxisX / axisXLength;
            const Vec3 planeNormal = rawNormal / normalLength;
            const Vec3 planeAxisZ = rawAxisZ / axisZLength;
            const float planeHalfWidth = std::abs(plane->halfWidthX) * axisXLength;
            const float planeHalfLength = std::abs(plane->halfLengthZ) * axisZLength;

            const float signedDistance = Dot(worldPosition - planeCentre, planeNormal);
            const Vec3 contactNormal = signedDistance >= 0.0f ? planeNormal : -planeNormal;
            const float axisDotNormal = Dot(axleAxis, contactNormal);
            const float radialFraction = std::sqrt((std::max)(0.0f, 1.0f - axisDotNormal * axisDotNormal));
            const float supportExtent = halfWidth * std::abs(axisDotNormal) + radius * radialFraction;
            const float gap = std::abs(signedDistance) - supportExtent;
            if (gap > restLength || gap >= bestGap)
                continue;

            // Find the cylinder's support point facing the plane, then project it onto the plane.
            const Vec3 towardPlane = -contactNormal;
            const float axialDirection = Dot(axleAxis, towardPlane);
            Vec3 supportPoint = worldPosition;
            if (axialDirection > directionTolerance)
                supportPoint += axleAxis * halfWidth;
            else if (axialDirection < -directionTolerance)
                supportPoint -= axleAxis * halfWidth;

            const Vec3 radialDirection = towardPlane - axleAxis * axialDirection;
            const float radialLength = radialDirection.Length();
            if (radialLength > directionTolerance)
                supportPoint += radialDirection * (radius / radialLength);

            const Vec3 planePoint = supportPoint - planeNormal * Dot(supportPoint - planeCentre, planeNormal);
            const Vec3 planeOffset = planePoint - planeCentre;
            const float coordinateX = Dot(planeOffset, planeAxisX);
            const float coordinateZ = Dot(planeOffset, planeAxisZ);
            const float edgeAllowance = radius + halfWidth;
            if (std::abs(coordinateX) > planeHalfWidth + edgeAllowance || std::abs(coordinateZ) > planeHalfLength + edgeAllowance)
            {
                continue;
            }

            foundGround = true;
            bestGap = gap;
            bestPoint = planePoint;
            bestNormal = contactNormal;
        }

        if (!foundGround)
            return false;

        grounded = true;
        groundContactPoint = bestPoint;
        groundContactNormal = bestNormal;
        suspensionCompression = (std::clamp)(restLength - bestGap,0.0f, (std::max)(0.0f, maximumSuspensionTravel));

        const Vec3 chassisPosition = scene.GetWorldPosition(chassisEntity);
        const Vec3 leverArm = groundContactPoint - chassisPosition;
        const Vec3 contactVelocity = chassisBody.velocity + Cross(chassisBody.angularVelocity, leverArm);
        const float velocityAlongNormal = Dot(contactVelocity, groundContactNormal);
        const float springForce = springStiffness * suspensionCompression;
        const float dampingForce = damperCoefficient * velocityAlongNormal;
        normalForce = (std::max)(0.0f, springForce - dampingForce);

        if (!chassisBody.isKinematic && chassisBody.inverseMass > 0.0f && normalForce > 0.0f)
            chassisBody.ApplyForceAtPoint(groundContactNormal * normalForce, leverArm);

        return true;
    }
}
