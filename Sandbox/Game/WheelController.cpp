#include "Game/WheelController.h"

#include "QMEC/ECS/Registry.h"
#include "QMEC/Math/Mat4.h"
#include "QMEC/Scene/Components/ColliderComponent.h"
#include "QMEC/Scene/Components/RigidBodyComponent.h"
#include "QMEC/Scene/Components/TransformComponent.h"
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

        geometryValid = std::isfinite(worldPosition.x) && std::isfinite(worldPosition.y) && std::isfinite(worldPosition.z) && std::isfinite(radius) && std::isfinite(halfWidth) &&
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
        float bestRayDistance = (std::numeric_limits<float>::max)();
        Vec3 bestPoint{};
        Vec3 bestNormal{};
        bool foundGround = false;

        const float restLength = (std::max)(0.0f, suspensionRestLength);
        const Vec3 rawSuspensionDirection = scene.GetWorldMatrix(chassisEntity).TransformDirection({ 0.0f, -1.0f, 0.0f });
        const float suspensionDirectionLength = rawSuspensionDirection.Length();
        if (suspensionDirectionLength <= directionTolerance)
            return false;
        const Vec3 suspensionDirection = rawSuspensionDirection / suspensionDirectionLength;

        const Mat4 chassisWorld = scene.GetWorldMatrix(chassisEntity);
        const Vec3 chassisPosition = scene.GetWorldPosition(chassisEntity);
        if (!suspensionAnchorInitialized_ || anchorWheelEntity_ != wheelEntity || anchorChassisEntity_ != chassisEntity)
        {
            Mat4 chassisWorldInverse{};
            if (!chassisWorld.TryInverse(chassisWorldInverse))
                return false;

            const Vec3 chassisUp = chassisWorld.TransformDirection({ 0.0f, 1.0f, 0.0f }).Normalized();
            const Vec3 anchorWorld = worldPosition + chassisUp * restLength;
            suspensionAnchorLocal_ = chassisWorldInverse.TransformDirection(anchorWorld - chassisPosition);
            anchorWheelEntity_ = wheelEntity;
            anchorChassisEntity_ = chassisEntity;
            suspensionAnchorInitialized_ = true;
        }

        const Vec3 suspensionAnchorWorld = chassisPosition + chassisWorld.TransformDirection(suspensionAnchorLocal_);
        const float maximumRayDistance = radius + restLength;

        const auto moveWheelCenter = [&](const Vec3& targetWorldCenter)
        {
            auto* wheelTransform = registry.GetComponent<TransformComponent>(wheelEntity);
            if (wheelTransform == nullptr)
                return;

            const Vec3 worldDelta = targetWorldCenter - worldPosition;
            const Entity parent = wheelTransform->GetParent();
            if (parent.IsValid() && registry.IsAlive(parent))
            {
                Mat4 parentWorldInverse{};
                if (!scene.GetWorldMatrix(parent).TryInverse(parentWorldInverse))
                    return;
                wheelTransform->position += parentWorldInverse.TransformDirection(worldDelta);
            }
            else
            {
                wheelTransform->position += worldDelta;
            }
            worldPosition = targetWorldCenter;
        };

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

            const float rayDenominator = Dot(suspensionDirection, planeNormal);
            if (std::abs(rayDenominator) <= directionTolerance)
                continue;

            const float rayDistance = Dot(planeCentre - suspensionAnchorWorld, planeNormal) / rayDenominator;
            if (rayDistance < 0.0f || rayDistance > maximumRayDistance || rayDistance >= bestRayDistance)
                continue;

            const Vec3 planePoint = suspensionAnchorWorld + suspensionDirection * rayDistance;
            const Vec3 planeOffset = planePoint - planeCentre;
            const float coordinateX = Dot(planeOffset, planeAxisX);
            const float coordinateZ = Dot(planeOffset, planeAxisZ);
            if (std::abs(coordinateX) > planeHalfWidth || std::abs(coordinateZ) > planeHalfLength)
            {
                continue;
            }

            foundGround = true;
            bestRayDistance = rayDistance;
            bestPoint = planePoint;
            bestNormal = Dot(worldPosition - planeCentre, planeNormal) >= 0.0f ? planeNormal : -planeNormal;
        }

        if (!foundGround)
        {
            moveWheelCenter(suspensionAnchorWorld + suspensionDirection * restLength);
            return false;
        }

        grounded = true;
        groundContactPoint = bestPoint;
        groundContactNormal = bestNormal;
        const float suspensionLength = bestRayDistance - radius;
        suspensionCompression = (std::clamp)(restLength - suspensionLength, 0.0f,(std::max)(0.0f, maximumSuspensionTravel));
        moveWheelCenter(suspensionAnchorWorld + suspensionDirection *(std::clamp)(suspensionLength, 0.0f, restLength));

        const Vec3 leverArm = groundContactPoint - chassisPosition;
        const Vec3 contactVelocity = chassisBody.velocity + Cross(chassisBody.angularVelocity, leverArm);
        const float velocityAlongNormal = Dot(contactVelocity, groundContactNormal);
        const float safeSpringStiffness = (std::max)(0.0f, springStiffness);
        const float safeMass = (std::max)(0.0f, chassisBody.mass);
        const float safeDampingRatio = (std::clamp)(dampingRatio, 0.0f, 2.0f);
        const float sprungMass = safeMass * 0.25f;
        const float dampingCoefficient = 2.0f * safeDampingRatio * std::sqrt(safeSpringStiffness * sprungMass);
        const float springForce = safeSpringStiffness * suspensionCompression;
        const float dampingForce = dampingCoefficient * velocityAlongNormal;
        normalForce = (std::max)(0.0f, springForce - dampingForce);

        if (!chassisBody.isKinematic && chassisBody.inverseMass > 0.0f && normalForce > 0.0f)
            chassisBody.ApplyForceAtPoint(groundContactNormal * normalForce, leverArm);

        return true;
    }
}
