#include "Game/WheelController.h"

#include "QMEC/ECS/Registry.h"
#include "QMEC/Math/Mat4.h"
#include "QMEC/Scene/Components/ColliderComponent.h"
#include "QMEC/Scene/Components/RigidBodyComponent.h"
#include "QMEC/Scene/Components/TransformComponent.h"
#include "QMEC/Scene/Scene.h"

#include <algorithm>
#include <numbers>
#include <cmath>
#include <limits>
#include <variant>
#include <iostream>

namespace qmec::game
{
    void WheelController::Initialize(const ScriptContext& context,Entity WheelEntity,Entity chasisentity)
    {
        wheelEntity = WheelEntity;
        scene = &context.scene;
        registry = &context.scene.GetRegistry();
        wheelTransform_ = registry->GetComponent<TransformComponent>(wheelEntity);
        chassisEntity = chasisentity;
        chasisRigidbody = registry->GetComponent<RigidBodyComponent>(chasisentity);
        if (wheelTransform_ == nullptr)
            return;

        constexpr float minimumRadius = 1.0e-5f;

         safeRadius = (std::max)(radius, minimumRadius);
         grip = (std::max)(0.0f, grip);
        initialRotation = wheelTransform_->rotation;
        wheelSpinAngle_ = 0.0f;
        if (!RefreshColliderState())
        {
            __debugbreak();
        }
            
    }

    void WheelController::UpdateInertiaRotation(float deltaTime, float vehicleSpeed)
    {
        Vec3 forward = Cross(axleAxis, groundContactNormal);
        const float forwardLength = forward.Length();
        float angularVel = vehicleSpeed / radius;
        wheelSpinAngle_ += angularVel * deltaTime;

        const Quat steering = Quat::FromAxisAngle({ 0.0f,0.0f, 1.0f }, std::numbers::pi_v<float> *currentSteeringAngle / 180.0f);
        const Quat spin = Quat::FromAxisAngle({ 0.0f, 1.0f, 0.0f }, wheelSpinAngle_);
        wheelTransform_->rotation = (steering * initialRotation * spin).Normalized();

    }

    bool WheelController::RefreshColliderState() noexcept
    {
   
        worldPosition = {};
        axleAxis = { 0.0f, 1.0f, 0.0f };
        radius = 0.0f;
        halfWidth = 0.0f;
        geometryValid = false;



        if (!registry->IsAlive(wheelEntity))
        {
            grounded = false;
            return false;
        }

        const auto* collider = registry->GetComponent<ColliderComponent>(wheelEntity);

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

        const Mat4 world = scene->GetWorldMatrix(wheelEntity);

        const Vec3 localX = world.TransformDirection({ 1.0f, 0.0f, 0.0f });
        const Vec3 localY = world.TransformDirection({ 0.0f, 1.0f, 0.0f });
        const Vec3 localZ = world.TransformDirection({ 0.0f, 0.0f, 1.0f });

        worldPosition = world.TransformDirection(cylinder->centre) +scene->GetWorldPosition(wheelEntity);

        axleAxis = localY.Normalized();

        radius = std::abs(cylinder->radius) *(std::max)(localX.Length(), localZ.Length());

        halfWidth = std::abs(cylinder->halfHeight) * localY.Length();

        geometryValid =std::isfinite(worldPosition.x) &&std::isfinite(worldPosition.y) &&std::isfinite(worldPosition.z) &&std::isfinite(radius) &&std::isfinite(halfWidth) &&
            radius > 0.0f &&
            halfWidth > 0.0f &&
            axleAxis.LengthSquared() > 0.0f;

        if (!geometryValid)
            grounded = false;

        return geometryValid;
        
    }

    bool WheelController::UpdateGroundContact() noexcept
    {
        grounded = false;
        normalForce = 0.0f;
        suspensionCompression = 0.0f;
        groundContactPoint = {};
        groundContactNormal = { 0.0f, 1.0f, 0.0f };

        if (!RefreshColliderState())
            return false;

        if (wheelTransform_ == nullptr)
            return false;

        auto* wheelCollider = registry->GetComponent<ColliderComponent>(wheelEntity);

        if (wheelCollider == nullptr)
            return false;

        wheelCollider->isTrigger = true;

        constexpr float directionTolerance = 1.0e-6f;

        float bestRayDistance = (std::numeric_limits<float>::max)();

        Vec3 bestPoint{};
        Vec3 bestNormal{};

        bool foundGround = false;

        const float restLength = (std::max)(0.0f, suspensionRestLength);

        const Mat4 chassisWorld = scene->GetWorldMatrix(chassisEntity);
        const Vec3 chassisPosition = scene->GetWorldPosition(chassisEntity);

        const Vec3 rawSuspensionDirection = chassisWorld.TransformDirection({ 0.0f, -1.0f, 0.0f });

        const float suspensionDirectionLength = rawSuspensionDirection.Length();

        if (suspensionDirectionLength <= directionTolerance)
            return false;

        const Vec3 suspensionDirection = rawSuspensionDirection / suspensionDirectionLength;

        if (!suspensionAnchorInitialized_ ||anchorWheelEntity_ != wheelEntity ||anchorChassisEntity_ != chassisEntity)
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
                const Vec3 worldDelta = targetWorldCenter - worldPosition;
                const Entity parent = wheelTransform_->GetParent();

                if (parent.IsValid() && registry->IsAlive(parent))
                {
                    Mat4 parentWorldInverse{};

                    if (!scene->GetWorldMatrix(parent).TryInverse(parentWorldInverse))
                        return;

                    wheelTransform_->position += parentWorldInverse.TransformDirection(worldDelta);
                }
                else
                {
                    wheelTransform_->position += worldDelta;
                }

                worldPosition = targetWorldCenter;
            };
        Entity ground{};
        for (Entity groundEntity : registry->GetEntitiesWith<ColliderComponent>())
        {
            if (groundEntity == wheelEntity || groundEntity == chassisEntity)
                continue;

            const auto* groundCollider = registry->GetComponent<ColliderComponent>(groundEntity);

            if (groundCollider == nullptr)
                continue;

            const auto* plane = std::get_if<PlaneShape>(&groundCollider->shape);

            if (plane == nullptr)
                continue;

            ground = groundEntity;

            const Mat4 planeWorld = scene->GetWorldMatrix(groundEntity);


            const Vec3 rawAxisX = planeWorld.TransformDirection({ 1.0f, 0.0f, 0.0f });

            const Vec3 rawNormal = planeWorld.TransformDirection({ 0.0f, 1.0f, 0.0f });

            const Vec3 rawAxisZ = planeWorld.TransformDirection({ 0.0f, 0.0f, 1.0f });

            const float axisXLength = rawAxisX.Length();
            const float normalLength = rawNormal.Length();
            const float axisZLength = rawAxisZ.Length();

            if (axisXLength <= directionTolerance ||normalLength <= directionTolerance ||axisZLength <= directionTolerance)
                continue;

            const Vec3 planeCentre =planeWorld.TransformDirection(plane->centre) +scene->GetWorldPosition(groundEntity);

            const Vec3 planeAxisX = rawAxisX / axisXLength;
            const Vec3 planeNormal = rawNormal / normalLength;
            const Vec3 planeAxisZ = rawAxisZ / axisZLength;

            const float planeHalfWidth =std::abs(plane->halfWidthX) * axisXLength;

            const float planeHalfLength =std::abs(plane->halfLengthZ) * axisZLength;

            const float rayDenominator =Dot(suspensionDirection, planeNormal);

            if (std::abs(rayDenominator) <= directionTolerance)
                continue;

            const float rayDistance =Dot(planeCentre - suspensionAnchorWorld, planeNormal) /rayDenominator;

            if (rayDistance < 0.0f ||rayDistance > maximumRayDistance ||rayDistance >= bestRayDistance)
                continue;

            const Vec3 planePoint =suspensionAnchorWorld + suspensionDirection * rayDistance;

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

        suspensionCompression =(std::clamp)(restLength - suspensionLength,0.0f,(std::max)(0.0f, maximumSuspensionTravel));

        moveWheelCenter(suspensionAnchorWorld +suspensionDirection * (std::clamp)(suspensionLength, 0.0f, restLength));

        const Vec3 leverArm = groundContactPoint - chassisPosition;

        const Vec3 contactVelocity = chasisRigidbody->velocity + Cross(chasisRigidbody->angularVelocity, leverArm);

        const float velocityAlongNormal = Dot(contactVelocity, groundContactNormal);
        const float safeSpringStiffness = (std::max)(0.0f, springStiffness);
        const float safeDampingRatio = (std::clamp)(dampingRatio, 0.0f, 2.0f);
        const float sprungMass = chasisRigidbody->mass * 0.25f;
        const float dampingCoefficient = 2.0f * safeDampingRatio *std::sqrt(safeSpringStiffness * sprungMass);
        const float springForce = safeSpringStiffness * suspensionCompression;
        const float dampingForce = dampingCoefficient * velocityAlongNormal;

        normalForce = (std::max)(0.0f, springForce - dampingForce);
        float staticMu = registry->GetComponent<RigidBodyComponent>(ground)->material.staticFriction;
        float dynamicMu = registry->GetComponent<RigidBodyComponent>(ground)->material.dynamicFriction;
         maxStaticFriction = staticMu * normalForce * grip;
        maxDynamicFriction = dynamicMu * normalForce * grip;

        if (!chasisRigidbody->isKinematic && chasisRigidbody->inverseMass > 0.0f && normalForce > 0.0f)
        {
            chasisRigidbody->ApplyForceAtPoint(groundContactNormal * normalForce,leverArm);
        }
      
        return true;
    }

    void WheelController::ApplySteering(float targetAngle,float deltaTime,float steeringStrength)
    {
        if (wheelTransform_ == nullptr)
            return;

        currentSteeringAngle = std::lerp(currentSteeringAngle,targetAngle,std::clamp(steeringStrength * deltaTime, 0.0f, 1.0f));
        const Quat steering = Quat::FromAxisAngle({ 0.0f, 0.0f, 1.0f },std::numbers::pi_v<float> *currentSteeringAngle / 180.0f);
      //  const Quat spin = Quat::FromAxisAngle({ 0.0f, 1.0f, 0.0f },wheelSpinAngle_);
        wheelTransform_->rotation = (initialRotation * steering ).Normalized();
    }

    void WheelController::ApplyDriveTorque(float torque,float deltaTime,const Vec3& chassisWorldPosition,RigidBodyComponent& chassisBody) noexcept
    {
        if (wheelTransform_ == nullptr ||deltaTime <= 0.0f ||!std::isfinite(deltaTime) ||!std::isfinite(torque) ||chassisBody.isKinematic)
        {
            return;
        }

        constexpr float minimumInertia = 1.0e-5f;
        const float safeInertia = (std::max)(angularInertia, minimumInertia);
        const float brakeAngularChange = (std::min)(std::abs(angularVelocity),(std::max)(0.0f, brakeTorque) *deltaTime / safeInertia);

        if (brakeAngularChange > 0.0f)
        {
            angularVelocity -= std::copysign(brakeAngularChange, angularVelocity);
        }

        angularVelocity += torque * deltaTime / safeInertia;

        if (grounded && normalForce > 0.0f && chassisBody.inverseMass > 0.0f)
        {
            surfaceSpeed = angularVelocity * safeRadius;
            slipVelocity = surfaceSpeed;
            lateralSlipVelocity = 0.0f;
            slipping = false;
        }


        wheelSpinAngle_ += angularVelocity * deltaTime;
        const Quat steering = Quat::FromAxisAngle({ 0.0f,0.0f, 1.0f },std::numbers::pi_v<float> *currentSteeringAngle / 180.0f);
        const Quat spin = Quat::FromAxisAngle({ 0.0f, 1.0f, 0.0f },wheelSpinAngle_);
        wheelTransform_->rotation = (steering * initialRotation * spin).Normalized();
    }

    void WheelController::OnUpdate(const ScriptContext& context)
    {
       // UpdateGroundContact();
      //  ApplyTireForces(context.deltaTime);
    }

    void WheelController::ApplyTireForces(float deltaTime)
    {
        constexpr float minimumRadius = 1.0e-5f;
        constexpr float minimumInertia = 1.0e-5f;
        const float safeRadius = (std::max)(radius, minimumRadius);
        const float safeInertia = (std::max)(angularInertia, minimumInertia);

        Vec3 forward = Cross(axleAxis, groundContactNormal);

        const float forwardLength = forward.Length();

        if (forwardLength > minimumRadius)
        {
            forward = forward / forwardLength;
            Vec3 lateral = Cross(groundContactNormal, forward);
            const float lateralLength = lateral.Length();
            if (lateralLength > minimumRadius)
            {
                lateral = lateral / lateralLength;
                const Vec3 leverArm = groundContactPoint - chasisRigidbody->position;
                const Vec3 chassisContactVelocity = chasisRigidbody->velocity + Cross(chasisRigidbody->angularVelocity, leverArm);
                surfaceSpeed = angularVelocity * safeRadius;
                slipVelocity = Dot(chassisContactVelocity, forward) - surfaceSpeed;
                lateralSlipVelocity = Dot(chassisContactVelocity, lateral);
                const auto calculateInverseMass = [&](const Vec3& direction)
                    {
                        const Vec3 armCrossDirection = Cross(leverArm, direction);
                        const Vec3 angularResponse = chasisRigidbody->inverseInertiaWorld.TransformVector(armCrossDirection);
                        const float angularTerm = Dot(Cross(angularResponse, leverArm), direction);
                        return chasisRigidbody->inverseMass + (std::max)(0.0f, angularTerm);
                    };

                const float longitudinalInverseMass = calculateInverseMass(forward) + (safeRadius * safeRadius) / safeInertia;
                const float lateralInverseMass = calculateInverseMass(lateral);

                if (longitudinalInverseMass > 0.0f && lateralInverseMass > 0.0f)
                {
                    constexpr float relaxation = 0.17f;
                    float longitudinalImpulse = -relaxation * slipVelocity / longitudinalInverseMass;
                    float lateralImpulse = -relaxation * lateralSlipVelocity / lateralInverseMass;


                    const float maximumStaticImpulse = maxStaticFriction * deltaTime;
                    const float maximumDynamicImpulse = maxDynamicFriction * deltaTime;
                    const float desiredImpulseMagnitude = std::sqrt(longitudinalImpulse * longitudinalImpulse + lateralImpulse * lateralImpulse);

                    // Longitudinal and lateral grip share one friction circle.
                    if (desiredImpulseMagnitude > maximumStaticImpulse && desiredImpulseMagnitude > minimumRadius)
                    {
                        const float dynamicScale = maximumDynamicImpulse / desiredImpulseMagnitude;

                        longitudinalImpulse *= dynamicScale;
                        lateralImpulse *= dynamicScale;
                    }
                   
                    const Vec3 tireImpulse = forward * longitudinalImpulse + lateral * lateralImpulse;
                    const Vec3 appliedTorque = Cross(leverArm, tireImpulse / deltaTime);
                    std::clog << "[Tire] wheel=" << wheelEntity.index
                        << " leverArm=(" << leverArm.x << "," << leverArm.y << "," << leverArm.z << ")"
                        << " tireForce=(" << (tireImpulse / deltaTime).x << "," << (tireImpulse / deltaTime).y << "," << (tireImpulse / deltaTime).z << ")"
                        << " yawTorque=" << appliedTorque.y << '\n';
                    chasisRigidbody->ApplyForceAtPoint(tireImpulse / deltaTime, leverArm);
                    angularVelocity -= longitudinalImpulse * safeRadius / safeInertia;
                    const float totalSlip = std::sqrt(slipVelocity * slipVelocity + lateralSlipVelocity * lateralSlipVelocity);
                    slipping = totalSlip > 0.05f && desiredImpulseMagnitude > maximumStaticImpulse;
                }
            }
            else
            {
                lateralSlipVelocity = 0.0f;
            }
        }

    }

  
}