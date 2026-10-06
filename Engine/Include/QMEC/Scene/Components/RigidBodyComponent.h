#pragma once
#include "QMEC/Math/Vec3.h"
#include "QMEC/Math/Quat.h"
#include "QMEC/Math/Mat3.h"

namespace qmec::scene::components
{
    struct PhysicsMaterial
    {
        float staticFriction{ 0.5f };
        float dynamicFriction{ 0.3f };
        float rollingResistance{0.015f};
        float Elasticity{};
    };


    struct RigidBodyComponent
    {
        Vec3 position;
        Quat rotation;
        float mass{};
        Vec3 velocity{};
        Vec3 angularVelocity{};
        Vec3 accumulatedForce{};
        Vec3 accumulatedTorque{};
        bool isKinematic{false};
        float inverseMass{};
        Mat3 inverseInertiaBody{};
        Mat3 inverseInertiaWorld{};
       

        PhysicsMaterial material{};

        // Forces accumulate until the next PhysicsWorld fixed step.
        void ApplyForce(const Vec3& force) noexcept
        {
            if (!isKinematic)
                accumulatedForce += force;
        }

        void ApplyTorque(const Vec3& torque) noexcept
        {
            if (!isKinematic)
                accumulatedTorque += torque;
        }

        // leverArm is a world-space vector from the body's center of mass to
        // the point where this world-space force is applied.
        void ApplyForceAtPoint(const Vec3& force, const Vec3& leverArm) noexcept
        {
            ApplyForce(force);
            ApplyTorque(Cross(leverArm, force));
        }

        void ClearForceAccumulators() noexcept
        {
            accumulatedForce = {};
            accumulatedTorque = {};
        }

        void ApplyImpulse(const Vec3& impulse)
        {
            velocity += impulse * inverseMass;
        }

        void ApplyAngularImpulse(const Vec3& angularImpulse) noexcept
        {
            if (!isKinematic && inverseMass > 0.0f)
            {
                angularVelocity += inverseInertiaWorld.TransformVector(angularImpulse);
            }
        }
    };
}

namespace qmec
{
    using scene::components::PhysicsMaterial;
    using scene::components::RigidBodyComponent;
}
