#include "QMEC/Physics/PhyscisWorld.h"
#include "QMEC/Scene/Components/RigidBodyComponent.h"
#include "QMEC/Scene/Components/ColliderComponent.h"
#include "QMEC/Scene/Components/TransformComponent.h"
#include "QMEC/ECS/Registry.h"
#include "QMEC/Physics/BroadPhase.h"
#include "QMEC/Physics/Solver/ContactSolver.h"

#include "QMEC/Physics/Collider/WorldShapes.h"
#include <variant>
#include <cmath>
#include <chrono>
#include <cstdint>
#include <cstdio>

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>

namespace qmec::physics
{
    namespace
    {
       
        Vec3 CalculateInertiaMoments(const BoxShape& shape,const TransformComponent& transform, float mass) noexcept
        {
            const Vec3 h{shape.halfExtents.x * transform.scale.x,shape.halfExtents.y * transform.scale.y,
                shape.halfExtents.z * transform.scale.z};
            return {mass * (h.y*h.y + h.z*h.z) / 3.0f,
                mass * (h.x*h.x + h.z*h.z) / 3.0f,
                mass * (h.x*h.x + h.y*h.y) / 3.0f};
        }

        Vec3 CalculateInertiaMoments(const SphereShape& shape,const TransformComponent& transform, float mass) noexcept
        {
            const float radius = ToWorldShape(shape, transform).radius;
            const float moment = (2.0f / 5.0f) * mass * radius * radius;
            return {moment, moment, moment};
        }

        Vec3 CalculateInertiaMoments(const CylinderShape& shape, const TransformComponent& transform, float mass) noexcept
        {
            const WorldCylinder cylinder = ToWorldShape(shape, transform);
            const float radiusSquared = cylinder.radius * cylinder.radius;
            const float height = 2.0f * cylinder.halfHeight;
            const float across = mass * (3.0f * radiusSquared + height * height) / 12.0f;
            const float around = 0.5f * mass * radiusSquared;
            return {across, around, across}; 
        }

        Vec3 CalculateInertiaMoments(const PlaneShape&,const TransformComponent&, float) noexcept
        {
            return {};
        }
    }

    void PhysicsWorld::UpdateInverseInertia(RigidBodyComponent& body,
        const ColliderComponent* collider, const TransformComponent& transform) const noexcept
    {
        body.inverseInertiaBody = {};
        body.inverseInertiaWorld = {};
        if (!collider || body.isKinematic || body.inverseMass <= 0.0f || !std::isfinite(body.mass) || body.mass <= 0.0f)
            return;
       
        const Vec3 localMoments = std::visit([&](const auto& shape)
            {
                return CalculateInertiaMoments(shape, transform, body.mass);
            }, collider->shape);
        const float moments[]{localMoments.x, localMoments.y, localMoments.z};
        for (int axis = 0; axis < 3; ++axis)
            if (std::isfinite(moments[axis]) && moments[axis] > 0.00000001f)
                body.inverseInertiaBody.values[axis][axis] = 1.0f / moments[axis];

        const Quat rotation = transform.rotation.Normalized();
        const Vec3 x = rotation.Rotate({1,0,0});
        const Vec3 y = rotation.Rotate({0,1,0});
        const Vec3 z = rotation.Rotate({0,0,1});
        const Mat3 r{{{x.x,y.x,z.x}, {x.y,y.y,z.y}, {x.z,y.z,z.z}}};

        for (int row = 0; row < 3; ++row)
            for (int col = 0; col < 3; ++col)
                for (int axis = 0; axis < 3; ++axis)
                    body.inverseInertiaWorld.values[row][col] +=
                        r.values[row][axis] * body.inverseInertiaBody.values[axis][axis] * r.values[col][axis];
    }

    void PhysicsWorld::UpdateColliderBounds(ColliderComponent& collider,const TransformComponent& transform) const noexcept
    {
        collider.worldBounds = std::visit([&](const auto& shape)
            {
                return CalculateWorldBounds(shape, transform); 
            }, collider.shape);
    }

    AABB PhysicsWorld::CalculateWorldBounds(const BoxShape& shape,const TransformComponent& transform) const noexcept
    {
        return CalculateBounds(ToWorldShape(shape, transform));
    }

    AABB PhysicsWorld::CalculateWorldBounds(const PlaneShape& shape,const TransformComponent& transform) const noexcept
    {
        return CalculateBounds(ToWorldShape(shape, transform));
    }

    AABB PhysicsWorld::CalculateWorldBounds(const SphereShape& shape,const TransformComponent& transform) const noexcept
    {
        return CalculateBounds(ToWorldShape(shape, transform));
    }

    AABB PhysicsWorld::CalculateWorldBounds(const CylinderShape& shape, const TransformComponent& transform) const noexcept
    {
        return CalculateBounds(ToWorldShape(shape, transform));
    }

    void PhysicsWorld::Simulate(float frameDt)
    {
        accumulator += frameDt;
        while (accumulator >= fixedDeltaTime)
        {
            step(fixedDeltaTime);
            accumulator -= fixedDeltaTime;
        }
    }

    void PhysicsWorld::step(float dt)
    {
        using Clock = std::chrono::steady_clock;
        const auto stepStart = Clock::now();
        static std::uint64_t stepCount = 0;
        const std::uint64_t currentStep = ++stepCount;
        const bool traceThisStep = currentStep % 60U == 0U;
        const auto logCheckpoint = [traceThisStep, currentStep, stepStart](
            const char* phase,
            size_t bodyCount,
            size_t colliderCount,
            size_t pairCount,
            size_t narrowPairCount,
            size_t manifoldCount)
        {
            if (!traceThisStep)
            {
                return;
            }

            const double elapsedMs = std::chrono::duration<double, std::milli>(
                Clock::now() - stepStart).count();
            char message[256]{};
            std::snprintf(message, sizeof(message),
                "[Physics step %llu] phase=%s bodies=%zu colliders=%zu pairs=%zu narrowPairs=%zu manifolds=%zu elapsed=%.3fms\n",
                static_cast<unsigned long long>(currentStep), phase,
                bodyCount, colliderCount, pairCount, narrowPairCount, manifoldCount, elapsedMs);
            OutputDebugStringA(message);
        };

        logCheckpoint("begin", 0, 0, 0, 0, 0);
        auto rigidBodies = registry_.GetEntitiesWith<RigidBodyComponent>();

        for (Entity entity : rigidBodies)
        {
            
            auto* body = registry_.GetComponent<RigidBodyComponent>(entity);
            auto* collider = registry_.GetComponent<ColliderComponent>(entity);
            auto* transform = registry_.GetComponent<TransformComponent>(entity);

            if (body == nullptr)
                continue;

            if (transform == nullptr)
            {
                body->ClearForceAccumulators();
                continue;
            }

            UpdateInverseInertia(*body, collider, *transform);
            if (body->isKinematic)
            {
                body->ClearForceAccumulators();
                continue;
            }

            body->velocity += gravity_ * dt;
            if (body->inverseMass > 0.0f)
            {
                body->velocity += body->accumulatedForce * (body->inverseMass * dt);
                body->angularVelocity += body->inverseInertiaWorld.TransformVector(
                    body->accumulatedTorque) * dt;
            }
            body->ClearForceAccumulators();
            transform->position += body->velocity * dt;

            const float angularSpeed = body->angularVelocity.Length();
            if (angularSpeed > 0.000001f)
            {
                const Vec3 axis = body->angularVelocity * (1.0f / angularSpeed);
                const Quat deltaRotation = Quat::FromAxisAngle(axis, angularSpeed * dt);
                transform->rotation = (deltaRotation * transform->rotation).Normalized();
                UpdateInverseInertia(*body, collider, *transform);
            }

        }
        logCheckpoint("bodies-integrated", rigidBodies.size(), 0, 0, 0, 0);
        auto colliders = registry_.GetEntitiesWith<ColliderComponent>();
        std::vector<BroadPhaseCollider> broadPhaseList;
        broadPhaseList.reserve(colliders.size());

        for (Entity entity : colliders)
        {
            auto* collider = registry_.GetComponent<ColliderComponent>(entity);
            const auto* transform = registry_.GetComponent<TransformComponent>(entity);
            if (collider == nullptr || transform == nullptr)
                continue;

            UpdateColliderBounds(*collider, *transform);
            // Trigger colliders are sensors; they are not submitted to the physical solver.
            if (collider->isTrigger)
                continue;

            broadPhaseList.push_back({entity, collider->worldBounds});
        }
        logCheckpoint("bounds-updated", rigidBodies.size(), broadPhaseList.size(), 0, 0, 0);

        std::vector<CollisionPair> collisionPairs = broadPhase_.FindPotentialPairs(broadPhaseList);
        logCheckpoint("broadphase-complete", rigidBodies.size(), broadPhaseList.size(), collisionPairs.size(), 0, 0);

        std::vector<NarrowPhaseColliderPairs> narrowPhasePairs;
        narrowPhasePairs.reserve(collisionPairs.size());

        for (const CollisionPair& pair : collisionPairs)
        {
             auto* firstCollider = registry_.GetComponent<ColliderComponent>(pair.first);
             auto* firstTransform = registry_.GetComponent<TransformComponent>(pair.first);
             auto* firstRigidbody = registry_.GetComponent<RigidBodyComponent>(pair.first);
             auto* secondCollider = registry_.GetComponent<ColliderComponent>(pair.second);
             auto* secondTransform = registry_.GetComponent<TransformComponent>(pair.second);
             auto* secondRigidbody = registry_.GetComponent<RigidBodyComponent>(pair.second);

            if (firstCollider == nullptr || firstTransform == nullptr || secondCollider == nullptr || secondTransform == nullptr)
            {
                continue;
            }

            narrowPhasePairs.push_back({{pair.first, firstCollider, firstTransform,firstRigidbody}, {pair.second, secondCollider, secondTransform,secondRigidbody}});
        }
        logCheckpoint("pairs-ready", rigidBodies.size(), broadPhaseList.size(), collisionPairs.size(), narrowPhasePairs.size(), 0);
        std::vector<ContactManifold> manifolds;
        narrowPhase_.findNarrowPhasePairs(narrowPhasePairs, manifolds);
        logCheckpoint("narrowphase-complete", rigidBodies.size(), broadPhaseList.size(), collisionPairs.size(), narrowPhasePairs.size(), manifolds.size());
        for (const ContactManifold& manifold : manifolds)
        {
            solver_.Solve(manifold);
        }
        logCheckpoint("solver-complete", rigidBodies.size(), broadPhaseList.size(), collisionPairs.size(), narrowPhasePairs.size(), manifolds.size());

    }
}
