#include "QMEC/Physics/PhyscisWorld.h"
#include "QMEC/Scene/Components/RigidBodyComponent.h"
#include "QMEC/Scene/Components/ColliderComponent.h"
#include "QMEC/Scene/Components/TransformComponent.h"
#include "QMEC/Scene/Components/NameComponent.h"
#include "QMEC/ECS/Registry.h"
#include "QMEC/Physics/BroadPhase.h"
#include "QMEC/Physics/Solver/ContactSolver.h"

#include "QMEC/Physics/Collider/WorldShapes.h"
#include "QMEC/Math/Mat4.h"
#include <variant>
#include <cmath>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <unordered_map>

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>
#include <iostream>

namespace qmec::physics
{
    namespace
    {
        Mat4 BuildLocalMatrix(const TransformComponent& transform) noexcept
        {
            Mat4 scale = Mat4::Identity();
            scale.values[0][0] = transform.scale.x;
            scale.values[1][1] = transform.scale.y;
            scale.values[2][2] = transform.scale.z;
            const Quat& q = transform.rotation;

            return scale * transform.rotation.Normalized().ToMat4() * Mat4::Translation(transform.position.x, transform.position.y, transform.position.z);
        }

        TransformComponent GetWorldTransform(const Registry& registry, Entity entity) noexcept
        {
            Mat4 worldMatrix = Mat4::Identity();
            Entity current = entity;
            const std::size_t maximumDepth = registry.GetLivingEntityCount();
            std::size_t depth = 0U;

            while (current.IsValid() && registry.IsAlive(current))
            {
                if (depth++ >= maximumDepth)
                    return {};

                const auto* transform = registry.GetComponent<TransformComponent>(current);
                if (transform == nullptr)
                    break;

                const auto* name = registry.GetComponent<NameComponent>(current);

                const Quat& q = transform->rotation;

                worldMatrix = worldMatrix * BuildLocalMatrix(*transform);
                current = transform->GetParent();
            }

            Vec3 rows[3]
            {
                {worldMatrix.values[0][0], worldMatrix.values[0][1], worldMatrix.values[0][2]},
                {worldMatrix.values[1][0], worldMatrix.values[1][1], worldMatrix.values[1][2]},
                {worldMatrix.values[2][0], worldMatrix.values[2][1], worldMatrix.values[2][2]}
            };

            Vec3 scale{rows[0].Length(), rows[1].Length(), rows[2].Length()};
            if (scale.x <= 1.0e-6f || scale.y <= 1.0e-6f || scale.z <= 1.0e-6f)
                return {};

            if (Dot(rows[0], Cross(rows[1], rows[2])) < 0.0f)
                scale.x = -scale.x;
            rows[0] = rows[0] / scale.x;
            rows[1] = rows[1] / scale.y;
            rows[2] = rows[2] / scale.z;

            const float m00 = rows[0].x;
            const float m01 = rows[1].x;
            const float m02 = rows[2].x;
            const float m10 = rows[0].y;
            const float m11 = rows[1].y;
            const float m12 = rows[2].y;
            const float m20 = rows[0].z;
            const float m21 = rows[1].z;
            const float m22 = rows[2].z;
            const float trace = m00 + m11 + m22;

            Quat rotation{};
            if (trace > 0.0f)
            {
                const float s = std::sqrt(trace + 1.0f) * 2.0f;
                rotation = {(m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s, 0.25f * s};
            }
            else if (m00 > m11 && m00 > m22)
            {
                const float s = std::sqrt((std::max)(0.0f, 1.0f + m00 - m11 - m22)) * 2.0f;
                if (s <= 1.0e-6f) return {};
                rotation = {0.25f * s, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s};
            }
            else if (m11 > m22)
            {
                const float s = std::sqrt((std::max)(0.0f, 1.0f + m11 - m00 - m22)) * 2.0f;
                if (s <= 1.0e-6f) return {};
                rotation = {(m01 + m10) / s, 0.25f * s, (m12 + m21) / s, (m02 - m20) / s};
            }
            else
            {
                const float s = std::sqrt((std::max)(0.0f, 1.0f + m22 - m00 - m11)) * 2.0f;
                if (s <= 1.0e-6f) return {};
                rotation = {(m02 + m20) / s, (m12 + m21) / s, 0.25f * s, (m10 - m01) / s};
            }

            TransformComponent result{};
            result.position = {worldMatrix.values[3][0], worldMatrix.values[3][1], worldMatrix.values[3][2]};
            result.rotation = rotation.Normalized();
            result.scale = scale;
            return result;
        }

        Entity FindRigidBodyOwner(const Registry& registry, Entity colliderEntity) noexcept
        {
            Entity current = colliderEntity;
            const std::size_t maximumDepth = registry.GetLivingEntityCount();
            std::size_t depth = 0U;
            while (current.IsValid() && registry.IsAlive(current) && depth++ < maximumDepth)
            {
                if (registry.HasComponent<RigidBodyComponent>(current))
                    return current;

                const auto* transform = registry.GetComponent<TransformComponent>(current);
                if (transform == nullptr)
                    break;
                current = transform->GetParent();
            }
            return {};
        }

        Vec3 CalculateInertiaMoments(const BoxShape& shape,const TransformComponent& transform, float mass) noexcept
        {
            const Vec3 h{shape.halfExtents.x * transform.scale.x,shape.halfExtents.y * transform.scale.y, shape.halfExtents.z * transform.scale.z};
            return {mass * (h.y*h.y + h.z*h.z) / 3.0f,mass * (h.x*h.x + h.z*h.z) / 3.0f,mass * (h.x*h.x + h.y*h.y) / 3.0f};
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
        const auto logCheckpoint = [traceThisStep, currentStep, stepStart](const char* phase,size_t bodyCount,size_t colliderCount,size_t pairCount,size_t narrowPairCount,size_t manifoldCount)
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

            TransformComponent worldTransform = GetWorldTransform(registry_, entity);
            UpdateInverseInertia(*body, collider, worldTransform);
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
                worldTransform = GetWorldTransform(registry_, entity);
                UpdateInverseInertia(*body, collider, worldTransform);
            }

        }
        logCheckpoint("bodies-integrated", rigidBodies.size(), 0, 0, 0, 0);
        auto colliders = registry_.GetEntitiesWith<ColliderComponent>();
        struct ColliderStepData
        {
            Entity entity{};
            ColliderComponent* collider{};
            TransformComponent worldTransform{};
            Entity bodyEntity{};
            RigidBodyComponent* rigidbody{};
            TransformComponent* bodyTransform{};
        };

        std::vector<ColliderStepData> colliderCache;
        colliderCache.reserve(colliders.size());
        std::unordered_map<EntityIndex, std::size_t> colliderCacheIndices;
        colliderCacheIndices.reserve(colliders.size());
        std::vector<BroadPhaseCollider> broadPhaseList;
        broadPhaseList.reserve(colliders.size());

        for (Entity entity : colliders)
        {
            auto* collider = registry_.GetComponent<ColliderComponent>(entity);
            auto* transform = registry_.GetComponent<TransformComponent>(entity);
            if (collider == nullptr || transform == nullptr)
                continue;

            const Entity bodyEntity = FindRigidBodyOwner(registry_, entity);
            auto* rigidbody = bodyEntity.IsValid()? registry_.GetComponent<RigidBodyComponent>(bodyEntity) : nullptr;
            auto* bodyTransform = bodyEntity.IsValid()? registry_.GetComponent<TransformComponent>(bodyEntity) : transform;

            ColliderStepData stepData{entity,collider,GetWorldTransform(registry_, entity),bodyEntity,rigidbody,bodyTransform};
            UpdateColliderBounds(*collider, stepData.worldTransform);
            colliderCacheIndices.emplace(entity.index, colliderCache.size());
            colliderCache.push_back(stepData);

            if (collider->isTrigger)
                continue;

            broadPhaseList.push_back({entity, collider->worldBounds});
        }
        logCheckpoint("bounds-updated", rigidBodies.size(), broadPhaseList.size(), 0, 0, 0);

        std::vector<CollisionPair> collisionPairs = broadPhase_.FindPotentialPairs(broadPhaseList);
        logCheckpoint("broadphase-complete", rigidBodies.size(), broadPhaseList.size(), collisionPairs.size(), 0, 0);

        std::vector<NarrowPhaseColliderPairs> narrowPhasePairs;
        narrowPhasePairs.reserve(collisionPairs.size());
        const auto findCachedCollider = [&](Entity entity) -> ColliderStepData*
        {
            const auto found = colliderCacheIndices.find(entity.index);
            if (found == colliderCacheIndices.end())
                return nullptr;

            ColliderStepData& cached = colliderCache[found->second];
            return cached.entity.generation == entity.generation ? &cached : nullptr;
        };

        for (const CollisionPair& pair : collisionPairs)
        {
            ColliderStepData* first = findCachedCollider(pair.first);
            ColliderStepData* second = findCachedCollider(pair.second);
            if (first == nullptr || second == nullptr)
                continue;

            if (first->bodyEntity.IsValid() && first->bodyEntity == second->bodyEntity)
                continue;

            narrowPhasePairs.push_back({
                {first->entity, first->collider, &first->worldTransform, first->bodyTransform, first->rigidbody},
                {second->entity, second->collider, &second->worldTransform, second->bodyTransform, second->rigidbody}});
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
