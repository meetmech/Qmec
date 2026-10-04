#pragma once

#include "QMEC/ECS/Registry.h"
#include "QMEC/Physics/BroadPhase.h"
#include "QMEC/Physics/NarrowPhase.h"
#include "QMEC/Scene/Components/ColliderComponent.h"
#include "QMEC/Scene/Components/TransformComponent.h"
#include "QMEC/Scene/Components/RigidBodyComponent.h"
#include "QMEC/Physics/Solver/ContactSolver.h"

namespace qmec {

    class PhysicsWorld
    {
    public:
        PhysicsWorld(Registry& registry, float deltaTime = 1.0f / 60.0f, Vec3 gravity = {0,-9.8f,0})
            : fixedDeltaTime(deltaTime), gravity_(gravity), registry_(registry)
        {

        }

        void Simulate(float frameDt);

        void UpdateInverseInertia(RigidBodyComponent& body,
            const ColliderComponent* collider, const TransformComponent& transform) const noexcept;

        void UpdateColliderBounds(ColliderComponent& collider,
            const TransformComponent& transform) const noexcept;

    private:
        [[nodiscard]] AABB CalculateWorldBounds(const BoxShape& shape,const TransformComponent& transform) const noexcept;
        [[nodiscard]] AABB CalculateWorldBounds(const PlaneShape& shape,const TransformComponent& transform) const noexcept;
        [[nodiscard]] AABB CalculateWorldBounds(const SphereShape& shape,const TransformComponent& transform) const noexcept;
        [[nodiscard]] AABB CalculateWorldBounds(const CylinderShape& shape, const TransformComponent& transform) const noexcept;

        float fixedDeltaTime{};
        float accumulator = 0.0f;
        Vec3 gravity_{};
        void step(float frameDt);
        Registry& registry_;
        BroadPhase broadPhase_;
        NarrowPhase narrowPhase_;
        ContactSolver solver_;
    };
}
