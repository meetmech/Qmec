#include "QMEC/Scene/Scene.h"
#include "QMEC/Scene/Components/NameComponent.h"
#include "QMEC/Scene/Components/TransformComponent.h"
#include "QMEC/Math/Mat4.h"

#include <cstdint>
#include <cstddef>
#include <unordered_set>
#include <utility>
#include <queue>
#include <cmath>

namespace qmec
{
    namespace
    {
        Mat4 BuildLocalMatrix(const TransformComponent& transform) noexcept
        {
            Mat4 scale = Mat4::Identity();
            scale.values[0][0] = transform.scale.x;
            scale.values[1][1] = transform.scale.y;
            scale.values[2][2] = transform.scale.z;

            return scale * transform.rotation.Normalized().ToMat4()* Mat4::Translation(transform.position.x,transform.position.y,transform.position.z);
        }

        bool TryDecomposeLocalMatrix(const Mat4& matrix,TransformComponent& transform) noexcept
        {
            constexpr float tolerance = 1.0e-3f;
            if (std::fabs(matrix.values[0][3]) > tolerance || std::fabs(matrix.values[1][3]) > tolerance ||
                std::fabs(matrix.values[2][3]) > tolerance ||
                std::fabs(matrix.values[3][3] - 1.0f) > tolerance)
            {
                return false;
            }

            Vec3 rows[3]
            {
                {matrix.values[0][0], matrix.values[0][1], matrix.values[0][2]},
                {matrix.values[1][0], matrix.values[1][1], matrix.values[1][2]},
                {matrix.values[2][0], matrix.values[2][1], matrix.values[2][2]}
            };
            Vec3 scale{rows[0].Length(), rows[1].Length(), rows[2].Length()};
            if (!std::isfinite(scale.x) || !std::isfinite(scale.y) ||
                !std::isfinite(scale.z) || scale.x <= 1.0e-6f ||
                scale.y <= 1.0e-6f || scale.z <= 1.0e-6f)
            {
                return false;
            }

            const float determinant = Dot(rows[0], Cross(rows[1], rows[2]));
            if (!std::isfinite(determinant) || std::fabs(determinant) <= 1.0e-6f)
                return false;
            if (determinant < 0.0f)
                scale.x = -scale.x;

            rows[0] = rows[0] * (1.0f / scale.x);
            rows[1] = rows[1] * (1.0f / scale.y);
            rows[2] = rows[2] * (1.0f / scale.z);

            if (std::fabs(Dot(rows[0], rows[1])) > tolerance ||std::fabs(Dot(rows[0], rows[2])) > tolerance ||
                std::fabs(Dot(rows[1], rows[2])) > tolerance)
            {
                return false;
            }

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
                rotation.w = 0.25f * s;
                rotation.x = (m21 - m12) / s;
                rotation.y = (m02 - m20) / s;
                rotation.z = (m10 - m01) / s;
            }
            else if (m00 > m11 && m00 > m22)
            {
                const float s = std::sqrt(1.0f + m00 - m11 - m22) * 2.0f;
                rotation.w = (m21 - m12) / s;
                rotation.x = 0.25f * s;
                rotation.y = (m01 + m10) / s;
                rotation.z = (m02 + m20) / s;
            }
            else if (m11 > m22)
            {
                const float s = std::sqrt(1.0f + m11 - m00 - m22) * 2.0f;
                rotation.w = (m02 - m20) / s;
                rotation.x = (m01 + m10) / s;
                rotation.y = 0.25f * s;
                rotation.z = (m12 + m21) / s;
            }
            else
            {
                const float s = std::sqrt(1.0f + m22 - m00 - m11) * 2.0f;
                rotation.w = (m10 - m01) / s;
                rotation.x = (m02 + m20) / s;
                rotation.y = (m12 + m21) / s;
                rotation.z = 0.25f * s;
            }

            rotation = rotation.Normalized();
            if (!std::isfinite(rotation.x) || !std::isfinite(rotation.y) || !std::isfinite(rotation.z) || !std::isfinite(rotation.w))
            {
                return false;
            }

            transform.position = {matrix.values[3][0], matrix.values[3][1], matrix.values[3][2]};
            transform.rotation = rotation;
            transform.scale = scale;
            return true;
        }
    }

    void Scene::runPhysics(float dt)
    {
        physcisWorld_.Simulate(dt);
    }

    Registry& Scene::GetRegistry() noexcept
    {
        return registry_;
    }

    const Registry& Scene::GetRegistry() const noexcept
    {
        return registry_;
    }

    Mat4 Scene::GetWorldMatrix(Entity entity) const noexcept
    {
        if (!registry_.IsAlive(entity))
            return Mat4::Identity();

        Mat4 worldMatrix = Mat4::Identity();
        Entity current = entity;
        const std::size_t maximumDepth = registry_.GetLivingEntityCount();
        std::size_t depth = 0U;

        while (current.IsValid() && registry_.IsAlive(current))
        {
            if (depth++ >= maximumDepth)
            {
                return Mat4::Identity();
            }


            const auto* transform = registry_.GetComponent<TransformComponent>(current);
            if (transform == nullptr)
                break;

            worldMatrix = worldMatrix * BuildLocalMatrix(*transform);
            current = transform->GetParent();
        }

        return worldMatrix;
    }

    Vec3 Scene::GetWorldPosition(Entity entity) const noexcept
    {
        const Mat4 worldMatrix = GetWorldMatrix(entity);
        return {
            worldMatrix.values[3][0],
            worldMatrix.values[3][1],
            worldMatrix.values[3][2]};
    }

    std::vector<SceneEntityEntry> Scene::GetHierarchyEntities() const
    {
        std::vector<SceneEntityEntry> entries{};
        entries.reserve(registry_.GetLivingEntityCount());

        for (const Entity entity : registry_.GetEntities())
        {
            const auto* name = registry_.GetComponent<NameComponent>(entity);
            std::string displayName = name != nullptr ? name->name : std::string{};
            if (displayName.empty())
            {
                displayName = "Entity " + std::to_string(entity.index);
            }

            const auto* transform = registry_.GetComponent<TransformComponent>(entity);
            const Entity parent = transform != nullptr ? transform->GetParent() : Entity{};
            entries.push_back(SceneEntityEntry{entity, std::move(displayName), parent});
        }

        return entries;
    }

    std::uint64_t Scene::GetHierarchyRevision() const noexcept
    {
        return hierarchyRevision_;
    }

    bool Scene::SetParent(Entity child, Entity parent, bool keepWorldTransform)
    {
        if (!registry_.IsAlive(child))
            return false;

        auto* childTransform = registry_.GetComponent<TransformComponent>(child);
        if (childTransform == nullptr)
            return false;

        if (childTransform->GetParent() == parent)
            return true;

        if (parent.IsValid() && (parent == child || !registry_.IsAlive(parent)))
            return false;

        if (parent.IsValid())
        {
            Entity ancestor = parent;
            const std::size_t maximumAncestors = registry_.GetLivingEntityCount();
            std::size_t ancestorCount = 0U;
            while (ancestor.IsValid())
            {
                if (ancestor == child || !registry_.IsAlive(ancestor) ||
                    ancestorCount++ >= maximumAncestors)
                {
                    return false;
                }

                const auto* ancestorTransform = registry_.GetComponent<TransformComponent>(ancestor);
                if (ancestorTransform == nullptr)
                    return false;

                ancestor = ancestorTransform->GetParent();
            }
        }

        if (keepWorldTransform)
        {
            const Mat4 oldWorldMatrix = GetWorldMatrix(child);
            const Mat4 parentWorldMatrix = parent.IsValid()
                ? GetWorldMatrix(parent)
                : Mat4::Identity();
            Mat4 inverseParentWorld{};
            if (!parentWorldMatrix.TryInverse(inverseParentWorld))
                return false;

            TransformComponent updatedLocalTransform = *childTransform;
            if (!TryDecomposeLocalMatrix(
                oldWorldMatrix * inverseParentWorld,
                updatedLocalTransform))
            {
                return false;
            }

            updatedLocalTransform.SetParent(parent);
            *childTransform = updatedLocalTransform;
        }
        else
        {
            childTransform->SetParent(parent);
        }
        ++hierarchyRevision_;
        return true;
    }

    void Scene::Clear()
    {
        for (const Entity entity : registry_.GetEntities())
            registry_.DestroyEntity(entity);

        ++hierarchyRevision_;
    }

    std::vector<Entity> Scene::GetChildren(Entity parent) const
    {
        std::vector<Entity> children{};
        if (!registry_.IsAlive(parent))
            return children;

        for (const Entity entity : registry_.GetEntities())
        {
            const auto* transform = registry_.GetComponent<TransformComponent>(entity);
            if (transform != nullptr && transform->GetParent() == parent)
                children.push_back(entity);
        }

        return children;
    }

    std::vector<Entity> Scene::GetDescendants(Entity parent) const
    {
        std::vector<Entity> descendants{};

        if (!registry_.IsAlive(parent))
        {
            return descendants;
        }

        std::queue<Entity> pending{};
        std::unordered_set<EntityIndex> visited{};

        for (const Entity child : GetChildren(parent))
        {
            pending.push(child);
        }

        while (!pending.empty())
        {
            const Entity child = pending.front();
            pending.pop();

            if (!visited.insert(child.index).second)
                continue;

            descendants.push_back(child);

            for (const Entity grandChild : GetChildren(child))
            {
                pending.push(grandChild);
            }
        }

        return descendants;
    }

    PhysicsWorld& Scene::GetPhysicsWorld() noexcept
    {
        return physcisWorld_;
    }

    const PhysicsWorld& Scene::GetPhysicsWorld() const noexcept
    {
        return physcisWorld_;
    }
}
