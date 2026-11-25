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

namespace qmec::scene
{
    namespace
    {
        struct WorldTransformData
        {
            Vec3 position{};
            Quat rotation{};
            Vec3 scale{1.0f, 1.0f, 1.0f};
        };

        bool BuildWorldTransform(const Registry& registry,Entity entity,std::size_t remainingDepth,WorldTransformData& result) noexcept
        {
            if (!registry.IsAlive(entity) || remainingDepth == 0U)
                return false;

            const auto* local = registry.GetComponent<TransformComponent>(entity);
            if (local == nullptr)
                return false;

            const Entity parent = local->GetParent();
            if (!parent.IsValid())
            {
                result.position = local->position;
                result.rotation = local->rotation.Normalized();
                result.scale = local->scale;
                return true;
            }

            WorldTransformData parentWorld{};
            if (!BuildWorldTransform(registry, parent, remainingDepth - 1U, parentWorld))
                return false;

            const Vec3 scaledLocalPosition{local->position.x * parentWorld.scale.x,local->position.y * parentWorld.scale.y,local->position.z * parentWorld.scale.z};
            result.position = parentWorld.position + parentWorld.rotation.Rotate(scaledLocalPosition);
            result.rotation = (parentWorld.rotation * local->rotation).Normalized();

            if (local->inheritParentScale)
            {
                result.scale = {parentWorld.scale.x * local->scale.x * parentWorld.scale.y * local->scale.y,parentWorld.scale.z * local->scale.z};
            }
            else
            {
                result.scale = local->scale;
            }

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
        WorldTransformData worldTransform{};
        if (!BuildWorldTransform(
            registry_, entity, registry_.GetLivingEntityCount(), worldTransform))
        {
            return Mat4::Identity();
        }

        Mat4 scale = Mat4::Identity();
        scale.values[0][0] = worldTransform.scale.x;
        scale.values[1][1] = worldTransform.scale.y;
        scale.values[2][2] = worldTransform.scale.z;

        const Mat4 worldMatrix = scale * worldTransform.rotation.ToMat4()* Mat4::Translation(worldTransform.position.x,worldTransform.position.y,worldTransform.position.z);
        return worldMatrix;
    }

    Vec3 Scene::GetWorldPosition(Entity entity) const noexcept
    {
        const Mat4 worldMatrix = GetWorldMatrix(entity);
        return {worldMatrix.values[3][0],worldMatrix.values[3][1],worldMatrix.values[3][2]};
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
            WorldTransformData oldWorld{};
            if (!BuildWorldTransform(
                registry_, child, registry_.GetLivingEntityCount(), oldWorld))
                return false;

            WorldTransformData parentWorld{};
            if (parent.IsValid() && !BuildWorldTransform(
                registry_, parent, registry_.GetLivingEntityCount(), parentWorld))
            {
                return false;
            }

            constexpr float scaleTolerance = 1.0e-8f;
            if (std::fabs(parentWorld.scale.x) <= scaleTolerance ||
                std::fabs(parentWorld.scale.y) <= scaleTolerance ||
                std::fabs(parentWorld.scale.z) <= scaleTolerance)
            {
                return false;
            }

            const Quat inverseParentRotation = parentWorld.rotation.Conjugated().Normalized();
            const Vec3 localPosition = inverseParentRotation.Rotate(oldWorld.position - parentWorld.position);
            TransformComponent updatedLocalTransform = *childTransform;
            updatedLocalTransform.position = {localPosition.x / parentWorld.scale.x,localPosition.y / parentWorld.scale.y,localPosition.z / parentWorld.scale.z};
            updatedLocalTransform.rotation = (inverseParentRotation * oldWorld.rotation).Normalized();
            updatedLocalTransform.scale = oldWorld.scale;
            if (updatedLocalTransform.inheritParentScale)
            {
                updatedLocalTransform.scale = {oldWorld.scale.x / parentWorld.scale.x,oldWorld.scale.y / parentWorld.scale.y,oldWorld.scale.z / parentWorld.scale.z};
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
