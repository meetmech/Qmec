#pragma once

#include "QMEC/ECS/Registry.h"
#include "QMEC/Math/Mat4.h"
#include "QMEC/Physics/PhyscisWorld.h"

#include <cstdint>
#include <string>
#include <vector>

namespace qmec
{
    struct SceneEntityEntry
    {
        Entity entity{};
        std::string name{};
        Entity parent{};
    };

    class Scene
    {
    public:
        Scene(): registry_{}, physcisWorld_{ registry_ }
        {
        }

        void runPhysics(float dt);

        [[nodiscard]] Registry& GetRegistry() noexcept;
        [[nodiscard]] const Registry& GetRegistry() const noexcept;
        [[nodiscard]] std::vector<SceneEntityEntry> GetHierarchyEntities() const;
        [[nodiscard]] Mat4 GetWorldMatrix(Entity entity) const noexcept;
        [[nodiscard]] Vec3 GetWorldPosition(Entity entity) const noexcept;
        [[nodiscard]] std::uint64_t GetHierarchyRevision() const noexcept;
        bool SetParent(Entity child, Entity parent, bool keepWorldTransform = true);
        void Clear();
        [[nodiscard]] std::vector<Entity> GetChildren(Entity parent) const;
        [[nodiscard]] std::vector<Entity> GetDescendants(Entity parent) const;

        [[nodiscard]] PhysicsWorld& GetPhysicsWorld() noexcept;
        [[nodiscard]] const PhysicsWorld& GetPhysicsWorld() const noexcept;

    private:
        Registry registry_{};
        PhysicsWorld physcisWorld_;
        std::uint64_t hierarchyRevision_{};
    };
}
