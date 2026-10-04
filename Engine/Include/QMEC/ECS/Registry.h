#pragma once

#include "QMEC/ECS/ComponentPool.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <typeindex>
#include <unordered_map>
#include <vector>

namespace qmec
{
    class Registry
    {
    public:
        [[nodiscard]] Entity CreateEntity();
        bool DestroyEntity(Entity entity);

        [[nodiscard]] bool IsAlive(Entity entity) const noexcept;
        [[nodiscard]] std::size_t GetLivingEntityCount() const noexcept;
        [[nodiscard]] std::vector<Entity> GetEntities() const;

        template<typename Component>
        std::vector<Component> GetAllComponents(Entity entity);

        template<typename Component>
        [[nodiscard]] std::span<const Entity> GetEntitiesWith() const noexcept;

        template<typename Component>
        bool AddComponent(Entity entity, const Component& component);

        template<typename Component>
        bool RemoveComponent(Entity entity);

        template<typename Component>
        [[nodiscard]] bool HasComponent(Entity entity) const noexcept;

        template<typename Component>
        [[nodiscard]] Component* GetComponent(Entity entity) noexcept;

        template<typename Component>
        [[nodiscard]] const Component* GetComponent(Entity entity) const noexcept;

    private:
        template<typename Component>
        [[nodiscard]] ComponentPool<Component>& GetOrCreatePool();

        template<typename Component>
        [[nodiscard]] ComponentPool<Component>* FindPool() noexcept;

        template<typename Component>
        [[nodiscard]] const ComponentPool<Component>* FindPool()
            const noexcept;

        std::vector<EntityGeneration> generations_{};
        std::vector<EntityIndex> availableIndices_{};
        std::vector<std::uint8_t> alive_{};
        std::size_t livingEntityCount_{};

        std::unordered_map<std::type_index,std::unique_ptr<IComponentPool>> componentPools_{};
    };
}

#include "QMEC/ECS/Registry.inl"
