#include "Registry.h"
#pragma once

namespace qmec
{
    template<typename Component>
    inline std::vector<Component> Registry::GetAllComponents(Entity entity)
    {
        return std::vector<Component>();
    }

    template<typename Component>
    std::span<const Entity> Registry::GetEntitiesWith() const noexcept
    {
        const auto* pool = FindPool<Component>();
        return pool != nullptr ? pool->GetEntities() : std::span<const Entity>{};
    }

    template<typename Component>
    bool Registry::AddComponent(Entity entity,const Component& component)
    {
        if (!IsAlive(entity))
        {
            return false;
        }

        return GetOrCreatePool<Component>().Add(entity, component);
    }

    template<typename Component>
    bool Registry::RemoveComponent(Entity entity)
    {
        if (!IsAlive(entity))
        {
            return false;
        }

        ComponentPool<Component>* pool = FindPool<Component>();
        return pool != nullptr && pool->Remove(entity);
    }

    template<typename Component>
    bool Registry::HasComponent(Entity entity) const noexcept
    {
        if (!IsAlive(entity))
        {
            return false;
        }

        const ComponentPool<Component>* pool = FindPool<Component>();
        return pool != nullptr && pool->Has(entity);
    }

    template<typename Component>
    Component* Registry::GetComponent(Entity entity) noexcept
    {
        if (!IsAlive(entity))
        {
            return nullptr;
        }

        ComponentPool<Component>* pool = FindPool<Component>();
        return pool != nullptr ? pool->Get(entity) : nullptr;
    }

    template<typename Component>
    const Component* Registry::GetComponent(Entity entity) const noexcept
    {
        if (!IsAlive(entity))
        {
            return nullptr;
        }

        const ComponentPool<Component>* pool = FindPool<Component>();
        return pool != nullptr ? pool->Get(entity) : nullptr;
    }

    template<typename Component>
    ComponentPool<Component>& Registry::GetOrCreatePool()
    {
        const std::type_index componentType{typeid(Component)};
        const auto existingPool = componentPools_.find(componentType);

        if (existingPool != componentPools_.end())
        {
            return static_cast<ComponentPool<Component>&>(*existingPool->second);
        }

        auto newPool = std::make_unique<ComponentPool<Component>>();
        ComponentPool<Component>* poolPointer = newPool.get();
        componentPools_.emplace(componentType, std::move(newPool));

        return *poolPointer;
    }

    template<typename Component>
    ComponentPool<Component>* Registry::FindPool() noexcept
    {
        const auto pool = componentPools_.find(std::type_index{typeid(Component)});

        if (pool == componentPools_.end())
        {
            return nullptr;
        }

        return static_cast<ComponentPool<Component>*>(pool->second.get());
    }

    template<typename Component>
    const ComponentPool<Component>* Registry::FindPool() const noexcept
    {
        const auto pool = componentPools_.find(std::type_index{typeid(Component)});

        if (pool == componentPools_.end())
        {
            return nullptr;
        }

        return static_cast<const ComponentPool<Component>*>(pool->second.get());
    }
}
