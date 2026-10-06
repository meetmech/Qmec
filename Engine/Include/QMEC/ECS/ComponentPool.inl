#pragma once

#include <cstddef>
#include <utility>

namespace qmec::ecs
{
    template<typename Component>
    bool ComponentPool<Component>::Add(Entity entity,const Component& component)
    {
        if (!entity.IsValid())
        {
            return false;
        }

        const EntityIndex entityIndex = entity.index;

        if (entityIndex < sparse_.size()
            && sparse_[entityIndex] != InvalidEntityIndex)
        {
            return false;
        }

        if (entityIndex >= sparse_.size())
        {
            sparse_.resize(
                static_cast<std::size_t>(entityIndex) + 1U,
                InvalidEntityIndex);
        }

        const EntityIndex denseIndex =
            static_cast<EntityIndex>(denseComponents_.size());

        denseComponents_.push_back(component);
        denseEntities_.push_back(entity);
        sparse_[entityIndex] = denseIndex;

        return true;
    }

    template<typename Component>
    bool ComponentPool<Component>::Remove(Entity entity)
    {
        if (!Has(entity))
        {
            return false;
        }

        const EntityIndex entityIndex = entity.index;
        const EntityIndex removedDenseIndex = sparse_[entityIndex];
        const EntityIndex lastDenseIndex = static_cast<EntityIndex>(denseComponents_.size() - 1U);

        if (removedDenseIndex != lastDenseIndex)
        {
            std::swap(
                denseComponents_[removedDenseIndex],
                denseComponents_[lastDenseIndex]);

            std::swap(
                denseEntities_[removedDenseIndex],
                denseEntities_[lastDenseIndex]);

            const Entity movedEntity = denseEntities_[removedDenseIndex];
            sparse_[movedEntity.index] = removedDenseIndex;
        }

        denseComponents_.pop_back();
        denseEntities_.pop_back();
        sparse_[entityIndex] = InvalidEntityIndex;

        return true;
    }

    template<typename Component>
    bool ComponentPool<Component>::Has(Entity entity) const noexcept
    {
        if (!entity.IsValid() || entity.index >= sparse_.size())
        {
            return false;
        }

        const EntityIndex denseIndex = sparse_[entity.index];

        if (denseIndex == InvalidEntityIndex
            || denseIndex >= denseEntities_.size())
        {
            return false;
        }

        return denseEntities_[denseIndex] == entity;
    }

    template<typename Component>
    Component* ComponentPool<Component>::Get(Entity entity) noexcept
    {
        if (!Has(entity))
        {
            return nullptr;
        }

        return &denseComponents_[sparse_[entity.index]];
    }

    template<typename Component>
    const Component* ComponentPool<Component>::Get(Entity entity) const noexcept
    {
        if (!Has(entity))
        {
            return nullptr;
        }

        return &denseComponents_[sparse_[entity.index]];
    }
}
