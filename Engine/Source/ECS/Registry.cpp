#include "QMEC/ECS/Registry.h"

#include <limits>
#include <stdexcept>

namespace qmec::ecs
{
    Entity Registry::CreateEntity()
    {
        EntityIndex index{};

        if (!availableIndices_.empty())
        {
            index = availableIndices_.back();
            availableIndices_.pop_back();
            alive_[index] = 1U;
        }
        else
        {
            if (generations_.size() >= std::numeric_limits<EntityIndex>::max())
            {
                throw std::runtime_error("QMEC entity index limit reached.");
            }

            index = static_cast<EntityIndex>(generations_.size());
            generations_.push_back(0U);
            alive_.push_back(1U);
        }

        ++livingEntityCount_;
        return Entity{index, generations_[index]};
    }

    bool Registry::DestroyEntity(Entity entity)
    {
        if (!IsAlive(entity))
        {
            return false;
        }

        for (auto& componentPool : componentPools_)
        {
            componentPool.second->Remove(entity);
        }

        alive_[entity.index] = 0U;
        ++generations_[entity.index];
        availableIndices_.push_back(entity.index);
        --livingEntityCount_;

        return true;
    }

    bool Registry::IsAlive(Entity entity) const noexcept
    {
        if (!entity.IsValid() || entity.index >= generations_.size())
        {
            return false;
        }

        return alive_[entity.index] != 0U && generations_[entity.index] == entity.generation;
    }

    std::size_t Registry::GetLivingEntityCount() const noexcept
    {
        return livingEntityCount_;
    }

    std::vector<Entity> Registry::GetEntities() const
    {
        std::vector<Entity> entities{};
        entities.reserve(livingEntityCount_);

        for (EntityIndex index = 0; index < generations_.size(); ++index)
        {
            if (alive_[index] != 0U)
            {
                entities.push_back(Entity{index, generations_[index]});
            }
        }

        return entities;
    }

}
