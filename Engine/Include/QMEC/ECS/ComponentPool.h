#pragma once

#include "QMEC/ECS/Entity.h"

#include <vector>
#include <span>

namespace qmec::ecs
{
    class IComponentPool
    {
    public:
        virtual ~IComponentPool() = default;

        virtual bool Remove(Entity entity) = 0;
    };

    template<typename Component>
    class ComponentPool final : public IComponentPool
    {
    public:
        bool Add(Entity entity, const Component& component);
        bool Remove(Entity entity) override;
        [[nodiscard]] bool Has(Entity entity) const noexcept;
        [[nodiscard]] Component* Get(Entity entity) noexcept;
        [[nodiscard]] const Component* Get(Entity entity) const noexcept;

        [[nodiscard]] std::span<const Entity> GetEntities() const noexcept
        {
            return denseEntities_;
        }

    private:
        std::vector<Component> denseComponents_{};
        std::vector<Entity> denseEntities_{};
        std::vector<EntityIndex> sparse_{};
    };
}

#include "QMEC/ECS/ComponentPool.inl"

namespace qmec
{
    using ecs::IComponentPool;
    using ecs::ComponentPool;
}
