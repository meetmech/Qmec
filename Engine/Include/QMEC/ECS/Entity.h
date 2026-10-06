#pragma once

#include <cstdint>

namespace qmec::ecs
{
    using EntityIndex = std::uint32_t;
    using EntityGeneration = std::uint32_t;

     constexpr EntityIndex InvalidEntityIndex = 0xFFFFFFFFU;
    struct Entity
    {
        EntityIndex index{InvalidEntityIndex};
        EntityGeneration generation{};

        [[nodiscard]] bool IsValid() const noexcept
        {
            return index != InvalidEntityIndex;
        }

        bool operator==(const Entity&) const noexcept = default;
    };
}

namespace qmec
{
    using ecs::Entity;
    using ecs::EntityIndex;
    using ecs::EntityGeneration;
    using ecs::InvalidEntityIndex;
}
