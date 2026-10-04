#pragma once

#include <cstdint>

namespace qmec
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
