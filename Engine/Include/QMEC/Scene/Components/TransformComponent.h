#pragma once

#include "QMEC/ECS/Entity.h"
#include "QMEC/Math/Quat.h"
#include "QMEC/Math/Vec3.h"

namespace qmec::scene::components
{
    struct TransformComponent
    {
        Vec3 position{};
        Quat rotation{};
        Vec3 scale{1.0f, 1.0f, 1.0f};
        Entity parent{};

        void SetParent(Entity parentEntity) noexcept
        {
            parent = parentEntity;
        }

        void ClearParent() noexcept
        {
            parent = {};
        }

        [[nodiscard]] bool HasParent() const noexcept
        {
            return parent.IsValid();
        }

        [[nodiscard]] Entity GetParent() const noexcept
        {
            return parent;
        }
    };
}

namespace qmec
{
    using scene::components::TransformComponent;
}
