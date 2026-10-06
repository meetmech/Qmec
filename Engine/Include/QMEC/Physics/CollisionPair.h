#pragma once

#include "QMEC/ECS/Entity.h"

namespace qmec::physics
{
    struct CollisionPair
    {
        Entity first;
        Entity second;
    };
}
namespace qmec
{
    using physics::CollisionPair;
}
