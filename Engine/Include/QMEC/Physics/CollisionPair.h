#pragma once

#include "QMEC/ECS/Entity.h"

namespace qmec
{
    struct CollisionPair
    {
        Entity first;
        Entity second;
    };
}