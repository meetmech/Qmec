#pragma once

#include "QMEC/Physics/AABB.h"
#include "QMEC/Physics/Collider/ColliderShapes.h"

#include <variant>

namespace qmec
{
    struct ColliderComponent
    {  
        std::variant<BoxShape,PlaneShape,SphereShape,CylinderShape> shape{BoxShape{}};
        AABB worldBounds{};
        bool isTrigger{false};
    };

}
