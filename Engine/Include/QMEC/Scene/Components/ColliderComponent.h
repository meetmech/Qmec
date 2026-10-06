#pragma once

#include "QMEC/Physics/AABB.h"
#include "QMEC/Physics/Collider/ColliderShapes.h"

#include <variant>

namespace qmec::scene::components
{
    struct ColliderComponent
    {  
        std::variant<BoxShape,PlaneShape,SphereShape,CylinderShape> shape{BoxShape{}};
        AABB worldBounds{};
        bool isTrigger{false};
    };

}

namespace qmec
{
    using scene::components::ColliderComponent;
}
