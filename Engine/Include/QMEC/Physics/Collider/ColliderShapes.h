#pragma once

#include "QMEC/Math/Vec3.h"

namespace qmec::physics
{
  
    struct BoxShape
    {
        Vec3 centre{};
        Vec3 halfExtents{0.5f, 0.5f, 0.5f};
    };

    
    struct PlaneShape
    {
        Vec3 centre{};
        float halfWidthX{0.5f};  
        float halfLengthZ{0.5f}; 
    };

    struct SphereShape
    {
        Vec3 centre{};
        float radius{};
    };

    struct CylinderShape
    {
        Vec3 centre{};
        float radius{};
        float halfHeight{};
    };
}

namespace qmec
{
    using physics::BoxShape;
    using physics::PlaneShape;
    using physics::SphereShape;
    using physics::CylinderShape;
}
