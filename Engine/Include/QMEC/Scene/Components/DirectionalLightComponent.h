#pragma once

#include "QMEC/Math/Vec3.h"

namespace qmec::scene::components
{
    struct DirectionalLightComponent
    {
        Vec3 color{1.0f, 1.0f, 1.0f};
        float intensity{1.0f};
        bool enabled{true};
    };
}

namespace qmec
{
    using scene::components::DirectionalLightComponent;
}
