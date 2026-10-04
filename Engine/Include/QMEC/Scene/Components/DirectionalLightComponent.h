#pragma once

#include "QMEC/Math/Vec3.h"

namespace qmec
{
    struct DirectionalLightComponent
    {
        Vec3 color{1.0f, 1.0f, 1.0f};
        float intensity{1.0f};
        bool enabled{true};
    };
}
