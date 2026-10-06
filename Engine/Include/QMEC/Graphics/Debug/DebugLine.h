#pragma once

#include "QMEC/Math/Vec3.h"

namespace qmec::graphics::debug
{
    struct DebugLine
    {
        Vec3 start{};
        Vec3 end{};
        Vec3 color{1.0f, 1.0f, 1.0f};
    };
}

namespace qmec
{
    using graphics::debug::DebugLine;
}
