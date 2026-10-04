#pragma once

#include "QMEC/Math/Vec3.h"

namespace qmec
{
   
    struct Mat3
    {
        float values[3][3]{};

        [[nodiscard]] Vec3 TransformVector(const Vec3& vector) const noexcept
        {
            return {
                values[0][0] * vector.x + values[0][1] * vector.y + values[0][2] * vector.z,
                values[1][0] * vector.x + values[1][1] * vector.y + values[1][2] * vector.z,
                values[2][0] * vector.x + values[2][1] * vector.y + values[2][2] * vector.z};
        }
    };
}
