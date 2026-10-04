#pragma once

#include "QMEC/Graphics/MeshData.h"

namespace qmec
{
    class IMeshGenerator
    {
    public:
        virtual ~IMeshGenerator() = default;
        [[nodiscard]] virtual MeshData Generate() const = 0;
    };
}
