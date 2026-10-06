#pragma once

#include "QMEC/Graphics/MeshData.h"

namespace qmec::graphics
{
    class IMeshGenerator
    {
    public:
        virtual ~IMeshGenerator() = default;
        [[nodiscard]] virtual MeshData Generate() const = 0;
    };
}

namespace qmec
{
    using graphics::IMeshGenerator;
}
