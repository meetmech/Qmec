#pragma once

#include "QMEC/Assets/AssetManager.h"

#include <cstdint>

namespace qmec
{
    struct MeshRendererComponent
    {
        MeshHandle mesh{};
        std::uint32_t materialIndex{};
        bool visible{true};
    };
}
