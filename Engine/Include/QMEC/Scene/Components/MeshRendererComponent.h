#pragma once

#include "QMEC/Assets/AssetManager.h"

#include <cstdint>

namespace qmec::scene::components
{
    struct MeshRendererComponent
    {
        MeshHandle mesh{};
        std::uint32_t materialIndex{};
        bool visible{true};
    };
}

namespace qmec
{
    using scene::components::MeshRendererComponent;
}
