#pragma once

#include "QMEC/Math/Vec3.h"

#include <string>

namespace qmec::scene::components
{
    struct MaterialComponent
    {
        Vec3 albedoColor{1.0f, 1.0f, 1.0f};
        std::string albedoTexturePath{};
        std::string normalTexturePath{};

        float normalStrength{1.0f};
        float metallic{0.0f};
        float roughness{0.5f};
        float specularLevel{0.5f};
    };
}

namespace qmec
{
    using scene::components::MaterialComponent;
}
