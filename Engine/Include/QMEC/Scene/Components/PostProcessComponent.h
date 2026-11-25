#pragma once

namespace qmec::scene::components
{
    struct PostProcessComponent
    {
        bool enabled{true};
        float brightness{1.0f};
        float contrast{};
        float saturation{1.0f};
        float hueShift{};
    };
}

namespace qmec
{
    using scene::components::PostProcessComponent;
}
