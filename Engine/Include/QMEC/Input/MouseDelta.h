#pragma once

namespace qmec
{
    enum class MouseButton
    {
        Left,
        Middle,
        Right,
        Count
    };

    struct MouseDelta
    {
        float x{};
        float y{};
    };

}
