#pragma once

namespace qmec::input
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

namespace qmec
{
    using input::MouseButton;
    using input::MouseDelta;
}
