#pragma once

namespace qmec::input
{
    enum class Key
    {
        W,
        A,
        S,
        D,
        Q,
        E,
        F,
        Shift,
        Control,
        Alt,
        Space,
        Escape,
        Count
    };
}

namespace qmec
{
    using input::Key;
}
