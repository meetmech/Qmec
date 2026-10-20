#pragma once

#include "QMEC/Input/Key.h"

#include <array>
#include <cstddef>

namespace qmec::input
{
    class InputSystem final
    {
    public:
        [[nodiscard]] static InputSystem& Instance() noexcept;

        // Poll once per frame. Input is cleared when disabled or when the target window is inactive.
        void Update(void* targetWindow, bool enabled) noexcept;

        [[nodiscard]] bool IsKeyDown(Key key) const noexcept;
        [[nodiscard]] bool WasKeyPressed(Key key) const noexcept;

    private:
        static constexpr std::size_t KeyCount = static_cast<std::size_t>(Key::Count);
        std::array<bool, KeyCount> currentKeys_{};
        std::array<bool, KeyCount> previousKeys_{};
    };
}

namespace qmec
{
    using input::InputSystem;
}
