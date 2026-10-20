#include "QMEC/Input/InputSystem.h"

#include <Windows.h>

namespace qmec::input
{
    namespace
    {
        int ToVirtualKey(Key key) noexcept
        {
            switch (key)
            {
            case Key::W: return 'W';
            case Key::A: return 'A';
            case Key::S: return 'S';
            case Key::D: return 'D';
            case Key::Q: return 'Q';
            case Key::E: return 'E';
            case Key::F: return 'F';
            case Key::Shift: return VK_SHIFT;
            case Key::Control: return VK_CONTROL;
            case Key::Alt: return VK_MENU;
            case Key::Space: return VK_SPACE;
            case Key::Escape: return VK_ESCAPE;
            case Key::Count: break;
            }
            return 0;
        }
    }

    InputSystem& InputSystem::Instance() noexcept
    {
        static InputSystem input{};
        return input;
    }

    void InputSystem::Update(void* targetWindow, bool enabled) noexcept
    {
        previousKeys_ = currentKeys_;
        currentKeys_.fill(false);

        const HWND window = static_cast<HWND>(targetWindow);
        if (!enabled || window == nullptr || GetForegroundWindow() != window)
            return;

        for (std::size_t index = 0; index < KeyCount; ++index)
        {
            const int virtualKey = ToVirtualKey(static_cast<Key>(index));
            if (virtualKey != 0)
                currentKeys_[index] = (GetAsyncKeyState(virtualKey) & 0x8000) != 0;
        }
    }

    bool InputSystem::IsKeyDown(Key key) const noexcept
    {
        const std::size_t index = static_cast<std::size_t>(key);
        return index < currentKeys_.size() && currentKeys_[index];
    }

    bool InputSystem::WasKeyPressed(Key key) const noexcept
    {
        const std::size_t index = static_cast<std::size_t>(key);
        return index < currentKeys_.size() && currentKeys_[index] && !previousKeys_[index];
    }
}
