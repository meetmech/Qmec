#pragma once
#include "QMEC/Input/Key.h"
#include <cstdint>
#include <array>
#include <cstddef>
#include "QMEC/Input/MouseDelta.h"

namespace qmec::platform::windows
{
    class Win32Window final
    {
    public:
        Win32Window() noexcept = default;
        ~Win32Window() noexcept;

        Win32Window(const Win32Window&) = delete;
        Win32Window& operator=(const Win32Window&) = delete;

        [[nodiscard]] bool Initialize(
            const wchar_t* title,
            std::uint32_t clientWidth,
            std::uint32_t clientHeight) noexcept;

        [[nodiscard]] bool ProcessMessages() noexcept;
        [[nodiscard]] void* NativeHandle() const noexcept;
        [[nodiscard]] std::uint32_t ClientWidth() const noexcept;
        [[nodiscard]] std::uint32_t ClientHeight() const noexcept;
        [[nodiscard]] bool IsKeyDown(Key key) const noexcept;
        [[nodiscard]] bool IsMouseButtonDown(MouseButton button) const noexcept;
        [[nodiscard]] MouseDelta ConsumeMouseDelta() noexcept;
        [[nodiscard]] float ConsumeMouseWheelDelta() noexcept;

    private:
        void* instance_{nullptr};
        void* handle_{nullptr};
        std::uint32_t clientWidth_{0U};
        std::uint32_t clientHeight_{0U};
        MouseDelta mouseDelta_{};
        float mouseWheelDelta_{};
        bool registeredWindowClass_{false};
        static constexpr std::size_t KeyCount = static_cast<std::size_t>(Key::Count);
        static constexpr std::size_t MouseButtonCount =
            static_cast<std::size_t>(MouseButton::Count);
        std::array<bool, KeyCount> keyStates_{};
        std::array<bool, MouseButtonCount> mouseButtonStates_{};
    };
}

namespace qmec
{
    using platform::windows::Win32Window;
}
