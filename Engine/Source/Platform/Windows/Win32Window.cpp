#include "QMEC/Graphics/Platform/Win32Window.h"
#include "QMEC/Input/MouseDelta.h"
#include <Windows.h>

namespace
{
    constexpr wchar_t WindowClassName[] = L"QMECWindowClass";

    LRESULT CALLBACK WindowProcedure(
        HWND window,
        UINT message,
        WPARAM wParam,
        LPARAM lParam) noexcept
    {
        switch(message)
        {
        case WM_CLOSE:
            DestroyWindow(window);
            return 0;

        case WM_DESTROY:
            PostQuitMessage(0);
            return 0;

        default:
            return DefWindowProcW(window, message, wParam, lParam);
        }
    }

    qmec::input::Key TranslateKey(WPARAM virtualKey) noexcept
    {
        switch (virtualKey)
        {
        case 'W':
            return qmec::input::Key::W;
        case 'A':
            return qmec::input::Key::A;
        case 'S':
            return qmec::input::Key::S;
        case 'D':
            return qmec::input::Key::D;
        case 'Q':
            return qmec::input::Key::Q;
        case 'E':
            return qmec::input::Key::E;
        case 'F':
            return qmec::input::Key::F;
        case VK_SHIFT:
            return qmec::input::Key::Shift;
        case VK_CONTROL:
            return qmec::input::Key::Control;
        case VK_MENU:
            return qmec::input::Key::Alt;
        case VK_ESCAPE:
            return qmec::input::Key::Escape;
        default:
            return qmec::input::Key::Count;
        }
    }
}

namespace qmec::platform::windows
{
    Win32Window::~Win32Window() noexcept
    {
        const HWND window = static_cast<HWND>(handle_);
        if(window != nullptr && IsWindow(window) != FALSE)
        {
            DestroyWindow(window);
        }

        if(registeredWindowClass_)
        {
            UnregisterClassW(
                WindowClassName,
                static_cast<HINSTANCE>(instance_));
        }
    }

    bool Win32Window::Initialize(
        const wchar_t* title,
        std::uint32_t clientWidth,
        std::uint32_t clientHeight) noexcept
    {
        if(handle_ != nullptr || title == nullptr ||
           clientWidth == 0U || clientHeight == 0U)
        {
            return false;
        }

        const HINSTANCE instance = GetModuleHandleW(nullptr);

        WNDCLASSW windowClass{};
        windowClass.lpfnWndProc = WindowProcedure;
        windowClass.hInstance = instance;
        windowClass.lpszClassName = WindowClassName;
        windowClass.hCursor = LoadCursorW(nullptr, IDC_ARROW);

        if(RegisterClassW(&windowClass) == 0)
        {
            return false;
        }

        registeredWindowClass_ = true;
        instance_ = instance;

        constexpr DWORD WindowStyle = WS_OVERLAPPEDWINDOW;
        RECT windowRectangle{
            0,
            0,
            static_cast<LONG>(clientWidth),
            static_cast<LONG>(clientHeight)
        };

        if(AdjustWindowRect(&windowRectangle, WindowStyle, FALSE) == FALSE)
        {
            return false;
        }

        const HWND window = CreateWindowExW(
            0,
            WindowClassName,
            title,
            WindowStyle,
            CW_USEDEFAULT,
            CW_USEDEFAULT,
            windowRectangle.right - windowRectangle.left,
            windowRectangle.bottom - windowRectangle.top,
            nullptr,
            nullptr,
            instance,
            nullptr);

        if(window == nullptr)
        {
            return false;
        }

        handle_ = window;
        clientWidth_ = clientWidth;
        clientHeight_ = clientHeight;

        RAWINPUTDEVICE mouseDevice{};
        mouseDevice.hwndTarget = window;
        mouseDevice.usUsagePage = 0x01;
        mouseDevice.usUsage = 0x02;
        mouseDevice.dwFlags = 0;
        if (RegisterRawInputDevices(&mouseDevice,1U,sizeof(mouseDevice)) == FALSE)
        {
            return false;
        }
        
        ShowWindow(window, SW_SHOW);
        UpdateWindow(window);
        return true;
    }

    bool Win32Window::ProcessMessages() noexcept
    {
        MSG message{};
        while(PeekMessageW(&message, nullptr, 0, 0, PM_REMOVE) != FALSE)
        {
            if (message.message == WM_QUIT)
            {
                return false;
            }

            if (message.message == WM_INPUT)
            {
                RAWINPUT rawInput{};
                UINT dataSize = sizeof(rawInput);
               

                const UINT copiedBytes = GetRawInputData(
                    reinterpret_cast<HRAWINPUT>(message.lParam),
                    RID_INPUT,
                    &rawInput,
                    &dataSize,
                    sizeof(RAWINPUTHEADER));

                if (copiedBytes != static_cast<UINT>(-1) && rawInput.header.dwType == RIM_TYPEMOUSE)
                {
                    mouseDelta_.x +=
                        static_cast<float>(rawInput.data.mouse.lLastX);

                    mouseDelta_.y +=
                        static_cast<float>(rawInput.data.mouse.lLastY);
                }
            }

            switch(message.message)
            {
            case WM_LBUTTONDOWN:
                mouseButtonStates_[static_cast<std::size_t>(MouseButton::Left)] = true;
                SetCapture(static_cast<HWND>(handle_));
                break;
            case WM_LBUTTONUP:
                mouseButtonStates_[static_cast<std::size_t>(MouseButton::Left)] = false;
                break;
            case WM_MBUTTONDOWN:
                mouseButtonStates_[static_cast<std::size_t>(MouseButton::Middle)] = true;
                SetCapture(static_cast<HWND>(handle_));
                break;
            case WM_MBUTTONUP:
                mouseButtonStates_[static_cast<std::size_t>(MouseButton::Middle)] = false;
                break;
            case WM_RBUTTONDOWN:
                mouseButtonStates_[static_cast<std::size_t>(MouseButton::Right)] = true;
                SetCapture(static_cast<HWND>(handle_));
                break;
            case WM_RBUTTONUP:
                mouseButtonStates_[static_cast<std::size_t>(MouseButton::Right)] = false;
                break;
            case WM_MOUSEWHEEL:
                mouseWheelDelta_ += static_cast<float>(
                    GET_WHEEL_DELTA_WPARAM(message.wParam)) /
                    static_cast<float>(WHEEL_DELTA);
                break;
            default:
                break;
            }

            const bool anyMouseButtonDown =
                mouseButtonStates_[static_cast<std::size_t>(MouseButton::Left)] ||
                mouseButtonStates_[static_cast<std::size_t>(MouseButton::Middle)] ||
                mouseButtonStates_[static_cast<std::size_t>(MouseButton::Right)];

            if(!anyMouseButtonDown && GetCapture() == static_cast<HWND>(handle_))
            {
                ReleaseCapture();
            }

            const bool keyPressed = message.message == WM_KEYDOWN || message.message == WM_SYSKEYDOWN;

            const bool keyReleased = message.message == WM_KEYUP || message.message == WM_SYSKEYUP;

            if (keyPressed || keyReleased)
            {
                const Key key = TranslateKey(message.wParam);

                if (key != Key::Count)
                {
                    const std::size_t index = static_cast<std::size_t>(key);

                    keyStates_[index] = keyPressed;
                }
            }
            else if (message.message == WM_KILLFOCUS)
            {
                keyStates_.fill(false);
                mouseButtonStates_.fill(false);
                mouseDelta_ = {};
                mouseWheelDelta_ = 0.0f;
            }

            TranslateMessage(&message);
            DispatchMessageW(&message);
        }

        return true;
    }

    bool Win32Window::IsKeyDown(Key key) const noexcept
    {
        const std::size_t index = static_cast<std::size_t>(key);

        return index < keyStates_.size() && keyStates_[index];
    }

    bool Win32Window::IsMouseButtonDown(MouseButton button) const noexcept
    {
        const std::size_t index = static_cast<std::size_t>(button);

        return index < mouseButtonStates_.size() && mouseButtonStates_[index];
    }

    MouseDelta Win32Window::ConsumeMouseDelta() noexcept
    {
       
        const MouseDelta result = mouseDelta_;
        mouseDelta_ = {};
        return result;
    }

    float Win32Window::ConsumeMouseWheelDelta() noexcept
    {
        const float result = mouseWheelDelta_;
        mouseWheelDelta_ = 0.0f;
        return result;
    }

    void* Win32Window::NativeHandle() const noexcept
    {
        return handle_;
    }

    std::uint32_t Win32Window::ClientWidth() const noexcept
    {
        return clientWidth_;
    }

    std::uint32_t Win32Window::ClientHeight() const noexcept
    {
        return clientHeight_;
    }
}
