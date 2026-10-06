#pragma once

namespace qmec::scene::components
{
    struct CameraComponent
    {
        float verticalFieldOfViewRadians{1.0471975512f};
        float nearPlane{0.1f};
        float farPlane{100.0f};
        bool primary{};
    };
}

namespace qmec
{
    using scene::components::CameraComponent;
}
