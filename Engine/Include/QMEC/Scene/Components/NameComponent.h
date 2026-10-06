#pragma once

#include <string>

namespace qmec::scene::components
{
    struct NameComponent
    {
        std::string name{"Entity"};
    };
}

namespace qmec
{
    using scene::components::NameComponent;
}
