#pragma once
#include "QMEC/Scene/Scene.h"

#include <string>

namespace qmec::scene
{
    class SceneSerializer
    {
    public:
        static bool Save(const Scene& scene, const std::string& path);
        static bool Load(Scene& scene, const std::string& path);
    };
}

namespace qmec
{
    using scene::SceneSerializer;
}
