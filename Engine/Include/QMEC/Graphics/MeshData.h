#pragma once

#include <cstdint>
#include <initializer_list>
#include <vector>
#include "QMEC/Math/Vec3.h"

namespace qmec::graphics
{

    struct Vertex
    {
        Vec3 position;
        Vec3 normal;
        Vec3 tangent;
        Vec3 color;
        float uv[2]{};

        Vertex() = default;

        Vertex(const Vec3& vertexPosition, const Vec3& vertexNormal,const Vec3& vertexColor, std::initializer_list<float> textureCoordinates) noexcept
            : position(vertexPosition), normal(vertexNormal), color(vertexColor)
        {
            auto coordinate = textureCoordinates.begin();
            if (coordinate != textureCoordinates.end())
                uv[0] = *coordinate++;
            if (coordinate != textureCoordinates.end())
                uv[1] = *coordinate;
        }

    };

    struct MeshData
    {
        std::vector<Vertex> vertices;
        std::vector<std::uint32_t> indices;

    };
}

namespace qmec
{
    using graphics::Vertex;
    using graphics::MeshData;
}
