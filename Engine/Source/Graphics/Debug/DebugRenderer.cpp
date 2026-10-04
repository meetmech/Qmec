#include "QMEC/Graphics/Debug/DebugRenderer.h"
#include "QMEC/Graphics/Debug/DebugLine.h"
#include "QMEC/Physics/AABB.h"
#include "QMEC/Physics/Collider/WorldShapes.h"
#include <cmath>

namespace qmec
{
    void DebugRenderer::DrawLine(const Vec3& start, const Vec3& end, const Vec3& color)
    {
       lines_.push_back(DebugLine{start, end, color});

    }

    void DebugRenderer::DrawRay(const Vec3& origin, const Vec3& direction, float length, const Vec3& color)
    {
        if (length <= 0.0f || direction.LengthSquared() <= 0.000000000001f)
            return;

        DrawLine(origin, origin + direction.Normalized() * length, color);
    }

    void DebugRenderer::DrawAABB(const AABB& bounds, const Vec3& color)
    {
        const Vec3 corners[8] =
        {
            { bounds.min.x, bounds.min.y, bounds.min.z },
            { bounds.max.x, bounds.min.y, bounds.min.z },
            { bounds.max.x, bounds.max.y, bounds.min.z },
            { bounds.min.x, bounds.max.y, bounds.min.z },

            { bounds.min.x, bounds.min.y, bounds.max.z },
            { bounds.max.x, bounds.min.y, bounds.max.z },
            { bounds.max.x, bounds.max.y, bounds.max.z },
            { bounds.min.x, bounds.max.y, bounds.max.z }
        };

        constexpr int edges[12][2] =
        {
            { 0, 1 },
            { 1, 2 },
            { 2, 3 },
            { 3, 0 },

            { 4, 5 },
            { 5, 6 },
            { 6, 7 },
            { 7, 4 },

            { 0, 4 },
            { 1, 5 },
            { 2, 6 },
            { 3, 7 }
        };

        for (const auto& edge : edges)
        {
            DrawLine(corners[edge[0]], corners[edge[1]], color);
        }
    }

    void DebugRenderer::DrawSphere( const Vec3& center,float radius,const Vec3& color)
    {
        if (radius <= 0.0f || sphereSegments < 3)
            return;

        constexpr float PI = 3.14159265359f;

        auto GetPoint = [&](int lat, int lon)
            {
                if (lat == 0)
                    return center + Vec3{0.0f, radius, 0.0f};
                if (lat == sphereSegments - 1)
                    return center + Vec3{0.0f, -radius, 0.0f};

                const float phi =static_cast<float>(lat) /static_cast<float>(sphereSegments - 1) *PI;

                const float theta =static_cast<float>(lon) /static_cast<float>(sphereSegments) *2.0f * PI;

                return center + Vec3{radius * std::sin(phi) * std::cos(theta),
                               radius * std::cos(phi),
                           radius * std::sin(phi) * std::sin(theta)};
            };

        for (int lat = 0; lat < sphereSegments; ++lat)
        {
            for (int lon = 0; lon < sphereSegments; ++lon)
            {
                const Vec3 current = GetPoint(lat, lon);

                const int nextLon =(lon + 1) % sphereSegments;

                const Vec3 nextLongitude = GetPoint(lat, nextLon);

                if (lat > 0 && lat < sphereSegments - 1)
                    DrawLine(current, nextLongitude, color);

          
                if (lat + 1 < sphereSegments)
                {
                    const Vec3 nextLatitude = GetPoint(lat + 1, lon);

                    DrawLine(current, nextLatitude, color);
                }
            }
        }
    }

    void DebugRenderer::DrawCylinder(const WorldCylinder& cylinder, const Vec3& color)
    {
        if (cylinder.radius <= 0.0f || cylinder.halfHeight <= 0.0f)
            return;

        constexpr int segments = 24;
        constexpr float twoPi = 6.28318530718f;
        const Vec3 height = cylinder.axisY * cylinder.halfHeight;
        for (int i = 0; i < segments; ++i)
        {
            const float angle = twoPi * static_cast<float>(i) / segments;
            const float nextAngle = twoPi * static_cast<float>((i + 1) % segments) / segments;
            const Vec3 current = cylinder.centre + cylinder.axisX * (cylinder.radius * std::cos(angle)) + cylinder.axisZ * (cylinder.radius * std::sin(angle));
            const Vec3 next = cylinder.centre
                + cylinder.axisX * (cylinder.radius * std::cos(nextAngle))
                + cylinder.axisZ * (cylinder.radius * std::sin(nextAngle));

            DrawLine(current - height, next - height, color);
            DrawLine(current + height, next + height, color);
            DrawLine(current - height, current + height, color);
        }
    }

    void DebugRenderer::DrawPlane(const WorldPlane& plane, const Vec3& color)
    {
        const Vec3 x = plane.axisX * plane.halfWidthX;
        const Vec3 z = plane.axisZ * plane.halfLengthZ;
        const Vec3 corners[]{
            plane.centre - x - z, plane.centre + x - z,
            plane.centre + x + z, plane.centre - x + z};
        for (int i = 0; i < 4; ++i)
            DrawLine(corners[i], corners[(i + 1) % 4], color);
    }

    void DebugRenderer::DrawBox(const Vec3& center, const Vec3& halfExtents, const Vec3& color)
    {
        DrawAABB(AABB{center - halfExtents, center + halfExtents}, color);
    }

    void DebugRenderer::DrawBox(const WorldBox& box, const Vec3& color)
    {
        const Vec3 x = box.axisX * box.halfExtents.x;
        const Vec3 y = box.axisY * box.halfExtents.y;
        const Vec3 z = box.axisZ * box.halfExtents.z;
        const std::array<Vec3, 8> corners{
            box.centre - x - y - z, box.centre + x - y - z,
            box.centre + x + y - z, box.centre - x + y - z,
            box.centre - x - y + z, box.centre + x - y + z,
            box.centre + x + y + z, box.centre - x + y + z};
        DrawFrustum(corners, color);
    }

    void DebugRenderer::DrawFrustum(const std::array<Vec3, 8>& corners, const Vec3& color)
    {
        for (int i = 0; i < 4; ++i)
        {
            const int next = (i + 1) % 4;
            DrawLine(corners[i], corners[next], color);
            DrawLine(corners[i + 4], corners[next + 4], color);
            DrawLine(corners[i], corners[i + 4], color);
        }
    }

    void DebugRenderer::Clear() noexcept
    {
        lines_.clear(); // Retain capacity for the next frame.
    }

    std::span<const DebugLine> DebugRenderer::GetLines() const noexcept
    {
        return {lines_.data(), lines_.size()};
    }
}
