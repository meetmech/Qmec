#pragma once

#include "QMEC/Graphics/Debug/DebugLine.h"

#include <array>
#include <span>
#include <vector>

namespace qmec
{
    struct AABB;
    struct WorldPlane;
    struct WorldBox;
    struct WorldCylinder;

 
    class DebugRenderer
    {
    public:
        void DrawLine(const Vec3& start, const Vec3& end,const Vec3& color = {1.0f, 1.0f, 1.0f});

        
        void DrawRay(const Vec3& origin, const Vec3& direction, float length,const Vec3& color = {1.0f, 1.0f, 1.0f});

        void DrawAABB(const AABB& bounds,const Vec3& color = {1.0f, 1.0f, 1.0f});

        void DrawSphere(const Vec3& center, float radius,const Vec3& color = {1.0f, 1.0f, 1.0f});

        void DrawPlane(const WorldPlane& plane,const Vec3& color = {1.0f, 1.0f, 1.0f});

        void DrawCylinder(const WorldCylinder& cylinder, const Vec3& color = {1.0f, 1.0f, 1.0f});

  
        void DrawBox(const Vec3& center, const Vec3& halfExtents, const Vec3& color = {1.0f, 1.0f, 1.0f});
        void DrawBox(const WorldBox& box,const Vec3& color = {1.0f, 1.0f, 1.0f});

        void DrawGrid();

        
        void DrawFrustum(const std::array<Vec3, 8>& corners,const Vec3& color = {1.0f, 1.0f, 1.0f});

        
        void Clear() noexcept;
        [[nodiscard]] std::span<const DebugLine> GetLines() const noexcept;

    private:
        std::vector<DebugLine> lines_;

        int sphereSegments = 24;
    };
}
