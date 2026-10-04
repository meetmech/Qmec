#pragma once

#include "QMEC/Math/Vec3.h"
#include "QMEC/Physics/NarrowPhase.h"

namespace qmec
{
   
    struct MinkowskiVertex
    {
        Vec3 pointA{};
        Vec3 pointB{};
        Vec3 difference{};
    };

    struct Simplex
    {
        MinkowskiVertex points[4]{};
        int count{ 0 };

        void Add(const MinkowskiVertex& point);
    };

    struct ConvexContact
    {
        Vec3 normal{};
        float penetration{ 0.0f };
        Vec3 position{};
    };

    class ConvexCollision
    {
    public:
        // GJK returns the overlap simplex because EPA needs it as its seed.
        static bool GJK(
            const NarrowPhaseCollider& colliderA,
            const NarrowPhaseCollider& colliderB,
            Simplex& simplexOut);

        static bool EPA(
            const NarrowPhaseCollider& colliderA,
            const NarrowPhaseCollider& colliderB,
            const Simplex& simplex,
            ConvexContact& contactOut);

        // Convenience entry point used by NarrowPhase.
        static bool Collide(
            const NarrowPhaseCollider& colliderA,
            const NarrowPhaseCollider& colliderB,
            ConvexContact& contactOut);

    private:
        static MinkowskiVertex Support(
            const NarrowPhaseCollider& colliderA,
            const NarrowPhaseCollider& colliderB,
            const Vec3& direction);

        static bool UpdateSimplex(Simplex& simplex, Vec3& direction);
        static bool HandleLine(Simplex& simplex, Vec3& direction);
        static bool HandleTriangle(Simplex& simplex, Vec3& direction);
        static bool HandleTetrahedron(Simplex& simplex, Vec3& direction);
    };
}
