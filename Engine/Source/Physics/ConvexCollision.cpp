#include "QMEC/Physics/ConvexCollision.h"

#include "QMEC/Physics/Collider/WorldShapes.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <unordered_map>
#include <utility>
#include <vector>

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>

namespace qmec
{
    namespace
    {
        constexpr float GjkEpsilon = 0.000001f;
        constexpr float GjkEpsilonSquared = GjkEpsilon * GjkEpsilon;
        constexpr float EpaEpsilon = 0.00001f;
        constexpr int GjkMaximumIterations = 32;
        constexpr int EpaMaximumIterations = 64;
        constexpr size_t EpaMaximumVertices = 128;
        constexpr size_t EpaMaximumFaces = 512;
        constexpr size_t EpaMaximumBoundaryEdges = 512;

        struct EpaFace
        {
            int a{ 0 };
            int b{ 0 };
            int c{ 0 };
            Vec3 normal{};
            float distance{ 0.0f };
        };

        struct EpaEdge
        {
            int a{ 0 };
            int b{ 0 };
        };

        struct EpaEdgeUse
        {
            int low{ 0 };
            int high{ 0 };
            int directionBalance{ 0 };
            int useCount{ 0 };
        };

        bool IsSupportedConvexShape(const NarrowPhaseCollider& collider) noexcept
        {
            if (collider.collider == nullptr || collider.transform == nullptr)
            {
                return false;
            }

            return std::holds_alternative<BoxShape>(collider.collider->shape) || std::holds_alternative<CylinderShape>(collider.collider->shape);
        }

        Vec3 GetWorldCentre(const NarrowPhaseCollider& collider) noexcept
        {
            if (const auto* box = std::get_if<BoxShape>(&collider.collider->shape))
            {
                return ToWorldShape(*box, *collider.transform).centre;
            }

            if (const auto* cylinder = std::get_if<CylinderShape>(&collider.collider->shape))
            {
                return ToWorldShape(*cylinder, *collider.transform).centre;
            }

            return collider.transform->position;
        }

        Vec3 GetWorldSupportPoint(const NarrowPhaseCollider& collider,const Vec3& direction) noexcept
        {
            if (const auto* boxShape = std::get_if<BoxShape>(&collider.collider->shape))
            {
                const WorldBox box = ToWorldShape(*boxShape, *collider.transform);
                const float xSign = Dot(direction, box.axisX) >= 0.0f ? 1.0f : -1.0f;
                const float ySign = Dot(direction, box.axisY) >= 0.0f ? 1.0f : -1.0f;
                const float zSign = Dot(direction, box.axisZ) >= 0.0f ? 1.0f : -1.0f;

                return box.centre + box.axisX * (xSign * box.halfExtents.x) + box.axisY * (ySign * box.halfExtents.y) + box.axisZ * (zSign * box.halfExtents.z);
            }

            if (const auto* cylinderShape = std::get_if<CylinderShape>(&collider.collider->shape))
            {
                const WorldCylinder cylinder = ToWorldShape(*cylinderShape, *collider.transform);
                const float axialProjection = Dot(direction, cylinder.axisY);
                const float capSign = axialProjection >= 0.0f ? 1.0f : -1.0f;
                Vec3 support = cylinder.centre + cylinder.axisY * (capSign * cylinder.halfHeight);

                const Vec3 radialDirection = direction - cylinder.axisY * axialProjection;
                const float radialLengthSquared = radialDirection.LengthSquared();
                if (radialLengthSquared > GjkEpsilonSquared)
                {
                    support += radialDirection * (cylinder.radius / std::sqrt(radialLengthSquared));
                }

                return support;
            }

            return {};
        }

        bool MakeEpaFace(
            const std::vector<MinkowskiVertex>& vertices,
            int a,
            int b,
            int c,
            EpaFace& faceOut) noexcept
        {
            Vec3 normal = Cross(vertices[b].difference - vertices[a].difference,vertices[c].difference - vertices[a].difference);
            const float normalLengthSquared = normal.LengthSquared();
            if (normalLengthSquared <= GjkEpsilonSquared)
            {
                return false;
            }

            normal = normal / std::sqrt(normalLengthSquared);
            float distance = Dot(normal, vertices[a].difference);
            if (distance < 0.0f)
            {
                std::swap(b, c);
                normal = -normal;
                distance = -distance;
            }

            faceOut = { a, b, c, normal, distance };
            return true;
        }

        Vec3 InterpolateWitness(const MinkowskiVertex& a,const MinkowskiVertex& b,const MinkowskiVertex& c,float weightA,float weightB,float weightC,bool usePointA) noexcept
        {
            const Vec3& pointA = usePointA ? a.pointA : a.pointB;
            const Vec3& pointB = usePointA ? b.pointA : b.pointB;
            const Vec3& pointC = usePointA ? c.pointA : c.pointB;
            return pointA * weightA + pointB * weightB + pointC * weightC;
        }

        void BuildContactFromFace(const std::vector<MinkowskiVertex>& vertices,const EpaFace& face,ConvexContact& contactOut) noexcept
        {
            const MinkowskiVertex& a = vertices[face.a];
            const MinkowskiVertex& b = vertices[face.b];
            const MinkowskiVertex& c = vertices[face.c];
            const Vec3 closestPoint = face.normal * face.distance;

            const Vec3 edgeAB = b.difference - a.difference;
            const Vec3 edgeAC = c.difference - a.difference;
            const Vec3 toPoint = closestPoint - a.difference;
            const float d00 = Dot(edgeAB, edgeAB);
            const float d01 = Dot(edgeAB, edgeAC);
            const float d11 = Dot(edgeAC, edgeAC);
            const float d20 = Dot(toPoint, edgeAB);
            const float d21 = Dot(toPoint, edgeAC);
            const float denominator = d00 * d11 - d01 * d01;

            float weightA = 1.0f;
            float weightB = 0.0f;
            float weightC = 0.0f;
            if (std::abs(denominator) > GjkEpsilonSquared)
            {
                weightB = (d11 * d20 - d01 * d21) / denominator;
                weightC = (d00 * d21 - d01 * d20) / denominator;
                weightA = 1.0f - weightB - weightC;
                weightA = (std::max)(0.0f, weightA);
                weightB = (std::max)(0.0f, weightB);
                weightC = (std::max)(0.0f, weightC);
                const float weightSum = weightA + weightB + weightC;
                if (weightSum > GjkEpsilon)
                {
                    weightA /= weightSum;
                    weightB /= weightSum;
                    weightC /= weightSum;
                }
                else
                {
                    weightA = weightB = weightC = 1.0f / 3.0f;
                }
            }

            const Vec3 witnessA = InterpolateWitness(a, b, c, weightA, weightB, weightC, true);
            const Vec3 witnessB = InterpolateWitness(a, b, c, weightA, weightB, weightC, false);

            contactOut.normal = face.normal;
            contactOut.penetration = (std::max)(0.0f, face.distance);
            contactOut.position = (witnessA + witnessB) * 0.5f;
        }
    }

    void Simplex::Add(const MinkowskiVertex& point)
    {
        const int newCount = (std::min)(count + 1, 4);
        for (int index = newCount - 1; index > 0; --index)
        {
            points[index] = points[index - 1];
        }

        points[0] = point;
        count = newCount;
    }

    bool ConvexCollision::GJK(const NarrowPhaseCollider& colliderA,const NarrowPhaseCollider& colliderB,Simplex& simplexOut)
    {
        simplexOut = {};
        if (!IsSupportedConvexShape(colliderA) || !IsSupportedConvexShape(colliderB))
        {
            return false;
        }

        Vec3 direction = GetWorldCentre(colliderB) - GetWorldCentre(colliderA);
        if (direction.LengthSquared() <= GjkEpsilonSquared)
        {
            direction = { 1.0f, 0.0f, 0.0f };
        }

        simplexOut.Add(Support(colliderA, colliderB, direction));
        direction = -simplexOut.points[0].difference;
        if (direction.LengthSquared() <= GjkEpsilonSquared)
        {
            return true;
        }

        for (int iteration = 0; iteration < GjkMaximumIterations; ++iteration)
        {
            const MinkowskiVertex supportPoint = Support(colliderA, colliderB, direction);
            const float projection = Dot(supportPoint.difference, direction);
            const float projectionTolerance = GjkEpsilon * (std::max)(1.0f, direction.Length() * supportPoint.difference.Length());

            if (projection < -projectionTolerance)
            {
                return false;
            }

            bool duplicateSupport = false;
            for (int pointIndex = 0; pointIndex < simplexOut.count; ++pointIndex)
            {
                if ((supportPoint.difference - simplexOut.points[pointIndex].difference)
                    .LengthSquared() <= GjkEpsilonSquared)
                {
                    duplicateSupport = true;
                    break;
                }
            }

            if (duplicateSupport)
            {
                return projection <= projectionTolerance;
            }

            simplexOut.Add(supportPoint);
            if (UpdateSimplex(simplexOut, direction))
            {
                return true;
            }

            if (direction.LengthSquared() <= GjkEpsilonSquared)
            {
                return true;
            }
        }

        return false;
    }

    bool ConvexCollision::EPA(const NarrowPhaseCollider& colliderA, const NarrowPhaseCollider& colliderB,const Simplex& simplex,ConvexContact& contactOut)
    {
        if (simplex.count != 4 || !IsSupportedConvexShape(colliderA) || !IsSupportedConvexShape(colliderB))
        {
            return false;
        }

        std::vector<MinkowskiVertex> vertices(simplex.points, simplex.points + simplex.count);
        std::vector<EpaFace> faces;
        faces.reserve(64);

        const bool isBoxCylinderPair = std::holds_alternative<BoxShape>(colliderA.collider->shape)&& std::holds_alternative<CylinderShape>(colliderB.collider->shape);
        static std::uint64_t boxCylinderEpaCallCount = 0;
        const bool traceThisCall = isBoxCylinderPair&& (++boxCylinderEpaCallCount % 60U == 1U);

        const int initialTriangles[4][3]
        {
            { 0, 1, 2 }, { 0, 3, 1 }, { 0, 2, 3 }, { 1, 3, 2 }
        };
        for (const auto& triangle : initialTriangles)
        {
            EpaFace face{};
            if (MakeEpaFace(vertices, triangle[0], triangle[1], triangle[2], face))
            {
                faces.push_back(face);
            }
        }
        if (faces.size() < 4)
        {
            return false;
        }

        const auto closestFaceIndex = [&faces]() -> size_t
        {
            size_t closestIndex = 0;
            for (size_t faceIndex = 1; faceIndex < faces.size(); ++faceIndex)
            {
                if (faces[faceIndex].distance < faces[closestIndex].distance)
                {
                    closestIndex = faceIndex;
                }
            }
            return closestIndex;
        };

        for (int iteration = 0; iteration < EpaMaximumIterations; ++iteration)
        {
            if (faces.empty())
            {
                return false;
            }

            if (traceThisCall)
            {
                char message[192]{};
                std::snprintf(message, sizeof(message),"[EPA box-cylinder] iter=%d start vertices=%zu faces=%zu\n",iteration, vertices.size(), faces.size());
                OutputDebugStringA(message);
            }

            const size_t closestIndex = closestFaceIndex();
            const EpaFace closestFace = faces[closestIndex];
            if (vertices.size() >= EpaMaximumVertices
                || faces.size() > EpaMaximumFaces)
            {
                BuildContactFromFace(vertices, closestFace, contactOut);
                return true;
            }

            const MinkowskiVertex supportPoint = Support(
                colliderA, colliderB, closestFace.normal);
            const float supportDistance = Dot(
                supportPoint.difference, closestFace.normal);
            const float convergenceTolerance = EpaEpsilon
                * (std::max)(1.0f, std::abs(supportDistance));

            bool duplicateSupport = false;
            for (const MinkowskiVertex& vertex : vertices)
            {
                if ((supportPoint.difference - vertex.difference).LengthSquared()<= GjkEpsilonSquared)
                {
                    duplicateSupport = true;
                    break;
                }
            }

            if (supportDistance - closestFace.distance <= convergenceTolerance
                || duplicateSupport)
            {
                BuildContactFromFace(vertices, closestFace, contactOut);
                return true;
            }

            const int newVertexIndex = static_cast<int>(vertices.size());
            vertices.push_back(supportPoint);
            std::unordered_map<std::uint64_t, EpaEdgeUse> edgeUses;
            edgeUses.reserve((std::min)(faces.size() * 3U, EpaMaximumBoundaryEdges));
            std::vector<EpaFace> remainingFaces;
            remainingFaces.reserve(faces.size());
            size_t visibleFaceCount = 0;
            size_t nextBoundarySizeLog = 1;

            const auto addBoundaryEdge = [
                &edgeUses, &nextBoundarySizeLog, &vertices, &faces,
                traceThisCall, iteration](int edgeA, int edgeB)
            {
                const int low = (std::min)(edgeA, edgeB);
                const int high = (std::max)(edgeA, edgeB);
                const std::uint64_t edgeKey = (static_cast<std::uint64_t>(static_cast<std::uint32_t>(low)) << 32U) | static_cast<std::uint32_t>(high);
                auto edge = edgeUses.find(edgeKey);

                if (edge == edgeUses.end()
                    && edgeUses.size() >= EpaMaximumBoundaryEdges)
                {
                    return false;
                }

                if (traceThisCall && edgeUses.size() >= nextBoundarySizeLog)
                {
                    char message[192]{};
                    std::snprintf(message, sizeof(message),"[EPA box-cylinder] iter=%d scan uniqueEdges=%zu vertices=%zu faces=%zu\n",iteration, edgeUses.size(), vertices.size(), faces.size());
                    OutputDebugStringA(message);
                    nextBoundarySizeLog *= 2;
                }

                if (edge == edgeUses.end())
                {
                    const int direction = edgeA == low ? 1 : -1;
                    edgeUses.emplace(edgeKey, EpaEdgeUse{ low, high, direction, 1 });
                }
                else
                {
                    EpaEdgeUse& use = edge->second;
                    ++use.useCount;
                    use.directionBalance += edgeA == low ? 1 : -1;
                    if (use.useCount > 2)
                    {
                        return false;
                    }
                }

                return true;
            };

            bool validHorizon = true;
            for (const EpaFace& face : faces)
            {
                const Vec3 fromFace = supportPoint.difference - vertices[face.a].difference;
                if (Dot(face.normal, fromFace) > EpaEpsilon)
                {
                    ++visibleFaceCount;
                    if (!addBoundaryEdge(face.a, face.b)|| !addBoundaryEdge(face.b, face.c)|| !addBoundaryEdge(face.c, face.a))
                    {
                        validHorizon = false;
                        break;
                    }
                }
                else
                {
                    remainingFaces.push_back(face);
                }
            }

            std::vector<EpaEdge> boundaryEdges;
            boundaryEdges.reserve(edgeUses.size());
            if (validHorizon && visibleFaceCount > 0)
            {
                for (const auto& [key, use] : edgeUses)
                {
                    (void)key;
                    if (use.useCount == 1)
                    {
                        boundaryEdges.push_back(use.directionBalance > 0? EpaEdge{ use.low, use.high } : EpaEdge{ use.high, use.low });
                    }
                    else if (use.useCount != 2 || use.directionBalance != 0)
                    {
                        validHorizon = false;
                        break;
                    }
                }
            }

            if (!validHorizon || visibleFaceCount == 0 || boundaryEdges.empty() || boundaryEdges.size() > EpaMaximumBoundaryEdges)
            {
                BuildContactFromFace(vertices, closestFace, contactOut);
                return true;
            }

            std::vector<EpaFace> expandedFaces = std::move(remainingFaces);
            expandedFaces.reserve((std::min)( EpaMaximumFaces, expandedFaces.size() + boundaryEdges.size()));
            for (const EpaEdge& edge : boundaryEdges)
            {
                EpaFace face{};
                if (!MakeEpaFace(vertices, edge.a, edge.b, newVertexIndex, face) || expandedFaces.size() >= EpaMaximumFaces)
                {
                    validHorizon = false;
                    break;
                }
                expandedFaces.push_back(face);
            }

            if (!validHorizon || expandedFaces.size() < 4)
            {
                BuildContactFromFace(vertices, closestFace, contactOut);
                return true;
            }

            faces.swap(expandedFaces);
        }

        if (faces.empty())
        {
            return false;
        }

        BuildContactFromFace(vertices, faces[closestFaceIndex()], contactOut);
        return true;
    }

    bool ConvexCollision::Collide(const NarrowPhaseCollider& colliderA,const NarrowPhaseCollider& colliderB, ConvexContact& contactOut)
    {
        Simplex simplex{};
        if (!GJK(colliderA, colliderB, simplex))
        {
            return false;
        }

        if (!EPA(colliderA, colliderB, simplex, contactOut))
        {
            Vec3 normal = (GetWorldCentre(colliderB) - GetWorldCentre(colliderA)).Normalized();
            if (normal.LengthSquared() <= GjkEpsilonSquared)
            {
                normal = { 1.0f, 0.0f, 0.0f };
            }

            const Vec3 pointA = GetWorldSupportPoint(colliderA, normal);
            const Vec3 pointB = GetWorldSupportPoint(colliderB, -normal);
            contactOut.normal = normal;
            contactOut.penetration = 0.0f;
            contactOut.position = (pointA + pointB) * 0.5f;
        }

        const Vec3 centreDirection = GetWorldCentre(colliderB) - GetWorldCentre(colliderA);
        if (Dot(contactOut.normal, centreDirection) < 0.0f)
        {
            contactOut.normal = -contactOut.normal;
        }
        return true;
    }

    MinkowskiVertex ConvexCollision::Support(const NarrowPhaseCollider& colliderA,const NarrowPhaseCollider& colliderB,const Vec3& direction)
    {
        const Vec3 pointA = GetWorldSupportPoint(colliderA, direction);
        const Vec3 pointB = GetWorldSupportPoint(colliderB, -direction);
        return { pointA, pointB, pointA - pointB };
    }

    bool ConvexCollision::UpdateSimplex(Simplex& simplex, Vec3& direction)
    {
        switch (simplex.count)
        {
        case 2:
            return HandleLine(simplex, direction);
        case 3:
            return HandleTriangle(simplex, direction);
        case 4:
            return HandleTetrahedron(simplex, direction);
        default:
            direction = simplex.count > 0 ? -simplex.points[0].difference : Vec3{ 1.0f, 0.0f, 0.0f };
            return direction.LengthSquared() <= GjkEpsilonSquared;
        }
    }

    bool ConvexCollision::HandleLine(Simplex& simplex, Vec3& direction)
    {
        const Vec3 pointA = simplex.points[0].difference;
        const Vec3 pointB = simplex.points[1].difference;
        const Vec3 ao = -pointA;
        const Vec3 ab = pointB - pointA;

        if (Dot(ab, ao) > 0.0f)
        {
            direction = Cross(Cross(ab, ao), ab);
            if (direction.LengthSquared() <= GjkEpsilonSquared)
            {
                const float abLengthSquared = ab.LengthSquared();
                if (abLengthSquared <= GjkEpsilonSquared)
                {
                    direction = ao;
                    return direction.LengthSquared() <= GjkEpsilonSquared;
                }

                const float t = std::clamp(Dot(ao, ab) / abLengthSquared, 0.0f, 1.0f);
                const Vec3 closestPoint = pointA + ab * t;
                if (closestPoint.LengthSquared() <= GjkEpsilonSquared)
                {
                    return true;
                }

                direction = -closestPoint;
            }
        }
        else
        {
            simplex.count = 1;
            direction = ao;
        }

        return direction.LengthSquared() <= GjkEpsilonSquared;
    }

    bool ConvexCollision::HandleTriangle(Simplex& simplex, Vec3& direction)
    {
        const MinkowskiVertex pointA = simplex.points[0];
        const MinkowskiVertex pointB = simplex.points[1];
        const MinkowskiVertex pointC = simplex.points[2];
        const Vec3 ao = -pointA.difference;
        const Vec3 ab = pointB.difference - pointA.difference;
        const Vec3 ac = pointC.difference - pointA.difference;
        const Vec3 abc = Cross(ab, ac);

        if (abc.LengthSquared() <= GjkEpsilonSquared)
        {
            if (ab.LengthSquared() >= ac.LengthSquared())
            {
                simplex.points[1] = pointB;
            }
            else
            {
                simplex.points[1] = pointC;
            }
            simplex.count = 2;
            return HandleLine(simplex, direction);
        }

        const Vec3 abOutside = Cross(ab, abc);
        if (Dot(abOutside, ao) > GjkEpsilon)
        {
            simplex.points[1] = pointB;
            simplex.count = 2;
            return HandleLine(simplex, direction);
        }

        const Vec3 acOutside = Cross(abc, ac);
        if (Dot(acOutside, ao) > GjkEpsilon)
        {
            simplex.points[1] = pointC;
            simplex.count = 2;
            return HandleLine(simplex, direction);
        }

        if (Dot(abc, ao) >= 0.0f)
        {
            direction = abc;
        }
        else
        {
            simplex.points[1] = pointC;
            simplex.points[2] = pointB;
            direction = -abc;
        }

        return false;
    }

    bool ConvexCollision::HandleTetrahedron(Simplex& simplex, Vec3& direction)
    {
        const MinkowskiVertex pointA = simplex.points[0];
        const MinkowskiVertex pointB = simplex.points[1];
        const MinkowskiVertex pointC = simplex.points[2];
        const MinkowskiVertex pointD = simplex.points[3];
        const Vec3 ao = -pointA.difference;

        const auto testFace = [&](const MinkowskiVertex& pointOnFaceB, const MinkowskiVertex& pointOnFaceC, const MinkowskiVertex& oppositePoint) -> bool
        {
            Vec3 faceNormal = Cross(pointOnFaceB.difference - pointA.difference, pointOnFaceC.difference - pointA.difference);
            if (faceNormal.LengthSquared() <= GjkEpsilonSquared)
            {
                return false;
            }

            if (Dot(faceNormal, oppositePoint.difference - pointA.difference) > 0.0f)
            {
                faceNormal = -faceNormal;
            }

            if (Dot(faceNormal, ao) > GjkEpsilon)
            {
                simplex.points[1] = pointOnFaceB;
                simplex.points[2] = pointOnFaceC;
                simplex.count = 3;
                return true;
            }
            return false;
        };

        if (testFace(pointB, pointC, pointD))
        {
            return HandleTriangle(simplex, direction);
        }
        if (testFace(pointC, pointD, pointB))
        {
            return HandleTriangle(simplex, direction);
        }
        if (testFace(pointD, pointB, pointC))
        {
            return HandleTriangle(simplex, direction);
        }

        return true;
    }
}
