#include "QMEC/Physics/BroadPhase.h"
#include "QMEC/ECS/Registry.h"
#include "QMEC/Physics/NarrowPhase.h"
#include "QMEC/Physics/CollisionPair.h"
#include "QMEC/Physics/ConvexCollision.h"
#include "QMEC/Math/Vec3.h"
#include "QMEC/Math/Mat4.h"
#include "QMEC/Physics/ContactManifold.h"
#include "QMEC/Physics/Collider/WorldShapes.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>
#include <utility>

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>

namespace qmec::physics
{
    static std::vector<Vec3> sutherlandHodgmanClip(const std::vector<Vec3>& clipPolygon,const clipPlane& plane)
    {
        std::vector<Vec3> clippedPoints;

        if (clipPolygon.empty())
            return clippedPoints;

        clippedPoints.reserve(clipPolygon.size() + 2);

        for (size_t i = 0; i < clipPolygon.size(); ++i)
        {
            const Vec3& pointOne = clipPolygon[i];
            const Vec3& pointTwo = clipPolygon[(i + 1) % clipPolygon.size()];

            float distanceOne = Dot(pointOne, plane.normal) - plane.d;

            float distanceTwo = Dot(pointTwo, plane.normal) - plane.d;

            bool oneInside = distanceOne <= 0.0f;
            bool twoInside = distanceTwo <= 0.0f;
            if (oneInside && twoInside)
            {
                clippedPoints.push_back(pointTwo);
            }

       
            else if (oneInside && !twoInside)
            {
                Vec3 edge = pointTwo - pointOne;

                float denominator = Dot(plane.normal, edge);

                if (std::abs(denominator) > 0.000001f)
                {
                    float t = (plane.d - Dot(plane.normal, pointOne)) / denominator;

                    Vec3 intersectionPoint =pointOne + edge * t;

                    clippedPoints.push_back(intersectionPoint);
                }
            }

         
            else if (!oneInside && twoInside)
            {
                Vec3 edge = pointTwo - pointOne;

                float denominator = Dot(plane.normal, edge);

                if (std::abs(denominator) > 0.000001f)
                {
                    float t = (plane.d - Dot(plane.normal, pointOne))/ denominator;

                    Vec3 intersectionPoint = pointOne + edge * t;

                    clippedPoints.push_back(intersectionPoint);
                }

                clippedPoints.push_back(pointTwo);
            }

        }

        return clippedPoints;
    }

    static bool buildConvexManifold(
        const NarrowPhaseCollider& colliderA,
        const NarrowPhaseCollider& colliderB,
        ContactManifold& manifoldOut)
    {
        if (colliderA.transform == nullptr || colliderB.transform == nullptr)
        {
            return false;
        }

        ConvexContact convexContact{};
        if (!ConvexCollision::Collide(colliderA, colliderB, convexContact))
        {
            return false;
        }

        ContactManifold manifold{};
        manifold.bodyOne.transform = colliderA.transform;
        manifold.bodyOne.rigidBody = colliderA.rigidbody;
        manifold.bodyTwo.transform = colliderB.transform;
        manifold.bodyTwo.rigidBody = colliderB.rigidbody;
        manifold.normal = convexContact.normal;

        ContactPoint contact{};
        contact.penetration = convexContact.penetration;
        contact.position = convexContact.position;
        manifold.contacts.push_back(contact);

        manifoldOut = std::move(manifold);
        return true;
    }


    bool NarrowPhase::findNarrowPhasePairs(std::vector<NarrowPhaseColliderPairs> broadPhaseCollisions, std::vector<ContactManifold>& manifoldsOut)
    {
        manifoldsOut.clear();
        manifoldsOut.reserve(broadPhaseCollisions.size());

        for (size_t i = 0; i < broadPhaseCollisions.size(); ++i)
        {
            auto& first = broadPhaseCollisions[i].first;
            auto& second = broadPhaseCollisions[i].second;

            auto& shapeA = first.collider->shape;
            auto& shapeB = second.collider->shape;
            ContactManifold manifold{};
            bool foundCollision = false;
           
            if (std::holds_alternative<BoxShape>(shapeA) && std::holds_alternative<PlaneShape>(shapeB))
            {
                if (testBoxPlane(first, second, manifold))
                    foundCollision = true;
            }
            else if (std::holds_alternative<PlaneShape>(shapeA) && std::holds_alternative<BoxShape>(shapeB))
            {
                
                if (testBoxPlane(second, first, manifold))
                    foundCollision = true;
            }

            if (std::holds_alternative<SphereShape>(shapeA) && std::holds_alternative<PlaneShape>(shapeB))
            {
                if (testSpherePlane(first, second, manifold))
                    foundCollision = true;
            }
            else if (std::holds_alternative<PlaneShape>(shapeA) && std::holds_alternative<SphereShape>(shapeB))
            {

                if (testSpherePlane(second, first, manifold))
                    foundCollision = true;
            }


            if (std::holds_alternative<SphereShape>(shapeA) && std::holds_alternative<BoxShape>(shapeB))
            {
                if (testSphereBox(second, first, manifold))
                    foundCollision = true;
            }
            else if (std::holds_alternative<BoxShape>(shapeA) && std::holds_alternative<SphereShape>(shapeB))
            {

                if (testSphereBox(first, second, manifold))
                    foundCollision = true;
            }

            if (std::holds_alternative<SphereShape>(shapeA) && std::holds_alternative<SphereShape>(shapeB))
            {
                if (testSphereSphere(first, second, manifold))
                    foundCollision = true;
            }

            if (std::holds_alternative<BoxShape>(shapeA) && std::holds_alternative<BoxShape>(shapeB))
            {
                if (testBoxBox(first, second, manifold))
                    foundCollision = true;
            }


            if (std::holds_alternative<SphereShape>(shapeA) && std::holds_alternative<CylinderShape>(shapeB))
            {
                if (testCylinderSphere(second, first, manifold))
                    foundCollision = true;
            }

            if (std::holds_alternative<CylinderShape>(shapeA) && std::holds_alternative<SphereShape>(shapeB))
            {
                if (testCylinderSphere(first, second, manifold))
                    foundCollision = true;
            }


            if (std::holds_alternative<CylinderShape>(shapeA) && std::holds_alternative<PlaneShape>(shapeB))
            {
                if (testCylinderPlane(first, second, manifold))
                    foundCollision = true;
            }
            else if (std::holds_alternative<PlaneShape>(shapeA) && std::holds_alternative<CylinderShape>(shapeB))
            {
                if (testCylinderPlane(second, first, manifold))
                    foundCollision = true;
            }

            if (std::holds_alternative<BoxShape>(shapeA)
                && std::holds_alternative<CylinderShape>(shapeB))
            {
                if (testBoxCylinder(first, second, manifold))
                    foundCollision = true;
            }
            else if (std::holds_alternative<CylinderShape>(shapeA)
                && std::holds_alternative<BoxShape>(shapeB))
            {
                if (testBoxCylinder(second, first, manifold))
                    foundCollision = true;
            }

            if (std::holds_alternative<CylinderShape>(shapeA)
                && std::holds_alternative<CylinderShape>(shapeB))
            {
                if (testCylinderCylinder(first, second, manifold))
                    foundCollision = true;
            }

            if (foundCollision)
                manifoldsOut.push_back(std::move(manifold));
        }

        return !manifoldsOut.empty();
    }


    bool NarrowPhase::testBoxPlane(const NarrowPhaseCollider& boxCollider,const NarrowPhaseCollider& planeCollider,ContactManifold& outManifold)
    {
        const WorldBox boxShape = ToWorldShape(std::get<BoxShape>(boxCollider.collider->shape), *boxCollider.transform);

        const WorldPlane planeShape = ToWorldShape(std::get<PlaneShape>(planeCollider.collider->shape), *planeCollider.transform);

        Vec3 axisX = boxShape.axisX;

        Vec3 axisY = boxShape.axisY;

        Vec3 axisZ = boxShape.axisZ;

        Vec3 axisXPlane = planeShape.axisX;

        Vec3 planeNormal = planeShape.normal;

        Vec3 axisZPlane = planeShape.axisZ;
   

        float projectedRadius =boxShape.halfExtents.x *std::abs(Dot(axisX, planeNormal)) + boxShape.halfExtents.y * std::abs(Dot(axisY, planeNormal)) + boxShape.halfExtents.z * std::abs(Dot(axisZ, planeNormal));

        float distance =Dot(boxShape.centre - planeShape.centre,planeNormal);

        if (distance > projectedRadius)
            return false;

        float xDot = Dot(planeNormal, axisX);
        float yDot = Dot(planeNormal, axisY);
        float zDot = Dot(planeNormal, axisZ);

        float xAlignment = std::abs(xDot);
        float yAlignment = std::abs(yDot);
        float zAlignment = std::abs(zDot);

        float maxAlignment = std::max({xAlignment,yAlignment,zAlignment});

        Vec3 correctAxis{};
        Vec3 uDirection{};
        Vec3 vDirection{};

        float correctExtent = 0.0f;
        float uExtent = 0.0f;
        float vExtent = 0.0f;
        float sign = 1.0f;

        if (maxAlignment == xAlignment)
        {
            correctAxis = axisX;

            float axisPlaneSign = xDot >= 0.0f ? 1.0f : -1.0f;

            sign = -axisPlaneSign;

            correctExtent = boxShape.halfExtents.x;

            uDirection = axisY;
            vDirection = axisZ;

            uExtent = boxShape.halfExtents.y;
            vExtent = boxShape.halfExtents.z;
        }
        else if (maxAlignment == yAlignment)
        {
            correctAxis = axisY;

            float axisPlaneSign = yDot >= 0.0f ? 1.0f : -1.0f;

            sign = -axisPlaneSign;

            correctExtent = boxShape.halfExtents.y;

            uDirection = axisX;
            vDirection = axisZ;

            uExtent = boxShape.halfExtents.x;
            vExtent = boxShape.halfExtents.z;
        }
        else
        {
            correctAxis = axisZ;

            float axisPlaneSign = zDot >= 0.0f ? 1.0f : -1.0f;

            sign = -axisPlaneSign;

            correctExtent = boxShape.halfExtents.z;

            uDirection = axisX;
            vDirection = axisY;

            uExtent = boxShape.halfExtents.x;
            vExtent = boxShape.halfExtents.y;
        }


        Vec3 faceCentre = boxShape.centre + correctAxis * (correctExtent * sign);

        std::vector<Vec3> faceVertices
        {
            faceCentre + uDirection * uExtent - vDirection * vExtent,

            faceCentre + uDirection * uExtent + vDirection * vExtent,

            faceCentre - uDirection * uExtent + vDirection * vExtent,

            faceCentre - uDirection * uExtent- vDirection * vExtent
        };


        float planeCentreX = Dot(planeShape.centre, axisXPlane);
        float planeCentreZ = Dot(planeShape.centre, axisZPlane);


        clipPlane positiveX
        {
            axisXPlane,
            planeCentreX + planeShape.halfWidthX
        };

        clipPlane negativeX
        {
            axisXPlane * -1.0f,
            -planeCentreX + planeShape.halfWidthX
        };

        clipPlane positiveZ
        {
            axisZPlane,
            planeCentreZ + planeShape.halfLengthZ
        };

        clipPlane negativeZ
        {
            axisZPlane * -1.0f,
            -planeCentreZ + planeShape.halfLengthZ
        };


        std::vector<Vec3> clipped = faceVertices;

        clipped = sutherlandHodgmanClip(clipped,positiveX);

        clipped = sutherlandHodgmanClip(clipped,negativeX);

        clipped = sutherlandHodgmanClip(clipped,positiveZ);

        clipped =sutherlandHodgmanClip(clipped,negativeZ);

        if (clipped.empty())
            return false;


        ContactManifold manifold{};

        manifold.normal = -planeNormal;
        manifold.bodyOne.rigidBody = boxCollider.rigidbody;
        manifold.bodyOne.transform = boxCollider.transform;
        manifold.bodyTwo.rigidBody = planeCollider.rigidbody;
        manifold.bodyTwo.transform = planeCollider.transform;

        for (const Vec3& point : clipped)
        {
            float pointDistance = Dot(point - planeShape.centre,planeNormal);
            Vec3 contactPoint = point - planeNormal * pointDistance;
            const float penetration = -pointDistance;

            if (penetration >= 0.0f)
            {
                manifold.contacts.push_back(ContactPoint{penetration,contactPoint});
            }
        }


        if (manifold.contacts.empty())
            return false;

        outManifold = std::move(manifold);

        return true;
    }


    bool NarrowPhase::testSpherePlane(const NarrowPhaseCollider& sphereCollider, const NarrowPhaseCollider& planeCollider, ContactManifold& outManifold)
    {
        const WorldSphere sphere = ToWorldShape(std::get<SphereShape>(sphereCollider.collider->shape), *sphereCollider.transform);
        const WorldPlane plane = ToWorldShape(std::get<PlaneShape>(planeCollider.collider->shape), *planeCollider.transform);

        const Vec3 offset = sphere.centre - plane.centre;

        const float x = std::clamp(Dot(offset, plane.axisX), -plane.halfWidthX, plane.halfWidthX);
        const float z = std::clamp(Dot(offset, plane.axisZ), -plane.halfLengthZ, plane.halfLengthZ);
        const Vec3 closestPoint = plane.centre + plane.axisX * x + plane.axisZ * z;
        const Vec3 towardPlane = closestPoint - sphere.centre;
        const float distanceSquared = Dot(towardPlane, towardPlane);
        if (distanceSquared > sphere.radius * sphere.radius)
            return false;

        const float distance = std::sqrt(distanceSquared);
        ContactManifold manifold{};
        manifold.normal = distance > 0.000001f? towardPlane * (Dot(towardPlane, plane.normal) <= 0.0f ? 1.0f / distance : -1.0f / distance): -plane.normal;
        manifold.contacts.push_back(ContactPoint{sphere.radius - distance, closestPoint});

        manifold.bodyOne.rigidBody = sphereCollider.rigidbody;
        manifold.bodyOne.transform = sphereCollider.transform;
        manifold.bodyTwo.rigidBody = planeCollider.rigidbody;
        manifold.bodyTwo.transform = planeCollider.transform;

        outManifold = std::move(manifold);
        return true;
    }

    bool NarrowPhase::testSphereSphere(const NarrowPhaseCollider& sphereCollider, const NarrowPhaseCollider& sphereColliderTwo, ContactManifold& manifoldOut)
    {
        const WorldSphere sphereOne = ToWorldShape(std::get<SphereShape>(sphereCollider.collider->shape), *sphereCollider.transform);
        const WorldSphere sphereTwo = ToWorldShape(std::get<SphereShape>(sphereColliderTwo.collider->shape), *sphereColliderTwo.transform);

        Vec3 offset = sphereTwo.centre - sphereOne.centre;
        float radiusDistance = sphereOne.radius + sphereTwo.radius;
        float distanceSquared = offset.LengthSquared();

        if (distanceSquared <= radiusDistance * radiusDistance)
        {
            float penetration =  radiusDistance - offset.Length();
            Vec3 contactNormal{};
            if (distanceSquared <= 0.000001f)
            {

                contactNormal = { 1.0f, 0.0f, 0.0f };
            }
            else
            {
                contactNormal = offset.Normalized();
            }

            ContactManifold manifold{};
           
            Vec3 contactPoint = sphereOne.centre + sphereOne.radius * contactNormal;

            ContactPoint point{ .penetration = penetration , .position = contactPoint};


            manifold.contacts.push_back(point);
            manifold.normal = contactNormal;
            
            manifold.bodyOne.rigidBody = sphereCollider.rigidbody;
            manifold.bodyOne.transform = sphereCollider.transform;
            manifold.bodyTwo.rigidBody = sphereColliderTwo.rigidbody;
            manifold.bodyTwo.transform = sphereColliderTwo.transform;


            manifoldOut = manifold;
            return true;
        }

        return false;
    }

    bool NarrowPhase::testSphereBox(const NarrowPhaseCollider& boxCollider, const NarrowPhaseCollider& sphereCollider, ContactManifold& manifoldOut)
    {
        const WorldBox box = ToWorldShape(std::get<BoxShape>(boxCollider.collider->shape), *boxCollider.transform);
        const WorldSphere sphere = ToWorldShape(std::get<SphereShape>(sphereCollider.collider->shape), *sphereCollider.transform);

        const Vec3 relative = sphere.centre - box.centre;
        const Vec3 sphereLocalPosition{ Dot(box.axisX, relative), Dot(box.axisY, relative), Dot(box.axisZ, relative)};

        Vec3 closestPointLocal{
            std::clamp(sphereLocalPosition.x, -box.halfExtents.x, box.halfExtents.x),
            std::clamp(sphereLocalPosition.y, -box.halfExtents.y, box.halfExtents.y),
            std::clamp(sphereLocalPosition.z, -box.halfExtents.z, box.halfExtents.z)
        };

        const Vec3 boxToSphereLocal = sphereLocalPosition - closestPointLocal;
        const float distanceSquared = boxToSphereLocal.LengthSquared();
        const float radiusSquared = sphere.radius * sphere.radius;
        const float distance = std::sqrt(distanceSquared);
        if (distanceSquared > radiusSquared)
        {

            return false;
        }

        ContactManifold manifold{};
        ContactPoint contactPoint{};
        int faceAxis = -1;
        const Vec3 closestPointWorld = box.centre + box.axisX * closestPointLocal.x + box.axisY * closestPointLocal.y + box.axisZ * closestPointLocal.z;

        if (distanceSquared > 0.000001f)
        {
            const Vec3 boxToSphereWorld = box.axisX * boxToSphereLocal.x + box.axisY * boxToSphereLocal.y + box.axisZ * boxToSphereLocal.z;
            manifold.normal = -boxToSphereWorld / distance;
            contactPoint.penetration = sphere.radius - distance;
            contactPoint.position = closestPointWorld;
        }
        else
        {
      
            const Vec3 faceDistance{
                box.halfExtents.x - std::abs(sphereLocalPosition.x),
                box.halfExtents.y - std::abs(sphereLocalPosition.y),
                box.halfExtents.z - std::abs(sphereLocalPosition.z)
            };

            const RigidBodyComponent* sphereBody = sphereCollider.rigidbody;
            const RigidBodyComponent* boxBody = boxCollider.rigidbody;
            const Vec3 sphereVelocity = sphereBody ? sphereBody->velocity + Cross(sphereBody->angularVelocity, sphere.centre - sphereCollider.transform->position) : Vec3{};
            const Vec3 boxVelocity = boxBody ? boxBody->velocity + Cross(boxBody->angularVelocity, sphere.centre - boxCollider.transform->position) : Vec3{};
            const Vec3 relativeVelocity = sphereVelocity - boxVelocity;
            const Vec3 localRelativeVelocity{ Dot(box.axisX, relativeVelocity),Dot(box.axisY, relativeVelocity),Dot(box.axisZ, relativeVelocity)
            };

            faceAxis = 0;
            if (faceDistance.y < faceDistance.x && faceDistance.y <= faceDistance.z)
                faceAxis = 1;
            else if (faceDistance.z < faceDistance.x && faceDistance.z < faceDistance.y)
                faceAxis = 2;

            const float incomingSpeedX = std::abs(localRelativeVelocity.x);
            const float incomingSpeedY = std::abs(localRelativeVelocity.y);
            const float incomingSpeedZ = std::abs(localRelativeVelocity.z);
            if ((std::max)({incomingSpeedX, incomingSpeedY, incomingSpeedZ}) > 0.001f)
            {
                if (incomingSpeedX >= incomingSpeedY && incomingSpeedX >= incomingSpeedZ)
                    faceAxis = 0;
                else if (incomingSpeedY >= incomingSpeedZ)
                    faceAxis = 1;
                else
                    faceAxis = 2;
            }

            Vec3 outwardNormal{};
            if (faceAxis == 0)
            {
                const float sign = incomingSpeedX > 0.001f ? (localRelativeVelocity.x < 0.0f ? 1.0f : -1.0f) : (sphereLocalPosition.x >= 0.0f ? 1.0f : -1.0f);
                closestPointLocal.x = sign * box.halfExtents.x;
                outwardNormal = box.axisX * sign;
                contactPoint.penetration = sphere.radius + faceDistance.x;
            }
            else if (faceAxis == 1)
            {
                const float sign = incomingSpeedY > 0.001f ? (localRelativeVelocity.y < 0.0f ? 1.0f : -1.0f) : (sphereLocalPosition.y >= 0.0f ? 1.0f : -1.0f);
                closestPointLocal.y = sign * box.halfExtents.y;
                outwardNormal = box.axisY * sign;
                contactPoint.penetration = sphere.radius + faceDistance.y;
            }
            else
            {
                const float sign = incomingSpeedZ > 0.001f? (localRelativeVelocity.z < 0.0f ? 1.0f : -1.0f) : (sphereLocalPosition.z >= 0.0f ? 1.0f : -1.0f);
                closestPointLocal.z = sign * box.halfExtents.z;
                outwardNormal = box.axisZ * sign;
                contactPoint.penetration = sphere.radius + faceDistance.z;
            }

            manifold.normal = -outwardNormal;
            contactPoint.position = box.centre + box.axisX * closestPointLocal.x + box.axisY * closestPointLocal.y + box.axisZ * closestPointLocal.z;
        }

        manifold.contacts.push_back(contactPoint);
        manifold.bodyOne.rigidBody = sphereCollider.rigidbody;
        manifold.bodyOne.transform = sphereCollider.transform;
        manifold.bodyTwo.rigidBody = boxCollider.rigidbody;
        manifold.bodyTwo.transform = boxCollider.transform;

        const RigidBodyComponent* sphereBody = sphereCollider.rigidbody;
        const RigidBodyComponent* boxBody = boxCollider.rigidbody;
        const Vec3 sphereContactVelocity = sphereBody ? sphereBody->velocity + Cross(sphereBody->angularVelocity,contactPoint.position - sphereCollider.transform->position): Vec3{};
        const Vec3 boxContactVelocity = boxBody ? boxBody->velocity + Cross(boxBody->angularVelocity, contactPoint.position - boxCollider.transform->position): Vec3{};
        const float normalVelocity = Dot(boxContactVelocity - sphereContactVelocity, manifold.normal);

        

        manifoldOut = std::move(manifold);
        return true;

    }

    bool NarrowPhase::testCylinderSphere(const NarrowPhaseCollider& cylinderCollider,const NarrowPhaseCollider& sphereCollider, ContactManifold& manifoldOut)
    {
        const WorldCylinder cylinder = ToWorldShape(std::get<CylinderShape>(cylinderCollider.collider->shape), *cylinderCollider.transform);
        const WorldSphere sphere = ToWorldShape(std::get<SphereShape>(sphereCollider.collider->shape), *sphereCollider.transform);

        const Vec3 offset = sphere.centre - cylinder.centre;
        const Vec3 localSphere{Dot(offset, cylinder.axisX), Dot(offset, cylinder.axisY), Dot(offset, cylinder.axisZ)};
        const Vec3 radial{localSphere.x, 0.0f, localSphere.z};
        const float radialLength = radial.Length();
        const Vec3 radialDirection = radialLength > 0.0f ? radial * (1.0f / radialLength) : Vec3{1.0f, 0.0f, 0.0f};

        Vec3 closest = radial;
        closest.y = std::clamp(localSphere.y, -cylinder.halfHeight, cylinder.halfHeight);
        if (radialLength > cylinder.radius)
        {
            closest.x = radialDirection.x * cylinder.radius;
            closest.z = radialDirection.z * cylinder.radius;
        }

        Vec3 localNormal{};
        float penetration = 0.0f;
        const bool inside = radialLength <= cylinder.radius && std::abs(localSphere.y) <= cylinder.halfHeight;
        if (inside)
        {
            const float sideGap = cylinder.radius - radialLength;
            const float capGap = cylinder.halfHeight - std::abs(localSphere.y);
            if (sideGap < capGap)
            {
                localNormal = radialDirection;
                closest = radialDirection * cylinder.radius;
                closest.y = localSphere.y;
                penetration = sphere.radius + sideGap;
            }
            else
            {
                const float sign = localSphere.y >= 0.0f ? 1.0f : -1.0f;
                localNormal = {0.0f, sign, 0.0f};
                closest = {localSphere.x, sign * cylinder.halfHeight, localSphere.z};
                penetration = sphere.radius + capGap;
            }
        }
        else
        {
            const Vec3 difference = localSphere - closest;
            const float distanceSquared = difference.LengthSquared();
            if (distanceSquared > sphere.radius * sphere.radius)
                return false;

            const float distance = std::sqrt(distanceSquared);
            localNormal = difference * (1.0f / distance);
            penetration = sphere.radius - distance;
        }

        ContactManifold manifold{};
        
        manifold.normal = cylinder.axisX * localNormal.x + cylinder.axisY * localNormal.y + cylinder.axisZ * localNormal.z;
        const Vec3 contactPoint = cylinder.centre + cylinder.axisX * closest.x  + cylinder.axisY * closest.y + cylinder.axisZ * closest.z;
        manifold.contacts.push_back(ContactPoint{penetration, contactPoint});
        manifold.bodyOne = {cylinderCollider.transform, cylinderCollider.rigidbody};
        manifold.bodyTwo = {sphereCollider.transform, sphereCollider.rigidbody};
        manifoldOut = std::move(manifold);
        return true;
    }

    bool NarrowPhase::testCylinderPlane(const NarrowPhaseCollider& cylinderCollider,const NarrowPhaseCollider& planeCollider, ContactManifold& manifoldOut)
    {
        const WorldCylinder cylinder = ToWorldShape( std::get<CylinderShape>(cylinderCollider.collider->shape), *cylinderCollider.transform);
        const WorldPlane plane = ToWorldShape( std::get<PlaneShape>(planeCollider.collider->shape), *planeCollider.transform);

        const float distance = Dot(cylinder.centre - plane.centre, plane.normal);
        const Vec3 towardPlane = distance >= 0.0f ? -plane.normal : plane.normal;
        const float axial = Dot(cylinder.axisY, towardPlane);
        const Vec3 radial = towardPlane - cylinder.axisY * axial;
        const float radialLength = radial.Length();
        const float extent = cylinder.halfHeight * std::abs(axial) + cylinder.radius * radialLength;
        if (std::abs(distance) > extent)
            return false;

        Vec3 support = cylinder.centre;
        if (axial > 0.0f) support += cylinder.axisY * cylinder.halfHeight;
        else if (axial < 0.0f) support -= cylinder.axisY * cylinder.halfHeight;
        if (radialLength > 0.000001f)
            support += radial * (cylinder.radius / radialLength);

        Vec3 planePoint = support - plane.normal * Dot(support - plane.centre, plane.normal);
        const Vec3 localPoint = planePoint - plane.centre;
        const Vec3 cylinderOffset = planePoint - cylinder.centre;
        const float pointHeight = Dot(cylinderOffset, cylinder.axisY);
        const Vec3 pointRadial = cylinderOffset - cylinder.axisY * pointHeight;
      
        if (std::abs(Dot(localPoint, plane.axisX)) > plane.halfWidthX|| std::abs(Dot(localPoint, plane.axisZ)) > plane.halfLengthZ
                || std::abs(pointHeight) > cylinder.halfHeight + 0.000001f
                 || pointRadial.LengthSquared() > cylinder.radius * cylinder.radius + 0.000001f)
        {
          
            const Vec3 u = plane.axisX * plane.halfWidthX;
            const Vec3 v = plane.axisZ * plane.halfLengthZ;
            std::vector<Vec3> patch{ plane.centre - u - v, plane.centre + u - v, plane.centre + u + v, plane.centre - u + v};
            const float centreHeight = Dot(cylinder.centre, cylinder.axisY);
            patch = sutherlandHodgmanClip(patch,clipPlane{cylinder.axisY, centreHeight + cylinder.halfHeight});
            patch = sutherlandHodgmanClip(patch, clipPlane{-cylinder.axisY, -centreHeight + cylinder.halfHeight});
            if (patch.empty())
                return false;

            Vec3 contactSum{};
            int contactCount = 0;
            for (size_t i = 0; i < patch.size(); ++i)
            {
                const Vec3 startOffset = patch[i] - cylinder.centre;
                const Vec3 edge = patch[(i + 1) % patch.size()] - patch[i];
                const Vec3 radialStart = startOffset - cylinder.axisY * Dot(startOffset, cylinder.axisY);
                const Vec3 radialEdge = edge - cylinder.axisY * Dot(edge, cylinder.axisY);
                const float lengthSquared = radialEdge.LengthSquared();
                const float t = lengthSquared > 0.000000000001f? std::clamp(-Dot(radialStart, radialEdge) / lengthSquared, 0.0f, 1.0f) : 0.0f;
                const Vec3 radialPoint = radialStart + radialEdge * t;
                if (radialPoint.LengthSquared() <= cylinder.radius * cylinder.radius + 0.000001f)
                {
                    contactSum += patch[i] + edge * t;
                    ++contactCount;
                }
            }

            const float axisDotNormal = Dot(cylinder.axisY, plane.normal);
            if (std::abs(axisDotNormal) > 0.000001f)
            {
                const float alongAxis = -distance / axisDotNormal;
                const Vec3 axisPoint = cylinder.centre + cylinder.axisY * alongAxis;
                const Vec3 offset = axisPoint - plane.centre;
                if (std::abs(alongAxis) <= cylinder.halfHeight
                    && std::abs(Dot(offset, plane.axisX)) <= plane.halfWidthX
                    && std::abs(Dot(offset, plane.axisZ)) <= plane.halfLengthZ)
                {
                    contactSum += axisPoint;
                    ++contactCount;
                }
            }
            if (contactCount == 0)
                return false;

          
            planePoint = contactSum * (1.0f / static_cast<float>(contactCount));
        }

        ContactManifold manifold{};
        manifold.normal = towardPlane;
        manifold.contacts.push_back(ContactPoint{extent - std::abs(distance), planePoint});
        manifold.bodyOne = {cylinderCollider.transform, cylinderCollider.rigidbody};
        manifold.bodyTwo = {planeCollider.transform, planeCollider.rigidbody};
        manifoldOut = std::move(manifold);
        return true;
    }

    bool qmec::physics::NarrowPhase::testBoxBox(const NarrowPhaseCollider& boxColliderOne,const NarrowPhaseCollider& boxColliderTwo,ContactManifold& manifoldOut)
    {
        const WorldBox boxOne =ToWorldShape(std::get<BoxShape>(boxColliderOne.collider->shape),*boxColliderOne.transform);

        const WorldBox boxTwo =ToWorldShape(std::get<BoxShape>(boxColliderTwo.collider->shape),*boxColliderTwo.transform);

        Vec3 BestAxis{};
        float bestDistance = std::numeric_limits<float>::max();

        SATAxisType bestAxisType = SATAxisType::FaceA;

        Vec3 bestEdgeAxisA{};
        Vec3 bestEdgeAxisB{};

        auto testAxis =[&](Vec3 currentAxis,SATAxisType axisType,Vec3 edgeAxisA = {},Vec3 edgeAxisB = {}) -> bool
            {
                if (currentAxis.LengthSquared() <= 0.000001f)
                    return true;

                currentAxis = currentAxis.Normalized();

                float Ra = std::abs(Dot(boxOne.axisX, currentAxis)) * boxOne.halfExtents.x + std::abs(Dot(boxOne.axisY, currentAxis)) * boxOne.halfExtents.y + std::abs(Dot(boxOne.axisZ, currentAxis)) * boxOne.halfExtents.z;

                float Rb = std::abs(Dot(boxTwo.axisX, currentAxis)) * boxTwo.halfExtents.x + std::abs(Dot(boxTwo.axisY, currentAxis)) * boxTwo.halfExtents.y + std::abs(Dot(boxTwo.axisZ, currentAxis)) * boxTwo.halfExtents.z;

                float centerDistance = std::abs(Dot(boxOne.centre, currentAxis) -Dot(boxTwo.centre, currentAxis));

                if (centerDistance > Ra + Rb)
                    return false;

                float penetration = Ra + Rb - centerDistance;

                if (penetration < bestDistance)
                {
                    bestDistance = penetration;
                    BestAxis = currentAxis;
                    bestAxisType = axisType;

                    if (axisType == SATAxisType::EdgeEdge)
                    {
                        bestEdgeAxisA = edgeAxisA;
                        bestEdgeAxisB = edgeAxisB;
                    }
                }

                return true;
            };


        // =========================
        // Face axes - Box One
        // =========================

        if (!testAxis(boxOne.axisX, SATAxisType::FaceA))
            return false;

        if (!testAxis(boxOne.axisY, SATAxisType::FaceA))
            return false;

        if (!testAxis(boxOne.axisZ, SATAxisType::FaceA))
            return false;


        // =========================
        // Face axes - Box Two
        // =========================

        if (!testAxis(boxTwo.axisX, SATAxisType::FaceB))
            return false;

        if (!testAxis(boxTwo.axisY, SATAxisType::FaceB))
            return false;

        if (!testAxis(boxTwo.axisZ, SATAxisType::FaceB))
            return false;


        // =========================
        // Edge-Edge axes
        // =========================

        if (!testAxis(Cross(boxOne.axisX, boxTwo.axisX),SATAxisType::EdgeEdge, boxOne.axisX, boxTwo.axisX))
        {
            return false;
        }

        if (!testAxis(Cross(boxOne.axisX, boxTwo.axisY),SATAxisType::EdgeEdge, boxOne.axisX,boxTwo.axisY))
        {
            return false;
        }

        if (!testAxis(Cross(boxOne.axisX, boxTwo.axisZ),SATAxisType::EdgeEdge,boxOne.axisX,boxTwo.axisZ))
        {
            return false;
        }


        if (!testAxis(Cross(boxOne.axisY, boxTwo.axisX),SATAxisType::EdgeEdge, boxOne.axisY,boxTwo.axisX))
        {
            return false;
        }

        if (!testAxis(Cross(boxOne.axisY, boxTwo.axisY),SATAxisType::EdgeEdge,boxOne.axisY, boxTwo.axisY))
        {
            return false;
        }

        if (!testAxis(Cross(boxOne.axisY, boxTwo.axisZ),SATAxisType::EdgeEdge, boxOne.axisY,boxTwo.axisZ))
        {
            return false;
        }


        if (!testAxis(Cross(boxOne.axisZ, boxTwo.axisX),SATAxisType::EdgeEdge,boxOne.axisZ,boxTwo.axisX))
        {
            return false;
        }

        if (!testAxis(Cross(boxOne.axisZ, boxTwo.axisY),SATAxisType::EdgeEdge,boxOne.axisZ,boxTwo.axisY))
        {
            return false;
        }

        if (!testAxis(Cross(boxOne.axisZ, boxTwo.axisZ),SATAxisType::EdgeEdge,boxOne.axisZ,boxTwo.axisZ))
        {
            return false;
        }



        BestAxis = BestAxis.Normalized();

        Vec3 centerDelta = boxTwo.centre - boxOne.centre;

        if (Dot(centerDelta, BestAxis) < 0.0f)
            BestAxis = -BestAxis;


        // =========================================================
        // Edge-Edge
        // =========================================================

        if (bestAxisType == SATAxisType::EdgeEdge)
        {
            
            const Vec3 axesA[]{boxOne.axisX, boxOne.axisY, boxOne.axisZ};
            const Vec3 axesB[]{boxTwo.axisX, boxTwo.axisY, boxTwo.axisZ};
            const float extentsA[]{boxOne.halfExtents.x, boxOne.halfExtents.y, boxOne.halfExtents.z};
            const float extentsB[]{boxTwo.halfExtents.x, boxTwo.halfExtents.y, boxTwo.halfExtents.z};
            Vec3 edgeCentreA = boxOne.centre;
            Vec3 edgeCentreB = boxTwo.centre;
            float halfLengthA = 0.0f;
            float halfLengthB = 0.0f;

            for (int axis = 0; axis < 3; ++axis)
            {
                if (std::abs(Dot(axesA[axis], bestEdgeAxisA)) > 0.5f)
                {
                    halfLengthA = extentsA[axis];
                }
                else
                {
                    edgeCentreA += axesA[axis] * extentsA[axis] * (Dot(axesA[axis], BestAxis) >= 0.0f ? 1.0f : -1.0f);
                }

                if (std::abs(Dot(axesB[axis], bestEdgeAxisB)) > 0.5f)
                {
                    halfLengthB = extentsB[axis];
                }
                else
                {
                    edgeCentreB += axesB[axis] * extentsB[axis] * (Dot(axesB[axis], BestAxis) >= 0.0f ? -1.0f : 1.0f);
                }
            }

         
            const Vec3 offset = edgeCentreA - edgeCentreB;
            const float alignment = Dot(bestEdgeAxisA, bestEdgeAxisB);
            const float offsetA = Dot(bestEdgeAxisA, offset);
            const float offsetB = Dot(bestEdgeAxisB, offset);
            const float denominator = 1.0f - alignment * alignment;
            float s = 0.0f;
            if (denominator > 0.000001f)
                s = std::clamp((alignment * offsetB - offsetA) / denominator, -halfLengthA, halfLengthA);

            float t = alignment * s + offsetB;
            if (t < -halfLengthB || t > halfLengthB)
            {
                t = std::clamp(t, -halfLengthB, halfLengthB);
                s = std::clamp(alignment * t - offsetA, -halfLengthA, halfLengthA);
            }

            const Vec3 pointA = edgeCentreA + bestEdgeAxisA * s;
            const Vec3 pointB = edgeCentreB + bestEdgeAxisB * t;

            ContactManifold manifold{};
            manifold.normal = BestAxis;
            manifold.contacts.push_back(ContactPoint{bestDistance, (pointA + pointB) * 0.5f});
            manifold.bodyOne.rigidBody = boxColliderOne.rigidbody;
            manifold.bodyOne.transform = boxColliderOne.transform;
            manifold.bodyTwo.rigidBody = boxColliderTwo.rigidbody;
            manifold.bodyTwo.transform = boxColliderTwo.transform;
            manifoldOut = std::move(manifold);
            return true;
        }


        // =========================================================
        // Choose Reference / Incident Box
        // =========================================================

        const WorldBox* referenceBox = nullptr;
        const WorldBox* incidentBox = nullptr;

        if (bestAxisType == SATAxisType::FaceA)
        {
            referenceBox = &boxOne;
            incidentBox = &boxTwo;
        }
        else
        {
            referenceBox = &boxTwo;
            incidentBox = &boxOne;
        }


        // Reference Face


        Vec3 faceAxis{};
        float faceExtent{};

        Vec3 uaxis{};
        Vec3 vaxis{};

        float uExtent{};
        float vExtent{};

        float sign{};


      

        Vec3 referenceDirection = BestAxis;

        if (bestAxisType == SATAxisType::FaceB)
            referenceDirection = -BestAxis;


        float dotX = Dot(referenceBox->axisX, referenceDirection);
        float dotY = Dot(referenceBox->axisY, referenceDirection);
        float dotZ = Dot(referenceBox->axisZ, referenceDirection);

        float alignX = std::abs(dotX);
        float alignY = std::abs(dotY);
        float alignZ = std::abs(dotZ);

        // Reference Face - X
  

        float bestAlignment = alignX;

        faceAxis = referenceBox->axisX;
        faceExtent = referenceBox->halfExtents.x;

        uaxis = referenceBox->axisY;
        vaxis = referenceBox->axisZ;

        uExtent = referenceBox->halfExtents.y;
        vExtent = referenceBox->halfExtents.z;

        sign = dotX >= 0.0f ? 1.0f : -1.0f;


        // Reference Face - Y

        if (alignY > bestAlignment)
        {
            bestAlignment = alignY;

            faceAxis = referenceBox->axisY;
            faceExtent = referenceBox->halfExtents.y;

            uaxis = referenceBox->axisX;
            vaxis = referenceBox->axisZ;

            uExtent = referenceBox->halfExtents.x;
            vExtent = referenceBox->halfExtents.z;

            sign = dotY >= 0.0f ? 1.0f : -1.0f;
        }


        // Reference Face - Z

        if (alignZ > bestAlignment)
        {
            bestAlignment = alignZ;

            faceAxis = referenceBox->axisZ;
            faceExtent = referenceBox->halfExtents.z;

            uaxis = referenceBox->axisX;
            vaxis = referenceBox->axisY;

            uExtent = referenceBox->halfExtents.x;
            vExtent = referenceBox->halfExtents.y;

            sign = dotZ >= 0.0f ? 1.0f : -1.0f;
        }

        // Reference Face Vertices

        Vec3 referenceFaceNormal = faceAxis * sign;

        Vec3 faceCenter =referenceBox->centre +faceAxis * faceExtent * sign;

        std::vector<Vec3> faceVertexA{};
        faceVertexA.reserve(4);

        Vec3 vertexOne = faceCenter -uaxis * uExtent +vaxis * vExtent;

        Vec3 vertexTwo = faceCenter +uaxis * uExtent +vaxis * vExtent;

        Vec3 vertexThree = faceCenter + uaxis * uExtent -vaxis * vExtent;

        Vec3 vertexFour =faceCenter -uaxis * uExtent -vaxis * vExtent;

        faceVertexA.push_back(vertexOne);
        faceVertexA.push_back(vertexTwo);
        faceVertexA.push_back(vertexThree);
        faceVertexA.push_back(vertexFour);

        // Incident Face

        Vec3 incidentFaceAxis{};
        float incidentFaceExtent{};

        Vec3 incidentUAxis{};
        Vec3 incidentVAxis{};

        float incidentUExtent{};
        float incidentVExtent{};

        float incidentSign{};


        const Vec3 incidentDirection = bestAxisType == SATAxisType::FaceA? -BestAxis: BestAxis;
        const float incidentDotX = Dot(incidentBox->axisX, incidentDirection);
        const float incidentDotY = Dot(incidentBox->axisY, incidentDirection);
        const float incidentDotZ = Dot(incidentBox->axisZ, incidentDirection);

        float incidentBestAlignment = std::abs(incidentDotX);

        incidentFaceAxis = incidentBox->axisX;
        incidentFaceExtent = incidentBox->halfExtents.x;

        incidentUAxis = incidentBox->axisY;
        incidentVAxis = incidentBox->axisZ;

        incidentUExtent = incidentBox->halfExtents.y;
        incidentVExtent = incidentBox->halfExtents.z;

        incidentSign = incidentDotX >= 0.0f ? 1.0f : -1.0f;

        if (std::abs(incidentDotY) > incidentBestAlignment)
        {
            incidentBestAlignment = std::abs(incidentDotY);

            incidentFaceAxis = incidentBox->axisY;
            incidentFaceExtent = incidentBox->halfExtents.y;

            incidentUAxis = incidentBox->axisX;
            incidentVAxis = incidentBox->axisZ;

            incidentUExtent = incidentBox->halfExtents.x;
            incidentVExtent = incidentBox->halfExtents.z;

            incidentSign = incidentDotY >= 0.0f ? 1.0f : -1.0f;
        }


        if (std::abs(incidentDotZ) > incidentBestAlignment)
        {
            incidentBestAlignment = std::abs(incidentDotZ);

            incidentFaceAxis = incidentBox->axisZ;
            incidentFaceExtent = incidentBox->halfExtents.z;

            incidentUAxis = incidentBox->axisX;
            incidentVAxis = incidentBox->axisY;

            incidentUExtent = incidentBox->halfExtents.x;
            incidentVExtent = incidentBox->halfExtents.y;

            incidentSign = incidentDotZ >= 0.0f ? 1.0f : -1.0f;
        }

        // Incident Face Vertices

        Vec3 incidentFaceCenter =incidentBox->centre +incidentFaceAxis *incidentFaceExtent *incidentSign;

        std::vector<Vec3> faceVertexB{};
        faceVertexB.reserve(4);

        Vec3 incidentVertexOne =incidentFaceCenter -incidentUAxis * incidentUExtent +incidentVAxis * incidentVExtent;

        Vec3 incidentVertexTwo =incidentFaceCenter +incidentUAxis * incidentUExtent +incidentVAxis * incidentVExtent;

        Vec3 incidentVertexThree =incidentFaceCenter +incidentUAxis * incidentUExtent -incidentVAxis * incidentVExtent;

        Vec3 incidentVertexFour =incidentFaceCenter - incidentUAxis * incidentUExtent -incidentVAxis * incidentVExtent;

        faceVertexB.push_back(incidentVertexOne);
        faceVertexB.push_back(incidentVertexTwo);
        faceVertexB.push_back(incidentVertexThree);
        faceVertexB.push_back(incidentVertexFour);


        //Face Cliping

        float faceCentreU = Dot(faceCenter, uaxis);
        float faceCentreV = Dot(faceCenter, vaxis);

        clipPlane positiveU
        {
          uaxis,
         faceCentreU + uExtent
        };

        clipPlane negativeU
        {
            -uaxis,
            -faceCentreU + uExtent
        };

        clipPlane positiveV
        {
            vaxis,
            faceCentreV + vExtent
        };

        clipPlane negativeV{
            -vaxis,
            -faceCentreV + vExtent
        };
        
        std::vector<Vec3> clippedPolygon = faceVertexB;

        clippedPolygon = sutherlandHodgmanClip(clippedPolygon, positiveU);

        clippedPolygon = sutherlandHodgmanClip(clippedPolygon, negativeU);

        clippedPolygon = sutherlandHodgmanClip(clippedPolygon, positiveV);

        clippedPolygon = sutherlandHodgmanClip(clippedPolygon, negativeV);

        ContactManifold manifold{};
       

        for (const Vec3& point : clippedPolygon)
        {
            float pointDistance = Dot(point - faceCenter, referenceFaceNormal);

            if (pointDistance <= 0.0f)
            {
                float penetration = -pointDistance;

                Vec3 contactPoint = point - referenceFaceNormal * pointDistance;

                manifold.contacts.push_back(ContactPoint{ penetration, contactPoint });
            }
        }

        if (manifold.contacts.empty())
            return false;

         centerDelta = boxTwo.centre - boxOne.centre;

        if (Dot(centerDelta, BestAxis) < 0.0f)
        {
            BestAxis = -BestAxis;
        }

        manifold.normal = BestAxis;


        manifold.bodyOne.rigidBody = boxColliderOne.rigidbody;
        manifold.bodyOne.transform = boxColliderOne.transform;
        manifold.bodyTwo.rigidBody = boxColliderTwo.rigidbody;
        manifold.bodyTwo.transform = boxColliderTwo.transform;

        manifoldOut = manifold;

        return true;
    }
    
    bool NarrowPhase::testBoxCylinder(
        const NarrowPhaseCollider& boxCollider,
        const NarrowPhaseCollider& cylinderCollider,
        ContactManifold& manifoldOut)
    {
        return buildConvexManifold(boxCollider, cylinderCollider, manifoldOut);
    }

    bool NarrowPhase::testCylinderCylinder(
        const NarrowPhaseCollider& cylinderColliderA,
        const NarrowPhaseCollider& cylinderColliderB,
        ContactManifold& manifoldOut)
    {
        return buildConvexManifold(cylinderColliderA, cylinderColliderB, manifoldOut);
    }

}
