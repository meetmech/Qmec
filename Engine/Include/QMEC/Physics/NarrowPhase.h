#pragma once
#include "QMEC/Scene/Components/ColliderComponent.h"
#include "QMEC/Scene/Components/TransformComponent.h"
#include "QMEC/Scene/Components/RigidBodyComponent.h"
#include "QMEC/Physics/ContactManifold.h"
#include "QMEC/ECS/Entity.h"
#include <span>
#include <vector>

namespace qmec
{

	struct clipPlane
	{
		Vec3 normal;
		float d;
	};

	struct NarrowPhaseCollider
	{
		Entity entity{};
		const ColliderComponent* collider{ nullptr };
		TransformComponent* transform{ nullptr };
		RigidBodyComponent* rigidbody{ nullptr };
	};

	struct NarrowPhaseColliderPairs
	{
		NarrowPhaseCollider first;
		NarrowPhaseCollider second;
	};

	enum class SATAxisType
	{
		FaceA,
		FaceB,
		EdgeEdge

	};
	

	class NarrowPhase
	{
	public :
      
	    bool findNarrowPhasePairs(std::vector<NarrowPhaseColliderPairs> broadPhaseCollisions,std::vector<ContactManifold>& manifoldsOut);
	private:
		
		bool testBoxPlane(const NarrowPhaseCollider& a,const NarrowPhaseCollider& b,ContactManifold& outManifold);

		bool testSpherePlane(const NarrowPhaseCollider& a, const NarrowPhaseCollider& b, ContactManifold& outManifold);

		bool testSphereSphere(const NarrowPhaseCollider& a,const NarrowPhaseCollider& b, ContactManifold& manifoldOut);
		bool testSphereBox(const NarrowPhaseCollider& a, const NarrowPhaseCollider& b, ContactManifold& manifoldOut);
		bool testCylinderSphere(const NarrowPhaseCollider& a, const NarrowPhaseCollider& b, ContactManifold& manifoldOut);
		bool testCylinderPlane(const NarrowPhaseCollider& a, const NarrowPhaseCollider& b, ContactManifold& manifoldOut);
		bool testBoxBox(const NarrowPhaseCollider& a, const NarrowPhaseCollider& b, ContactManifold& manifoldOut);
		bool testBoxCylinder(const NarrowPhaseCollider& a, const NarrowPhaseCollider& b, ContactManifold& manifoldOut);
		bool testCylinderCylinder(const NarrowPhaseCollider& a, const NarrowPhaseCollider& b, ContactManifold& manifoldOut);
		

	};
}
