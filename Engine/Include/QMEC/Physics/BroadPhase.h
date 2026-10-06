#pragma once
#include "QMEC/Physics/AABB.h"
#include "QMEC/ECS/Entity.h"
#include "QMEC/Physics/CollisionPair.h"
#include <span>
#include <vector>

namespace qmec::physics
{
	struct BroadPhaseCollider
	{
		Entity entity;
		AABB bounds;
	};


	class BroadPhase
	{
	public:
		std::vector<CollisionPair> FindPotentialPairs(std::span<const BroadPhaseCollider> colliders);
	};
}

namespace qmec
{
    using physics::BroadPhaseCollider;
    using physics::BroadPhase;
}
