#include "QMEC/Physics/BroadPhase.h"
#include "QMEC/ECS/Registry.h"

namespace qmec::physics
{
	std::vector<CollisionPair> BroadPhase::FindPotentialPairs(std::span<const BroadPhaseCollider> colliders)
	{
		std::vector<CollisionPair> pairs;
		for (size_t i = 0; i < colliders.size(); ++i)
		{
			for (size_t j = i + 1; j < colliders.size(); ++j)
			{
				if (colliders[i].bounds.max.x >= colliders[j].bounds.min.x &&
					colliders[j].bounds.max.x >= colliders[i].bounds.min.x &&
					colliders[i].bounds.max.y >= colliders[j].bounds.min.y &&
					colliders[j].bounds.max.y >= colliders[i].bounds.min.y &&
					colliders[i].bounds.max.z >= colliders[j].bounds.min.z &&
					colliders[j].bounds.max.z >= colliders[i].bounds.min.z)
				{
					pairs.push_back({colliders[i].entity,colliders[j].entity});
				}
			}
		}
		return pairs;
	}
}
