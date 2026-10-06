#pragma once
#include <vector>
#include "QMEC/Math/Vec3.h"
#include "QMEC/Scene/Components/TransformComponent.h"
#include "QMEC/Scene/Components/RigidBodyComponent.h"

namespace qmec::physics {

	struct PhysicsBody
	{
		TransformComponent* transform{nullptr};
		RigidBodyComponent* rigidBody{nullptr};
	};


	struct ContactPoint
	{
		float penetration;
		Vec3 position;
		float accumulatedNormalImpulse{0.0f};
		Vec3 accumulatedFrictionImpulse{};
		Vec3 accumulatedRollingImpulse{};
	};


	struct ContactManifold
	{
		PhysicsBody bodyOne{};
		PhysicsBody bodyTwo{};
		std::vector<ContactPoint> contacts;
		Vec3 normal;
	};

}

namespace qmec
{
    using physics::PhysicsBody;
    using physics::ContactPoint;
    using physics::ContactManifold;
}
