#pragma once
#include "QMEC/Math/Vec3.h"

namespace qmec::physics
{
	struct AABB
	{
		Vec3 min{};
		Vec3 max{};
	};
}
namespace qmec
{
    using physics::AABB;
}
