#pragma once
#include "QMEC/Math/Vec3.h"

namespace qmec
{
	struct AABB
	{
		Vec3 min{};
		Vec3 max{};
	};
}