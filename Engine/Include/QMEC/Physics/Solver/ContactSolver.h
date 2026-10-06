#pragma once
#include "QMEC/ECS/Registry.h"
#include "QMEC/Physics/ContactManifold.h"

namespace qmec::physics
{

	class ContactSolver
	{
	public:
		ContactSolver(float iterations = 8) :solverIterations(iterations)
		{

		}

		void Solve(ContactManifold manifold);


	private:
		float solverIterations{};
	};

	
}

namespace qmec
{
    using physics::ContactSolver;
}
