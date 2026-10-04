#include "QMEC/Physics/Solver/ContactSolver.h"
#include "QMEC/Math/Vec3.h"
#include <algorithm>
#include <iostream>
#include <vector>

namespace qmec
{
    void ContactSolver::Solve(ContactManifold manifold)
    {
        auto* transformOne = manifold.bodyOne.transform;
        auto* transformTwo = manifold.bodyTwo.transform;
        auto* rigidbodyOne = manifold.bodyOne.rigidBody;
        auto* rigidbodyTwo = manifold.bodyTwo.rigidBody;
        if (!transformOne || !transformTwo)
            return;

        const float inverseMassOne = rigidbodyOne && !rigidbodyOne->isKinematic ? rigidbodyOne->inverseMass : 0.0f;
        const float inverseMassTwo = rigidbodyTwo && !rigidbodyTwo->isKinematic ? rigidbodyTwo->inverseMass : 0.0f;
        const float totalInvMass = inverseMassOne + inverseMassTwo;
        const PhysicsMaterial defaultMaterial{};
        const PhysicsMaterial& materialOne = rigidbodyOne ? rigidbodyOne->material : defaultMaterial;
        const PhysicsMaterial& materialTwo = rigidbodyTwo ? rigidbodyTwo->material : defaultMaterial;
        const float staticFriction = (std::max)(0.0f, (materialOne.staticFriction + materialTwo.staticFriction) * 0.5f);
        const float dynamicFriction = (std::max)(0.0f, (materialOne.dynamicFriction + materialTwo.dynamicFriction) * 0.5f);
        const float rollingResistance = std::clamp((materialOne.rollingResistance + materialTwo.rollingResistance) * 0.5f, 0.0f, 1.0f);
        const float elasticityOne = rigidbodyOne ? std::clamp(rigidbodyOne->material.Elasticity, 0.0f, 1.0f) : 0.0f;
        const float elasticityTwo = rigidbodyTwo ? std::clamp(rigidbodyTwo->material.Elasticity, 0.0f, 1.0f) : 0.0f;
        const float elasticity = (elasticityOne + elasticityTwo) * 0.5f;

        if (totalInvMass <= 0.0f)
            return;

        const Vec3 centreOne = transformOne->position;
        const Vec3 centreTwo = transformTwo->position;
        float maxPenetration = 0.0f;
        for (const ContactPoint& contact : manifold.contacts)
        {
            maxPenetration = std::max(maxPenetration, contact.penetration);
        }
        transformOne->position -= manifold.normal * (maxPenetration * inverseMassOne / totalInvMass);
        transformTwo->position += manifold.normal * (maxPenetration * inverseMassTwo / totalInvMass);

        std::vector<float> restitutionTargets(manifold.contacts.size(), 0.0f);
        for (size_t i = 0; i < manifold.contacts.size(); ++i)
        {
            const ContactPoint& contact = manifold.contacts[i];
            const Vec3 rOne = contact.position - centreOne;
            const Vec3 rTwo = contact.position - centreTwo;
            const Vec3 velocityOne = rigidbodyOne ? rigidbodyOne->velocity + Cross(rigidbodyOne->angularVelocity, rOne) : Vec3{};
            const Vec3 velocityTwo = rigidbodyTwo ? rigidbodyTwo->velocity + Cross(rigidbodyTwo->angularVelocity, rTwo) : Vec3{};
            const float initialNormalVelocity = Dot(velocityTwo - velocityOne, manifold.normal);
            if (initialNormalVelocity < 0.0f)
            {
                restitutionTargets[i] = -elasticity * initialNormalVelocity;
            }
        }

        for (size_t iteration = 0; iteration < solverIterations; ++iteration)
        {
            for (size_t contactIndex = 0; contactIndex < manifold.contacts.size(); ++contactIndex)
            {
                ContactPoint& contact = manifold.contacts[contactIndex];
                const Vec3 rOne = contact.position - centreOne;
                const Vec3 rTwo = contact.position - centreTwo;

                const Vec3 velocityOne = rigidbodyOne ? rigidbodyOne->velocity + Cross(rigidbodyOne->angularVelocity, rOne) : Vec3{};
                const Vec3 velocityTwo = rigidbodyTwo ? rigidbodyTwo->velocity + Cross(rigidbodyTwo->angularVelocity, rTwo) : Vec3{};

                Vec3 relativeVelocity = velocityTwo - velocityOne;
                const float normalVelocity = Dot(relativeVelocity, manifold.normal);

                Vec3 velocityAlongNormal = manifold.normal * normalVelocity;
                Vec3 tangentialVelocity = relativeVelocity - velocityAlongNormal;
                const Vec3 frictionDirection = -tangentialVelocity.Normalized();
                const float tVelocity = Dot(tangentialVelocity, frictionDirection);

                const Vec3 rCrossNormalOne = Cross(rOne, manifold.normal);
                const Vec3 rCrossNormalTwo = Cross(rTwo, manifold.normal);
                const float angularTermOne = inverseMassOne > 0.0f ? Dot(rCrossNormalOne, rigidbodyOne->inverseInertiaWorld.TransformVector(rCrossNormalOne)) : 0.0f;
                const float angularTermTwo = inverseMassTwo > 0.0f ? Dot(rCrossNormalTwo, rigidbodyTwo->inverseInertiaWorld.TransformVector(rCrossNormalTwo)) : 0.0f;
                const Vec3 rCrossFrictionOne = Cross(rOne, frictionDirection);
                const Vec3 rCrossFrictionTwo = Cross(rTwo, frictionDirection);

                const float angularFrictionOne = inverseMassOne > 0.0f ? Dot(rCrossFrictionOne, rigidbodyOne->inverseInertiaWorld.TransformVector(rCrossFrictionOne)) : 0.0f;
                const float angularFrictionTwo = inverseMassTwo > 0.0f ? Dot(rCrossFrictionTwo, rigidbodyTwo->inverseInertiaWorld.TransformVector(rCrossFrictionTwo)) : 0.0f;

                const float normalDenominator = totalInvMass + angularTermOne + angularTermTwo;
                const float normalImpulseDelta = (restitutionTargets[contactIndex] - normalVelocity) / normalDenominator;



                const float oldNormalImpulse = contact.accumulatedNormalImpulse;
                contact.accumulatedNormalImpulse = (std::max)(0.0f, oldNormalImpulse + normalImpulseDelta);
                const float appliedNormalImpulse = contact.accumulatedNormalImpulse - oldNormalImpulse;

                const float frictionDenominator = totalInvMass + angularFrictionOne + angularFrictionTwo;
                const float frictionImpulseDelta = -tVelocity / frictionDenominator;
                const Vec3 oldFrictionImpulse = contact.accumulatedFrictionImpulse;
                Vec3 candidateFrictionImpulse = oldFrictionImpulse + frictionDirection * frictionImpulseDelta;
                const float staticFrictionLimit = staticFriction * contact.accumulatedNormalImpulse;
                const float candidateFrictionMagnitude = candidateFrictionImpulse.Length();
                if (candidateFrictionMagnitude > staticFrictionLimit)
                {
                    const float dynamicFrictionLimit = dynamicFriction * contact.accumulatedNormalImpulse;
                    if (candidateFrictionMagnitude > 0.0f)
                    {
                        candidateFrictionImpulse = candidateFrictionImpulse.Normalized() * dynamicFrictionLimit;
                    }
                }
                contact.accumulatedFrictionImpulse = candidateFrictionImpulse;
                const Vec3 appliedFrictionImpulse = contact.accumulatedFrictionImpulse - oldFrictionImpulse;

                const Vec3 impulseVector = manifold.normal * appliedNormalImpulse;

                if (inverseMassOne > 0.0f)
                {
                    rigidbodyOne->ApplyImpulse(appliedFrictionImpulse * -1.0f);
                    rigidbodyOne->ApplyAngularImpulse(Cross(rOne, appliedFrictionImpulse * -1.0f));
                    rigidbodyOne->ApplyImpulse(impulseVector * -1.0f);
                    rigidbodyOne->ApplyAngularImpulse(Cross(rOne, impulseVector * -1.0f));
                }
                if (inverseMassTwo > 0.0f)
                {
                    rigidbodyTwo->ApplyImpulse(appliedFrictionImpulse);
                    rigidbodyTwo->ApplyAngularImpulse(Cross(rTwo, appliedFrictionImpulse));
                    rigidbodyTwo->ApplyImpulse(impulseVector);
                    rigidbodyTwo->ApplyAngularImpulse(Cross(rTwo, impulseVector));
                }


                const Vec3 angularVelocityOne = rigidbodyOne ? rigidbodyOne->angularVelocity : Vec3{};
                const Vec3 angularVelocityTwo = rigidbodyTwo ? rigidbodyTwo->angularVelocity : Vec3{};
                const Vec3 relativeAngularVelocity = angularVelocityTwo - angularVelocityOne;
                const Vec3 rollingVelocity = relativeAngularVelocity - manifold.normal * Dot(relativeAngularVelocity, manifold.normal);
                const float rollingSpeed = rollingVelocity.Length();
                if (rollingResistance > 0.0f && rollingSpeed > 0.0f)
                {
                    const Vec3 rollingAxis = rollingVelocity / rollingSpeed;
                    const Vec3 inverseInertiaAxisOne = inverseMassOne > 0.0f ? rigidbodyOne->inverseInertiaWorld.TransformVector(rollingAxis) : Vec3{};
                    const Vec3 inverseInertiaAxisTwo = inverseMassTwo > 0.0f ? rigidbodyTwo->inverseInertiaWorld.TransformVector(rollingAxis) : Vec3{};
                    const float angularDenominator = Dot(rollingAxis, inverseInertiaAxisOne + inverseInertiaAxisTwo);

                    if (angularDenominator > 0.0f)
                    {
                        const Vec3 oldRollingImpulse = contact.accumulatedRollingImpulse;
                        const Vec3 rollingImpulseDelta = rollingAxis * (-rollingSpeed / angularDenominator);
                        Vec3 candidateRollingImpulse = oldRollingImpulse + rollingImpulseDelta;
                        float contactRadius = inverseMassOne > 0.0f ? rOne.Length() : rTwo.Length();
                        if (inverseMassOne > 0.0f && inverseMassTwo > 0.0f)
                        {
                            contactRadius = (rOne.Length() + rTwo.Length()) * 0.5f;
                        }

                        const float rollingImpulseLimit = rollingResistance * contact.accumulatedNormalImpulse * contactRadius;
                        const float candidateMagnitude = candidateRollingImpulse.Length();
                        if (candidateMagnitude > rollingImpulseLimit && candidateMagnitude > 0.0f)
                        {
                            candidateRollingImpulse = candidateRollingImpulse.Normalized() * rollingImpulseLimit;
                        }

                        contact.accumulatedRollingImpulse = candidateRollingImpulse;
                        const Vec3 appliedRollingImpulse = contact.accumulatedRollingImpulse - oldRollingImpulse;

                        if (inverseMassOne > 0.0f)
                            rigidbodyOne->ApplyAngularImpulse(appliedRollingImpulse * -1.0f);
                        if (inverseMassTwo > 0.0f)
                            rigidbodyTwo->ApplyAngularImpulse(appliedRollingImpulse);
                    }
                }
            }
        }
    }
}
