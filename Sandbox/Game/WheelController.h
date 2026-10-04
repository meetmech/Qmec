#pragma once

#include "QMEC/ECS/Entity.h"
#include "QMEC/Math/Vec3.h"

namespace qmec
{
    class Scene;
    struct RigidBodyComponent;

    struct GroundMaterial
    {
        float staticFriction{ 0.9f };
        float dynamicFriction{ 0.7f };
    };

   
    class WheelController final
    {
    public:
        Entity entity{};
        Vec3 worldPosition{};
        Vec3 axleAxis{ 0.0f, 1.0f, 0.0f };
        float radius{};
        float halfWidth{};
        bool geometryValid{ false };

        Vec3 groundContactPoint{};
        Vec3 groundContactNormal{ 0.0f, 1.0f, 0.0f };
        float suspensionCompression{};
        float suspensionRestLength{ 0.35f };
        float maximumSuspensionTravel{ 0.35f };
        float springStiffness{ 18000.0f };
        float damperCoefficient{ 1800.0f };

        float grip{ 1.0f };
        float angularInertia{ 1.0f };

        float angularVelocity{ 0.0f };
        float motorTorque{ 0.0f };
        float brakeTorque{ 0.0f };

        float normalForce{ 0.0f };
        bool grounded{ false };

        float surfaceSpeed{ 0.0f };
        float slipVelocity{ 0.0f };
        float maxStaticFriction{ 0.0f };
        float maxDynamicFriction{ 0.0f };

        bool slipping{ false };

        [[nodiscard]] bool RefreshColliderState(const Scene& scene, Entity wheelEntity) noexcept;
        [[nodiscard]] bool UpdateGroundContact(Scene& scene,Entity wheelEntity,Entity chassisEntity,RigidBodyComponent& chassisBody) noexcept;
        void Update(float deltaTime, float vehicleSpeed, const GroundMaterial& ground);

    private:
        void CalculateSlip(float vehicleSpeed);
        void CalculateFrictionLimits(const GroundMaterial& ground);
    };
}
