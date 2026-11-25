#pragma once

#include "QMEC/ECS/Entity.h"
#include "QMEC/Math/Quat.h"
#include "QMEC/Math/Vec3.h"
#include "QMEC/Scripting/Script.h"

namespace qmec::scene { class Scene; }
namespace qmec::scene::components
{
    struct RigidBodyComponent;
    struct TransformComponent;
}

namespace qmec::ecs
{
    class Registry;
}

namespace qmec::game
{
   

    struct TireGripSettings
    {
        float staticFrictionCoefficient{ 1.0f };
        float dynamicFrictionCoefficient{ 0.8f };
    };

    class WheelController final : public Script
    {
    public:
        Entity wheelEntity{};
        Entity chassisEntity{};
        qmec::scene::Scene* scene{ nullptr };
        qmec::ecs::Registry* registry{ nullptr };
        qmec::scene::components::RigidBodyComponent* chasisRigidbody{ nullptr };
        Vec3 worldPosition{};
        Vec3 axleAxis{ 0.0f, 1.0f, 0.0f };
        Quat initialRotation{0,0,0,1};
        Vec3 steeringAxis{ 1.0f, 0.0f, 0.0f };
        float radius{};
        float halfWidth{};
        bool geometryValid{ false };

        Vec3 groundContactPoint{};
        Vec3 groundContactNormal{ 0.0f, 1.0f, 0.0f };
        float suspensionCompression{};
        float suspensionRestLength{ 0.35f };
        float maximumSuspensionTravel{ 0.35f };
        float springStiffness{ 18000.0f };
        float dampingRatio{ 0.9f };

        float grip{ 1.0f };
        float angularInertia{ 1.0f };
        TireGripSettings tireGripSettings{};

        float angularVelocity{ 0.0f };
        float motorTorque{ 0.0f };
        float brakeTorque{ 0.0f };

        float normalForce{ 0.0f };
        bool grounded{ false };

        float surfaceSpeed{ 0.0f };
        float slipVelocity{ 0.0f };
        float lateralSlipVelocity{ 0.0f };
        float maxStaticFriction{ 0.0f };
        float maxDynamicFriction{ 0.0f };
        float steerYaw{ 0.0f };
        float currentSteeringAngle{ 0.0f };

        bool slipping{ false };

        [[nodiscard]] bool RefreshColliderState() noexcept;
        [[nodiscard]] bool UpdateGroundContact() noexcept;

        void Initialize(const ScriptContext& context, Entity wheelEntity, Entity chasisentity);

        void UpdateInertiaRotation(float deltaTime, float vehicleSpeed);
        void ApplySteering(float targetAngle,float deltaTime,float steeringStrength);
        void ApplyDriveTorque(float torque,float deltaTime,const Vec3& chassisWorldPosition,qmec::scene::components::RigidBodyComponent& chassisBody) noexcept;
        void ApplyTireForces(float deltaTime);
        void OnUpdate(const ScriptContext& context) override;


    private:
        Entity anchorWheelEntity_{};
        Entity anchorChassisEntity_{};

        Vec3 suspensionAnchorLocal_{};



        bool suspensionAnchorInitialized_{ false };

        qmec::scene::components::TransformComponent* wheelTransform_{ nullptr };

        float safeRadius{};
        float wheelSpinAngle_{ 0.0f };


       // void CalculateSlip(float vehicleSpeed);
       // void CalculateFrictionLimits();
    };
}

namespace qmec
{
    using game::TireGripSettings;
    using game::WheelController;
}