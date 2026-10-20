#pragma once

#include "QMEC/Scripting/Script.h"
#include "Game/WheelController.h"

namespace qmec::game
{
    class VehicleController final : public Script
    {
    public:
     
        Entity frontLeftWheel{};
        Entity frontRightWheel{};
        Entity rearLeftWheel{};
        Entity rearRightWheel{};

        float suspensionRestLength{ 0.35f };
        float maximumSuspensionTravel{ 0.35f };
        float springStiffness{ 18000.0f };
        float dampingRatio{ 0.9f };

        void OnUpdate(const ScriptContext& context) override;

    private:
        void ReadInput() noexcept;
        void ApplyThrottle(qmec::scene::components::RigidBodyComponent& chassisBody,
            float deltaTime) noexcept;
        void ApplySteering(qmec::scene::components::RigidBodyComponent& chassisBody,
            float deltaTime) noexcept;
        void ApplyBraking(qmec::scene::components::RigidBodyComponent& chassisBody,
            float deltaTime) noexcept;

        float throttleInput_{};
        float steeringInput_{};
        float brakeInput_{};

        WheelController frontLeftState{};
        WheelController frontRightState{};
        WheelController rearLeftState{};
        WheelController rearRightState{};
    };
}

namespace qmec
{
    using game::VehicleController;
}
