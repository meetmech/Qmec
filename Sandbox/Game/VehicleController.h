#pragma once

#include "QMEC/Scripting/Script.h"
#include "Game/WheelController.h"
#include <array>

namespace qmec::game
{
    class VehicleController final : public Script
    {
    public:
        struct TorquePoint
        {
            float rpm;
            float torque;
        };

        struct EngineComponent
        {

            float engineHorsePower{ 200 };
            float rpm = 0;
            float idleRPM = 800.0f;
            float maxRPM = 6500.0f;
            float maxTorque = 300.0f;
            float throttle = 0.0f;
            bool clutchEngaged{true};
            bool AWD{ false };
            float rotationalInertia{ 0.25f };
            float frictionTorque{ 20.0f };
            std::array<TorquePoint, 7> torqueCurve
            {
                { {  800.0f, 120.0f },
                { 1500.0f, 200.0f },
                { 2500.0f, 280.0f },
                { 3500.0f, 320.0f },
                { 4500.0f, 310.0f },
                { 5500.0f, 270.0f },
                { 6500.0f, 200.0f }}
              };

        };

        struct Drivetrain
        {
            float gearRatios[5]{1.0f, 0.8f, 0.65f, 0.5f, 0.4f};
            int currentGearRatio{0};
            float finalDriveRatio{ 2.0f };
            float efficiency{ 0.9f };
        };

        Entity frontLeftWheel{};
        Entity frontRightWheel{};
        Entity rearLeftWheel{};
        Entity rearRightWheel{};

        float suspensionRestLength{ 0.35f };
        float maximumSuspensionTravel{ 0.35f };
        float springStiffness{ 18000.0f };
        float dampingRatio{ 0.9f };
        float brakeTorquePerWheel{8000.0f };
        float directionChangeSpeed{ 0.5f };
        float steeringStrength{ 5.0f };
        EngineComponent engine;
        Drivetrain drivetrain;
        
        qmec::ecs::Registry* registry{ nullptr };
        void OnCreate(const ScriptContext& context) override;
        void OnUpdate(const ScriptContext& context) override;

    private:
        void ReadInput() noexcept;
        float SampleTorqueCurve(const EngineComponent& engine, float rpm);
        void ApplyThrottle(qmec::scene::components::RigidBodyComponent& chassisBody,float deltaTime) noexcept;
        void ApplySteering(qmec::scene::components::RigidBodyComponent& chassisBody,float deltaTime) noexcept;
        void ApplyBraking(qmec::scene::components::RigidBodyComponent& chassisBody,float deltaTime) noexcept;

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
