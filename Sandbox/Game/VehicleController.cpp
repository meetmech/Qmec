#include "Game/VehicleController.h"

#include "QMEC/Scene/Components/RigidBodyComponent.h"
#include "QMEC/Scene/Scene.h"

#include <cstdint>
#include <iostream>

namespace qmec::game
{
    void VehicleController::OnUpdate(const ScriptContext& context)
    {
        auto& registry = context.scene.GetRegistry();
        auto* chassisBody = registry.GetComponent<RigidBodyComponent>(context.entity);
        if (chassisBody == nullptr || chassisBody->isKinematic || chassisBody->inverseMass <= 0.0f)
        {
            return;
        }

        ReadInput();
        ApplyThrottle(*chassisBody, context.deltaTime);
        ApplySteering(*chassisBody, context.deltaTime);
        ApplyBraking(*chassisBody, context.deltaTime);

        const auto updateWheel = [&](WheelController& wheel, Entity wheelEntity)
        {
            wheel.suspensionRestLength = suspensionRestLength;
            wheel.maximumSuspensionTravel = maximumSuspensionTravel;
            wheel.springStiffness = springStiffness;
            wheel.dampingRatio = dampingRatio;
            (void)wheel.UpdateGroundContact(context.scene, wheelEntity, context.entity, *chassisBody);
        };

        updateWheel(frontLeftState, frontLeftWheel);
        updateWheel(frontRightState, frontRightWheel);
        updateWheel(rearLeftState, rearLeftWheel);
        updateWheel(rearRightState, rearRightWheel);

#ifndef NDEBUG
        static std::uint32_t diagnosticUpdateCount = 0U;
        if (++diagnosticUpdateCount >= 60U)
        {
            diagnosticUpdateCount = 0U;
            const float totalNormalForce = frontLeftState.normalForce + frontRightState.normalForce +
                rearLeftState.normalForce + rearRightState.normalForce;
            constexpr float gravityMagnitude = 9.81f;

            std::clog << "[Vehicle suspension] mass=" << chassisBody->mass
                << " weight=" << chassisBody->mass * gravityMagnitude
                << " support=" << totalNormalForce
                << " | FL " << frontLeftState.grounded << ":" << frontLeftState.suspensionCompression
                << ":" << frontLeftState.normalForce
                << " FR " << frontRightState.grounded << ":" << frontRightState.suspensionCompression
                << ":" << frontRightState.normalForce
                << " RL " << rearLeftState.grounded << ":" << rearLeftState.suspensionCompression
                << ":" << rearLeftState.normalForce
                << " RR " << rearRightState.grounded << ":" << rearRightState.suspensionCompression
                << ":" << rearRightState.normalForce << '\n';
        }
#endif
    }

    void VehicleController::ReadInput() noexcept
    {
        // TODO: Read throttle, steering, and brake keys into the input values.
    }

    void VehicleController::ApplyThrottle(
        qmec::scene::components::RigidBodyComponent&, float) noexcept
    {
        // TODO: Apply throttleInput_ to the driven wheels / chassis.
    }

    void VehicleController::ApplySteering(
        qmec::scene::components::RigidBodyComponent&, float) noexcept
    {
        // TODO: Apply steeringInput_ to the front wheels.
    }

    void VehicleController::ApplyBraking(
        qmec::scene::components::RigidBodyComponent&, float) noexcept
    {
        // TODO: Apply brakeInput_ to the wheels.
    }
}
