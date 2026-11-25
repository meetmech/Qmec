#include "Game/VehicleController.h"

#include "QMEC/Scene/Components/RigidBodyComponent.h"
#include "QMEC/Scene/Components/TransformComponent.h"
#include "QMEC/Input/InputSystem.h"
#include "QMEC/Scene/Scene.h"
#include <algorithm>
#include <numbers>
#include <cstdint>
#include <iostream>
#include <cmath>

namespace qmec::game
{

    void VehicleController::OnCreate(const ScriptContext& context)
    {
         registry = &context.scene.GetRegistry();

        frontLeftState.Initialize(context, frontLeftWheel,context.entity);
        frontRightState.Initialize(context, frontRightWheel, context.entity);
        rearLeftState.Initialize(context, rearLeftWheel, context.entity);
        rearRightState.Initialize(context, rearRightWheel, context.entity);
        const auto updateWheelContact = [&](WheelController& wheel, Entity wheelEntity)
            {
                wheel.suspensionRestLength = suspensionRestLength;
                wheel.maximumSuspensionTravel = maximumSuspensionTravel;
                wheel.springStiffness = springStiffness;
                wheel.dampingRatio = dampingRatio;
            };

        updateWheelContact(frontLeftState, frontLeftWheel);
        updateWheelContact(frontRightState, frontRightWheel);
        updateWheelContact(rearLeftState, rearLeftWheel);
        updateWheelContact(rearRightState, rearRightWheel);
    }


    void VehicleController::OnUpdate(const ScriptContext& context)
    {
        
        auto* chassisBody = registry->GetComponent<RigidBodyComponent>(context.entity);
        if (chassisBody == nullptr || chassisBody->isKinematic || chassisBody->inverseMass <= 0.0f)
        {
            return;
        }

        ReadInput();
        frontLeftState.motorTorque = 0.0f;
        frontRightState.motorTorque = 0.0f;
        rearLeftState.motorTorque = 0.0f;
        rearRightState.motorTorque = 0.0f;
        const Vec3 chassisWorldPosition = context.scene.GetWorldPosition(context.entity);
        Vec3 vehicleForward{};
        const WheelController* referenceWheel = nullptr;
        if (frontLeftState.grounded) referenceWheel = &frontLeftState;
        else if (frontRightState.grounded) referenceWheel = &frontRightState;
        else if (rearLeftState.grounded) referenceWheel = &rearLeftState;
        else if (rearRightState.grounded) referenceWheel = &rearRightState;

        if (referenceWheel != nullptr)
            vehicleForward = Cross(referenceWheel->axleAxis, referenceWheel->groundContactNormal).Normalized();
        if (vehicleForward.LengthSquared() <= 1.0e-6f)
            vehicleForward = context.scene.GetWorldMatrix(context.entity).TransformDirection({ 1.0f, 0.0f, 0.0f }).Normalized();

        const float longitudinalSpeed = Dot(chassisBody->velocity, vehicleForward);
        if (std::abs(longitudinalSpeed) > directionChangeSpeed && throttleInput_ * longitudinalSpeed < 0.0f)
        {
            // Opposite input brakes first; reverse/forward drive engages once
            // the chassis is nearly stopped.
            brakeInput_ = (std::max)(brakeInput_, std::abs(throttleInput_));
            throttleInput_ = 0.0f;
        }
        if (brakeInput_ > 0.0f)
            throttleInput_ = 0.0f;

        engine.throttle = throttleInput_;
        ApplyThrottle(*chassisBody, context.deltaTime);
        ApplySteering(*chassisBody, context.deltaTime);
        ApplyBraking(*chassisBody, context.deltaTime);

        // Drive all wheels

        const auto applyWheelMotion = [&](WheelController& wheel, Entity wheelEntity)
            {
                const float targetAngle = std::clamp(steeringInput_ * 25.0f, -25.0f, 25.0f);
                wheel.UpdateInertiaRotation( context.deltaTime, longitudinalSpeed);
            };

        const auto applyWheelDrive = [&](WheelController& wheel, Entity wheelEntity)
            {
               // wheel.ApplyDriveTorque(wheel.motorTorque,context.deltaTime,chassisWorldPosition,*chassisBody);
            };

        const auto applyWheelSteering = [&](WheelController& wheel, Entity wheelEntity)
            {
                const float targetAngle = std::clamp(steeringInput_ * 25.0f,-25.0f,25.0f);
                wheel.ApplySteering(targetAngle, context.deltaTime,steeringStrength);
            };

        applyWheelMotion(frontLeftState, frontLeftWheel);
        applyWheelMotion(frontRightState, frontRightWheel);
        applyWheelMotion(rearLeftState, rearLeftWheel);
        applyWheelMotion(rearRightState, rearRightWheel);

        applyWheelDrive(frontLeftState, frontLeftWheel);
        applyWheelDrive(frontRightState, frontRightWheel);
        applyWheelDrive(rearLeftState, rearLeftWheel);
        applyWheelDrive(rearRightState, rearRightWheel);

        applyWheelSteering(frontLeftState, frontLeftWheel);
        applyWheelSteering(frontRightState, frontRightWheel);


        frontLeftState.UpdateGroundContact();
        frontRightState.UpdateGroundContact();
        rearLeftState.UpdateGroundContact();
        rearRightState.UpdateGroundContact();
        frontLeftState.ApplyTireForces(context.deltaTime);
        std::clog << "after FL: angVel.y=" << chassisBody->angularVelocity.y << '\n';
        frontRightState.ApplyTireForces(context.deltaTime);
        std::clog << "after FR: angVel.y=" << chassisBody->angularVelocity.y << '\n';
        rearLeftState.ApplyTireForces(context.deltaTime);
        std::clog << "after RL: angVel.y=" << chassisBody->angularVelocity.y << '\n';
        rearRightState.ApplyTireForces(context.deltaTime);
        std::clog << "after RR: angVel.y=" << chassisBody->angularVelocity.y << '\n';
        // Steering angle actually reached
        std::clog << "steerAngle FL=" << frontLeftState.currentSteeringAngle
            << " FR=" << frontRightState.currentSteeringAngle << '\n';

        // Which way the front wheel is actually facing, in world space
        Vec3 flForward = Cross(frontLeftState.axleAxis, frontLeftState.groundContactNormal);
        std::clog << "FL forward=(" << flForward.x << "," << flForward.z << ")\n";

        // Chassis's own current forward direction + yaw rate
        Vec3 chassisForward = context.scene.GetWorldMatrix(context.entity).TransformDirection({ 1,0,0 });
        std::clog << "chassis forward=(" << chassisForward.x << "," << chassisForward.z
            << ") angVel.y=" << chassisBody->angularVelocity.y << '\n';


#ifndef NDEBUG
        static std::uint32_t diagnosticUpdateCount = 0U;
        if (++diagnosticUpdateCount >= 60U)
        {
            diagnosticUpdateCount = 0U;
            const float totalNormalForce = frontLeftState.normalForce + frontRightState.normalForce + rearLeftState.normalForce + rearRightState.normalForce;
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
        const auto& input = qmec::input::InputSystem::Instance();
        throttleInput_ = (input.IsKeyDown(qmec::input::Key::W) ? 1.0f : 0.0f) - (input.IsKeyDown(qmec::input::Key::S) ? 1.0f : 0.0f);
        brakeInput_ = input.IsKeyDown(qmec::input::Key::Space) ? 1.0f : 0.0f;
        steeringInput_ = (input.IsKeyDown(qmec::input::Key::A) ? 1.0f : 0.0f) - (input.IsKeyDown(qmec::input::Key::D) ? 1.0f : 0.0f);

    }

    float VehicleController::SampleTorqueCurve(const EngineComponent& engine, float rpm)
    {
        const auto& curve = engine.torqueCurve;

        if (rpm <= curve.front().rpm)
        {
            return curve.front().torque;
        }

        if (rpm >= curve.back().rpm)
        {
            return curve.back().torque;
        }

        for (std::size_t i = 0; i + 1 < curve.size(); ++i)
        {
            const auto& a = curve[i];
            const auto& b = curve[i + 1];

            if (rpm <= b.rpm)
            {
                const float t =(rpm - a.rpm) / (b.rpm - a.rpm);

                return a.torque + t * (b.torque - a.torque);
            }
        }

        return 0.0f;
    }

    void VehicleController::ApplyThrottle(qmec::scene::components::RigidBodyComponent& ,float deltaTime) noexcept
    {
        const float availableTorque = SampleTorqueCurve(engine, engine.rpm);
        const float engineTorque = availableTorque * engine.throttle;

        if (!engine.clutchEngaged)
        {
            const float netTorque = engineTorque - engine.frictionTorque;
            const float angularAcceleration = netTorque / engine.rotationalInertia;
            const float rpmChange = angularAcceleration * deltaTime * (60.0f / (2.0f * std::numbers::pi_v<float>));
            engine.rpm = std::clamp(engine.rpm + rpmChange,0.0f,engine.maxRPM);
            return;
        }
        const float gearRatio = drivetrain.gearRatios[drivetrain.currentGearRatio];
        const float totalGearRatio = gearRatio * drivetrain.finalDriveRatio;
        const float totalWheelTorque = engineTorque* totalGearRatio* drivetrain.efficiency;
        const float drivenWheelCount = engine.AWD ? 4.0f : 2.0f;
        const float torquePerDrivenWheel = totalWheelTorque / drivenWheelCount;
        if (!engine.AWD)
        {
            rearLeftState.motorTorque = torquePerDrivenWheel;
            rearRightState.motorTorque = torquePerDrivenWheel;
            return;
        }
        rearLeftState.motorTorque = torquePerDrivenWheel;
        rearRightState.motorTorque = torquePerDrivenWheel;
        frontLeftState.motorTorque = torquePerDrivenWheel;
        frontRightState.motorTorque = torquePerDrivenWheel;

    }

    void VehicleController::ApplySteering(qmec::scene::components::RigidBodyComponent&, float) noexcept
    {
        frontLeftState.steerYaw = steeringInput_ * steeringStrength;
        frontRightState.steerYaw = steeringInput_ * steeringStrength;
    }

    void VehicleController::ApplyBraking(qmec::scene::components::RigidBodyComponent&, float) noexcept
    {
        const float wheelBrakeTorque = (std::max)(0.0f, brakeInput_) * (std::max)(0.0f, brakeTorquePerWheel);
        frontLeftState.brakeTorque = wheelBrakeTorque;
        frontRightState.brakeTorque = wheelBrakeTorque;
        rearLeftState.brakeTorque = wheelBrakeTorque;
        rearRightState.brakeTorque = wheelBrakeTorque;
    }
}
