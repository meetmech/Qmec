#include "Game/VehicleController.h"

#include "QMEC/Scene/Components/RigidBodyComponent.h"
#include "QMEC/Scene/Scene.h"

namespace qmec
{
    void VehicleController::OnUpdate(const ScriptContext& context)
    {
        auto& registry = context.scene.GetRegistry();
        auto* chassisBody = registry.GetComponent<RigidBodyComponent>(context.entity);
        if (chassisBody == nullptr || chassisBody->isKinematic || chassisBody->inverseMass <= 0.0f)
        {
            return;
        }

        // Each wheel probes the ground and applies its suspension force to this chassis body.
        (void)frontLeftState.UpdateGroundContact(context.scene, frontLeftWheel, context.entity, *chassisBody);
        (void)frontRightState.UpdateGroundContact(context.scene, frontRightWheel, context.entity, *chassisBody);
        (void)rearLeftState.UpdateGroundContact(context.scene, rearLeftWheel, context.entity, *chassisBody);
        (void)rearRightState.UpdateGroundContact(context.scene, rearRightWheel, context.entity, *chassisBody);
    }
}
