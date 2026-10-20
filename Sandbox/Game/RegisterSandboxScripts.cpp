#include "Game/RegisterSandboxScripts.h"

#include "QMEC/Scripting/Script.h"
#include "Game/VehicleController.h"

namespace qmec::game
{
    void RegisterSandboxScripts()
    {
        static const bool registered =
            ScriptTypeRegistry::Instance().Register<VehicleController>(
                "vehicle_controller", "Vehicle Controller",
                {
                    MakeEntityReferenceField("frontLeftWheel", "Front Left Wheel",
                        &VehicleController::frontLeftWheel),
                    MakeEntityReferenceField("frontRightWheel", "Front Right Wheel",
                        &VehicleController::frontRightWheel),
                    MakeEntityReferenceField("rearLeftWheel", "Rear Left Wheel",
                        &VehicleController::rearLeftWheel),
                    MakeEntityReferenceField("rearRightWheel", "Rear Right Wheel",
                        &VehicleController::rearRightWheel)
                },
                {
                    MakeFloatField("suspensionRestLength", "Suspension Rest Length",
                        &VehicleController::suspensionRestLength, 0.0f, 5.0f, 0.01f),
                    MakeFloatField("maximumSuspensionTravel", "Maximum Suspension Travel",
                        &VehicleController::maximumSuspensionTravel, 0.0f, 2.0f, 0.01f),
                    MakeFloatField("springStiffness", "Spring Stiffness",
                        &VehicleController::springStiffness, 0.0f, 100000.0f, 100.0f),
                    MakeFloatField("dampingRatio", "Damping Ratio",
                        &VehicleController::dampingRatio, 0.0f, 2.0f, 0.05f)
                });
        (void)registered;
    }
}
