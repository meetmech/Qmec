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
                });
        (void)registered;
    }
}
