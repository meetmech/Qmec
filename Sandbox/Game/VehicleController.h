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

        void OnUpdate(const ScriptContext& context) override;

    private:
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
