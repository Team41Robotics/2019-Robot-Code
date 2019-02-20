#pragma once
#include <frc/WPILib.h>
#include <ctre/Phoenix.h>

using namespace frc;

class Driving {
    public:
        Driving();
        ~Driving();
        void ControllerMove(Joystick *leftJoy, Joystick *rightJoy);
        void Drive(double left, double right);
    private:
        TalonSRX *LFTalon;
        TalonSRX *LBTalon;
        TalonSRX *RFTalon;
        TalonSRX *RBTalon;
};