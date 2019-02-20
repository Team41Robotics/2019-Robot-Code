#pragma once
#include <frc/WPILib.h>
using namespace frc;

class Vision {
    public:
        Vision();
        ~Vision();
        void Swivel();
        void Lift(Joystick *joy);
    private:
        Servo *swivelServo, *liftServo;
};