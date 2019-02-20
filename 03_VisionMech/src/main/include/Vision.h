#pragma once

#include <frc/WPILib.h>

using namespace frc;

class Vision {
    public:
        Vision();
        ~Vision();
        void Swivel();
    private:
        Servo *servo;
};