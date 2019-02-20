#pragma once

#include <frc/WPILib.h>

using namespace frc;

class Vision
{
  public:
	Vision();
	virtual ~Vision();
    void Swivel(Joystick *leftJoy, Joystick *rightJoy);
    void Lift(Joystick *leftJoy, Joystick *rightJoy);
  private:
	Potentiometer *pot;
    Servo *servo;
  DigitalInput *limitSwitchTop;
   DigitalInput *limitSwitchBot;
};