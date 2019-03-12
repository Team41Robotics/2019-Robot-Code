#pragma once

#include <frc/WPILib.h>

using namespace frc;

class Vision
{
	public:
		Vision();
	virtual ~Vision();
		void Swivel();
		void Lift(Joystick *buttonBoard);
	private:
		Servo *swivel, *lift;
};