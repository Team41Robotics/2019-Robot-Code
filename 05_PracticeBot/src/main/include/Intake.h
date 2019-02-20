#pragma once

#include <frc/WPILib.h>
#include <ctre/Phoenix.h>
#include "Ports.h"

#define kPercentOutput ctre::phoenix::motorcontrol::ControlMode::PercentOutput

#define INTAKE_KP 1
#define INTAKE_KI 0
#define INTAKE_KD 0
#define TOLERANCE 8

using namespace frc;

class Intake {
	public:
		Intake();	
		void ControllerMove(Joystick *buttonBoard);
		void IntakeRotationSet(double num);
		bool IntakePID(double goal);
	private:
		TalonSRX *intake, *intakeUp0, *intakeUp1;
		Encoder *encoder;
		double integral, prevError;
		bool firstTime;
};

enum INTAKE_HEIGHTS {
	START = 0,
	BALL = 0,
	LEVEL = 0
};