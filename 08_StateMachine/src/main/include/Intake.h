#pragma once

#include <frc/WPILib.h>
#include <ctre/Phoenix.h>
#include "Ports.h"

#define kPercentOutput ctre::phoenix::motorcontrol::ControlMode::PercentOutput

#define INTAKE_KP 2
#define INTAKE_KI 0.05
#define INTAKE_KD 0.0
#define PID_RANGE 200000
#define TOLERANCE 1000

#define INTAKE_ROTATION_MAX -0.5

using namespace frc;

class Intake {
	public:
		Intake();	
		void ControllerMove(Joystick *buttonBoard, bool manual);
		void IntakeRotationSet(double num);
		bool IntakePID(double goal);
		void Reset();
		bool Zeroed();
		bool intakeZeroed;
		DigitalInput *topLimit;
	private:
		TalonSRX *intake, *intakeUp0, *intakeUp1;
		Encoder *encoder;
		double integral, prevError;
		bool firstTime;
};