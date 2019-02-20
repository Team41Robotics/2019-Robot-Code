/*
 * Arm.h
 *
 *  Created on: Jan 2, 2019
 *      Author: team 41 robotics
 */

#pragma once

#include <frc/WPILib.h>
#include <ctre/Phoenix.h>
#include "Ports.h"
#include "PID.h"

#define kPercentOutput ctre::phoenix::motorcontrol::ControlMode::PercentOutput

// Important PID macros
#define ARM_MIN 0
#define ARM_MAX 1000
#define ARM_KP 1.0
#define ARM_KI 0.1
#define ARM_KD 10
#define ARM_TOLERANCE 1000

#define LIFT_MIN 0
#define LIFT_MAX 800000
#define LIFT_KP 2.0
#define LIFT_KI 0
#define LIFT_KD 0
#define LIFT_TOLERANCE 2000
#define LIFT_PID_RANGE 200000
#define LIFT_INTEGRAL_MAX 0.0
#define LIFT_MAX_DOWN -0.5

using namespace frc;

class Arm
{
	public:
		Arm();
		virtual ~Arm();
		void ControllerMove(Joystick *rightJoy, Joystick *buttonBoard);

	private:
		void ArmSet(double num);
		void LiftSet(double num);
		bool BothPID(double goalA, double goalL);
		bool ArmPID(double goal);
		bool LiftPID(double goal);
		PID *armPid, *liftPid;
		DoubleSolenoid *clawSol, *shootSol;
		Timer timer;
		bool timing;
		TalonSRX *arm0, *arm1, *lift0, *lift1;
		frc::Encoder *armEnc, *liftEnc;
		double arm_integral, arm_prevError, lift_integral, lift_prevError;
		bool armFirstTime, liftFirstTime;
		bool openClaw;
		bool clawButtonPressed;
		DigitalInput *liftTopLimit, *liftBottomLimit, *armTopLimit, *armBottomLimit;
};

namespace ARM_HEIGHTS {
	enum VALUES {
		HATCH_1 = 100,
		HATCH_2 = 200,
		HATCH_3 = 300,
		CARGO_1 = 100000,
		CARGO_2 = 250,
		CARGO_3 = 200000
	};
}

namespace LIFT_HEIGHTS {
	enum VALUES {
		HATCH_1 = 200000,
		HATCH_2 = 200,
		HATCH_3 = 300,
		CARGO_1 = 150,
		CARGO_2 = 250,
		CARGO_3 = 350
	};
}