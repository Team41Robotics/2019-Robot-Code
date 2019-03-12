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

#define kPercentOutput ctre::phoenix::motorcontrol::ControlMode::PercentOutput

// Important PID macros
#ifdef FINAL_ROBOT //PID FOR FINAL BOT

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
#define LIFT_MAX_DOWN -0.2

#else // PRACTICE ROBOT

#define ARM_MIN 0
#define ARM_MAX 1000
#define ARM_KP 1.0
#define ARM_KI 0.06
#define ARM_KD 0.0
#define ARM_TOLERANCE 1000
#define ARM_MAX_UP 0.4
#define ARM_FF 0.1

#define LIFT_MIN 0
#define LIFT_MAX 800000
#define LIFT_KP 2.0
#define LIFT_KI 0
#define LIFT_KD 0
#define LIFT_TOLERANCE 20000
#define LIFT_PID_RANGE 200000
#define LIFT_INTEGRAL_MAX 0.0
#define LIFT_MAX_DOWN (-0.2)

#endif

using namespace frc;

class Arm
{
	public:
		Arm();
		virtual ~Arm();
		void ControllerMove(Joystick *rightJoy, Joystick *buttonBoard, bool manual);
		void Reset();
		bool Zeroed();
		bool ArmPID(double goal);
		bool LiftPID(double goal);
		bool armZeroed, liftZeroed;
		void ArmSet(double num);
		void LiftSet(double num);
		DigitalInput *liftTopLimit, *liftBottomLimit, *armTopLimit;

	private:
		bool BothPID(double goalA, double goalL);
		DoubleSolenoid *clawSol, *shootSol;
		Timer timer;
		bool timing;
		TalonSRX *arm0, *arm1, *lift0, *lift1;
		frc::Encoder *armEnc, *liftEnc;
		double arm_integral, arm_prevError, lift_integral, lift_prevError;
		bool armFirstTime, liftFirstTime;
		bool openClaw;
		bool clawButtonPressed;
};