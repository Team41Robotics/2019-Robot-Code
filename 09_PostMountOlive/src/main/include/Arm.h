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
#include <cmath>

#define kPercentOutput ctre::phoenix::motorcontrol::ControlMode::PercentOutput

#define PI 3.14159

// Dimension macros
#define LIFT_SHAFT_RADIUS (0.875/39.37)
#define LIFT_GEAR_RATIO (100.0/3.0)
#define ARM_GEAR_RATIO 300.0
#define ARM_LENGTH (15/39.37)

// Important PID macros
#ifdef FINAL_ROBOT //PID FOR FINAL BOT

#define ARM_MIN 0
#define ARM_MAX 1000
#define ARM_KP 1.0
#define ARM_KI 0.1
#define ARM_KD 10
#define ARM_TOLERANCE 1000
#define ARM_MAX_UP 0.4
#define ARM_FF 0.1


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
#define LIFT_TOLERANCE 1
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
		void ControllerMove(Joystick *rightJoy, Joystick *buttonBoard);
		void Reset();
		bool armZeroed, liftZeroed;
		void Grab();

	private:
		void ArmSet(double num);
		void LiftSet(double num);
		bool BothPID(double goalA, double goalL, bool fine);
		bool ArmPID(double goal, bool fine);
		bool ArmPIDFine();
		bool LiftPID(double goal, bool fine);
		PID *armPid, *liftPid;
		DoubleSolenoid *clawSol, *shootSol;
		Timer timer;
		bool timing;
		TalonSRX *arm0, *arm1, *lift0, *lift1;
		frc::Encoder *armEnc, *liftEnc;
		double arm_integral, arm_prevError, lift_integral, lift_prevError, fine_integral;
		bool armFirstTime, liftFirstTime;
		bool openClaw;
		bool clawButtonPressed;
		DigitalInput *liftTopLimit, *liftBottomLimit, *armTopLimit;
};

#ifdef FINAL_ROBOT
namespace LIFT_HEIGHTS {
	enum VALUES {
		HATCH_LOAD = 272644,
		HATCH_1 = 487238, // 672001
		HATCH_2 = 487238,
		HATCH_3 = 996858
	};
}

namespace ARM_HEIGHTS {
	enum VALUES {
		HATCH_LOAD = -201284, // 368179.0
		HATCH_1 = -252979 ,//-274796 too low
		HATCH_2 = -97123,
		HATCH_3 = -76553
	};
}

#else // PRACTICE BOT
namespace LIFT_HEIGHTS {
	enum VALUES {
		HATCH_LOAD = 323048,
		HATCH_1 = 323048, // 672001
		HATCH_2 = 696027,
		HATCH_3 = 1028275
	};
}

namespace ARM_HEIGHTS {
	enum VALUES {
		HATCH_LOAD = -263800, // 368179.0
		HATCH_1 = -263800 ,//-274796 too low
		HATCH_2 = -182156,
		HATCH_3 = -98880
	};
}
#endif