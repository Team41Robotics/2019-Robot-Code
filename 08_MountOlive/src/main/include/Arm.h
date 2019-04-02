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
#define LIFT_TOLERANCE 2000
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
		DigitalInput *liftTopLimit, *liftBottomLimit, *armTopLimit;
};

#ifdef FINAL_ROBOT
namespace ARM_HEIGHTS {
	enum VALUES {
		HATCH_1 = -311765,
		HATCH_1_LOW = -335396, //-311765
		HATCH_1_HIGH = -338401, //-311765
		HATCH_2 = -9747,
		HATCH_3 = -34744,
		CARGO_1 = -340777,
		CARGO_2 = 485,
		CARGO_3 = -334,
		BALL = -303093
	};
}

namespace LIFT_HEIGHTS {
	enum VALUES {
		HATCH_1 = 714750,
		HATCH_1_LOW = 717668, // 672001
		HATCH_1_HIGH = 754719, // 714750,
		HATCH_2 = 294046,
		HATCH_3 = 917728,
		CARGO_1 = 840095,
		CARGO_2 = 353904,
		CARGO_3 = 1000000,
		BALL = 317680
	};
}

#else // PRACTICE BOT
namespace ARM_HEIGHTS {
	enum VALUES {
		HATCH_1 = -336000, // 368179.0
		HATCH_2 = -35623,
		HATCH_3 = -34000,
		CARGO_1 = -352477,
		CARGO_2 = 485,
		CARGO_3 = -334
	};
}

namespace LIFT_HEIGHTS {
	enum VALUES {
		HATCH_1 = 760000,
		HATCH_2 = 353382,
		HATCH_3 = 1008113,
		CARGO_1 = 826562,
		CARGO_2 = 353904,
		CARGO_3 = 1000000
	};
}
#endif