/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include "Arm.h"
#include "Intake.h"
#include "Ports.h"

#define Evt eventData::eventTypes

class eventData {
	public:
		enum eventTypes {
      NONE,
			START,
			BALL, LEVEL,
			CARGO_1, CARGO_2, CARGO_3,
			HATCH_1, HATCH_2, HATCH_3,
			HAB_12, HAB_23
		};
};

class PosControl {
	public:
  	PosControl(Arm *a, Intake *i);
		bool RunPresets(Joystick *buttonBoard);
		void Reset();
	private:
		Arm *arm;
		Intake *intake;
		int step;
		Evt currentState = Evt::START;
		Evt GetTargetState(int pov0, int pov1);
		void DoTransition(Evt targetState);
		bool IsCargo (Evt e);
		bool IsHatch(Evt e);
		bool IsCargoOrHatch(Evt e);
		bool IsStart(Evt e);
		bool IsBall(Evt e);
		int ArmHeight(Evt targetState);
		int LiftHeight(Evt targetState);
    std::string State2String(Evt e);

		// Transitions
		bool Ball2Cargo(Evt targetState);
		bool Cargo2Ball();
		bool Cargo2Cargo(Evt targetState);
		bool Anything2Level();
		void HoldPosition();
		bool ToStart();
};

#ifdef FINAL_ROBOT
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

#else // PRACTICE BOT
namespace ARM_HEIGHTS {
	enum VALUES {
		HATCH_1 = -215504, // 368179.0
		HATCH_2 = -35623,
		HATCH_3 = -34000,
		CARGO_1 = -352477,
		CARGO_2 = 485,
		CARGO_3 = -334,
		START = 0,
    	BALL = -313379
	};
}

namespace LIFT_HEIGHTS {
	enum VALUES {
		HATCH_1 = 470872,
		HATCH_2 = 353382,
		HATCH_3 = 1008113,
		CARGO_1 = 826562,
		CARGO_2 = 353904,
		CARGO_3 = 1000000,
		START = 0,
    	BALL = 350733,
    	INTERMEDIATE = 870000
	};
}

namespace INTAKE_HEIGHTS {
  enum VALUES {
    START = 0,
    BALL = 624056,
    LEVEL = 776092
  };
}
#endif