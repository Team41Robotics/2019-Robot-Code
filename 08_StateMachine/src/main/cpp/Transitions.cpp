#include "PosControl.h"

void PosControl::HoldPosition(){
	//arm->ArmPID(ArmHeight(currentState));
	//arm->LiftPID(LiftHeight(currentState));
	arm->ArmSet(0.1);
	arm->LiftSet(0.1);
}

bool PosControl::ToStart(){
	if (arm->liftBottomLimit->Get() && arm->armTopLimit->Get() && intake->topLimit->Get()){
		return true;
	} else {
		arm->liftZeroed = false;
		arm->armZeroed = false;
		intake->intakeZeroed = false;
		return false;
	}
}

bool PosControl::Ball2Cargo(Evt targetState) { // Technically works for hatches too, but wouldn't happen
	switch (step) {
		case 0: // Raise lift first
			if (arm->LiftPID(LIFT_HEIGHTS::INTERMEDIATE))
				step++;
			break;
		case 1: // Then take intake in, raise arm, and continue raising lift
			bool check1 = intake->IntakePID(INTAKE_HEIGHTS::START);
			bool check2 = arm->LiftPID(LiftHeight(targetState));
			bool check3 = arm->ArmPID(ArmHeight(targetState));
			if (check1 && check2 && check3)
				step = -1;
			break;
	}
	return (step == -1) ? true : false;
}

bool PosControl::Cargo2Ball(){
	switch (step) {
		case 0: // If we were at a low cargo or hatch, raise the lift first
			if (currentState == Evt::CARGO_1 || currentState == Evt::HATCH_1 || IsStart(currentState)){
				if (arm->LiftPID(LIFT_HEIGHTS::INTERMEDIATE))
					step++;
			} else {
				step++; // If not, don't worry about it
			}
			break;
		case 1: {// Put the intake out
			arm->LiftPID(LIFT_HEIGHTS::INTERMEDIATE);
			if (intake->IntakePID(INTAKE_HEIGHTS::BALL))
				step++;
			break;
		}
		case 2: // Lower the arm
			if (arm->ArmPID(ARM_HEIGHTS::BALL))
				step++;
			break;
		case 3: // Lower the lift
			if (arm->LiftPID(LIFT_HEIGHTS::BALL))
				step = -1;
			break;
	}
	return (step == -1) ? true : false;
}

bool PosControl::Cargo2Cargo(Evt targetState){
	// Move lift and arm
	bool check1 = arm->ArmPID(ArmHeight(targetState));
	bool check2 = arm->LiftPID(LiftHeight(targetState));
	return check1 && check2;
}

bool PosControl::Anything2Level() {
	return intake->IntakePID(INTAKE_HEIGHTS::LEVEL);
}