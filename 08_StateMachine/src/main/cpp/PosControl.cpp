/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "PosControl.h"

PosControl::PosControl(Arm *a, Intake *i) {
	arm = a;
	intake = i;
}

void PosControl::Reset(){
	currentState = Evt::START;
	SmartDashboard::PutString("current state", State2String(currentState));
}

/**
 * Checks for collisions
 */
bool PosControl::RunPresets(Joystick *buttonBoard) {
	int pov0 = buttonBoard->GetPOV(0) / 45;
	int pov1 = buttonBoard->GetPOV(1) / 45;

	// Nothing is selected on either touch screen
	// defer to teleop controls
	if (pov0 == 0 && pov1 == 3)
		return true;

	// The meat of the function: it's secretly a state machine!
	Evt targetState = GetTargetState(pov0, pov1);
	
	// Figure out what transition to use
	DoTransition(targetState);

	return false; // We are doing stuff, so no teleop controls
}

void PosControl::DoTransition(Evt targetState) {
	SmartDashboard::PutString("transition","working");
    SmartDashboard::PutString("target state", State2String(targetState));
    SmartDashboard::PutString("current state", State2String(currentState));
	SmartDashboard::PutNumber("current step", step);
	// To start
	if (targetState == Evt::START && arm->liftBottomLimit->Get()){
		if (ToStart()){
			currentState = targetState;
		}
	}
	// If in the right place, maintain position
	if (currentState == targetState){
		HoldPosition();
	}
	// Anything to Level
	else if (targetState == Evt::LEVEL) {
		if (Anything2Level()) {
			currentState = targetState;
			step = 0;
		}
	}
	// If we picked up a ball and are going to place it now
	else if (IsBall(currentState) && IsCargoOrHatch(targetState)) {
		if (Ball2Cargo(targetState)) {
			currentState = targetState;
			step = 0;
		}
	}
	// If we want to go from a cargo or hatch to ball
	else if (IsCargoOrHatch(currentState) && IsBall(targetState)) {
		if (Cargo2Ball()) {
			currentState = targetState;
			step = 0;
		}
	}
	// Cargo or hatch to cargo or hatch, including start
	else if ((IsCargoOrHatch(currentState)) && IsCargoOrHatch(targetState)) {
		if (Cargo2Cargo(targetState)) {
			currentState = targetState;
			step = 0;
		}
	}
	else SmartDashboard::PutString("transition","ERROR!");
}

Evt PosControl::GetTargetState(int pov0, int pov1) {
	Evt targetState = Evt::NONE;
	// Interpreting data from the right touch screen
	switch(pov1) {
		case 0:
			targetState = Evt::START;
			break;
		case 1:
			targetState = Evt::BALL;
			break;
		case 2:
			targetState = Evt::LEVEL;
			break;
        default:
            targetState = Evt::START;
            break;
	}
	// Interpreting data from the left touch screen
	// This is done second because its data is a priority over the right screen
	switch (pov0) {
		case 1:
			targetState = Evt::HATCH_1;
			break;
		case 2:
			targetState = Evt::CARGO_1;
			break;
		case 3:
			targetState = Evt::HATCH_2;
			break;
		case 4:
			targetState = Evt::CARGO_2;
			break;
		case 5:
			targetState = Evt::HATCH_3;
			break;
		case 6:
			targetState = Evt::CARGO_3;
			break;
		case 7:
			targetState = Evt::HAB_12;
			break;
		case 8:
			targetState = Evt::HAB_23;
			break;
	}
	return targetState;
}


/* Evt PosControl::GetTargetState(int pov0, int pov1){
	int value = pov0 + (9 * pov1); // Can be 0 to 80
	switch (value){
		
	}
} */

bool PosControl::IsCargo(Evt e) {
	return e == Evt::CARGO_1 || e == Evt::CARGO_2 || e == Evt::CARGO_3;
}

bool PosControl::IsHatch(Evt e) {
	return e == Evt::HATCH_1 || e == Evt::HATCH_2 || e == Evt::HATCH_3;
}

bool PosControl::IsCargoOrHatch(Evt e) {
	return IsCargo(e) || IsHatch(e) || IsStart(e);
}

bool PosControl::IsStart(Evt e) {
	return e == Evt::START || e == Evt::HAB_12 || e == Evt::HAB_23;
}

bool PosControl::IsBall(Evt e) {
	return e == Evt::BALL || e == Evt::LEVEL;
}

/**
 * Returns the arm encoder value of the desired target state
 */
int PosControl::ArmHeight(Evt targetState) {
	switch(targetState) {
		case Evt::CARGO_1:
			return ARM_HEIGHTS::CARGO_1;
		case Evt::CARGO_2:
			return ARM_HEIGHTS::CARGO_2;
		case Evt::CARGO_3:
			return ARM_HEIGHTS::CARGO_3;
		case Evt::HATCH_1:
			return ARM_HEIGHTS::HATCH_1;
		case Evt::HATCH_2:
			return ARM_HEIGHTS::HATCH_2;
		case Evt::HATCH_3:
			return ARM_HEIGHTS::HATCH_3;
		case Evt::BALL:
			return ARM_HEIGHTS::BALL;
		default:
			return 0;
	}
}

/**
 * Returns the lift encoder value of the desired target state
 */
int PosControl::LiftHeight(Evt targetState) {
	switch(targetState) {
		case Evt::CARGO_1:
			return LIFT_HEIGHTS::CARGO_1;
		case Evt::CARGO_2:
			return LIFT_HEIGHTS::CARGO_2;
		case Evt::CARGO_3:
			return LIFT_HEIGHTS::CARGO_3;
		case Evt::HATCH_1:
			return LIFT_HEIGHTS::HATCH_1;
		case Evt::HATCH_2:
			return LIFT_HEIGHTS::HATCH_2;
		case Evt::HATCH_3:
			return LIFT_HEIGHTS::HATCH_3;
		case Evt::BALL:
			return LIFT_HEIGHTS::BALL;
		default:
			return 0;
	}
}

std::string PosControl::State2String(Evt e){
    switch (e){
        case Evt::NONE:
            return "NONE"; break;
        case Evt::START:
            return "START"; break;
        case Evt::BALL:
            return "BALL"; break;
        case Evt::LEVEL:
            return "LEVEL"; break;
        case Evt::CARGO_1:
            return "CARGO_1"; break;
        case Evt::CARGO_2:
            return "CARGO_2"; break;
        case Evt::CARGO_3:
            return "CARGO_3"; break;
        case Evt::HATCH_1:
            return "HATCH_1"; break;
        case Evt::HATCH_2:
            return "HATCH_2"; break;
        case Evt::HATCH_3:
            return "HATCH_3"; break;
        case Evt::HAB_12:
            return "HAB_12"; break;
        case Evt::HAB_23:
            return "HAB_23"; break;
        default:
            return "unknown"; break;
    }
}