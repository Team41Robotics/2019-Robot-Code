/*
 * Arm.cpp
 *
 *  Created on: Jan 2, 2019
 *      Author: team 41 robotics
 */

#include "Arm.h"
#include "Ports.h"

Arm::Arm() {
	clawSol = new DoubleSolenoid(PORTS::SOL_CLAW_PCM, PORTS::SOL_CLAW_IN,PORTS::SOL_CLAW_OUT);
	shootSol = new DoubleSolenoid(PORTS::SOL_SHOOT_PCM, PORTS::SOL_SHOOT_IN,PORTS::SOL_SHOOT_OUT);
	// For gearbox motors
	arm0 = new TalonSRX(PORTS::ARM_TALON0);
	arm1 = new TalonSRX(PORTS::ARM_TALON1);
	lift0 = new TalonSRX(PORTS::LIFT_TALON0);
	lift1 = new TalonSRX(PORTS::LIFT_TALON1);
	armEnc = new Encoder(PORTS::ARM_ENCODER_A, PORTS::ARM_ENCODER_B, false, Encoder::EncodingType::k4X);
	armEnc->Reset();
	liftEnc = new Encoder(PORTS::LIFT_ENCODER_A, PORTS::LIFT_ENCODER_B, false, Encoder::EncodingType::k4X);
	liftEnc->Reset();

	openClaw = false;
	clawButtonPressed = false;

	liftTopLimit = new DigitalInput(PORTS::LIFT_TOP_LIMIT);
	liftBottomLimit = new DigitalInput(PORTS::LIFT_BOTTOM_LIMIT);
	armTopLimit = new DigitalInput(PORTS::ARM_TOP_LIMIT);
	armBottomLimit = new DigitalInput(PORTS::ARM_BOTTOM_LIMIT);


	// armPid = new PID(ARM_KP, ARM_KI, ARM_KD, ARM_TOLERANCE);
	// liftPid = new PID(LIFT_KP, LIFT_KI, LIFT_KD, LIFT_TOLERANCE);
}

Arm::~Arm() {
	// TODO Auto-generated destructor stub
}

void Arm::ArmSet(double num) {
	// if ((num > 0 && armTopLimit->Get()) || (num < 0 && armBottomLimit->Get())) return;
	arm0->Set(kPercentOutput, num);
	arm1->Set(kPercentOutput, num);
}

void Arm::LiftSet(double num) {
	// if ((num > 0 && liftTopLimit->Get()) || (num < 0 && liftBottomLimit->Get())) return;
	lift0->Set(kPercentOutput, -num); // Lift motors are wired in reverse
	lift1->Set(kPercentOutput, -num);
	SmartDashboard::PutNumber("lift talon", num);
}

void Arm::ControllerMove(Joystick *rightJoy, Joystick *buttonBoard) {
    SmartDashboard::PutNumber("lift enc raw", liftEnc->GetRaw());
    SmartDashboard::PutNumber("arm enc raw", armEnc->GetRaw());

	SmartDashboard::PutNumber("lift top limit", liftTopLimit->Get());
	SmartDashboard::PutNumber("lift bot limit", liftBottomLimit->Get());
	SmartDashboard::PutNumber("arm top limit", armBottomLimit->Get());
	SmartDashboard::PutNumber("arm bot limit", armTopLimit->Get());

	// Open and close claw
	if (buttonBoard->GetRawButton(BUTTONS::TOGGLE_CLAW)) {
		if(!clawButtonPressed) {
			openClaw = !openClaw;
			clawButtonPressed = true;
		}
		clawSol->Set(openClaw ? DoubleSolenoid::Value::kReverse : DoubleSolenoid::Value::kForward);
	}
	else {
		clawSol->Set(DoubleSolenoid::Value::kOff);
		clawButtonPressed = false;
	}
	if (buttonBoard->GetRawButton(BUTTONS::DRIVER_OVERRIDE)) {
		if (buttonBoard->GetRawButton(BUTTONS::CLAW_OPEN)) clawSol->Set(DoubleSolenoid::Value::kReverse);
		else if (buttonBoard->GetRawButton(BUTTONS::CLAW_CLOSE)) clawSol->Set(DoubleSolenoid::Value::kForward);
	}

	// Fire shooting piston
	if (rightJoy->GetRawButton(BUTTONS::CLAW_SHOOT)) {
		shootSol->Set(DoubleSolenoid::Value::kReverse);
		clawSol->Set(DoubleSolenoid::Value::kReverse); // Open claw
		timer.Reset();
		timer.Start();

		timing = true;
	}
	else if (timing && timer.Get() >= 0.5) { // Make sure to retract piston 0.5 sec after firing
		shootSol->Set(DoubleSolenoid::Value::kForward);
		if (timer.Get() >= 0.52) { // Gives .02 second for piston to retract?
			timer.Stop();
			timing = false;
		}
	}
	else
		shootSol->Set(DoubleSolenoid::Value::kOff);
	// Moving the arm and lift
	double armVal = armEnc->GetRaw();
	double liftVal = liftEnc->GetRaw();
	// Manually move arm
	double arm_speed = 0.5;
	if (buttonBoard->GetRawButton(BUTTONS::ARM_DEFAULT)) arm_speed = -buttonBoard->GetRawAxis(BUTTONS::ARM_SPEED) / 2.0 + 0.5;
	if (buttonBoard->GetRawButton(BUTTONS::ARM_UP) /* && !armTopLimit->Get() */)
		ArmSet(arm_speed);
	else if (buttonBoard->GetRawButton(BUTTONS::ARM_DOWN) /* && !armBottomLimit->Get() */)
		ArmSet(-arm_speed*0.5);
	else
		ArmSet(0.1);
	// Manually move lift
	double lift_speed = 0.8;
	// if (buttonBoard->GetRawButton(BUTTONS::LIFT_DEFAULT)) lift_speed = -buttonBoard->GetRawAxis(BUTTONS::LIFT_SPEED) / 2.0 + 0.5;
	int level = buttonBoard->GetPOV(0) / 45;
	// First things first, let's zero the lift
	// if (!liftBottomLimit->Get()) {
	// 	LiftSet(LIFT_MAX_DOWN);
	// 	liftEnc->Reset();
	// } else {
		if (buttonBoard->GetRawButton(BUTTONS::LIFT_UP) /* && !liftTopLimit->Get() */)
			LiftSet(lift_speed);
		else if (buttonBoard->GetRawButton(BUTTONS::LIFT_DOWN) /* && !liftBottomLimit->Get() */)
			LiftSet(-lift_speed*0.25);
		else {
			// PID movement
			switch (level){
				// case 1:
				// 	BothPID(ARM_HEIGHTS::HATCH_1, LIFT_HEIGHTS::HATCH_1); break;
				// case 3:
				// 	BothPID(ARM_HEIGHTS::HATCH_2, LIFT_HEIGHTS::HATCH_2); break;
				// case 5:
				// 	BothPID(ARM_HEIGHTS::HATCH_3, LIFT_HEIGHTS::HATCH_3); break;
				// case 2:
				// 	BothPID(ARM_HEIGHTS::CARGO_1, LIFT_HEIGHTS::CARGO_1); break;
				// case 4:
				// 	BothPID(ARM_HEIGHTS::CARGO_2, LIFT_HEIGHTS::CARGO_2); break;
				// case 6:
				// 	BothPID(ARM_HEIGHTS::CARGO_3, LIFT_HEIGHTS::CARGO_3); break;
				case 1:
					LiftPID(LIFT_HEIGHTS::HATCH_1); break;
				case 6:
					ArmPID(ARM_HEIGHTS::CARGO_3); break;
				case 4:
					ArmPID(ARM_HEIGHTS::CARGO_1); break;
				default:
					LiftSet(0.0);
			}
		}
	// }
}

bool Arm::BothPID(double goalA, double goalL) {
	return ArmPID(goalA) && LiftPID(goalL);
	// if (armFirstTime) {
	// 	arm_integral = 0;
	// 	armFirstTime = false;
	// }
	// if (liftFirstTime) {
	// 	lift_integral = 0;
	// 	liftFirstTime = false;
	// }
	// double armSpeed = armPid->GetPID(armEnc->Get(), goalA);
	// double liftSpeed = liftPid->GetPID(liftEnc->Get(), goalL);
	// bool armSuccess = false;
	// bool liftSuccess = false;
	// if(armSpeed == -1) {
	// 	armFirstTime = true;
	// 	arm_integral = 0;
	// 	ArmSet(0.0);
	// 	armPid->Reset();
	// 	armSuccess = true;
	// }
	// else ArmSet(armSpeed);
	// if(liftSpeed == -1) {
	// 	liftFirstTime = true;
	// 	lift_integral = 0;
	// 	LiftSet(0.0);
	// 	liftPid->Reset();
	// 	liftSuccess = true;
	// }
	// else LiftSet(liftSpeed);
	// return armSuccess && liftSuccess;
}

bool Arm::ArmPID(double goal) {
	if (armFirstTime) {
		arm_integral = 0;
		arm_prevError = 0;
		armFirstTime = false;
	}
	double error = goal - armEnc->GetRaw();
	if (fabs(error) < ARM_TOLERANCE) {
		// End the function right here
		armFirstTime = true;
		arm_integral = 0;
		ArmSet(0.1);
		return true;
	}
	SmartDashboard::PutNumber("arm PID error", error);
	error /= 200000.0;
	arm_integral += error;
	if (arm_integral > 0.3/ARM_KI) arm_integral = 0.3/ARM_KI;
	if (arm_integral < -0.3/ARM_KI) arm_integral = -0.3/ARM_KI;
	double derivative = error - arm_prevError;
	SmartDashboard::PutNumber("arm PID d", derivative);
	SmartDashboard::PutNumber("arm PID i", arm_integral);
	double speed = error * ARM_KP + arm_integral * ARM_KI + derivative * ARM_KD;
	arm_prevError = error;
	if (speed > 1) speed = 1;
	if (speed < -0.25) speed = -0.25;
	SmartDashboard::PutNumber("arm PID speed", speed);
	ArmSet(speed);
	return false	;
}

bool Arm::LiftPID(double goal) {
	if (liftFirstTime) {
		lift_integral = 0;
		lift_prevError = 0;
		liftFirstTime = false;
	}
	double error = goal - liftEnc->GetRaw();
	if (fabs(error) < LIFT_TOLERANCE) {
		// End the function right here
		liftFirstTime = true;
		lift_integral = 0;
		LiftSet(0.0);
		return true;
	}
	if (fabs(error) > LIFT_PID_RANGE) {
		LiftSet(error > 0 ? 0.8 : -0.5);
		return false;
	}

	SmartDashboard::PutNumber("error", error);

	error /= LIFT_PID_RANGE;

	lift_integral += error;
	if (lift_integral > LIFT_INTEGRAL_MAX) lift_integral = LIFT_INTEGRAL_MAX;
	else if (lift_integral < -LIFT_INTEGRAL_MAX) lift_integral = -LIFT_INTEGRAL_MAX;

	double derivative = error - lift_prevError;

	double speed = error * LIFT_KP + lift_integral * LIFT_KI + derivative * LIFT_KD;
	lift_prevError = error;
	if (speed > 0.8) speed = 0.8;
	if (speed < -0.5) speed = -0.5;
	// if (speed >= 0 && speed < 0.25) speed = 0.25;
	// if (speed < 0 && speed > -0.25) speed = -0.25;
	SmartDashboard::PutNumber("PID speed", speed);
	LiftSet(speed);
	return false;
}
