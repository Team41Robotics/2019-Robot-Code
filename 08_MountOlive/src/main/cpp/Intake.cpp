#include "Intake.h"

Intake::Intake() {
    intake = new TalonSRX(PORTS::TALON_INTAKE);
    intakeUp0 = new TalonSRX(PORTS::TALON_INTAKE_UP0);
    intakeUp1 = new TalonSRX(PORTS::TALON_INTAKE_UP1);
    encoder = new frc::Encoder(PORTS::INTAKE_ENC_A,PORTS::INTAKE_ENC_B, true, Encoder::EncodingType::k4X);
    topLimit = new DigitalInput(PORTS::INTAKE_TOP_LIMIT);

    intakeZeroed = false;
}

void Intake::Reset(){
    intakeZeroed = false;
    integral = 0;
}

void Intake::IntakeRotationSet(double num){
    if(num < 0 && !topLimit->Get())
        num = 0;
     intakeUp0->Set(kPercentOutput,-num);
    intakeUp1->Set(kPercentOutput,-num);
}

void Intake::ControllerMove(Joystick *buttonBoard) {
    SmartDashboard::PutNumber("intake enc raw", encoder->GetRaw());
    SmartDashboard::PutNumber("intake top limit", topLimit->Get());
    //Sucking the ball
	if (buttonBoard->GetRawButton(BUTTONS::INTAKE_IN)){
		intake->Set(kPercentOutput, -1.0); 
	} else if (buttonBoard->GetRawButton(BUTTONS::INTAKE_OUT)){
		intake->Set(kPercentOutput, 1.0); 
	} else {
        intake->Set(kPercentOutput, 0);
    }
    // Moving the intake up and down
    int level = buttonBoard->GetPOV(1) / 45;

    // First things first, let's zero the intake rotation
    // intakeZeroed = true;
    if(topLimit->Get() && !intakeZeroed) {
        IntakeRotationSet(INTAKE_ROTATION_MAX);
    } else {
        if(!intakeZeroed)
            encoder->Reset();
            intakeZeroed = true;
        switch (level){
            case 0: { // Start
                //IntakePID(INTAKE_HEIGHTS::START); break;
                if (topLimit->Get()) intakeZeroed = false;
                else IntakeRotationSet(0);
                break;
            }
            case 1: // Ball
                IntakePID(INTAKE_HEIGHTS::BALL); break;
            case 2: // Level
                IntakePID(INTAKE_HEIGHTS::LEVEL); break;
            case 3: { // Override
                double intake_speed = 0.7;
                if (buttonBoard->GetRawButton(BUTTONS::INTAKE_DEFAULT)) intake_speed = -buttonBoard->GetRawAxis(BUTTONS::INTAKE_SPEED) / 2.0 + 0.5;
                if (buttonBoard->GetRawButton(BUTTONS::INTAKE_UP) /* && encoder->GetRaw() < INTAKE_HEIGHTS::LEVEL */){
                    IntakeRotationSet(intake_speed);
                } else if (buttonBoard->GetRawButton(BUTTONS::INTAKE_DOWN) && topLimit->Get()) {
                    IntakeRotationSet(-intake_speed);
                } else {
                    IntakeRotationSet(0);
                }
                break;}
            default: // Unknown
                IntakeRotationSet(0);
        }
    }
}

bool Intake::IntakePID(double goal) {
	if (firstTime) {
		integral = 0;
		prevError = 0;
		firstTime = false;
	}
	double error = goal - encoder->GetRaw();
	if (fabs(error) < TOLERANCE) {
		// End the function right here
		firstTime = true;
		integral = 0;
		IntakeRotationSet(0.0);
		return true;
	}

	if (fabs(error) > PID_RANGE) {
		IntakeRotationSet(error > 0 ? 0.8 : -0.8);
		return false;
	}

	SmartDashboard::PutNumber("intake PID error", error);

	error /= 800000;

	integral += error;

	double derivative = error - prevError;

	double speed = error * INTAKE_KP + integral * INTAKE_KI + derivative * INTAKE_KD;
	prevError = error;
	if (speed > 0.8) speed = 0.8;
	if (speed < -0.8) speed = -0.8;
	// if (speed < 0 && speed > -0.25) speed = -0.25;
	SmartDashboard::PutNumber("intake PID speed", speed);
	IntakeRotationSet(speed);
	return false;
}