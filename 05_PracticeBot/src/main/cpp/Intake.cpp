#include "Intake.h"

Intake::Intake() {
    intake = new TalonSRX(PORTS::TALON_INTAKE);
    intakeUp0 = new TalonSRX(PORTS::TALON_INTAKE_UP0);
    intakeUp1 = new TalonSRX(PORTS::TALON_INTAKE_UP1);
    encoder = new frc::Encoder(PORTS::INTAKE_ENC_A,PORTS::INTAKE_ENC_B, false, Encoder::EncodingType::k4X);
    encoder->Reset();
}

void Intake::IntakeRotationSet(double num){
    intakeUp0->Set(kPercentOutput,num);
    intakeUp1->Set(kPercentOutput,num);
}

void Intake::ControllerMove(Joystick *buttonBoard) {
    SmartDashboard::PutNumber("intake enc raw", encoder->GetRaw());
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
    switch (level){
        case 0: // Start
            IntakePID(INTAKE_HEIGHTS::START); break;
        case 1: // Ball
            IntakePID(INTAKE_HEIGHTS::BALL); break;
        case 2: // Level
            IntakePID(INTAKE_HEIGHTS::LEVEL); break;
        case 3: { // Custom
            double intake_speed = 0.7;
            if (buttonBoard->GetRawButton(BUTTONS::INTAKE_DEFAULT)) intake_speed = -buttonBoard->GetRawAxis(BUTTONS::INTAKE_SPEED) / 2.0 + 0.5;
            if (buttonBoard->GetRawButton(BUTTONS::INTAKE_UP)){
                IntakeRotationSet(-intake_speed);
            } else if (buttonBoard->GetRawButton(BUTTONS::INTAKE_DOWN)) {
                IntakeRotationSet(intake_speed);
            } else {
                IntakeRotationSet(0);
            }
            break;}
        default: // Unknown
            IntakeRotationSet(0);
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

	SmartDashboard::PutNumber("intake PID error", error);

	error /= 52000;

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