/*
 * Driving.cpp
 *
 *  Created on: Jan 2, 2019
 *      Author: team 41 robotics
 */

#include "Driving.h"
#include "Ports.h"

Driving::Driving() {

	// Init motor controllers
	sparkLF = new rev::CANSparkMax(PORTS::SPARK_LF, kBrushless);
	sparkLB = new rev::CANSparkMax(PORTS::SPARK_LB, kBrushless);
	sparkRF = new rev::CANSparkMax(PORTS::SPARK_RF, kBrushless);
	sparkRB = new rev::CANSparkMax(PORTS::SPARK_RB, kBrushless);

	// Init encoders
	encLF = new rev::CANEncoder(sparkLF->GetEncoder());
	encLB = new rev::CANEncoder(sparkLB->GetEncoder());
	encRF = new rev::CANEncoder(sparkRF->GetEncoder());
	encRB = new rev::CANEncoder(sparkRB->GetEncoder());

	// Init gyro
	//navX = new AHRS(SerialPort::Port::kUSB1);

	comp = new Compressor(PORTS::COMPRESSOR);
	comp->Start();
	sol = new DoubleSolenoid(PORTS::SOL_SHIFT_PCM, PORTS::SOL_SHIFT_IN, PORTS::SOL_SHIFT_OUT);
	gear = GEAR::STAY;

	gearButtonPressed = false;
	shiftUp = false; // Starts in low gear
}

Driving::~Driving() {
	// TODO Auto-generated destructor stub
}

void Driving::ControllerMove(Joystick *leftJoy, Joystick *rightJoy, Joystick *buttonBoard){
	// Tank drive
	leftStick = -leftJoy->GetRawAxis(1);
	rightStick = -rightJoy->GetRawAxis(1);
	if (fabs(leftStick) < 0.04) leftStick = 0.0;
	if (fabs(rightStick) < 0.04) rightStick = 0.0;
	Drive(leftStick, rightStick);
	// Transmission
	if (leftJoy->GetRawButton(BUTTONS::TOGGLE_SHIFT)) {
		if(!gearButtonPressed) {
			shiftUp = !shiftUp;
			gearButtonPressed = true;
		}
		Shift(shiftUp ? GEAR::HIGH : GEAR::LOW);
	} else { // Off
		Shift(GEAR::STAY);
		gearButtonPressed = false;
	}
	if (buttonBoard->GetRawButton(BUTTONS::DRIVER_OVERRIDE)){
		if (buttonBoard->GetRawButton(BUTTONS::SHIFT_UP)) Shift(GEAR::HIGH);
		else if (buttonBoard->GetRawButton(BUTTONS::SHIFT_DOWN)) Shift(GEAR::LOW);
	}
}

void Driving::Drive(double left, double right){

	// sparkLF->Set(left);
	// sparkLB->Set(left);
	// sparkRF->Set(-right);
	// sparkRB->Set(-right);
	sparkLF->Set(/* gear == GEAR::HIGH ? left*left*left : */ left);
	sparkLB->Set(/* gear == GEAR::HIGH ? left*left*left : */ left);
	sparkRF->Set(/* gear == GEAR::HIGH ? -right*right*right : */ -right);
	sparkRB->Set(/* gear == GEAR::HIGH ? -right*right*right : */ -right);
	// Calculate linear and angular velocity for ROS feedback
	double velLeft = encLF->GetVelocity();
	double velRight = -encRF->GetVelocity();
	double linVel = (velLeft + velRight) / 2.0;
	double angVel = (velRight - velLeft) / WHEEL_BASE;
	// SmartDashboard::PutNumber("linear velocity encoder", linVel);
	// SmartDashboard::PutNumber("angular velocity encoder", angVel);
	// SmartDashboard::PutNumber("linear velocity gyro", navX->GetVelocityX());
	// double current = navX->GetAngle();
	// SmartDashboard::PutNumber("angular velocity gyro", current-prev);
	// prev = current;
}

void Driving::Shift(int g){
	if (g == GEAR::LOW){ // Shift down
		sol->Set(DoubleSolenoid::Value::kReverse);
		gear = GEAR::LOW;
	} else if (g == GEAR::HIGH){ // Shift up
		sol->Set(DoubleSolenoid::Value::kForward);
		gear = GEAR::HIGH;
	} else { // Stay
		sol->Set(DoubleSolenoid::Value::kOff);
	}
}

void Driving::AutoShift(){
	// First, let's figure out the typical ratio of velocity to joystick value
	double speeds[] = {encLB->GetVelocity(), encLF->GetVelocity(), encRB->GetVelocity(), encRF->GetVelocity()};
	double sum = 0.0;
	double len = sizeof(speeds)/sizeof(speeds[0]);
	for (int i = 0; i < len; i++) sum += speeds[i];
	double speed = sum / len;
	SmartDashboard::PutNumberArray("Speeds", speeds);
	SmartDashboard::PutNumber("Average Speed", speed);
	double joy = (fabs(leftStick) + fabs(rightStick)) / 2.0;
	SmartDashboard::PutNumber("Average Joy", joy);
	double ratio = (joy == 0) ? 0 : speed / joy;
	SmartDashboard::PutNumber("Speed-Joy", ratio);

    // Shift up if we're going fast
    if (speed > VEL_HIGH_THRESHOLD && gear == GEAR::LOW) {
        Shift(GEAR::HIGH);
    }

	SmartDashboard::PutNumber("angle",navX->GetAngle());
} 