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
	navX = new AHRS(SerialPort::Port::kUSB1);

	comp = new Compressor(PORTS::COMPRESSOR);
	comp->Start();
	sol = new DoubleSolenoid{PORTS::SOL_SHIFT_IN, PORTS::SOL_SHIFT_OUT};
	gear = GEAR::STAY;
}

Driving::~Driving() {
	// TODO Auto-generated destructor stub
}

void Driving::ControllerMove(Joystick *leftJoy, Joystick *rightJoy){
	// Tank drive
	leftStick = leftJoy->GetRawAxis(1);
	rightStick = rightJoy->GetRawAxis(1);
	if (fabs(leftStick) < 0.02) leftStick = 0.0;
	if (fabs(rightStick) < 0.02) rightStick = 0.0;
	Drive(leftStick, rightStick);
	// Transmission
	if (leftJoy->GetRawButton(BUTTONS::SHIFT_DOWN)){ // Shift down
		Shift(GEAR::LOW);
	} else if (rightJoy->GetRawButton(BUTTONS::SHIFT_UP)){ // Shift up
		Shift(GEAR::HIGH);
	} else { // Off
		Shift(GEAR::STAY);
	}
}

void Driving::Drive(double left, double right){
	SmartDashboard::PutNumber("LEFT", left);
	SmartDashboard::PutNumber("RIGHT", right);

	sparkLF->Set(left);
	sparkLB->Set(left);
	sparkRF->Set(-right);
	sparkRB->Set(-right);
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
    // Shift down if we're going slow
    /*else if (vel < VEL_LOW_THRESHOLD && gear == GEAR::HIGH) {
        Shift(GEAR::LOW);
    }
    // Shift down if we're pushing (low speed, high current)
    if (ratio < VEL_JOY_THRESHOLD && gear == GEAR::HIGH){
        Shift(GEAR::LOW);
    }*/

	SmartDashboard::PutNumber("angle",navX->GetAngle());
} 