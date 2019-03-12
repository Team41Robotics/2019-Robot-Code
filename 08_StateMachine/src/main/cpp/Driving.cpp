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
	sparkRF->SetInverted(true);
	sparkRB->SetInverted(true);

	// Init encoders
	encLF = new rev::CANEncoder(sparkLF->GetEncoder());
	encLB = new rev::CANEncoder(sparkLB->GetEncoder());
	encRF = new rev::CANEncoder(sparkRF->GetEncoder());
	encRB = new rev::CANEncoder(sparkRB->GetEncoder());

	pidLF = new rev::CANPIDController((*sparkLF));
	pidLB = new rev::CANPIDController((*sparkLB));
	pidRF = new rev::CANPIDController((*sparkRF));
	pidRB = new rev::CANPIDController((*sparkRB));

	SetPIDConstants(pidLF);
	SetPIDConstants(pidLB);
	SetPIDConstants(pidRF);
	SetPIDConstants(pidRB);

	// Init gyro
	navX = new AHRS(SerialPort::Port::kUSB1);

	comp = new Compressor(PORTS::COMPRESSOR);
	comp->Start();
	// comp->Stop();
	sol = new DoubleSolenoid(PORTS::SOL_SHIFT_PCM, PORTS::SOL_SHIFT_IN, PORTS::SOL_SHIFT_OUT);
	gear = GEAR::HIGH;

	gearButtonPressed = false;
	shiftUp = false; // Starts in low gear
}

Driving::~Driving() {
	// TODO Auto-generated destructor stub
}

void Driving::Reset() {
	navX->Reset();
	prev = 0;
	timer.Start();
	integrated_encoder = 0;
}

/**
 * Uses controller input to move the robot
 * This is called during teleop periodic
 */
void Driving::ControllerMove(Joystick *leftJoy, Joystick *rightJoy, Joystick *buttonBoard) {
	if (buttonBoard->GetRawButton(BUTTONS::DRIVING_ASSIST)){
		VelocityPID(
			SmartDashboard::GetNumber("target_linear_velocity", 0),
			SmartDashboard::GetNumber("target_angular_velocity", 0)
		);
		return;
	}
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
		//Shift(GEAR::STAY);
		gearButtonPressed = false;
	}
	if (buttonBoard->GetRawButton(BUTTONS::DRIVER_OVERRIDE)) {
		if (buttonBoard->GetRawButton(BUTTONS::SHIFT_UP)) Shift(GEAR::HIGH);
		else if (buttonBoard->GetRawButton(BUTTONS::SHIFT_DOWN)) Shift(GEAR::LOW);
	}
}

/**
 * Sets the motor controllers to move the robot
 */
void Driving::Drive(double left, double right) {

	// sparkLF->Set(left);
	// sparkLB->Set(left);
	// sparkRF->Set(-right);
	// sparkRB->Set(-right);
	sparkLF->Set(/* gear == GEAR::HIGH ? left*left*left : */ left);
	sparkLB->Set(/* gear == GEAR::HIGH ? left*left*left : */ left);
	sparkRF->Set(/* gear == GEAR::HIGH ? -right*right*right : */ right);
	sparkRB->Set(/* gear == GEAR::HIGH ? -right*right*right : */ right);
	// Calculate linear and angular velocity for ROS feedback
	double ratio = (gear == GEAR::HIGH) ? GEAR_RATIO_HIGH : GEAR_RATIO_LOW;
	double velLeft = encLF->GetVelocity() * (2 * PI * WHEEL_RADIUS) / 60.0 / ratio;
	double velRight = encRF->GetVelocity() * (2 * PI * WHEEL_RADIUS) / 60.0 / ratio;
	double linVel = (velLeft + velRight) / 2.0;
	double angVel = (velRight - velLeft) / WHEEL_BASE;

	//integrated_encoder += angVel*timer.Get();

	double gyro = navX->GetYaw() * PI/180;

	integrated_encoder += linVel *timer.Get();

	SmartDashboard::PutNumber("linear_velocity", linVel);
	// SmartDashboard::PutNumber("angular_velocity_encoder", angVel);
	SmartDashboard::PutNumber("angle", gyro);
	SmartDashboard::PutNumber("dt", timer.Get());

	//SmartDashboard::PutNumber("positionLF", encLF->GetPosition()/ratio*2*PI*WHEEL_RADIUS);
	//SmartDashboard::PutNumber("position RF", encRF->GetPosition()/ratio*2*PI*WHEEL_RADIUS);

	// SmartDashboard::PutNumber("linear_velocity_gyro", navX->GetVelocityX());
	SmartDashboard::PutNumber("angular_velocity", navX->GetRate());
	// SmartDashboard::PutNumber("left encoder", encLF->GetVelocity());
	// SmartDashboard::PutNumber("right encoder", encRF ->GetVelocity());

	// integrated_gyro += navX->GetRate()*timer.Get();
	// SmartDashboard::PutNumber("integrated angular_velocity", integrated_encoder*180/PI);
	// SmartDashboard::PutNumber("integrated angular_velocity_gyro", integrated_gyro*180/PI);
	SmartDashboard::PutNumber("integrated linvel", integrated_encoder);

	timer.Reset();
}

/**
 * Shifts gears using the given parameter g
 */
void Driving::Shift(int g) {
	if (g == GEAR::LOW) { // Shift down
		sol->Set(DoubleSolenoid::Value::kReverse);
		gear = GEAR::LOW;
	} else if (g == GEAR::HIGH) { // Shift up
		sol->Set(DoubleSolenoid::Value::kForward);
		gear = GEAR::HIGH;
	} else { // Stay
		sol->Set(DoubleSolenoid::Value::kOff);
	}
}


void Driving::AutoShift() {
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

void Driving::SetPIDConstants(rev::CANPIDController *p) {
	// Default PID coefficients
	p->SetP(4e-8);//5e-5//4e-8
	p->SetI(2.5e-7);//1e-6
	p->SetD(4e-9);
	p->SetIZone(0);
	p->SetFF(0.0);//0.000156
	p->SetOutputRange(-1, 1);
}

void Driving::VelocityPID(double linear, double angular) { // speed in m/s and rad/s
	double ratio = (gear == GEAR::HIGH) ? GEAR_RATIO_HIGH : GEAR_RATIO_LOW;
	double left_target = linear - angular * WHEEL_BASE / 2.0;
	double right_target = linear + angular * WHEEL_BASE / 2.0;
	left_target *= 1.0 / (2 * PI * WHEEL_RADIUS) * ratio * 60.0;
	right_target *= 1.0 / (2 * PI * WHEEL_RADIUS) * ratio * 60.0; // Convert robot m/s to motor rpm
	SmartDashboard::PutNumber("left_target",left_target);
	SmartDashboard::PutNumber("right_target",right_target);

	pidLF->SetReference(left_target, rev::ControlType::kVelocity);
	pidLB->SetReference(left_target, rev::ControlType::kVelocity);
	pidRF->SetReference(right_target, rev::ControlType::kVelocity);
	pidRB->SetReference(right_target, rev::ControlType::kVelocity);

	SmartDashboard::PutNumber("left_vel", encLF->GetVelocity());
	SmartDashboard::PutNumber("right_error",right_target-encRF->GetVelocity());

	integrated_encoder += encLF->GetVelocity()* (2 * PI * WHEEL_RADIUS) / 60.0 / ratio *.02;

	SmartDashboard::PutNumber("x integrated", integrated_encoder);

}