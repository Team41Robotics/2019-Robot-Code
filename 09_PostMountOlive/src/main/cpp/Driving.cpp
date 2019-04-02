/*
 * Driving.cpp
 *
 *  Created on: Jan 2, 2019
 *      Author: team 41 robotics
 */

#include "Driving.h"
#include "Ports.h"

Driving::Driving(Arm *_arm) {
	arm = _arm;
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
	soll = new DoubleSolenoid(PORTS::SOL_SHIFT_L_PCM, PORTS::SOL_SHIFT_L_IN, PORTS::SOL_SHIFT_L_OUT);
	solr = new DoubleSolenoid(PORTS::SOL_SHIFT_R_PCM, PORTS::SOL_SHIFT_R_IN, PORTS::SOL_SHIFT_R_OUT);
	gear = GEAR::HIGH;

	gearButtonPressed = false;
	shiftUp = false; // Starts in low gear

	prevOffset = 0;
}

Driving::~Driving() {
	// TODO Auto-generated destructor stub
}

void Driving::Reset() {
	// navX->Reset();
	prev = 0;
	timer.Start();
	integrated_encoder = 0;
}

/**
 * Uses controller input to move the robot
 * This is called during teleop periodic
 */
void Driving::ControllerMove(Joystick *leftJoy, Joystick *rightJoy, Joystick *buttonBoard) {
	SmartDashboard::PutNumber("left velocity", encLF->GetVelocity());
	// filled pressure?
	SmartDashboard::PutBoolean("pressurized?",	comp->GetPressureSwitchValue());
	
	if (buttonBoard->GetRawButton(BUTTONS::DRIVING_ASSIST)) {
		AssistedDriving();
		SmartDashboard::PutBoolean("send_nav_goal", true);
		return;
	} else SmartDashboard::PutBoolean("send_nav_goal", false);
	if (buttonBoard->GetRawButton(BUTTONS::ASSIST_LITE)) {
		AssistedDrivingLite();
		SmartDashboard::PutBoolean("Assisted?", true);
		return;
	} 
	else SmartDashboard::PutBoolean("Assisted?", false);
    if (buttonBoard->GetPOV(0)/45 == 7){ // HAB 1 to 2 button
		return;
	}
	// Tank drive
	leftStick = -leftJoy->GetRawAxis(1);
	rightStick = -rightJoy->GetRawAxis(1);
	if (fabs(leftStick) < 0.04) leftStick = 0.0;
	if (fabs(rightStick) < 0.04) rightStick = 0.0;
	Drive(leftStick, rightStick, buttonBoard->GetRawButton(BUTTONS::DRIVER_OVERRIDE));
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
void Driving::Drive(double left, double right, bool sensitive) {

	// sparkLF->Set(left);
	// sparkLB->Set(left);
	// sparkRF->Set(-right);
	// sparkRB->Set(-right);
	if (sensitive){
		left = left*left*left;
		right = right*right*right;
	}
	sparkLF->Set(left);
	sparkLB->Set(left);
	sparkRF->Set(right);
	sparkRB->Set(right);
	voltageL = left;
	voltageR = right;
	SendOdometry();
}

/**
 * Shifts gears using the given parameter g
 */
void Driving::Shift(int g) {
	if (g == GEAR::LOW) { // Shift down
		soll->Set(DoubleSolenoid::Value::kReverse);
		solr->Set(DoubleSolenoid::Value::kReverse);
		gear = GEAR::LOW;
	} else if (g == GEAR::HIGH) { // Shift up
		soll->Set(DoubleSolenoid::Value::kForward);
		solr->Set(DoubleSolenoid::Value::kForward);
		gear = GEAR::HIGH;
	} else { // Stay
		soll->Set(DoubleSolenoid::Value::kOff);
		solr->Set(DoubleSolenoid::Value::kOff);
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

	// SmartDashboard::PutNumber("angle",navX->GetAngle());
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

	double voltage = sparkLF->GetBusVoltage();
	pidLF->SetReference(left_target, rev::ControlType::kVelocity);
	pidLB->SetReference(left_target, rev::ControlType::kVelocity);
	pidRF->SetReference(right_target, rev::ControlType::kVelocity);
	pidRB->SetReference(right_target, rev::ControlType::kVelocity);//, 0, voltage * voltageR);

	SmartDashboard::PutNumber("left_vel", encLF->GetVelocity());
	SmartDashboard::PutNumber("right_error",right_target-encRF->GetVelocity());

	SendOdometry();

}

void Driving::SendOdometry() {
	// Calculate linear and angular velocity for ROS feedback
	double ratio = (gear == GEAR::HIGH) ? GEAR_RATIO_HIGH : GEAR_RATIO_LOW;
	// integrated_encoder += encLF->GetVelocity()* (2 * PI * WHEEL_RADIUS) / 60.0 / ratio *.02;
	// Prateek, shouldn't we average the left and right velocity for this in order to get the linear velocity?

	double velLeft = encLF->GetVelocity() * (2 * PI * WHEEL_RADIUS) / 60.0 / ratio;
	double velRight = encRF->GetVelocity() * (2 * PI * WHEEL_RADIUS) / 60.0 / ratio;
	double linVel = (velLeft + velRight) / 2.0;
	double angVel = (velRight - velLeft) / WHEEL_BASE;

	//integrated_encoder += angVel*timer.Get();

	double gyro = navX->GetYaw() * PI/180;

	x += linVel*timer.Get()*cos(-gyro);
	y += linVel*timer.Get()*sin(-gyro);
	
	SmartDashboard::PutNumber("linear_velocity", linVel);
	SmartDashboard::PutNumber("angle", -gyro);
	SmartDashboard::PutNumber("dt", timer.Get());
	SmartDashboard::PutNumber("angular_velocity", -navX->GetRate());
	SmartDashboard::PutNumber("x", x);
	SmartDashboard::PutNumber("y", y);

	//SmartDashboard::PutNumber("positionLF", encLF->GetPosition()/ratio*2*PI*WHEEL_RADIUS);
	//SmartDashboard::PutNumber("position RF", encRF->GetPosition()/ratio*2*PI*WHEEL_RADIUS);

	// SmartDashboard::PutNumber("angular_velocity_encoder", angVel);
	// SmartDashboard::PutNumber("linear_velocity_gyro", navX->GetVelocityX());
	// SmartDashboard::PutNumber("left encoder", encLF->GetVelocity());
	// SmartDashboard::PutNumber("right encoder", encRF ->GetVelocity());

	// integrated_gyro += navX->GetRate()*timer.Get();
	// SmartDashboard::PutNumber("integrated angular_velocity", integrated_encoder*180/PI);
	// SmartDashboard::PutNumber("integrated angular_velocity_gyro", integrated_gyro*180/PI);
	// SmartDashboard::PutNumber("integrated linvel", integrated_encoder);
	integrated_encoder += linVel *timer.Get();
	// SmartDashboard::PutNumber("x integrated", integrated_encoder);


	timer.Reset();
}

/* void Driving::AssistedDriving(){
	if (!SmartDashboard::GetBoolean("send_nav_goal", false)) {
		assistStep = 0;
		prevOffset = 0;
		offsetIntegral = 0;
	}
	VelocityPID(
		SmartDashboard::GetNumber("target_linear_velocity", 0),
		SmartDashboard::GetNumber("target_angular_velocity", 0)
	);
} */

// with initial align 
void Driving::AssistedDrivingLite() {
	double offset = SmartDashboard::GetNumber("horizontal_offset_normalized", 0.0);
	double angle = SmartDashboard::GetNumber("Angle of Line", 0.0); // In degrees
	double distance = SmartDashboard::GetNumber("Distance to Line", 0.0) / cos(angle * PI / 180.0);
	distance -= 0.7366;
	if (!SmartDashboard::GetBoolean("Assisted?", false)) {
		assistStep = 0;
		prevOffset = 0;
		offsetIntegral = 0;
		prevDistance = distance; 
	}
	voltageL = 0;
	voltageR = 0;
	SmartDashboard::PutNumber("Assist Step", assistStep);
	if (assistStep >= 2) {
		Drive(0.0,0.0);
		if (assistStep <= 27){
			// arm->Grab();
			assistStep += 1;
		}
		return;
	}
	voltageL *= .9;
	voltageR *= .9;
	double currSpeed = SmartDashboard::GetNumber("linear_velocity", 0);
	currSpeed = currSpeed == 0 ? 0.1 : currSpeed;
	if (distance - prevDistance > 0.6){
		distance = prevDistance + (distance - prevDistance)*0.2;
	}
	prevDistance = distance;
	double minDistance = currSpeed * 0.65; // 0.12;
	double angSpeed = 0;
	double linSpeed = 0;
	if (assistStep == 0 || distance >= .07) {
		if (fabs(offset) > 0.05) {
			double kP = 0.4;
			double kI = 0.06;
			double kD = 0.6;
			if (assistStep == 1){
				kP = 2; // 2
				kI = 0;
				kD = 1;
			}
			offsetIntegral += offset;
			angSpeed = kP * offset + kI * offsetIntegral + kD * (offset - prevOffset);
			prevOffset = offset;
		}
	}
	SmartDashboard::PutNumber("rotational velcoity lite", angSpeed);
	if (assistStep == 1) {
		if (distance > minDistance || true) {
			double kP_lin = 0.65;
			double max_speed = 1;
			linSpeed = kP_lin * distance * distance;
			if (linSpeed > max_speed) linSpeed = max_speed;
		} else {
			assistStep++;
			// Stop! Quickly!
			double kP_stop = 0.1;
			double lSpeed = encLF->GetVelocity();
			double rSpeed = encRF->GetVelocity();
			Drive(-kP_stop * lSpeed, -kP_stop * rSpeed);
			return;
		}
	}
	if(assistStep == 0 && fabs(offset) < 1) {
		assistStep++;
		//Drive(0,0);
		prevOffset = 0;
		offsetIntegral = 0;
		return;
	}
	VelocityPID(linSpeed, -angSpeed);
}


// with initial align 
/* void Driving::AssistedDriving() {
	if (!SmartDashboard::GetBoolean("Assisted?", false)) {
		assistStep = 0;
		prevOffset = 0;
		offsetIntegral = 0;
	}
	SmartDashboard::PutNumber("Assist Step", assistStep);
	
	double offset = SmartDashboard::GetNumber("horizontal_offset_normalized", 0.0);
	double angle = SmartDashboard::GetNumber("Angle of Line", 0.0); // In degrees
	double distance = SmartDashboard::GetNumber("Distance to Line", 0.0) / cos(angle * PI / 180.0);
	distance -= 29.0 / 39.37;
	double minDistance = 0.04;
	double angSpeed = 0;
	double linSpeed = 0;
	if (assistStep == 0 || distance >= 1) {
		if (fabs(offset) > 0.05) {
			double kP = 0.4;
			double kI = 0.06;
			double kD = 0.6;
			if (assistStep == 1){
				kP = 0.4;
				kI = 0;
				kD = 10.0;
			}
			offsetIntegral += offset;
			angSpeed = kP * offset + kI * offsetIntegral + kD * prevOffset;
			prevOffset = offset;
		}
	}
	if (assistStep == 1) {
		if (distance > minDistance) {
			double kP_lin = 1.5;
			double max_speed = 1.0;
			linSpeed = kP_lin * distance * distance;
			if (linSpeed > max_speed) linSpeed = max_speed;
		} else {
			assistStep++;
			Drive(0, 0);
			return;;
		}
	}
	if(assistStep == 0 && fabs(offset) < .05) {
		assistStep++;
		Drive(0,0);
		prevOffset = 0;
		offsetIntegral = 0;
		return;
	}
	VelocityPID(linSpeed, -angSpeed);
} */

void Driving::AssistedDriving(){
	// Source: Hadzic math
	double x = SmartDashboard::GetNumber("base_link_y", 0); // From camera, intentionally swapped for ROS
	double y = SmartDashboard::GetNumber("base_link_x", 0) - 10/39.37 + 0.04044;
	double theta = SmartDashboard::GetNumber("Angle of Line", 0); // From lidar, angle for rotation
	double d = sqrt(x*x + y*y); // Distance to hatch
	double phi = atan(y/x); // Angle for translation
	double R = d / 2.0 / cos(theta + phi); // Turn radius
	double v = 0.5; // Linear speed is constant
	double omega = v / R; // Angular speed is dependent on linear speed
	VelocityPID(v, omega); // Go time!
}