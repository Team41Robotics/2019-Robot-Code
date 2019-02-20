/*
 * Driving.cpp
 *
 *  Created on: Jan 2, 2019
 *      Author: team 41 robotics
 */

#include <Driving.h>
#include <math.h>

#define ControlMode ctre::phoenix::motorcontrol::ControlMode::PercentOutput

Driving::Driving() {

	/*
	// Init talons
	talonLF = new TalonSRX(TALON::LEFT_FRONT);
	talonLB = new TalonSRX(TALON::LEFT_BACK);
	talonRF = new TalonSRX(TALON::RIGHT_FRONT);
	talonRB = new TalonSRX(TALON::RIGHT_BACK);

	// Init encoders
	encoderL = talonLF;
	encoderR = talonRF;
	encoderL->SetStatusFramePeriod(StatusFrameEnhanced::Status_3_Quadrature,20,500);
	encoderR->SetStatusFramePeriod(StatusFrameEnhanced::Status_3_Quadrature,20,500);

	// Init navX
	gyro = new AHRS(SerialPort::Port::kUSB1);

	// Reset sensors
	ResetEncoders();
	ResetGyro();
	*/

	sparkRF = new Spark(SPARK::RIGHT_BACK);
	sparkRB = new Spark(SPARK::RIGHT_BACK);
	sparkLB = new Spark(SPARK::LEFT_BACK);
	sparkLF = new Spark(SPARK::LEFT_FRONT);
}

Driving::~Driving() {
	// TODO Auto-generated destructor stub
}

void Driving::ControllerMove(Joystick *leftJoy, Joystick *rightJoy){
	Drive(leftJoy->GetRawAxis(1), rightJoy->GetRawAxis(1));
}

void Driving::Drive(double left, double right){
	SmartDashboard::PutNumber("LEFT", left);
	SmartDashboard::PutNumber("RIGHT", right);
	/*talonLF->Set(ControlMode, left);
	talonLB->Set(ControlMode, left);
	talonRF->Set(ControlMode, -right);
	talonRB->Set(ControlMode, -right);
*/
	sparkLF->Set(ControlMode, left);
	sparkLB->Set(ControlMode, left);
	sparkRF->Set(ControlMode, -right);
	sparkRB->Set(ControlMode, -right);
}

void Driving::ResetEncoders(){
	encoderL->GetSensorCollection().SetQuadraturePosition(0,500);
	encoderR->GetSensorCollection().SetQuadraturePosition(0,500);
}

void Driving::ResetGyro(){
	gyro->Reset();
}

void Driving::UpdateIMU(){
	posL = (-encoderL->GetSensorCollection().GetQuadraturePosition() / 4096.0) * ( 2.0 * PI * WHEEL_RADIUS);
	posR = (encoderR->GetSensorCollection().GetQuadraturePosition() / 4096.0) * ( 2.0 * PI * WHEEL_RADIUS);
	angle = gyro->GetAngle();
}

bool Driving::ForwardPID(double goal) {
	// PID constants
	double kP = 1.0;
	double kI = 1.0;
	double kD = 1.0;
	double TOLERANCE = 0.25;
	double MAX_INTEGRAL = 2.0; 

	if (firstTime){
		ResetEncoders();
		posL = 0.0;
		posR = 0.0;
		integral = 0.0;
		prevError = 0.0;
		firstTime = false;
	}

	double current = (encoderL + encoderR) / 2.0;
	double error = goal - current;
	integral += error;
	if (integral > MAX_INTEGRAL) {
		integral = MAX_INTEGRAL;
	}
	else if (integral < -MAX_INTEGRAL) {
		integral = -MAX_INTEGRAL;
	}
	double derivative = error - prevError;
	prevError = error;
	double speed = (kP * error) + (kI * integral) + (kD * derivative);

	if (fabs(error) < TOLERANCE) {
		Drive(0.0, 0.0);
		firstTime = true;
		return true;
	}
	else {
		Drive(speed, speed);
	}
}

bool Driving::TurnPID(double goal) {
	// PID constants
	double kP = 1.0;
	double kI = 1.0;
	double kD = 1.0;
	double TOLERANCE = 0.25;
	double MAX_INTEGRAL = 2.0;

	if (firstTime){
		ResetGyro();
		integral = 0.0;
		prevError = 0.0;
		firstTime = false;
	}

	double current = angle;
	double error = goal - current;
	integral += error;
	if (integral > MAX_INTEGRAL) {
		integral = MAX_INTEGRAL;
	}
	else if (integral < -MAX_INTEGRAL) {
		integral = -MAX_INTEGRAL;
	}
	double derivative = error - prevError;
	prevError = error;
	double speed = (kP * error) + (kI * integral) + (kD * derivative);

	if (fabs(error) < TOLERANCE) {
		Drive(0.0, 0.0);
		firstTime = true;
		return true;
	}
	else {
		Drive(speed, -speed);
	}
}

double Driving::GetAngle()
{
	double encL = (-encoderL->GetSensorCollection().GetQuadraturePosition() / 4096.0) * 360;
	double encR = (encoderR->GetSensorCollection().GetQuadraturePosition() / 4096.0) * 360;
	double encAngle = (encR+encL)/2;
	double gyroWeight = 0.7;
	return (angle * gyroWeight + encAngle * (1-gyroWeight));
}

void Driving::AutoTrans()
{
	double encL = (-encoderL->GetSensorCollection().GetQuadraturePosition() / 4096.0)
	double encR = (encoderR->GetSensorCollection().GetQuadraturePosition() / 4096.0)
	double difference = fabs(((encL-preEncL) + (encR-preEncR))/2); //Average between the difference of encoders
	SmartDashboard::PutNumber("difference", difference);
	double prevEncL = encL;
	double prevEncR = encR;
	double shift = 1.5; //Get a real number for this at top speed(Gear 1)
	if(difference>shift)
	{
		sol->Set(DoubleSolenoid::Value::kForward);
	}
	else
	{
		sol->Set(DoubleSolenoid::Value::kReverse);
	}
}