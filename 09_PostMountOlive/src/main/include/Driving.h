/*
 * Driving.h
 *
 *  Created on: Jan 2, 2019
 *      Author: team 41 robotics
 */

#ifndef SRC_DRIVING_H_
#define SRC_DRIVING_H_

#include <frc/WPILib.h>
#include <rev/CANSparkMax.h>
#include <rev/CANSparkMaxLowLevel.h>
#include <rev/CANEncoder.h>
#include <rev/CANPIDController.h>
#include <AHRS.h>
#include <ctime>
#include <cmath>
#include "Arm.h"

#define kBrushless rev::CANSparkMaxLowLevel::MotorType::kBrushless

#define VEL_HIGH_THRESHOLD 5600
#define VEL_LOW_THRESHOLD 1.00
#define VEL_JOY_THRESHOLD 1.00

#define WHEEL_BASE 0.6858
#define WHEEL_RADIUS 0.1
#define GEAR_RATIO_HIGH 12.6176
#define GEAR_RATIO_LOW 32.5000

#define PI 3.141596

using namespace frc;

class Driving
{
  public:
	Driving(Arm *_arm);
	virtual ~Driving();
	void ControllerMove(Joystick *left, Joystick *right, Joystick *buttonBoard);
	void Drive(double left, double right, bool sensitive = false);
	void Shift(int g);
	void AutoShift();
	void Reset();
	void VelocityPID(double linear, double angular);
	void AssistedDrivingLite();
	void AssistedDriving();

  private:
	Arm *arm;
	rev::CANSparkMax *sparkRF, *sparkRB, *sparkLF, *sparkLB;
	rev::CANEncoder *encRF, *encRB, *encLF, *encLB;
	Compressor *comp;
	DoubleSolenoid *soll,*solr;
	AHRS *navX;
	int gear;
	double leftStick = 0, rightStick = 0;
	double prev;
	Timer timer;
	double x = 0.0, y = 0.0;
	double integrated_gyro = 0, integrated_encoder = 0;

	bool gearButtonPressed, shiftUp;
	rev::CANPIDController *pidRF, *pidRB, *pidLF, *pidLB;
	void SetPIDConstants(rev::CANPIDController *p);
	void SendOdometry();
	int assistStep;

	double prevOffset, offsetIntegral;
	double voltageL, voltageR;
	double prevDistance;
};

enum GEAR { LOW=1, HIGH=2, STAY=0 };

#endif /* SRC_DRIVING_H_ */