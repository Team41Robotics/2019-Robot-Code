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
#include <AHRS.h>

#define kBrushless rev::CANSparkMaxLowLevel::MotorType::kBrushless

#define VEL_HIGH_THRESHOLD 5600
#define VEL_LOW_THRESHOLD 1.00
#define VEL_JOY_THRESHOLD 1.00

using namespace frc;

class Driving
{
  public:
	Driving();
	virtual ~Driving();
	void ControllerMove(Joystick *left, Joystick *right);
	void Drive(double left, double right);
	void Shift(int g);
	void AutoShift();

  private:
	rev::CANSparkMax *sparkRF, *sparkRB, *sparkLF, *sparkLB;
	rev::CANEncoder *encRF, *encRB, *encLF, *encLB;
	Compressor *comp;
	DoubleSolenoid *sol;
	AHRS *navX;
	int gear;
	double leftStick = 0, rightStick = 0;
};

#endif /* SRC_DRIVING_H_ */

enum GEAR { LOW=1, HIGH=2, STAY=0 };