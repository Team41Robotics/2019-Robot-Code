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
#include <rev/CANEncoder.h>

using namespace frc;

class Driving
{
  public:
	Driving();
	virtual ~Driving();
	void ControllerMove(Joystick *left, Joystick *right);
	void Drive(double left, double right);
	void Shift(int s);
	void AutoShift();

  private:
	//Spark *sparkRF, *sparkRB, *sparkLF, *sparkLB;
	rev::CANSparkMax *sparkRF, *sparkRB, sparkLF, sparkLB;
	rev::CANEncoder *encLeft, *encRight;
	Compressor *comp;
	DoubleSolenoid *sol;
	int gear;
};

#endif /* SRC_DRIVING_H_ */

enum SPARK
{
	LEFT_FRONT = 2,  // 11, labeled 1
	LEFT_BACK = 3,   // 9, labeled 2
	RIGHT_FRONT = 0, // 8, labeled 3
	RIGHT_BACK = 1   // 3, labeled 4
};
