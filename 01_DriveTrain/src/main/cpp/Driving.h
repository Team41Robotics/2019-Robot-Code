/*
 * Driving.h
 *
 *  Created on: Jan 2, 2019
 *      Author: team 41 robotics
 */

#ifndef SRC_DRIVING_H_
#define SRC_DRIVING_H_

#include <WPILib.h>

#include "ctre/Phoenix.h"

#define PI 3.1415926
#define WHEEL_RADIUS (5.75/2.0)
#define WHEELBASE 22.5

class Driving {
public:
	Driving();
	virtual ~Driving();
	void ControllerMove(Joystick *left, Joystick *right);
private:
	TalonSRX *talonRF, *talonRB, *talonLF, *talonLB, *encoderL, *encoderR;
	Spark *sparkRF, *sparkRB, *sparkLF, *sparkLB; 
	double posL, posR, angle;
	bool firstTime = true;
	double prevError, integral;
	void Drive(double left, double right);
	void ResetEncoders();
	void ResetGyro();
	void UpdateIMU();
	bool ForwardPID(double goal);
};

#endif /* SRC_DRIVING_H_ */

enum TALON {
	LEFT_FRONT = 11, // 11, labeled 1
	LEFT_BACK = 9, // 9, labeled 2
	RIGHT_FRONT = 8, // 8, labeled 3
	RIGHT_BACK = 3 // 3, labeled 4
};
enum SPARK {
	LEFT_FRONT = 1,
	LEFT_BACK = 2,
	RIGHT_FRONT = 3, 
	RIGHT_BACK = 4 
};