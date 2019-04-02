/*
 * PID.cpp
 *
 *  Created on: Feb 14, 2019
 *      Author: team 41 robotics
 */

#include "PID.h"

PID::PID(double _Kp, double _Ki, double _Kd, double tol) {
    Kp = _Kp;
    Ki = _Ki;
    Kd = _Kd;
    tolerance = tol;
    prev_error = 0;
    integral = 0;
}

void PID::Reset() {
    prev_error = 0;
    integral = 0;
}

double PID::GetPID(double current, double goal) {
    double error = goal - current;
    if (fabs(error) < tolerance) {
		return -1;
	}
    double derivative = error - prev_error;
    integral += error;
    double speed = Kp * error + Ki * integral + Kd * derivative;
    prev_error = error;
    return speed;
}