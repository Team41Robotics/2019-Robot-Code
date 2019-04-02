/*
 * PID.h
 *
 *  Created on: Feb 14, 2019
 *      Author: team 41 robotics
 */
#pragma once

#include <math.h>

class PID {
    public:
        PID(double _Kp, double _Ki, double _Kd, double tol);
        double GetPID(double current, double goal);
        void Reset();
    private:
        double Kp, Ki, Kd, tolerance;
        double prev_error, integral;
};