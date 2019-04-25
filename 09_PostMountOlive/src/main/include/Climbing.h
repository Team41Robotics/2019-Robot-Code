/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/WPILib.h>
#include "Ports.h"
#include "Driving.h"
#define EXTEND_TIME 1.5
#define DISTANCE_TO_CONTACT 40.0/39.37
#define VELOCITY (DISTANCE_TO_CONTACT/EXTEND_TIME)

using namespace frc;

class Climbing {
  public:
    Climbing(Driving *drive);
    void ControllerMove(Joystick *buttonBoard);
    void PressureSense();
  private:
    DoubleSolenoid *sol0, *sol1;
    Driving *drv;
    Timer timer;
    AnalogInput *ultra;
    AnalogInput *pressureSensor;
    bool timing;
    void Extend(bool ext);
    double GetDistance();
    bool timer_started;
};
