/*
 * Arm.h
 *
 *  Created on: Jan 2, 2019
 *      Author: team 41 robotics
 */

#ifndef SRC_ARM_H_
#define SRC_ARM_H_

#include <frc/WPILib.h>
#include <ctre/Phoenix.h>

#define kPercentOutput ctre::phoenix::motorcontrol::ControlMode::PercentOutput

using namespace frc;

class Arm
{
  public:
	  Arm();
	  virtual ~Arm();
    void ControllerMove(Joystick *leftJoy, Joystick *rightJoy);

  private:
	  DoubleSolenoid *clawSol, *shootSol;
    Timer timer;
    bool timing;
    TalonSRX *intake;
};

#endif /* SRC_ARM_H_ */