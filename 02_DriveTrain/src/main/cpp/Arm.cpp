/*
 * Arm.cpp
 *
 *  Created on: Jan 2, 2019
 *      Author: team 41 robotics
 */

#include "Arm.h"
#include "Ports.h"

Arm::Arm() {
	clawSol = new DoubleSolenoid{PORTS::SOL_CLAW_IN,PORTS::SOL_CLAW_OUT};
    shootSol = new DoubleSolenoid{PORTS::SOL_SHOOT_IN,PORTS::SOL_SHIFT_OUT};
    intake = new TalonSRX(PORTS::TALON_INTAKE); 
}

Arm::~Arm() {
	// TODO Auto-generated destructor stub
}

void Arm::ControllerMove(Joystick *leftJoy, Joystick *rightJoy){
    // Open and close claw
    if (leftJoy->GetRawButton(BUTTONS::CLAW_OPEN)) clawSol->Set(DoubleSolenoid::Value::kForward);
    else if (leftJoy->GetRawButton(BUTTONS::CLAW_CLOSE)) clawSol->Set(DoubleSolenoid::Value::kReverse);
    else clawSol->Set(DoubleSolenoid::Value::kOff);

    // Fire shooting piston
    if (rightJoy->GetRawButton(BUTTONS::SHOOT_BALL)){
        shootSol->Set(DoubleSolenoid::Value::kForward);
        timer.Reset();
        timer.Start();

        timing = true;
    }
    else if (timing && timer.Get() >= 0.5){ // Make sure to retract piston 0.5 sec after firing
        shootSol->Set(DoubleSolenoid::Value::kReverse);
        if (timer.Get() >= 0.52){
            timer.Stop();
            timing = false;
        }
    }
    else {
        shootSol->Set(DoubleSolenoid::Value::kOff);
    }
    //Sucking the ball
    if (right->GetRawButton(BUTTONS::SUCK_IN)){
        intake->Set(kPercentOutput, 1.0);
    }
    else if (right->GetRawButton(5)){
        intake->Set(kPercentOutput, -1.0);
    } 
    else {
        intake->Set(kPercentOutput, 0);
    }
}
