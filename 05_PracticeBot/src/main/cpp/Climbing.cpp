/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Climbing.h"

Climbing::Climbing() {
    sol0 = new DoubleSolenoid(PORTS::CLIMB_SOL_PCM, PORTS::CLIMB_SOL0_0, PORTS::CLIMB_SOL0_1);
    sol1 = new DoubleSolenoid(PORTS::CLIMB_SOL_PCM, PORTS::CLIMB_SOL1_0, PORTS::CLIMB_SOL1_1);
    //timer = new Timer();
    // Retract climbing pistons on the first run
    timer.Reset();
    timer.Start();
    timing = true;
}

void Climbing::ControllerMove(Joystick *buttonBoard){
    if (buttonBoard->GetRawButton(BUTTONS::CLIMB_EXTEND)){ // Button is being pressed
        //timing = false;
        sol0->Set(DoubleSolenoid::Value::kForward);
        sol1->Set(DoubleSolenoid::Value::kForward);
    } else { // Button is not being pressed
        //if (timing){
            sol0->Set(DoubleSolenoid::Value::kReverse);
            sol1->Set(DoubleSolenoid::Value::kReverse);
        //     if (timer.Get() >= 0.5){
        //         timer.Stop();
        //         timer.Reset();
        //         timing = false;
        //     }
        // } else {
        //     sol0->Set(DoubleSolenoid::Value::kOff);
        //     sol1->Set(DoubleSolenoid::Value::kOff);
        // }
    }
}