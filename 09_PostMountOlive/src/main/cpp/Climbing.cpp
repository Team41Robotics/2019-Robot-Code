/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Climbing.h"

Climbing::Climbing(Driving *drive) {
    sol0 = new DoubleSolenoid(PORTS::CLIMB_SOL_PCM, PORTS::CLIMB_SOL0_0, PORTS::CLIMB_SOL0_1);
    sol1 = new DoubleSolenoid(PORTS::CLIMB_SOL_PCM, PORTS::CLIMB_SOL1_0, PORTS::CLIMB_SOL1_1);
    // sol1 = new DoubleSolenoid(1, 4, 5);
    //timer = new Timer();
    // Retract climbing pistons on the first run
    timer.Stop();
    timer.Reset();
    timing = true;
    ultra = new AnalogInput(PORTS::ULTRASONIC);
    pressureSensor = new AnalogInput(PORTS::PRESSURE_SENSOR);
    drv = drive;

    timer_started = false;
}

void Climbing::Extend(bool ext){
    if (ext){
        sol0->Set(DoubleSolenoid::Value::kForward); // Extend
        sol1->Set(DoubleSolenoid::Value::kReverse);
    } else {
        sol0->Set(DoubleSolenoid::Value::kReverse); // Retract
        sol1->Set(DoubleSolenoid::Value::kForward);
    }
}

void Climbing::PressureSense(){
    // Read pressure sensor
    double supply_voltage_normalized = 2.623290747 / (0.004 * 115 + 0.1);
    double voltage = pressureSensor->GetVoltage();
    SmartDashboard::PutNumber("Pressure Sensor Voltage", voltage);
    double pressure = 250.0 * voltage / supply_voltage_normalized - 25;
    // pressure = ((int)(pressure * 100)) / 100.0;
    SmartDashboard::PutNumber("Pressure Sensor (PSI)", pressure);
    if (pressure > 60){
        SmartDashboard::PutBoolean("Pressure Above 60 PSI", true);
    } else SmartDashboard::PutBoolean("Pressure Above 60 PSI", false);
}

void Climbing::ControllerMove(Joystick *buttonBoard){

    if (buttonBoard->GetPOV(0)/45 == 5){ // HAB 1 to 2 button
        SmartDashboard::PutNumber("time elapsed", timer.Get());
        SmartDashboard::PutNumber("ULTRASONIC", GetDistance());
        if(!timer_started) {
            timer.Reset();
            timer.Start();
            timer_started = true;
        }
        if(timer.Get() > EXTEND_TIME) {
            drv->Drive(0.0, 0.0);
            Extend(false);
        }
        else {  
            drv->VelocityPID(VELOCITY,0.0);
            Extend(true);
        }
        // Check that we are not extended past 7 inches
        //if (GetDistance() >= 7) Extend(false);
        //else Extend(true);

    }
    /*else if (buttonBoard->GetPOV(0)/45 == 6){ // HAB 2 to 3 button

    }*/
    else if (buttonBoard->GetRawButton(BUTTONS::CLIMB_EXTEND)){ // Button is being pressed
        Extend(true);       
    }
    else { // Normal teleop, probably won't be used
        Extend(false);
        if (timer_started){
            timer.Stop();
            timer_started = false;
        }
    }

}

double Climbing::GetDistance(){
    double voltage_source = 5.0;
    double scaling = voltage_source / 1024.0; // Volts per 5 mm
    double dist = 5 * (ultra->GetValue() / scaling); // Distance in mm
    return dist * 0.0393701; // Distance in inches
}