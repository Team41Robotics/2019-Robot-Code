/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

using namespace frc;

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  joy0 = new frc::Joystick(2);
  joy1 = new frc::Joystick(1);
  // Arm
  arm0 = new TalonSRX(10);
  arm1 = new TalonSRX(5);

  intake0 = new TalonSRX(11);
  intake1 = new TalonSRX(1);

  ultra = new AnalogInput(0);
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
  double voltage_source = 5.0;
  double scaling = voltage_source / 1024.0; // Volts per 5 mm
  double dist = 5 * (ultra->GetValue() / scaling); // Distance in mm
  dist *= 0.0393701; // Distance in inches
  SmartDashboard::PutNumber("distance", dist);
  SmartDashboard::PutNumber("ultra", ultra->GetValue());
  SmartDashboard::PutNumber("ultra scaled", ultra->GetValue() * 5.0 / 1024.0 / 1000.0 * 39.37);
}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {
//	liftBottomLimit = new DigitalInput(PORTS::LIFT_BOTTOM_LIMIT);
//	armTopLimit = new DigitalInput(PORTS::ARM_TOP_LIMIT);
//	armBottomLimit = new DigitalInput(PORTS::ARM_BOTTOM_LIMIT);

  // comp->Start();
}

void Robot::TeleopPeriodic() {
  
  // arm0->Set(kPercentOutput, -joy0->GetRawAxis(1));
  // arm1->Set(kPercentOutput, -joy1->GetRawAxis(1));
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
