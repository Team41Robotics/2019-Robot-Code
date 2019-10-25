/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  fl = new rev::CANSparkMax(3, kBrushless);
  bl = new rev::CANSparkMax(2, kBrushless);
  fr = new rev::CANSparkMax(4, kBrushless);
  br = new rev::CANSparkMax(1, kBrushless);
  sol = new DoubleSolenoid(0, 1);
  comp = new Compressor(0);
  joy0 = new Joystick(1);
  joy1 = new Joystick(2);
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

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
  comp->Start();
}

void Robot::TeleopPeriodic() {
  double left = joy0->GetRawAxis(1);
  double right = -joy1->GetRawAxis(1);
  if (fabs(left) < 0.08) left = 0;
  if (fabs(right) < 0.08) right = 0; 
  fl->Set(left);
  bl->Set(left);
  fr->Set(right);
  br->Set(right);
  if (joy0->GetRawButton(1)) { // Shift down
		sol->Set(DoubleSolenoid::Value::kReverse);
	} else if (joy1->GetRawButton(1)) { // Shift up
		sol->Set(DoubleSolenoid::Value::kForward);
	}
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
