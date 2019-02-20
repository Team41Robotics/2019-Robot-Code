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
  arm0 = new TalonSRX(7);
  arm1 = new TalonSRX(3);
  SmartDashboard::PutNumber("arm speed",0.5);
  // enc7 = new Encoder(0, 1, false, Encoder::EncodingType::k4X);
  // enc7->Reset();
  // enc3 = new Encoder(0, 1, false, Encoder::EncodingType::k4X);
  // enc3->Reset();

  // Lift
  lift0 = new TalonSRX(10);
  lift1 = new TalonSRX(5);
  liftTopLimit = new DigitalInput(0);
  //SmartDashboard::PutNumber("lift speed",0.5);
  // enc10 = new Encoder(0, 1, false, Encoder::EncodingType::k4X);
  // enc10->Reset();
  // enc1 = new Encoder(0, 1, false, Encoder::EncodingType::k4X);
  // enc1->Reset();

/*
  // Intake
  intake0 = new TalonSRX(11);
  intake1 = new TalonSRX(1);
  SmartDashboard::PutNumber("intake rotation speed", 0.5);
  intake = new TalonSRX(2);

  comp = new Compressor(0);
  comp->Start();

  shootSol = new DoubleSolenoid(0,0,1);
  clawSol = new DoubleSolenoid(1,6,7); */

  // Vision assist
  lift_servo = new Servo(1);
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
//	liftBottomLimit = new DigitalInput(PORTS::LIFT_BOTTOM_LIMIT);
//	armTopLimit = new DigitalInput(PORTS::ARM_TOP_LIMIT);
//	armBottomLimit = new DigitalInput(PORTS::ARM_BOTTOM_LIMIT);

  // comp->Start();
}

void Robot::TeleopPeriodic() {

  //test limit switches
  SmartDashboard::PutNumber("lift top limit", liftTopLimit->Get());
	//SmartDashboard::PutNumber("lift bot limit", liftBottomLimit->Get());
	//SmartDashboard::PutNumber("arm top limit", armBottomLimit->Get());
	//SmartDashboard::PutNumber("arm bot limit", armTopLimit->Get());

  //SmartDashboard::PutNumber("joystick axis value", joy0->GetRawAxis(1));
  // Arm
  double arm_speed = SmartDashboard::GetNumber("arm speed",0.0);
  /*if (joy0->GetRawButton(3)){ // Up
    arm_speed *= -1;
  } else if (joy0->GetRawButton(2)){ // Down
    arm_speed *= 0.5;
  } else {
    arm_speed = -0.1;
  }*/
  arm_speed = joy0->GetRawAxis(0);

  //if (joy0->GetRawButton(4)) arm_speed = -0.1;
  // arm0->Set(kPercentOutput, arm_speed);
  // arm1->Set(kPercentOutput, arm_speed);
  // SmartDashboard::PutNumber("encoder 7", enc7->Get());
  // SmartDashboard::PutNumber("encoder 3", enc3->Get());
  
  // Lift
  double lift_speed = SmartDashboard::GetNumber("lift speed", 0.0);
  /*if (joy0->GetRawButton(11)){ // Up
    lift_speed *= -1;
  } else if (joy0->GetRawButton(10)){ // Down
    lift_speed *= 0.25;
  } else {
    lift_speed = 0;
  }*/
  lift_speed = joy0->GetRawAxis(1);
  lift0->Set(kPercentOutput, lift_speed);
  lift1->Set(kPercentOutput, lift_speed);

  /*
  // Intake
  double intake_speed = SmartDashboard::GetNumber("intake rotation speed", 0.0);
  if (joy0->GetRawButton(6)){ // Up
    intake_speed *= 1;
  } else if (joy0->GetRawButton(7)){ // Down
    intake_speed *= -1;
  } else {
    intake_speed = 0;
  }
  intake0->Set(kPercentOutput, intake_speed);
  intake1->Set(kPercentOutput, intake_speed);

  //if (joy0->GetRawButton(5)) intake->Set(kPercentOutput, 1.0);
  //else if (joy0->GetRawButton(4)) intake->Set(kPercentOutput, -1.0);
  //else intake->Set(kPercentOutput, 0.0);

  // Shoot
  if (joy0->GetRawButton(9)) shootSol->Set(DoubleSolenoid::Value::kForward);
  else if (joy0->GetRawButton(8)) shootSol->Set(DoubleSolenoid::Value::kReverse);
  else shootSol->Set(DoubleSolenoid::Value::kOff);

  // Open claw
  if (joy0->GetRawButton(5)) clawSol->Set(DoubleSolenoid::Value::kForward);
  else if (joy0->GetRawButton(4)) clawSol->Set(DoubleSolenoid::Value::kReverse);
  else clawSol->Set(DoubleSolenoid::Value::kOff); 
  intake0->Set(kPercentOutput, joy0->GetRawAxis(1));
  intake1->Set(kPercentOutput, joy0->GetRawAxis(1));*/

  if (joy1->GetRawButton(1)) lift_servo->Set(0.36);
  else lift_servo->Set(0);
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
