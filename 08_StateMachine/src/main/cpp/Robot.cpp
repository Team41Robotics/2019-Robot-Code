/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <frc/SmartDashboard/SmartDashboard.h>

/**
 * Initializes functions and variables for the robot
 */
void Robot::RobotInit() {
	// m_chooser.AddDefault(kAutoNameDefault, kAutoNameDefault);
	// m_chooser.AddObject(kAutoNameCustom, kAutoNameCustom);
	// frc::SmartDashboard::PutDa	ta("Auto Modes", &m_chooser);
	m_chooser.AddDefault("Yes zero","zero");
	m_chooser.AddObject("No zero","notzero");
	frc::SmartDashboard::PutData("Zeroing at Init", &m_chooser);
	// Joystick inputs
	leftJoy = new Joystick(1);
	rightJoy = new Joystick(2);
	buttonBoard = new Joystick(3);
	// Helper classes
	drv = new Driving();
	arm = new Arm();
	intake = new Intake();
	viz = new Vision();
	climb = new Climbing(drv);
	pos = new PosControl(arm, intake);
}

/**
 * Called at the beginning of the autonomous period
 * 
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
	// m_autoSelected = SmartDashboard::GetString(
	// 		"Auto Selector", kAutoNameDefault);

	if (m_autoSelected == kAutoNameCustom) {
		// Custom Auto goes here
	} else {
		// Default Auto goes here
	}
}

/**
 * Called periodically during the autonomous period
 */
void Robot::AutonomousPeriodic() {
	if (m_autoSelected == kAutoNameCustom) {
		// Custom Auto goes here
	} else {
		// Default Auto goes here
	}
}

/**
 * Called at the beginning of the teleop (manual control) period
 */
void Robot::TeleopInit() {
	drv->Reset();
	if (m_chooser.GetSelected().compare("zero") == 0){
		intake->Reset();
		arm->Reset();
		pos->Reset();
	} else {
		intake->intakeZeroed = true;
		arm->armZeroed = true;
		arm->liftZeroed = true;
	}
}

/**
 * Called periodically during the telop (manual control) period
 */
void Robot::TeleopPeriodic() {
	// Driving
	drv->ControllerMove(leftJoy, rightJoy, buttonBoard);
	//drv->AutoShift();
	// Vision
	viz->Swivel();
	viz->Lift(buttonBoard);
	// Climbing
	climb->ControllerMove(buttonBoard);
	bool manualControls = true;
	SmartDashboard::PutString("Zeroed","no");
	if (arm->Zeroed() && intake->Zeroed()){ // If already zeroed
		manualControls = pos->RunPresets(buttonBoard);
		SmartDashboard::PutString("Zeroed","yes");
	}
	SmartDashboard::PutString("Manual controls", manualControls ? "Yes" : "No");
	// Arm
	arm->ControllerMove(rightJoy, buttonBoard, manualControls);
	// Intake
	intake->ControllerMove(buttonBoard, manualControls);
	
}

void Robot::TestPeriodic() {}

START_ROBOT_CLASS(Robot)