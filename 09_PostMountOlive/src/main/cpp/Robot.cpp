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
	// frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
	m_chooser.AddOption("Yes zero", "zero");
	m_chooser.SetDefaultOption("No zero","notzero");
	frc::SmartDashboard::PutData("Zeroing at Init", &m_chooser);
	// Joystick inputs
	leftJoy = new Joystick(1);
	rightJoy = new Joystick(2);
	buttonBoard = new Joystick(3);
	// Helper classes
	arm = new Arm();
	drv = new Driving(arm);
	intake = new Intake();
	viz = new Vision();
	climb = new Climbing(drv);
}

void Robot::RobotPeriodic(){
	climb->PressureSense();
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
	drv->Reset();
	if (m_chooser.GetSelected().compare("zero") == 0){
		intake->Reset();
		arm->Reset();
	} else {
		intake->intakeZeroed = true;
		arm->armZeroed = true;
		arm->liftZeroed = true;
	}
}

/**
 * Called periodically during the autonomous period
 */
void Robot::AutonomousPeriodic() {
	TeleopPeriodic();
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
	/* drv->Reset();
	if (m_chooser.GetSelected().compare("zero") == 0){
		intake->Reset();
		arm->Reset();
	} else {
		intake->intakeZeroed = true;
		arm->armZeroed = true;
		arm->liftZeroed = true;
	} */
}

/**
 * Called periodically during the telop (manual control) period
 */
void Robot::TeleopPeriodic() {
	// Driving
	// drv->VelocityPID(1,0.0);
	drv->ControllerMove(leftJoy, rightJoy, buttonBoard);
	// drv->AutoShift();
	// Arm
	arm->ControllerMove(rightJoy, buttonBoard);
	// Intake
	intake->ControllerMove(buttonBoard);
	// Vision
	viz->Swivel(buttonBoard);

	// viz->Lift(buttonBoard);
	// Climbing
	climb->ControllerMove(buttonBoard);
}

void Robot::TestPeriodic() {}

START_ROBOT_CLASS(Robot)