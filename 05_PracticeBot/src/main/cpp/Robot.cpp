/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <frc/SmartDashboard/SmartDashboard.h>

void Robot::RobotInit() {
	// m_chooser.AddDefault(kAutoNameDefault, kAutoNameDefault);
	// m_chooser.AddObject(kAutoNameCustom, kAutoNameCustom);
	// frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
	// Joystick inputs
	leftJoy = new Joystick(1);
	rightJoy = new Joystick(2);
	buttonBoard = new Joystick(3);
	// Helper classes
	drv = new Driving();
	arm = new Arm();
	intake = new Intake();
	viz = new Vision();
	climb = new Climbing();
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
	// m_autoSelected = SmartDashboard::GetString(
	// 		"Auto Selector", kAutoNameDefault);

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

}

void Robot::TeleopPeriodic() {
	// Driving
	drv->ControllerMove(leftJoy, rightJoy, buttonBoard);
	//drv->AutoShift();
	// Arm
	arm->ControllerMove(rightJoy, buttonBoard);
	// Intake
	intake->ControllerMove(buttonBoard);
	// Vision
	//viz->Swivel();
	viz->Lift(buttonBoard);
	// Climbing
	//climb->ControllerMove(buttonBoard);
}

void Robot::TestPeriodic() {}

START_ROBOT_CLASS(Robot)