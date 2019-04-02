/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>

#include <frc/WPILib.h>

// comment out for practice robot
#define FINAL_ROBOT 

#include "Driving.h"
#include "Arm.h"
#include "Vision.h"
#include "Intake.h"
#include "Climbing.h"
#include "PosControl.h"

#include <frc/TimedRobot.h>
#include <frc/SmartDashboard/SendableChooser.h>

class Robot : public frc::TimedRobot {
public:
	void RobotInit() override;
	void AutonomousInit() override;
	void AutonomousPeriodic() override;
	void TeleopInit() override;
	void TeleopPeriodic() override;
	void TestPeriodic() override;

private:
	frc::SendableChooser<std::string> m_chooser;
	const std::string kAutoNameDefault = "Default";
	const std::string kAutoNameCustom = "My Auto";
	std::string m_autoSelected;
	Joystick *leftJoy, *rightJoy, *buttonBoard;
	Driving *drv;
	Arm *arm;
	Vision *viz;
	Intake *intake;
	Climbing *climb;
	PosControl *pos;
};
