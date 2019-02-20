/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/WPILib.h>
#include <ctre/Phoenix.h>

#define kPercentOutput ctre::phoenix::motorcontrol::ControlMode::PercentOutput

using namespace frc;

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
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
  TalonSRX *arm0, *arm1, *lift0, *lift1, *intake0, *intake1, *intake;
  DigitalInput *liftTopLimit;
  Servo *lift_servo;

  // Encoder *enc7, *enc3;
  frc::Joystick *joy0, *joy1;
  Compressor *comp;
  DoubleSolenoid *shootSol, *clawSol;
};
