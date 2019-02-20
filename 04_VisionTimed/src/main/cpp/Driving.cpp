#include "Driving.h"

Driving::Driving() {
    LFTalon = new TalonSRX(0);
    LBTalon = new TalonSRX(3);
    RFTalon = new TalonSRX(1);
    RBTalon = new TalonSRX(2);
}

Driving::~Driving() {}

void Driving::ControllerMove(Joystick *leftJoy, Joystick *rightJoy) {
    double leftSpd = leftJoy->GetRawAxis(1);
    double rightSpd = rightJoy->GetRawAxis(1);
    Drive(leftSpd, rightSpd);
}

void Driving::Drive(double left, double right){
    SmartDashboard::PutNumber("MOVELEFT", left);
    SmartDashboard::PutNumber("MOVERIGHT", -right);
    LFTalon->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, left);
    LBTalon->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, left);
    RFTalon->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -right);
    RBTalon->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -right);
}