#include "Vision.h"

Vision::Vision() {
    swivelServo = new Servo(2);
    liftServo = new Servo(1);
    // liftServo->EnableDeadbandElimination(true);
    liftServo->SetBounds(1.8, 0, 0, 0, 1.2);
}

Vision::~Vision() {}

void Vision::Swivel() {
    double angle = SmartDashboard::GetNumber("Angle of Line", 0);
    double corrected = 90 + angle;
    swivelServo->Set(corrected / 180.0); //normalize
}

void Vision::Lift(Joystick *joy) {
    // if(joy->GetRawButton(19)){
    //     liftServo->Set(0.16667);
    //     SmartDashboard::PutNumber("servo set", 1.0);
    // } else if(joy->GetRawButton(20)) {
    //     liftServo->Set(0);
    //     SmartDashboard::PutNumber("servo set", 0.0);
    // }// else liftServo->Set(SmartDashboard::GetNumber("servo set", 0.0));
    // liftServo->Set(-joy->GetRawAxis(1) / 2.0 * (0.7 - 0.3) + (0.7 + 0.3)/2);
    liftServo->Set(-joy->GetRawAxis(1) / 2.0 + 0.5);
}