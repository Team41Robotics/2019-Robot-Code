#include "Vision.h"
#include "Ports.h"

#define SERVO_kP 0.1

Vision::Vision(){
    pot = new AnalogPotentiometer(PORTS::POTENTIOMETER, 360, 0);
    servo = new Servo(PORTS::SERVO);
}

Vision::~Vision(){

}

void Vision::Swivel(Joystick *leftJoy, Joystick *rightJoy){
    /* double goal = SmartDashboard::GetNumber("camera_angle", 0);
    double current = pot->Get();
    double error = current - goal;
    double speed = error * SERVO_kP;
    if (speed > 1) speed = 1;
    if (speed < -1) speed = -1;
    servo->Set(speed); */
    SmartDashboard::PutNumber("potentiometer", pot->Get());
    servo->Set(leftJoy->GetRawAxis(0));
    SmartDashboard::PutNumber("servo_set", leftJoy->GetRawAxis(0));
    SmartDashboard::PutNumber("servo_get", servo->Get());
}

void Vision::Lift(Joystick *leftJoy, Joystick *rightJoy){
    if (rightJoy->GetRawButton(BUTTONS::MOVE_UP) && !limitSwitchTop->Get()) {
        servo->Set(1);
    }
    if (rightJoy->GetRawButton(BUTTONS::MOVE_DOWN) && !limitSwitchBot->Get()) {
        servo->Set(0);
    }
}