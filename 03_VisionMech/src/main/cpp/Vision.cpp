#include "Vision.h"

Vision::Vision(){
    servo = new Servo(2);
}

Vision::~Vision(){

}

void Vision::Swivel(){
    double angle = SmartDashboard::GetNumber("Angle of Line", 0.0);
    double correction = 90 - angle;
    servo->Set(correction / 180.0);
}