#include "Vision.h"
#include "Ports.h"

Vision::Vision(){
    swivel = new Servo(PORTS::SWIVEL);
    lift = new Servo(PORTS::LIDAR_LIFT);
}

Vision::~Vision(){

}

void Vision::Swivel(){
    double angle = SmartDashboard::GetNumber("Angle of Line", 0);
    double dist = 1.8 - SmartDashboard::GetNumber("Distance to Line",1.8);
    double offset = dist <= 0 ? 0 : 20.0 / (1.8-0.76) * (dist);
    double corrected = 90 + angle + offset;
    swivel->Set(corrected / 180.0); //normalize
}

void Vision::Lift(Joystick *buttonBoard){
     int level = buttonBoard->GetPOV(0) / 45;
     if (buttonBoard->GetRawButton(BUTTONS::LIDAR_UP)) {
        lift->Set(0.225);
    }
    else if (buttonBoard->GetRawButton(BUTTONS::LIDAR_DOWN)) {
        lift->Set(0.4);
    }
    else {
        if(level > 0 && level % 2 == 0) lift->Set(0.36); // Even level means cargo
        else if(level % 2 == 1) lift->Set(0); // Odd level means hatch
    }
}