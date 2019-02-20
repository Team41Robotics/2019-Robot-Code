#pragma once

enum PORTS {
    // Pneumatics
    COMPRESSOR = 0,
    // Drive train
    SPARK_LF = 3,
	SPARK_LB = 4,
	SPARK_RF = 1,
	SPARK_RB = 2,
    SOL_SHIFT_IN = 0,
    SOL_SHIFT_OUT = 1,
    // Arm
    SOL_CLAW_IN = 4,
    SOL_CLAW_OUT = 5,
    SOL_SHOOT_IN = 2,
    SOL_SHOOT_OUT = 3,
    TALON_INTAKE = 9,
    // User input
    LEFT_JOYSTICK = 1,
    RIGHT_JOYSTICK = 2,
    // Vision assist
    POTENTIOMETER = 0,
    SERVO = 0

};

enum BUTTONS {
    //Left Joystick
    CLAW_OPEN = 3,
    CLAW_CLOSE = 2,
    SHIFT_DOWN = 1,
    //Right Joystick
    SUCK_IN = 4,
    SHOOT_BALL = 3,
    SHIFT_UP = 1,
    //Lift for LIDAR
    MOVE_UP = 11,
    MOVE_DOWN = 10
};