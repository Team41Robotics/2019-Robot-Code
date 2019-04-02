#pragma once
#define FINAL_ROBOT
#ifdef FINAL_ROBOT
namespace PORTS {
    enum VALUES {
        // Pneumatics
        COMPRESSOR = 1,
        // Drive train
        SPARK_LF = 3,
        SPARK_LB = 1,
        SPARK_RF = 4,
        SPARK_RB = 2,
        SOL_SHIFT_L_PCM = 1,
        SOL_SHIFT_L_IN = 2,
        SOL_SHIFT_L_OUT = 3,
        SOL_SHIFT_R_PCM = 1,
        SOL_SHIFT_R_IN = 0,
        SOL_SHIFT_R_OUT = 1,
        // Claw
        SOL_CLAW_PCM = 1,
        SOL_CLAW_IN = 4,
        SOL_CLAW_OUT = 5,
        SOL_SHOOT_PCM = 1,
        SOL_SHOOT_IN = 7,
        SOL_SHOOT_OUT = 6,
        // Arm
        ARM_TALON0 = 7,
        ARM_TALON1 = 6,
        ARM_ENCODER_A = 6,
        ARM_ENCODER_B = 7,
        ARM_TOP_LIMIT = 2,
        ARM_BOTTOM_LIMIT = 3,
        // Intake
        TALON_INTAKE = 3,
        TALON_INTAKE_UP0 = 1,
        TALON_INTAKE_UP1 = 2,
        INTAKE_ENC_A = 8,
        INTAKE_ENC_B = 9,
        INTAKE_TOP_LIMIT = 3,

        // Lift
        LIFT_TALON0 = 4,
        LIFT_TALON1 = 5,
        LIFT_ENCODER_A = 4,
        LIFT_ENCODER_B = 5,
        LIFT_TOP_LIMIT = 0,
        LIFT_BOTTOM_LIMIT = 1,
        // Vision assist
        SWIVEL = 0,
        LIDAR_LIFT = 1,
        // Climbing
        CLIMB_SOL_PCM = 2,
        CLIMB_SOL0_0 = 4,
        CLIMB_SOL0_1 = 5,
        CLIMB_SOL1_0 = 6,
        CLIMB_SOL1_1 = 7,
        ULTRASONIC = 0
    };
}

#else // PRACTICE BOT
namespace PORTS {
    enum VALUES {
        // Pneumatics
        COMPRESSOR = 0,
        // Drive train
        SPARK_LF = 3,
        SPARK_LB = 1,
        SPARK_RF = 4,
        SPARK_RB = 2,
        SOL_SHIFT_PCM = 1,
        SOL_SHIFT_IN = 6,
        SOL_SHIFT_OUT = 7,
        // Claw
        SOL_CLAW_PCM = 0,
        SOL_CLAW_IN = 2, // ???
        SOL_CLAW_OUT = 3, // ???
        SOL_SHOOT_PCM = 0,
        SOL_SHOOT_IN = 1,
        SOL_SHOOT_OUT = 0,
        // Arm
        ARM_TALON0 = 7,
        ARM_TALON1 = 6,
        ARM_ENCODER_A = 5,
        ARM_ENCODER_B = 4,
        ARM_TOP_LIMIT = 2,
        // Intake
        TALON_INTAKE = 2,
        TALON_INTAKE_UP0 = 11,
        TALON_INTAKE_UP1 = 1,
        INTAKE_ENC_A = 8,
        INTAKE_ENC_B = 9,
        INTAKE_TOP_LIMIT = 3,
        // Lift
        LIFT_TALON0 = 10,
        LIFT_TALON1 = 5,
        LIFT_ENCODER_A = 6,
        LIFT_ENCODER_B = 7,
        LIFT_TOP_LIMIT = 0,
        LIFT_BOTTOM_LIMIT = 1,
        // Vision assist
        SWIVEL = 0,
        LIDAR_LIFT = 1,
        // Climbing
        CLIMB_SOL_PCM = 1,
        CLIMB_SOL0_0 = 3,
        CLIMB_SOL0_1 = 2,
        CLIMB_SOL1_0 = 5,
        CLIMB_SOL1_1 = 4,
        ULTRASONIC = 0
    };
}
#endif


namespace BUTTONS {
    enum VALUES {
        // Left joystick
        TOGGLE_SHIFT = 1,
        // Right joystick
        CLAW_SHOOT = 1,
        // Button board
        // Lift
        LIFT_UP = 24,
        LIFT_DOWN = 23,
        LIFT_DEFAULT = 27,
        LIFT_SPEED = 4, // Axis
        HATCH_1_HIGH = 26,
        // Arm
        ARM_UP = 22,
        ARM_DOWN = 21,
        ARM_DEFAULT = 27,
        ARM_SPEED = 3, // Axis
        // Lidar lift
        LIDAR_UP = 20,
        LIDAR_DOWN = 19,
        // Intake
        INTAKE_UP = 18,
        INTAKE_DOWN = 17,
        INTAKE_IN = 3,
        INTAKE_OUT = 2,
        INTAKE_DEFAULT = 25,
        INTAKE_SPEED = 0, // Axis
        // Claw
        TOGGLE_CLAW = 1,
        CLAW_OPEN = 10,
        CLAW_CLOSE = 7,
        // Drive train
        DRIVER_OVERRIDE = 6,
        SHIFT_UP = 8,
        SHIFT_DOWN = 9,
        // Climbing
        CLIMB_EXTEND = 5,
        DRIVING_ASSIST = 25
    };
}