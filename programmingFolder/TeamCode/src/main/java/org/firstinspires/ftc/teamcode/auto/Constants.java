package org.firstinspires.ftc.teamcode.auto;

public class Constants {
    public static final double
            // ROBOT_WIDTH = 17,
            ROBOT_LENGTH = 15.25,
            TILE_LENGTH = 23.75,
            COUNTS_PER_MOTOR_REV = 1440,
            DRIVE_GEAR_REDUCTION = 0.5, // this is < 1.0 if geared UP
            WHEEL_DIAMETER_INCHES = 4.0,
            COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI),
            DRIVE_SPEED = 0.4,
            TURN_SPEED = 0.3,
            ARM_SPEED = 0.3,
            LEFT_FOUNDATION_DOWN = 0,
            LEFT_FOUNDATION_UP = 1,
            RIGHT_FOUNDATION_DOWN = 1,
            RIGHT_FOUNDATION_UP = 0,
            GRABBER_GRAB = 0.0,
            GRABBER_RELEASE = 0.7,
            GEAR_IN = 32,
            GEAR_OUT = 16,
            GEAR_RATIO_ARM = GEAR_OUT/GEAR_IN,
            P_DISTANCE_PER_ROTATION = 20.8,
            TICKS_PER_MM_ARM = (GEAR_RATIO_ARM * COUNTS_PER_MOTOR_REV)/P_DISTANCE_PER_ROTATION,
            CORRECTION = 3,
            DEGREE_THRESHOLD = 0.5;
}
