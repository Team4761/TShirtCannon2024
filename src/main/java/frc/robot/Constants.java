package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class Constants {
    
    // Controller
    public static final int CONTROLLER_PORT = 0;    // Port on the Driverstation


    // Swerve
    public static final int FRONT_LEFT_TURN_MOTOR_PORT = 0;     // WIP CAN Port
    public static final int FRONT_LEFT_DRIVE_MOTOR_PORT = 1;    // WIP CAN Port
    public static final int FRONT_RIGHT_TURN_MOTOR_PORT = 2;     // WIP CAN Port
    public static final int FRONT_RIGHT_DRIVE_MOTOR_PORT = 3;    // WIP CAN Port
    public static final int BACK_LEFT_TURN_MOTOR_PORT = 4;     // WIP CAN Port
    public static final int BACK_LEFT_DRIVE_MOTOR_PORT = 5;    // WIP CAN Port
    public static final int BACK_RIGHT_TURN_MOTOR_PORT = 6;     // WIP CAN Port
    public static final int BACK_RIGHT_DRIVE_MOTOR_PORT = 7;    // WIP CAN Port

    public static final double WHEEL_RADIUS = 0.0d;             // WIP in meters
    public static final int TURNING_ENCODER_RESOLUTION = 4096;  // WIP Number of ticks it takes to turn the wheel (swerve) completely.
    public static final int MOVING_ENCODER_RESOLUTION = 4096;   // WIP Number of ticks it takes the driving wheel to make a full rotation.
    public static final int TURNING_GEAR_RATIO = 1;             // WIP The gear ratio to divide by when converting the encoder ticks to distance.
    public static final int MOVING_GEAR_RATIO = 1;              // WIP The gear ratio to divide by when converting the encoder ticks to distance.

    public static final double MAX_ANGULAR_VELOCITY = Units.degreesToRadians(180);          // WIP Radians per second.
    public static final double MAX_ANGULAR_ACCELERATION = Units.degreesToRadians(360);      // WIP Radians per second squared.
    public static final double MAX_DRIVE_SPEED = 3.0;     // Meters per second.

    public static final Rotation2d GYRO_OFFSET = new Rotation2d(0.0);  // The number inside is in degrees and must be measured.


    // Vision
    public static final int RESOLUTION_X = 640;     // In pixels
    public static final int RESOLUTION_Y = 480;     // In pixels


    // Shooter
    public static final int SOLENOID_PORT = 0;              // WIP PWM port I believe.
    public static final int SHOOTER_TURN_MOTOR_PORT = 8;    // WIP CAN port.
    public static final int SHOOTER_BARREL_SWITCH_PORT = 0; // WIP DIO port.
}