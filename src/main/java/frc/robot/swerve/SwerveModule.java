package frc.robot.swerve;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

/**
 * This represents a single swerve module.
 * This consists of a motor for steering the wheel and another for driving the wheel.
 */
public class SwerveModule {
    
    Spark steerMotor;
    Spark driveMotor;


    /**
     * This represents a single swerve module.
     * @param steerMotorPort This is the port for the motor that rotates the wheel's direction. CAN port.
     * @param driveMotorPort This is the port driving the wheel. CAN port.
     */
    public SwerveModule(int steerMotorPort, int driveMotorPort) {
        steerMotor = new Spark(steerMotorPort);
        driveMotor = new Spark(driveMotorPort);
    }

}
