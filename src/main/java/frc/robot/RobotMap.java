package frc.robot;

import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

/**
 * 4
 * This will contain all of our subsystems.
 * We're doing this so we can disable specific subsystems at a time for testing.
 * 
 * All code that calls subsystems must account for them being null.
 */
public class RobotMap {
    
    // Start all the subsystems as null so Java doesn't throw errors.
    public LEDSubsystem leds = null;
    public ShooterSubsystem shooter = null;
    public SwerveSubsystem swerve = null;
    public VisionSubsystem vision = null;


    /**
     * This will only be called in one place: Robot.java statically
     * To disable specific parts of the robot, just comment out the line that initializes the subsystem!
    */
    public RobotMap() {
        // leds = new LEDSubsystem();
        // shooter = new ShooterSubsystem();
        // swerve = new SwerveSubsystem();
        // vision = new VisionSubsystem();
    }
}