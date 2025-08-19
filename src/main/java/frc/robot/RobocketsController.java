package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.shooter.RotateBarrelCommand;
import frc.robot.subsystems.shooter.ShootAndRotateCommand;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class RobocketsController extends XboxController{

    // Just for debugging when controlling each motor individually
    private int currentMotor = 0;

    private CommandScheduler commandScheduler = CommandScheduler.getInstance();


    // Slew rate limiters caps the transition between different speeds. (ex, a limiter of 3 means that the max change in speed is 3 per second.)
    // A rateLimit of 3 means that it will take ~1/3 to go from 0 -> 1.
    private final SlewRateLimiter leftXLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter leftYLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter rightXLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter rightYLimiter = new SlewRateLimiter(3);
    

    /**
     * Creates a new Xbox controller that's connected to the computer (drivestation).
     * @param port The USB port as decided by the Driverstation App.
     */
    public RobocketsController(int port) {
        super(port);
    }


    //   Y
    // X   B
    //   A
    /**
     * Must be called during the teleopPeriodic() method in Robot.java to properly use the key binds.
     */
    public void teleopPeriodic() {
        // Shooter keybinds
        if (Robot.map.shooter != null) {
            // Pivot the barrel on button pushes (cause swerve needs joysticks)
            if (getAButton()) {
                Robot.map.shooter.pivotBarrel(0.15);
            }
            else if (getYButton()) {
                Robot.map.shooter.pivotBarrel(-0.15);
            }
            // Get rid of movement if not pressing either button
            else {
                Robot.map.shooter.pivotBarrel(0.0);
            }
            // Rotate the barrel on button pushes
            if (getBButtonPressed()) {
                // commandScheduler.schedule(new RotateBarrelCommand(false));
            }
            // Shoot and rotate if use the bumpers
            if (getLeftBumperPressed()) {
                // commandScheduler.schedule(new ShootAndRotateCommand(true));
            }
            if (getRightBumperPressed()) {
                commandScheduler.schedule(new ShootAndRotateCommand(false));
            }

            if (getPOV() == 90) {
                if (!ShooterSubsystem.activelyRotating)
                    commandScheduler.schedule(new RotateBarrelCommand(false));
            }
            if (getPOV() == 270) {
                if (!ShooterSubsystem.activelyRotating)
                    commandScheduler.schedule(new RotateBarrelCommand(true));
            }
            if (getPOV() == 0) {
                Robot.map.shooter.pivotBarrel(-0.15);
            }
            if (getPOV() == 180) {
                Robot.map.shooter.pivotBarrel(0.15);
            }

            // Robot.map.shooter.rotateBarrel(getLeftX()*0.2);
        }

        // Swerve
        if (Robot.map.swerve != null) {
            Robot.map.swerve.drive(
                -getLeftY(),   // Negative to make up the positive direction
                -getLeftX(),
                -getRightX(),  // Negative to make left (counterclockwise) the positive direction.
                Robot.periodSeconds
            );

            if (getXButtonPressed()) {
                Robot.map.swerve.reZero();
            }
        }
    }


    /**
     * Must be called during the testPeriodic() method in Robot.java to properly use the key binds.
     */
    public void testPeriodic() {
        // Motor selection
        if (getLeftBumperPressed()) {
            currentMotor = Math.abs(currentMotor - 1) % 4;
        }
        if (getRightBumperPressed()) {
            currentMotor = (currentMotor + 1) % 4;
        }

        // Swerve
        if (Robot.map.swerve != null) {
            if (currentMotor == 0) {
                Robot.map.swerve.getFLModule().setTurnSpeed(getLeftX() * 0.3);
                Robot.map.swerve.getFLModule().setDriveSpeed(getRightX() * 0.3);
                SmartDashboard.putNumber("FL Rotation", Robot.map.swerve.getFLModule().getRotation().getDegrees());
            }
            if (currentMotor == 1) {
                Robot.map.swerve.getFRModule().setTurnSpeed(getLeftX() * 0.3);
                Robot.map.swerve.getFRModule().setDriveSpeed(getRightX() * 0.3);
                SmartDashboard.putNumber("FR Rotation", Robot.map.swerve.getFRModule().getRotation().getDegrees());
            }
            if (currentMotor == 2) {
                Robot.map.swerve.getBLModule().setTurnSpeed(getLeftX() * 0.3);
                Robot.map.swerve.getBLModule().setDriveSpeed(getRightX() * 0.3);
                SmartDashboard.putNumber("BL Rotation", Robot.map.swerve.getBLModule().getRotation().getDegrees());
            }
            if (currentMotor == 3) {
                Robot.map.swerve.getBRModule().setTurnSpeed(getLeftX() * 0.3);
                Robot.map.swerve.getBRModule().setDriveSpeed(getRightX() * 0.3);
                SmartDashboard.putNumber("BR Rotation", Robot.map.swerve.getBRModule().getRotation().getDegrees());
            }
        }


        if (getXButtonPressed()) {
            currentMotor++;
        }
    }


    // Applying slew limiters and deadzone (deadband) to every single axis.
    @Override
    public double getLeftX() {
        return leftXLimiter.calculate(MathUtil.applyDeadband(super.getLeftX(), 0.02));
    }
    @Override
    public double getLeftY() {
        return leftYLimiter.calculate(MathUtil.applyDeadband(super.getLeftY(), 0.02));
    }
    @Override
    public double getRightX() {
        return rightXLimiter.calculate(MathUtil.applyDeadband(super.getRightX(), 0.02));
    }
    @Override
    public double getRightY() {
        return rightYLimiter.calculate(MathUtil.applyDeadband(super.getRightY(), 0.02));
    }
    
}
