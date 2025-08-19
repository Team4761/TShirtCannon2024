package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

/**
 * This command uses the Swerve Subsystem to move a set distance using the Swerve odometry.
 */
public class MoveCommand extends Command {

    private static final double MAX_SPEED = 0.4;    // In percents where 1.0 = 100%

    // x,y are in meters. +x = forwards. +y = left
    private double x;
    private double y;
    private Rotation2d rotation;

    // This is where the robot will END at.
    private Pose2d targetPosition;
    
    /**
     * Makes the robot move (x,y) meters. This does NOT set it's set position, it applies an offset!
     * +x is forwards. +y is left. +rotation is CCW
     * @param x The distance forwards in meters.
     * @param y The distance to the left in meters.
     * @param rotation The rotation counterclock wise.
     */
    public MoveCommand(double x, double y, Rotation2d rotation) {
        this.x = x;
        this.y = y;
        this.rotation = rotation;
    }


    /**
     * Initialize the target position.
     */
    @Override
    public void initialize() {
        targetPosition = Robot.map.swerve.getPosition().plus(new Transform2d(this.x, this.y, this.rotation));
    }


    /**
     * Makes the robot move towards it's target position
     */
    @Override
    public void execute() {
        Transform2d distanceLeft = targetPosition.minus(Robot.map.swerve.getPosition());
        Robot.map.swerve.drive(
            Math.signum(distanceLeft.getX()) * MAX_SPEED,
            Math.signum(distanceLeft.getY()) * MAX_SPEED,
            Math.signum(distanceLeft.getRotation().getDegrees()) * MAX_SPEED,
            Robot.periodSeconds
        );
    }
}
