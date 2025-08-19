package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

/**
 * This will rotate the barrel of the shooter until the switch has been activated.
 */
public class RotateBarrelCommand extends Command {

    private static final Rotation2d ROTATION_BETWEEN_BARRELS = new Rotation2d(Units.degreesToRadians(45));

    private boolean isCCW;
    private boolean lastSwitchState = false;
    private Rotation2d rotationGoal = ROTATION_BETWEEN_BARRELS;

    /**
     * This will rotate the barrel of the shooter until the switch has been activated.
     * If there is a switch, then it will use the switch. Otherwise, it will use the encoder units.
     * @param isCounterClockwise True if the barrel should rotate counter clockwise. False if it should rotate clockwise.
     */
    public RotateBarrelCommand(boolean isCounterClockwise) {
        this.isCCW = isCounterClockwise;
        if (ShooterSubsystem.HAS_SWITCH)
            this.lastSwitchState = true;    // Assume that the switch is being pressed at the start
    }

    @Override
    public void initialize() {
        ShooterSubsystem.activelyRotating = true;

        System.out.println("Started rotating! Counterclockwise: " + isCCW);
        if (isCCW)
            Robot.map.shooter.rotateBarrel(0.2);
        else
            Robot.map.shooter.rotateBarrel(-0.2);
        // If there is no limit switch, set the target goal
        if (!ShooterSubsystem.HAS_SWITCH) {
            if (isCCW)
                rotationGoal = Robot.map.shooter.getBarrelRotation().plus(rotationGoal);
            else
                rotationGoal = Robot.map.shooter.getBarrelRotation().minus(rotationGoal);
        }
    }

    @Override
    public void end(boolean interrupted) {
        ShooterSubsystem.activelyRotating = false;

        System.out.println("Finished rotating!");
        Robot.map.shooter.rotateBarrel(0);
    }

    /**
     * If the last time we checked, the barrel was not on the switch, but it is now, finish the command.
     */
    @Override
    public boolean isFinished() {
        if (ShooterSubsystem.HAS_SWITCH)
            return (lastSwitchState == false && Robot.map.shooter.getSwitchState() == true);
        // If there is no limit switch, use the encoder units instead to find if the barrel is within 4 degrees of the target rotation.
        else
            return (Math.abs(rotationGoal.minus(Robot.map.shooter.getBarrelRotation()).getDegrees()) <= 4);

    }
}
