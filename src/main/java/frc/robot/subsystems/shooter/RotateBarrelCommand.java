package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

/**
 * This will rotate the barrel of the shooter until the switch has been activated.
 */
public class RotateBarrelCommand extends Command {

    private boolean isCCW;
    private boolean lastSwitchState;

    /**
     * This will rotate the barrel of the shooter until the switch has been activated.
     * @param isCounterClockwise True if the barrel should rotate counter clockwise. False if it should rotate clockwise.
     */
    public RotateBarrelCommand(boolean isCounterClockwise) {
        this.isCCW = isCounterClockwise;
        this.lastSwitchState = true;    // Assume that the switch is being pressed at the start
    }

    @Override
    public void initialize() {
        if (isCCW)
            Robot.map.shooter.rotateBarrel(0.1);
        else
            Robot.map.shooter.rotateBarrel(-0.1);
    }

    @Override
    public void end(boolean interrupted) {
        Robot.map.shooter.rotateBarrel(0);
    }

    /**
     * If the last time we checked, the barrel was not on the switch, but it is now, finish the command.
     */
    @Override
    public boolean isFinished() {
        return (lastSwitchState == false && Robot.map.shooter.getSwitchState() == true);
    }
}
