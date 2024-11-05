package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * I know this might sound crazy, but this shoots and then rotates the barrel.
 */
public class ShootAndRotateCommand extends SequentialCommandGroup {

    /**
     * I know this might sound crazy, but this shoots and then rotates the barrel.
     * @param isCounterClockwise True if the barrel should rotate counter clockwise. False if it should rotate clockwise.
     */
    public ShootAndRotateCommand(boolean isCounterClockwise) {
        super(
            new ShootCommand(),
            new RotateBarrelCommand(isCounterClockwise)
        );
    }
}
