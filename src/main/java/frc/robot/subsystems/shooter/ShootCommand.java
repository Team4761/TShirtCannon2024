package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

/**
 * This will simply open up the solenoid and then close it after 0.2 seconds.
 */
public class ShootCommand extends Command {
    
    // Will store when the command ends
    private long endTime;

    @Override
    public void initialize() {
        // Open the air flow
        Robot.map.shooter.setSolenoid(true);
        // 0.2s after
        endTime = System.currentTimeMillis() + 200;
    }

    @Override
    public void end(boolean interrupted) {
        // Close the air flow
        Robot.map.shooter.setSolenoid(false);
    }

    @Override
    public boolean isFinished() {
        if (endTime <= System.currentTimeMillis())
            return true;
        return false;
    }
}
