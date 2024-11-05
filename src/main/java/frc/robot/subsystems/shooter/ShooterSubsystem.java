package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants;

/**
 * Has a motor (NEO) for turning the barrel and a solenoid to shoot.
 * There is also a switch that tells the code when the barrel has rotated.
 */
public class ShooterSubsystem {
    
    // Used to open/close the air flow in the barrel
    private Solenoid solenoid;
    // Used to rotate the barrel.
    private CANSparkMax turnMotor;
    // Used to check when the barrel is in position.
    private DigitalInput barrelSwitch;


    public ShooterSubsystem() {
        solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.SOLENOID_PORT);
        turnMotor = new CANSparkMax(Constants.SHOOTER_TURN_MOTOR_PORT, MotorType.kBrushless);
        barrelSwitch = new DigitalInput(Constants.SHOOTER_BARREL_SWITCH_PORT);
    }


    /**
     * Opens and closes the solenoid.
     * @param state True to open the solenoid. False to close it.
     */
    public void setSolenoid(boolean state) {
        solenoid.set(state);
    }


    /**
     * Sets the speed of the rotation of the barrel.
     * +speed = counter clock wise = rotate to the left
     * @param speed A number between -1 and 1 where 1 represents 100% speed.
     */
    public void rotateBarrel(double speed) {
        turnMotor.set(speed);
    }


    /**
     * Gets whether or not the barrel is in place based upon the switch.
     * @return True if the barrel is lined up. False if it is not.
     */
    public boolean getSwitchState() {
        return barrelSwitch.get();
    }
}
