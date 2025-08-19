package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Has a motor (NEO) for turning the barrel and a solenoid to shoot.
 * There is also a switch that tells the code when the barrel has rotated.
 */
public class ShooterSubsystem extends SubsystemBase {

    public static boolean activelyRotating = false;

    // private static final double TURN_MOTOR_UNITS_TO_RADIANS = (2 * Math.PI) / Constants.SHOOTER_BARREL_TURN_MOTOR_RESOLUTION / Constants.SHOOTER_BARREL_GEAR_RATIO;
    // private static final double TURN_MOTOR_UNITS_TO_RADIANS = 1.0 / 40.0;
    private static final double TURN_MOTOR_UNITS_TO_RADIANS = (Math.PI * 2.0) / 106.0;
    private static final double PIVOT_MOTOR_TO_RADIANS = (2 * Math.PI) / Constants.SHOOTER_PIVOT_MOTOR_RESOLUTION / Constants.SHOOTER_PIVOT_GEAR_RATIO;

    public static final boolean HAS_SWITCH = false;
    
    // Used to open/close the air flow in the barrel
    private Solenoid solenoid;
    // Used to rotate the barrel.
    private CANSparkMax turnMotor;
    // Used to pivot/angle the barrel up and down where + is up
    private CANSparkMax pivotMotor;
    // Used to check when the barrel is in position.
    private DigitalInput barrelSwitch = null;


    public ShooterSubsystem() {
        solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.SOLENOID_PORT);
        turnMotor = new CANSparkMax(Constants.SHOOTER_TURN_MOTOR_PORT, MotorType.kBrushless);
        pivotMotor = new CANSparkMax(Constants.SHOOTER_PIVOT_MOTOR_PORT, MotorType.kBrushless);

        // Previous values = 1.0 / 16.0 / 1.1
        turnMotor.getEncoder().setPositionConversionFactor(TURN_MOTOR_UNITS_TO_RADIANS);

        if (HAS_SWITCH)
            barrelSwitch = new DigitalInput(Constants.SHOOTER_BARREL_SWITCH_PORT);
    }


    /**
     * Currently just used for logging telemetry data.
     */
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Barrel Rotation", getBarrelRotation().getDegrees());
        SmartDashboard.putNumber("Barrel Pivot", getBarrelPivot().getDegrees());
        SmartDashboard.putNumber("RELATIVE Barrel Rotation", turnMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("ABSOLUTE Barrel Rotation", turnMotor.getAbsoluteEncoder().getPosition());
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
     * @param speed A number between -1 and 1 where 1 represents 100% speed where + = counterclock wise.
     */
    public void rotateBarrel(double speed) {
        turnMotor.set(speed);
    }


    /**
     * Sets the speed of the pivot of the barrel up and down.
     * @param speed A number between -1 and 1 where 1 represents 100% speed and + = rotating the top of the barrel upwards.
     */
    public void pivotBarrel(double speed) {
        pivotMotor.set(speed);
    }


    /**
     * By using the encoder units of the NEO, this gets the current rotation of the barrel.
     * @return The current rotation of the barrel after converting from encoder units to rotation where + = counterclock wise
     */
    public Rotation2d getBarrelRotation() {
        return new Rotation2d(turnMotor.getEncoder().getPosition());
    }


    /**
     * By using the encoder units of the NEO, this gets the current pivot of the barrel.
     * @return The current pivot of the barrel after converting from encoder units to rotation where + = upwards.
     */
    public Rotation2d getBarrelPivot() {
        return new Rotation2d(pivotMotor.getEncoder().getPosition() * PIVOT_MOTOR_TO_RADIANS);
    }


    /**
     * Gets whether or not the barrel is in place based upon the switch.
     * This does not account for if the switch is null, since the code that calls this method should handle it.
     * @return True if the barrel is lined up. False if it is not.
     */
    public boolean getSwitchState() {
        return barrelSwitch.get();
    }
}
