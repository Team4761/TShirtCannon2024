// Heavily inspired by https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/swervebot/Drivetrain.java
package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import frc.robot.Constants;

/**
 * Makes a swerve drive with 4 modules of NEOs.
 * +x = Right. +y = Forwards. Picture a top down view with the front of the robot at the top.
 */
public class SwerveSubsystem {
    
    // The module offsets from the CENTER of the robot to the CENTER of the wheel on each module.
    // All in meters. +x = right. +y = forwards.
    private final Translation2d frontLeftLocation = new Translation2d(+0.0, -0.0);
    private final Translation2d frontRightLocation = new Translation2d(+0.0, +0.0);
    private final Translation2d backLeftLocation = new Translation2d(-0.0, -0.0);
    private final Translation2d backRightLocation = new Translation2d(-0.0, +0.0);

    private final SwerveModule frontLeft = new SwerveModule(Constants.FRONT_LEFT_TURN_MOTOR_PORT, Constants.FRONT_LEFT_DRIVE_MOTOR_PORT);
    private final SwerveModule frontRight = new SwerveModule(Constants.FRONT_RIGHT_TURN_MOTOR_PORT, Constants.FRONT_RIGHT_DRIVE_MOTOR_PORT);
    private final SwerveModule backLeft = new SwerveModule(Constants.BACK_LEFT_TURN_MOTOR_PORT, Constants.BACK_LEFT_DRIVE_MOTOR_PORT);
    private final SwerveModule backRight = new SwerveModule(Constants.BACK_RIGHT_TURN_MOTOR_PORT, Constants.BACK_RIGHT_DRIVE_MOTOR_PORT);

    // This is just the type of gyro we have.
    private final ADIS16470_IMU gyro = new ADIS16470_IMU();
    
    // The kinematics is used for going from desired positions to actual speeds and vice versa.
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

    // The odometry is what allows us to get things like the current position and expected position.
    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(
            kinematics,
            getGyroRotation(),
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
        }
    );


    /**
     * I honestly don't know if this constructor is needed yet.
     */
    public SwerveSubsystem() {
        gyro.reset();
    }


    /**
     * Drives the robot given the desired speeds and whether or not it should be field relative.
     * Field relative means that no matter which direction the robot is facing, the forwards direction will always be the same (literally the direction is relative to the field).
     * @param xSpeed Speed of the robot in the x direction (+x = right) (meters per second?)
     * @param ySpeed Speed of the robot in the y direction (+y = forwards) (meters per second?)
     * @param rot Angular rate of the robot (+rot = CCW or rotating left) (radians per second?)
     * @param fieldRelative Whether the provided x and y speeds are relative to the field.
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, double periodSeconds) {
        // Represents the speed of the entire robot essentially
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rot);
        // Makes the speeds relative to the field if true (where +x represents the right[?] side of the field)
        if (fieldRelative) {
            chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(chassisSpeeds, getGyroRotation());
        }
        // Essentially discretizing it makes it so that each direction (x, y, rotation) work independently based on time rather than each other (I think).
        chassisSpeeds = ChassisSpeeds.discretize(chassisSpeeds, periodSeconds);
        // Converts the desired chassis speeds into speeds for each swerve module.
        SwerveModuleState[] swerveModuleStates = kinematics.toWheelSpeeds(chassisSpeeds).states;
        // Max the speeds
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.MAX_DRIVE_SPEED);
        frontLeft.setDesiredState(swerveModuleStates[0]);
        frontRight.setDesiredState(swerveModuleStates[1]);
        backLeft.setDesiredState(swerveModuleStates[2]);
        backRight.setDesiredState(swerveModuleStates[3]);
    }


    /**
     * Updates the field relative position (odometry) of the robot. 
     */
    public void updateOdometry() {
        odometry.update(
            getGyroRotation(),
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
            }
        );
    }


    /**
     * Converts the gyro's rotation into a Rotation2d after applying an offset.
     * @return The gyro's rotation after accounting for the offset.
     */
    public Rotation2d getGyroRotation() {
        return new Rotation2d(Units.degreesToRadians(gyro.getAngle())).minus(Constants.GYRO_OFFSET);
    }
}
