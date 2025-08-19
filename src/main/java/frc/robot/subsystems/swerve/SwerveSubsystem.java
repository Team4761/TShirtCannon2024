// Heavily inspired by https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/swervebot/Drivetrain.java
package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Makes a swerve drive with 4 modules of NEOs.
 * +x = Forwards. +y = Left. Picture a top down view with the front of the robot at the top.
 */
public class SwerveSubsystem extends SubsystemBase{

    private Rotation2d gyroOffset = Constants.GYRO_OFFSET;
    
    // The module offsets from the CENTER of the robot to the CENTER of the wheel on each module.
    // All in meters. +x = forwards. +y = left.
    private final Translation2d frontLeftLocation = new Translation2d(+0.305, +0.305);
    private final Translation2d frontRightLocation = new Translation2d(+0.305, -0.305);
    private final Translation2d backLeftLocation = new Translation2d(-0.305, +0.305);
    private final Translation2d backRightLocation = new Translation2d(-0.305, -0.305);

    private final SwerveModule frontLeft = new SwerveModule(Constants.FRONT_LEFT_TURN_MOTOR_PORT, Constants.FRONT_LEFT_DRIVE_MOTOR_PORT, Constants.FRONT_LEFT_CANCODER_PORT, new Rotation2d(Units.degreesToRadians(-61.347))); //-1624
    private final SwerveModule frontRight = new SwerveModule(Constants.FRONT_RIGHT_TURN_MOTOR_PORT, Constants.FRONT_RIGHT_DRIVE_MOTOR_PORT, Constants.FRONT_RIGHT_CANCODER_PORT, new Rotation2d(Units.degreesToRadians(327.656))); //-114
    private final SwerveModule backLeft = new SwerveModule(Constants.BACK_LEFT_TURN_MOTOR_PORT, Constants.BACK_LEFT_DRIVE_MOTOR_PORT, Constants.BACK_LEFT_CANCODER_PORT, new Rotation2d(Units.degreesToRadians(-202.587))); //93
    private final SwerveModule backRight = new SwerveModule(Constants.BACK_RIGHT_TURN_MOTOR_PORT, Constants.BACK_RIGHT_DRIVE_MOTOR_PORT, Constants.BACK_RIGHT_CANCODER_PORT, new Rotation2d(Units.degreesToRadians(-146.25))); //-1.3

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

    // DETERMINES IF IT IS FIELD OR ROBOT ORIENTED.
    private boolean fieldRelative = true;


    /**
     * I honestly don't know if this constructor is needed yet.
     */
    public SwerveSubsystem() {
        gyro.reset();
    }


    /**
     * SHOULD make it so that the robot is now oriented in the direction it's looking (for field oriented)
     * Makes it so that the current direction is 0 degrees by changing the offset.
     */
    public void reZero() {
        gyroOffset = gyroOffset.plus(getGyroRotation());
    }


    /**
     * Essentially makes it so that the current position is 0,0
     */
    public void resetOdometry() {
        odometry.resetPosition(
            getGyroRotation(), 
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
            }, 
            getPosition()
        );
    }


    /** 
     * This just puts data on the smart dashboard
     */
    @Override
    public void periodic() {
        updateOdometry();

        SmartDashboard.putNumber("FL Rotation", frontLeft.getRotation().getDegrees());
        SmartDashboard.putNumber("FR Rotation", frontRight.getRotation().getDegrees());
        SmartDashboard.putNumber("BL Rotation", backLeft.getRotation().getDegrees());
        SmartDashboard.putNumber("BR Rotation", backRight.getRotation().getDegrees());

        SmartDashboard.putNumber("X", odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("Y", odometry.getPoseMeters().getY());
        SmartDashboard.putNumber("Odometry Rotation", odometry.getPoseMeters().getRotation().getDegrees());
        SmartDashboard.putNumber("Gyro Rotation", getGyroRotation().getRadians());
    }


    /**
     * Drives the robot given the desired speeds and whether or not it should be field relative.
     * Field relative means that no matter which direction the robot is facing, the forwards direction will always be the same (literally the direction is relative to the field).
     * @param xSpeed Speed of the robot in the x direction (+x = forwards) (meters per second?)
     * @param ySpeed Speed of the robot in the y direction (+y = left) (meters per second?)
     * @param rot Angular rate of the robot (+rot = CCW or rotating left) (radians per second?)
     * @param fieldRelative Whether the provided x and y speeds are relative to the field.
     */
    public void drive(double xSpeed, double ySpeed, double rot, double periodSeconds) {
        // Represents the speed of the entire robot essentially
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rot);
        // Makes the speeds relative to the field if true (where +x represents the front side of the field)
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

        SmartDashboard.putNumber("FL Desired", swerveModuleStates[0].angle.getDegrees());
        SmartDashboard.putNumber("FR Desired", swerveModuleStates[1].angle.getDegrees());
        SmartDashboard.putNumber("BL Desired", swerveModuleStates[2].angle.getDegrees());
        SmartDashboard.putNumber("BR Desired", swerveModuleStates[3].angle.getDegrees());
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
        // Is negative because it was placed on upside down :/
        return new Rotation2d(-Units.degreesToRadians(gyro.getAngle(IMUAxis.kYaw))).minus(gyroOffset);
    }


    /**
     * Gets the current position based on the odometry.
     * @return The current position of the robot in meters.
     */
    public Pose2d getPosition() {
        return odometry.getPoseMeters();
    }


    /**
     * GET ALL THE MODULES FOR DEBUGGING PURPOSES ONLY
     */
    public SwerveModule getFLModule() { return this.frontLeft; }
    public SwerveModule getFRModule() { return this.frontRight; }
    public SwerveModule getBLModule() { return this.backLeft; }
    public SwerveModule getBRModule() { return this.backRight; }
}
