package frc.robot.drive;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;


public class SwerveDrive extends DriveBase {
    /**
     * The maximum voltage that will be delivered to the drive motors.
     * <p>
     * This can be reduced to cap the robot's maximum speed. Typically, this is
     * useful during initial testing of the robot.
     */
    public static final double MAX_VOLTAGE = 12.0;
    /**
     * The maximum velocity of the robot in meters per second.
     * <p>
     * This is a measure of how fast the robot should be able to drive in a straight
     * line.
     */
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 5880.0 / 60.0
            / SdsModuleConfigurations.MK4I_L1.getDriveReduction() * SdsModuleConfigurations.MK4I_L1.getWheelDiameter()
            * Math.PI;
    /**
     * The maximum angular velocity of the robot in radians per second.
     * <p>
     * This is a measure of how fast the robot can rotate in place.
     */
    // Here we calculate the theoretical maximum angular velocity. You can also
    // replace this with a measured amount.
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
            Math.hypot(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0);

    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                    Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Front right
            new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                    -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back left
            new Translation2d(-Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                    Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back right
            new Translation2d(-Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                    -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0));

    private final AHRS m_navx = (AHRS)getGyro(); // NavX connected over MXP

    // These are our modules. We initialize them in the constructor.
    private final SwerveModule m_frontLeftModule;
    private final SwerveModule m_frontRightModule;
    private final SwerveModule m_backLeftModule;
    private final SwerveModule m_backRightModule;

    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(m_kinematics,
            Rotation2d.fromDegrees(m_navx.getFusedHeading()));

    private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    SwerveDrive() {
        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Drivetrain");

        m_frontLeftModule = Mk4iSwerveModuleHelper.createNeo(
                shuffleboardTab.getLayout("Front Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0),
                Mk4iSwerveModuleHelper.GearRatio.L1,
                Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
                Constants.FRONT_LEFT_MODULE_STEER_MOTOR,
                Constants.FRONT_LEFT_MODULE_STEER_ENCODER,
                Constants.FRONT_LEFT_MODULE_STEER_OFFSET);

        m_frontRightModule = Mk4iSwerveModuleHelper.createNeo(
                shuffleboardTab.getLayout("Front Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(2, 0),
                Mk4iSwerveModuleHelper.GearRatio.L1,
                Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                Constants.FRONT_RIGHT_MODULE_STEER_MOTOR,
                Constants.FRONT_RIGHT_MODULE_STEER_ENCODER,
                Constants.FRONT_RIGHT_MODULE_STEER_OFFSET);

        m_backLeftModule = Mk4iSwerveModuleHelper.createNeo(
                shuffleboardTab.getLayout("Back Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(4, 0),
                Mk4iSwerveModuleHelper.GearRatio.L1,
                Constants.BACK_LEFT_MODULE_DRIVE_MOTOR,
                Constants.BACK_LEFT_MODULE_STEER_MOTOR,
                Constants.BACK_LEFT_MODULE_STEER_ENCODER,
                Constants.BACK_LEFT_MODULE_STEER_OFFSET);

        m_backRightModule = Mk4iSwerveModuleHelper.createNeo(
                shuffleboardTab.getLayout("Back Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(6, 0),
                Mk4iSwerveModuleHelper.GearRatio.L1,
                Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR,
                Constants.BACK_RIGHT_MODULE_STEER_MOTOR,
                Constants.BACK_RIGHT_MODULE_STEER_ENCODER,
                Constants.BACK_RIGHT_MODULE_STEER_OFFSET);

        shuffleboardTab.addNumber("Gyroscope Angle", () -> getGyroscopeRotation().getDegrees());
        shuffleboardTab.addNumber("Pose X", () -> odometry.getPoseMeters().getX());
        shuffleboardTab.addNumber("Pose Y", () -> odometry.getPoseMeters().getY());
    }

    @Override
    public void resetEncoders() {
        // TODO Auto-generated method stub
    }

    @Override
    public double getLeftEncoderDistance() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getRightEncoderDistance() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getAverageEncoderDistance() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public void setRampRate(double rate) {
        // TODO Auto-generated method stub

    }

    @Override
    public void moveDistanceWithPID(double distance) throws Exception {
        // TODO Auto-generated method stub

    }

    @Override
    public void setIdleMode(int idleMode) {
        // TODO Auto-generated method stub

    }

    /**
     * Sets the gyroscope angle to zero. This can be used to set the direction the
     * robot is currently facing to the
     * 'forwards' direction.
     */
    public void zeroGyroscope() {
        odometry.resetPosition(
                new Pose2d(odometry.getPoseMeters().getTranslation(), Rotation2d.fromDegrees(0.0)),
                Rotation2d.fromDegrees(m_navx.getFusedHeading()));
    }

    @Override
    public void resetGyro() {
        zeroGyroscope();
    }

    public Rotation2d getGyroscopeRotation() {
        return odometry.getPoseMeters().getRotation();
    }

    @Override
    public void swerveDrive(DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier, DoubleSupplier rotationSupplier) {
        m_chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            translationXSupplier.getAsDouble(),
            translationYSupplier.getAsDouble(),
            rotationSupplier.getAsDouble(),
            getGyroscopeRotation());
    }

    @Override
    public void periodic() {
        odometry.update(Rotation2d.fromDegrees(m_navx.getFusedHeading()),
                new SwerveModuleState(m_frontLeftModule.getDriveVelocity(),
                        new Rotation2d(m_frontLeftModule.getSteerAngle())),
                new SwerveModuleState(m_frontRightModule.getDriveVelocity(),
                        new Rotation2d(m_frontRightModule.getSteerAngle())),
                new SwerveModuleState(m_backLeftModule.getDriveVelocity(),
                        new Rotation2d(m_backLeftModule.getSteerAngle())),
                new SwerveModuleState(m_backRightModule.getDriveVelocity(),
                        new Rotation2d(m_backRightModule.getSteerAngle())));

        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);

        m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                states[0].angle.getRadians());
        m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                states[1].angle.getRadians());
        m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                states[2].angle.getRadians());
        m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                states[3].angle.getRadians());
    }

}
