/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.drivetrain;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;
import com.swervedrivespecialties.swervelib.MotorType;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;

public class Drivetrain extends SubsystemBase {

  // These are our modules. We initialize them in the constructor.
  public final SwerveModule m_frontLeftModule;
  public final SwerveModule m_frontRightModule;
  public final SwerveModule m_backLeftModule;
  public final SwerveModule m_backRightModule;

  private AHRS m_gyro = new AHRS(SPI.Port.kMXP, (byte) 200);

  private final SwerveDriveOdometry odometry;

  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  public Drivetrain() {
    ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Drivetrain");

    m_frontLeftModule =
        new MkSwerveModuleBuilder()
            .withLayout(
                shuffleboardTab
                    .getLayout("Front Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(0, 0))
            .withGearRatio(SdsModuleConfigurations.MK4_L1)
            .withDriveMotor(MotorType.NEO, Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR)
            .withSteerMotor(MotorType.NEO, Constants.FRONT_LEFT_MODULE_STEER_MOTOR)
            .withSteerEncoderPort(Constants.FRONT_LEFT_MODULE_STEER_ENCODER)
            .withSteerOffset(Constants.FRONT_LEFT_MODULE_STEER_OFFSET)
            .build();

    m_frontRightModule =
        new MkSwerveModuleBuilder()
            .withLayout(
                shuffleboardTab
                    .getLayout("Front Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(2, 0))
            .withGearRatio(SdsModuleConfigurations.MK4_L1)
            .withDriveMotor(MotorType.NEO, Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR)
            .withSteerMotor(MotorType.NEO, Constants.FRONT_RIGHT_MODULE_STEER_MOTOR)
            .withSteerEncoderPort(Constants.FRONT_RIGHT_MODULE_STEER_ENCODER)
            .withSteerOffset(Constants.FRONT_RIGHT_MODULE_STEER_OFFSET)
            .build();

    m_backLeftModule =
        new MkSwerveModuleBuilder()
            .withLayout(
                shuffleboardTab
                    .getLayout("Back Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(4, 0))
            .withGearRatio(SdsModuleConfigurations.MK4_L1)
            .withDriveMotor(MotorType.NEO, Constants.BACK_LEFT_MODULE_DRIVE_MOTOR)
            .withSteerMotor(MotorType.NEO, Constants.BACK_LEFT_MODULE_STEER_MOTOR)
            .withSteerEncoderPort(Constants.BACK_LEFT_MODULE_STEER_ENCODER)
            .withSteerOffset(Constants.BACK_LEFT_MODULE_STEER_OFFSET)
            .build();

    m_backRightModule =
        new MkSwerveModuleBuilder()
            .withLayout(
                shuffleboardTab
                    .getLayout("Back Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(6, 0))
            .withGearRatio(SdsModuleConfigurations.MK4_L1)
            .withDriveMotor(MotorType.NEO, Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR)
            .withSteerMotor(MotorType.NEO, Constants.BACK_RIGHT_MODULE_STEER_MOTOR)
            .withSteerEncoderPort(Constants.BACK_RIGHT_MODULE_STEER_ENCODER)
            .withSteerOffset(Constants.BACK_RIGHT_MODULE_STEER_OFFSET)
            .build();

    // m_gyro.reset();

    odometry =
        new SwerveDriveOdometry(
            DrivetrainConstants.kKinematics,
            Rotation2d.fromDegrees(getInvertedYaw()),
            new SwerveModulePosition[] {
              m_frontLeftModule.getPosition(),
              m_frontRightModule.getPosition(),
              m_backLeftModule.getPosition(),
              m_backRightModule.getPosition()
            });

    shuffleboardTab.addNumber("Gyroscope Angle", () -> getGyroscopeRotation().getDegrees());
    shuffleboardTab.addNumber("Pose X", () -> odometry.getPoseMeters().getX());
    shuffleboardTab.addNumber("Pose Y", () -> odometry.getPoseMeters().getY());
  }

  /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently
   * facing to the 'forwards' direction.
   */
  public void zeroGyroscope() {
    odometry.resetPosition(
        Rotation2d.fromDegrees(getInvertedYaw()),
        new SwerveModulePosition[] {
          m_frontLeftModule.getPosition(),
          m_frontRightModule.getPosition(),
          m_backLeftModule.getPosition(),
          m_backRightModule.getPosition()
        },
        new Pose2d(odometry.getPoseMeters().getTranslation(), Rotation2d.fromDegrees(0.0)));
  }

  public Rotation2d getGyroscopeRotation() {
    return odometry.getPoseMeters().getRotation();
  }

  @Override
  public void periodic() {
    odometry.update(
        Rotation2d.fromDegrees(getInvertedYaw()),
        new SwerveModulePosition[] {
          m_frontLeftModule.getPosition(),
          m_frontRightModule.getPosition(),
          m_backLeftModule.getPosition(),
          m_backRightModule.getPosition()
        });

    SwerveModuleState[] states =
        DrivetrainConstants.kKinematics.toSwerveModuleStates(m_chassisSpeeds);

    m_frontLeftModule.set(
        (states[0].speedMetersPerSecond
                / Constants.MAX_VELOCITY_METERS_PER_SECOND
                * Constants.MAX_VOLTAGE)
            * -1,
        states[0].angle.getRadians());
    m_frontRightModule.set(
        (states[1].speedMetersPerSecond
                / Constants.MAX_VELOCITY_METERS_PER_SECOND
                * Constants.MAX_VOLTAGE)
            * -1,
        states[1].angle.getRadians());
    m_backLeftModule.set(
        (states[2].speedMetersPerSecond
                / Constants.MAX_VELOCITY_METERS_PER_SECOND
                * Constants.MAX_VOLTAGE)
            * -1,
        states[2].angle.getRadians());
    m_backRightModule.set(
        states[3].speedMetersPerSecond
            / Constants.MAX_VELOCITY_METERS_PER_SECOND
            * Constants.MAX_VOLTAGE,
        states[3].angle.getRadians());
  }

  public void swerveDrive(
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      DoubleSupplier rotationSupplier) {

    m_chassisSpeeds =
        /** Driver Oriented */
        new ChassisSpeeds(
            translationXSupplier.getAsDouble(),
            translationYSupplier.getAsDouble(),
            rotationSupplier.getAsDouble());
    /** Field Oriented */
    // ChassisSpeeds.fromFieldRelativeSpeeds(
    // translationXSupplier.getAsDouble(),
    // translationYSupplier.getAsDouble(),
    // rotationSupplier.getAsDouble(),
    // getGyroscopeRotation());
  }

  public void stopDrive() {
    m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
  }

  public void resetGyro() {
    m_gyro.reset();
  }

  public double getGyroAngle() {
    return m_gyro.getAngle();
  }

  private double getInvertedYaw() {
    return (360 - m_gyro.getYaw());
  }

  public double getAverageEncoder() {
    double averagedistance =
        m_backLeftModule.getDriveDistance()
            + m_backRightModule.getDriveDistance()
            + m_frontLeftModule.getDriveDistance()
            + m_frontRightModule.getDriveDistance();
    return averagedistance / 4;
  }

  public void resetEncoder() {}
}
