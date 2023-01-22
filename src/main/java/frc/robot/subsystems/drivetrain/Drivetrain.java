/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.drivetrain;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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

  private AHRS m_gyro;

  private final SwerveDriveOdometry odometry;

  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  public Drivetrain(
      AHRS gyro,
      SwerveModule frontLeftModule,
      SwerveModule frontRightModule,
      SwerveModule backLeftModule,
      SwerveModule backRightModule) {
    m_gyro = gyro;
    m_frontLeftModule = frontLeftModule;
    m_frontRightModule = frontRightModule;
    m_backLeftModule = backLeftModule;
    m_backRightModule = backRightModule;

    m_gyro.reset();

    odometry =
        new SwerveDriveOdometry(
            DrivetrainConstants.kKinematics,
            Rotation2d.fromDegrees(m_gyro.getFusedHeading()),
            new SwerveModulePosition[] {
              m_frontLeftModule.getPosition(),
              m_frontRightModule.getPosition(),
              m_backLeftModule.getPosition(),
              m_backRightModule.getPosition()
            });

    ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Drivetrain");
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
        Rotation2d.fromDegrees(m_gyro.getFusedHeading()),
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
        Rotation2d.fromDegrees(m_gyro.getFusedHeading()),
        new SwerveModulePosition[] {
          m_frontLeftModule.getPosition(),
          m_frontRightModule.getPosition(),
          m_backLeftModule.getPosition(),
          m_backRightModule.getPosition()
        });

    SwerveModuleState[] states =
        DrivetrainConstants.kKinematics.toSwerveModuleStates(m_chassisSpeeds);

    m_frontLeftModule.set(
        states[0].speedMetersPerSecond
            / Constants.MAX_VELOCITY_METERS_PER_SECOND
            * Constants.MAX_VOLTAGE,
        states[0].angle.getRadians());
    m_frontRightModule.set(
        states[1].speedMetersPerSecond
            / Constants.MAX_VELOCITY_METERS_PER_SECOND
            * Constants.MAX_VOLTAGE,
        states[1].angle.getRadians());
    m_backLeftModule.set(
        states[2].speedMetersPerSecond
            / Constants.MAX_VELOCITY_METERS_PER_SECOND
            * Constants.MAX_VOLTAGE,
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
  
  public double getAverageEncoder(){
    return 0.0;
  }
}
