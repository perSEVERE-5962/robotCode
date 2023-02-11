/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.drivetrain;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;
import com.swervedrivespecialties.swervelib.MotorType;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  private static final double MAX_VOLTAGE = 12.0;
  public static final double MAX_VELOCITY_METERS_PER_SECOND =
      5880.0
          / 60.0
          * SdsModuleConfigurations.MK4I_L1.getDriveReduction()
          * SdsModuleConfigurations.MK4I_L1.getWheelDiameter()
          * Math.PI;
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND =
      MAX_VELOCITY_METERS_PER_SECOND
          / Math.hypot(
              Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
              Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0);

  // These are our modules. We initialize them in the constructor.
  public final SwerveModule m_frontLeftModule;
  public final SwerveModule m_frontRightModule;
  public final SwerveModule m_backLeftModule;
  public final SwerveModule m_backRightModule;

  private AHRS m_gyro = new AHRS(SPI.Port.kMXP, (byte) 200);

  public static final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          // Front left
          new Translation2d(
              Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
              Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Front right
          new Translation2d(
              Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
              -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back left
          new Translation2d(
              -Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
              Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back right
          new Translation2d(
              -Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
              -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0));

  private final SwerveDriveOdometry m_odometry;

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
            .withGearRatio(SdsModuleConfigurations.MK4I_L1)
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
            .withGearRatio(SdsModuleConfigurations.MK4I_L1)
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
            .withGearRatio(SdsModuleConfigurations.MK4I_L1)
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
            .withGearRatio(SdsModuleConfigurations.MK4I_L1)
            .withDriveMotor(MotorType.NEO, Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR)
            .withSteerMotor(MotorType.NEO, Constants.BACK_RIGHT_MODULE_STEER_MOTOR)
            .withSteerEncoderPort(Constants.BACK_RIGHT_MODULE_STEER_ENCODER)
            .withSteerOffset(Constants.BACK_RIGHT_MODULE_STEER_OFFSET)
            .build();

    m_gyro.reset();
    resetEncoder();

    m_odometry =
        new SwerveDriveOdometry(
            m_kinematics,
            // We have to invert the angle of the NavX so that rotating the robot
            // counter-clockwise makes the angle increase
            Rotation2d.fromDegrees(getInvertedYaw()),
            new SwerveModulePosition[] {
              m_frontLeftModule.getPosition(),
              m_frontRightModule.getPosition(),
              m_backLeftModule.getPosition(),
              m_backRightModule.getPosition()
            });

    shuffleboardTab.addNumber("Gyroscope Angle", () -> getRotation().getDegrees());
    shuffleboardTab.addNumber("Pose X", () -> m_odometry.getPoseMeters().getX());
    shuffleboardTab.addNumber("Pose Y", () -> m_odometry.getPoseMeters().getY());
  }

  /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently
   * facing to the 'forwards' direction.
   */
  public void zeroGyroscope() {
    m_odometry.resetPosition(
        // We have to invert the angle of the NavX so that rotating the robot
        // counter-clockwise makes the angle increase
        Rotation2d.fromDegrees(getInvertedYaw()),
        new SwerveModulePosition[] {
          m_frontLeftModule.getPosition(),
          m_frontRightModule.getPosition(),
          m_backLeftModule.getPosition(),
          m_backRightModule.getPosition()
        },
        new Pose2d(m_odometry.getPoseMeters().getTranslation(), Rotation2d.fromDegrees(0.0)));
  }

  public Rotation2d getRotation() {
    return m_odometry.getPoseMeters().getRotation();
  }

  @Override
  public void periodic() {
    m_odometry.update(
        // We have to invert the angle of the NavX so that rotating the robot
        // counter-clockwise makes the angle increase
        Rotation2d.fromDegrees(getInvertedYaw()),
        new SwerveModulePosition[] {
          m_frontLeftModule.getPosition(),
          m_frontRightModule.getPosition(),
          m_backLeftModule.getPosition(),
          m_backRightModule.getPosition()
        });

    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);

    m_frontLeftModule.set(
        (states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE) * -1,
        states[0].angle.getRadians());
    m_frontRightModule.set(
        (states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE) * -1,
        states[1].angle.getRadians());
    m_backLeftModule.set(
        (states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE) * -1,
        states[2].angle.getRadians());
    m_backRightModule.set(
        states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
        states[3].angle.getRadians());
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    m_chassisSpeeds = chassisSpeeds;
  }

  public void stopDrive() {
    m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
  }

  // public void resetGyro() {
  // m_gyro.reset();
  // }

  // public double getGyroAngle() {
  // return m_gyro.getAngle();
  // }

  private double getInvertedYaw() {
    return (360 - m_gyro.getYaw());
  }

  public double getAverageEncoder() {
    double averagedistance =
        Math.abs(m_backLeftModule.getDriveDistance())
            + Math.abs(m_backRightModule.getDriveDistance())
            + Math.abs(m_frontLeftModule.getDriveDistance())
            + Math.abs(m_frontRightModule.getDriveDistance());
    SmartDashboard.putNumber("Front Left Encoder", Math.abs(m_frontLeftModule.getDriveDistance()));
    SmartDashboard.putNumber(
        "Front Right Encoder", Math.abs(m_frontRightModule.getDriveDistance()));
    SmartDashboard.putNumber("Back Left Encoder", Math.abs(m_backLeftModule.getDriveDistance()));
    SmartDashboard.putNumber("Back Right Encoder", Math.abs(m_backRightModule.getDriveDistance()));
    return averagedistance / 4;
  }

  public void resetEncoder() {
    ((CANSparkMax) m_frontLeftModule.getDriveMotor()).getEncoder().setPosition(0);
    ((CANSparkMax) m_frontRightModule.getDriveMotor()).getEncoder().setPosition(0);
    ((CANSparkMax) m_backLeftModule.getDriveMotor()).getEncoder().setPosition(0);
    ((CANSparkMax) m_backRightModule.getDriveMotor()).getEncoder().setPosition(0);
  }

  public double getAveragePositionMeters() {
    double averagePositionMeters =
        m_backLeftModule.getPosition().distanceMeters
            + m_backRightModule.getPosition().distanceMeters
            + m_frontLeftModule.getPosition().distanceMeters
            + m_frontRightModule.getPosition().distanceMeters;
    return averagePositionMeters / 4;
  }

  public double getPitch() {
    return m_gyro.getPitch();
  }
}
