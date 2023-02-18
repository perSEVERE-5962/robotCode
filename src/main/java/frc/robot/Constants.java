// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class AutonomousStartPosition {
    public static final int position1 = 1;
    public static final int position2 = 2;
  }

  public static final class MotorControllerIdleModes {
    public static final int kBrake = 0;
    public static final int kCoast = 1;
  }

  public static final class ModuleConstants {
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kDriveMotorGearRatio = 1 / 8.14; // 8.14:1
    public static final double kTurningMotorGearRatio = 1 / (150 / 7); // 150/7:1
    public static final double kDriveEncoderRot2Meter =
        kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
    public static final double kPTurning = 0.5;
  }

  public static final class DriveConstants {

    public static final double kTrackWidth = Units.inchesToMeters(24.857);
    // Distance between right and left wheels
    public static final double kWheelBase = Units.inchesToMeters(24.857);
    // Distance between front and back wheels
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

    public static final int kFrontLeftDriveMotorPort = 40;
    public static final int kBackLeftDriveMotorPort = 20;
    public static final int kFrontRightDriveMotorPort = 30;
    public static final int kBackRightDriveMotorPort = 10;

    public static final int kFrontLeftTurningMotorPort = 41;
    public static final int kBackLeftTurningMotorPort = 21;
    public static final int kFrontRightTurningMotorPort = 31;
    public static final int kBackRightTurningMotorPort = 11;

    public static final boolean kFrontLeftTurningEncoderReversed = true;
    public static final boolean kBackLeftTurningEncoderReversed = true;
    public static final boolean kFrontRightTurningEncoderReversed = true;
    public static final boolean kBackRightTurningEncoderReversed = true;

    public static final boolean kFrontLeftDriveEncoderReversed = true;
    public static final boolean kBackLeftDriveEncoderReversed = true;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kBackRightDriveEncoderReversed = false;

    public static final int kFrontLeftDriveAbsoluteEncoderPort = 42;
    public static final int kBackLeftDriveAbsoluteEncoderPort = 22;
    public static final int kFrontRightDriveAbsoluteEncoderPort = 32;
    public static final int kBackRightDriveAbsoluteEncoderPort = 12;

    public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
    public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
    public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
    public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

    public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = Math.toRadians(0);
    public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = Math.toRadians(0);
    public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = Math.toRadians(0);
    public static final double kBackRightDriveAbsoluteEncoderOffsetRad = Math.toRadians(0);

    public static final double kPhysicalMaxSpeedMetersPerSecond = 3.6576; // 12.0 ft/sec
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

    public static final double kTeleDriveMaxSpeedMetersPerSecond =
        kPhysicalMaxSpeedMetersPerSecond / 4;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
        kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond =
        DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
    public static final double kMaxAngularSpeedRadiansPerSecond = //
        DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
    public static final double kPXController = 1.5;
    public static final double kPYController = 1.5;
    public static final double kPThetaController = 3;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularAccelerationRadiansPerSecondSquared);
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kCoPilotControllerPort = 1;

    public static final int kDriverYAxis = 1;
    public static final int kDriverXAxis = 0;
    public static final int kDriverRotAxis = 4;
    public static final int kDriverFieldOrientedButtonIdx = 1;
    public static final int kZeroHeadingButtonIdx = 2;

    public static final double kDeadband = 0.05;
  }

  public static final float Vert1 = 1;
  public static final float Vert2 = 2;
  public static final float Vert3 = 3;
  public static final float Hor1 = 1;
  public static final float Hor2 = 2;
  public static final float Hor3 = 3;

  public static final float PITCH_CLIMBING = 7.0f;
  public static final float PITCH_ENGAGED = 2.5f;

  public static final class MotorControllerDeviceID {
    public static final int LinearSlideDeviceID = 30;
  }

  public static final class LinearSlidePIDCoefficients {
    public static final double kP = 0.1; // 0.1, 0, -0.1, -2
    public static final double kI = 0; // 1e-4,
    public static final double kD = 0; // 1, 0.5, 0.1
    public static final double kIz = 0;
    public static final double kFF = 0; // 0,
    public static final double kMaxOutput = 0.5; // raise?
    public static final double kMinOutput = -0.25; // lower ?
  }
}
