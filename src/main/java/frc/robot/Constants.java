// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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
  private static final double BACK_RIGHT_OFFSET = 0.478760;
  private static final double BACK_LEFT_OFFSET = 0.051025;
  private static final double FRONT_RIGHT_OFFSET = 0.034180;
  private static final double FRONT_LEFT_OFFSET = 0.278320;

  public static final int TEAM_COLOR_RED = 1;
  public static final int TEAM_COLOR_BLUE = 2;
  public static int kTeamColor = TEAM_COLOR_RED;

  public static final class ModuleConstants {
    public static final double kWheelDiameterMeters = Units.inchesToMeters(3.9);
    public static final double kWheelDiameterInches = 3.9;
    public static final double kPTurning = 0.5;

    // L1 swerve
    public static final class L1 {
      public static final double kDriveMotorGearRatio = 1 / 8.14; // 8.14:1
      public static final double kTurningMotorGearRatio = 1 / (150 / 7); // 150/7:1
      public static final double kDriveEncoderRot2Meter =
          kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
      public static final double kDriveEncoderRot2Inch =
          kDriveMotorGearRatio * Math.PI * kWheelDiameterInches;
      public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
      public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
      public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
    }

    // L3 swerve
    public static final class L3 {
      public static final double kDriveMotorGearRatio = 1 / 6.12; // 6.12:1
      public static final double kTurningMotorGearRatio = 1 / (150 / 7); // 150/7:1
      public static final double kDriveEncoderRot2Meter =
          kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
      public static final double kDriveEncoderRot2Inch =
          kDriveMotorGearRatio * Math.PI * kWheelDiameterInches;
      public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
      public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
      public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
    }
  }

  public static final class DriveConstants {
    public static final String kCanBusName = "rio";
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

    public static final boolean kFrontLeftTurningEncoderReversed = false;
    public static final boolean kBackLeftTurningEncoderReversed = false;
    public static final boolean kFrontRightTurningEncoderReversed = false;
    public static final boolean kBackRightTurningEncoderReversed = false;

    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kBackLeftDriveEncoderReversed = false;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kBackRightDriveEncoderReversed = false;

    public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
    public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
    public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
    public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

    // Back Right
    public static final double kBackRightDriveAbsoluteEncoderOffsetDeg = Rotation2d.fromRotations(BACK_RIGHT_OFFSET).getDegrees() + 180;
    // Back Left
    public static final double kBackLeftDriveAbsoluteEncoderOffsetDeg = Rotation2d.fromRotations(BACK_LEFT_OFFSET).getDegrees() + 180;
    // Front Right
    public static final double kFrontRightDriveAbsoluteEncoderOffsetDeg = Rotation2d.fromRotations(FRONT_RIGHT_OFFSET).getDegrees() + 180;
    // Front Left
    public static final double kFrontLeftDriveAbsoluteEncoderOffsetDeg = Rotation2d.fromRotations(FRONT_LEFT_OFFSET).getDegrees() + 180;

    // Back Right
    public static final double kBackRightDriveAbsoluteEncoderOffsetRad =
        Math.toRadians(kBackRightDriveAbsoluteEncoderOffsetDeg);
    // Back Left
    public static final double kBackLeftDriveAbsoluteEncoderOffsetRad =
        Math.toRadians(kBackLeftDriveAbsoluteEncoderOffsetDeg);
    // Front Right
    public static final double kFrontRightDriveAbsoluteEncoderOffsetRad =
        Math.toRadians(kFrontRightDriveAbsoluteEncoderOffsetDeg);
    // Front Left
    public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad =
        Math.toRadians(kFrontLeftDriveAbsoluteEncoderOffsetDeg);

    public static final double kPhysicalMaxSpeedMetersPerSecond = 1.5; // 3.6576; // 12.0 ft/sec
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

    public static final double kTeleDriveMaxSpeedMetersPerSecond =
        kPhysicalMaxSpeedMetersPerSecond / 4;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond =
        kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
    // public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
    // public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kCoPilotControllerPort = 1;

    public static final int kDriverYAxis = 1;
    public static final int kDriverXAxis = 0;
    public static final int kDriverRotAxis = 4;
    public static final int kDriverRotAxis_Logitech = 2;
    public static final int kDriverFieldOrientedButtonIdx = 6;
    public static final int kDriverFieldOrientedButtonIdx_Logitech = 1;
    public static final int kZeroHeadingButtonIdx = 2;

    public static final double kDeadband = 0.15; // 0.05;
  }

  public static final class CANDeviceIDs {
    // drive motors
    public static final int kFrontLeftDriveMotorID = 40;
    public static final int kBackLeftDriveMotorID = 20;
    public static final int kFrontRightDriveMotorID = 30;
    public static final int kBackRightDriveMotorID = 10;
    // steer motors
    public static final int kFrontLeftTurningMotorID = 41;
    public static final int kBackLeftTurningMotorID = 21;
    public static final int kFrontRightTurningMotorID = 31;
    public static final int kBackRightTurningMotorID = 11;
    // absolute encoders
    public static final int kFrontLeftDriveAbsoluteEncoderID = 42;
    public static final int kBackLeftDriveAbsoluteEncoderID = 22;
    public static final int kFrontRightDriveAbsoluteEncoderID = 32;
    public static final int kBackRightDriveAbsoluteEncoderID = 12;
    // manipulator
    public static final int kIntakeMotorID = 50;
    public static final int kFeederMotorID = 51;
    public static final int kPCMID24V = 0;
  }

  public static final class SpeakerConstants {
    // The speaker has 2 april tags from it
    public static final int kSpeakerBlueAprilTag1Id = 7;
    public static final int kSpeakerBlueAprilTag2Id = 8;
    public static final int kSpeakerRedAprilTag1Id = 4;
    public static final int kSpeakerRedAprilTag2Id = 3;
  }

  public static final class ColorConstants {
    public static final int BlueHue = 103;
    public static final int YellowHue = 20;
  public static final class UltrasonicConstants {
    public static final int kIntake_Analog_Channel = 0;
    public static final int kIntake_PCM_Channel = 0;
 public static final int kFeeder_Analog_Channel = 1;
    public static final int kFeeder_PCM_Channel = 1;
    public static final double kMinRange = 6;
    public static final double kMaxRange = 7;
  }
}
