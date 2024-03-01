// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
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
  /*
   * L1 values:
   * 0.472168
   * 0.051758
   * 0.028809
   * 0.277588
   * L3 values:
   * 0.25
   * 0.168701
   * 0.3396
   * 0.376465
   */
  public static final boolean kUseL1Ratio = true;
  private static final double BACK_RIGHT_OFFSET = kUseL1Ratio ? 0.472168 : 0.246582;
  private static final double BACK_LEFT_OFFSET = kUseL1Ratio ? 0.051758 : 0.173096;
  private static final double FRONT_RIGHT_OFFSET = kUseL1Ratio ? 0.028809: 0.342529;
  private static final double FRONT_LEFT_OFFSET = kUseL1Ratio ? 0.277588: 0.376221;
  public static final double kmaxShooterRPM = 5676.0;
  public static final int TEAM_COLOR_BLUE = 0;
  public static final int TEAM_COLOR_RED = 1;
  public static int kTeamColor = 0;

  public static final boolean kUseJoystick = false; // true for joystick, false for xbox

  public static final class ModuleConstants {
    public static final double kWheelDiameterMeters = Units.inchesToMeters(3.9);
    public static final double kWheelDiameterInches = 3.9;
    public static final double kPTurning = kUseL1Ratio ? 0.34 : 0.34;
    public static final double kITuning = kUseL1Ratio ? 0.0 : 0.0;
    public static final double kDTuning = kUseL1Ratio ? 0.0 : 0.0;

    public static final double kL1Ratio = 1 / 8.14;
    public static final double kL3Ratio = 1 / 6.12;
    
    public static final double kDriveMotorGearRatio = kUseL1Ratio ? kL1Ratio : kL3Ratio;
    public static final double kTurningMotorGearRatio = 1.0 / (150.0 / 7.0);
    public static final double kDriveEncoderRot2Meter =
        kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kDriveEncoderRot2Inch =
        kDriveMotorGearRatio * Math.PI * kWheelDiameterInches;
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60.0;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60.0;
  }

  public static final class DriveConstants {
    public static final String kCanBusName = "rio";
    public static final double kTrackWidth = Units.inchesToMeters(20.5); 
    // Distance between right and left wheels
    public static final double kWheelBase = Units.inchesToMeters(20.5);
    // Distance between front and back wheels
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0),
            new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0),
            new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0),
            new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0));

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
    public static final double kBackRightDriveAbsoluteEncoderOffsetDeg = Rotation2d.fromRotations(BACK_RIGHT_OFFSET).getDegrees() + 180.0;
    // Back Left
    public static final double kBackLeftDriveAbsoluteEncoderOffsetDeg = Rotation2d.fromRotations(BACK_LEFT_OFFSET).getDegrees() + 180.0;
    // Front Right
    public static final double kFrontRightDriveAbsoluteEncoderOffsetDeg = Rotation2d.fromRotations(FRONT_RIGHT_OFFSET).getDegrees() + 180.0;
    // Front Left
    public static final double kFrontLeftDriveAbsoluteEncoderOffsetDeg = Rotation2d.fromRotations(FRONT_LEFT_OFFSET).getDegrees() + 180.0;
                                                              
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

    public static final double kPhysicalMaxSpeedMetersPerSecond = Units.feetToMeters((kUseL1Ratio ? 12.5 : 16.6));
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2.0 * 2.0 * Math.PI;

    public static final double kTeleDriveMaxSpeedMetersPerSecond =
        kPhysicalMaxSpeedMetersPerSecond / 4.0;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond =
        kPhysicalMaxAngularSpeedRadiansPerSecond / 4.0;
    public static final double kTeleDriveMaxAccelerationMetersPerSecondSquared = 3.0;
    public static final double kTeleDriveMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4.0;

    public static final double kPXController =kUseL1Ratio ? 1.5 : 1.0;
    public static final double kPYController =kUseL1Ratio ? 1.5:1.0;
    public static final double kPThetaController = kUseL1Ratio ? 3.0:0.5;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kTeleDriveMaxAngularSpeedRadiansPerSecond,
                        kTeleDriveMaxAngularAccelerationRadiansPerSecondSquared);

  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kCoPilotControllerPort = 1;

    public static final int kDriverYAxis = 1;
    public static final int kDriverXAxis = 0;
    public static final int kDriverRotAxis = 4;
    public static final int kDriverRotAxis_Logitech = 2;
    public static final int kDriverFieldOrientedButtonIdx = 6; // xbox
    public static final int kDriverFieldOrientedButtonIdx_Logitech = 2; // logitech
    //public static final int kZeroHeadingButtonIdx = 2;

    // xbox shoot = driver btn 6
    // xbox intake = driver btn 5

    // logitech shoot = driver btn 1
    // logitech intake = driver btn 3

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
    public static final int kPCMID24V = 1;
    public static final int kShooter1MotorID = 52;
    public static final int kShooter2MotorID = 53;
  }

  public static final class MiscSubsystemConstants {
    public static final boolean kFeederInverted = false;
    public static final boolean kIntakeInverted = false;
  }

  public static final class ColorConstants {
    public static final int BlueHue = 103;
    public static final int YellowHue = 20;
    public static final int RedHue = 0;
  }

  public static final class UltrasonicConstants {
    public static final int kIntake_Analog_Channel = 0;
    public static final int kIntake_PCM_Channel = 0;
    public static final int kFeeder_Analog_Channel = 1;
    public static final int kFeeder_PCM_Channel = 1;
    public static final int kFeeder2_Analog_Channel = 2;
    public static final int kFeeder2_PCM_Channel = 2;
    public static final double kMinRange = 6.0;
    public static final double kMaxRange = 7.0;
  }

  public static final class SpeakerConstants {
      // The speaker has 2 april tags from it
      public static final int kBlueSpeakerAprilTag1Id = 7;
      public static final int kBlueSpeakerAprilTag2Id = 8;
      public static final int kRedSpeakerAprilTag1Id = 4;
      public static final int kRedSpeakerAprilTag2Id = 3;
      // Blue: 7 center (id 1), 8 side (id 2)
      // Red: 4 center (id 1), 3 side (id 2)
  }

  public static final class CameraConstants{
    public static final int kAprilTagCamera = 0;
    public static final int kFrontCamera = 1;
    public static final int kBackCamera = 2;
  }
}
