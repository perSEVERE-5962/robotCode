// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

  public static final class AutonomousStartPosition {
    public static final int position1 = 1;
    public static final int position2 = 2;
  }

  public static final class MotorControllerIdleModes {
    public static final int kBrake = 0;
    public static final int kCoast = 1;
  }

  public static final class ModuleConstants {
    public static final double kWheelDiameterMeters = Units.inchesToMeters(3.9);
    public static final double kWheelDiameterInches = 3.9;
    public static final double kDriveMotorGearRatio = 1 / 8.14; // 8.14:1
    public static final double kTurningMotorGearRatio = 1 / (150 / 7); // 150/7:1
    public static final double kDriveEncoderRot2Meter =
        kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kDriveEncoderRot2Inch =
        kDriveMotorGearRatio * Math.PI * kWheelDiameterInches;
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
    public static final double kBackRightDriveAbsoluteEncoderOffsetRad =
        Math.toRadians(182.637 + 180);
    // Back Left
    public static final double kBackLeftDriveAbsoluteEncoderOffsetRad =
        Math.toRadians(8.340 + 180);
    // Front Right
    public static final double kFrontRightDriveAbsoluteEncoderOffsetRad =
        Math.toRadians(13.535 + 180);
    // Front Left
    public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad =
        Math.toRadians(100.723 + 180);

    public static final double kPhysicalMaxSpeedMetersPerSecond = 1.5; // 3.6576; // 12.0 ft/sec
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

    public static final double kTeleDriveMaxSpeedMetersPerSecond =
        kPhysicalMaxSpeedMetersPerSecond / 4;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond =
        kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
    //public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
    //public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
  }

  // public static final class AutoConstants {
  //   public static final double kMaxSpeedMetersPerSecond =
  //       DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
  //   public static final double kMaxAngularSpeedRadiansPerSecond = //
  //       DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
  //   public static final double kMaxAccelerationMetersPerSecondSquared = 3;
  //   public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
  //   public static final double kPXController = 1.5;
  //   public static final double kPYController = 1.5;
  //   public static final double kPThetaController = 3;

  //   public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
  //       new TrapezoidProfile.Constraints(
  //           kMaxAngularSpeedRadiansPerSecond, kMaxAngularAccelerationRadiansPerSecondSquared);
  // }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kCoPilotControllerPort = 1;

    public static final int kDriverYAxis = 1;
    public static final int kDriverXAxis = 0;
    public static final int kDriverRotAxis = 4;
    public static final int kDriverFieldOrientedButtonIdx = 1;
    public static final int kZeroHeadingButtonIdx = 2;

    public static final double kDeadband = 0.15; // 0.05;
  }

  public static final float Vert1 = 1;
  public static final float Vert2 = 2;
  public static final float Vert3 = 3;
  public static final float Hor1 = 1;
  public static final float Hor2 = 2;
  public static final float Hor3 = 3;

  public static double PITCH_OFFSET = 0.0;
  public static final float PITCH_CLIMBING = 10.0f;
  public static final float PITCH_LEVEL = 2.0f;

  public static double YAW_OFFSET = 0.0;

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
    public static final int kLiftLeadID = 50;
    public static final int kLiftFollowID = 51;
    public static final int kReachID = 60;
    public static final int kWristID = 61;
    // pmc
    public static final int kPCMID = 3;
  }

  public static final class LiftConstants {
    public static final double kP = 0.5; // 0.1, 0, -0.1, -2
    public static final double kI = 0; // 1e-4,
    public static final double kD = 0; // 1, 0.5, 0.1
    public static final double kIz = 0;
    public static final double kFF = 0; // 0,
    public static final double kMaxOutput = 0.5; // raise?
    public static final double kMinOutput = -0.25; // lower ?
    public static final float kLowerSoftLimit = 0; // kReverse
    public static final float kRaiseSoftLimit = 21; // kForward
    public static final double kPos1 = 8; // cone grid position 1
    public static final double kPos2 = 8; // cone grid position 2
    public static final double kPos3 = 21; // cone grid position 3
    // public static final double[] position = {kPos1, kPos2, kPos3};
    public static final double kSubStation = 21; // double substation/cone collection
  }

  public static final class ReachConstants {
    public static final double kP = 0.5; // 0.1, 0, -0.1, -2
    public static final double kI = 0; // 1e-4,
    public static final double kD = 0; // 1, 0.5, 0.1
    public static final double kIz = 0;
    public static final double kFF = 0; // 0,
    public static final double kMaxOutput = 0.5; // extend?
    public static final double kMinOutput = -0.25; // retract ?
    public static final float kRetractSoftLimit = 0; // kReverse
    public static final float kExtendSoftLimit = 15.5f; // kForward
    public static final double kPos1 = 0; // cone grid position 1
    public static final double kPos2 = 0; // cone grid position 2
    public static final double kPos3 = 15.5; // cone grid position 3
    // public static final double[] position = {kPos1, kPos2, kPos3};
    public static final double kSubStation = 0; // double substation/cone collection
  }

  public static final class WristConstants {
    public static final double kP = 0.5; // 0.1, 0, -0.1, -2
    public static final double kI = 0; // 1e-4,
    public static final double kD = 0; // 1, 0.5, 0.1
    public static final double kIz = 0;
    public static final double kFF = 0; // 0,
    public static final double kMaxOutput = 0.5; // extend?
    public static final double kMinOutput = -0.25; // retract ?
    public static final float kLowerSoftLimit = -55; // kReverse
    public static final float kRaiseSoftLimit = 0; // kForward
    public static final float kSubStation = -46; // half way down
  }

  public static final class tabs {
    public static final String kManipulators = "Manipulators";
    public static final String kAngle = "Angle";
    public static final String kLineDetector = "Line Detector";
    public static final String kSwerveSubsystem = "Swerve Subsystem";
  }

  public static final class GripperConstants {
    public static final int kSensorChannel = 0;
    public static final int kSol1_Channel1 = 4;
    public static final int kSol1_Channel2 = 5;
    public static final int kSol2_Channel1 = 2; // no longer used
    public static final int kSol2_Channel2 = 3; // no longer used
    public static final int kSol3_Channel1 = 0; // no longer used
    public static final int kSol3_Channel2 = 1; // no longer used
    public static final double kMinRange = 5;
    public static final double kMaxRange = 8;
    
  }

  // Pos 1: 66 inches
  //
}
