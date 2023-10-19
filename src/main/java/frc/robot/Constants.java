// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxAlternateEncoder.Type;
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
  private static final double BACK_RIGHT_OFFSET = 170.684;
  private static final double BACK_LEFT_OFFSET = 18.545;
  private static final double FRONT_RIGHT_OFFSET = 12.393;
  private static final double FRONT_LEFT_OFFSET = 100.020;

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
    public static final double kBackRightDriveAbsoluteEncoderOffsetDeg = BACK_RIGHT_OFFSET + 180;
    // Back Left
    public static final double kBackLeftDriveAbsoluteEncoderOffsetDeg = BACK_LEFT_OFFSET + 180;
    // Front Right
    public static final double kFrontRightDriveAbsoluteEncoderOffsetDeg = FRONT_RIGHT_OFFSET + 180;
    // Front Left
    public static final double kFrontLeftDriveAbsoluteEncoderOffsetDeg = FRONT_LEFT_OFFSET + 180;

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
    public static final int kDriverRotAxis_Logitech = 2;
    public static final int kDriverFieldOrientedButtonIdx = 6;
    public static final int kDriverFieldOrientedButtonIdx_Logitech = 1;
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
    public static final int kRollerId = 62;
    // pmc
    public static final int kPCMID = 3;
    public static final int kPCMID24V = 0;
  }

  public static final class LiftConstants {
    // PID
    public static final double kP = 0.5; // 0.1, 0, -0.1, -2
    public static final double kI = 0; // 1e-4,
    public static final double kD = 0; // 1, 0.5, 0.1
    public static final double kIz = 0;
    public static final double kFF = 0; // 0,

    // Limits
    public static final double kMaxOutput = 0.5; // raise?
    public static final double kMinOutput = -0.25; // lower ?
    public static final float kLowerSoftLimit = 0; // kReverse
    public static final float kRaiseSoftLimit = 21; // kForward

    // Cone
    public static final double kConeScorePos1 = 0;
    public static final double kConeScorePos2 = 0;
    public static final double kConeScorePos3 = 20;
    public static final double kConeScoreAuto = 20;
    public static final double kConeSubStation = 8; // double substation

    // Cubes (Unused)
    public static final double kCubeScorePos1 = 8;
    public static final double kCubeScorePos2 = 8;
    public static final double kCubeScorePos3 = 21;
    public static final double kCubeSubStation = 8; // double substation
  }

  public static final class ReachConstants {
    // PID
    public static final double kP = 0.1; // 0.1, 0, -0.1, -2
    public static final double kI = 0; // 1e-4,
    public static final double kD = 0; // 1, 0.5, 0.1
    public static final double kIz = 0;
    public static final double kFF = 0; // 0,

    // Limits
    public static final double kMaxOutput = 0.4; // extend?
    public static final double kMinOutput = -0.25; // retract ?
    public static final float kRetractSoftLimit = 0; // kReverse
    public static final float kExtendSoftLimit = 15.5f; // kForward

    // Cone
    public static final double kConeScorePos1 = 0;
    public static final double kConeScorePos2 = 0;
    public static final double kConeScorePos3 = 15;
    public static final double kConeScoreAuto = 15;
    public static final double kConeSubStation = 0; // double substation

    // Cubes (Unused)
    public static final double kCubeScorePos1 = 0;
    public static final double kCubeScorePos2 = 0;
    public static final double kCubeScorePos3 = 15.5;
    public static final double kCubeSubStation = 0; // double substation
  }

  public static final float DEGREES(float val) {
    return ((val * (8192.0f / 360.0f)) / 8192.0f);
  }

  public static final class WristConstants {
    // PID
    public static final double kP = 10; // 0.5; // 0.1, 0, -0.1, -2
    public static final double kI = 0; // 1e-4,
    public static final double kD = 0; // 1, 0.5, 0.1
    public static final double kIz = 0;
    public static final double kFF = 0; // 0,

    // Globals
    // Insert degrees for the literal numbers
    public static final int kTicks = 8192;
    public static final float ticksPerDeg = (float) kTicks / 360.0f;
    public static final Type kEncoderType = SparkMaxAlternateEncoder.Type.kQuadrature;
    public static final double kManualVoltage = 8;

    // Limits
    public static final double kMaxOutput = 1.0; // 0.5; // extend?
    public static final double kMinOutput = -1.0; // -0.25; // retract ?
    public static final float kRaiseSoftLimit = 0; // kForward // How far back the wrist can go
    public static final float kClearChain = DEGREES(10); // Clear chain for the cone
    public static final float kLowerSoftLimit =
        DEGREES(90); // kReverse // How far forward the wrist can go

    // Cones
    public static final float kConeScorePosition1 = DEGREES(72);
    public static final float kConeScorePosition2 = DEGREES(72);
    public static final float kConeScorePosition3 = DEGREES(87);
    public static final float kConeScoreAuto = DEGREES(75);
    public static final float kConeSubStation = DEGREES(66);

    // Cubes (Unused)
    public static final float kCubeScorePosition1 = DEGREES(75);
    public static final float kCubeScorePosition2 = DEGREES(75);
    public static final float kCubeScorePosition3 = DEGREES(70);
    public static final float kCubeSubStation = DEGREES(70);
  }

  public static final class RollerConstants {
    public static final double kMinOutput = -1;
    public static final double kMaxOutput = 1;
    public static final double kMaxVoltage = 5;
    // Values assume LT is intake
    public static final int kIntakeCone = 1;
    public static final int kIntakeCube = -1;
  }

  public static final class tabs {
    public static final String kManipulators = "Manipulators";
    public static final String kAngle = "Angle";
    public static final String kLineDetector = "Line Detector";
    public static final String kSwerveSubsystem = "Swerve Subsystem";
  }

  /** Unused */
  public static final class GripperConstants {
    public static final int kSol1_Channel1 = 4;
    public static final int kSol1_Channel2 = 5;
    public static final int kSol2_Channel1 = 2; // no longer used
    public static final int kSol2_Channel2 = 3; // no longer used
    public static final int kSol3_Channel1 = 0; // no longer used
    public static final int kSol3_Channel2 = 1; // no longer used
  }

  public static final class UltrasonicConstants {
    public static final int kSensor_Analog_Channel = 0;
    public static final int kSensor_PCM_Channel = 6; // analog ultrasonic sensor
    public static final int kTrainSensor_PCM_Channel = 7; // used to train the sensor
    public static final double kMinRange = 6;
    public static final double kMaxRange = 7;
  }
}