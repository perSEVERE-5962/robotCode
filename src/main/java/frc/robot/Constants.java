// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  // public static final int TEAM_COLOR_BLUE = 0;
  // public static final int TEAM_COLOR_RED = 1;
  // public static int kTeamColor = 0;

  public static final boolean kUseJoystick = false; // true for joystick, false for xbox

  /**
   * Types of SDS Swerve Drive Modules
   */
  public static final class SDSModuleType {
    public static final int kL1 = 1; // L1 Gear Ratio
    public static final int kL2 = 2; // L2 Gear Ratio
    public static final int kL3 = 3; // L3 Gear Ratio
    public static final int kCurrent = kL2; // set the module that we are using
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

    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2.0 * 2.0 * Math.PI;

    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond =
        kPhysicalMaxAngularSpeedRadiansPerSecond / 4.0;
    public static final double kTeleDriveMaxAccelerationMetersPerSecondSquared = 3.0;
    public static final double kTeleDriveMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4.0;

    // Autonomous settings
    public static final double kPID_XKP = 2.25
    ; //2.1
    public static final double kPID_XKI = 0.0; 
    public static final double kPID_XKD = 0.0; 
    public static final double kPID_YKP = 2.25; //2.1
    public static final double kPID_YKI = 0.0; 
    public static final double kPID_YKD = 0.0; 
    public static final double KPID_TKP = 2.25; //0.9
    public static final double KPID_TKI = 0.0; 
    public static final double KPID_TKD = 0.0; 

    // Teleop settings
    public static final double kPID_XKP_tele = 5.0;
    public static final double kPID_YKP_tele = 4.0;
    public static final double kPID_TKP_tele = 4.0;

    public static final double kAutoMaxAngularVelocity = 9.0;
    public static final double kAutoMaxAngularAcceleration = 9.0 * 5.0;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kAutoMaxAngularVelocity,
                        kAutoMaxAngularAcceleration);

    // Temporary until there's enough time and testing for a better solution
    public static final class TrajectoryConstants {
      public static final Pose2d kTrajectoryCommonStart = new Pose2d(0, 0, new Rotation2d());
      public static final double kTolerance = 0.1;

      public static final List<Translation2d> kTrajectory1Waypoints = List.of(new Translation2d(0.86, 0));
      public static final Pose2d kTrajectory1End = new Pose2d(2.05, 0, Rotation2d.fromDegrees(0));

      public static final List<Translation2d> kTrajectory2Waypoints = List.of(new Translation2d(0.85, 0));
      public static final Pose2d kTrajectory2End = new Pose2d(1.74, 0, Rotation2d.fromDegrees(0));
    }
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kCoPilotControllerPort = 1;

    public static final int kDriverYAxis = 1;
    public static final int kDriverXAxis = 0;
    public static final int kDriverRotAxis = 4;
    public static final int kDriverRotAxis_Logitech = 2;
    public static final int kDriverFieldOrientedButtonIdx = XboxController.Button.kX.value; // xbox
    public static final int kDriverFieldOrientedButtonIdx_Logitech = 2; // logitech
    //public static final int kZeroHeadingButtonIdx = 2;
    public static final double kDeadband = 0.15; // 0.05;
  }

  public static final class CANDeviceIDs {
    //Intake
    public static final int kIntakeID = 50;
    //Arm
    public static final int kArmID = 52;
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
    // PCM
    public static final int kPCMID24V = 1;
  }

  public static final class UltrasonicConstants {
    public static final double kMinRange = 6.0;
    public static final double kMaxRange = 11.0;
    public static final double kNotDetectedRange = 12.0;
    public static final int kArm_Analog_Channel = 1;
  }

  public static final class CameraConstants{
    //public static final int kAprilTagCamera = 0;
    //public static final int kFrontCamera = 1;
    public static final int kBackCamera = 0;
    public static final double kCameraHeightInches = 16;
    public static final double kCameraTargetHeightInches = 51.75;
    public static final double kCameraPitchDegrees = 3;
  }

  public static final class ArmConstants {
    // PID
    public static final double kP = 0.1; // 0.1, 0, -0.1, -2
    public static final double kI = 0; // 1e-4,
    public static final double kD = 0; // 1, 0.5, 0.1
    public static final double kIz = 0;
    public static final double kFF = 0; // 0,

    // Limits
    public static final double kMaxOutput = 0.4; // extend?
    public static final double kMinOutput = -0.25; // retract ?
    public static final float kLowerSoftLimit = 0; // kReverse
    public static final float kUpperSoftLimit = 15.5f; // kForward
  }
}
