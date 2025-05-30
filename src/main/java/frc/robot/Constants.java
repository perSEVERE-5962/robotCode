// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import com.revrobotics.spark.SparkMaxAlternateEncoder;
import com.revrobotics.spark.config.AlternateEncoderConfig.Type;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final int TEAM_COLOR_BLUE = 0;
  public static final int TEAM_COLOR_RED = 1;
  public static int kTeamColor = 0;

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
    public static final double kTrackWidth = Units.inchesToMeters(20.75);
    // Distance between right and left wheels
    public static final double kWheelBase = Units.inchesToMeters(20.75);
    // Distance between front and back wheels
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0), // FL
        new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0), // FR
        new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0), // BL
        new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0)); // BR

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

    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond
        / 4.0;
    public static final double kTeleDriveMaxAccelerationMetersPerSecondSquared = 3.0;
    public static final double kTeleDriveMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4.0;

    // Autonomous settings
    public static final double kPID_XKP = 0.8; // 2.1
    public static final double kPID_XKI = 0.1;
    public static final double kPID_XKD = 0.0;
    public static final double kPID_XKIzone=0.8;
    public static final double kPID_YKP = 0.001; // 2.1
    public static final double kPID_YKI = 0.001;
    public static final double kPID_YKD = 0.0;
    public static final double kPID_YKIzone=0.0;
    public static final double KPID_TKP = 0.001; // 0.9
    public static final double KPID_TKI = 0.001;
    public static final double KPID_TKD = 0.0;
    public static final double kPID_TKIzone=0.0;

    // Teleop settings
    public static final double kPID_XKP_tele = 5.0;
    public static final double kPID_YKP_tele = 4.0;
    public static final double kPID_TKP_tele = 4.0;

    public static final double kAutoMaxAngularVelocity = 4.0;
    public static final double kAutoMaxAngularAcceleration = 4.0 * 5.0;

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
    public static final int kTestingControllerPort = 2;
    public static final int kDriverYAxis = 1;
    public static final int kDriverXAxis = 0;
    public static final int kDriverRotAxis = 4;
    public static final int kDriverRotAxis_Logitech = 2;
    public static final int kDriverFieldOrientedButtonIdx = XboxController.Button.kX.value; // xbox
    public static final int kDriverFieldOrientedButtonIdx_Logitech = 2; // logitech
    // public static final int kZeroHeadingButtonIdx = 2;
    public static final double kDeadband = 0.15; // 0.05;
  }

  public static final class CANDeviceIDs {
    // Intake
    public static final int kIntakeID = 51;
    //public static final int kFollowerID = 53;
    
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

  public static final class CameraConstants {
    // public static final int kAprilTagCamera = 0;
    // public static final int kFrontCamera = 1;
    public static final int kBackCamera = 0;
    public static final double kCameraHeightInches = 16;
    public static final double kCameraTargetHeightInches = 51.75;
    public static final double kCameraPitchDegrees = 3;
  }

  public static final class ReachConstants {
    public static final int kReachID = 50;
    // PID
    public static final double kP = 1; 
    public static final double kI = 0; 
    public static final double kD = 0; 
    public static final double kIz = 0;
    public static final double kFF = 0; 

    // feedforward
    public static final double ks = 0.0;
    public static final double kv = 0.0;
    public static final double kPostionConversionFactor = 0; // (end angle - start angle) / value at end angle

    // Limits
    public static final double kMaxOutput = 0.8; // extend?
    public static final double kMinOutput = -0.8; // retract ?
    public static final float kLowerSoftLimit = 0; // kReverse
    public static final float kUpperSoftLimit = -73.3f; // kForward
  }

  public static final class PivotConstants {
    public static final int kPivotID = 52;
    public static final int kFollowerID = 60;
    // PID
    public static final double kP = 15.0; 
    public static final double kI = 0.0000; 
    public static final double kD = 0;
    public static final double kIz = 0.0;
    public static final double kFF = 0; 

    // feedforward
    public static final double ks = 0.0;
    public static final double kv = 0.0;
    public static final double kPostionConversionFactor = 0; // (end angle - start angle) / value at end angle

    // Limits
    public static final double kMaxOutput = 0.8; // extend?
    public static final double kMinOutput = -0.8; // retract ?
    public static final float kLowerSoftLimit = 0.53f; // kReverse
    public static final float kUpperSoftLimit = 0.84f; // kForward

    // absolute encoder
    public static final int kTicks = 8192;
    public static final float ticksPerDeg = (float) kTicks / 360.0f;
    public static final Type kEncoderType = Type.kQuadrature;
//offsets
    public static final double koffSet=0.69;

  }

  public static final class WristConstants {
    public static final int kWristID = 53;//53

    // PID
    //L1 was 15, 0, 0.1
    public static final double kP = 8; 
    public static final double kI = 0; 
    public static final double kD = 0; 
    public static final double kIz = 0;
    public static final double kFF = 0; 

    // feedforward
    public static final double ks = 0.0;
    public static final double kv = 0.0;
    public static final double kPostionConversionFactor = 0; // (end angle - start angle) / value at end angle

    // Limits
    public static final double kMaxOutput = 0.4; // extend?
    public static final double kMinOutput = -0.4; // retract ?
    public static final float kLowerSoftLimit = 0.195f;//0; // kReverse
    public static final float kUpperSoftLimit =  0.9f;//30.5f; // kForward
    public static final float kL1Limit = 1.5f;
    public static final float kL2Limit = 1.6f;
    public static final float kL3Limit = 1.9f;
    public static final float kIntakeLimit = 2.0f;
    //Offsets
    public static final double koffSet=0.0;
  }

  public static final class ScoringConstants {
    public static final double[][] postions = { 
        { 17.5, 0.37, 0.806},   //L1 (1.Reach, 2.Wrist, 3.Pivot)(0,-24.2,0.69)
        { 15.0, 0.76, 0.532},   //L2 (1.Reach, 2.Wrist, 3.Pivot)
        { 50.0, 0.84, 0.431},   //L3 (1.Reach, 2.Wrist, 3.Pivot)pivot=0.52
        { 15,0.195,0.530},   //L4
        { 11.57, 0.467, 0.663},//corall Station
      {15,0.195,0.530},// Straight up
      {15,0.195,0.530}, // Starting position
      {15,0.195,0.530}, // auto L1
      {15,0.195,0.530},//testing PID values
      {15,0.195,0.530}, //intialize move
      {15,0.195,0.530}, //overshoot for reset fuuction 
      {25.5, 0.365, 0.160}//Coral bucket 
    }; // Reset function

          public static final int kL1=0;
    public static final int kL2=1;
    public static final int kL3=2;
    public static final int kL4=3;
    public static final int kStation=4;
    public static final int kStraightUp=5;
    public static final int kStartPos=6;
    public static final int kAutoL1=7;
    public static final int kReach=0;
    public static final int kWrist=1;
    public static final int kPivot=2;
    public static final int kIntial = 9;
    public static final int kOvershoot = 10;
    public static final int kWristandReachBucket = 11; 
  }
  public static final class PhotonVisionConstant {
    public static final double kArmOffset = 0.0406; //meters
    public static final double kTargetXPos = 0.56; //meters
    public static final double kTagToPost = 0.1651; //meters
    public static final double CameraOffsets = 0.15;

    public static final double YoffsetsForCameras=0.15;
    public static AprilTagFieldLayout fieldLayout =AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
     public static final class FrontLeft{
       public static String name="Arducam_OV2311_USB_Camera";
       public static Transform3d cameraposeFrontLeft = new Transform3d(new Translation3d(0.31,0.17,0.32),new Rotation3d(0,0,0));
     }
     public static final class FrontRight{
       public static String name="5962_camera_1";
       public static Transform3d cameraposeFrontRight = new Transform3d(new Translation3d(0,0,0),new Rotation3d(0,0,0));
     }
     public static final class BackLeft{
       public static String name="";
       public static Transform3d cameraposeBackLeft = new Transform3d(new Translation3d(0,0,0),new Rotation3d(0,0,0));
     }
    public static final class BackRight{
       public static String name="Arducam_OV2311_USB_Camera (1)";
       public static Transform3d cameraposeBackRight = new Transform3d(new Translation3d(0,0,0),new Rotation3d(0,0,0));

     }

    
     public static PhotonCamera[] CameraNames={
     new PhotonCamera(FrontLeft.name),
     new PhotonCamera(FrontRight.name),
     new PhotonCamera(BackLeft.name),
     new PhotonCamera(BackRight.name)};
     public static Transform3d[] CameraPoses={
       FrontLeft.cameraposeFrontLeft,
       FrontRight.cameraposeFrontRight,
    BackLeft.cameraposeBackLeft,
       BackRight.cameraposeBackRight};
        } 
       
      
      public static final class StartingPos { 
         
      


   public static double statingRotation2dx=0;
   public static  double statingRotation2dy=0;
   public static  double startingTranslation2dx=0;
   public static  double startingTranslation2dy=0;
   //Not a Constant
   public static Translation2d translation2d = new Translation2d(0, 0);
   public static Rotation2d rotation2d = new Rotation2d(0);
    public static Transform2d  poseTransform2d=new Transform2d(translation2d, rotation2d);

    public static Translation2d translation2d_2= new Translation2d(0, 0);
   public static Rotation2d rotation2d_2= new Rotation2d(0);
    public static Transform2d  poseTransform2d_2=new Transform2d(translation2d, rotation2d);
//April tags
public static final double[][] apriltagsToReef = { 
  { 20, 11},   //Right
  { 21,10},   //center
  { 22, 9}};//left
  
  }
  public static final class ColorConstants {
    public static final int BlueHue = 103;
    public static final int YellowHue = 20;
    public static final int RedHue = 0;
    public static final int GreenHue = 60;
}
}

