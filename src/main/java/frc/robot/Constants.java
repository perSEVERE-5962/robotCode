// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  /**
   * Types of motor controllers
   *
   * <p>kCTRE represents {@link com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX} with a {@link
   * com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX} follower
   *
   * <p>kREV represents {@link com.revrobotics.CANSparkMax} with a {@link
   * com.revrobotics.CANSparkMax} follower
   */
  public static final class ChassisType {
    public static final int kCTRE = 1; // Kit chassis with Talon SPX and Victor SPX & Cims
    public static final int kREV = 2; // Kit chassis with Rev Spark Max & Neos
    public static final int kHybrid = 3; // Kit Chassis with Rev Spark Max, Neos, and Cims
    public static final int kRomi = 4; // Romi robot
    public static final int kSwerve = 5; // MK4i Swerve Chassis
  }

  public static final class CompetitionRobot {
    public static final int leftLeadDeviceID = 23;
    public static final int leftFollowerDeviceID = 20;

    public static final int rightLeadDeviceID = 22;
    public static final int rightFollowerDeviceID = 21;
    public static final double kDriveTrainGearRatio = 10.71;
    public static final double kDriveTrainWheelDiameter = 6;
  }

  public static final class Romi {
    public static final int leftDeviceID = 0;
    public static final int rightDeviceID = 1;
    public static final int leftEncoderChannelA = 4;
    public static final int leftEncoderChannelB = 5;
    public static final int rightEncoderChannelA = 6;
    public static final int rightEncoderChannelB = 7;
    public static final double kCountsPerRevolution = 1440.0;
    public static final double kWheelDiameterMeter = 0.07;
  }

  public static final class DrivePIDCoeffients {
    public static final double kP = 0.1;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kIz = 0;
    public static final double kFF = 0;
    public static final double kMaxOutput = 0.5;
    public static final double kMinOutput = -0.5;
  }

  public static final class MotorControllerIdleModes {
    public static final int kBrake = 0;
    public static final int kCoast = 1;
  }

  // Swerve Drive Stuff
  /**
   * The left-to-right distance between the drivetrain wheels
   *
   * <p>Should be measured from center to center.
   */
  public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.632; // 24.875 inches
  /**
   * The front-to-back distance between the drivetrain wheels.
   *
   * <p>Should be measured from center to center.
   */
  public static final double DRIVETRAIN_WHEELBASE_METERS = 0.632; // 24.875 inches

  // public static final int DRIVETRAIN_PIGEON_ID = 0;

  public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 10;
  public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 11;
  public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 12;
  public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(170.77);

  public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 20;
  public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 21;
  public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 22;
  public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(14.50);

  public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 30;
  public static final int BACK_LEFT_MODULE_STEER_MOTOR = 31;
  public static final int BACK_LEFT_MODULE_STEER_ENCODER = 32;
  public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(12.83);

  public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 40;
  public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 41;
  public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 42;
  public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(37.44);
  /**
   * The maximum voltage that will be delivered to the drive motors.
   *
   * <p>This can be reduced to cap the robot's maximum speed. Typically, this is useful during
   * initial testing of the robot.
   */
  public static final double MAX_VOLTAGE = 3; // 12.0;

  /**
   * The maximum velocity of the robot in meters per second.
   *
   * <p>This is a measure of how fast the robot should be able to drive in a straight line.
   */
  public static final double MAX_VELOCITY_METERS_PER_SECOND =
      5880.0
          / 60.0
          / ((14.0 / 50.0)
              * (25.0 / 19.0)
              * (15.0 / 45.0)) // SdsModuleConfigurations.MK4I_L1.getDriveReduction()
          * 0.10033 // SdsModuleConfigurations.MK4I_L1.getWheelDiameter()
          * Math.PI;
  /**
   * The maximum angular velocity of the robot in radians per second.
   *
   * <p>This is a measure of how fast the robot can rotate in place.
   */
  // Here we calculate the theoretical maximum angular velocity. You can also
  // replace this with a measured amount.
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND =
      MAX_VELOCITY_METERS_PER_SECOND
          / Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);


  public static final class DriverOrientation {
    public static final int kDriver = 0;
    public static final int kField = 1;
  }

}
