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
    public static final double kWheelDiameterInch = 2.75591; // 70 mm
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
}
