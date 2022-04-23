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
  public static final class MotorControllerType {
    public static final int kCTRE = 1;
    public static final int kREV = 2;
    public static final int kHybrid = 3;
  }

  public static final class MotorControllerDeviceID {
    public static final int leftLeadDeviceID = 23;
    public static final int leftFollowerDeviceID = 20;

    public static final int rightLeadDeviceID = 22;
    public static final int rightFollowerDeviceID = 21;

    public static final int armDeviceID = 30;

    public static final int intakeDeviceID = 19;

    public static final int telescopingDeviceID = 32;
    public static final int angleLeadDeviceID = 31;
    public static final int angleFollowDeviceID = 33;
  }

  public static final class AutonomousStartPosition {
    public static final int position1 = 1;
    public static final int position2 = 2;
    public static final int position3 = 3;
    public static final int position4 = 4;
  }

  public static final class ArmPositions {
    public static final double lowerLimit = -21.6;
    public static final double upperLimit = 0;
    public static final double shoot = -2.0;
    public static final double shootmin = -1.85;
    public static final double shootmax = -2.15;
  }

  public static final class HangerPositions {
    public static final double forwardLimit = 9;
    public static final double reverseLimit = 0;
  }

  public static final double driveTrainGearRatio = 10.71;
  public static final double driveTrainWheelDiameter = 6;

  public static final class ArmPIDCoeffients {
    public static final double kP = 0.5; // 0.1, 0, -0.1, -2
    public static final double kI = 0; // 1e-4,
    public static final double kD = 0; // 1, 0.5, 0.1
    public static final double kIz = 0;
    public static final double kFF = 0; // 0,
    public static final double kMaxOutput = 0.5;   // raise?
    public static final double kMinOutput = -0.25;  // lower ?
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

  public static final class HangerPIDCoeffients {
    public static final double kP = 0.5;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kIz = 0;
    public static final double kFF = 0;
    public static final double kMaxOutput = 1;
    public static final double kMinOutput = -1;
  }

  public static final class MotorControllerIdleModes {
    public static final int kBrake = 0;
    public static final int kCoast = 1;
  }
}
