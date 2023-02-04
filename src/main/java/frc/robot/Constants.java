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

  public static final class AutonomousStartPosition {
    public static final int position1 = 1;
    public static final int position2 = 2;
  }

  public static final class MotorControllerIdleModes {
    public static final int kBrake = 0;
    public static final int kCoast = 1;
  }

  public static final double kForward = 1;
  public static final double kBackward = -1;
  public static final double kLeft = -1;
  public static final double kRight = 1;
  public static final double kTurnLeft = -1;
  public static final double kTurnRight = 1;

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
  public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(0);

  public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 20;
  public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 21;
  public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 22;
  public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(0);

  public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 30;
  public static final int BACK_LEFT_MODULE_STEER_MOTOR = 31;
  public static final int BACK_LEFT_MODULE_STEER_ENCODER = 32;
  public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(0);

  public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 40;
  public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 41;
  public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 42;
  public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(0);

  public static final float PITCH_CLIMBING = 7.0f;
  public static final float PITCH_ENGAGED = 2.5f;

  public static final class DriverOrientation {
    public static final int kDriver = 0;
    public static final int kField = 1;
  }
}
