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
  }

  public static final class MotorControllerDeviceID {
    public static final int leftLeadDeviceID = 23;
    public static final int leftFollowerDeviceID = 20;

    public static final int rightLeadDeviceID = 22;
    public static final int rightFollowerDeviceID = 21;

    public static final int armDeviceID = 30;
  }
}
