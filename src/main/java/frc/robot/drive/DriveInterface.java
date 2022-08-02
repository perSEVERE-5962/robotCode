// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

/** Add your docs here. */
public interface DriveInterface {
  /**
   * Tank drive method for differential drive platform. The calculated values will be squared to
   * decrease sensitivity at low speeds.
   *
   * @param leftSpeed The robot's left side speed along the X axis [-1.0..1.0]. Forward is positive.
   * @param rightSpeed The robot's right side speed along the X axis [-1.0..1.0]. Forward is
   *     positive.
   */
  void tankDrive(double leftSpeed, double rightSpeed);

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  void tankDriveVolts(double leftVolts, double rightVolts);

  /**
   * Arcade drive method for differential drive platform. The calculated values will be squared to
   * decrease sensitivity at low speeds.
   *
   * @param xSpeed The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
   * @param zRotation The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is
   *     positive.
   */
  void arcadeDrive(double xSpeed, double zRotation);

  /** reset the encoders */
  void resetEncoders();

  /**
   * get the distance of the left encoder in meters
   *
   * @return the left encoder distance in meters
   */
  double getLeftEncoderDistance();

  /**
   * get the distance of the left encoder in meters
   *
   * @return the left encoder distance in meters
   */
  double getRightEncoderDistance();

  /**
   * get the distance of the left & right encoders averaged together in meters
   *
   * @return the distance of the left & right encoders averaged together in meters
   */
  double getAverageEncoderDistance();

  /**
   * Sets the ramp rate for open loop control modes This is the maximum rate at which the motor
   * controller's output is allowed to change
   *
   * @param rate - Time in seconds to go from 0 to full throttle
   */
  void setRampRate(double rate);

  void moveDistanceWithPID(double distance) throws Exception;

  void setIdleMode(int idleMode);

  void periodic();

  public void stopDrive();

  public double getGyroAngle();

  public void resetGyroAngle();
}
