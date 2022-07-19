// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants;

/** Add your docs here. */
public class RomiDrive extends DriveBase {
  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final Spark m_leftMotor = new Spark(Constants.Romi.leftDeviceID);
  private final Spark m_rightMotor = new Spark(Constants.Romi.rightDeviceID);

  // The Romi has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoder =
      new Encoder(Constants.Romi.leftEncoderChannelA, Constants.Romi.leftEncoderChannelB);
  private final Encoder m_rightEncoder =
      new Encoder(Constants.Romi.rightEncoderChannelA, Constants.Romi.rightEncoderChannelB);

  RomiDrive() {
    // Use inches as unit for encoder distances
    m_leftEncoder.setDistancePerPulse(
        (Math.PI * Constants.Romi.kWheelDiameterMeter) / Constants.Romi.kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse(
        (Math.PI * Constants.Romi.kWheelDiameterMeter) / Constants.Romi.kCountsPerRevolution);

    init(m_leftMotor, m_rightMotor);
  }

  @Override
  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  @Override
  public double getLeftEncoderDistance() {
    return m_leftEncoder.getDistance();
  }

  @Override
  public double getRightEncoderDistance() {
    return m_rightEncoder.getDistance();
  }

  @Override
  public double getAverageEncoderDistance() {
    return (getLeftEncoderDistance() + getRightEncoderDistance()) / 2.0;
  }

  @Override
  public void setRampRate(double rate) {
    // Not applicable for Romi
  }

  @Override
  public void moveDistanceWithPID(double distance) throws Exception {
    throw new Exception("moveDistanceWithPID not implemented for RomiDrive  ");
  }

  @Override
  public void setIdleMode(int idleMode) {
    // Not applicable for Romi
  }
}
