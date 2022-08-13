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
    init(m_leftMotor, m_rightMotor);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_leftMotor.setInverted(true);


    // Use inches as unit for encoder distances
    m_leftEncoder.setDistancePerPulse(
        (Math.PI * Constants.Romi.kWheelDiameterInch) / Constants.Romi.kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse(
        (Math.PI * Constants.Romi.kWheelDiameterInch) / Constants.Romi.kCountsPerRevolution);

    resetEncoders();
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

  @Override
  public void resetGyroAngle() {
    resetEncoders(); // the romi uses encoders for turning, not a gyro
  }

  @Override
  public double getGyroAngle() {
    /* Need to convert distance travelled to degrees. The Standard
       Romi Chassis found here, https://www.pololu.com/category/203/romi-chassis-kits,
       has a wheel placement diameter (149 mm) - width of the wheel (8 mm) = 141 mm
       or 5.551 inches. We then take into consideration the width of the tires.
    */
    double inchPerDegree = Math.PI * 5.551 / 360;
    return getAverageTurningDistance() / inchPerDegree;
  }

  private double getAverageTurningDistance() {
    double leftDistance = Math.abs(m_leftEncoder.getDistance());
    double rightDistance = Math.abs(m_rightEncoder.getDistance());
    double averageTurnDistance = (leftDistance + rightDistance) / 2.0;

    //adjust the value based on the turn direction 
    if (m_leftEncoder.getDistance()<0)
      averageTurnDistance = averageTurnDistance*-1;

    return averageTurnDistance;
  }
}
