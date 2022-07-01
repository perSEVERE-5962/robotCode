// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.gyro;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

/** Add your docs here. */
public class AHRSGyro implements GyroInterface {
  private AHRS m_ahrs = new AHRS(SPI.Port.kMXP);

  @Override
  public double getGyroAngle() {
    return m_ahrs.getAngle();
  }

  @Override
  public void resetGyro() {
    m_ahrs.reset();
  }
}
